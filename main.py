"""一条命令启动（文档 §1）：配置检查 -> 视频检查 -> 模型加载 -> 串口检查 -> 日志初始化。
跑：python3 main.py        （mock 全链路）
    GIMBAL_REAL_MOTION=1 python3 main.py   （真实云台，需先过守门顺序）
停：Ctrl+C，先走停止路径，再关串口/视频/网页。
"""
import os
import signal
import sys
import threading
import time

sys.path.insert(0, ".")

from config_center import load_config
from state_machine import StateMachine, TRACK, STOPPED
from detection.interface import MockDetector
from ibvs import IBVSTracker, IBVSConfig
from video import source as video_source
from video import mjpeg_server
from drivers import rangefinder as rng_mod
from drivers.ts6004_driver import TS6004, resolve_port
from runlog.run_logger import RunLogger

try:
    import cv2
except ImportError:
    cv2 = None


def main():
    cfg = load_config()                       # 1. 配置检查+打印生效值
    logger = RunLogger(cfg)                   # 5. 日志初始化（先建目录，事件可记）
    logger.event("启动：配置校验通过")

    src = video_source.create(cfg)            # 2. 视频检查
    src.open()
    f0 = src.read()

    from detection.ros_adapter import create_detector
    if f0 is not None:
        img_h, img_w = f0.image.shape[:2]
        logger.event(f"视频 OK：{img_w}x{img_h}")
    else:
        # Unknown RTSP size must be resolved from the eventual live frame.
        img_w = cfg["video"]["width"] or 0
        img_h = cfg["video"]["height"] or 0
        if cfg["video"]["source"] == "rtsp":
            logger.event("警告：RTSP 首帧未到，主循环将归零并持续重连")
    det = create_detector(cfg, img_w, img_h)  # 3. 模型加载：mock | ros_track
    det.open()
    backend = os.environ.get("SMART_GIMBAL_DETECTOR", cfg["model"]["backend"])
    logger.event(f"检测器 backend={backend}")

    g = cfg["gimbal"]
    port = g["port"]
    if port == "auto":
        if g.get("serial"):
            found = resolve_port(g.get("vid_pid", ""), g["serial"])
            if found:
                port = found
                logger.event(f"串口自动识别：{port}")
                print(f"[串口] 自动识别到 {port}")
            else:
                port = "/dev/ttyUSB1"
                logger.event("串口自动识别未命中，回退 /dev/ttyUSB1")
        else:
            # 序列号为空时禁止按 VID:PID 裸识别——旧云台也是 CP2102N，会认错设备
            port = "/dev/ttyUSB1"
            msg = ("序列号未填，跳过自动识别（旧云台同为 CP2102N，裸认会发错设备）。"
                   "新转接器序列号拿到后填进 serial，或把 port 写死。回退 /dev/ttyUSB1")
            logger.event(msg)
            print(f"[串口] {msg}")
    drv_kwargs = {"baud": g["baud"]}
    if port != "auto":
        drv_kwargs["port"] = port
    drv = TS6004(**drv_kwargs)
    drv.open()                                # 4. 串口检查（含 mock 提示）
    real = drv.real
    mode = "real" if real else "mock"
    if real:
        if not g["soft_limit_deg"]:
            sys.exit("real 模式软限位为空，拒绝启动")
        lo, hi = g["soft_limit_deg"]
        for sid in (g["roll_id"], g["pitch_id"]):
            drv.setup_motion(sid, max_speed_dps=g["motion_max_speed_dps"],
                             accel=g["motion_accel"], decel=g["motion_decel"])
            drv.set_soft_limits(sid, lo, hi)
        logger.event(f"云台 real：限位 [{lo}, {hi}]°，运动曲线已下发")
    else:
        logger.event("云台 mock：指令只记录不发送")

    rng = rng_mod.create(cfg)
    rng.open()

    from drivers import focus as focus_mod
    from focus.af_chain import RangingAF
    af, focus_drv = None, None
    if cfg.get("autofocus", {}).get("enabled"):
        focus_drv = focus_mod.create(cfg)
        focus_drv.open()
        af = RangingAF(cfg, focus_drv)
        logger.event(f"自动对焦开：driver={cfg['autofocus']['driver']} "
                     f"f={cfg['autofocus']['f_mm']}mm")
    else:
        logger.event("自动对焦关（autofocus.enabled=false），硬件到了再开")

    sm = StateMachine(cfg)
    tracker = IBVSTracker(IBVSConfig(cfg))
    from tracking.fine_point import FinePointTracker, FinePointResult
    fine = FinePointTracker(cfg)
    fc = cfg.get("fine_point", {})
    tracker_fine = IBVSTracker(IBVSConfig(
        cfg, kp=fc.get("kp", 0.08), deadzone_px=fc.get("deadzone_px", 3.0),
        max_vel_dps=fc.get("max_vel_dps", 10.0),
        slew_dps2=fc.get("slew_dps2", 60.0)))
    logger.event(f"精瞄 {'开' if fine.enabled else '关'}：engage 半径 "
                 f"{fine.engage_radius_px}px，精瞄档死区 "
                 f"{fc.get('deadzone_px', 3.0)}px")
    srv, mjpeg_port = mjpeg_server.serve(cfg["video"]["mjpeg_port"])
    logger.event(f"画面服务端口 {mjpeg_port}")
    rate = g["rate_hz"]
    dt = 1.0 / rate
    frames, t_start = 0, time.monotonic()
    fps, last_fps_t, last_fps_n = 0.0, t_start, 0

    print(f"[主循环] {rate}Hz 启动，Ctrl+C 退出")
    logger.event("主循环启动")
    stream_down = False
    stop_requested = threading.Event()
    stop_reason = ["正常结束"]
    shutdown_done = False

    def request_stop(signum, _frame):
        stop_reason[0] = signal.Signals(signum).name
        stop_requested.set()

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)

    def shutdown(reason):
        nonlocal shutdown_done
        if shutdown_done:
            return
        shutdown_done = True
        sm.stop(reason)
        cleanup_errors = []
        try:
            if real:
                drv.stop_all()
        except Exception as exc:
            cleanup_errors.append(f"运动停止：{exc}")
        for name, component in (("检测器", det), ("测距", rng), ("视频", src),
                                ("云台驱动", drv), ("对焦", focus_drv)):
            if component is not None:
                try:
                    component.close()
                except Exception as exc:
                    cleanup_errors.append(f"{name}关闭：{exc}")
        for error in cleanup_errors:
            logger.event(f"停机清理异常：{error}")
        logger.finish({"mode": mode, "frames": frames,
                       "state_history": sm.history, "result": reason,
                       "cleanup_errors": cleanup_errors})
        if cleanup_errors:
            raise RuntimeError("停机清理异常：" + "; ".join(cleanup_errors))

    try:
        while not stop_requested.is_set():
            t0 = time.monotonic()
            frame = src.read()
            if frame is None:
                # 断流：状态机同步进 LOST 计时，控制归零，程序不崩，持续重连（文档 §7）
                if not stream_down:
                    stream_down = True
                    sm.update(False, 0.0, dt=dt)
                    logger.event("视频断流：控制归零，持续重连")
                mjpeg_server.update_status({"backend": backend, "motion_mode": mode,
                                           "stream_connected": False, "visible": False,
                                           "focus_dac": None, "dist_m": None,
                                           "dist_valid": False,
                                           "state": sm.state})
                if real:
                    drv.stop(g["pitch_id"])
                time.sleep(0.2)
                continue
            if stream_down:
                stream_down = False
                logger.event("视频恢复")
            logger.lat.mark(frame.frame_id, "got")

            d = det.process(frame)            # 同一 frame_id 全链路
            logger.lat.mark(frame.frame_id, "infer")

            # 精瞄（粗精两级之精级）：有效观测时在 ROI 内算精确点；
            # 无效/预测/找不到团块时回退用框中心，绝不硬算假点。
            if d.visible and not d.predicted:
                fine_res = fine.update(frame.image, d.bbox_xyxy)
            else:
                fine_res = FinePointResult(valid=False, reason="no-obs")
                fine.reset()
            engage_r = fine.engage_radius_px
            use_fine = (fine_res.valid and abs(d.du_px) < engage_r
                        and abs(d.dv_px) < engage_r)

            state = sm.update(d.visible and not d.predicted, d.conf, dt=dt)
            pv_c, yv_c = tracker.update(d.du_px, d.dv_px, state == TRACK, dt)
            pv_f, yv_f = tracker_fine.update(
                fine_res.fdu_px, fine_res.fdv_px,
                state == TRACK and use_fine, dt)
            pv, yv = (pv_f, yv_f) if use_fine else (pv_c, yv_c)
            aim_mode = "fine" if use_fine else "coarse"
            logger.lat.mark(frame.frame_id, "cmd")

            fault = None
            try:
                if real:
                    pos = drv.read_position_deg(g["pitch_id"])
                    lo, hi = g["soft_limit_deg"]
                    if not (lo <= pos <= hi):
                        fault = f"俯仰 {pos:.1f}° 超限位"
                    else:
                        drv.command_velocity_dps(g["pitch_id"], pv, dt,
                                                 soft_limit_deg=(lo, hi))
                if isinstance(det, MockDetector):
                    det.apply_motion(pv, dt)  # mock 闭环：虚拟云台带动虚拟目标
            except Exception as e:
                fault = f"驱动异常：{e}"
            if fault:
                sm.update(False, 0, fault=fault, fault_reason=fault)
                logger.event(fault)
                if real:
                    drv.emergency_stop(g["pitch_id"])

            r = rng.read()
            # 测距辅助对焦：只跟有效目标的距离；无有效目标/测距无效时保持上次位置
            focus_dac, focus_moved = None, False
            if af is not None:
                fd_valid = d.visible and not d.predicted and r.valid
                focus_dac, focus_moved = af.update(r.distance_m if fd_valid else None,
                                                   fd_valid)
            if cv2 is not None:
                img = mjpeg_server.draw_overlay(frame.image.copy(), d, state, fps, r,
                                                mode, focus_dac, frame_id=frame.frame_id,
                                                backend=backend, fine_res=fine_res,
                                                aim_mode=aim_mode)
                ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    metadata = {"frame_id": frame.frame_id, "width": img.shape[1],
                                "height": img.shape[0], "fps": round(fps, 1),
                                "backend": backend, "observation_source": getattr(d, "observation_source", None),
                                "motion_mode": mode,
                                "visible": bool(d.visible), "predicted": bool(d.predicted),
                                "confidence": float(d.conf), "du_px": float(d.du_px),
                                "dv_px": float(d.dv_px), "state": state,
                                "label": d.label, "bbox": d.bbox_xyxy,
                                "aim_mode": aim_mode,
                                "fine_valid": bool(fine_res.valid),
                                "fine_fx": round(fine_res.fx, 1) if fine_res.valid else None,
                                "fine_fy": round(fine_res.fy, 1) if fine_res.valid else None,
                                "focus_dac": focus_dac,
                                "dist_m": r.distance_m if r.valid else None,
                                "dist_valid": bool(r.valid),
                                "stream_connected": True}
                    if hasattr(src, "stats"):
                        metadata["source"] = src.stats()
                    mjpeg_server.push_frame(jpg.tobytes(), metadata)
            logger.lat.mark(frame.frame_id, "show")

            logger.frame(frame.frame_id, d, state, pv, r, mode, focus_dac, focus_moved,
                         fine_res=fine_res, aim_mode=aim_mode)
            frames += 1
            last_fps_n += 1
            if time.monotonic() - last_fps_t >= 2.0:
                fps = last_fps_n / (time.monotonic() - last_fps_t)
                last_fps_t, last_fps_n = time.monotonic(), 0

            time.sleep(max(0.0, dt - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("\nCtrl+C，停机")
        shutdown("用户退出")
    except Exception as e:
        logger.event(f"主循环异常：{e}")
        shutdown(f"异常退出：{e}")
        raise
    else:
        shutdown(stop_reason[0])
    finally:
        srv.shutdown()
        srv.server_close()


if __name__ == "__main__":
    main()
