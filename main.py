"""一条命令启动（文档 §1）：配置检查 -> 视频检查 -> 模型加载 -> 串口检查 -> 日志初始化。
跑：python3 main.py        （mock 全链路）
    GIMBAL_REAL_MOTION=1 python3 main.py   （真实云台，需先过守门顺序）
停：Ctrl+C，先走停止路径，再关串口/视频/网页。
"""
import os
import sys
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
        img_w = cfg["video"]["width"] or 1280
        img_h = cfg["video"]["height"] or 720
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

    sm = StateMachine(cfg)
    tracker = IBVSTracker(IBVSConfig(cfg))
    srv, mjpeg_port = mjpeg_server.serve(cfg["video"]["mjpeg_port"])
    logger.event(f"画面服务端口 {mjpeg_port}")
    rate = g["rate_hz"]
    dt = 1.0 / rate
    frames, t_start = 0, time.monotonic()
    fps, last_fps_t, last_fps_n = 0.0, t_start, 0

    print(f"[主循环] {rate}Hz 启动，Ctrl+C 退出")
    logger.event("主循环启动")
    stream_down = False

    def shutdown(reason):
        sm.stop(reason)
        try:
            if real:
                drv.stop_all()
        finally:
            det.close(); rng.close(); src.close(); drv.close()
        logger.finish({"mode": mode, "frames": frames,
                       "state_history": sm.history, "result": reason})

    try:
        while True:
            t0 = time.monotonic()
            frame = src.read()
            if frame is None:
                # 断流：状态机同步进 LOST 计时，控制归零，程序不崩，持续重连（文档 §7）
                if not stream_down:
                    stream_down = True
                    sm.update(False, 0.0, dt=dt)
                    logger.event("视频断流：控制归零，持续重连")
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

            state = sm.update(d.visible and not d.predicted, d.conf, dt=dt)
            pv, yv = tracker.update(d.du_px, d.dv_px, state == TRACK, dt)
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
            if cv2 is not None:
                img = mjpeg_server.draw_overlay(frame.image.copy(), d, state, fps, r, mode)
                ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    mjpeg_server.push_frame(jpg.tobytes())
            logger.lat.mark(frame.frame_id, "show")

            logger.frame(frame.frame_id, d, state, pv, r, mode)
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


if __name__ == "__main__":
    main()
