"""一键联调自检（文档 §5 调试顺序）。
跑：python3 bringup.py
做：依赖检查 -> 配置校验 -> 视频源检查 -> 串口枚举(按 VID:PID，不用固定编号)
    -> 云台只读三帧 -> 引导式 2° 点动（real 需 GIMBAL_REAL_MOTION=1）
"""
import glob
import os
import sys

sys.path.insert(0, ".")


def check_deps():
    print("== 依赖 ==")
    for mod, hint in [("yaml", "pip install pyyaml"), ("cv2", "pip install opencv-python"),
                      ("serial", "pip install pyserial")]:
        try:
            __import__(mod)
            print(f"  {mod} OK")
        except ImportError:
            print(f"  {mod} 缺失：{hint}")


def check_config():
    print("== 配置 ==")
    from config_center import load_config
    return load_config()


def check_video(cfg):
    print("== 视频 ==")
    from video.source import create
    src = create(cfg)
    try:
        src.open()
        f = src.read()
        print(f"  取流 OK：frame_id={f.frame_id} 尺寸={f.image.shape[1]}x{f.image.shape[0]}"
              if f is not None else "  取流失败：返回 None（主循环会归零并重连）")
        src.close()
    except Exception as e:
        print(f"  视频异常：{e}")


def check_model(cfg):
    print("== 检测后端 ==")
    backend = os.environ.get("SMART_GIMBAL_DETECTOR", cfg["model"]["backend"])
    print(f"  生效后端：{backend}")
    if backend == "ros_track":
        try:
            import rclpy  # noqa
            print("  rclpy OK")
        except ImportError:
            print("  rclpy 缺失：先 source /opt/ros/humble/setup.bash")
        try:
            import anti_drone_interfaces.msg  # noqa
            print("  anti_drone_interfaces OK")
        except ImportError:
            print("  anti_drone_interfaces 缺失：先 colcon build 装好")
        print(f"  话题：{cfg['model'].get('ros_topic')}  ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(未设，默认0)')}")


def check_serial(cfg):
    print("== 串口 ==")
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    print(f"  发现串口：{ports or '无'}")
    print("  边界：ttyUSB0=旧云台，ttyACM0=飞控，勿动")
    g = cfg["gimbal"]
    from drivers.ts6004_driver import resolve_port
    if g.get("serial"):
        found = resolve_port(g.get("vid_pid", ""), g["serial"])
        if found:
            print(f"  按 VID:PID+序列号识别到新云台转接器：{found}")
        else:
            print("  序列号已填但未命中，检查转接器是否插好")
    else:
        print("  序列号未填：禁止按 VID:PID 裸识别（旧云台同为 CP2102N，会认错设备）")
        print("  新转接器插上后，Codex 跑：ls /dev/serial/by-id/ 找新序列号，填进 config.yaml")
        print("  确认后安装 udev 规则：sudo cp 99-ts6004-rs485.rules /etc/udev/rules.d/")


def check_gimbal_readonly(cfg):
    print("== 云台只读 ==")
    g = cfg["gimbal"]
    if g["mode"] != "real" or os.environ.get("GIMBAL_REAL_MOTION") != "1":
        print("  mock 模式，跳过硬件只读（real 时自动执行三帧验证）")
        return
    from drivers.ts6004_driver import TS6004
    d = TS6004()
    d.open()
    try:
        for slave in (g["roll_id"], g["pitch_id"]):
            st = d.read_status(slave)
            pos = d.read_position_deg(slave)
            spd = d.read_speed_dps(slave)
            print(f"  站{slave}：状态={st} 位置={pos:.1f}° 速度={spd:.1f}°/s")
        print("  只读三帧 OK，下一步：引导式 2° 空载点动")
    finally:
        d.close()


def main():
    print("# 机载智能跟踪云台 · 联调自检\n")
    check_deps()
    cfg = check_config()
    check_video(cfg)
    check_model(cfg)
    check_serial(cfg)
    check_gimbal_readonly(cfg)
    print("\n自检完成。mock 全绿即可跑主程序：python3 main.py")


if __name__ == "__main__":
    main()
