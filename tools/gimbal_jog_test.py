"""台架点动测试（0x3E 协议）：0xA7 增量位置单次小点动（守门第 2 步）。
用法：GIMBAL_REAL_MOTION=1 python3 tools/gimbal_jog_test.py --port /dev/ttyUSB0 --id 1 --deg 2.0 --baud 460800
单次上限 5°，电机必须空载。
"""
import argparse
import os
import sys
import time

sys.path.insert(0, ".")

MAX_DEG = 5.0


def main():
    if os.environ.get("GIMBAL_REAL_MOTION") != "1":
        print("FAIL：需要 GIMBAL_REAL_MOTION=1")
        return 1
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--id", type=int, required=True)
    ap.add_argument("--deg", type=float, default=2.0, help="点动角度（度，上限5）")
    ap.add_argument("--baud", type=int, default=460800)
    args = ap.parse_args()
    if abs(args.deg) > MAX_DEG:
        print(f"FAIL：{args.deg}° 超过单次上限 {MAX_DEG}°")
        return 1

    from drivers.mwd_3e_driver import MWD3E
    drv = MWD3E(port=args.port, baud=args.baud)
    try:
        drv.open()
    except Exception as e:
        print(f"FAIL：串口打不开：{e}")
        return 1
    try:
        sid = args.id
        drv.enable(sid)
        time.sleep(0.3)
        before = drv.read_position_deg(sid)
        print(f"起点：{before:.2f}°，发增量 {args.deg:+.2f}°…")
        drv.command_increment_deg(sid, args.deg)
        time.sleep(3.0)
        after = drv.read_position_deg(sid)
        moved = after - before
        print(f"终点：{after:.2f}°，实测移动 {moved:+.2f}°")
        if abs(moved - args.deg) > 0.5:
            print("WARN：实测与指令差 0.5° 以上")
            return 1
        print("PASS：点动完成")
        return 0
    except Exception as e:
        print(f"FAIL：{e}")
        return 1
    finally:
        drv.close()


if __name__ == "__main__":
    sys.exit(main())
