"""台架只读测试（0x3E 协议）：纯读，不发运动指令（守门第 1 步）。
用法：GIMBAL_REAL_MOTION=1 python3 tools/gimbal_bench_test.py --port /dev/ttyUSB0 --ids 1 2 --baud 460800
"""
import argparse
import os
import sys
import time

sys.path.insert(0, ".")


def main():
    if os.environ.get("GIMBAL_REAL_MOTION") != "1":
        print("FAIL：需要 GIMBAL_REAL_MOTION=1")
        return 1
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--ids", nargs="+", type=int, default=[1, 2])
    ap.add_argument("--baud", type=int, default=460800)
    args = ap.parse_args()

    from drivers.mwd_3e_driver import MWD3E
    drv = MWD3E(port=args.port, baud=args.baud)
    try:
        drv.open()
    except Exception as e:
        print(f"FAIL：串口打不开：{e}")
        return 1

    ok = True
    try:
        for sid in args.ids:
            print(f"== 站号 {sid} ==")
            for i in range(3):
                try:
                    s1 = drv.read_status1(sid)
                    pos = drv.read_position_deg(sid)
                    spd = drv.read_speed_dps(sid)
                    print(f"  帧{i+1}：温度={s1['temp']}℃ 电压={s1['vbus']:.2f}V "
                          f"电流={s1['ibus']:.2f}A 运行={s1['run']:#x} 故障={s1['fault']:#x}")
                    print(f"       位置={pos:.2f}° 速度={spd:.1f}°/s")
                    if s1["fault"]:
                        print(f"  WARN：站{sid} 有故障标志 {s1['fault']:#x}")
                except Exception as e:
                    print(f"  帧{i+1} FAIL：{e}")
                    ok = False
                    break
                time.sleep(0.2)
            if not ok:
                break
    finally:
        drv.close()

    if ok:
        print("PASS：只读全通，下一步点动")
        return 0
    print("FAIL：有读失败，检查接线/站号/波特率")
    return 1


if __name__ == "__main__":
    sys.exit(main())
