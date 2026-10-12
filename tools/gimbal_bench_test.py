"""台架只读测试：纯读寄存器，不发任何运动指令（守门第 1 步）。

用法（Jetson 上）：
    python3 tools/gimbal_bench_test.py --port /dev/ttyUSBx --ids 1 2

只做三件事，每站号读三帧：
  1. 状态寄存器（6007）
  2. 位置反馈（5004，u64 脉冲 + 换算度）
  3. 速度反馈
全部读通才算过；任何一帧超时/异常都算失败，不继续。
"""
import argparse
import sys
import time

sys.path.insert(0, ".")

from drivers.ts6004_driver import TS6004


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="RS485 串口，如 /dev/ttyUSB1")
    ap.add_argument("--ids", nargs="+", type=int, default=[1, 2],
                    help="站号列表，默认 1 2")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    drv = TS6004(port=args.port, baud=args.baud)
    try:
        drv.open()
    except Exception as e:
        print(f"FAIL：串口打不开 {args.port}：{e}")
        return 1

    ok = True
    try:
        for sid in args.ids:
            print(f"== 站号 {sid} ==")
            for i in range(3):
                try:
                    st = drv.read_status(sid)
                    pos_pulse = drv.read_position_pulses(sid)
                    pos_deg = drv.read_position_deg(sid)
                    spd = drv.read_speed_dps(sid)
                    print(f"  帧{i+1}：状态={st} 位置={pos_pulse}脉冲 "
                          f"({pos_deg:.2f}°) 速度={spd:.2f}°/s")
                except Exception as e:
                    print(f"  帧{i+1}：FAIL：{e}")
                    ok = False
                    break
                time.sleep(0.2)
            if not ok:
                break
    finally:
        drv.close()

    if ok:
        print("PASS：双站号只读三帧全通，下一步：空载小角度点动")
        return 0
    print("FAIL：有读失败，检查接线/站号/波特率，不进入点动")
    return 1


if __name__ == "__main__":
    sys.exit(main())
