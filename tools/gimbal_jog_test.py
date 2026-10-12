"""台架点动测试：单次小角度点动，实测脉冲当量（守门第 2 步）。"""
import argparse
import sys
import time

sys.path.insert(0, ".")

from drivers.ts6004_driver import (
    TS6004, REG_TGT_LOOPS, REG_TGT_ANGLE, REG_STATE, STATE_RUN, PULSE_PER_REV,
)

MAX_PULSES = 5000


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--id", type=int, required=True, help="站号")
    ap.add_argument("--pulses", type=int, default=1000,
                    help="点动脉冲数（默认1000，上限5000）")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()
    if abs(args.pulses) > MAX_PULSES:
        print(f"FAIL：脉冲数 {args.pulses} 超过单次上限 {MAX_PULSES}")
        return 1
    drv = TS6004(port=args.port, baud=args.baud)
    try:
        drv.open()
    except Exception as e:
        print(f"FAIL：串口打不开：{e}")
        return 1
    try:
        sid = args.id
        drv.setup_motion(sid, max_speed_dps=10.0, accel=3000, decel=3000)
        time.sleep(0.3)
        before = drv.read_position_pulses(sid)
        print(f"起点：{before} 脉冲")
        tgt = before + args.pulses
        loops, angle = divmod(tgt, PULSE_PER_REV)
        drv.write_u32(sid, REG_TGT_LOOPS, loops)
        drv.write_u32(sid, REG_TGT_ANGLE, angle)
        drv.write_u16(sid, REG_STATE, STATE_RUN)
        print(f"已发目标：{tgt} 脉冲（+{args.pulses}），等 3 秒...")
        time.sleep(3.0)
        after = drv.read_position_pulses(sid)
        moved = after - before
        print(f"终点：{after} 脉冲，实测移动 {moved} 脉冲")
        deg_if_out = moved / 36000.0 * 360.0
        deg_if_motor = moved / 360000.0 * 360.0
        print(f"  若 36000=输出轴侧：转了 {deg_if_out:.2f} 度")
        print(f"  若 36000=电机侧（1:10）：转了 {deg_if_motor:.2f} 度")
        if abs(moved - args.pulses) > args.pulses * 0.2:
            print("WARN：实测移动量与指令差 20% 以上，检查是否丢步")
            return 1
        print("PASS：点动完成。把实测转了多少度告诉我，我定脉冲当量。")
        return 0
    except Exception as e:
        print(f"FAIL：{e}")
        return 1
    finally:
        drv.close()


if __name__ == "__main__":
    sys.exit(main())
