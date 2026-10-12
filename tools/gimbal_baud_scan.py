"""扫描波特率：对每个常用波特率发一次 Modbus 读，看哪个有合法回帧。"""
import argparse
import os
import sys
import time

sys.path.insert(0, ".")

BAUDS = [9600, 19200, 38400, 57600, 115200]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--id", type=int, default=1)
    args = ap.parse_args()
    if os.environ.get("GIMBAL_REAL_MOTION") != "1":
        print("FAIL：需要 GIMBAL_REAL_MOTION=1")
        return 1
    try:
        import serial
    except ImportError:
        print("FAIL：没装 pyserial")
        return 1
    from drivers.ts6004_driver import _crc16, REG_FB_POS

    for baud in BAUDS:
        ser = serial.Serial(args.port, baud, timeout=0.5)
        time.sleep(0.15)
        try:
            # 读 5004（u64 位置），功能码 0x04，读 4 个寄存器
            req = __import__("struct").pack(">BBHH", args.id, 0x04, REG_FB_POS, 4)
            req += _crc16(req)
            ser.reset_input_buffer()
            ser.write(req)
            ser.flush()
            head = ser.read(3)
            if len(head) < 3:
                print(f"  {baud}：无回帧")
                continue
            if head[0] == args.id and head[1] == 0x04:
                n = head[2]
                data = ser.read(n + 2)
                print(f"  {baud}：*** 合法 Modbus 回帧！站号={head[0]} 功能码=0x{head[1]:02x} ***")
                ser.close()
                print(f"RESULT：波特率 = {baud}")
                return 0
            else:
                print(f"  {baud}：有回帧但非 Modbus（站号={head[0]} 功能码=0x{head[1]:02x}）")
        except Exception as e:
            print(f"  {baud}：异常 {e}")
        finally:
            ser.close()
    print("RESULT：所有波特率都无合法 Modbus 回帧，检查协议模式/接线/站号")
    return 1


if __name__ == "__main__":
    sys.exit(main())
