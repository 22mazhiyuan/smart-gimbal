"""SDBM-100 到货验证工具：打印原始 hex 帧和解析结果，核对协议。
用法：
  python3 tools/rangefinder_test.py --port /dev/ttyUSB2 --frames 10
  python3 tools/rangefinder_test.py --selftest   # 无硬件，自检帧构造/解析
"""
import argparse
import sys
import time

sys.path.insert(0, ".")
from drivers.rangefinder import SDBM100


def selftest():
    d = SDBM100()
    cmd = d.build_read_result()
    assert cmd.hex() == "aa800022a2", cmd.hex()
    print(f"读命令 OK: {cmd.hex()}")

    # 构造一帧：距离 12.345m（12345mm 大端），质量 0x0064
    body = bytes([0xAA, 0x00, 0x00, 0x22, 0x00, 0x00]) + (12345).to_bytes(4, "big") \
        + (100).to_bytes(2, "big")
    frame = body + bytes([SDBM100.checksum(body[1:])])
    assert len(frame) == 13
    r = d.parse_frame(frame)
    assert r == (12.345, 100), r
    print(f"解析 OK: frame={frame.hex()} -> {r[0]}m quality={r[1]}")

    # 坏校验必须拒收
    bad = frame[:-1] + bytes([(frame[-1] + 1) & 0xFF])
    assert d.parse_frame(bad) is None
    print("坏校验拒收 OK")

    # 全 0（无回波嫌疑）判无效
    zero = bytes([0xAA] + [0] * 11)
    zero = zero + bytes([SDBM100.checksum(zero[1:])])
    assert d.parse_frame(zero) is None
    print("无回波帧判无效 OK")

    # 小端模式
    d2 = SDBM100(byteorder="little")
    body_le = bytes([0xAA, 0x00, 0x00, 0x22, 0x00, 0x00]) + (12345).to_bytes(4, "little") \
        + (100).to_bytes(2, "little")
    frame_le = body_le + bytes([SDBM100.checksum(body_le[1:])])
    assert d2.parse_frame(frame_le) == (12.345, 100)
    print("小端解析 OK")
    print("自检全过")


def live(port, baud, nframes):
    d = SDBM100(port=port, baud=baud)
    d.open()
    print(f"发送读命令: {d.build_read_result().hex()}")
    print("等待有效帧（两种字节序都试算，帮你确认模块真实字节序）：")
    got = 0
    t0 = time.time()
    d_le = SDBM100(byteorder="little")
    while got < nframes and time.time() - t0 < 30:
        r = d.read()
        if r.valid:
            got += 1
            raw = bytes.fromhex(r.raw_code)
            le = d_le.parse_frame(raw)
            print(f"[{got}] raw={r.raw_code}  大端={r.distance_m:.3f}m"
                  f"  小端={le[0]:.3f}m  quality={r.raw_code and int.from_bytes(raw[10:12],'big')}"
                  f"  age={r.age_ms:.0f}ms")
        time.sleep(0.05)
    d.close()
    if got == 0:
        print("30 秒没收到有效帧：检查接线/波特率（试 19200）/port 是否写对")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="auto")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--frames", type=int, default=10)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        selftest()
    else:
        live(a.port, a.baud, a.frames)
