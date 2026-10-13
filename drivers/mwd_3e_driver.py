"""
慕纬度 TS6004-10 0x3E 厂家协议驱动（Jetson 侧）
——按《慕纬度电机 RS485 通讯协议 V3.0.1》实现。

帧格式：[0x3E][CMD][ID][DataLen][SumHead][DATA...][SumData]
  SumHead = (0x3E+CMD+ID+DataLen) & 0xFF
  SumData = sum(DATA) & 0xFF
  多字节小端，角度单位 0.01°/LSB，速度单位 0.01dps/LSB

波特率：460800（用户 Windows 上位机实测值；协议支持 9600~4M）
站号：1~32
"""
import os
import struct
import time

try:
    import serial
except ImportError:
    serial = None

ID_ROLL = 1
ID_PITCH = 2

# 命令字
CMD_MOTOR_OFF = 0x80
CMD_ESTOP = 0x81
CMD_MOTOR_RUN = 0x88
CMD_READ_STATUS1 = 0x9A
CMD_READ_STATUS2 = 0x9C
CMD_READ_MULTITURN_ANGLE = 0x92
CMD_SPEED = 0xA2
CMD_POS_MULTITURN = 0xA3
CMD_POS_INCREMENT = 0xA7
CMD_POS_INCREMENT_LIM = 0xA8

REAL_MOTION = os.environ.get("GIMBAL_REAL_MOTION", "0") == "1"

def resolve_port(vid_pid="", serial_short=""):
    """按 VID:PID/序列号找串口，返回 /dev/ttyUSBn。
    硬保护：serial_short 为空时拒绝按 VID:PID 裸识别——旧云台也是
    CP2102N（10c4:ea60），裸识别会把命令发到旧云台上。"""
    if vid_pid and not serial_short:
        raise ValueError("serial 为空：禁止按 VID:PID 裸识别（旧云台同为 CP2102N，会发错设备）。"
                         "填新转接器序列号，或把 config 的 port 写死。")
    import glob
    import subprocess
    for dev in sorted(glob.glob("/dev/ttyUSB*")):
        try:
            out = subprocess.run(["udevadm", "info", "-n", dev],
                                 capture_output=True, text=True, timeout=5).stdout
        except Exception:
            continue
        props = {}
        for line in out.splitlines():
            line = line.strip()
            if line.startswith("E: ") and "=" in line:
                k, v = line[3:].split("=", 1)
                props[k] = v
        if vid_pid and f"{props.get('ID_VENDOR_ID','')}:{props.get('ID_MODEL_ID','')}".lower() \
                != vid_pid.lower():
            continue
        if serial_short and props.get("ID_SERIAL_SHORT", "") != serial_short:
            continue
        return dev
    return None




class MWD3EError(Exception):
    pass


def _sum_head(cmd, sid, datalen):
    return (0x3E + cmd + sid + datalen) & 0xFF


def _sum_data(data: bytes):
    return sum(data) & 0xFF


def build_frame(cmd: int, sid: int, data: bytes = b"") -> bytes:
    sh = _sum_head(cmd, sid, len(data))
    head = bytes([0x3E, cmd, sid, len(data), sh])
    if not data:
        return head  # DataLen=0 时无数据区、无数据校验字节（协议 V3.0.1 示例帧皆为 5 字节）
    return head + data + bytes([_sum_data(data)])


class MWD3E:
    """TS6004-10 0x3E 协议驱动。"""

    def __init__(self, port="/dev/ttyUSB0", baud=460800):
        self.port = port
        self.baud = baud
        self._ser = None
        self.real = REAL_MOTION and serial is not None
        if REAL_MOTION and serial is None:
            raise MWD3EError("GIMBAL_REAL_MOTION=1 但没装 pyserial")

    def open(self):
        if not self.real:
            print("[mock] MWD-3E 驱动：真实运动未开启，只打印不发送")
            return
        self._ser = serial.Serial(self.port, self.baud, timeout=0.5)
        time.sleep(0.15)

    def close(self):
        if self._ser:
            self._ser.close()
            self._ser = None

    def _xfer(self, cmd: int, sid: int, data: bytes = b"",
              expect_len: int = 0) -> bytes:
        """发一帧，收一帧，返回应答的数据区。"""
        if not self.real:
            return b"\x00" * expect_len
        req = build_frame(cmd, sid, data)
        self._ser.reset_input_buffer()
        self._ser.write(req)
        self._ser.flush()
        # 应答头 5 字节
        head = self._ser.read(5)
        if len(head) < 5:
            raise MWD3EError(f"站{sid} CMD=0x{cmd:02X} 超时无回帧")
        if head[0] != 0x3E or head[1] != cmd or head[2] != sid:
            raise MWD3EError(
                f"回帧头不符：{head.hex()}（期望 3E {cmd:02X} {sid:02X}）")
        datalen = head[3]
        if _sum_head(head[1], head[2], datalen) != head[4]:
            raise MWD3EError(f"站{sid} 头校验错")
        if datalen:
            rdata = self._ser.read(datalen + 1)
            if len(rdata) < datalen + 1:
                raise MWD3EError(f"站{sid} 数据区不完整")
            payload, sd = rdata[:datalen], rdata[datalen]
            if _sum_data(payload) != sd:
                raise MWD3EError(f"站{sid} 数据校验错")
        else:
            # 0x80/0x81/0x88 这类无数据指令的应答只有 5 字节头，
            # 无数据区也无数据校验字节（协议 V3.0.1 §5-7），不能再读
            payload = b""
        return payload

    # ---------- 只读 ----------
    def read_status1(self, sid: int) -> dict:
        """0x9A：温度/电压/电流/运行状态/故障。"""
        d = self._xfer(CMD_READ_STATUS1, sid, expect_len=7)
        temp = struct.unpack("b", d[0:1])[0]
        vbus = struct.unpack("<H", d[1:3])[0] / 100.0
        ibus = struct.unpack("<H", d[3:5])[0] / 100.0
        run = d[5]
        fault = d[6]
        return {"temp": temp, "vbus": vbus, "ibus": ibus,
                "run": run, "fault": fault}

    def read_status2(self, sid: int) -> dict:
        """0x9C：温度/转矩电流/转速/编码器。"""
        d = self._xfer(CMD_READ_STATUS2, sid, expect_len=7)
        temp = struct.unpack("b", d[0:1])[0]
        iq = struct.unpack("<h", d[1:3])[0]
        speed = struct.unpack("<h", d[3:5])[0]  # 1dps/LSB
        enc = struct.unpack("<H", d[5:7])[0]
        return {"temp": temp, "iq": iq, "speed_dps": speed, "enc": enc}

    def read_position_deg(self, sid: int) -> float:
        """0x92：多圈绝对角度，0.01°/LSB。
        协议 V3.0.1 §26：应答数据区 7 字节有符号小端数，需符号扩展到 8 字节
        再按 int64 解析；直接补 0x00 会把负角度解析成巨大正数。"""
        d = self._xfer(CMD_READ_MULTITURN_ANGLE, sid, expect_len=8)
        if len(d) < 8:
            pad = b"\xff" if (d[-1] & 0x80) else b"\x00"
            d = d + pad * (8 - len(d))
        angle = struct.unpack("<q", d[:8])[0]
        return angle / 100.0

    def read_speed_dps(self, sid: int) -> float:
        return float(self.read_status2(sid)["speed_dps"])

    # ---------- 使能/急停 ----------
    def enable(self, sid: int):
        if not self.real:
            print(f"[mock] 站{sid} 使能")
            return
        self._xfer(CMD_MOTOR_RUN, sid)

    def disable(self, sid: int):
        if not self.real:
            print(f"[mock] 站{sid} 关闭")
            return
        self._xfer(CMD_MOTOR_OFF, sid)

    def estop(self, sid: int):
        if not self.real:
            print(f"[mock] 站{sid} 急停")
            return
        self._xfer(CMD_ESTOP, sid)

    # ---------- 运动 ----------
    def command_increment_deg(self, sid: int, delta_deg: float,
                              max_speed_dps: float = 0):
        """增量位置：0xA7（无限速）或 0xA8（带限速）。
        delta_deg 正负区分方向，内部转 0.01°/LSB。
        IBVS 直接调这个：每周期 target增量 = vel*dt。"""
        if not self.real:
            print(f"[mock] 站{sid} 增量 {delta_deg:+.3f}°")
            return
        inc = int(round(delta_deg * 100.0))
        if max_speed_dps and max_speed_dps > 0:
            spd = int(round(max_speed_dps * 100.0))
            data = struct.pack("<h", spd) + struct.pack("<i", inc)
            # 0xA8 数据区：限速uint16 + 增量int32 = 6 字节？按协议 8 字节补齐
            # 协议 DataLen=8：DATA0~1 限速，DATA2~5 增量，DATA6~7 保留
            data = struct.pack("<H", spd) + struct.pack("<i", inc) + b"\x00\x00"
            self._xfer(CMD_POS_INCREMENT_LIM, sid, data)
        else:
            data = struct.pack("<i", inc)
            self._xfer(CMD_POS_INCREMENT, sid, data)

    def command_speed_dps(self, sid: int, vel_dps: float):
        """速度环 0xA2：0.01dps/LSB。"""
        if not self.real:
            print(f"[mock] 站{sid} 速度 {vel_dps:+.2f}°/s")
            return
        data = struct.pack("<i", int(round(vel_dps * 100.0)))
        self._xfer(CMD_SPEED, sid, data)

    def stop(self, sid: int):
        """停转：速度环发 0。"""
        self.command_speed_dps(sid, 0.0)
    def stop_all(self):
        """两轴都停。"""
        for sid in (ID_ROLL, ID_PITCH):
            try:
                self.stop(sid)
            except Exception as e:
                print(f"站{sid} 停止失败：{e}")


