"""
TS6004-10 Modbus-RTU 驱动（Jetson 侧）
——按 2026-10-08 审计的官方协议（RS485_Modbus_V5.0）实现。

运动模型（位置模式，无直接速度寄存器）：
  6007=1       位置模式
  6014+6012    多圈目标位置（圈数+圈内角度）
  6022/6024/6026  最大速度 / 加速度 / 减速度（运动曲线参数，不是速度指令！）
  写 6006=2    触发运动；写 6006=0 急停
  写 6005=1    参数锁存（每次改参数后）

IBVS 的速度指令按“位置增量”实现：
  每周期 target = 上周期指令位置 + vel*dt，钳制到软件限位后下发。
  速度 0 = 目标位置冻结 = 停转（不是最高档）。

反馈：
  5002 状态 / 5004 多圈位置(u64脉冲) / 5008 速度 / 5010 单圈角度
"""

import os
import struct
import time

try:
    import serial
except ImportError:
    serial = None

ID_ROLL = 1    # 横滚轴（拨码 OFF OFF OFF）
ID_PITCH = 2   # 俯仰轴（拨码 OFF OFF ON）

REG_WORKMODE = 6007
REG_STATE = 6006
REG_TGT_ANGLE = 6012
REG_TGT_LOOPS = 6014
REG_MAX_SPEED = 6022   # 最大速度（曲线参数！！不是速度指令）
REG_ACCEL = 6024
REG_DECEL = 6026
REG_SYNC = 6005
REG_LIMIT_EN = 6033
REG_MIN_POS = 6038
REG_MAX_POS = 6040
REG_FB_STATUS = 5002
REG_FB_POS = 5004      # u64 多圈位置（脉冲）
REG_FB_SPEED = 5008
REG_FB_ANGLE = 5010

PULSE_PER_DEG = 100.0   # 36000 脉冲/圈；！！输出轴还是电机侧待首次点动验证
PULSE_PER_REV = 36000

STATE_RUN = 2
STATE_STOP = 0

REAL_MOTION = os.environ.get("GIMBAL_REAL_MOTION", "0") == "1"


def resolve_port(vid_pid="", serial_short=""):
    """按 VID:PID/序列号找串口，返回 /dev/ttyUSBn（不用固定编号，文档 §5）。
    认准的是转接器硬件，不是插槽顺序。

    硬保护：serial_short 为空时拒绝按 VID:PID 裸识别——旧云台也是
    CP2102N（10c4:ea60），裸识别会把 Modbus 命令发到旧云台上。
    2026-10-09 用户插拔确认旧云台序列号后加固。"""
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


def _crc16(frame: bytes) -> bytes:
    """标准 CRC-16/ARC，厂家实例要求高字节在前。"""
    crc = 0xFFFF
    for b in frame:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([(crc >> 8) & 0xFF, crc & 0xFF])


class ModbusError(Exception):
    pass


class TS6004:
    def __init__(self, port="/dev/ttyUSB1", baud=115200):
        self.port = port
        self.baud = baud
        self._ser = None
        self._tgt = {}  # slave -> 当前指令位置（脉冲）
        self.real = REAL_MOTION and serial is not None
        if REAL_MOTION and serial is None:
            raise ModbusError("GIMBAL_REAL_MOTION=1 但没装 pyserial，先 pip install pyserial")

    def open(self):
        if not self.real:
            print("[mock] TS6004 驱动：真实运动未开启，运动指令只打印不发送")
            return
        # 端口边界：/dev/ttyUSB1 新云台预留；ttyUSB0=旧云台；ttyACM0=飞控。勿碰。
        self._ser = serial.Serial(self.port, self.baud, timeout=0.5)
        time.sleep(0.1)

    def close(self):
        if self._ser:
            self._ser.close()
            self._ser = None

    # ---------- 底层 ----------
    def _xfer(self, slave: int, func: int, addr: int, payload: bytes) -> bytes:
        req = struct.pack(">BBH", slave, func, addr) + payload
        req += _crc16(req)
        self._ser.reset_input_buffer()
        self._ser.write(req)
        self._ser.flush()
        head = self._ser.read(3)
        if len(head) < 3:
            raise ModbusError(f"站{slave} 超时无回帧")
        if head[0] != slave or head[1] != func:
            raise ModbusError(f"回帧站号/功能码不符：{head[0]}/{head[1]:#x}")
        if func == 0x04:
            n = head[2]
            data = self._ser.read(n + 2)
            if len(data) < n + 2:
                raise ModbusError("回帧数据不完整")
            body, crc = data[:n], data[n:n + 2]
            if _crc16(head + body) != crc:
                raise ModbusError("CRC 校验失败，先查字节序/波特率")
            return body
        tail = self._ser.read(5)  # 写回帧固定 8 字节
        if len(tail) < 5:
            raise ModbusError("写回帧不完整")
        if _crc16(head + tail[:3]) != tail[3:5]:
            raise ModbusError("写回帧 CRC 失败")
        return b""

    def _read_regs(self, slave: int, addr: int, count: int) -> bytes:
        return self._xfer(slave, 0x04, addr, struct.pack(">H", count))

    def read_u16(self, slave: int, addr: int) -> int:
        return struct.unpack(">H", self._read_regs(slave, addr, 1))[0]

    def read_u32(self, slave: int, addr: int) -> int:
        return struct.unpack(">I", self._read_regs(slave, addr, 2))[0]

    def read_u64(self, slave: int, addr: int) -> int:
        return struct.unpack(">Q", self._read_regs(slave, addr, 4))[0]

    def write_u16(self, slave: int, addr: int, value: int):
        self._xfer(slave, 0x06, addr, struct.pack(">H", value & 0xFFFF))

    def write_u32(self, slave: int, addr: int, value: int):
        self._xfer(slave, 0x10, addr,
                   struct.pack(">H", 2) + bytes([4]) + struct.pack(">I", value & 0xFFFFFFFF))

    def _sync(self, slave: int):
        self.write_u16(slave, REG_SYNC, 1)

    # ---------- 只读 ----------
    def read_status(self, slave: int) -> int:
        return self.read_u16(slave, REG_FB_STATUS)

    def read_position_pulses(self, slave: int) -> int:
        return self.read_u64(slave, REG_FB_POS)

    def read_position_deg(self, slave: int) -> float:
        return self.read_position_pulses(slave) / PULSE_PER_DEG

    def read_speed_dps(self, slave: int) -> float:
        return self.read_u32(slave, REG_FB_SPEED) / PULSE_PER_DEG

    # ---------- 配置（真实运动前调用一次） ----------
    def setup_motion(self, slave: int, max_speed_dps=60.0, accel=3000, decel=3000):
        """位置模式 + 运动曲线。max_speed 要大于 IBVS 的 max_vel_dps，否则跟不上。"""
        if not self.real:
            print(f"[mock] 站{slave} setup: 位置模式, 最大速度{max_speed_dps}°/s")
            return
        self.write_u16(slave, REG_WORKMODE, 1)
        self.write_u32(slave, REG_MAX_SPEED, int(max_speed_dps * PULSE_PER_DEG))
        self.write_u32(slave, REG_ACCEL, accel)
        self.write_u32(slave, REG_DECEL, decel)
        self._sync(slave)
        self._tgt[slave] = self.read_position_pulses(slave)

    def set_soft_limits(self, slave: int, lo_deg: float, hi_deg: float):
        if not self.real:
            print(f"[mock] 站{slave} 软限位 [{lo_deg}, {hi_deg}]°")
            return
        self.write_u32(slave, REG_MIN_POS, int(lo_deg * PULSE_PER_DEG) & 0xFFFFFFFF)
        self.write_u32(slave, REG_MAX_POS, int(hi_deg * PULSE_PER_DEG) & 0xFFFFFFFF)
        self.write_u16(slave, REG_LIMIT_EN, 1)
        self._sync(slave)

    # ---------- 运动 ----------
    def command_velocity_dps(self, slave: int, vel_dps: float, dt: float,
                             soft_limit_deg=None):
        """IBVS 速度接口：内部转成位置增量下发。速度 0 = 目标冻结 = 停转。"""
        if not self.real:
            print(f"[mock] 站{slave} 速度指令 {vel_dps:+.2f}°/s")
            return
        if slave not in self._tgt:
            self._tgt[slave] = self.read_position_pulses(slave)
        tgt = self._tgt[slave] + vel_dps * dt * PULSE_PER_DEG
        if soft_limit_deg:
            lo = int(soft_limit_deg[0] * PULSE_PER_DEG)
            hi = int(soft_limit_deg[1] * PULSE_PER_DEG)
            tgt = max(lo, min(hi, int(round(tgt))))
            if int(round(self._tgt[slave] + vel_dps * dt * PULSE_PER_DEG)) != tgt:
                vel_dps = 0.0  # 顶到限位：本周期不再往外走
                tgt = self._tgt[slave]
        tgt = int(round(tgt))
        self._tgt[slave] = tgt
        loops, angle = divmod(tgt, PULSE_PER_REV)
        self.write_u32(slave, REG_TGT_LOOPS, loops)
        self.write_u32(slave, REG_TGT_ANGLE, angle)
        self.write_u16(slave, REG_STATE, STATE_RUN)

    def stop(self, slave: int):
        """受控停转：速度归零，位置保持。"""
        if not self.real:
            print(f"[mock] 站{slave} 停转")
            return
        if slave in self._tgt:
            tgt = self._tgt[slave]
            loops, angle = divmod(tgt, PULSE_PER_REV)
            self.write_u32(slave, REG_TGT_LOOPS, loops)
            self.write_u32(slave, REG_TGT_ANGLE, angle)
            self.write_u16(slave, REG_STATE, STATE_RUN)

    def emergency_stop(self, slave: int):
        """急停：STATE=0，驱动器立即失能。限位触发/串口异常时用这个。"""
        if not self.real:
            print(f"[mock] 站{slave} 急停")
            return
        self.write_u16(slave, REG_STATE, STATE_STOP)

    def stop_all(self):
        for s in (ID_ROLL, ID_PITCH):
            try:
                self.emergency_stop(s)
            except Exception as e:
                print(f"站{s} 急停失败：{e} —— 按预案人工断电")
