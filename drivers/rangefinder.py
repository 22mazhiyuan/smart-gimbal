"""测距：SDBM-100 真实驱动 + mock 占位（文档 §6）。
距离只做显示、记录和上层状态输入，不直接驱动云台速度环。
无效/超时不在画面上冒充新值；光轴标定后才把距离认作目标距离。

协议：Siman AA 帧（厂家资料）。
  发送读测量结果：AA 80 00 22 A2
    [0]=0xAA 帧头；[1]=地址|0x80（bit7=1 表示读）；[2:3]=寄存器 0x0022；
    [4]=校验 = sum([1:4]) & 0xFF
  接收 13 字节：[0]=0xAA；[6:10]=距离 4 字节；[10:12]=信号质量；[12]=校验
  退出连续输出：单字节 0x58
  20Hz 超强版波特率默认 115200（厂家资料：高频版 115200，标准版 19200）。

待实物核对（已做成配置项，不用改代码）：
  距离字节序（默认大端）、距离单位（默认毫米）、无回波时的原始值、
  模块上电是否自动连续输出（poll_each_read=false 则只收不问）。
到货后先跑 tools/rangefinder_test.py，看原始 hex 对解析。
"""
import time


class RangeResult:
    def __init__(self, distance_m=None, valid=False, sample_ts=0.0, raw_code=""):
        self.distance_m = distance_m
        self.valid = valid
        self.sample_ts = sample_ts
        self.raw_code = raw_code

    @property
    def age_ms(self):
        return (time.monotonic() - self.sample_ts) * 1000.0


class BaseRangefinder:
    def open(self): ...
    def close(self): ...
    def read(self) -> RangeResult: raise NotImplementedError


class MockRangefinder(BaseRangefinder):
    """到货前的占位：valid=False，画面显示"距离无效"，不干扰跟踪。"""
    def open(self):
        print("[mock] 测距仪占位：无真实距离输出")

    def close(self): pass

    def read(self):
        return RangeResult(valid=False, sample_ts=time.monotonic(), raw_code="MOCK")


class SDBM100(BaseRangefinder):
    HEAD = 0xAA
    REG_RESULT = 0x0022
    EXIT_CONT = 0x58
    FRAME_LEN = 13

    def __init__(self, port="auto", baud=115200, timeout_s=1.0, address=0x00,
                 byteorder="big", max_age_ms=500, poll_each_read=True):
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self.address = address & 0x7F
        self.byteorder = byteorder
        self.max_age_ms = max_age_ms
        self.poll_each_read = poll_each_read
        self.ser = None
        self._buf = b""
        self._last_m = None
        self._last_quality = None
        self._last_ts = 0.0
        self._last_raw = ""

    # ---------- 帧工具 ----------
    @staticmethod
    def checksum(body: bytes) -> int:
        """8 位累加校验：sum(body) & 0xFF（厂家资料）。"""
        return sum(body) & 0xFF

    def build_read_result(self) -> bytes:
        body = bytes([(self.address | 0x80), 0x00, 0x22])
        return bytes([self.HEAD]) + body + bytes([self.checksum(body)])

    def parse_frame(self, f: bytes):
        """解析 13 字节响应帧；成功返回 (distance_m, quality)，失败返回 None。"""
        if len(f) != self.FRAME_LEN or f[0] != self.HEAD:
            return None
        if self.checksum(f[1:12]) != f[12]:
            return None
        dist_raw = int.from_bytes(f[6:10], self.byteorder)
        quality = int.from_bytes(f[10:12], self.byteorder)
        # 无回波/超量程的原始值待实物确认；全 0 或全 F 先判无效
        if dist_raw == 0 or dist_raw >= 0xFFFFFF00:
            return None
        return dist_raw / 1000.0, quality  # 厂家资料距离单位按毫米

    # ---------- 串口 ----------
    def _auto_find(self):
        """逐个探测 /dev/ttyUSB*：发读命令，300ms 内回有效帧的就是测距仪。
        Modbus 设备收到坏 CRC 帧会直接忽略，探测无副作用。"""
        import glob
        for dev in sorted(glob.glob("/dev/ttyUSB*")):
            try:
                import serial
                s = serial.Serial(dev, self.baud, timeout=0.3)
                s.reset_input_buffer()
                s.write(self.build_read_result())
                data = s.read(64)
                s.close()
                if self._scan(data) is not None:
                    return dev
            except Exception:
                continue
        return None

    def _scan(self, data: bytes):
        """从字节流里找最后一个校验通过的 13 字节帧。"""
        self._buf = (self._buf + data)[-512:]
        best = None
        i = 0
        while i < len(self._buf):
            if self._buf[i] == self.HEAD and i + self.FRAME_LEN <= len(self._buf):
                r = self.parse_frame(self._buf[i:i + self.FRAME_LEN])
                if r is not None:
                    best = (r, self._buf[i:i + self.FRAME_LEN].hex())
                    i += self.FRAME_LEN
                    continue
            i += 1
        return best

    def open(self):
        import serial
        port = self.port
        if port == "auto":
            port = self._auto_find()
            if not port:
                raise RuntimeError("auto 未找到 SDBM-100：把 config rangefinder.port 写死为设备路径")
            print(f"[sdbm100] 自动找到串口：{port}")
        # 读超时钳在 0.2s 以内：read() 每帧都调，超时太长会拖慢主循环节拍
        self.ser = serial.Serial(port, self.baud, timeout=min(self.timeout_s, 0.2))
        self.ser.reset_input_buffer()
        self.ser.write(bytes([self.EXIT_CONT]))  # 退出可能存在的连续输出
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self._buf = b""
        print(f"[sdbm100] 已打开 {port} @{self.baud}")

    def close(self):
        try:
            if self.ser:
                self.ser.close()
        finally:
            self.ser = None

    def read(self) -> RangeResult:
        now = time.monotonic()
        try:
            if self.poll_each_read:
                self.ser.write(self.build_read_result())
            # 只等一帧（13 字节）：原来 read(256) 在 poll 模式下每帧都等满超时，
            # 真实硬件一接上主循环就被拖到 ~10fps；按帧长读，健康模块 2ms 内返回
            data = self.ser.read(self.FRAME_LEN)
            hit = self._scan(data)
            if hit is not None:
                (self._last_m, self._last_quality), self._last_raw = hit
                self._last_ts = now
        except Exception as e:
            return RangeResult(valid=False, sample_ts=now, raw_code=f"ERR:{e}")
        age = (now - self._last_ts) * 1000.0 if self._last_ts else float("inf")
        valid = self._last_m is not None and age <= self.max_age_ms
        return RangeResult(distance_m=self._last_m, valid=valid,
                           sample_ts=self._last_ts or now,
                           raw_code=self._last_raw)


def create(cfg):
    c = cfg["rangefinder"]
    if c["backend"] == "sdbm100":
        return SDBM100(port=c.get("port", "auto"), baud=c.get("baud", 115200),
                       timeout_s=c.get("timeout_s", 1.0),
                       address=c.get("address", 0x00),
                       byteorder=c.get("byteorder", "big"),
                       max_age_ms=c.get("max_age_ms", 500),
                       poll_each_read=c.get("poll_each_read", True))
    return MockRangefinder()
