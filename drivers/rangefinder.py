"""测距：SDBM-100 接口 + mock 占位（文档 §6）。
3Hz 只做显示、记录和上层状态输入，不直接驱动云台速度环。
无效/超时不在画面上冒充新值；光轴标定后才把距离认作目标距离。
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
    """真实 SDBM-100：协议按实物资料由 Codex 对接；接口已固定。"""
    def __init__(self, port="auto", baud=115200, timeout_s=1.0):
        self.port, self.baud, self.timeout_s = port, baud, timeout_s

    def open(self):
        raise NotImplementedError("SDBM-100 协议待实物资料对接，先用 mock")

    def close(self): pass

    def read(self):
        raise NotImplementedError


def create(cfg):
    c = cfg["rangefinder"]
    if c["backend"] == "sdbm100":
        return SDBM100(c["port"], c["baud"], c["timeout_s"])
    return MockRangefinder()
