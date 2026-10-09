"""测距辅助对焦链路：测距仪距离 -> 薄透镜公式 -> VCM DAC -> 电机。
就是手机"激光对焦"的原理：距离已知，直接算像距，不用爬山搜索。

薄透镜：1/f = 1/do + 1/di，镜片相对无穷远位置的位移 Δ = f²/(do - f)。
f=16mm 时：30m->8.5µm，100m->2.6µm，10m->25.6µm（VCM 行程通常 100~300µm，够用）。

链路保护：无效距离保持不动；DAC 死区防抖动；限频防 hunting。
"""
import time


def displacement_um(f_mm: float, d_m: float) -> float:
    """镜片相对无穷远位置的位移（µm）。"""
    do = d_m * 1000.0
    if do <= f_mm or d_m <= 0:
        return 0.0
    return f_mm * f_mm / (do - f_mm) * 1000.0


def distance_to_dac(d_m: float, f_mm: float, dac_infinity: int, um_per_dac: float) -> int:
    return int(round(dac_infinity + displacement_um(f_mm, d_m) / um_per_dac))


class RangingAF:
    def __init__(self, cfg, driver):
        c = cfg.get("autofocus", {})
        self.f_mm = c.get("f_mm", 16.0)
        self.dac_inf = c.get("dac_infinity", 512)
        self.um_per_dac = c.get("um_per_dac", 0.2)
        self.deadband = c.get("deadband_dac", 3)
        self.min_interval = 1.0 / c.get("max_rate_hz", 2)
        self.driver = driver
        self._last_dac = None
        self._last_t = 0.0

    def update(self, distance_m, valid) -> tuple:
        """返回 (dac, moved)。无效距离 -> 保持上次位置不动。"""
        if not valid or distance_m is None or distance_m <= 0:
            return self._last_dac, False
        dac = distance_to_dac(distance_m, self.f_mm, self.dac_inf, self.um_per_dac)
        now = time.monotonic()
        if self._last_dac is not None and abs(dac - self._last_dac) < self.deadband:
            return self._last_dac, False  # 死区内不动
        if now - self._last_t < self.min_interval:
            return self._last_dac, False  # 限频
        self.driver.set_dac(dac)
        self._last_dac, self._last_t = dac, now
        return dac, True
