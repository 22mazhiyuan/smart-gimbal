"""IBVS 视觉伺服：像素偏差 -> 角速度（文档 §4）。
在 gimbal_tracker/ibvs.py 基础上加变化率限制（slew），防检测框抖动变成电机猛动。
非 TRACK 状态调用方只传 visible=False，本模块自然输出零。
"""
from state_machine import TRACK  # noqa: F401  状态名统一

IDLE, TRACK, LOST = "IDLE", "TRACK", "LOST"


class IBVSConfig:
    def __init__(self, cfg=None, **kw):
        c = cfg["ibvs"] if cfg else {}
        get = lambda k, d: kw.get(k, c.get(k, d))
        self.kp = get("kp", 0.05)
        self.deadzone_px = get("deadzone_px", 10.0)
        self.max_vel_dps = get("max_vel_dps", 30.0)
        self.slew_dps2 = get("slew_dps2", 120.0)
        self.dir_sign_pitch = get("dir_sign_pitch", 1)
        self.dir_sign_yaw = get("dir_sign_yaw", 1)
        self.enable_yaw = get("enable_yaw", False)


class IBVSTracker:
    def __init__(self, cfg: IBVSConfig):
        self.cfg = cfg
        self._prev = (0.0, 0.0)

    def _axis(self, err, sign, prev, dt):
        v = 0.0 if abs(err) < self.cfg.deadzone_px else self.cfg.kp * err * sign
        v = max(-self.cfg.max_vel_dps, min(self.cfg.max_vel_dps, v))
        # 变化率限制
        dv_max = self.cfg.slew_dps2 * dt
        v = prev + max(-dv_max, min(dv_max, v - prev))
        return v

    def update(self, du, dv, visible, dt):
        """返回 (pitch_vel, yaw_vel)。不可见 -> 零（状态机管 LOST 计时）。"""
        if dt <= 0:
            dt = 0.05
        if not visible:
            self._prev = (0.0, 0.0)
            return 0.0, 0.0
        pv = self._axis(dv, self.cfg.dir_sign_pitch, self._prev[0], dt)
        yv = self._axis(du, self.cfg.dir_sign_yaw, self._prev[1], dt) \
            if self.cfg.enable_yaw else 0.0
        self._prev = (pv, yv)
        return pv, yv
