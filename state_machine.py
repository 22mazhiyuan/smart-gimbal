"""状态机：IDLE / TRACK / LOST / ERROR / STOPPED（文档 §3）。
状态变化记录时间+原因；纯预测不进 TRACK；非 TRACK 业务层只输出零速度。
"""
import time

IDLE, TRACK, LOST, ERROR, STOPPED = "IDLE", "TRACK", "LOST", "ERROR", "STOPPED"


class StateMachine:
    def __init__(self, cfg):
        c = cfg["state_machine"]
        self.min_conf = c["track_min_conf"]
        self.min_hits = c["track_min_hits"]
        self.lost_confirm = c["lost_confirm_s"]
        self.state = IDLE
        self._hits = 0
        self._lost_t = 0.0
        self.history = []  # (时间, 从, 到, 原因)

    def _go(self, to, reason):
        if to != self.state:
            self.history.append((time.strftime("%H:%M:%S"), self.state, to, reason))
            print(f"[状态] {self.state} -> {to}：{reason}")
            self.state = to

    def update(self, visible, conf, fault=None, fault_reason="", dt=0.05):
        if fault:
            self._go(ERROR, fault_reason or "故障")
            return self.state
        if self.state == ERROR:
            # 故障恢复：需重新满足 TRACK 条件
            self._hits = 0
            self._go(IDLE, "故障清除，重新捕获")
        good = visible and conf >= self.min_conf
        if self.state in (IDLE, LOST):
            if good:
                self._hits += 1
                if self._hits >= self.min_hits:
                    self._go(TRACK, f"连续{self._hits}帧有效观测")
            else:
                self._hits = 0
        elif self.state == TRACK:
            if good:
                self._lost_t = 0.0
            else:
                self._lost_t += dt
                if self._lost_t >= self.lost_confirm:
                    self._hits = 0
                    self._go(LOST, f"丢失{self._lost_t:.1f}s")
        return self.state

    def stop(self, reason="退出"):
        self._go(STOPPED, reason)
