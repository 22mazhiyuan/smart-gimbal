"""检测接口：统一 frame_id 数据链（文档 §2）。
真实双模型由 Codex 按 BaseDetector 实现；mock 先跑通全链路。
"""
import time
from dataclasses import dataclass, field


@dataclass
class DetectionResult:
    frame_id: int
    visible: bool
    bbox_xyxy: tuple = (0, 0, 0, 0)   # 原图坐标
    label: str = ""
    conf: float = 0.0
    du_px: float = 0.0
    dv_px: float = 0.0
    ts: float = field(default_factory=time.monotonic)
    predicted: bool = False           # 预测不作为有效观测（文档 §3）


class BaseDetector:
    def open(self): ...
    def close(self): ...
    def process(self, frame) -> DetectionResult:
        raise NotImplementedError


class MockDetector(BaseDetector):
    """虚拟目标：从 +200px 漂进中心，测整条链路，不依赖模型。"""
    def __init__(self, width=1280, height=720):
        self.w, self.h = width, height
        self._dv = 200.0

    def open(self):
        print("[mock] 检测器占位：虚拟目标")

    def close(self): pass

    def process(self, frame) -> DetectionResult:
        cx, cy = self.w // 2, self.h // 2
        bw, bh = 60, 40
        x1, y1 = cx - bw // 2, int(cy + self._dv) - bh // 2
        return DetectionResult(
            frame_id=frame.frame_id, visible=True,
            bbox_xyxy=(x1, y1, x1 + bw, y1 + bh),
            label="MULTIROTOR", conf=0.9,
            du_px=0.0, dv_px=self._dv)

    def apply_motion(self, pitch_vel_dps, dt, px_per_deg=20.0):
        self._dv -= pitch_vel_dps * dt * px_per_deg
