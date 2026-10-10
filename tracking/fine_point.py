"""精瞄：ROI 内质心法算精确瞄准点（粗精两级中的精级）。

粗级（YOLO/FEAR/Kalman）只给出包围盒，盒中心含背景边距，未必是飞机本体中心。
精级在检测框外扩的 ROI 内做二值化，找最大团块求质心，输出原图坐标的精确点。
找不到有效团块（背景杂乱/目标太小）时返回 valid=False，调用方回退用框中心，
绝不硬算一个假点冒充精度。
"""
from dataclasses import dataclass

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


@dataclass
class FinePointResult:
    valid: bool = False
    fx: float = 0.0            # 精瞄点原图 x
    fy: float = 0.0            # 精瞄点原图 y
    fdu_px: float = 0.0        # 精瞄点相对画面中心的像素偏差
    fdv_px: float = 0.0
    area_px: float = 0.0       # 团块像素面积（排错用）
    reason: str = ""           # 无效原因（排错用）


_INVALID = FinePointResult(valid=False)


class FinePointTracker:
    def __init__(self, cfg=None):
        c = (cfg or {}).get("fine_point", {})
        self.enabled = bool(c.get("enabled", True))
        self.roi_expand = float(c.get("roi_expand", 1.5))
        self.min_area_px = float(c.get("min_area_px", 9.0))
        self.max_roi_cover = float(c.get("max_roi_cover", 0.5))
        self.smooth_alpha = float(c.get("smooth_alpha", 0.5))
        self.engage_radius_px = float(c.get("engage_radius_px", 40.0))
        self._prev = None  # EMA 平滑的上一点 (fx, fy)

    def _centroid_of(self, binary):
        cnts, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None, 0.0
        h, w = binary.shape[:2]
        cx0, cy0 = w / 2.0, h / 2.0
        best, best_score = None, None
        for cnt in cnts:
            area = float(cv2.contourArea(cnt))
            if area < self.min_area_px:
                continue
            m = cv2.moments(cnt)
            if m["m00"] <= 0:
                continue
            cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
            # 离 ROI 中心越近、面积越大越像目标（bbox 中心≈目标）
            score = ((cx - cx0) ** 2 + (cy - cy0) ** 2) / (area + 1.0)
            if best_score is None or score < best_score:
                best, best_score = (cx, cy, area), score
        if best is None:
            return None, 0.0
        return (best[0], best[1]), best[2]

    def update(self, image, bbox_xyxy):
        """输入原图 BGR 与检测框，返回 FinePointResult。"""
        if not self.enabled or cv2 is None or image is None:
            return FinePointResult(valid=False, reason="disabled" if self.enabled else "no-cv2")
        try:
            x1, y1, x2, y2 = (float(v) for v in bbox_xyxy)
        except (TypeError, ValueError):
            return FinePointResult(valid=False, reason="bad-bbox")
        ih, iw = image.shape[:2]
        if x2 <= x1 or y2 <= y1:
            return FinePointResult(valid=False, reason="bad-bbox")
        bw, bh = x2 - x1, y2 - y1
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        rw, rh = bw * self.roi_expand, bh * self.roi_expand
        rx1 = max(0, int(cx - rw / 2.0))
        ry1 = max(0, int(cy - rh / 2.0))
        rx2 = min(iw, int(cx + rw / 2.0))
        ry2 = min(ih, int(cy + rh / 2.0))
        if rx2 - rx1 < 3 or ry2 - ry1 < 3:
            return FinePointResult(valid=False, reason="roi-too-small")
        roi = image[ry1:ry2, rx1:rx2]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 0, 255,
                                 cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        (cc, area) = self._centroid_of(binary)
        if cc is None:
            return FinePointResult(valid=False, reason="no-blob")
        roi_area = float((rx2 - rx1) * (ry2 - ry1))
        if area / max(1.0, roi_area) > self.max_roi_cover:
            # 最大"团块"占了半个 ROI，多半是把背景当成了目标，取反重试
            (cc, area) = self._centroid_of(cv2.bitwise_not(binary))
            if cc is None:
                return FinePointResult(valid=False, reason="bg-dominant")
        fx = rx1 + cc[0]
        fy = ry1 + cc[1]
        if self._prev is not None and self.smooth_alpha < 1.0:
            a = self.smooth_alpha
            fx = a * fx + (1.0 - a) * self._prev[0]
            fy = a * fy + (1.0 - a) * self._prev[1]
        self._prev = (fx, fy)
        return FinePointResult(valid=True, fx=fx, fy=fy,
                               fdu_px=fx - iw / 2.0, fdv_px=fy - ih / 2.0,
                               area_px=area)

    def reset(self):
        self._prev = None
