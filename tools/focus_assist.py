"""对焦辅助：实时计算画面清晰度（Laplacian 方差），手动转镜头时看分数调到最高再锁死。
跑：python3 tools/focus_assist.py
按 Ctrl+C 退出，自动保存对焦曲线 focus_curve.csv。

原理：M12 手动头没有调焦电机，代码动不了镜头，只能测清晰度辅助人手调。
想全自动闭环，得加调焦执行机构（步进电机拧镜头，或换电动变焦一体机）。
"""
import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config_center import load_config
from video import source as video_source

try:
    import cv2
except ImportError:
    sys.exit("没装 opencv")


def sharpness(gray, roi_frac=0.4):
    """中心 ROI 的 Laplacian 方差；跟踪目标一般在中心，中心清晰最重要。"""
    h, w = gray.shape
    x1, y1 = int(w * (0.5 - roi_frac / 2)), int(h * (0.5 - roi_frac / 2))
    x2, y2 = int(w * (0.5 + roi_frac / 2)), int(h * (0.5 + roi_frac / 2))
    roi = gray[y1:y2, x1:x2]
    return float(cv2.Laplacian(roi, cv2.CV_64F).var())


def main():
    cfg = load_config()
    src = video_source.create(cfg)
    src.open()
    print("对焦辅助启动：缓慢转动镜头，看分数往最高走，最高时锁死。Ctrl+C 退出。")
    curve, peak, t0 = [], 0.0, time.monotonic()
    try:
        while True:
            f = src.read()
            if f is None:
                time.sleep(0.2)
                continue
            gray = cv2.cvtColor(f.image, cv2.COLOR_BGR2GRAY)
            s = sharpness(gray)
            peak = max(peak, s)
            curve.append((round(time.monotonic() - t0, 1), round(s, 1)))
            bar = "#" * int(40 * s / max(peak, 1e-6))
            print(f"\r清晰度 {s:8.1f}  峰值 {peak:8.1f} |{bar:<40}|", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        src.close()
        with open("focus_curve.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_s", "sharpness"])
            w.writerows(curve)
        print(f"\n峰值 {peak:.1f}，曲线已存 focus_curve.csv（回看确认调到了峰顶）")


if __name__ == "__main__":
    main()
