"""视频源：RTSP / mock。frame_id 连续编号；断流返回 None，主循环归零并重连（文档 §3/§7）。"""
import time

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


class Frame:
    def __init__(self, frame_id, image, ts):
        self.frame_id = frame_id
        self.image = image
        self.ts = ts  # 单调时钟


class RTSPSource:
    def __init__(self, url, reconnect_s=2.0, width=0, height=0):
        self.url, self.reconnect_s = url, reconnect_s
        self.w, self.h = width, height
        self._cap, self._fid, self._last_try = None, 0, 0.0

    def open(self):
        if not HAS_CV2:
            raise RuntimeError("没装 opencv，Jetson 上 pip install opencv-python")
        # ！！/dev/video0 被 jiami.py 占用，只能走 RTSP 只读流，别碰摄像头设备
        self._connect()

    def _connect(self):
        # ffmpeg 打开/读取超时设 5 秒：代理没在跑时快速失败，不卡 30 秒
        self._cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG, [
            cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000,
            cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000,
        ])
        self._last_try = time.monotonic()

    def read(self):
        if self._cap is None or not self._cap.isOpened():
            if time.monotonic() - self._last_try > self.reconnect_s:
                print("[视频] RTSP 断开，重连中…（控制已归零）")
                self._connect()
            return None
        ok, img = self._cap.read()
        if not ok:
            self._cap.release()
            self._cap = None
            return None
        self._fid += 1
        return Frame(self._fid, img, time.monotonic())

    def close(self):
        if self._cap:
            self._cap.release()


class MockSource:
    def __init__(self, width=1280, height=720):
        self.w, self.h, self._fid = width, height, 0

    def open(self): print("[mock] 视频源占位")

    def read(self):
        if not HAS_CV2:
            raise RuntimeError("mock 视频源也需要 opencv")
        self._fid += 1
        img = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        return Frame(self._fid, img, time.monotonic())

    def close(self): pass


def create(cfg):
    c = cfg["video"]
    if c["source"] == "rtsp":
        return RTSPSource(c["rtsp_url"], c["reconnect_interval_s"], c["width"], c["height"])
    return MockSource(c["width"] or 1280, c["height"] or 720)
