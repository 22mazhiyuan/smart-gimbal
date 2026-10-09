"""ROS 2 检测适配：订阅 /counter_uav/tracking/primary_target -> DetectionResult。

rclpy 在后台线程 spin，主循环照常跑，不阻塞。
visible 判定：is_primary 且 lost_frames==0 且置信度达标。
du/dv = 目标中心 - 画面中心（画面尺寸按视频帧实际宽高，不写死）。
"""
import threading

from detection.interface import BaseDetector, DetectionResult


class RosTrackDetector(BaseDetector):
    def __init__(self, img_w=0, img_h=0,
                 topic="/counter_uav/tracking/primary_target",
                 conf_thresh=0.4):
        self.img_w, self.img_h = img_w, img_h
        self.topic = topic
        self.conf_thresh = conf_thresh
        self._latest = None
        self._lock = threading.Lock()
        self._node = None

    def open(self):
        try:
            import rclpy
            from rclpy.node import Node
        except ImportError:
            raise RuntimeError("没装 rclpy")
        try:
            from anti_drone_interfaces.msg import Track
        except ImportError:
            raise RuntimeError("没找到 anti_drone_interfaces，先 colcon build 装好")
        node = Node("smart_gimbal_track_sub")

        def cb(msg):
            with self._lock:
                self._latest = msg

        node.create_subscription(Track, self.topic, cb, 10)
        self._node = node
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        print(f"[ROS] 已订阅 {self.topic}（后台线程 spin）")

    def close(self):
        if self._node:
            self._node.destroy_node()
            self._node = None

    def process(self, frame) -> DetectionResult:
        with self._lock:
            msg = self._latest
        if msg is None:
            return DetectionResult(frame_id=frame.frame_id, visible=False)
        try:
            conf = float(getattr(msg, "confidence", 0.0) or 0.0)
            valid = bool(getattr(msg, "is_primary", False)) \
                and int(getattr(msg, "lost_frames", 1) or 0) == 0 \
                and conf >= self.conf_thresh
        except Exception:
            valid, conf = False, 0.0
        if not valid:
            return DetectionResult(frame_id=frame.frame_id, visible=False, conf=conf)
        w = self.img_w or frame.image.shape[1]
        h = self.img_h or frame.image.shape[0]
        cu = float(msg.center_u_px)
        cv = float(msg.center_v_px)
        seq = int(getattr(msg, "sequence", 0) or frame.frame_id)
        return DetectionResult(
            frame_id=seq, visible=True,
            bbox_xyxy=(float(msg.xmin_px), float(msg.ymin_px),
                       float(msg.xmax_px), float(msg.ymax_px)),
            label=str(getattr(msg, "class_name", "") or ""),
            conf=conf, du_px=cu - w / 2.0, dv_px=cv - h / 2.0)


def create_detector(cfg, img_w=0, img_h=0):
    b = cfg["model"]["backend"]
    if b == "ros_track":
        return RosTrackDetector(img_w, img_h,
                                cfg["model"].get("ros_topic",
                                                "/counter_uav/tracking/primary_target"),
                                cfg["model"].get("conf_thresh", 0.4))
    from detection.interface import MockDetector
    return MockDetector()
