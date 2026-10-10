#!/usr/bin/env python3
"""Connect the existing camera/ROS node to real YOLO -> FEAR -> Kalman.

This module never creates a camera, RTSP server, ROS node or actuator. A caller
must give it the current captured BGR frame and that frame's monotonic timestamp.
Pure Kalman predictions are deliberately hidden, never published as detections.
"""
from __future__ import annotations

import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any


OFFICIAL_FEAR_CHECKPOINT_SHA256 = "8efe7dfd3498e385f332fd655f360a848d723f5fd77c2d433b2c084029616be5"
OFFICIAL_FEAR_BLOCKS_SHA256 = "b3460f867f3e8aa301ecd752fee7a12bfea32d1df0fa54e555624702764a3ab8"
MOBILE_VISION_REVISION = "51804a6873ae1029257cf652179c960cceeecc75"


def sha256_file(path: Any) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _number(value: Any, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a number, not bool")
    result = float(value)
    if not math.isfinite(result) or result < 0 or (positive and result == 0):
        raise ValueError(f"{name} must be finite and {'positive' if positive else 'nonnegative'}")
    return result


def build_fusion_config(smart_config: dict, tracker_config: dict | None = None,
                        fusion_config_class: Any = None) -> tuple[Any, dict]:
    """Map the smart-gimbal YAML into the existing FusionConfig, no tuning.

    YAML Q/R are variances. The core squares acceleration_std/measurement_std,
    so sqrt(Q)/sqrt(R) is required, rather than passing variances as stddevs.
    tracker_config is the complete tracker config.json, not only its subsection.
    This helper deliberately has no FEAR/CUDA/camera/ROS imports for fixture QA.
    """
    if not isinstance(smart_config, dict):
        raise TypeError("smart config must be a mapping")
    target = smart_config.get("target_select")
    model = smart_config.get("model")
    if not isinstance(target, dict) or not isinstance(model, dict):
        raise ValueError("smart config needs target_select and model mappings")
    required = ("kalman_process_noise", "kalman_measure_noise", "predict_horizon_s",
                "recapture_iou_thresh", "priority")
    missing = [key for key in required if key not in target]
    if missing or "conf_thresh" not in model:
        raise ValueError(f"Required smart fusion settings missing: {missing}; model.conf_thresh required")
    if target["priority"] != "largest_conf":
        raise ValueError("Only the configured largest_conf target priority is supported; no silent substitution")
    process_variance = _number(target["kalman_process_noise"], "kalman_process_noise", positive=True)
    measure_variance = _number(target["kalman_measure_noise"], "kalman_measure_noise", positive=True)
    prediction_horizon = _number(target["predict_horizon_s"], "predict_horizon_s")
    recapture_iou = _number(target["recapture_iou_thresh"], "recapture_iou_thresh")
    detection_threshold = _number(model["conf_thresh"], "model.conf_thresh")
    tracking = (tracker_config or {}).get("tracking", {})
    if tracking.get("mode", "YOLO_FEAR_KALMAN") != "YOLO_FEAR_KALMAN":
        raise ValueError("The production bridge requires YOLO_FEAR_KALMAN")
    settings = dict(tracking.get("config", {}))
    original_max_coast = _number(settings.get("max_coast_s", 0.25), "tracker.max_coast_s")
    settings.update(
        detection_threshold=detection_threshold,
        acceleration_std=math.sqrt(process_variance),
        measurement_std=math.sqrt(measure_variance),
        max_coast_s=min(original_max_coast, prediction_horizon),
        match_iou_min=recapture_iou,
    )
    if fusion_config_class is None:
        from fusion_core import FusionConfig
        fusion_config_class = FusionConfig
    config = fusion_config_class(**settings)
    report = {
        "mode": "YOLO_FEAR_KALMAN",
        "priority": "largest_conf",
        "priority_semantics": "existing core highest-confidence acquisition; spatial association thereafter",
        "smart_to_core": {
            "kalman_process_noise": {
                "input_variance": process_variance, "acceleration_std": config.acceleration_std,
                "effective_process_variance": config.acceleration_std ** 2,
                "Q_semantics": "Q(dt) = G(dt) @ G(dt).T * kalman_process_noise",
            },
            "kalman_measure_noise": {
                "input_variance": measure_variance, "measurement_std": config.measurement_std,
                "effective_measure_variance": config.measurement_std ** 2,
                "R_semantics": "R = I4 * kalman_measure_noise",
            },
            "predict_horizon_s": prediction_horizon,
            "tracker_max_coast_s": original_max_coast,
            "effective_max_coast_s": config.max_coast_s,
            "recapture_iou_thresh": config.match_iou_min,
            "recapture_semantics": "existing core IoU OR normalized-center gate; no change to association algorithm",
            "model.conf_thresh": config.detection_threshold,
        },
        "tracking_threshold": config.tracking_threshold,
        "verify_interval_s": config.verify_interval_s,
        "confirm_frames": config.confirm_frames,
        "max_frame_age_s": config.max_frame_age_s,
        "prediction_policy": "hidden; no primary_target and no valid observation on pure prediction",
        "threshold_tuning_performed": False,
    }
    return config, report


def publication_payload(row: dict) -> dict:
    """Pure ROS publication gate; FEAR is measured tracking, not recognition.

    A class-verified FEAR observation may be primary even though recognized is
    False. An unobserved Kalman prediction may never be primary, even when the
    core still remembers a formerly verified class and retains a finite box.
    """
    source = row.get("source", "NONE")
    hidden = source == "PREDICTION"
    payload = {"primary": False, "bbox": None, "score": 0.0,
               "prediction_hidden": hidden}
    if (source not in ("YOLO", "FEAR") or not row.get("accepted", False)
            or not row.get("class_verified", False)
            or not row.get("primary_allowed", True)):
        return payload
    try:
        bbox = [float(value) for value in row.get("bbox", ())]
        score = float(row.get("score"))
    except (TypeError, ValueError):
        return payload
    if (len(bbox) != 4 or not all(math.isfinite(value) for value in bbox)
            or bbox[2] <= 0 or bbox[3] <= 0 or not math.isfinite(score)
            or not 0 <= score <= 1):
        return payload
    payload.update(primary=True, bbox=bbox, score=score)
    return payload


class FusionBridge:
    """A synchronous consumer of the current camera owner's latest frame."""

    def __init__(self, model: Any, model_path: str,
                 tracker_root: str = "/home/jetson/multirotor_tracker",
                 smart_config: str = "/home/jetson/visual_gimbal_lab/smart_gimbal/config.yaml",
                 runtime_params: dict | None = None):
        if os.environ.get("GIMBAL_REAL_MOTION", "0") == "1":
            raise RuntimeError("GIMBAL_REAL_MOTION=1 is forbidden for this mock-only integration")
        self.closed = False
        self.model = model
        self.model_path = Path(model_path).expanduser().resolve()
        self.tracker_root = Path(tracker_root).expanduser().resolve()
        self.smart_config_path = Path(smart_config).expanduser().resolve()
        if not self.model_path.is_file():
            raise FileNotFoundError(f"Missing existing YOLO weights: {self.model_path}")
        tracker_config_path = self.tracker_root / "config.json"
        tracker_cfg = json.loads(tracker_config_path.read_text(encoding="utf-8"))
        import yaml
        smart_cfg = yaml.safe_load(self.smart_config_path.read_text(encoding="utf-8"))
        if not isinstance(smart_cfg, dict) or smart_cfg.get("gimbal", {}).get("mode") != "mock":
            raise RuntimeError("smart config gimbal.mode must remain mock")
        params = dict(tracker_cfg.get("runtime", {}))
        params.update(runtime_params or {})
        self.device = int(params.get("device", 0))
        self.detector_conf = float(params.get("conf", 0.35))
        self.imgsz = int(params.get("imgsz", 640))
        self.detector_iou = float(params.get("iou", 0.7))
        # Keep existing detector prefilter; YAML 0.4 gates FusionCore acceptance.
        if self.detector_conf != 0.35 or self.imgsz != 640 or self.detector_iou != 0.7:
            raise ValueError("Existing YOLO parameters must stay conf=0.35,imgsz=640,iou=0.7; no threshold tuning")
        fear_cfg = dict(tracker_cfg.get("tracking", {}).get("fear", {}))
        if fear_cfg.get("backend", "torch_cuda") != "torch_cuda":
            raise ValueError("Use the current validated torch_cuda FEAR backend; no CPU/ONNX substitution")
        checkpoint = self._project_path(fear_cfg.get("checkpoint_path", "models/FEAR-XS-NoEmbs.ckpt"))
        source = self._project_path(fear_cfg.get("source_dir", "vendor/fear_original"))
        blocks = source / "model_training" / "model" / "blocks.py"
        if not checkpoint.is_file() or not blocks.is_file():
            raise FileNotFoundError(f"Required real FEAR inputs missing: checkpoint={checkpoint}, blocks={blocks}")
        checkpoint_hash, blocks_hash = sha256_file(checkpoint), sha256_file(blocks)
        if checkpoint_hash != OFFICIAL_FEAR_CHECKPOINT_SHA256:
            raise ValueError(f"FEAR checkpoint SHA256 mismatch: {checkpoint_hash}")
        if blocks_hash != OFFICIAL_FEAR_BLOCKS_SHA256:
            raise ValueError(f"Official pinned FEAR blocks.py SHA256 mismatch: {blocks_hash}")
        mobile_root = self.tracker_root / "vendor" / ("mobile-vision-" + MOBILE_VISION_REVISION)
        dependency_root = self.tracker_root / "vendor" / "python_deps"
        if not (mobile_root / "mobile_cv").is_dir() or not dependency_root.is_dir():
            raise FileNotFoundError(f"Pinned FEAR dependencies required: {mobile_root}; {dependency_root}")
        # Loading only these existing project paths leaves global packages untouched.
        for dependency in reversed([self.tracker_root, mobile_root, dependency_root]):
            if str(dependency) not in sys.path:
                sys.path.insert(0, str(dependency))
        core_module = importlib.import_module("fusion_core")
        fear_module = importlib.import_module("fear_backend")
        for module, filename in ((core_module, "fusion_core.py"), (fear_module, "fear_backend.py")):
            if Path(module.__file__).resolve() != self.tracker_root / filename:
                raise ImportError(f"Wrong {module.__name__} provenance: {module.__file__}; expected {self.tracker_root / filename}")
        fusion_config, mapping_report = build_fusion_config(smart_cfg, tracker_cfg, core_module.FusionConfig)
        self.config = fusion_config
        self._detections: list[dict] = []
        self._detector_callback_calls = 0
        self._last_detector_ms = 0.0
        self._frames_processed = 0
        self._prediction_suppressed = 0
        self._publication_counts = {"YOLO": 0, "FEAR": 0, "PREDICTION": 0, "NONE": 0}
        self.backend = fear_module.FEARBackend(
            backend="torch_cuda", checkpoint_path=str(checkpoint), source_dir=str(source),
            expected_checkpoint_sha256=OFFICIAL_FEAR_CHECKPOINT_SHA256,
            device_id=self.device, smooth=False,
        )
        self.core = core_module.FusionCore(
            mode="YOLO_FEAR_KALMAN", detector=self._detect,
            tracker=self.backend, config=self.config,
        )
        self.config_report = dict(mapping_report,
            tracker_root=str(self.tracker_root), smart_config=str(self.smart_config_path),
            camera_ownership="unchanged: frame injected by video_detect_runtime.FrameStore",
            actuator_access=False, motion_backend="mock",
            yolo_predict={"device": self.device, "conf": self.detector_conf, "imgsz": self.imgsz,
                          "iou": self.detector_iou, "rect": False},
            sha256={
                "yolo_weights": sha256_file(self.model_path),
                "FEAR_checkpoint": checkpoint_hash, "official_blocks": blocks_hash,
                "fusion_core": sha256_file(self.tracker_root / "fusion_core.py"),
                "fear_backend": sha256_file(self.tracker_root / "fear_backend.py"),
                "smart_config": sha256_file(self.smart_config_path),
                "tracker_config": sha256_file(tracker_config_path),
            },
        )
        # This warms CUDA kernels only. It never initializes a track or claims
        # target recognition. Backend call counts are recorded separately.
        try:
            self._warmup(max(0, int(params.get("warmup_calls", 3))))
        except Exception:
            self.backend.close()
            raise

    def _project_path(self, value: Any) -> Path:
        path = Path(str(value)).expanduser()
        return (path if path.is_absolute() else self.tracker_root / path).resolve()

    def _warmup(self, calls: int) -> None:
        import numpy as np
        started = time.monotonic()
        before = dict(self.backend.description)
        for _ in range(calls):
            features = self.backend.runtime.template(np.zeros((1, 3, 128, 128), dtype=np.float32))
            self.backend.runtime.search(np.zeros((1, 3, 256, 256), dtype=np.float32), features)
        self.config_report["cuda_warmup"] = {
            "calls": calls, "duration_ms": (time.monotonic() - started) * 1000,
            "template_calls": self.backend.description.get("template_calls", 0) - before.get("template_calls", 0),
            "search_calls": self.backend.description.get("search_calls", 0) - before.get("search_calls", 0),
            "scope": "synthetic tensor CUDA initialization only; not target acceptance",
        }

    def _detect(self, frame: Any) -> list[dict]:
        import numpy as np
        started = time.monotonic()
        self._detector_callback_calls += 1
        result = self.model.predict(
            frame, device=self.device, conf=self.detector_conf, imgsz=self.imgsz,
            rect=False, iou=self.detector_iou, verbose=False, save=False,
        )[0]
        detections: list[dict] = []
        if result.boxes is not None:
            for box in result.boxes:
                xyxy = box.xyxy[0].detach().cpu().numpy()
                score, cls = float(box.conf[0]), int(box.cls[0])
                label = str(result.names[cls])
                if not np.isfinite(xyxy).all() or not math.isfinite(score) or score < self.detector_conf:
                    continue
                x1, y1, x2, y2 = (float(x) for x in xyxy)
                if x2 <= x1 or y2 <= y1:
                    continue
                # Only the actual model class is recognized. Never relabel a
                # foreign class merely because its box has high confidence.
                detections.append({"bbox": [x1, y1, x2 - x1, y2 - y1], "score": score,
                                   "label": label.strip().upper(), "model_label": label,
                                   "xyxy": [x1, y1, x2, y2], "source": "YOLO",
                                   "recognized": label.strip().upper() == "MULTIROTOR"})
        self._detections = detections
        self._last_detector_ms = (time.monotonic() - started) * 1000
        return detections

    @staticmethod
    def _bounded_bbox(bbox: Any, width: int, height: int) -> list[float] | None:
        if bbox is None or len(bbox) != 4:
            return None
        x, y, w, h = map(float, bbox)
        if not all(math.isfinite(v) for v in (x, y, w, h)) or w <= 0 or h <= 0:
            return None
        x1, y1 = max(0.0, x), max(0.0, y)
        x2, y2 = min(float(width), x + w), min(float(height), y + h)
        if x2 - x1 < 3 or y2 - y1 < 3:
            return None
        return [x1, y1, x2 - x1, y2 - y1]

    def process(self, frame: Any, seq: int, source_mono: float, now: float | None = None) -> dict:
        if self.closed:
            raise RuntimeError("FusionBridge is closed")
        started = time.monotonic()
        self._detections = []
        self._last_detector_ms = 0.0
        result = self.core.process(frame, int(seq), float(source_mono), now=now)
        events = list(result["events"])
        # Core records exceptions rather than propagating them. A production
        # bridge must make those visible and refuse to quietly become YOLO-only.
        failures = [event for event in events if event.startswith(
            ("DETECTOR_ERROR:", "TRACKER_ERROR:", "TRACKER_INIT_ERROR:", "TRACKER_UNAVAILABLE", "DETECTOR_UNAVAILABLE"))]
        if failures:
            raise RuntimeError("Fusion runtime failed closed: " + " | ".join(failures))
        source = result["source"]
        observed = source in ("YOLO", "FEAR") and bool(result["accepted"]) and bool(result["class_verified"])
        width, height = int(frame.shape[1]), int(frame.shape[0])
        bbox = self._bounded_bbox(result.get("bbox"), width, height) if observed else None
        raw_bbox = self._bounded_bbox(result.get("raw_bbox"), width, height) if observed else None
        score = result.get("score")
        if observed and (bbox is None or raw_bbox is None or score is None):
            observed = False
            events.append("OBSERVATION_INVALID_OR_OUTSIDE_FRAME")
        frame_age = time.monotonic() - float(source_mono)
        if frame_age > self.config.max_frame_age_s:
            observed = False
            events.append("FUSION_COMPLETION_EXPIRED_FRAME")
        if not observed:
            bbox = raw_bbox = None
        if source == "PREDICTION":
            self._prediction_suppressed += 1
            events.append("PREDICTION_HIDDEN_NOT_PRIMARY")
        self._frames_processed += 1
        self._publication_counts[source] = self._publication_counts.get(source, 0) + 1
        output = dict(result)
        output.update(
            bbox=bbox, raw_bbox=raw_bbox, source=source, recognized=source == "YOLO" and observed,
            primary_allowed=observed, display_allowed=observed,
            label="MULTIROTOR" if observed else None,
            annotation={"YOLO": "DETECT YOLO+KF", "FEAR": "TRACK FEAR+KF",
                        "PREDICTION": "PREDICT HIDDEN", "NONE": "NO OBSERVATION"}.get(source, source),
            events=events, detections=list(self._detections),
            prediction_only=source == "PREDICTION", pure_prediction_published=False,
            suppressed_prediction_bbox=result.get("bbox") if source == "PREDICTION" else None,
            source_mono=float(source_mono), frame_age_s=frame_age,
            latency_ms=(time.monotonic() - started) * 1000, detector_latency_ms=self._last_detector_ms,
            metrics=self.metrics, backend=self.backend_description,
        )
        return output

    @property
    def backend_description(self) -> dict:
        return dict(self.backend.description)

    @property
    def metrics(self) -> dict:
        return dict(self.core.metrics, bridge_frames_processed=self._frames_processed,
                    bridge_detector_callbacks=self._detector_callback_calls,
                    bridge_prediction_suppressed=self._prediction_suppressed,
                    source_counts=dict(self._publication_counts))

    def close(self) -> dict:
        if not self.closed:
            self.closed = True
            self.backend.close()
        return {"metrics": self.metrics, "backend": self.backend_description,
                "config_report": self.config_report}
