"""运行日志：每次运行独立目录，配置快照 + 结构化日志 + 延迟统计 + 验收摘要（文档 §3/§7）。"""
import json
import os
import shutil
import statistics
import time


class LatencyTracker:
    """同一 frame_id 记录收帧/推理/指令/编码时间；输出中位数、P95、最大（文档 §7）。"""
    def __init__(self):
        self._marks = {}

    def mark(self, fid, stage):
        self._marks.setdefault(fid, {})[stage] = time.monotonic()

    def report(self):
        spans = {}
        for fid, m in self._marks.items():
            if all(k in m for k in ("got", "infer", "cmd", "show")):
                spans.setdefault("收帧->推理", []).append(m["infer"] - m["got"])
                spans.setdefault("推理->指令", []).append(m["cmd"] - m["infer"])
                spans.setdefault("指令->画面", []).append(m["show"] - m["cmd"])
        out = {}
        for k, v in spans.items():
            v.sort()
            out[k] = {"median_ms": round(statistics.median(v) * 1000, 1),
                      "p95_ms": round(v[int(len(v) * 0.95)] * 1000, 1),
                      "max_ms": round(max(v) * 1000, 1),
                      "n": len(v)}
        return out


class RunLogger:
    def __init__(self, cfg, config_path="config.yaml"):
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.dir = os.path.join(cfg["logging"]["dir"], ts)
        os.makedirs(self.dir, exist_ok=True)
        shutil.copy(config_path, os.path.join(self.dir, "config_snapshot.yaml"))
        self._log = open(os.path.join(self.dir, "frames.jsonl"), "a", encoding="utf-8")
        self._events = open(os.path.join(self.dir, "events.log"), "a", encoding="utf-8")
        self.lat = LatencyTracker()
        print(f"[日志] 本次运行目录：{self.dir}")

    def frame(self, fid, det, state, pv, rng, mode, focus_dac=None, focus_moved=False):
        self._log.write(json.dumps({
            "frame_id": fid, "ts": round(time.time(), 3),
            "visible": det.visible, "bbox": det.bbox_xyxy,
            "du_px": round(det.du_px, 1), "dv_px": round(det.dv_px, 1),
            "state": state, "pitch_vel": round(pv, 2),
            "dist_m": rng.distance_m if rng.valid else None,
            "dist_valid": rng.valid, "mode": mode,
            "focus_dac": focus_dac, "focus_moved": focus_moved,
        }, ensure_ascii=False) + "\n")

    def event(self, text):
        line = f"{time.strftime('%H:%M:%S')} {text}"
        self._events.write(line + "\n")
        self._events.flush()
        print(f"[事件] {line}")

    def finish(self, summary):
        summary["latency_ms"] = self.lat.report()
        summary["ended_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        with open(os.path.join(self.dir, "summary.json"), "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        self._log.close()
        self._events.close()
        print(f"[日志] 摘要已写入 {self.dir}/summary.json")
