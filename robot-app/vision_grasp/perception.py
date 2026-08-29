#!/usr/bin/env python3
"""
感知层：把一帧图像变成「可抓取目标的像素区域」。

两种后端（由 config.yaml 的 perception.backend 选择）：
- HttpPerception : 调用集成文档 Phase 1 约定的多模态模型服务
                   POST {image_base64, prompt} -> [{"label","bbox","score"}, ...]
- MockPerception : 返回画面中央固定包围框，用于无模型时验证链路。
"""
import base64
import json
import urllib.request

import cv2


class PerceptionError(RuntimeError):
    """感知服务相关错误：超时 / 返回格式错误 / 无有效检测。"""


class Detection:
    """单个检测结果：bbox 为 [x1, y1, x2, y2] 像素坐标。"""

    def __init__(self, label, bbox, score):
        self.label = label
        self.bbox = [float(v) for v in bbox]
        self.score = float(score)

    @property
    def center(self):
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

    def __repr__(self):
        x1, y1, x2, y2 = self.bbox
        return f"Detection({self.label}, score={self.score:.2f}, bbox=[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}])"


class HttpPerception:
    """多模态模型服务客户端（模型建议跑在 Windows 侧 GPU，WSL 经 HTTP 调用）。"""

    def __init__(self, cfg_p):
        self.url = cfg_p["url"]
        self.prompt = cfg_p.get("prompt", "找出可抓取物体")
        self.timeout = float(cfg_p.get("timeout", 8.0))

    def detect(self, frame):
        ok, buf = cv2.imencode(".jpg", frame)
        if not ok:
            raise PerceptionError("帧编码 jpg 失败")
        body = json.dumps({
            "image_base64": base64.b64encode(buf.tobytes()).decode("ascii"),
            "prompt": self.prompt,
        }).encode("utf-8")
        req = urllib.request.Request(
            self.url, data=body, headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                payload = json.loads(resp.read().decode("utf-8"))
        except Exception as e:
            raise PerceptionError(
                f"模型服务调用失败（{self.url}）: {e}。请确认服务已启动，"
                "或把 perception.backend 改为 mock 先验证机械链路。"
            )
        # 兼容 {"detections": [...]} 与裸列表两种返回
        items = payload.get("detections") if isinstance(payload, dict) else payload
        if not isinstance(items, list):
            raise PerceptionError(f"模型服务返回格式异常: {str(payload)[:200]}")
        dets = []
        for it in items:
            try:
                dets.append(Detection(it["label"], it["bbox"], it.get("score", 0.0)))
            except (KeyError, TypeError, ValueError):
                continue
        return dets


class MockPerception:
    """无模型替身：固定返回画面中央区域，保证坐标链路可被验证。"""

    def __init__(self, cfg_p):
        pass

    def detect(self, frame):
        h, w = frame.shape[:2]
        return [Detection("mock_object", [w * 0.45, h * 0.45, w * 0.55, h * 0.55], 0.99)]


def create_perception(cfg):
    cfg_p = cfg["perception"]
    backend = cfg_p.get("backend", "mock")
    if backend == "http":
        return HttpPerception(cfg_p)
    if backend == "mock":
        return MockPerception(cfg_p)
    raise PerceptionError(f"未知 perception.backend: {backend}（可选 http / mock）")


def pick_best(dets, min_score=0.6):
    """按置信度选出最佳检测；无有效结果返回 None。"""
    valid = [d for d in dets if d.score >= min_score]
    if not valid:
        return None
    return max(valid, key=lambda d: d.score)
