#!/usr/bin/env python3
"""x4_sdk_bridge（WSL 侧，S2）：从 Windows 取帧服务取一帧，供 X4SdkCamera 委托。

对接 x4_frame_server.py（Windows，默认端口 8200）：
    GET http://<host>:8200/frame.jpg

契约（见 x4_camera.py 的 X4SdkCamera 注释）：
    capture_frame() -> np.ndarray (BGR)   失败抛 CameraError
    open() / close() 可选

宿主地址解析顺序：
    1. 环境变量 X4_FRAME_SERVER（如 192.168.30.246:8200 或 http://...）
    2. /etc/resolv.conf 的 nameserver（WSL NAT 网关即 Windows 宿主）
    3. localhost（Windows 侧镜像网络或本机运行时）
"""
import io
import json
import os
import re
import subprocess
import urllib.request

import cv2
import numpy as np


class CameraError(RuntimeError):
    """与 x4_camera.CameraError 同语义，避免循环导入，单独定义。"""


DEFAULT_PORT = 8200
_TIMEOUT = 3.0


def _default_gateway():
    """WSL NAT 模式下，默认网关即 Windows 宿主（新版 WSL 的 resolv.conf 是
    10.255.255.254 DNS 代理，不可用作宿主地址，故网关优先）。"""
    try:
        out = subprocess.run(
            ["ip", "route", "show", "default"],
            capture_output=True, text=True, timeout=2,
        ).stdout
        m = re.search(r"via\s+(\d+\.\d+\.\d+\.\d+)", out)
        if m:
            return m.group(1)
    except Exception:
        pass
    return None


def _base_url():
    env = os.environ.get("X4_FRAME_SERVER", "").strip()
    if env:
        if not env.startswith("http"):
            env = "http://" + env
        return env.rstrip("/")
    # WSL NAT：默认网关即宿主（用 io.open：本模块定义了同名 open() 探活函数，会遮蔽内建 open）
    gw = _default_gateway()
    if gw:
        return f"http://{gw}:{DEFAULT_PORT}"
    try:
        with io.open("/etc/resolv.conf", encoding="ascii", errors="ignore") as f:
            for line in f:
                if line.startswith("nameserver"):
                    ip = line.split()[1]
                    return f"http://{ip}:{DEFAULT_PORT}"
    except OSError:
        pass
    return f"http://localhost:{DEFAULT_PORT}"


def _get(path):
    url = _base_url() + path
    try:
        with urllib.request.urlopen(url, timeout=_TIMEOUT) as resp:
            return resp.status, resp.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()
    except Exception as e:
        raise CameraError(
            f"取帧服务不可达（{url}）: {e}。请确认 Windows 侧 x4_frame_server.py "
            "已启动、端口 8200 防火墙已放行，或设置环境变量 X4_FRAME_SERVER。"
        )


def open():
    """探活：确认取帧服务在线且有帧。"""
    status, body = _get("/status")
    info = {}
    try:
        info = json.loads(body)
    except (ValueError, TypeError):
        pass
    if status != 200 or not info.get("ok"):
        raise CameraError(f"取帧服务状态异常（HTTP {status}）：{str(body)[:200]}")
    print(f"[x4-bridge] 取帧服务在线：{json.dumps(info, ensure_ascii=False)}")


def capture_frame():
    """取最新一帧，返回 BGR ndarray；任何失败显式抛 CameraError。"""
    status, body = _get("/frame.jpg")
    if status != 200:
        raise CameraError(
            f"取帧失败（HTTP {status}）：相机可能掉线或服务未就绪，"
            "请检查 Windows 侧相机连接与服务状态。"
        )
    frame = cv2.imdecode(np.frombuffer(body, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise CameraError("JPEG 解码失败：帧数据损坏。")
    return frame


def close():
    pass  # 无状态连接，无需清理
