#!/usr/bin/env python3
"""
相机抽象层：对上层统一提供「打开 → 停稳取帧 → 关闭」接口。

三种实现（由 config.yaml 的 camera.source 选择）：
- X4SdkCamera : Insta360 X4 官方 SDK 接入位（预留，两种接法任选其一）
- V4L2Camera  : usbipd 映射进 WSL 后，OpenCV 直读 /dev/videoN（集成文档 Phase 2 路线）
- MockCamera  : 从目录轮播 jpg，无相机时跑通全流程逻辑

约定：
- 帧格式统一为 BGR ndarray；
- 任何取帧失败必须抛出 CameraError 并给出重连提示，禁止静默返回（文档 VIS-005 要求）。
"""
import time
from pathlib import Path

import cv2
import numpy as np


class CameraError(RuntimeError):
    """相机相关错误：掉线 / 打开失败 / 读取失败 / SDK 未接入。"""


class BaseCamera:
    """相机接口：open() / capture_frame() / close()。"""

    def open(self):
        raise NotImplementedError

    def capture_frame(self):
        """返回一帧 BGR ndarray；失败抛 CameraError。"""
        raise NotImplementedError

    def close(self):
        pass

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False


class X4SdkCamera(BaseCamera):
    """Insta360 X4 官方 SDK 接入位（当前为占位实现）。

    接入方式二选一：
    1) 直接在本类中补全：`open()` 里初始化 SDK 会话，`capture_frame()` 里
       取一帧并转为 BGR ndarray 返回；
    2) 在本目录新建 `x4_sdk_bridge.py`，实现模块级函数
       `capture_frame() -> np.ndarray (BGR)`，本类会自动委托给它，
       无需改动抓取脚本。
    """

    def __init__(self, cfg_cam):
        self.cfg = cfg_cam
        self._bridge = None

    def open(self):
        try:
            import x4_sdk_bridge  # 用户自建的桥接模块（可选）
            if hasattr(x4_sdk_bridge, "capture_frame"):
                self._bridge = x4_sdk_bridge
                print("[camera] 已加载 x4_sdk_bridge，使用用户实现的 X4 取帧")
                if hasattr(x4_sdk_bridge, "open"):
                    x4_sdk_bridge.open()
                return
        except ImportError:
            pass
        raise CameraError(
            "X4 SDK 尚未接入：请在 x4_camera.py 的 X4SdkCamera 中补全实现，"
            "或在本目录提供 x4_sdk_bridge.py（实现 capture_frame()）。"
            "在此之前可把 config.yaml 的 camera.source 改为 v4l2 或 mock。"
        )

    def capture_frame(self):
        if self._bridge is None:
            raise CameraError("X4SdkCamera 未 open")
        frame = self._bridge.capture_frame()
        if frame is None:
            raise CameraError(
                "X4 取帧失败（可能过温停机或掉线）：请检查相机状态，"
                "必要时重新执行 scripts/bind-camera-to-wsl.ps1 或重启相机。"
            )
        return frame

    def close(self):
        if self._bridge is not None and hasattr(self._bridge, "close"):
            self._bridge.close()


class V4L2Camera(BaseCamera):
    """OpenCV + V4L2 直读（/dev/videoN），对应集成文档 Phase 2 路线。"""

    def __init__(self, cfg_cam):
        self.cfg = cfg_cam
        self.cap = None

    def open(self):
        idx = int(self.cfg.get("v4l2_index", 0))
        self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise CameraError(
                f"无法打开 /dev/video{idx}：请确认已执行 bind-camera-to-wsl.ps1，"
                "Windows 侧没有程序占用相机，且当前用户在 video 组"
                "（sudo usermod -aG video $USER 后重启 WSL）。"
            )
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg.get("width", 2880))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg.get("height", 1440))
        # 丢弃前若干帧，等自动曝光稳定
        for _ in range(int(self.cfg.get("warmup_frames", 10))):
            self.cap.read()
        w, h = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"[camera] V4L2 /dev/video{idx} 已打开，实际分辨率 {w}x{h}")

    def capture_frame(self):
        if self.cap is None or not self.cap.isOpened():
            raise CameraError("V4L2Camera 未 open")
        # 连读 3 帧取最新，降低缓冲区延迟
        frame = None
        for _ in range(3):
            ok, f = self.cap.read()
            if not ok:
                raise CameraError(
                    "cap.read() 失败：相机可能掉线或过温停机。请检查 USB 连接 / "
                    "重新执行 bind-camera-to-wsl.ps1 后再运行，脚本不会静默继续。"
                )
            frame = f
        return frame

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


class MockCamera(BaseCamera):
    """从目录按文件名排序轮播 jpg，用于无相机时验证全流程逻辑。"""

    def __init__(self, cfg_cam, mock_dir):
        self.cfg = cfg_cam
        self.dir = Path(mock_dir)
        self.files = []
        self.idx = 0

    def open(self):
        if not self.dir.exists():
            raise CameraError(
                f"mock 帧目录不存在: {self.dir}。请创建该目录并放入若干张场景 jpg，"
                "或把 config.yaml 的 camera.source 改为 v4l2 / x4_sdk。"
            )
        self.files = sorted(self.dir.glob("*.jpg")) + sorted(self.dir.glob("*.png"))
        if not self.files:
            raise CameraError(f"mock 帧目录为空: {self.dir}，请放入至少一张图片。")
        print(f"[camera] MockCamera：{len(self.files)} 张图片轮播，目录 {self.dir}")

    def capture_frame(self):
        frame = cv2.imread(str(self.files[self.idx % len(self.files)]))
        self.idx += 1
        if frame is None:
            raise CameraError(f"mock 图片读取失败: {self.files[(self.idx - 1) % len(self.files)]}")
        return frame


def create_camera(cfg, cfg_dir):
    """按配置创建相机实例。"""
    cfg_cam = cfg["camera"]
    source = cfg_cam.get("source", "mock")
    if source == "x4_sdk":
        return X4SdkCamera(cfg_cam)
    if source == "v4l2":
        return V4L2Camera(cfg_cam)
    if source == "mock":
        from common import cfg_path
        return MockCamera(cfg_cam, cfg_path(cfg_dir, cfg_cam.get("mock_dir", "mock_frames")))
    raise CameraError(f"未知 camera.source: {source}（可选 x4_sdk / v4l2 / mock）")


def settle_and_capture(camera, settle_time=0.2):
    """机械臂停稳后的标准取帧：消振等待 → 取帧（集成文档「停稳再拍」要求）。"""
    if settle_time > 0:
        time.sleep(settle_time)
    return camera.capture_frame()
