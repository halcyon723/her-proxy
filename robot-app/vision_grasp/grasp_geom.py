#!/usr/bin/env python3
"""
抓取几何：像素 → 相机系射线 → 桌面平面求交 → 基座系 3D 点（look-then-move 的核心）。

依赖两个标定产物（缺失时会给出明确的补做提示）：
- calibration/calib_x4.yaml   （Phase 3 内参）期望字段：
    model: fisheye | pinhole
    K: 3x3 内参
    D: 畸变系数（fisheye 为 4x1）
    image_size: [w, h]（标定时的分辨率）
- calibration/handeye_x4.yaml （Phase 4 手眼）期望字段：
    T_cam2gripper: 4x4，相机系点 → 夹爪/末端系点（即 p_g = T @ p_c）
    num_poses / residual: 记录信息

坐标链：
  pixel →(内参反投影)→ 相机系射线 →(与基座系桌面平面求交)→ 相机系 3D 点
        →(T_cam2gripper)→ 末端系 →(当前 FK 的 T_base_gripper)→ 基座系
注意：手眼矩阵方向以 calibrateHandEye 的 eye-in-hand 语义为准，实施时须按
集成文档 Phase 4 用触点验证（≤5 mm）确认，不得只凭推导。
"""
import numpy as np
import yaml

from common import GraspAbort


def _load_yaml(path, what):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        raise GraspAbort(
            f"{what}标定文件不存在: {path}。"
            "几何抓取模式需要先完成集成文档 Phase 3 / Phase 4 标定；"
            "若只想验证策略网络，可用 run_grasp.py --mode policy。"
        )


class GraspGeometry:
    def __init__(self, intrinsic_path, handeye_path, table_cfg):
        # ---- 内参 ----
        calib = _load_yaml(intrinsic_path, "内参")
        self.model = str(calib.get("model", "fisheye")).lower()
        self.K = np.array(calib["K"], dtype=float).reshape(3, 3)
        self.D = np.array(calib["D"], dtype=float).reshape(-1, 1)
        self.calib_size = tuple(int(v) for v in calib["image_size"])  # (w, h)

        # ---- 手眼 ----
        he = _load_yaml(handeye_path, "手眼")
        self.T_cam2gripper = np.array(he["T_cam2gripper"], dtype=float).reshape(4, 4)

        # ---- 桌面平面与安全区（基座系） ----
        self.plane_point = np.array(table_cfg["plane_point"], dtype=float)
        n = np.array(table_cfg["plane_normal"], dtype=float)
        self.plane_normal = n / np.linalg.norm(n)
        box = table_cfg["safe_box"]
        self.safe_min = np.array([box["x"][0], box["y"][0], box["z"][0]], dtype=float)
        self.safe_max = np.array([box["x"][1], box["y"][1], box["z"][1]], dtype=float)

    # ---------- 像素 → 相机系射线（已去畸变，单位方向） ----------
    def pixel_to_ray(self, pixel, frame_size=None):
        """pixel=(u,v) 为当前帧像素坐标；若当前帧分辨率与标定不一致需传 frame_size=(w,h) 做缩放。"""
        u, v = float(pixel[0]), float(pixel[1])
        if frame_size is not None and tuple(frame_size) != self.calib_size:
            u *= self.calib_size[0] / float(frame_size[0])
            v *= self.calib_size[1] / float(frame_size[1])
        pt = np.array([[[u, v]]], dtype=np.float64)
        if self.model == "fisheye":
            import cv2
            pts = cv2.fisheye.undistortPoints(pt, self.K, self.D, P=self.K)
        else:
            import cv2
            pts = cv2.undistortPoints(pt, self.K, self.D, P=self.K)
        up, vp = pts[0, 0]
        ray = np.linalg.solve(self.K, np.array([up, vp, 1.0]))
        return ray / np.linalg.norm(ray)

    # ---------- 像素 → 基座系 3D 点（桌面平面假设） ----------
    def pixel_to_base_point(self, pixel, T_base_gripper, frame_size=None):
        """返回基座系 3D 点；射线与平面近平行等退化情况抛 GraspAbort。"""
        T_base_gripper = np.array(T_base_gripper, dtype=float).reshape(4, 4)
        T_base_cam = T_base_gripper @ self.T_cam2gripper

        ray_cam = self.pixel_to_ray(pixel, frame_size)
        origin = T_base_cam[:3, 3]
        direction = T_base_cam[:3, :3] @ ray_cam

        denom = float(self.plane_normal @ direction)
        if abs(denom) < 1e-6:
            raise GraspAbort("视线与桌面平面近平行，无法求交：请调整观察位姿。")
        s = float(self.plane_normal @ (self.plane_point - origin)) / denom
        if s <= 0:
            raise GraspAbort("射线反向与平面求交（目标在相机背后）：请检查标定与观察位姿。")
        return origin + s * direction

    # ---------- 桌面安全区检查 ----------
    def check_safe(self, p_base):
        """目标点越出安全包围盒即抛 GraspAbort（文档 Phase 5 安全约束）。"""
        p = np.asarray(p_base, dtype=float)
        if not (np.all(p >= self.safe_min) and np.all(p <= self.safe_max)):
            raise GraspAbort(
                f"目标点 {np.round(p, 3).tolist()} 超出桌面安全区 "
                f"[{self.safe_min.tolist()} ~ {self.safe_max.tolist()}]，抓取中止。"
                "请检查检测框/标定/桌面平面参数，或调整 safe_box。"
            )
        return p
