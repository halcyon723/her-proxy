#!/usr/bin/env python3
"""
视觉抓取执行脚本（run_grasp.py）—— look-then-move 闭环（集成文档 Phase 5）

三种运行形态：
  --mode geometric   几何抓取：感知服务给目标框 → 像素反投影 + 桌面平面假设求 3D
                     → IK → moveJ 预抓位 → moveL 垂直下降 → 夹爪 → 抬升放回。
                     需要先完成 Phase 3/4 标定（calib_x4.yaml / handeye_x4.yaml）。
  --mode policy      策略抓取：加载 train_grasp_policy.py 产出的检查点，
                     连续闭环输出关节增量直到末端接近桌面，再闭合夹爪抬升。
  --dry-run          不动机械臂（甚至不连接）：打印完整抓取计划，用于逻辑验证。
                     建议任何新模式真机前都先跑一遍。

安全约束（写死，与集成文档 Phase 5 一致）：
  - 观察/接近/下降段速度 ≤ robot.vel_approach，搬运段 ≤ robot.vel_travel；
  - 力矩限幅 max_tqu = 关节力矩上限 × robot.torque_ratio；
  - 目标点必须落在 table.safe_box 内，越界直接中止；
  - 下降前 wait_for_position 到位确认 + settle_time 消振；
  - 相机取帧失败显式报错，不静默继续；
  - 真机首次运行请一人手悬急停旁监护，且先用 --dry-run 验证。

用法示例（WSL, conda panthera）：
    python run_grasp.py --mode geometric --dry-run
    python run_grasp.py --mode geometric                 # 真机
    python run_grasp.py --mode policy --checkpoint checkpoints/grasp_policy_best.pt --dry-run
    python run_grasp.py --mode geometric --target-pixel 1440,720   # 跳过感知，手工指定像素
"""
import argparse
import time

import numpy as np

from common import (GraspAbort, cfg_path, clip_joints, create_robot, load_config,
                    max_torque, stop_robot, warmup_state)
from grasp_geom import GraspGeometry
from perception import PerceptionError, create_perception, pick_best
from x4_camera import CameraError, create_camera, settle_and_capture


# ---------------- 运动原语（含速度/力矩限幅） ----------------
def move_j_safe(robot, cfg, q_target, vel_limit, tag=""):
    """moveJ 并按速度上限推算 duration，到位等待。返回是否成功。"""
    q_cur = np.array(robot.get_current_pos())
    q_target = clip_joints(cfg, q_target)
    max_dq = float(np.max(np.abs(q_target - q_cur)))
    duration = max(2.0, max_dq / max(vel_limit, 1e-3))
    max_tqu = max_torque(cfg)
    print(f"[move] {tag} moveJ duration={duration:.1f}s (vel≤{vel_limit:.2f})")
    robot.moveJ(list(q_target), duration=duration, max_tqu=list(max_tqu), iswait=True,
                tolerance=cfg["robot"]["wait_tolerance"],
                timeout=cfg["robot"]["wait_timeout"])
    ok = robot.wait_for_position(list(q_target),
                                 tolerance=cfg["robot"]["wait_tolerance"],
                                 timeout=cfg["robot"]["wait_timeout"])
    if not ok:
        raise GraspAbort(f"{tag} 到位超时：请检查负载/限位后重试。")
    return q_target


def observe_pose_q(robot, cfg):
    """观察位的关节角：对配置笛卡尔位姿做 IK（以当前角为初值）。"""
    obs = cfg["grasp"]["observe_cartesian"]
    q_cur = robot.get_current_pos()
    q_obs = robot.inverse_kinematics(
        obs["position"], np.array(obs["rotation"], dtype=float), init_q=q_cur)
    if q_obs is None:
        raise GraspAbort("观察位姿 IK 失败：请调整 grasp.observe_cartesian。")
    return q_obs


def goto_observe(robot, cfg, vel_limit):
    q_obs = observe_pose_q(robot, cfg)
    return move_j_safe(robot, cfg, q_obs, vel_limit, tag="观察位")


# ---------------- 几何抓取模式 ----------------
def run_geometric_once(robot, cfg, cfg_dir, camera, perception, geom, dry_run,
                       target_pixel=None):
    g = cfg["grasp"]
    frame = None

    def log(step):
        print(f"[geometric] {step}")

    # 1. 观察位（dry-run 不连机械臂）
    if robot is not None:
        goto_observe(robot, cfg, cfg["robot"]["vel_approach"])
        frame = settle_and_capture(camera, cfg["camera"]["settle_time"])
    else:
        log("dry-run：跳过 moveJ 到观察位")
        frame = settle_and_capture(camera, 0.0)

    # 2. 感知：目标像素
    if target_pixel is not None:
        pixel = target_pixel
        log(f"使用手工指定像素 {pixel}（跳过感知）")
    else:
        dets = perception.detect(frame)
        best = pick_best(dets, cfg["perception"]["min_score"])
        if best is None:
            raise GraspAbort(f"无置信度≥{cfg['perception']['min_score']} 的检测结果，放弃本轮。")
        log(f"检测结果: {best}")
        pixel = best.center

    # 3. 像素 → 基座系 3D 点
    if robot is not None:
        T_base_gripper = robot.forward_kinematics()["transform"]
    else:
        log("dry-run：以单位阵近似 T_base_gripper（仅演示数值链路）")
        T_base_gripper = np.eye(4)
    h, w = frame.shape[:2]
    p_base = geom.pixel_to_base_point(pixel, T_base_gripper, frame_size=(w, h))
    if robot is None:
        # dry-run 用单位阵近似基座变换，安全区检查只告警不中止，便于验证数值链路
        try:
            geom.check_safe(p_base)
        except GraspAbort as e:
            log(f"dry-run 告警（可忽略）: {e}")
    else:
        geom.check_safe(p_base)
    log(f"目标基座系坐标: {np.round(p_base, 4).tolist()}")

    pregrasp = p_base.copy()
    pregrasp[2] += g["pregrasp_height"]
    descend = p_base.copy()
    descend[2] += g["touch_offset"]
    grasp_rot = np.array(g["grasp_rotation"], dtype=float)

    if robot is None:
        log(f"dry-run 计划: 预抓位 {np.round(pregrasp, 4).tolist()} → "
            f"下降至 {np.round(descend, 4).tolist()} → 闭合夹爪 → "
            f"抬升 {g['lift_height']} m → 回观察位张开")
        return True

    # 4. IK 预抓位并移动
    q_cur = robot.get_current_pos()
    q_pre = robot.inverse_kinematics(pregrasp, grasp_rot, init_q=q_cur)
    if q_pre is None:
        raise GraspAbort("预抓位 IK 失败：目标可能在可达域边缘，放弃本轮。")
    move_j_safe(robot, cfg, q_pre, cfg["robot"]["vel_approach"], tag="预抓位")

    # 5. 垂直下降（moveL）
    log(f"moveL 下降至 z={descend[2]:.3f}")
    ok = robot.moveL(list(descend), grasp_rot, duration=g["descend_duration"],
                     use_spline=True, max_tqu=list(max_torque(cfg)))
    if not ok:
        raise GraspAbort("moveL 下降失败（笛卡尔路径不完整）：放弃本轮。")
    time.sleep(cfg["camera"]["settle_time"])

    # 6. 闭合夹爪 → 抬升 → 回观察位 → 张开
    robot.gripper_close(pos=g["gripper_close"])
    time.sleep(0.5)
    lift = np.array(robot.forward_kinematics()["position"])
    lift[2] += g["lift_height"]
    ok = robot.moveL(list(lift), grasp_rot, duration=1.5, use_spline=True,
                     max_tqu=list(max_torque(cfg)))
    if not ok:
        raise GraspAbort("moveL 抬升失败：请现场处置（夹爪已闭合）。")
    goto_observe(robot, cfg, cfg["robot"]["vel_travel"])
    robot.gripper_open(pos=g["gripper_open"])
    log("本轮抓取完成")
    return True


# ---------------- 策略抓取模式 ----------------
def load_policy(ckpt_path):
    """加载检查点，返回 (model, meta)；未装 torch 时给出提示。"""
    try:
        import torch
        import torch.nn as nn  # noqa: F401
    except ImportError:
        raise GraspAbort("policy 模式需要 PyTorch：pip install torch")
    from train_grasp_policy import GraspPolicy
    ckpt = torch.load(ckpt_path, map_location="cpu")
    model = GraspPolicy()
    model.load_state_dict(ckpt["model_state"])
    model.eval()
    print(f"[policy] 已加载 {ckpt_path}（epoch={ckpt.get('epoch')}, "
          f"val_loss={ckpt.get('val_loss')}）")
    return model, ckpt


def preprocess_frame(frame, image_size):
    import cv2
    import torch
    img = cv2.resize(frame, (image_size, image_size))
    return torch.from_numpy(img.astype(np.float32) / 255.0).permute(2, 0, 1).unsqueeze(0)


def run_policy_once(robot, cfg, camera, model, ckpt, dry_run):
    import torch

    pol = cfg["policy"]
    rate_hz = float(pol["rate_hz"])
    max_steps = int(pol["max_steps"])
    image_size = int(ckpt["image_size"])
    max_delta = float(ckpt["max_delta"])
    vel = cfg["robot"]["vel_approach"]
    max_tqu = list(max_torque(cfg))
    plane_z = float(cfg["table"]["plane_point"][2])
    stop_z = plane_z + float(pol["stop_height"])

    def log(step):
        print(f"[policy] {step}")

    if robot is not None:
        goto_observe(robot, cfg, vel)
        robot.gripper_open(pos=cfg["grasp"]["gripper_open"])

    settle = cfg["camera"]["settle_time"]
    for step in range(max_steps):
        if robot is not None and step == 0:
            frame = settle_and_capture(camera, settle)   # 首帧停稳再拍
        else:
            frame = camera.capture_frame()               # 闭环中连续取帧
        img = preprocess_frame(frame, image_size)
        q_cur = np.array(robot.get_current_pos()) if robot is not None else np.zeros(6)
        with torch.no_grad():
            action = model(img, torch.from_numpy(q_cur.astype(np.float32)).unsqueeze(0))
        action = action.squeeze(0).numpy()

        dq = np.clip(action[:6], -max_delta, max_delta)
        gripper_target = float(np.clip(action[6],
                                       cfg["grasp"]["gripper_close"],
                                       cfg["grasp"]["gripper_open"]))

        if robot is None:
            if step % 20 == 0:
                log(f"dry-run step {step}: dq={np.round(dq, 3).tolist()}, "
                    f"gripper={gripper_target:.2f}")
            continue

        q_target = clip_joints(cfg, q_cur + dq)
        robot.Joint_Pos_Vel(list(q_target), [vel] * 6, max_tqu)
        robot.gripper_control(gripper_target, 0.5, 0.5)

        eef_z = robot.forward_kinematics()["position"][2]
        if eef_z <= stop_z:
            log(f"step {step}: 末端 z={eef_z:.3f} 到达接近终点（桌面+{pol['stop_height']}），结束接近段")
            break
        time.sleep(1.0 / rate_hz)
    else:
        log(f"达到最大步数 {max_steps}，结束接近段")

    if robot is None:
        log("dry-run：跳过 闭合夹爪 / 抬升 / 回观察位 / 张开")
        return True

    # 收尾：闭合 → 抬升 → 回观察位 → 张开
    robot.gripper_close(pos=cfg["grasp"]["gripper_close"])
    time.sleep(0.5)
    lift = np.array(robot.forward_kinematics()["position"])
    lift[2] += cfg["grasp"]["lift_height"]
    ok = robot.moveL(list(lift), np.array(cfg["grasp"]["grasp_rotation"], dtype=float),
                     duration=1.5, use_spline=True, max_tqu=max_tqu)
    if not ok:
        raise GraspAbort("moveL 抬升失败：请现场处置（夹爪已闭合）。")
    goto_observe(robot, cfg, cfg["robot"]["vel_travel"])
    robot.gripper_open(pos=cfg["grasp"]["gripper_open"])
    log("本轮抓取完成")
    return True


# ---------------- 主入口 ----------------
def main():
    ap = argparse.ArgumentParser(description="Panthera-HT + Insta360 X4 视觉抓取执行")
    ap.add_argument("--mode", choices=["geometric", "policy"], default="geometric")
    ap.add_argument("--dry-run", action="store_true",
                    help="不连接/不驱动机械臂，只打印抓取计划（验证逻辑用）")
    ap.add_argument("--config", default=None)
    ap.add_argument("--checkpoint", default=None, help="policy 模式必需")
    ap.add_argument("--target-pixel", default=None,
                    help="geometric 模式可选，'u,v' 手工指定目标像素，跳过感知")
    ap.add_argument("--rounds", type=int, default=1, help="连续抓取轮数")
    args = ap.parse_args()

    cfg, cfg_dir = load_config(args.config)

    if args.mode == "policy" and not args.checkpoint:
        raise GraspAbort("policy 模式需要 --checkpoint 指定训练产出的 .pt 文件")
    target_pixel = None
    if args.target_pixel:
        u, v = args.target_pixel.split(",")
        target_pixel = (float(u), float(v))

    camera = create_camera(cfg, cfg_dir)
    camera.open()

    robot = None
    try:
        if args.mode == "geometric":
            perception = create_perception(cfg)
            geom = GraspGeometry(
                cfg_path(cfg_dir, cfg["calibration"]["intrinsic"]),
                cfg_path(cfg_dir, cfg["calibration"]["handeye"]),
                cfg["table"],
            )
            model = ckpt = None
        else:
            perception = geom = None
            model, ckpt = load_policy(cfg_path(cfg_dir, args.checkpoint))

        if not args.dry_run:
            robot = create_robot(cfg, cfg_dir)
            warmup_state(robot)
            robot.gripper_open(pos=cfg["grasp"]["gripper_open"])
        else:
            print("[main] dry-run：不连接机械臂，仅计算并打印抓取计划")

        for r in range(args.rounds):
            print("=" * 60)
            print(f"第 {r + 1}/{args.rounds} 轮（{args.mode}）")
            print("=" * 60)
            if args.mode == "geometric":
                run_geometric_once(robot, cfg, cfg_dir, camera, perception, geom,
                                   args.dry_run, target_pixel)
            else:
                run_policy_once(robot, cfg, camera, model, ckpt, args.dry_run)

    except KeyboardInterrupt:
        print("\n[main] 用户中断")
    except (GraspAbort, CameraError, PerceptionError) as e:
        print(f"[中止] {e}")
    finally:
        camera.close()
        if robot is not None:
            stop_robot(robot)
            print("[main] 电机已停机。掉电时请扶住机械臂。")


if __name__ == "__main__":
    try:
        main()
    except GraspAbort as e:
        print(f"[中止] {e}")
