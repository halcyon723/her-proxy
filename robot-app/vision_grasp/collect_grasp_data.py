#!/usr/bin/env python3
"""
抓取示教数据采集（collect_grasp_data.py）

用途：重力补偿模式下自由拖动机械臂演示抓取，同步记录相机帧 + 关节状态 + 末端位姿，
产出与 train_grasp_policy.py 对齐的数据集。

用法（WSL, conda panthera）：
    python robot-app/vision_grasp/collect_grasp_data.py [--config config.yaml] [--name cup]
流程：
    1. 启动后机械臂进入零刚度重力补偿，可自由拖动；夹爪可拖动开合；
    2. 按 record.rate_hz 采样：关节角/角速度、夹爪、末端位姿（FK）、图像存盘；
    3. 演示完成按 Ctrl+C 结束，标注本次为 成功/失败/丢弃；
    4. 重复多条演示，训练默认只使用标注为成功的片段。

输出目录（默认 robot-app/data/grasp_demos/）：
    episode_<时间戳>_<name>/
        meta.json     标注与采集元信息
        states.jsonl  每行一帧：{t, q, dq, gripper_q, gripper_dq, eef_pos, eef_rot, frame}
        frames/       000001.jpg ...（按 record.frame_width 等比缩放）

安全：拖动模式力矩限幅用 robot.drag_torque_limits；本脚本不做位置规划，
请在桌面安全区域内拖动，避免快速甩臂（相机腕装后末端质心改变）。
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from common import (GraspAbort, cfg_path, create_robot, load_config, stop_robot,
                    warmup_state)
from x4_camera import CameraError, create_camera


class EpisodeRecorder:
    """单条示教片段的落盘器：状态流（jsonl）+ 图像帧（jpg）。"""

    def __init__(self, out_dir, frame_width, camera_source, note=""):
        self.dir = Path(out_dir)
        self.frames_dir = self.dir / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.state_fd = open(self.dir / "states.jsonl", "w", encoding="utf-8")
        self.frame_width = int(frame_width)
        self.t0 = time.time()
        self.count = 0
        self.meta = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "camera_source": camera_source,
            "note": note,
            "label": "unlabeled",
        }

    def log(self, q, dq, gripper_q, gripper_dq, eef_pos, eef_rot, frame):
        self.count += 1
        fname = f"{self.count:06d}.jpg"
        h, w = frame.shape[:2]
        if w != self.frame_width:
            frame = cv2.resize(frame, (self.frame_width,
                                       int(h * self.frame_width / w)))
        cv2.imwrite(str(self.frames_dir / fname), frame)
        row = {
            "t": round(time.time() - self.t0, 4),
            "q": [round(float(v), 5) for v in q],
            "dq": [round(float(v), 5) for v in dq],
            "gripper_q": round(float(gripper_q), 5),
            "gripper_dq": round(float(gripper_dq), 5),
            "eef_pos": [round(float(v), 5) for v in eef_pos],
            "eef_rot": [round(float(v), 5) for v in np.asarray(eef_rot).ravel()],
            "frame": fname,
        }
        self.state_fd.write(json.dumps(row, ensure_ascii=False) + "\n")
        self.state_fd.flush()  # 立即落盘，Ctrl+C 不丢数据

    def finish(self, label):
        self.state_fd.close()
        self.meta["label"] = label
        self.meta["num_frames"] = self.count
        self.meta["duration_s"] = round(time.time() - self.t0, 2)
        with open(self.dir / "meta.json", "w", encoding="utf-8") as f:
            json.dump(self.meta, f, ensure_ascii=False, indent=2)
        print(f"[record] 已保存 {self.count} 帧 → {self.dir}（label={label}）")


def ask_label():
    while True:
        ans = input("本次示教结果标注 [s=成功 / f=失败 / d=丢弃]: ").strip().lower()
        if ans in ("s", "f", "d"):
            return {"s": "success", "f": "fail", "d": "discard"}[ans]
        print("请输入 s / f / d")


def main():
    ap = argparse.ArgumentParser(description="抓取示教数据采集（重力补偿拖动）")
    ap.add_argument("--config", default=None, help="配置文件路径（默认同目录 config.yaml）")
    ap.add_argument("--name", default="", help="片段名后缀，如 cup / bottle")
    args = ap.parse_args()

    cfg, cfg_dir = load_config(args.config)
    robot_cfg = cfg["robot"]
    rec_cfg = cfg["record"]

    robot = create_robot(cfg, cfg_dir)
    warmup_state(robot)

    camera = create_camera(cfg, cfg_dir)
    camera.open()

    # 重力补偿参数（与 5_record_trajectory.py 一致）
    Fc = np.array([0.15, 0.12, 0.12, 0.12, 0.04, 0.04])
    Fv = np.array([0.05, 0.05, 0.05, 0.03, 0.02, 0.02])
    vel_threshold = 0.02
    tau_limit = np.array(robot_cfg["drag_torque_limits"])
    zero = [0.0] * robot.motor_count

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = cfg_path(cfg_dir, rec_cfg["output_dir"]) / f"episode_{stamp}_{args.name or 'unnamed'}"
    recorder = EpisodeRecorder(out_dir, rec_cfg["frame_width"],
                               cfg["camera"].get("source"), note=args.name)

    rate_hz = float(rec_cfg["rate_hz"])
    period = 1.0 / rate_hz
    next_log = time.time()

    print("=" * 60)
    print("拖动示教中：机械臂零刚度可自由拖动，按 rate 同步记录图像与状态")
    print("演示完一条抓取后按 Ctrl+C 结束并标注")
    print("=" * 60)

    try:
        while True:
            q = robot.get_current_pos()
            dq = robot.get_current_vel()
            # 重力 + 摩擦补偿，力矩限幅，零刚度零阻尼（可自由拖动）
            tau = np.array(robot.get_Gravity()) + robot.get_friction_compensation(
                dq, Fc, Fv, vel_threshold)
            tau = np.clip(tau, -tau_limit, tau_limit)
            robot.pos_vel_tqe_kp_kd(zero, zero, tau, zero, zero)
            robot.gripper_control_MIT(0.0, 0.0, 0.0, 0.0, 0.0)

            now = time.time()
            if now >= next_log:
                next_log += period
                fk = robot.forward_kinematics(q)
                frame = camera.capture_frame()   # 掉线时显式抛错，不静默继续
                recorder.log(
                    q, dq,
                    robot.get_current_pos_gripper(),
                    robot.get_current_vel_gripper(),
                    fk["position"], fk["rotation"], frame,
                )
                print(f"\r已记录 {recorder.count} 帧", end="", flush=True)
            time.sleep(0.001)

    except CameraError as e:
        print(f"\n[相机中止] {e}")
        label = "discard"
    except KeyboardInterrupt:
        print("\n")
        label = ask_label()
    finally:
        recorder.finish(label)
        camera.close()
        stop_robot(robot)
        print("[record] 程序停止，电机已停机。掉电时请扶住机械臂。")


if __name__ == "__main__":
    try:
        main()
    except GraspAbort as e:
        print(f"[中止] {e}")
