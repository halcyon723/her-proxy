#!/usr/bin/env python3
"""
vision_grasp 公共工具：配置加载、路径解析、SDK 导入、安全参数换算。

所有脚本统一从本目录的 config.yaml 读取参数；配置中的相对路径一律相对
配置文件所在目录解析，保证从任意工作目录启动脚本结果一致。
"""
import sys
from pathlib import Path

import numpy as np
import yaml

THIS_DIR = Path(__file__).resolve().parent          # robot-app/vision_grasp
WORKSPACE_ROOT = THIS_DIR.parents[1]                # 仓库根目录
DEFAULT_CONFIG = THIS_DIR / "config.yaml"


class GraspAbort(Exception):
    """预期内中止（安全区越界 / 标定缺失 / 相机掉线 / IK 失败等）。

    捕获后只打印明确原因与处置建议，不输出堆栈；与未知异常区分开。
    """


def load_config(path=None):
    """加载 yaml 配置，返回 (cfg_dict, cfg_dir)。"""
    cfg_file = Path(path) if path else DEFAULT_CONFIG
    if not cfg_file.exists():
        raise GraspAbort(f"配置文件不存在: {cfg_file}")
    with open(cfg_file, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    return cfg, cfg_file.resolve().parent


def cfg_path(cfg_dir, p):
    """把配置中的相对路径解析为绝对路径（相对配置文件目录）。"""
    p = Path(p)
    return p if p.is_absolute() else (cfg_dir / p).resolve()


def import_panthera():
    """把 SDK 脚本目录加入 sys.path 并返回 Panthera 类。

    业务代码位于 robot-app/，SDK 位于 panthera-ht-sdk/panthera_python/scripts/，
    此处按仓库相对结构定位，无需安装即可导入。
    """
    sdk_scripts = WORKSPACE_ROOT / "panthera-ht-sdk" / "panthera_python" / "scripts"
    if not sdk_scripts.exists():
        raise GraspAbort(f"未找到 SDK 脚本目录: {sdk_scripts}")
    if str(sdk_scripts) not in sys.path:
        sys.path.insert(0, str(sdk_scripts))
    from Panthera_lib import Panthera
    return Panthera


def create_robot(cfg, cfg_dir):
    """按配置创建真机 Panthera 实例（使用 Follower.yaml）。"""
    Panthera = import_panthera()
    robot_cfg = cfg_path(cfg_dir, cfg["robot"]["config_path"])
    if not robot_cfg.exists():
        raise GraspAbort(f"机械臂配置文件不存在: {robot_cfg}")
    print(f"[robot] 加载配置 {robot_cfg}")
    return Panthera(str(robot_cfg))


def max_torque(cfg):
    """执行段力矩限幅 = 关节力矩上限 × torque_ratio（返回长度 6 的 ndarray）。"""
    return np.array(cfg["robot"]["joint_torque_limits"]) * cfg["robot"]["torque_ratio"]


def clip_joints(cfg, q):
    """把关节角裁剪到配置限位内。"""
    lo = np.array(cfg["robot"]["joint_lower"])
    hi = np.array(cfg["robot"]["joint_upper"])
    return np.clip(np.asarray(q, dtype=float), lo, hi)


def warmup_state(robot, rounds=10):
    """记录/控制前先循环读取状态，避免未收到反馈时关节角为 999（同 5_record_trajectory）。"""
    import time
    for _ in range(rounds):
        robot.send_get_motor_state_cmd()
        time.sleep(0.1)


def stop_robot(robot):
    """安全停机入口，任何异常路径都应调用。"""
    try:
        robot.set_stop()
    except Exception as e:  # 停机失败也不能掩盖原始异常
        print(f"[robot] set_stop 失败: {e}")
