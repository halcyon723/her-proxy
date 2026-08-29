#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂执行端：运行在 WSL 内，由 receiver.py --mode live 通过 wsl.exe 自动调用。

输入：argv[1] 为一条 JSON 指令（与发送端协议一致）。
输出：最后一行为结果 JSON，如 {"ok": true, "detail": "..."}；退出码 0 表示成功。

指令与动作映射：
    move_to  payload={"x","y","z"}          绝对坐标（米），逆解后 moveJ 到位
    move_by  payload={"dx","dy","dz"}       相对当前末端的小位移，moveL 直线运动
    grasp    （无参数）                      闭合夹爪
    release  （无参数）                      张开夹爪
    stop     （无参数）                      回到全零安全位

安全限制（如需调整，改下面的常量并同步告知团队）：
    - 工作空间包围盒：目标点超出范围直接拒绝，机械臂不动；
    - 单条 move_by 位移上限 20cm；
    - 每条动作固定执行 3 秒，宁慢勿快；
    - 逆运动学无解时拒绝执行。
"""
import json
import os
import sys

import numpy as np

# Panthera SDK 所在目录（含 Panthera_lib 与模型资源），切过去保证相对路径可用
SCRIPTS_DIR = "/mnt/f/hackson/独居女性/panthera-ht-sdk/panthera_python/scripts"
os.chdir(SCRIPTS_DIR)
sys.path.insert(0, SCRIPTS_DIR)

from Panthera_lib import Panthera  # noqa: E402

# ---------- 安全限制（单位：米） ----------
WORKSPACE = {"x": (0.05, 0.45), "y": (-0.30, 0.30), "z": (0.02, 0.50)}
MAX_STEP = 0.20      # 单条 move_by 指令的最大位移
DURATION = 3.0       # 每条动作的执行时长（秒）


def fail(msg):
    """打印失败结果并以非零退出码结束。"""
    print(json.dumps({"ok": False, "detail": msg}, ensure_ascii=False))
    sys.exit(1)


def check_limits(pos):
    for i, axis in enumerate("xyz"):
        lo, hi = WORKSPACE[axis]
        if not (lo <= pos[i] <= hi):
            fail(f"目标越界: {axis}={pos[i]:.3f} 超出允许范围 [{lo}, {hi}]")


def main():
    if len(sys.argv) < 2:
        fail("缺少指令参数")
    try:
        cmd = json.loads(sys.argv[1])
    except json.JSONDecodeError:
        fail("指令不是合法 JSON")
    action = cmd.get("action")
    payload = cmd.get("payload") or {}

    robot = Panthera()

    try:
        if action == "move_to":
            target = [float(payload["x"]), float(payload["y"]), float(payload["z"])]
            check_limits(target)
            q = robot.inverse_kinematics(target, None, robot.get_current_pos())
            if q is None:
                fail("逆运动学无解，机械臂未动")
            robot.moveJ(q, duration=DURATION, iswait=True)
            detail = f"已移动到 [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]"

        elif action == "move_by":
            delta = np.array([float(payload.get("dx", 0.0)),
                              float(payload.get("dy", 0.0)),
                              float(payload.get("dz", 0.0))])
            if np.linalg.norm(delta) > MAX_STEP:
                fail(f"单步位移 {np.linalg.norm(delta):.3f}m 超过上限 {MAX_STEP}m")
            fk = robot.forward_kinematics()
            target = np.array(fk["position"]) + delta
            check_limits(target)
            ok = robot.moveL(target_position=target, target_rotation=fk["rotation"],
                             duration=DURATION, use_spline=True)
            if not ok:
                fail("moveL 执行失败")
            detail = f"已相对移动 {np.linalg.norm(delta):.3f}m"

        elif action == "grasp":
            robot.gripper_close()
            detail = "夹爪已闭合"

        elif action == "release":
            robot.gripper_open()
            detail = "夹爪已张开"

        elif action == "stop":
            zero = [0.0] * robot.motor_count
            robot.moveJ(zero, duration=DURATION, iswait=True)
            detail = "已回到全零安全位"

        else:
            fail(f"未知 action: {action}")

    except (KeyError, TypeError, ValueError) as e:
        fail(f"payload 参数错误: {e}")

    print(json.dumps({"ok": True, "detail": detail}, ensure_ascii=False))


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        raise
    except Exception as e:  # 兜底：任何未预期异常都不允许静默成功
        fail(f"执行异常: {e}")
