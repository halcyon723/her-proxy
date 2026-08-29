#!/usr/bin/env python3
"""恢复零位：从任意姿态（含超软限位的下坠姿态）安全回到零位。

安全设计：
- 先多次刷新状态（防 999），读当前关节角；
- 若有关节超软限位（拖动下坠可能越界），先 moveJ 到"裁剪回限位内"的过渡位；
- 再缓速 moveJ 回零位（速度 ≤ 0.25 rad/s 折算 duration，力矩限幅 30%）；
- 结束读回校验。
"""
import time

import numpy as np

from common import load_config, create_robot, max_torque, clip_joints, warmup_state


def refresh_state(robot, n=3):
    warmup_state(robot, rounds=n)
    time.sleep(0.3)


def main():
    cfg, cfg_dir = load_config()
    max_tqu = list(max_torque(cfg))

    robot = create_robot(cfg, cfg_dir)
    warmup_state(robot)
    time.sleep(0.3)
    q = list(robot.get_current_pos())[:6]
    print("当前关节角:", [round(v, 3) for v in q])

    zero = [0.0] * 6

    def move_to(target, vel_limit):
        refresh_state(robot, 3)
        cur = list(robot.get_current_pos())[:6]
        max_dq = max(abs(t - c) for t, c in zip(target, cur))
        duration = max(4.0, min(15.0, max_dq / vel_limit))
        print(f"moveJ -> {[round(v,3) for v in target]}  duration={duration:.1f}s")
        ok = robot.moveJ(target, duration=duration, max_tqu=max_tqu, iswait=True)
        print(f"执行状态: {ok}")
        time.sleep(0.5)
        return ok

    # 第一步：若有关节越界，先裁剪回限位内的过渡位
    q_clip = [float(v) for v in clip_joints(cfg, q)]
    if any(abs(a - b) > 1e-3 for a, b in zip(q, q_clip)):
        print("检测到越界关节，先过渡到限位内:", [round(v, 3) for v in q_clip])
        move_to(q_clip, vel_limit=0.25)

    # 第二步：缓速回零
    move_to(zero, vel_limit=0.25)

    # 校验
    refresh_state(robot, 3)
    q_end = list(robot.get_current_pos())[:6]
    err = max(abs(v) for v in q_end)
    print("回零后关节角:", [round(v, 3) for v in q_end])
    print(f"最大偏差: {err:.4f} rad -> {'OK' if err < 0.02 else 'NOT_SETTLED'}")


if __name__ == "__main__":
    main()
