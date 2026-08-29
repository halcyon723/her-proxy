import time
import numpy as np
from Panthera_lib import Panthera

robot = None

try:
    print("正在初始化 Panthera...")
    robot = Panthera()

    print("读取当前状态...")
    current = np.asarray(robot.get_current_pos(), dtype=float)
    print("当前关节位置:", current)

    print()
    print("即将让第 1 个关节移动约 0.05 rad")
    print("确认机械臂周围安全后再按 Enter")
    input()

    target = current.copy()
    target[0] += 0.05

    velocity = [0.05] * robot.motor_count
    max_torque = [2.0] * robot.motor_count

    print("目标位置:", target)

    success = robot.Joint_Pos_Vel(
        pos=target,
        vel=velocity,
        max_tqu=max_torque,
        iswait=True,
        tolerance=0.02,
        timeout=5.0
    )

    print("执行结果:", success)

except KeyboardInterrupt:
    print("用户中断")

except Exception as e:
    print("测试失败:", repr(e))

finally:
    if robot is not None:
        try:
            robot.set_stop()
            print("已发送停止指令")
        except Exception as e:
            print("停止指令发送失败:", repr(e))
