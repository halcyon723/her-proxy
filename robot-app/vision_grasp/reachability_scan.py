#!/usr/bin/env python3
"""桌面抓取工作空间可达性扫描（纯运动学计算，不碰串口、不动机械臂）。

抓取姿态说明：本臂腕部行程决定了末端最大向下俯仰约 47°（且仅在后收极位），
无法实现"夹爪竖直向下"。本扫描逐点求位置 IK 后，用多种播种姿态中的最大向下
俯仰角作为该点的抓取能力，并按 15°/30° 两档统计可达域，产出推荐安全盒。
输出：可达率、关节限位裕量、推荐安全包围盒（供 config.yaml 的 table.safe_box）。

用法：
    python reachability_scan.py [--nx 8] [--ny 8] [--zs 0.05,0.15,0.25]
"""
import argparse
import os

import numpy as np
import pinocchio as pin

URDF = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "panthera-ht-sdk", "panthera_python",
    "Panthera-HT_description", "urdf", "Panthera-HT_description_follower.urdf",
))
EE_LINK = "tool_link"

# 扫描区域（略宽于 config.yaml 占位 safe_box，用于收敛边界）
X_MIN, X_MAX = 0.15, 0.60
Y_MIN, Y_MAX = -0.30, 0.30

PITCH_TIERS = (np.deg2rad(15.0), np.deg2rad(30.0))  # 可达性分档：向下俯仰 ≥ 15° / ≥ 30°
LIMIT_MARGIN = 0.15           # 关节距限位裕量阈值 (rad)

# 播种姿态：覆盖高肘 / 低肘 / 深折肘 / 左右偏摆（位置 IK 对播种敏感，多播种提升覆盖）
SEEDS = [
    [0.0, 0.8, 0.8, 0.3, 0.0, 0.0],
    [0.0, 0.3, 0.6, 0.2, 0.0, 0.0],
    [0.0, 1.2, 2.0, -0.8, 0.0, 0.0],
    [0.0, 1.4, 2.4, -1.0, 0.5, 0.0],
    [0.0, 1.4, 2.4, -1.0, -0.5, 0.0],
    [0.5, 1.2, 2.0, -0.8, 0.0, 0.0],
    [-0.5, 1.2, 2.0, -0.8, 0.0, 0.0],
    [0.0, 0.2, 3.0, -1.4, 0.0, 0.0],
    [0.0, 0.5, 2.5, -1.2, 0.0, 0.0],
    [0.3, 0.4, 2.0, -1.0, 0.0, 0.0],
    [-0.3, 0.4, 2.0, -1.0, 0.0, 0.0],
]


def ik_position(model, data, ee_id, p_target, q_seed, n_iter=300, eps=1e-3, dt=0.5, damp=1e-6):
    """位置优先的阻尼最小二乘 IK。返回 (位置收敛, q)。"""
    q = np.clip(q_seed, model.lowerPositionLimit, model.upperPositionLimit)
    ok = False
    for _ in range(n_iter):
        pin.framesForwardKinematics(model, data, q)
        pos_err = p_target - data.oMf[ee_id].translation
        if np.linalg.norm(pos_err) < eps:
            ok = True
            break
        J = pin.computeFrameJacobian(model, data, q, ee_id, pin.LOCAL_WORLD_ALIGNED)[:3, :]
        dq = J.T @ np.linalg.solve(J @ J.T + damp * np.eye(3), pos_err)
        q = pin.integrate(model, q, dq * dt)
    return ok, q


def end_pitch(model, data, ee_id, q):
    """末端 z 轴向下俯仰角 (rad)，向下为正。"""
    pin.framesForwardKinematics(model, data, q)
    return float(np.arcsin(np.clip(-data.oMf[ee_id].rotation[2, 2], -1.0, 1.0)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--nx", type=int, default=8)
    ap.add_argument("--ny", type=int, default=8)
    ap.add_argument("--zs", type=str, default="0.05,0.15,0.25")
    args = ap.parse_args()
    zs = [float(z) for z in args.zs.split(",")]

    model = pin.buildModelFromUrdf(URDF)
    ee_id = model.getFrameId(EE_LINK)
    data = model.createData()
    q_lo, q_hi = model.lowerPositionLimit, model.upperPositionLimit

    results = []  # (x, y, z, max_pitch, margin)；max_pitch 为 nan 表示位置不可达
    for z in zs:
        for x in np.linspace(X_MIN, X_MAX, args.nx):
            for y in np.linspace(Y_MIN, Y_MAX, args.ny):
                p = np.array([x, y, z])
                best_pitch = float("nan")
                best_margin = float("nan")
                for seed in SEEDS:
                    conv, q = ik_position(model, data, ee_id, p, seed)
                    if not conv:
                        continue
                    pitch = end_pitch(model, data, ee_id, q)
                    margin = float(min(np.min(q - q_lo), np.min(q_hi - q)))
                    # 取向下俯仰最大且裕量可接受的解；若俯仰相同则取裕量大的
                    if np.isnan(best_pitch) or pitch > best_pitch + 1e-3 or (
                        abs(pitch - best_pitch) <= 1e-3 and margin > best_margin
                    ):
                        best_pitch, best_margin = pitch, margin
                results.append((x, y, z, best_pitch, best_margin))

    # ---- 汇总 ----
    total = len(results)
    n_any = sum(not np.isnan(r[3]) for r in results)
    print(f"网格点总数: {total}  位置可达（姿态不限）: {n_any} ({100.0 * n_any / total:.1f}%)")
    for tier in PITCH_TIERS:
        n = sum(not np.isnan(r[3]) and r[3] >= tier for r in results)
        print(f"可抓取（向下俯仰≥{np.degrees(tier):.0f}°）: {n} ({100.0 * n / total:.1f}%)")
    for z in zs:
        sub = [r for r in results if abs(r[2] - z) < 1e-6]
        parts = [f"位置 {sum(not np.isnan(r[3]) for r in sub)}/{len(sub)}"]
        parts += [f"俯仰≥{np.degrees(t):.0f}° {sum(not np.isnan(r[3]) and r[3] >= t for r in sub)}/{len(sub)}" for t in PITCH_TIERS]
        print(f"  z={z:.2f} m: " + "  ".join(parts))

    # ---- 推荐安全盒（按可抓取档） ----
    inset = 0.03  # 内缩 3 cm 作为执行裕量
    tier = PITCH_TIERS[0]
    pts = np.array([[r[0], r[1]] for r in results if not np.isnan(r[3]) and r[3] >= tier])
    if len(pts) == 0:
        print("\n无满足抓取俯仰的可达点，请检查扫描范围或放宽分档。")
        return
    rx = (float(pts[:, 0].min()) + inset, float(pts[:, 0].max()) - inset)
    ry = (float(pts[:, 1].min()) + inset, float(pts[:, 1].max()) - inset)
    good = np.array([[r[0], r[1]] for r in results if not np.isnan(r[3]) and r[3] >= tier and r[4] >= LIMIT_MARGIN])
    print(f"\n可抓取域（俯仰≥{np.degrees(tier):.0f}°，坐标范围）: x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}]")
    print(f"  推荐 safe_box（内缩3cm）: x[{rx[0]:.3f},{rx[1]:.3f}] y[{ry[0]:.3f},{ry[1]:.3f}]")
    if len(good):
        gx = (float(good[:, 0].min()) + inset, float(good[:, 0].max()) - inset)
        gy = (float(good[:, 1].min()) + inset, float(good[:, 1].max()) - inset)
        print(f"  保守 safe_box（再剔除贴限位点）: x[{gx[0]:.3f},{gx[1]:.3f}] y[{gy[0]:.3f},{gy[1]:.3f}]")


if __name__ == "__main__":
    main()
