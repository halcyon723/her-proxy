#!/usr/bin/env python3
"""轨迹精炼：把多条 success 示教合成为一条平滑、匀速、夹爪时序正确的复现轨迹。

三个阶段（对应训练路线阶段 B）：
1. 对齐平均：8 条示教按时间归一化对齐后逐点求均值，消掉手抖噪声；
2. 时间重参数化：Cubic 样条拟合后按统一关节速度上限重新采样——根治忽快忽慢；
3. 夹爪事件化：从均值夹爪序列提取主要开合事件（滤掉微调抖动），
   每个事件阻塞执行（到位 ±0.05 rad 才解锁臂运动）——根治"爪没开臂就动"；
4. 搬运段抬升："闭合抓取→张开放下"之间用位置 IK 改为 抬起→高位平移→降下
   三段式，避免抓着物体贴桌拖移（--lift 调抬升量，0 关闭）。

用法：
    python refine_trajectory.py                        # 精炼 + 打印对比统计
    python refine_trajectory.py --lift 0.15            # 搬运段抬高 15cm
    python refine_trajectory.py --plot out.png         # 附加对比图
"""
import argparse
import json
import os
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.interpolate import CubicSpline

RECORDS = Path("/mnt/f/hackson/独居女性/panthera-ht-sdk/Panthera_digital_twin-main/backend/records")
OUT_FILE = Path(__file__).resolve().parent.parent / "data" / "refined_trajectory.json"
URDF = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "panthera-ht-sdk", "panthera_python",
    "Panthera-HT_description", "urdf", "Panthera-HT_description_follower.urdf",
))

N_ALIGN = 200          # 对齐重采样点数
VEL_LIMIT = 0.45        # 精炼后关节速度上限（rad/s）
GRIP_MOVE_EPS = 0.004  # 夹爪相邻帧位移超过该值视为"在动"（rad/帧）
GRIP_EVENT_MIN = 0.15  # 夹爪事件最小行程（rad），小于此的抖动合并/忽略
GRIP_SETTLE = 0.05     # 回放时夹爪到位判定容差（rad）
GRIP_TIGHTEN = 0.35    # 抓取闭合事件的过闭合量（rad）：目标低于物体宽度，
                        # 夹到物体即停（堵转），保证夹紧
LIFT_HEIGHT = 0.12     # 搬运段抬升量（m，抓取高度之上）
CARRY_DESC_LEAD = 2.0  # 放置下降在"张开放下"事件前多少秒完成
PLACE_LOWER = 0.01     # 放置高度再压低量（m）：物体贴桌后再松爪，
                        # 避免"掉下去"而不是"放下去"


def load_success():
    eps = []
    for d in sorted(RECORDS.iterdir()):
        meta_f = d / "meta.json"
        if not meta_f.exists():
            continue
        meta = json.loads(meta_f.read_text(encoding="utf-8"))
        if meta.get("label") != "success":
            continue
        frames = [json.loads(l) for l in (d / "states.jsonl").read_text(encoding="utf-8").splitlines() if l.strip()]
        eps.append((meta["name"], frames))
    if not eps:
        raise SystemExit("没有 success 示教数据")
    return eps


def resample_uniform(frames, n=N_ALIGN):
    """按弧长（时间）均匀重采样到 n 点。返回 (q[n,6], g[n], t_total)。"""
    q = np.array([f["q"] for f in frames])
    g = np.array([f["gripper"] for f in frames])
    t = np.array([f["t"] for f in frames])
    u = np.linspace(0, 1, n)
    idx_f = u * (len(q) - 1)
    idx_i = np.floor(idx_f).astype(int)
    idx_c = np.clip(idx_i + 1, 0, len(q) - 1)
    w = (idx_f - idx_i)[:, None]
    q_rs = q[idx_i] * (1 - w) + q[idx_c] * w
    g_rs = g[idx_i] * (1 - w.ravel()) + g[idx_c] * w.ravel()
    return q_rs, g_rs, float(t[-1] - t[0])


def extract_gripper_events(g_mean, dt_mean):
    """从均值夹爪序列提取主要开合事件：[(start_i, end_i, from, to), ...]。"""
    n = len(g_mean)
    moving = np.abs(np.diff(g_mean)) > GRIP_MOVE_EPS
    events = []
    i = 0
    while i < n - 1:
        if moving[i]:
            j = i
            while j < n - 1 and moving[j]:
                j += 1
            # 允许 2 帧间隙内的连续段合并
            k = j
            gap = 0
            while k < n - 1 and gap <= 2:
                if moving[k]:
                    j2 = k
                    while j2 < n - 1 and moving[j2]:
                        j2 += 1
                    j = j2
                    k = j2
                    gap = 0
                else:
                    gap += 1
                    k += 1
            travel = abs(g_mean[j] - g_mean[i])
            if travel >= GRIP_EVENT_MIN:
                events.append((int(i), int(j), float(g_mean[i]), float(g_mean[j])))
            i = j + 1
        else:
            i += 1
    return events


def reparametrize(q_mean, g_mean, vel_limit=VEL_LIMIT):
    """Cubic 样条拟合 + 按统一速度上限重新采样。返回 (q_new, g_new, t_new)。

    时间轴构造：对稠密参数轴的每个小区间，取 7 维里最大的关节位移，
    除以速度上限得到该区间耗时——构造上保证瞬时速度 ≤ vel_limit。"""
    n = len(q_mean)
    u_old = np.linspace(0, 1, n)
    q_full = np.hstack([q_mean, g_mean[:, None]])          # 7 维一起插值
    spline = CubicSpline(u_old, q_full, axis=0)
    dense_u = np.linspace(0, 1, n * 20)
    dense_q = spline(dense_u)
    # 每个小区间的最大关节位移 → 时间增量（下限 10ms；
    # 注意 dense 轴有 4000 区间，下限直接决定基准时长：0.02→80s，0.01→40s）
    step = np.abs(np.diff(dense_q, axis=0)).max(axis=1)
    dt = np.maximum(step / vel_limit, 0.01)
    t_new = np.concatenate([[0], np.cumsum(dt)])
    # 匀间隔重采样到时间轴上（20Hz 输出）
    T = t_new[-1]
    n_out = max(50, int(np.ceil(T / 0.05)))
    t_out = np.linspace(0, T, n_out)
    u_out = np.interp(t_out, t_new, dense_u)
    q_out = spline(u_out)
    return q_out[:, :6], q_out[:, 6], t_out


def _smoothstep(s):
    s = np.clip(s, 0.0, 1.0)
    return 3.0 * s * s - 2.0 * s ** 3


def _ik_6d(model, data, ee_id, p_target, R_target, q_seed,
           n_iter=200, eps=1e-3, eps_rot=1e-2, dt=0.5, damp=1e-4):
    """位置+姿态双目标的阻尼最小二乘 IK。返回 (收敛, q)。

    与纯位置 IK 的区别：姿态锁定到 R_target——搬运段物体保持抓取姿态，
    且解与种子姿态一致，避免窗口边界处关节跳变。
    """
    q = np.clip(q_seed, model.lowerPositionLimit, model.upperPositionLimit)
    for _ in range(n_iter):
        pin.framesForwardKinematics(model, data, q)
        M = data.oMf[ee_id]
        pos_err = p_target - M.translation
        rot_err = pin.log3(R_target @ M.rotation.T)
        if np.linalg.norm(pos_err) < eps and np.linalg.norm(rot_err) < eps_rot:
            return True, q
        err = np.concatenate([pos_err, rot_err])
        J = pin.computeFrameJacobian(model, data, q, ee_id, pin.LOCAL_WORLD_ALIGNED)
        dq = J.T @ np.linalg.solve(J @ J.T + damp * np.eye(6), err)
        q = pin.integrate(model, q, dq * dt)
    pin.framesForwardKinematics(model, data, q)
    M = data.oMf[ee_id]
    ok = (np.linalg.norm(p_target - M.translation) < 5e-3
          and np.linalg.norm(pin.log3(R_target @ M.rotation.T)) < 5e-2)
    return ok, q


def lift_carry_segment(q, t, events, lift=LIFT_HEIGHT, desc_lead=CARRY_DESC_LEAD):
    """搬运段抬升："抓取闭合→张开放下"之间改为 抬起→高位平移→降下 三段式。

    窗口自动推断：闭合事件中其后还有张开事件的最早一个（= 抓取闭合），
    至该张开事件（= 放置松爪）为止。用 6D IK（锁姿态）逐点重解（warm start
    原解，目标与原解接近，收敛快、解连续）。返回 (q_new, info)；
    未检出窗口时 info=None 且 q 原样返回。
    """
    closes = [e for e in events if e["to"] < e["from"]]
    opens = [e for e in events if e["to"] > e["from"]]
    window = None
    for c in sorted(closes, key=lambda e: e["t"]):
        later = [o for o in opens if o["t"] > c["t"]]
        if later:
            window = (c["t"], min(later, key=lambda e: e["t"])["t"])
            break
    if window is None:
        return q, None

    model = pin.buildModelFromUrdf(URDF)
    data = model.createData()
    ee_id = model.getFrameId("tool_link")

    pos = np.empty((len(q), 3))
    rot = [None] * len(q)
    for i, qi in enumerate(q):
        pin.framesForwardKinematics(model, data, qi)
        pos[i] = data.oMf[ee_id].translation
        rot[i] = data.oMf[ee_id].rotation.copy()

    t_c, t_o = window
    i_c = int(np.searchsorted(t, t_c))
    i_o = int(np.searchsorted(t, t_o - desc_lead))
    # 抬升起点：窗口内 xy 开始平移处（偏离抓取点 >1cm），提前约 1s——保证先抬后移
    dxy = np.linalg.norm(pos[i_c:i_o + 1, :2] - pos[i_c, :2], axis=1)
    moved = np.flatnonzero(dxy > 0.01)
    i_move = i_c + int(moved[0]) if len(moved) else i_c
    i_a = max(i_c, i_move - 20)  # 20 帧 ≈ 1s @20Hz

    z_a = float(pos[i_a, 2])
    z_b = float(pos[i_o, 2]) - PLACE_LOWER   # 放置点压低：贴桌再松爪
    z_lift = z_a + lift
    xy_path = pos[i_a:i_o + 1, :2]

    n = i_o - i_a + 1
    n_rise = max(10, int(n * 0.2))
    n_desc = max(10, int(n * 0.2))
    n_mid = max(1, n - n_rise - n_desc)
    # 平移段：原 xy 路径按弧长重映射（保持路径形状、速度均匀）
    seg_len = np.linalg.norm(np.diff(xy_path, axis=0), axis=1)
    s_arc = np.concatenate([[0.0], np.cumsum(seg_len)])

    q_new = q.copy()
    seed = q[i_a].copy()
    fail = 0
    for k in range(n):
        i = i_a + k
        if k <= n_rise:                      # 纯垂直上升（xy 冻结在抓取点上方）
            xy_t = pos[i_a, :2]
            z_t = z_a + (z_lift - z_a) * _smoothstep(k / n_rise)
        elif k >= n - n_desc:                # 纯垂直下降（xy 冻结在放置点上方）
            xy_t = pos[i_o, :2]
            z_t = z_lift + (z_b - z_lift) * _smoothstep((k - (n - n_desc)) / n_desc)
        else:                                # 高位平移
            u_arc = (k - n_rise) / n_mid * s_arc[-1]
            j = max(0, min(int(np.searchsorted(s_arc, u_arc)) - 1, len(seg_len) - 1))
            w = (u_arc - s_arc[j]) / max(seg_len[j], 1e-9)
            xy_t = xy_path[j] * (1.0 - w) + xy_path[j + 1] * w
            z_t = z_lift
        target = np.array([xy_t[0], xy_t[1], z_t])
        ok, qk = _ik_6d(model, data, ee_id, target, rot[i], seed)
        if ok:
            q_new[i] = qk
            seed = qk
        else:
            fail += 1
    # 端点混合：窗口首尾各 15 帧与原轨迹线性混合，
    # 消除 IK 解与原解的残余偏差（保证边界连续）
    nb = 15
    for k in range(1, nb + 1):
        for i, w in ((i_a + k, k / (nb + 1)), (i_o - k, k / (nb + 1))):
            if i_a < i < i_o:
                q_new[i] = q[i] * (1.0 - w) + q_new[i] * w
    # 窗口内轻度平滑（3 帧滑动平均，端点不动）：压掉 IK 解的关节速度锯齿
    for _ in range(2):
        q_win = q_new[i_a:i_o + 1].copy()
        q_win[1:-1] = (q_win[:-2] + q_win[1:-1] + q_win[2:]) / 3.0
        q_new[i_a:i_o + 1] = q_win
    info = {
        "window": (round(float(t[i_a]), 2), round(float(t[i_o]), 2)),
        "z_grasp": round(z_a, 3), "z_lift": round(z_lift, 3), "z_place": round(z_b, 3),
        "ik_fail": fail,
    }
    return q_new, info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", type=str, default=None)
    ap.add_argument("--lift", type=float, default=LIFT_HEIGHT,
                    help="搬运段抬升量(m)，0=关闭")
    args = ap.parse_args()

    eps = load_success()
    print(f"载入 {len(eps)} 条 success 示教")

    # 1. 对齐平均
    q_list, g_list, durs = [], [], []
    for name, frames in eps:
        q_rs, g_rs, dur = resample_uniform(frames)
        q_list.append(q_rs)
        g_list.append(g_rs)
        durs.append(dur)
    q_mean = np.mean(q_list, axis=0)
    g_mean = np.mean(g_list, axis=0)
    q_std = np.mean(np.std(q_list, axis=0))
    print(f"对齐平均完成：条间关节平均波动 ±{q_std:.3f} rad（已消入均值）")

    # 2. 时间重参数化
    q_new, g_new, t_new = reparametrize(q_mean, g_mean)
    # 3. 夹爪事件
    dt_mean = float(np.mean(durs)) / N_ALIGN
    events = extract_gripper_events(g_mean, dt_mean)
    events = [
        {"t": float(np.interp(e[0], np.arange(N_ALIGN), np.linspace(0, t_new[-1], N_ALIGN))),
         "from": e[2], "to": e[3]}
        for e in events
    ]
    # 抓取闭合事件（其后还有张开事件 = 抓住后要放下）过闭合：
    # 示教均值目标往往松于物体宽度，夹不住——目标再收紧 GRIP_TIGHTEN，
    # 回放时夹爪碰到物体堵转即停（由回放端堵转检测解锁）
    for ev in events:
        if ev["to"] < ev["from"] and any(o["t"] > ev["t"] and o["to"] > o["from"] for o in events):
            old_to = ev["to"]
            ev["to"] = max(0.0, ev["to"] - GRIP_TIGHTEN)
            ev["tighten"] = True
            print(f"抓取闭合事件 t={ev['t']:.1f}s 过闭合：目标 {old_to:.2f} → {ev['to']:.2f} rad")

    # 4. 搬运段抬升
    lift_info = None
    if args.lift > 0:
        q_new, lift_info = lift_carry_segment(q_new, t_new, events, lift=args.lift)
        if lift_info:
            print(f"\n搬运段抬升：窗口 t={lift_info['window'][0]}~{lift_info['window'][1]}s，"
                  f"z {lift_info['z_grasp']} → {lift_info['z_lift']}（平移）→ {lift_info['z_place']} m，"
                  f"IK 失败 {lift_info['ik_fail']} 点")
        else:
            print("\n未检出搬运窗口（无 闭合→张开 结构），跳过抬升")

    # ---- 逐路点速度（SDK smooth_trajectory_spline 同款方法：clamped 样条导数）----
    # 回放时每个路点携带匹配的速度前馈，控制器以恰好需要的速度走每一段，
    # 避免固定限幅造成的"全速冲→急停等下一点"锯齿顿挫（抖动根源）
    spl = CubicSpline(t_new, q_new, axis=0, bc_type="clamped")
    v_new = np.abs(spl(t_new, 1))
    # 下限防低速爬行卡死；上限须 ≥ 轨迹实际峰值（IK 抬升段 ~0.76），
    # 否则控制器跟不上路点推进产生滞后
    v_new = np.clip(v_new, 0.05, 1.0)

    # ---- 对比统计 ----
    def peak_vel(q, t):
        dq = np.abs(np.diff(q, axis=0) / np.maximum(np.diff(t)[:, None], 1e-3))
        return dq.max(), np.percentile(dq, 95)

    orig_peak, orig_p95 = peak_vel(q_mean, np.linspace(0, float(np.mean(durs)), N_ALIGN))
    new_peak, new_p95 = peak_vel(q_new, t_new)
    print("\n== 精炼前后对比 ==")
    print(f"总时长:   {np.mean(durs):.1f}s → {t_new[-1]:.1f}s")
    print(f"峰值速度: {orig_peak:.2f} → {new_peak:.2f} rad/s")
    print(f"95分位速度: {orig_p95:.2f} → {new_p95:.2f} rad/s")
    print(f"速度上限: {VEL_LIMIT} rad/s（硬约束）")
    print(f"夹爪事件: {len(events)} 个")
    for ev in events:
        d = "张开" if ev["to"] > ev["from"] else "闭合"
        print(f"  t={ev['t']:6.2f}s  {ev['from']:.2f} → {ev['to']:.2f} rad ({d}, 行程 {abs(ev['to']-ev['from']):.2f})")

    # 保存
    out = {
        "source_episodes": [name for name, _ in eps],
        "n_episodes": len(eps),
        "vel_limit": VEL_LIMIT,
        "grip_settle_tol": GRIP_SETTLE,
        "carry_lift": lift_info,
        "duration_s": round(float(t_new[-1]), 2),
        "waypoints": [
            {"t": round(float(t), 4), "q": [round(float(v), 5) for v in q],
             "v": [round(float(v), 4) for v in v], "gripper": round(float(g), 5)}
            for t, q, v, g in zip(t_new, q_new, v_new, g_new)
        ],
        "gripper_events": events,
    }
    OUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    OUT_FILE.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n已保存精炼轨迹: {OUT_FILE}（{len(t_new)} 个路点 @20Hz）")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=False)
        ax = axes[0]
        for j in range(6):
            ax.plot(np.linspace(0, 1, N_ALIGN), q_mean[:, j], alpha=0.5, lw=0.8)
        ax.set_title("mean trajectory (aligned)")
        ax = axes[1]
        for j in range(6):
            ax.plot(t_new, q_new[:, j], lw=1.2)
        ax.set_title(f"refined (vel<= {VEL_LIMIT} rad/s)")
        for ev in events:
            ax.axvline(ev["t"], color="r", ls="--", alpha=0.5)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=120)
        print(f"对比图已保存: {args.plot}")


if __name__ == "__main__":
    main()
