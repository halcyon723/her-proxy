#!/usr/bin/env python3
"""V1 视觉触发固定抓取：物体在固定点 ROI 内 → 触发既有精炼轨迹回放。

信息流：取帧（x4_sdk_bridge）→ 检测（color/mock/http）→ ROI 判定 → 触发回放。
机械臂侧零新代码：只调用数字孪生后端已验证的 /api/replay/start（速度、力矩、
安全限幅全部沿用回放实现），本脚本不产生任何其他机械臂指令。

安全设计：
- 默认 dry-run：只打印判定结果，不触发；加 --live 才真正触发回放；
- 触发速度默认 0.5（半速首跑），--speed 上限 1.0；
- 检测失败/无检测/不在 ROI → 明确报告并退出，绝不触发。

用法（WSL，panthera 环境）：
    python vision_trigger_grasp.py cal              # 校准模式：连续打印检测中心，辅助画 ROI
    python vision_trigger_grasp.py check            # 单次判定（dry-run）
    python vision_trigger_grasp.py check --live     # 单次判定并真的触发回放
    python vision_trigger_grasp.py stop             # 停止回放（急停用，随时可调）
    python vision_trigger_grasp.py watch            # 循环判定，物体进入 ROI 即触发（--live 才触发）

检测器：
    --detector color   HSV 色块检测（默认，需 --hsv-low/--hsv-high 适配物体颜色）
    --detector mock    画面中央固定框（无相机/无物体时验证链路）
    --detector http    调多模态模型服务（config.yaml perception.url）

ROI：--roi x1 y1 x2 y2（像素），默认画面中央 30% 方框；用 cal 模式实测后固化到命令行。
"""
import argparse
import json
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

import x4_sdk_bridge as bridge

REPLAY_BASE = "http://localhost:5000"   # 数字孪生后端（同 replay_demo.py）

# 固定点参考检测中心（2026-08-29 cal 实测）：未标定像素映射时的兜底参考。
# V2 起盒子不再要求对准固定点：检测偏差经 pixel_map.json 换算成基座系偏移，
# 由后端数值 IK 平移轨迹抓取段（真正的视觉引导，而非摆位依赖）。
REFERENCE_CENTER = [1848.0, 949.0]
DEFAULT_OFFSET_TOL = 150  # 未标定时的兜底门禁（px，≈5cm，与 ROI 同级）
MAX_COMP_M = 0.08  # 偏移补偿上限（米），超限拒绝触发（与 V2 敏感性网格 ±8cm 同口径）

PIXEL_MAP_FILE = Path(__file__).parent / "pixel_map.json"
SAMPLES_FILE = Path(__file__).parent / "sensitivity_samples.json"


# ---------------- 像素→基座映射（V2 偏移补偿标定）----------------
# 标定采样（用页面 📐采样 或 probe，以最后一个 p0 为基准的最近一轮）：
#   1. 盒子放固定点                          标签 p0
#   2. 沿机械臂正前方(+X)移 5cm / 正后方(-X)移 5cm   标签 x+5 / x-5（单边或双边均可）
#   3. 沿机械臂正左方(+Y)移 5cm / 正右方(-Y)移 5cm   标签 y+5 / y-5（单边或双边均可）
# 双边对称差分更稳（抵消盒子回位误差）；采样完成后 check/calmap 自动生成映射。

def _current_phase(data):
    """从后往前找最后一个 p0，取其后（含自身）为当前轮采样，避免早期废数据混入。"""
    for i in range(len(data) - 1, -1, -1):
        if str(data[i].get("label", "")).strip().lower().startswith("p0"):
            return data[i:]
    return data


def _pick(data, prefix, start=0):
    """phase 窗口内按标签前缀取最新一条采样记录。"""
    for i in range(len(data) - 1, start - 1, -1):
        if str(data[i].get("label", "")).strip().lower().startswith(prefix):
            return data[i]
    return None


def try_build_pixel_map(verbose=True):
    """当前轮标定点齐全时生成 pixel_map.json。返回 (pmap 或 None, 说明)。"""
    if not SAMPLES_FILE.exists():
        return None, "未找到 sensitivity_samples.json（先采样 p0 与偏移点）"
    data = json.loads(SAMPLES_FILE.read_text(encoding="utf-8"))
    phase = _current_phase(data)
    p0 = _pick(phase, "p0")
    if p0 is None:
        return None, "当前轮缺 p0 基准点"
    step = float(p0.get("step_m", 0.05))

    def axis_vec(sign):
        """某轴的像素位移向量：双边对称差分优先，单边回退到 p0 基准。"""
        rp = _pick(phase, sign + "+")
        rm = _pick(phase, sign + "-")
        if rp is not None and rm is not None:
            return [rp["mean"][0] - rm["mean"][0], rp["mean"][1] - rm["mean"][1]]
        if rp is not None:
            return [rp["mean"][0] - p0["mean"][0], rp["mean"][1] - p0["mean"][1]]
        if rm is not None:
            return [p0["mean"][0] - rm["mean"][0], p0["mean"][1] - rm["mean"][1]]
        return None

    u_x, u_y = axis_vec("x"), axis_vec("y")
    missing = [a for a, u in zip(("x±5", "y±5"), (u_x, u_y)) if u is None]
    if missing:
        return None, f"标定采样不全，缺: {', '.join(missing)}"
    nx, ny = float(np.hypot(*u_x)), float(np.hypot(*u_y))
    cosang = abs((u_x[0] * u_y[0] + u_x[1] * u_y[1]) / (nx * ny)) if nx * ny else 1.0
    # 对称差分跨 2×step，单边跨 step，折算到“每 step 的像素数”后同阈值校验
    px_per_step_x = nx / (2.0 if (_pick(phase, "x+") and _pick(phase, "x-")) else 1.0)
    px_per_step_y = ny / (2.0 if (_pick(phase, "y+") and _pick(phase, "y-")) else 1.0)
    if not (30 <= px_per_step_x <= 400 and 30 <= px_per_step_y <= 400):
        return None, (f"标定失败：{step*100:.0f}cm 位移像素异常"
                      f"（x:{px_per_step_x:.0f}px y:{px_per_step_y:.0f}px），"
                      "请确认每次精确移动且方向沿机械臂正前/正后方、正左/正右方")
    if cosang > 0.5:
        return None, "标定失败：x 与 y 方向位移在图像上近乎同向，请确认两组方向正交"
    pmap = {"p0": [float(v) for v in p0["mean"]],
            "u_x": [float(v) for v in u_x], "u_y": [float(v) for v in u_y],
            "step_m": step,
            "symmetric": [bool(_pick(phase, "x+") and _pick(phase, "x-")),
                          bool(_pick(phase, "y+") and _pick(phase, "y-"))],
            "built": time.strftime("%Y-%m-%d %H:%M:%S"),
            "samples": [r["label"] for r in phase]}
    PIXEL_MAP_FILE.write_text(json.dumps(pmap, ensure_ascii=False, indent=2),
                              encoding="utf-8")
    msg = (f"像素映射已生成：前方(X) {px_per_step_x:.0f}px/cm、左方(Y) {px_per_step_y:.0f}px/cm，"
           f"p0=({pmap['p0'][0]:.0f},{pmap['p0'][1]:.0f})")
    if verbose:
        print(f"[标定] {msg}")
    return pmap, msg


def load_pixel_map():
    if PIXEL_MAP_FILE.exists():
        try:
            return json.loads(PIXEL_MAP_FILE.read_text(encoding="utf-8"))
        except (ValueError, OSError):
            return None
    return None


def px_to_base_offset(dx, dy, pmap):
    """图像偏差(px) → 基座系 XY 偏移(米)。返回 (ox, oy) 或 None（病态映射）。
    对称差分采样的 u 向量对应 2×step，须除 2 归一。"""
    step = float(pmap.get("step_m", 0.05))
    sx, sy = pmap.get("symmetric", [False, False])
    U = np.array([[pmap["u_x"][0] / (2.0 if sx else 1.0),
                   pmap["u_y"][0] / (2.0 if sy else 1.0)],
                  [pmap["u_x"][1] / (2.0 if sx else 1.0),
                   pmap["u_y"][1] / (2.0 if sy else 1.0)]])
    try:
        b = step * np.linalg.inv(U) @ np.array([dx, dy])
    except np.linalg.LinAlgError:
        return None
    return float(b[0]), float(b[1])


# ---------------- 检测器 ----------------

def detect_color(frame, hsv_low, hsv_high, min_area=800):
    """HSV 色块检测，返回最大色块 bbox [x1,y1,x2,y2] 或 None。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_low), np.array(hsv_high))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in contours:
        if cv2.contourArea(c) < min_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if best is None or w * h > (best[2] - best[0]) * (best[3] - best[1]):
            best = [x, y, x + w, y + h]
    return best


def detect_mock(frame, _low, _high):
    h, w = frame.shape[:2]
    return [w * 0.45, h * 0.45, w * 0.55, h * 0.55]


def detect_box(frame, hsv_low, hsv_high, min_area=800):
    """色块最小外接矩形：返回 ((cx, cy), (w, h), angle°) 或 None。

    长方体盒子用外接矩形中心（几何对称中心）而非色块 bbox 中心，
    避免掩膜不完整时中心偏移导致抓空。
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_low), np.array(hsv_high))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, min_area
    for c in contours:
        a = cv2.contourArea(c)
        if a >= best_area:
            best, best_area = c, a
    if best is None:
        return None
    return cv2.minAreaRect(best)


def detect_http(frame, _low, _high):
    from perception import HttpPerception, PerceptionError
    raise PerceptionError("http 检测器请接入 config.yaml 后启用，当前先用 color/mock")


DETECTORS = {"color": detect_color, "mock": detect_mock, "http": detect_http}


# ---------------- 回放接口（仅 start/stop 两个动作，白名单式） ----------------

def replay_api(path, payload=None):
    url = f"{REPLAY_BASE}{path}"
    if payload is None:
        req = urllib.request.Request(url)
    else:
        req = urllib.request.Request(
            url, data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=10) as resp:
        return json.loads(resp.read().decode("utf-8"))


def trigger_replay(speed, offset_xy=None):
    """唯一会引发机械臂运动的入口。offset_xy=[x,y]（米）触发偏移补偿回放。"""
    payload = {"speed": speed}
    if offset_xy is not None:
        payload["offset_xy"] = [offset_xy["x"], offset_xy["y"]]
    return replay_api("/api/replay/start", payload)


def stop_replay():
    return replay_api("/api/replay/stop")


# ---------------- 判定逻辑 ----------------

def judge(det_bbox, roi, frame_shape):
    """返回 (是否触发, 消息)。判定规则：检测框中心落在 ROI 内。"""
    if det_bbox is None:
        return False, "未检测到物体"
    cx = (det_bbox[0] + det_bbox[2]) / 2
    cy = (det_bbox[1] + det_bbox[3]) / 2
    x1, y1, x2, y2 = roi
    inside = x1 <= cx <= x2 and y1 <= cy <= y2
    msg = (f"检测中心 ({cx:.0f},{cy:.0f}) 帧尺寸 {frame_shape[1]}x{frame_shape[0]} "
           f"ROI [{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}] -> "
           f"{'在 ROI 内，允许触发' if inside else '不在 ROI 内，拒绝触发'}")
    return inside, msg


def default_roi(frame):
    h, w = frame.shape[:2]
    return [w * 0.35, h * 0.35, w * 0.65, h * 0.65]


# ---------------- 主流程 ----------------

def one_check(args, roi, pmap=None):
    """单次判定。返回 (是否触发, 消息, 偏移补偿 dict 或 None)。"""
    frame = bridge.capture_frame()
    box = None
    if args.detector == "color":
        # 长方体检测：最小外接矩形 → 几何对称中心 + 尺寸 + 角度
        box = detect_box(frame, args.hsv_low, args.hsv_high)
        if box is None:
            return False, "未检测到物体（color: 无有效色块）", None
        (cx, cy), (w, h), angle = box
        det = [cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2]
    else:
        det = DETECTORS[args.detector](frame, args.hsv_low, args.hsv_high)
    ok, msg = judge(det, roi, frame.shape)
    if box is not None:
        # 偏差换算：有像素映射 → 基座系偏移（V2 补偿）；无映射 → 像素兜底门禁
        ref = pmap["p0"] if pmap else args.ref
        dx, dy = cx - ref[0], cy - ref[1]
        msg += f" | 盒子 {w:.0f}x{h:.0f}px {angle:.0f}° 偏差({dx:+.0f},{dy:+.0f})px"
        if ok and pmap is not None:
            off = px_to_base_offset(dx, dy, pmap)
            if off is None:
                return False, msg + " -> 像素映射病态，请删除 pixel_map.json 重新标定", None
            ox, oy = off
            dist = float(np.hypot(ox, oy))
            dirx = "前方" if ox >= 0 else "后方"
            diry = "左方" if oy >= 0 else "右方"
            msg += (f" -> 盒子偏离固定点 {dist * 100:.1f}cm"
                    f"（{dirx} {abs(ox) * 100:.1f}cm + {diry} {abs(oy) * 100:.1f}cm）")
            if dist > MAX_COMP_M:
                return False, msg + f"，超补偿上限 {MAX_COMP_M * 100:.0f}cm，拒绝触发", None
            msg += f"，将补偿抓取位置（±{MAX_COMP_M * 100:.0f}cm 内自动引导）"
            offset = {"x": round(ox, 4), "y": round(oy, 4),
                      "dx_px": round(dx, 1), "dy_px": round(dy, 1)}
            return True, msg, offset
        if ok and max(abs(dx), abs(dy)) > args.tol:
            return False, (msg + f" -> 偏离超 {args.tol:.0f}px 且未标定像素映射，无法补偿；"
                          "请采样 p0 / x+5 / y+5 三点启用偏移补偿"), None
    return ok, msg, None


def main():
    ap = argparse.ArgumentParser(description="V1 视觉触发固定抓取")
    ap.add_argument("cmd", choices=["cal", "check", "watch", "stop", "probe", "calmap"])
    ap.add_argument("--label", help="probe 模式：采样点标签（如 p0 / x+5cm）")
    ap.add_argument("--samples", type=int, default=15,
                    help="probe 模式采样帧数，默认 15")
    ap.add_argument("--live", action="store_true",
                    help="真正触发回放（不加则只 dry-run 打印）")
    ap.add_argument("--speed", type=float, default=0.5,
                    help="回放倍速，默认 0.5（半速），上限 1.0")
    ap.add_argument("--roi", type=float, nargs=4, metavar=("x1", "y1", "x2", "y2"),
                    help="判定 ROI 像素坐标，默认画面中央 30%% 方框")
    ap.add_argument("--detector", choices=list(DETECTORS), default="color")
    ap.add_argument("--ref", type=float, nargs=2, metavar=("x", "y"),
                    default=REFERENCE_CENTER,
                    help="固定点参考检测中心（像素），默认 1848 949")
    ap.add_argument("--tol", type=float, default=DEFAULT_OFFSET_TOL,
                    help="未标定映射时的兜底像素门禁，默认 150；标定后自动改用 ±8cm 物理补偿上限")
    ap.add_argument("--hsv-low", type=int, nargs=3, default=[0, 80, 80],
                    help="HSV 下限，默认 0 80 80")
    ap.add_argument("--hsv-high", type=int, nargs=3, default=[10, 255, 255],
                    help="HSV 上限，默认 10 255 255（红色系）")
    args = ap.parse_args()

    if args.speed > 1.0:
        print(f"[安全] 拒绝 speed={args.speed}：上限 1.0")
        return 2

    if args.cmd == "stop":
        print(stop_replay())
        return 0

    if args.cmd == "calmap":
        # 只重算像素映射（不开相机）：采样被编辑/删除后，删掉 pixel_map.json 后调此命令重建
        pm, m = try_build_pixel_map()
        print(m)
        return 0 if pm is not None else 1

    bridge.open()
    # 先取一帧确定 ROI（未指定时用默认中央框）
    frame = bridge.capture_frame()
    roi = args.roi if args.roi else default_roi(frame)
    print(f"[ROI] {roi}")

    if args.cmd == "cal":
        print("校准模式：把物体放固定点，观察检测中心；Ctrl+C 退出")
        try:
            while True:
                det = DETECTORS[args.detector](frame, args.hsv_low, args.hsv_high)
                if det is None:
                    print("\r未检测到物体（调整 --hsv-low/--hsv-high 或光照）    ", end="")
                else:
                    cx, cy = (det[0] + det[2]) / 2, (det[1] + det[3]) / 2
                    print(f"\r检测中心 ({cx:7.1f},{cy:7.1f}) "
                          f"建议 ROI: {cx-150:.0f} {cy-150:.0f} {cx+150:.0f} {cy+150:.0f}   ",
                          end="", flush=True)
                time.sleep(0.5)
                frame = bridge.capture_frame()
        except KeyboardInterrupt:
            print("\n校准结束")
        return 0

    if args.cmd == "probe":
        # 敏感性采样：多次取帧→检测中心均值±标准差，追加到 sensitivity_samples.json。
        # 用途：①同点重复采样量化基线噪声；②标定偏移点拟合像素→毫米映射（V2）。
        if not args.label:
            print("[probe] 需要 --label（如 --label p0 或 --label x+5cm）")
            return 2
        pts = []
        for _ in range(max(3, args.samples)):
            frame = bridge.capture_frame()
            if args.detector == "color":
                box = detect_box(frame, args.hsv_low, args.hsv_high)
                if box is not None:
                    pts.append([box[0][0], box[0][1]])  # 矩形对称中心，与 check 对准门禁同口径
            else:
                det = DETECTORS[args.detector](frame, args.hsv_low, args.hsv_high)
                if det is not None:
                    pts.append([(det[0] + det[2]) / 2, (det[1] + det[3]) / 2])
            time.sleep(0.2)
        if not pts:
            print("[probe] 未检测到物体，未记录")
            return 1
        arr = np.array(pts)
        mean, std = arr.mean(axis=0), arr.std(axis=0)
        print(f"[probe] {args.label}: 有效样本 {len(pts)}/{max(3, args.samples)}  "
              f"中心=({mean[0]:.1f},{mean[1]:.1f})  抖动σ=({std[0]:.1f},{std[1]:.1f})")
        rec = {"label": args.label, "mean": [round(x, 1) for x in mean],
               "std": [round(x, 1) for x in std], "n": len(pts),
               "frame": [frame.shape[1], frame.shape[0]],
               "time": time.strftime("%Y-%m-%d %H:%M:%S")}
        path = Path(__file__).parent / "sensitivity_samples.json"
        data = json.loads(path.read_text(encoding="utf-8")) if path.exists() else []
        data.append(rec)
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"[probe] 已追加到 {path.name}（累计 {len(data)} 条）")
        # 三点（p0/x+5/y+5）采齐后自动生成像素映射
        if not PIXEL_MAP_FILE.exists():
            pm, m = try_build_pixel_map(verbose=False)
            print(f"[标定] {m}")
        return 0

    if args.cmd in ("check", "watch"):
        # 像素映射：已有则加载；没有则尝试用采样记录自动生成
        pmap = load_pixel_map()
        if pmap is None:
            pmap, m = try_build_pixel_map()
            if pmap is None:
                print(f"[标定] {m}")

    if args.cmd == "check":
        ok, msg, offset = one_check(args, roi, pmap)
        print(msg)
        if not ok:
            print("[结果] 不触发")
            return 1
        if args.live:
            print(f"[触发] 调用回放 speed={args.speed}"
                  f"{' 偏移补偿=' + str([offset['x'], offset['y']]) if offset else ''}"
                  "（执行层限幅已由回放实现保障）")
            print(trigger_replay(args.speed, offset))
        else:
            print(f"[dry-run] 若加 --live 将以 speed={args.speed} 触发回放"
                  f"{'（含偏移补偿）' if offset else ''}")
        return 0

    if args.cmd == "watch":
        print("循环判定中（--live 才会触发）；Ctrl+C 退出")
        try:
            while True:
                ok, msg, offset = one_check(args, roi, pmap)
                print(f"\r{msg}    ", end="", flush=True)
                if ok:
                    print()
                    if args.live:
                        print(f"[触发] speed={args.speed}"
                              f"{' 偏移补偿=' + str([offset['x'], offset['y']]) if offset else ''}")
                        print(trigger_replay(args.speed, offset))
                        return 0
                    else:
                        print("[dry-run] 检测通过；加 --live 才会触发")
                        return 0
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n退出")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
