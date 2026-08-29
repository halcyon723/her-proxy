#!/usr/bin/env python3
"""仿真示教记录器：订阅数字孪生 Socket.IO 的 robot_state，把示教动作录成 episode 数据。

数据流：网页控制仿真臂 → 后端广播 robot_state → 本脚本 20Hz 落盘。
每个 episode 输出一个目录：states.jsonl（每帧 t/q/dq/gripper/mode）+ meta.json（标注信息）。

用法：
    # 自测（不写文件，验证连通并统计广播频率）
    python sim_teach_recorder.py selftest

    # 录制一次示教：先倒计时 5 秒再开始录，录 40 秒，标注成功
    python sim_teach_recorder.py record --seconds 40 --delay 5 --label success --name ep001
"""
import argparse
import json
import time
from pathlib import Path

import socketio

SIO_URL = "http://localhost:5000"
OUT_ROOT = Path(__file__).resolve().parent.parent / "data" / "sim_demos"


class Recorder:
    def __init__(self):
        self.sio = socketio.Client()
        self.frames = []
        self.first_t = None
        self.last_t = None

        @self.sio.on("robot_state")
        def on_state(data):
            t = data.get("timestamp") or time.time()
            if self.first_t is None:
                self.first_t = t
            self.last_t = t
            self.frames.append({
                "t": round(t - self.first_t, 4),
                "q": [round(v, 5) for v in data.get("positions", [])],
                "dq": [round(v, 5) for v in data.get("velocities", [])],
                "gripper": round(data.get("gripper_position", 0.0) or 0.0, 5),
                "mode": data.get("control_mode", ""),
            })

    def connect(self):
        self.sio.connect(SIO_URL, wait_timeout=10)
        print(f"[recorder] 已连接 {SIO_URL}")

    def record_for(self, seconds, delay=0.0):
        if delay > 0:
            for i in range(int(delay), 0, -1):
                print(f"[recorder] {i}...", flush=True)
                time.sleep(1)
        print("[recorder] ● 开始录制", flush=True)
        t0 = time.time()
        self.frames.clear()
        self.first_t = None
        while time.time() - t0 < seconds:
            time.sleep(0.1)
        print(f"[recorder] ■ 录制结束，共 {len(self.frames)} 帧", flush=True)

    def stats(self):
        if not self.frames:
            return None
        dur = self.frames[-1]["t"]
        rate = len(self.frames) / dur if dur > 0 else 0
        q = [f["q"] for f in self.frames]
        rng = [round(max(col) - min(col), 3) for col in zip(*q)]
        grippers = [f["gripper"] for f in self.frames]
        return {
            "frames": len(self.frames),
            "duration_s": round(dur, 2),
            "avg_rate_hz": round(rate, 1),
            "joint_travel_rad": rng,
            "gripper_range": [round(min(grippers), 3), round(max(grippers), 3)],
        }

    def save(self, name, label):
        out_dir = OUT_ROOT / name
        out_dir.mkdir(parents=True, exist_ok=True)
        with open(out_dir / "states.jsonl", "w", encoding="utf-8") as f:
            for frame in self.frames:
                f.write(json.dumps(frame, ensure_ascii=False) + "\n")
        meta = {
            "name": name,
            "label": label,
            "source": "digital_twin_demo",
            "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            **(self.stats() or {}),
        }
        with open(out_dir / "meta.json", "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        print(f"[recorder] 已保存 {out_dir}")
        return meta


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("selftest", help="验证连通与广播频率，不写文件")

    rec = sub.add_parser("record")
    rec.add_argument("--seconds", type=float, default=40)
    rec.add_argument("--delay", type=float, default=5)
    rec.add_argument("--label", choices=["success", "fail"], default="success")
    rec.add_argument("--name", default=None, help="episode 名，默认按时间自动生成")

    args = ap.parse_args()
    rec_ = Recorder()
    rec_.connect()
    try:
        if args.cmd == "selftest":
            print("[selftest] 监听 5 秒...")
            time.sleep(5)
            print("[selftest] 统计:", rec_.stats() or "未收到任何 robot_state（后端广播循环未运行？）")
        else:
            name = args.name or time.strftime("ep_%Y%m%d_%H%M%S")
            rec_.record_for(args.seconds, delay=args.delay)
            meta = rec_.save(name, args.label)
            print("[recorder] 统计:", json.dumps(meta, ensure_ascii=False))
    finally:
        rec_.sio.disconnect()


if __name__ == "__main__":
    main()
