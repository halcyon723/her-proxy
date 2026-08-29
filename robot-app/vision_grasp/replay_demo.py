#!/usr/bin/env python3
"""演示脚本：一键回放学习到的抓取轨迹（refined_trajectory.json）。

封装数字孪生后端的回放接口，供命令行/外部系统调用：

    python replay_demo.py start              # 开始演示（默认 1.0 倍速）
    python replay_demo.py start --speed 0.5  # 半速演示（更稳妥）
    python replay_demo.py stop               # 停止，机械臂原地停住
    python replay_demo.py status             # 查询进度
    python replay_demo.py watch              # 持续监视直到结束（Ctrl+C 退出）
"""
import argparse
import json
import sys
import time
import urllib.request

BASE = "http://localhost:5000"
STAGE_MAP = {
    "move_to_start": "对齐起始位",
    "playing": "执行轨迹",
    "finishing": "收尾",
    "done": "完成",
}


def api(path, payload=None):
    url = f"{BASE}{path}"
    if payload is None:
        req = urllib.request.Request(url)
    else:
        req = urllib.request.Request(
            url, data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="POST",
        )
    with urllib.request.urlopen(req, timeout=10) as resp:
        return json.loads(resp.read().decode("utf-8"))


def main():
    ap = argparse.ArgumentParser(description="抓取轨迹演示（回放精炼轨迹）")
    ap.add_argument("cmd", choices=["start", "stop", "status", "watch"])
    ap.add_argument("--speed", type=float, default=1.0, help="回放倍速 0.1~2.0")
    args = ap.parse_args()

    if args.cmd == "start":
        print(api("/api/replay/start", {"speed": args.speed}))
    elif args.cmd == "stop":
        print(api("/api/replay/stop"))
    elif args.cmd == "status":
        print(json.dumps(api("/api/replay/status"), ensure_ascii=False, indent=2))
    elif args.cmd == "watch":
        try:
            while True:
                s = api("/api/replay/status")
                stage = str(s.get("stage", ""))
                if stage.startswith("gripper_event"):
                    stage = f"夹爪动作 {stage.split('_')[-1]}"
                else:
                    stage = STAGE_MAP.get(stage, stage)
                print(f"\r[{time.strftime('%H:%M:%S')}] "
                      f"{'● 演示中' if s['running'] else '○ 已结束'} "
                      f"进度 {int((s.get('progress') or 0) * 100):3d}% "
                      f"夹爪事件 {s.get('events_done', 0)}/4 "
                      f"{stage}          ", end="", flush=True)
                if not s.get("running"):
                    print()
                    return
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\n（继续后台执行；停止请运行: python replay_demo.py stop）")


if __name__ == "__main__":
    sys.exit(main())
