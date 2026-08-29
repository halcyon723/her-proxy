# -*- coding: utf-8 -*-
"""发送端：运行在跑多模态识别的电脑上。

把识别处理后的结果以 JSON 指令的形式发给接收端。
协议：每条指令一行，格式为 UTF-8 编码的 JSON。

用法：
    # 发一条指令
    python sender.py --host <接收端IP> --action stop
    python sender.py --host 192.168.1.50 --action move_to --payload "{\"x\":0.3,\"y\":0.1,\"z\":0.2}"

    # 演示模式：连续发送 3 条示例指令，用于验证链路
    python sender.py --host <接收端IP> --demo
"""
import argparse
import json
import socket
import time


def send_once(host, port, cmd):
    """发送一条指令并等待接收端回复，返回回复文本。"""
    with socket.create_connection((host, port), timeout=5) as s:
        s.sendall((json.dumps(cmd, ensure_ascii=False) + "\n").encode("utf-8"))
        return s.recv(1024).decode("utf-8").strip()


def main():
    parser = argparse.ArgumentParser(description="远程指令发送端")
    parser.add_argument("--host", required=True, help="接收端电脑的局域网 IP")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--action", help="指令动作，如 move_to / grasp / release / stop")
    parser.add_argument("--payload", default="{}", help='附加参数的 JSON 字符串，如 \'{"x":0.3}\'')
    parser.add_argument("--demo", action="store_true", help="演示模式：连续发送 3 条示例指令")
    args = parser.parse_args()

    if args.demo:
        # 模拟一段多模态识别输出的完整流程：移动→抓取→抬起→放下→归零
        demo_cmds = [
            {"action": "move_to", "payload": {"x": 0.24, "y": 0.0, "z": 0.10}, "note": "识别：移动到目标上方"},
            {"action": "grasp", "payload": {"object": "cup", "confidence": 0.93}, "note": "识别：抓取水杯"},
            {"action": "move_by", "payload": {"dx": 0.0, "dy": 0.0, "dz": 0.05}, "note": "抬起 5cm"},
            {"action": "release", "payload": {}, "note": "放下物体"},
            {"action": "stop", "payload": {}, "note": "回到安全零位"},
        ]
        for cmd in demo_cmds:
            reply = send_once(args.host, args.port, cmd)
            print(f"已发送 {cmd['action']} -> 接收端回复: {reply}")
            time.sleep(1)
        return

    if not args.action:
        parser.error("非演示模式必须提供 --action")
    cmd = {"action": args.action, "payload": json.loads(args.payload)}
    reply = send_once(args.host, args.port, cmd)
    print(f"已发送 {args.action} -> 接收端回复: {reply}")


if __name__ == "__main__":
    main()
