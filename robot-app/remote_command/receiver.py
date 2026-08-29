# -*- coding: utf-8 -*-
"""接收端：运行在控制机械臂的电脑上。

在局域网内监听，接收另一台电脑发来的识别结果 / 控制指令。
协议：每条指令一行，格式为 UTF-8 编码的 JSON。

用法：
    python receiver.py [--port 9000]            # 模拟模式（默认）：只打印指令，不驱动真机
    python receiver.py --mode live              # 真机模式：指令转发给 WSL 内的机械臂执行端
"""
import argparse
import json
import socketserver
import subprocess

# 运行模式：main() 里根据 --mode 参数设置
MODE = "sim"

# ---------- 安全校验（白名单按需增删） ----------
ALLOWED_ACTIONS = {"move_to", "move_by", "grasp", "release", "stop"}

# ---------- WSL 机械臂执行桥（--mode live 时生效） ----------
WSL_DISTRO = "Ubuntu-24.04"
WSL_PYTHON = "/mnt/f/hackson/独居女性/panthera-ht-sdk/.venv-linux/bin/python"
WSL_EXECUTOR = "/mnt/f/hackson/独居女性/robot-app/remote_command/arm_executor.py"
LIVE_TIMEOUT = 90  # 秒，包含 SDK 初始化与动作执行


def validate_command(cmd):
    """校验指令格式，防止识别端的错误输出直接驱动机械臂。"""
    if not isinstance(cmd, dict):
        return False, "不是 JSON 对象"
    action = cmd.get("action")
    if action not in ALLOWED_ACTIONS:
        return False, f"action 不在白名单: {action!r}"
    return True, ""


def execute_in_wsl(cmd):
    """通过 wsl.exe 调用机械臂执行端，返回 (成功与否, 说明)。"""
    argv = ["wsl.exe", "-d", WSL_DISTRO, "--", WSL_PYTHON, WSL_EXECUTOR,
            json.dumps(cmd, ensure_ascii=False)]
    try:
        proc = subprocess.run(argv, capture_output=True, timeout=LIVE_TIMEOUT,
                              encoding="utf-8", errors="replace")
    except subprocess.TimeoutExpired:
        return False, "机械臂执行超时"
    lines = (proc.stdout or "").strip().splitlines()
    detail = lines[-1] if lines else (proc.stderr or "").strip()[-200:]
    return proc.returncode == 0, detail


def handle_command(cmd, addr):
    """校验通过后的入口，返回 (成功与否, 说明)。"""
    print(f"[指令] 来自 {addr}: {cmd}")
    if MODE == "sim":
        print("[模拟] sim 模式：不驱动真机，仅打印。连真机请用 --mode live")
        return True, "sim"
    ok, detail = execute_in_wsl(cmd)
    print(f"[执行] {'成功' if ok else '失败'}: {detail}")
    return ok, detail


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        addr = self.client_address[0]
        print(f"[连接] {addr} 已连接")
        for line in self.rfile:
            try:
                cmd = json.loads(line.decode("utf-8"))
            except json.JSONDecodeError:
                self.wfile.write(b"ERR bad json\n")
                continue
            ok, reason = validate_command(cmd)
            if ok:
                ok, detail = handle_command(cmd, addr)
                if ok:
                    self.wfile.write(b"OK\n")
                else:
                    self.wfile.write(f"ERR {detail}\n".encode("utf-8"))
            else:
                print(f"[拒绝] 来自 {addr}: {reason}")
                self.wfile.write(f"ERR {reason}\n".encode("utf-8"))
        print(f"[断开] {addr} 已断开")


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True


def main():
    global MODE
    parser = argparse.ArgumentParser(description="远程指令接收端")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--mode", choices=["sim", "live"], default="sim",
                        help="sim=只打印不驱动真机（默认）；live=转发给 WSL 内的机械臂")
    args = parser.parse_args()
    MODE = args.mode
    server = Server(("0.0.0.0", args.port), Handler)
    if MODE == "live":
        print("⚠ 真机模式：指令将驱动 Panthera-HT 机械臂，请确认周围无人、急停开关在手！")
    print(f"已在 0.0.0.0:{args.port} 监听（{MODE} 模式），等待识别端连接...")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n已停止")


if __name__ == "__main__":
    main()
