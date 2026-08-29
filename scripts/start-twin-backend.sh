#!/bin/bash
# 数字孪生后端启动脚本（WSL 内执行）。
# 用法（Windows 侧）：
#   Start-Process wsl.exe -ArgumentList '-d','Ubuntu-24.04','--','bash','/mnt/f/hackson/独居女性/scripts/start-twin-backend.sh' -WindowStyle Hidden
# 或直接在 WSL 终端里：bash /mnt/f/hackson/独居女性/scripts/start-twin-backend.sh
cd /mnt/f/hackson/独居女性/panthera-ht-sdk/Panthera_digital_twin-main/backend || exit 1
exec /mnt/f/hackson/独居女性/panthera-ht-sdk/.venv-linux/bin/python app.py \
    --config ../../panthera_python/robot_param/Follower.yaml --port 5000
