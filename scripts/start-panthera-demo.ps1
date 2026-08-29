$ErrorActionPreference = 'Stop'

wsl.exe -d Ubuntu-24.04 -- bash -lc 'cd /mnt/f/hackson/独居女性/panthera-ht-sdk/Panthera_digital_twin-main/backend && exec /mnt/f/hackson/独居女性/panthera-ht-sdk/.venv-linux/bin/python app.py --demo --port 5001'
