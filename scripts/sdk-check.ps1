$ErrorActionPreference = 'Stop'

wsl.exe -d Ubuntu-24.04 -- bash -lc 'cd /mnt/f/hackson/独居女性/panthera-ht-sdk/sdk-source/panthera_python/scripts && PYTHONPATH=/mnt/f/hackson/独居女性/panthera-ht-sdk/sdk-source/panthera_python/scripts:/mnt/f/hackson/独居女性/panthera-ht-sdk/sdk-source/panthera_python /mnt/f/hackson/独居女性/panthera-ht-sdk/.venv-linux/bin/python -c "import hightorque_robot, pinocchio, yaml, scipy; from Panthera_lib import Panthera; print(\"Panthera SDK OK\")"'
