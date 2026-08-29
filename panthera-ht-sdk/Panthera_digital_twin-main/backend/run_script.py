#!/usr/bin/env python3
"""
Run Panthera example scripts.

Usage:
    python run_script.py --demo <script_name>        # Simulation mode
    python run_script.py --config <yaml> <script>    # Real robot mode

In demo mode, PantheraSim is used and joint state is pushed to the backend
via HTTP.  In real-robot mode, the script controls the actual hardware
through the backend's existing robot instance.
"""
import sys
import os
import types
import argparse

BACKEND_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.normpath(os.path.join(
    BACKEND_DIR, '..', '..', 'panthera_python', 'scripts'))

sys.path.insert(0, BACKEND_DIR)
sys.path.insert(0, SCRIPTS_DIR)


def run_demo(script_name):
    """Run script with PantheraSim, pushing state to backend via HTTP."""
    from panthera_sim import PantheraSim

    panthera_lib = types.ModuleType('Panthera_lib')
    panthera_lib.Panthera = PantheraSim
    panthera_lib.TrajectoryRecorder = None
    sys.modules['Panthera_lib'] = panthera_lib
    sys.modules['Panthera_lib.Panthera'] = types.ModuleType('Panthera_lib.Panthera')
    sys.modules['Panthera_lib.Panthera'].Panthera = PantheraSim

    _execute_script(script_name, mode_label="PantheraSim")


def run_real(config_path, script_name):
    """Run script with the real Panthera SDK (hardware control)."""
    # Import the real Panthera — needs hightorque_robot installed
    from Panthera_lib import Panthera as RealPanthera

    panthera_lib = types.ModuleType('Panthera_lib')
    panthera_lib.TrajectoryRecorder = None

    # Panthera() will be called with config_path
    def _make_robot():
        return RealPanthera(config_path)

    panthera_lib.Panthera = _make_robot
    sys.modules['Panthera_lib'] = panthera_lib
    sys.modules['Panthera_lib.Panthera'] = types.ModuleType('Panthera_lib.Panthera')
    sys.modules['Panthera_lib.Panthera'].Panthera = RealPanthera

    _execute_script(script_name, mode_label="Real Robot")


def _execute_script(script_name, mode_label="Panthera"):
    """Locate and execute the target script."""
    script_path = os.path.join(SCRIPTS_DIR, script_name)
    if not script_name.endswith('.py'):
        script_path += '.py'

    if not os.path.exists(script_path):
        print(f"Script not found: {script_path}")
        sys.exit(1)

    print(f"╔══════════════════════════════════════════════════════════╗")
    print(f"║  Runner — {mode_label:<48s}║")
    print(f"║  Script:  {os.path.basename(script_path):<47s}║")
    print(f"╚══════════════════════════════════════════════════════════╝")
    print()
    print("Press Ctrl+C to stop.")
    print()

    with open(script_path) as f:
        code = compile(f.read(), script_path, 'exec')

    exec_globals = {'__name__': '__main__', '__file__': script_path}
    exec(code, exec_globals)


# ── CLI ────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    p = argparse.ArgumentParser(description='Panthera Script Runner')
    p.add_argument('--demo', action='store_true', help='Simulation mode')
    p.add_argument('--config', type=str, default=None, help='Robot config YAML')
    p.add_argument('script', nargs='?', default=None, help='Script name')
    args = p.parse_args()

    if not args.script:
        print("Available scripts:")
        for f in sorted(os.listdir(SCRIPTS_DIR)):
            if f.endswith('.py') and f[0].isdigit():
                print(f"  {f}")
        sys.exit(1)

    if args.demo or not args.config:
        run_demo(args.script)
    else:
        run_real(args.config, args.script)
