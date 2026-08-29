#!/usr/bin/env python3
"""
Digital Twin Backend - Flask + WebSocket Server
Connects to Panthera robot and streams real-time data to web interface.

Usage:
    python app.py                                   # Uses default config
    python app.py --config path/to/robot.yaml       # Custom config
    python app.py --demo                            # Demo mode without robot
"""
import sys
import os
import time
import threading
import logging
import argparse
import json
import re
import yaml
import numpy as np
import pinocchio as pin
from flask import Flask, jsonify, request, send_from_directory, Response
from flask_socketio import SocketIO, emit
from flask_cors import CORS

# Disable Flask's request logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# Add Panthera SDK to path
SDK_PATH = os.path.join(os.path.dirname(__file__), '..', '..', 'panthera_python')
sys.path.insert(0, SDK_PATH)
sys.path.insert(0, os.path.join(SDK_PATH, 'scripts'))

# Local config path (self-contained in digital_twin folder)
LOCAL_CONFIG_PATH = os.path.join(os.path.dirname(__file__), '..', 'robot_param', 'Follower.yaml')

app = Flask(__name__, static_folder='../frontend/dist', static_url_path='')
CORS(app, origins="*")
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# ============== GLOBAL SETTINGS ==============
CONTROL_FREQ = 200  # Hz - control loop frequency
BROADCAST_FREQ = 30  # Hz - WebSocket broadcast frequency
END_EFFECTOR_OFFSET = 0.07  # meters - offset from Link_6 origin to actual tool tip
ARM_JOINT_COUNT = 6
GRIPPER_JOINT_NAME = "gripper"
GRIPPER_DEFAULT_TARGET = 0.0
# =============================================

# Robot instance
robot = None
robot_config = None
demo_mode = False
script_mode = False  # True when external script is pushing state
script_state = {
    'positions': [0.0] * 6,
    'velocities': [0.0] * 6,
    'torques': [0.0] * 6,
    'fk': None
}

# Control state
target_positions = [0.0] * 6
target_velocity = 0.2
max_torque = [10.0, 10.0, 10.0, 10.0, 10.0, 2.0]
reset_profile_active = False
reset_profile_target = [0.0] * 6
reset_profile_started_at = 0.0
reset_profile_handoff_until = 0.0
reset_profile_handoff_positions = [0.0] * 6
reset_gripper_active = False
reset_gripper_target = 0.0
RESET_MAX_VELOCITY = 0.7
RESET_START_VELOCITY = 0.04
RESET_MIN_VELOCITY = 0.12
RESET_VELOCITY_GAIN = 1.8
RESET_VELOCITY_OFFSET = 0.06
RESET_NEAR_ZERO_THRESHOLD = 0.01
RESET_ACCEL_DURATION = 0.45
RESET_HANDOFF_DURATION = 0.18

# Current state
current_positions = [0.0] * 6
current_velocities = [0.0] * 6
current_torques = [0.0] * 6

# ============== CONTROL MODE SETTINGS ==============
# Modes: 'position', 'gravity_comp', 'gravity_friction', 'impedance'
control_mode = 'position'


# Gravity compensation parameters
gravity_gain = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
joint_offset = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
tau_limit = np.array([10.0, 10.0, 10.0, 10.0, 10.0, 3.7])
# Soft hold terms reduce sag from model error after the operator releases the arm.
gravity_hold_K = np.array([0.35, 0.55, 0.55, 0.25, 0.20, 0.15])
gravity_hold_B = np.array([0.12, 0.18, 0.18, 0.08, 0.06, 0.05])
gravity_hold_target = np.array([0.0] * 6)

# Impedance control parameters (PD + gravity)
# impedance_K = np.array([10.0, 21.0, 21.0, 16.0, 13.0, 1.0])  # Stiffness
# impedance_B = np.array([1.0, 2.0, 2.0, 0.9, 0.8, 0.1])    # Damping
impedance_K = np.array([5.0, 12.0, 12.0, 4.0, 4.0, 2.0])
impedance_B = np.array([0.5, 1.0, 1.0, 0.4, 0.4, 0.2])
impedance_target = np.array([0.0, 0.7, 0.7, -0.1, 0.0, 0.0])  # Target position

# Joint configuration
JOINT_CONFIG = []
URDF_PATH = None

# Thread control
target_lock = threading.Lock()
loop_running = False
connected_clients = set()

# Forward kinematics data
current_fk = {
    'position': [0.0, 0.0, 0.0],
    'euler': [0.0, 0.0, 0.0],  # Roll, Pitch, Yaw in degrees
    'rotation': [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
}

# Waypoints for trajectory execution (max 6)
MAX_WAYPOINTS = 6
waypoints = []
trajectory_running = False
trajectory_progress = 0.0
TRAJECTORY_START_BLEND_DURATION = 1.0

# Reusable graphical task files
TASKS_DIR = os.path.join(os.path.dirname(__file__), 'tasks')
os.makedirs(TASKS_DIR, exist_ok=True)
active_task_name = None
task_progress = 0.0


def rotation_matrix_to_euler(R):
    """Convert rotation matrix to euler angles (ZYX intrinsic order, degrees)

    Uses scipy.spatial.transform.Rotation for robust conversion.
    ZYX intrinsic = Yaw-Pitch-Roll convention.
    Returns: [roll, pitch, yaw] in degrees

    Note: For this robot's coordinate frame:
    - Roll = Y rotation (lateral tilt)
    - Pitch = X rotation (forward/back tilt)
    - Yaw = Z rotation (heading)
    """
    from scipy.spatial.transform import Rotation

    R = np.array(R)
    rot = Rotation.from_matrix(R)
    # ZYX intrinsic returns [Z, Y, X] = [yaw, pitch, roll] order
    zyx_angles = rot.as_euler('ZYX', degrees=True)
    # Swap to match robot's coordinate frame convention
    roll = zyx_angles[1]   # Y rotation
    pitch = zyx_angles[2]  # X rotation
    yaw = zyx_angles[0]    # Z rotation
    return [roll, pitch, yaw]

# Timing stats
timing_stats = {
    "loop_count": 0,
    "avg_cmd_time": 0.0,
    "overruns": 0
}

# External wrench (force/torque estimation)
current_wrench = [0.0] * 6
FT_LAMBDA = 0.05       # DLS damping for force estimation
FT_Fc = np.array([0.10, 0.12, 0.12, 0.08, 0.03, 0.02])
FT_Fv = np.array([0.04, 0.06, 0.06, 0.04, 0.02, 0.02])
FT_VEL_THRESH = 0.02
TOOL_OFFSET = np.array([0.165, 0.0, 0.0])


def compute_external_wrench(robot, joint_angles, joint_velocities, joint_torques):
    """Estimate external Cartesian force/torque from joint torque measurements.
    Returns: [Fx, Fy, Fz, Mx, My, Mz]
    """
    try:
        q = np.asarray(joint_angles)
        dq = np.asarray(joint_velocities)
        tau_measured = np.asarray(joint_torques)

        # Gravity compensation
        G = robot.get_Gravity(q)

        # Friction compensation
        F_friction = robot.get_friction_compensation(dq, FT_Fc, FT_Fv, FT_VEL_THRESH)

        # External joint torque = measured - model
        tau_ext = tau_measured - G - F_friction

        # Compute Jacobian at last joint
        q_pin = np.zeros(robot.model.nq)
        for i, name in enumerate(robot.joint_names):
            jid = robot.model.getJointId(name)
            q_pin[robot.model.joints[jid].idx_q] = q[i]

        pin.computeJointJacobians(robot.model, robot.data, q_pin)
        last_jid = robot.model.getJointId(robot.joint_names[-1])
        J_full = pin.getJointJacobian(
            robot.model, robot.data, last_jid,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )

        # Tool offset adjustment
        T_last = robot.data.oMi[last_jid]
        r_world = T_last.rotation @ TOOL_OFFSET
        skew_r = np.array([[0, -r_world[2], r_world[1]],
                           [r_world[2], 0, -r_world[0]],
                           [-r_world[1], r_world[0], 0]])
        J_tcp = J_full.copy()
        J_tcp[:3, :] -= skew_r @ J_full[3:, :]

        # Select 6 controlled joints
        cols = [robot.model.joints[robot.model.getJointId(n)].idx_v
                for n in robot.joint_names]
        J6 = J_tcp[:, cols]

        # DLS: F = J^T (J J^T + λ²I)^(-1) τ_ext
        JJT = J6 @ J6.T
        F_ext = J6.T @ np.linalg.solve(JJT + FT_LAMBDA**2 * np.eye(6), tau_ext)

        return F_ext.tolist()
    except Exception as e:
        # Silently return zero on any estimation error
        return [0.0] * 6


def precise_sleep(duration):
    """High precision sleep function"""
    if duration <= 0:
        return

    end_time = time.perf_counter() + duration

    # Use sleep for most of the time (leave 1ms margin)
    if duration > 0.001:
        time.sleep(duration - 0.001)

    # Busy wait for final precision
    while time.perf_counter() < end_time:
        pass


def _current_arm_positions():
    if robot is not None and not demo_mode:
        try:
            robot.send_get_motor_state_cmd()
            pos = robot.get_current_pos()
            return (pos.tolist() if hasattr(pos, 'tolist') else list(pos))[:ARM_JOINT_COUNT]
        except Exception:
            pass
    return current_positions.copy()


def _prepare_trajectory_from_current_pose():
    with target_lock:
        _cancel_reset_profile()

    current_start = _clamp_arm_positions(_current_arm_positions())
    waypoint_list = [wp['positions'] for wp in waypoints]
    gripper_list = [_clamp_gripper_position(wp.get('gripper', _gripper_target)) for wp in waypoints]
    durations = [wp['duration'] for wp in waypoints[:-1]]
    holds = [max(0.0, min(10.0, float(wp.get('settle', 0.5)))) for wp in waypoints[1:]]

    if waypoint_list:
        first_distance = max(
            abs(current_start[i] - waypoint_list[0][i])
            for i in range(min(len(current_start), len(waypoint_list[0])))
        )
        if first_distance > 0.01:
            waypoint_list = [current_start] + waypoint_list
            gripper_list = [_clamp_gripper_position(_read_gripper_position())] + gripper_list
            durations = [TRAJECTORY_START_BLEND_DURATION] + durations
            holds = [max(0.0, min(10.0, float(waypoints[0].get('settle', 0.5))))] + holds

    return waypoint_list, durations, gripper_list, holds


def _wait_for_waypoint_reached(target_pos, target_gripper, timeout=3.0):
    """Wait until the arm and gripper physically reach the target (or timeout)."""
    deadline = time.perf_counter() + max(0.1, timeout)
    while trajectory_running and time.perf_counter() < deadline:
        try:
            arm = np.asarray(_current_arm_positions(), dtype=float)
            arm_ok = len(arm) >= ARM_JOINT_COUNT and np.max(np.abs(arm[:ARM_JOINT_COUNT] - np.asarray(target_pos[:ARM_JOINT_COUNT]))) <= 0.035
            grip_ok = abs(float(_read_gripper_position()) - float(target_gripper)) <= 0.05
            if arm_ok and grip_ok:
                return True
        except Exception:
            pass
        precise_sleep(0.02)
    return False


def execute_trajectory_thread(waypoint_list, durations, gripper_list=None, holds=None, control_rate=100):
    """Execute trajectory in a separate thread"""
    global trajectory_running, trajectory_progress, target_positions, control_mode

    if len(waypoint_list) < 2:
        print("Need at least 2 waypoints for trajectory")
        return False

    if gripper_list is None or len(gripper_list) != len(waypoint_list):
        gripper_list = [_clamp_gripper_position(_gripper_target)] * len(waypoint_list)
    if holds is None or len(holds) != len(waypoint_list) - 1:
        holds = [0.5] * max(0, len(waypoint_list) - 1)

    if robot is None or demo_mode:
        print("Cannot execute trajectory: robot not available or in demo mode")
        # In demo mode, simulate trajectory execution
        if demo_mode:
            trajectory_running = True
            total_duration = sum(durations)
            elapsed = 0

            for seg_idx, duration in enumerate(durations):
                start_pos = waypoint_list[seg_idx]
                end_pos = waypoint_list[seg_idx + 1]
                start_gripper = gripper_list[seg_idx]
                end_gripper = gripper_list[seg_idx + 1]
                steps = int(duration * 30)  # Lower rate for demo

                for step in range(steps):
                    if not trajectory_running:
                        return False

                    t = step / steps
                    # Smooth interpolation
                    s = 3 * t**2 - 2 * t**3
                    pos = [start_pos[i] + s * (end_pos[i] - start_pos[i]) for i in range(len(start_pos))]

                    with target_lock:
                        target_positions[:] = pos
                    _set_gripper_target(start_gripper + s * (end_gripper - start_gripper), velocity=0.2)

                    elapsed += duration / steps
                    trajectory_progress = elapsed / total_duration
                    socketio.emit('trajectory_progress', {'progress': trajectory_progress})
                    time.sleep(1.0 / 30)
                _wait_for_waypoint_reached(end_pos, end_gripper, timeout=max(1.0, duration + 2.0))
                precise_sleep(holds[seg_idx])

            trajectory_progress = 1.0
            socketio.emit('trajectory_progress', {'progress': 1.0})
            socketio.emit('trajectory_complete', {'success': True})
            trajectory_running = False
            return True
        return False

    # Switch to trajectory mode - control_loop will skip, letting this thread have exclusive control.
    control_mode = 'trajectory'

    trajectory_running = True
    trajectory_progress = 0.0

    dt = 1.0 / control_rate
    total_duration = sum(durations)
    elapsed_total = 0

    try:
        for segment in range(len(durations)):
            start_pos = waypoint_list[segment]
            end_pos = waypoint_list[segment + 1]
            start_gripper = gripper_list[segment]
            end_gripper = gripper_list[segment + 1]
            duration = durations[segment]

            steps = int(duration * control_rate)
            segment_start = time.perf_counter()

            for step in range(steps):
                if not trajectory_running:
                    # Trajectory was cancelled
                    socketio.emit('trajectory_complete', {'success': False, 'cancelled': True})
                    return False

                target_time = segment_start + (step + 1) * dt
                current_time = step * dt

                # Generate interpolated trajectory using septic polynomial
                pos, vel, _ = robot.septic_interpolation(start_pos, end_pos, duration, current_time)

                # Send control command
                robot.Joint_Pos_Vel(pos, vel, max_torque)
                s = step / max(steps, 1)
                _set_gripper_target(start_gripper + s * (end_gripper - start_gripper), velocity=0.2)

                # Update progress
                elapsed_total = sum(durations[:segment]) + current_time
                trajectory_progress = elapsed_total / total_duration

                # Broadcast progress to clients
                socketio.emit('trajectory_progress', {'progress': trajectory_progress})

                # High precision wait
                wait_time = target_time - time.perf_counter()
                if wait_time > 0:
                    precise_sleep(wait_time)

            # Do not advance to the next timeline step until both mechanisms are settled.
            _wait_for_waypoint_reached(end_pos, end_gripper, timeout=max(1.0, duration + 2.0))
            precise_sleep(holds[segment])

        # Move to final position
        final_pos = waypoint_list[-1]
        robot.Joint_Pos_Vel(final_pos, [0.0] * robot.motor_count, max_torque)
        _set_gripper_target(gripper_list[-1], velocity=0.2)

        trajectory_progress = 1.0
        socketio.emit('trajectory_progress', {'progress': 1.0})
        socketio.emit('trajectory_complete', {'success': True})

    except Exception as e:
        print(f"Trajectory execution error: {e}")
        socketio.emit('trajectory_complete', {'success': False, 'error': str(e)})

    finally:
        trajectory_running = False
        control_mode = 'position'
        with target_lock:
            if waypoint_list:
                target_positions[:] = _clamp_arm_positions(waypoint_list[-1])

    return True


def load_config(config_path):
    """Load robot configuration from YAML file"""
    global robot_config, JOINT_CONFIG, URDF_PATH

    with open(config_path, 'r', encoding='utf-8') as f:
        robot_config = yaml.safe_load(f)

    config_dir = os.path.dirname(os.path.abspath(config_path))

    # Load URDF path
    urdf_relative = robot_config['urdf']['file_path']
    URDF_PATH = os.path.normpath(os.path.join(config_dir, urdf_relative))

    # Load joint configuration
    joint_names = robot_config['kinematics']['joint_names']
    lower_limits = robot_config['robot']['joint_limits']['lower']
    upper_limits = robot_config['robot']['joint_limits']['upper']

    JOINT_CONFIG = []
    for i, name in enumerate(joint_names):
        JOINT_CONFIG.append({
            "name": name,
            "index": i,
            "min": lower_limits[i],
            "max": upper_limits[i],
            "kind": "arm"
        })

    gripper_limits = robot_config.get('robot', {}).get('gripper_limits')
    if gripper_limits:
        JOINT_CONFIG.append({
            "name": GRIPPER_JOINT_NAME,
            "index": len(joint_names),
            "min": float(gripper_limits.get('lower', 0.0)),
            "max": float(gripper_limits.get('upper', 1.8)),
            "kind": "gripper"
        })

    print(f"Config loaded: {robot_config['robot']['name']}")
    print(f"URDF: {URDF_PATH}")
    print(f"Joints: {len(JOINT_CONFIG)}")

    return robot_config


def _arm_joint_config():
    return [jc for jc in JOINT_CONFIG if jc.get("kind") != "gripper"]


def _gripper_config():
    for jc in JOINT_CONFIG:
        if jc.get("kind") == "gripper" or jc.get("name") == GRIPPER_JOINT_NAME:
            return jc
    return None


def _gripper_limits_list():
    cfg = _gripper_config()
    if not cfg:
        return None
    return [cfg["min"], cfg["max"]]


def _clamp_gripper_position(position):
    cfg = _gripper_config()
    position = float(position)
    if cfg:
        position = max(cfg["min"], min(cfg["max"], position))
    return position


def _set_gripper_target(position, velocity=0.3):
    global _gripper_target
    position = _clamp_gripper_position(position)
    _gripper_target = position
    if robot is not None and not demo_mode:
        try:
            robot.gripper_control(position, velocity, 0.15)
        except Exception:
            pass
    return position


def _set_default_gripper_mit_hold():
    global _gripper_target
    _gripper_target = 0.0
    if robot is not None and not demo_mode:
        try:
            robot.gripper_control_MIT(0.0, 0.0, 0.0, 0.9, 0.06)
        except Exception:
            pass


def _release_gripper():
    """Release gripper torque so it can be moved freely by hand."""
    if robot is not None and not demo_mode:
        try:
            robot.gripper_control_MIT(0.0, 0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass


def _hold_gripper_impedance():
    """Keep gripper in a light MIT hold while arm impedance is active."""
    if robot is not None and not demo_mode:
        try:
            robot.gripper_control_MIT(1.0, 0.0, 0.0, 0.65, 0.06)
        except Exception:
            pass


def _read_gripper_position():
    global _gripper_target
    if robot is not None and not demo_mode:
        try:
            state = robot.get_current_state_gripper()
            _gripper_target = _clamp_gripper_position(state.position)
        except Exception:
            pass
    return _gripper_target


def _is_gripper_index(joint_index):
    cfg = _gripper_config()
    return cfg is not None and joint_index == cfg["index"]


def _reset_velocity_from_error(error):
    return max(
        RESET_MIN_VELOCITY,
        min(RESET_MAX_VELOCITY, RESET_VELOCITY_GAIN * error + RESET_VELOCITY_OFFSET)
    )


def _smoothstep(progress):
    progress = max(0.0, min(1.0, progress))
    return progress * progress * progress * (progress * (progress * 6.0 - 15.0) + 10.0)


def _reset_accel_velocity_cap(elapsed):
    if RESET_ACCEL_DURATION <= 0:
        return RESET_MAX_VELOCITY
    progress = _smoothstep(elapsed / RESET_ACCEL_DURATION)
    return RESET_START_VELOCITY + (RESET_MAX_VELOCITY - RESET_START_VELOCITY) * progress


def _clamp_arm_positions(positions):
    arm_config = _arm_joint_config()
    next_positions = list(positions[:len(target_positions)])
    for i, pos in enumerate(next_positions):
        if i < len(arm_config):
            jc = arm_config[i]
            next_positions[i] = max(jc["min"], min(jc["max"], pos))
    return next_positions


def _cancel_reset_profile():
    global reset_profile_active, reset_gripper_active, reset_profile_handoff_until
    reset_profile_active = False
    reset_gripper_active = False
    reset_profile_handoff_until = 0.0


def _start_smooth_position_move(positions, gripper_target=None, with_handoff=False):
    global target_positions, target_velocity, reset_profile_active, reset_profile_target, reset_profile_started_at
    global reset_profile_handoff_until, reset_profile_handoff_positions
    global reset_gripper_active, reset_gripper_target

    reset_profile_target[:] = _clamp_arm_positions(positions)
    target_positions[:] = reset_profile_target.copy()
    if gripper_target is not None:
        reset_gripper_target = _clamp_gripper_position(gripper_target)
        reset_gripper_active = True
    else:
        reset_gripper_active = False
    now = time.time()
    reset_profile_started_at = now
    reset_profile_handoff_until = 0.0
    reset_profile_handoff_positions[:] = current_positions[:len(reset_profile_handoff_positions)]
    if with_handoff:
        reset_profile_handoff_positions[:] = _clamp_arm_positions(_current_arm_positions())
        reset_profile_handoff_until = now + RESET_HANDOFF_DURATION
        reset_profile_started_at = reset_profile_handoff_until
    target_velocity = RESET_MAX_VELOCITY
    reset_profile_active = True


def _start_smooth_position_reset(with_handoff=False):
    _start_smooth_position_move(
        [0.0] * len(target_positions),
        gripper_target=0.0,
        with_handoff=with_handoff
    )


def _hold_current_position_target():
    global target_positions

    hold_positions = _clamp_arm_positions(_current_arm_positions())
    target_positions[:] = hold_positions
    _cancel_reset_profile()


def _set_control_mode(mode):
    global control_mode, impedance_target, reset_profile_active, reset_gripper_active, gravity_hold_target

    previous_mode = control_mode
    control_mode = mode

    if mode == 'position':
        if previous_mode in ['gravity_comp', 'gravity_friction', 'impedance']:
            _hold_current_position_target()
        return

    _cancel_reset_profile()

    if mode == 'impedance':
        impedance_target = np.array(current_positions)
    elif mode in ['gravity_comp', 'gravity_friction']:
        # Capture the pose at entry. A very soft hold term keeps the arm from
        # sagging when released while still allowing deliberate hand guidance.
        gravity_hold_target = np.array(_current_arm_positions(), dtype=float)


def init_robot(config_path):
    """Initialize robot connection"""
    global robot, demo_mode

    try:
        if demo_mode:
            # In demo mode, use PantheraSim for FK kinematics (no hardware needed)
            print("Running in DEMO mode (no motor control)")
            try:
                from panthera_sim import PantheraSim
                robot = PantheraSim(config_path)
                print(f"Kinematics initialized for {robot.motor_count} joints (FK available)")
            except Exception as e:
                print(f"Could not initialize kinematics: {e}")
                print("FK will not be available in demo mode")
                robot = None
            return True

        from scripts.Panthera_lib.Panthera import Panthera
        robot = Panthera(config_path)
        print(f"Robot initialized with {robot.motor_count} motors")

        # Read fresh motor state before getting positions
        robot.send_get_motor_state_cmd()
        robot.motor_send_cmd()
        time.sleep(0.1)

        pos = robot.get_current_pos()
        # Validate that positions are reasonable (not uninitialized 999)
        pos_list = pos.tolist() if hasattr(pos, 'tolist') else list(pos)
        if any(abs(p) > 100 for p in pos_list):
            print(f"[WARN] Motor positions look invalid ({pos_list}), defaulting to zero")
            pos_list = [0.0] * len(pos_list)

        global target_positions, current_positions, _gripper_target
        target_positions = pos_list
        current_positions = pos_list
        _set_default_gripper_mit_hold()

        return True
    except Exception as e:
        print(f"Failed to initialize robot: {e}")
        print("Falling back to DEMO mode")
        demo_mode = True
        return True


def control_loop():
    """Main control loop - sends commands to robot based on control mode"""
    global current_positions, current_velocities, current_torques, timing_stats
    global control_mode, impedance_target, reset_profile_active, reset_gripper_active

    dt = 1.0 / CONTROL_FREQ

    # Zero arrays for torque-only control
    zero_pos = [0.0] * 6
    zero_vel = [0.0] * 6
    zero_kp = [0.0] * 6
    zero_kd = [0.0] * 6

    while loop_running:
        loop_start = time.time()

        try:
            if script_mode:
                # Let the script thread control the robot exclusively
                pass
            elif not demo_mode and robot is not None:
                with target_lock:
                    mode = control_mode
                    targets = target_positions.copy()
                    vel_target = target_velocity
                    reset_active = reset_profile_active
                    reset_target = reset_profile_target.copy()
                    reset_started_at = reset_profile_started_at
                    reset_handoff_until = reset_profile_handoff_until
                    reset_handoff_positions = reset_profile_handoff_positions.copy()
                    gripper_reset_active = reset_gripper_active
                    gripper_reset_target = reset_gripper_target
                    imp_target = impedance_target.copy()

                # Process keyboard input (nudges targets)
                _process_keyboard()

                t1 = time.time()

                if mode == 'position':
                    # Position control mode - direct joint control
                    # Safety clamp
                    arm_config = _arm_joint_config()
                    if arm_config:
                        for i in range(min(len(targets), len(arm_config))):
                            jc = arm_config[i]
                            targets[i] = max(jc['min'], min(jc['max'], targets[i]))
                    if reset_active:
                        now = time.time()
                        if reset_handoff_until > now:
                            targets = reset_handoff_positions[:len(targets)]
                            vel = [0.0] * len(targets)
                        else:
                            targets = reset_target[:len(targets)]
                            errors = [abs(targets[i] - current_positions[i]) for i in range(len(targets))]
                            max_error = max(errors) if errors else 0.0
                            accel_cap = _reset_accel_velocity_cap(now - reset_started_at)
                            vel = [min(accel_cap, _reset_velocity_from_error(error)) for error in errors]
                            if max_error < RESET_NEAR_ZERO_THRESHOLD:
                                vel = [RESET_MIN_VELOCITY] * len(targets)
                    else:
                        if _replay_velocities is not None and len(_replay_velocities) == len(targets):
                            vel = list(_replay_velocities)
                        else:
                            vel = [vel_target] * len(targets)
                    robot.Joint_Pos_Vel(targets, vel, max_torque, iswait=False)

                    if gripper_reset_active:
                        current_gripper_position = _read_gripper_position()
                        gripper_error = abs(gripper_reset_target - current_gripper_position)
                        accel_cap = _reset_accel_velocity_cap(time.time() - reset_started_at)
                        gripper_velocity = min(accel_cap, _reset_velocity_from_error(gripper_error))
                        if gripper_error < RESET_NEAR_ZERO_THRESHOLD:
                            gripper_velocity = RESET_MIN_VELOCITY
                        _set_gripper_target(gripper_reset_target, velocity=gripper_velocity)

                elif mode == 'gravity_comp':
                    # Gravity compensation plus a very soft pose hold.
                    robot.send_get_motor_state_cmd()
                    q = robot.get_current_pos()
                    dq = robot.get_current_vel()
                    tor = robot.get_Gravity(q + joint_offset) * gravity_gain
                    tor += gravity_hold_K * (gravity_hold_target - q) - gravity_hold_B * dq
                    tor = np.clip(tor, -tau_limit, tau_limit)
                    robot.pos_vel_tqe_kp_kd(zero_pos, zero_vel, tor.tolist(), zero_kp, zero_kd)
                    _release_gripper()

                elif mode == 'gravity_friction':
                    # Gravity + friction compensation mode
                    robot.send_get_motor_state_cmd()
                    q = robot.get_current_pos() + joint_offset
                    dq = robot.get_current_vel()
                    tor_g = robot.get_Gravity(q + joint_offset) * gravity_gain
                    tor_f = robot.get_friction_compensation(dq, FT_Fc, FT_Fv, FT_VEL_THRESH)
                    tor = tor_g + tor_f + gravity_hold_K * (gravity_hold_target - q) - gravity_hold_B * dq
                    tor = np.clip(tor, -tau_limit, tau_limit)
                    robot.pos_vel_tqe_kp_kd(zero_pos, zero_vel, tor.tolist(), zero_kp, zero_kd)
                    _release_gripper()

                elif mode == 'impedance':
                    # Impedance control mode - PD + gravity compensation
                    robot.send_get_motor_state_cmd()
                    q_current = robot.get_current_pos()
                    vel_current = robot.get_current_vel()

                    # PD torque
                    tor_pd = impedance_K * (imp_target - q_current) + impedance_B * (0.0 - vel_current)

                    # Gravity compensation
                    G = robot.get_Gravity(q_current + joint_offset)

                    # Total torque
                    tor = tor_pd + G * gravity_gain
                    tor = np.clip(tor, -tau_limit, tau_limit)
                    robot.pos_vel_tqe_kp_kd(zero_pos, zero_vel, tor.tolist(), zero_kp, zero_kd)
                    _hold_gripper_impedance()

                elif mode == 'trajectory':
                    # Trajectory mode - control handled by execute_trajectory_thread
                    # Just skip, don't send any commands
                    pass

                cmd_time = (time.time() - t1) * 1000

                timing_stats["loop_count"] += 1
                timing_stats["avg_cmd_time"] = (
                    timing_stats["avg_cmd_time"] * 0.95 + cmd_time * 0.05
                )
        except Exception as e:
            print(f"Control loop error: {e}")

        elapsed = time.time() - loop_start
        if elapsed > dt:
            timing_stats["overruns"] += 1

        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


def state_broadcast_loop():
    """Broadcast robot state to connected clients"""
    global current_positions, current_velocities, current_torques, current_fk

    dt = 1.0 / BROADCAST_FREQ

    while loop_running:
        loop_start = time.time()

        try:
            if script_mode:
                # Script is running — still read real robot state for frontend
                # (the script thread controls motors; we only observe)
                if not demo_mode and robot is not None:
                    try:
                        pos = robot.get_current_pos()
                        vel = robot.get_current_vel()
                        tqe = robot.get_current_torque()
                        current_positions[:] = pos.tolist() if hasattr(pos, 'tolist') else list(pos)
                        current_velocities[:] = vel.tolist() if hasattr(vel, 'tolist') else list(vel)
                        current_torques[:] = tqe.tolist() if hasattr(tqe, 'tolist') else list(tqe)
                    except Exception:
                        pass

            elif not demo_mode and robot is not None:
                # Read fresh state from real robot
                robot.send_get_motor_state_cmd()
                robot.motor_send_cmd()

                pos = robot.get_current_pos()
                vel = robot.get_current_vel()
                tqe = robot.get_current_torque()

                current_positions = pos.tolist() if hasattr(pos, 'tolist') else list(pos)
                current_velocities = vel.tolist() if hasattr(vel, 'tolist') else list(vel)
                current_torques = tqe.tolist() if hasattr(tqe, 'tolist') else list(tqe)

                # Calculate forward kinematics
                try:
                    fk = robot.forward_kinematics(pos)
                    if fk:
                        current_fk['position'] = fk['position'].tolist() if hasattr(fk['position'], 'tolist') else list(fk['position'])
                        rotation = fk['rotation']
                        current_fk['rotation'] = rotation.tolist() if hasattr(rotation, 'tolist') else [list(row) for row in rotation]
                        current_fk['euler'] = rotation_matrix_to_euler(rotation)
                except Exception as fk_error:
                    pass  # FK calculation failed, keep previous values

            else:
                # Demo mode: smoothly interpolate to target
                with target_lock:
                    targets = target_positions.copy()

                for i in range(len(current_positions)):
                    diff = targets[i] - current_positions[i]
                    current_positions[i] += diff * 0.1  # Smooth interpolation

                # Demo mode FK: use robot if available, otherwise skip
                if robot is not None:
                    try:
                        fk = robot.forward_kinematics(np.array(current_positions))
                        if fk:
                            current_fk['position'] = fk['position'].tolist() if hasattr(fk['position'], 'tolist') else list(fk['position'])
                            rotation = fk['rotation']
                            current_fk['rotation'] = rotation.tolist() if hasattr(rotation, 'tolist') else [list(row) for row in rotation]
                            current_fk['euler'] = rotation_matrix_to_euler(rotation)
                    except Exception:
                        pass

            # External wrench estimation (only in live mode)
            if not demo_mode and robot is not None:
                try:
                    wrench = compute_external_wrench(
                        robot, current_positions, current_velocities, current_torques
                    )
                    current_wrench[:] = wrench
                except Exception:
                    pass

            # 示教录制：无论是否有前端连接，逐帧采集（与广播同频）
            if _record_state["recording"]:
                try:
                    with target_lock:
                        _rec_mode = control_mode
                        _rec_grip = _read_gripper_position()
                    _record_state["frames"].append({
                        "t": round(time.time() - _record_state["start_time"], 4),
                        "q": [round(v, 5) for v in current_positions],
                        "dq": [round(v, 5) for v in current_velocities],
                        "tau": [round(v, 5) for v in current_torques],
                        "gripper": round(_rec_grip, 5),
                        "mode": _rec_mode,
                    })
                except Exception as _e:
                    print(f"Record frame error: {_e}")

            # Broadcast to all connected clients
            if connected_clients:
                with target_lock:
                    mode = control_mode
                    imp_target = impedance_target.tolist()
                    gripper_position = _read_gripper_position()

                socketio.emit('robot_state', {
                    'positions': current_positions,
                    'velocities': current_velocities,
                    'torques': current_torques,
                    'target_positions': target_positions,
                    'gripper_position': gripper_position,
                    'control_mode': mode,
                    'impedance_target': imp_target,
                    'forward_kinematics': current_fk,
                    'ee_position': current_fk['position'],
                    'ee_euler': current_fk['euler'],
                    'external_wrench': current_wrench,
                    'timestamp': time.time()
                })

        except Exception as e:
            print(f"Broadcast loop error: {e}")

        elapsed = time.time() - loop_start
        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


def start_loops():
    """Start control and broadcast loops"""
    global loop_running
    loop_running = True

    # Control loop (high frequency)
    control_thread = threading.Thread(target=control_loop, daemon=True)
    control_thread.start()

    # Broadcast loop (medium frequency)
    broadcast_thread = threading.Thread(target=state_broadcast_loop, daemon=True)
    broadcast_thread.start()

    print(f"Control loop started at {CONTROL_FREQ} Hz")
    print(f"Broadcast loop started at {BROADCAST_FREQ} Hz")


# ============== Static Files ==============
ARM_DESCRIPTION_PATH = os.path.join(os.path.dirname(__file__), '..', 'arm_description')
PANTHERA_HT_PATH = os.path.join(os.path.dirname(__file__), '..', '..', 'panthera_python', 'Panthera-HT_description')


@app.route('/arm_description/<path:filename>')
def serve_arm_description(filename):
    """Serve files from arm_description folder"""
    return send_from_directory(ARM_DESCRIPTION_PATH, filename)


@app.route('/Panthera-HT_description/<path:filename>')
def serve_panthera_ht(filename):
    """Serve files from Panthera-HT_description folder"""
    return send_from_directory(PANTHERA_HT_PATH, filename)


# ============== REST API Routes ==============

@app.route('/')
def index():
    """Serve frontend"""
    return send_from_directory(app.static_folder, 'index.html')


@app.route('/api/config')
def get_config():
    """Get robot configuration for frontend"""
    return jsonify({
        "robot_name": robot_config['robot']['name'] if robot_config else "Unknown",
        "joints": JOINT_CONFIG,
        "urdf_path": URDF_PATH,
        "demo_mode": demo_mode,
        "control_freq": CONTROL_FREQ,
        "connected": robot is not None or demo_mode,
        "control_mode": control_mode,
        "end_effector_link": robot_config.get('urdf', {}).get('end_effector_link') if robot_config else None,
        "end_effector_offset": END_EFFECTOR_OFFSET,
        "gripper_limits": _gripper_limits_list(),
        "impedance_kp": impedance_K.tolist(),
    })


@app.route('/api/arm_description_files')
def get_arm_description_files():
    """Get list of files in arm_description and Panthera-HT_description folders for auto-loading"""
    files = {}

    def scan_directory(path, prefix='', url_prefix=''):
        """Recursively scan directory and build file list"""
        for entry in os.scandir(path):
            rel_path = os.path.join(prefix, entry.name) if prefix else entry.name
            if entry.is_file():
                url_path = rel_path
                if url_prefix:
                    url_path = f'{url_prefix}/{rel_path}'
                files[rel_path] = url_path
            elif entry.is_dir():
                scan_directory(entry.path, rel_path, url_prefix)

    try:
        # Scan arm_description
        scan_directory(ARM_DESCRIPTION_PATH, url_prefix='/arm_description')
        # Also scan Panthera-HT_description
        scan_directory(PANTHERA_HT_PATH, url_prefix='/Panthera-HT_description')
        return jsonify({
            "success": True,
            "files": files,
            "base_url": "/arm_description"
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500


@app.route('/api/status')
def get_status():
    """Get current robot status (REST fallback)"""
    return jsonify({
        "connected": robot is not None or demo_mode,
        "demo_mode": demo_mode,
        "script_mode": script_mode,
        "script_running": _script_is_running(),
        "current_script": _script_name,
        "positions": current_positions,
        "velocities": current_velocities,
        "torques": current_torques,
        "target_positions": target_positions,
        "target_velocity": target_velocity,
        "gripper_position": _read_gripper_position()
    })


@app.route('/api/move_joint', methods=['POST'])
def move_joint():
    """Move a single joint"""
    global target_positions

    data = request.json
    joint_index = data.get('joint')
    position = data.get('position')

    if joint_index is not None and position is not None:
        joint_index = int(joint_index)
        if _is_gripper_index(joint_index):
            _cancel_reset_profile()
            _set_gripper_target(position)
            return jsonify({"success": True})

        # Clamp to joint limits
        arm_config = _arm_joint_config()
        if arm_config and joint_index < len(arm_config):
            jc = arm_config[joint_index]
            position = max(jc['min'], min(jc['max'], position))

        if joint_index < len(target_positions):
            with target_lock:
                _cancel_reset_profile()
                target_positions[joint_index] = position

    return jsonify({"success": True})


@app.route('/api/move', methods=['POST'])
def move_all():
    """Move all joints"""
    global target_positions, target_velocity

    data = request.json
    gripper_velocity = data.get('velocity', 0.3)

    with target_lock:
        _cancel_reset_profile()
        if 'positions' in data:
            positions = list(data['positions'])
            target_positions[:] = _clamp_arm_positions(positions)
            gripper_cfg = _gripper_config()
            if gripper_cfg and len(positions) > gripper_cfg["index"]:
                _set_gripper_target(positions[gripper_cfg["index"]], velocity=gripper_velocity)

        if 'gripper' in data:
            _set_gripper_target(data['gripper'], velocity=gripper_velocity)

        if 'velocity' in data:
            target_velocity = data['velocity']

    return jsonify({"success": True})


@app.route('/api/home', methods=['POST'])
def go_home():
    """Move to home position"""
    with target_lock:
        _start_smooth_position_reset()

    return jsonify({"success": True})


@app.route('/api/stop', methods=['POST'])
def stop():
    """Stop at current position"""
    global target_positions

    _request_replay_stop()
    with target_lock:
        _cancel_reset_profile()
        target_positions[:] = current_positions.copy()

    return jsonify({"success": True})


# ── 示教路径录制 ───────────────────────────────────────
# 录制挂在广播循环里（~30Hz），由网页/REST 自主开始停止，不依赖外部进程同步。
import json as _json

_record_state = {
    "recording": False,
    "frames": [],
    "name": None,
    "label": None,
    "start_time": None,
}
RECORD_OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "records")


def _record_stats():
    frames = _record_state["frames"]
    if not frames:
        return {}
    dur = frames[-1]["t"]
    q = [f["q"] for f in frames]
    rng = [round(max(col) - min(col), 3) for col in zip(*q)] if q else []
    grips = [f["gripper"] for f in frames]
    return {
        "frames": len(frames),
        "duration_s": round(dur, 2),
        "avg_rate_hz": round(len(frames) / dur, 1) if dur > 0 else 0,
        "joint_travel_rad": rng,
        "gripper_range": [round(min(grips), 3), round(max(grips), 3)],
    }


@app.route('/api/record/start', methods=['POST'])
def record_start():
    if _record_state["recording"]:
        return jsonify({"success": False, "error": "已在录制中"}), 400
    data = request.json or {}
    _record_state.update({
        "recording": True,
        "frames": [],
        "name": data.get("name") or time.strftime("ep_%Y%m%d_%H%M%S"),
        "label": data.get("label", "success"),
        "start_time": time.time(),
    })
    print(f"[record] ● 开始录制 {_record_state['name']}")
    return jsonify({"success": True, "name": _record_state["name"]})


@app.route('/api/record/stop', methods=['POST'])
def record_stop():
    if not _record_state["recording"]:
        return jsonify({"success": False, "error": "当前没有在录制"}), 400
    data = request.json or {}
    if "label" in data:
        _record_state["label"] = data["label"]
    _record_state["recording"] = False

    stats = _record_stats()
    out_dir = os.path.join(RECORD_OUT_DIR, _record_state["name"])
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "states.jsonl"), "w", encoding="utf-8") as f:
        for frame in _record_state["frames"]:
            f.write(_json.dumps(frame, ensure_ascii=False) + "\n")
    meta = {
        "name": _record_state["name"],
        "label": _record_state["label"],
        "source": "digital_twin_backend",
        "demo_mode": demo_mode,
        "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        **stats,
    }
    with open(os.path.join(out_dir, "meta.json"), "w", encoding="utf-8") as f:
        _json.dump(meta, f, ensure_ascii=False, indent=2)
    _record_state["frames"] = []
    print(f"[record] ■ 录制结束 -> {out_dir} ({stats.get('frames', 0)} 帧)")
    return jsonify({"success": True, "saved_to": out_dir, **stats})


@app.route('/api/record/status')
def record_status():
    return jsonify({
        "recording": _record_state["recording"],
        "name": _record_state["name"],
        "label": _record_state["label"],
        "live_frames": len(_record_state["frames"]),
        "elapsed_s": round(time.time() - _record_state["start_time"], 1) if _record_state["recording"] and _record_state["start_time"] else 0.0,
        **_record_stats(),
    })


@app.route('/api/records/list')
def records_list():
    """已保存的录制列表（供录制页校对标注）"""
    items = []
    if os.path.isdir(RECORD_OUT_DIR):
        for name in sorted(os.listdir(RECORD_OUT_DIR)):
            meta_path = os.path.join(RECORD_OUT_DIR, name, "meta.json")
            if os.path.isfile(meta_path):
                try:
                    with open(meta_path, "r", encoding="utf-8") as f:
                        m = json.load(f)
                    items.append({
                        "name": name,
                        "label": m.get("label", "?"),
                        "duration_s": m.get("duration_s"),
                        "frames": m.get("frames"),
                        "gripper_range": m.get("gripper_range"),
                    })
                except Exception:
                    pass
    return jsonify({"count": len(items), "items": items})


@app.route('/api/record/label', methods=['POST'])
def record_set_label():
    """事后修正某个已保存 episode 的标注"""
    data = request.json or {}
    name = data.get("name")
    label = data.get("label")
    if not name or label not in ("success", "fail"):
        return jsonify({"success": False, "error": "需要 name 和 label(success/fail)"}), 400
    meta_path = os.path.join(RECORD_OUT_DIR, name, "meta.json")
    if not os.path.isfile(meta_path):
        return jsonify({"success": False, "error": f"找不到 {name}"}), 404
    with open(meta_path, "r", encoding="utf-8") as f:
        meta = json.load(f)
    meta["label"] = label
    meta.pop("fail_reason", None)
    if label == "fail" and data.get("reason"):
        meta["fail_reason"] = data["reason"]
    meta["label_updated_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    return jsonify({"success": True, "name": name, "label": label})


@app.route('/api/record/discard', methods=['POST'])
def record_discard():
    """作废并删除一段录制（操作失误时用，数据直接删掉不进训练集）"""
    data = request.json or {}
    name = data.get("name")
    if not name:
        return jsonify({"success": False, "error": "需要 name"}), 400
    out_dir = os.path.join(RECORD_OUT_DIR, name)
    if not os.path.isdir(out_dir):
        return jsonify({"success": False, "error": f"找不到 {name}"}), 404
    import shutil as _shutil
    _shutil.rmtree(out_dir)
    print(f"[record] 🗑 删除录制 {name}")
    return jsonify({"success": True, "deleted": name})


@app.route('/record')
def record_page():
    """录制控制页：常驻录制状态横幅 + 开始/停止 + 已录列表（改标注/删除）。"""
    return '''<!doctype html>
<html><head><meta charset="utf-8"><title>示教录制</title>
<style>
 body{font-family:sans-serif;max-width:640px;margin:24px auto;padding:0 16px;background:#111;color:#eee}
 button{font-size:16px;padding:10px 20px;margin:4px 6px 4px 0;border:none;border-radius:6px;cursor:pointer}
 #start{background:#c0392b;color:#fff} #stop{background:#27ae60;color:#fff}
 select,input{font-size:15px;padding:6px;background:#222;color:#eee;border:1px solid #555;border-radius:4px}
 #banner{position:fixed;top:0;left:0;right:0;background:#c0392b;color:#fff;font-weight:bold;
   text-align:center;padding:10px;font-size:20px;z-index:99;display:none}
 #status{margin-top:12px;padding:12px;background:#222;border-radius:6px;white-space:pre-wrap;font-family:monospace;font-size:13px}
 table{border-collapse:collapse;margin-top:10px;width:100%;font-size:14px}
 th,td{border:1px solid #444;padding:6px;text-align:left}
 .ok{color:#2ecc71}.bad{color:#e74c3c}.sel{background:#333}
 .btn-sm{font-size:12px;padding:4px 10px;margin:0}
</style></head><body>
<div id="banner">● 录制中 <span id="banner-info"></span></div>
<h2 style="margin-top:56px">示教路径录制</h2>
<p>先在 <a href="/" style="color:#6cf">预览窗口</a> 摆好起始姿态，再回来点开始。录完在下方列表里确认标注，失误的删掉。</p>
<div>
 <label>本次标注：<select id="label"><option>success</option><option>fail</option></select></label>
 <button id="start">● 开始录制</button>
 <button id="stop" disabled>■ 停止并保存</button>
</div>
<div id="status">未在录制</div>
<h3>已保存的录制</h3>
<table id="list"><thead><tr><th>名称</th><th>标注</th><th>时长</th><th>夹爪行程</th><th>操作</th></tr></thead><tbody></tbody></table>
<script>
const $=id=>document.getElementById(id);
let recording=false;
async function refresh(){
 try{const r=await fetch('/api/record/status');const s=await r.json();
  recording=!!s.recording;
  $('banner').style.display=s.recording?'block':'none';
  if(s.recording){$('banner-info').textContent=`${s.name||''}  ${s.elapsed_s||0}s  ${s.live_frames||0}帧`;document.title='● 录制中 - 示教';}
  else{document.title='示教录制';}
  $('status').textContent=JSON.stringify(s,null,2);
  $('start').disabled=recording; $('stop').disabled=!recording;
 }catch(e){$('status').textContent='后端不可达';}
}
async function refreshList(){
 try{const r=await fetch('/api/records/list');const d=await r.json();
  const tb=$('list').querySelector('tbody');tb.innerHTML='';
  (d.items||[]).slice().reverse().forEach(it=>{
   const tr=document.createElement('tr');
   tr.innerHTML=`<td>${it.name}</td><td class="${it.label==='success'?'ok':'bad'}">${it.label}</td><td>${it.duration_s||'?'}s</td><td>${JSON.stringify(it.gripper_range||[])}</td><td></td>`;
   const td=tr.lastChild;
   const b1=document.createElement('button');b1.className='btn-sm';b1.textContent=it.label==='success'?'改为fail':'改为success';
   b1.onclick=async()=>{await post('/api/record/label',{name:it.name,label:it.label==='success'?'fail':'success',reason:'网页事后修正'});refreshList();};
   const b2=document.createElement('button');b2.className='btn-sm';b2.textContent='🗑 删除';b2.style.color='#e74c3c';
   b2.onclick=async()=>{if(confirm(`确定删除 ${it.name}？（不可恢复，不会进训练集）`)){await post('/api/record/discard',{name:it.name});refreshList();}};
   td.append(b1,b2);tb.append(tr);
  });
 }catch(e){}}
async function post(url,body){await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})});}
$('start').onclick=async()=>{await post('/api/record/start',{label:$('label').value});refresh();};
$('stop').onclick=async()=>{await post('/api/record/stop',{label:$('label').value});refresh();refreshList();};
refresh();refreshList();setInterval(refresh,1000);setInterval(refreshList,5000);
</script></body></html>'''



# ── 精炼轨迹回放（refined_trajectory.json，夹爪事件阻塞） ──────
REPLAY_TRAJ_FILE = "/mnt/f/hackson/独居女性/robot-app/data/refined_trajectory.json"
_replay_state = {"running": False, "stop": False, "progress": 0.0, "stage": "", "events_done": 0}
# 回放期间的逐路点速度前馈（None=非回放，用 target_velocity 常数）。
# SDK smooth_trajectory_spline 同款方法：每点速度匹配轨迹实际快慢，
# 避免固定限幅造成的"全速冲→急停"锯齿顿挫
_replay_velocities = None


def _request_replay_stop():
    if _replay_state["running"]:
        _replay_state["stop"] = True


def _replay_thread(speed, offset_xy=None):
    """按时间轴推进 target_positions；夹爪事件处暂停等待到位后继续。
    offset_xy=[x,y]（米，基座系）时先对轨迹抓取段做视觉偏移补偿。"""
    global target_velocity, trajectory_running, _replay_velocities
    prev_vel = target_velocity
    try:
        with open(REPLAY_TRAJ_FILE, "r", encoding="utf-8") as f:
            traj = json.load(f)
        wps = traj["waypoints"]
        events = sorted(traj.get("gripper_events", []), key=lambda e: e["t"])
        settle_tol = float(traj.get("grip_settle_tol", 0.05))

        # 视觉偏移补偿：检测到的盒子偏移 → 抓取段整体平移（V2 视觉引导）
        comp_note = "无"
        if offset_xy is not None and np.linalg.norm(np.asarray(offset_xy, dtype=float)) > 1e-4:
            wps, comp_note = _compensate_waypoints(wps, events, offset_xy)
            if wps is None:
                raise RuntimeError(f"视觉偏移补偿失败：{comp_note}")
        print(f"[replay] 开始：{len(wps)} 路点，{len(events)} 夹爪事件，"
              f"速度 x{speed}，视觉补偿: {comp_note}")

        # 切换到位置模式并提高速度限幅以跟上轨迹（轨迹峰值 ≤速度上限）
        if control_mode != "position":
            _set_control_mode("position")
        target_velocity = max(0.3, 0.7 * speed)

        # 先运动到起始路点
        _replay_state["stage"] = "move_to_start"
        with target_lock:
            _cancel_reset_profile()
            target_positions[:] = _clamp_arm_positions(wps[0]["q"])
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 20.0:
            if _replay_state["stop"]:
                return
            arm = np.asarray(_current_arm_positions(), dtype=float)
            if np.max(np.abs(arm - np.asarray(wps[0]["q"], dtype=float))) <= 0.05:
                break
            precise_sleep(0.05)

        # 逐路点跟随（时间轴暂停式：夹爪事件期间冻结时间轴）
        _replay_state["stage"] = "playing"
        ev_idx = 0
        i = 0
        n = len(wps)
        start = time.perf_counter() - wps[0]["t"] / speed
        while i < n - 1 and not _replay_state["stop"]:
            t_elapsed = (time.perf_counter() - start) * speed
            while i < n - 1 and wps[i + 1]["t"] <= t_elapsed:
                i += 1
            v_wp = wps[i].get("v")
            with target_lock:
                _cancel_reset_profile()
                target_positions[:] = _clamp_arm_positions(wps[i]["q"])
                if isinstance(v_wp, (list, tuple)) and len(v_wp) == len(wps[i]["q"]):
                    # 前馈随 speed 缩放：时间轴压缩为 1/speed，实际所需速度也同步缩小，
                    # 否则控制器“冲过头→等下一拍→再冲”，半速回放会重现锯齿顿挫。
                    _replay_velocities = [float(x) * speed for x in v_wp]
                else:
                    _replay_velocities = None
            _replay_state["progress"] = i / max(1, n - 1)

            if ev_idx < len(events) and t_elapsed >= events[ev_idx]["t"]:
                ev = events[ev_idx]
                _replay_state["stage"] = f"gripper_event_{ev_idx + 1}"
                print(f"[replay] 夹爪事件{ev_idx+1}: {ev['from']:.2f} -> {ev['to']:.2f}，等待到位")
                _set_gripper_target(ev["to"], velocity=0.3)
                grip_start = float(_read_gripper_position())
                deadline = time.perf_counter() + 6.0
                settled = False
                stalled = False
                last_pos = grip_start
                last_move_t = time.perf_counter()
                while time.perf_counter() < deadline:
                    if _replay_state["stop"]:
                        break
                    pos = float(_read_gripper_position())
                    if abs(pos - last_pos) > 0.004:
                        last_pos = pos
                        last_move_t = time.perf_counter()
                    elif (time.perf_counter() - last_move_t > 0.7
                          and abs(pos - grip_start) > max(0.05, 0.5 * abs(ev["to"] - grip_start))):
                        # 堵转检测：已朝目标走过半程且位置不再变化 → 夹到物体
                        stalled = True
                        break
                    if abs(pos - float(ev["to"])) <= settle_tol:
                        settled = True
                        break
                    precise_sleep(0.05)
                print(f"[replay] 夹爪事件{ev_idx+1} "
                      f"{'夹紧(堵转) pos=%.2f' % last_pos if stalled else ('到位' if settled else '超时继续')}")
                ev_idx += 1
                _replay_state["events_done"] = ev_idx
                # 冻结期间的时间轴补偿：从当前路点重新计时
                start = time.perf_counter() - wps[i]["t"] / speed
                socketio.emit('replay_progress', {'progress': _replay_state['progress'], 'stage': _replay_state['stage']})
            precise_sleep(0.05)

        # 末段：跟到终点并短暂保持
        _replay_state["stage"] = "finishing"
        _replay_velocities = None
        with target_lock:
            target_positions[:] = _clamp_arm_positions(wps[-1]["q"])
        precise_sleep(1.0)
        _replay_state["progress"] = 1.0
        _replay_state["stage"] = "done"
        print(f"[replay] 完成：进度 100%，夹爪事件 {ev_idx}/{len(events)}")
        socketio.emit('replay_progress', {'progress': 1.0, 'stage': 'done'})
    except Exception as e:
        _replay_state["stage"] = f"error: {e}"
        print(f"[replay] 错误: {e}")
    finally:
        target_velocity = prev_vel
        _replay_velocities = None
        trajectory_running = False
        _replay_state["running"] = False


@app.route('/api/replay/start', methods=['POST'])
def replay_start():
    global trajectory_running
    if _replay_state["running"]:
        return jsonify({"success": False, "error": "回放已在进行"}), 400
    if trajectory_running:
        return jsonify({"success": False, "error": "轨迹执行中，请先停止"}), 400
    if not os.path.isfile(REPLAY_TRAJ_FILE):
        return jsonify({"success": False, "error": f"找不到精炼轨迹 {REPLAY_TRAJ_FILE}"}), 404
    data = request.json or {}
    speed = max(0.1, min(2.0, float(data.get("speed", 1.0))))
    # 可选视觉偏移补偿（米，基座系 XY）：由 vision_trigger_grasp 检测偏差换算而来
    offset_xy = data.get("offset_xy")
    if offset_xy is not None:
        try:
            offset_xy = [float(x) for x in offset_xy[:2]]
        except (TypeError, ValueError, IndexError):
            return jsonify({"success": False, "error": "offset_xy 格式错误"}), 400
        if max(abs(offset_xy[0]), abs(offset_xy[1])) > VISION_MAX_OFFSET_M:
            return jsonify({"success": False,
                            "error": f"offset_xy 超限（±{VISION_MAX_OFFSET_M * 100:.0f}cm）"}), 400
    _replay_state.update({"running": True, "stop": False, "progress": 0.0, "stage": "init", "events_done": 0})
    trajectory_running = True
    threading.Thread(target=_replay_thread, args=(speed, offset_xy), daemon=True).start()
    return jsonify({"success": True, "speed": speed, "offset_xy": offset_xy,
                    "note": "回放已启动，请勿在预览窗口操作关节"})


@app.route('/api/replay/stop', methods=['POST'])
def replay_stop():
    _request_replay_stop()
    with target_lock:
        _cancel_reset_profile()
        target_positions[:] = current_positions.copy()
    return jsonify({"success": True, "note": "已请求停止，机械臂停在当前位置"})


@app.route('/api/replay/status')
def replay_status():
    return jsonify(_replay_state)


# ── 视觉便捷测试（V1：视觉触发固定抓取）──────────────────────────
# 薄代理：子进程跑 vision_trigger_grasp.py（系统 python3 有 cv2，后端 venv 没有）。
# 永远以 dry-run 跑子进程，live 模式由本进程直接启动回放线程（避免子进程回调自己的死锁）。
VISION_GRASP_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__),
    '..', '..', '..', 'robot-app', 'vision_grasp'))
VISION_ROI = [1698, 799, 1998, 1099]  # 2026-08-29 cal 实测（固定点检测中心 1848,949 ±150）
VISION_MAX_OFFSET_M = 0.08  # 视觉偏移补偿上限（米），与 vision_trigger_grasp.MAX_COMP_M 同口径


def _fk_xy(q):
    p = np.asarray(robot.forward_kinematics(np.asarray(q, dtype=float))['position'], dtype=float)
    return p


def _solve_ik_xy(q_init, xyz_target, max_dq=0.25, iters=40, tol=0.004):
    """阻尼最小二乘数值 IK：末端 XYZ → xyz_target（X/Y 平移补偿，Z 锁原高度）。
    3 维约束抑制解在剩余自由度里漂移（只约束 XY 时构型会大幅下沉/翻转）。
    返回新关节角列表，未收敛/超关节修正限幅返回 None。"""
    q0 = np.array(q_init, dtype=float)
    q = q0.copy()
    for _ in range(iters):
        p = _fk_xy(q)
        err = np.array([xyz_target[0] - p[0], xyz_target[1] - p[1], xyz_target[2] - p[2]])
        if np.linalg.norm(err) < tol:
            return q.tolist()
        # 数值雅可比：末端 XYZ 对各关节
        J = np.zeros((3, len(q)))
        eps = 1e-4
        for j in range(len(q)):
            q2 = q.copy()
            q2[j] += eps
            p2 = _fk_xy(q2)
            J[:, j] = [(p2[0] - p[0]) / eps, (p2[1] - p[1]) / eps, (p2[2] - p[2]) / eps]
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(3), err)
        step = np.max(np.abs(dq))
        if step > 0.03:
            dq *= 0.03 / step  # 单次小步，防跳变
        q = q + dq
        if np.max(np.abs(q - q0)) > max_dq:
            return None  # 偏离原轨迹过多，安全拒绝
    p = _fk_xy(q)
    if np.hypot(p[0] - xyz_target[0], p[1] - xyz_target[1]) < 0.01 \
            and abs(p[2] - xyz_target[2]) < 0.01:
        return q.tolist()
    return None


def _compensate_waypoints(wps, events, offset_xy):
    """轨迹抓取段（下降→合爪→抬升）整体平移 offset_xy（基座系 XY，米）。

    段识别：用第一个合爪夹爪事件（to < from）时刻 ±3s 内的 z 最低点锁定
    抓取点（防误锁放置段；无事件时回退全局 z 最低点），向两侧外扩到
    z 回升超过自适应阈值（z_min + max(5cm, 50%×轨迹高差)）。
    锚点每 8 个路点做一次阻尼最小二乘 IK，锚点间 Δq 线性插值（偏移为
    常向量，段内姿态变化平缓，插值误差远小于回放跟随误差）。
    IK 未收敛/关节修正超 0.25rad/段占比超 60% → 整体拒绝补偿（安全第一）。
    返回 (新 wps, 说明) 或 (None, 失败原因)。"""
    if robot is None:
        return None, "robot 未初始化（FK 不可用）"
    off = np.asarray(offset_xy, dtype=float)
    zs = np.array([float(_fk_xy(wp["q"])[2]) for wp in wps])
    # 抓取点：合爪事件附近局部最低点，回退全局最低点
    closes = [e for e in events if e.get("to", 1.0) < e.get("from", 0.0)]
    k_g = int(zs.argmin())
    z_min_g = float(zs[k_g])
    if closes:
        t_c = float(closes[0]["t"])
        cand = [i for i, wp in enumerate(wps) if abs(float(wp["t"]) - t_c) <= 3.0]
        if cand:
            k_g = min(cand, key=lambda i: zs[i])
            z_min_g = float(zs[k_g])
    # 段边界：从抓取点向两侧外扩，z 回升超自适应阈值即停
    z_hi = float(np.quantile(zs, 0.9))
    z_thr = z_min_g + max(0.05, 0.5 * (z_hi - z_min_g))
    i0 = k_g
    while i0 > 0 and zs[i0 - 1] < z_thr:
        i0 -= 1
    i1 = k_g
    while i1 < len(wps) - 1 and zs[i1 + 1] < z_thr:
        i1 += 1
    if (i1 - i0 + 1) > 0.6 * len(wps):
        return None, f"抓取段识别异常（[{i0}:{i1}] 占比过大），拒绝补偿"
    stride = 8
    anchors = list(range(i0, i1 + 1, stride))
    if anchors[-1] != i1:
        anchors.append(i1)
    dqs = []
    for idx in anchors:
        p = _fk_xy(wps[idx]["q"])
        q_new = _solve_ik_xy(wps[idx]["q"], [p[0] + off[0], p[1] + off[1], p[2]])
        if q_new is None:
            return None, f"IK 未收敛（路点 {idx}，偏移过大或接近奇异），拒绝补偿"
        dqs.append(np.array(q_new) - np.array(wps[idx]["q"], dtype=float))
    new_wps = [dict(wp) for wp in wps]
    for ai in range(len(anchors) - 1):
        a, b = anchors[ai], anchors[ai + 1]
        da, db = dqs[ai], dqs[ai + 1]
        for k in range(a, b):  # 半开区间：锚点 b 由下一段/末锚点单独处理，防叠加两次
            r = (k - a) / max(1, b - a)
            dq = da + (db - da) * r
            q = np.array(new_wps[k]["q"], dtype=float) + dq
            wp2 = dict(new_wps[k])
            wp2["q"] = q.tolist()
            new_wps[k] = wp2
    q = np.array(new_wps[i1]["q"], dtype=float) + dqs[-1]  # 末锚点
    wp2 = dict(new_wps[i1])
    wp2["q"] = q.tolist()
    new_wps[i1] = wp2
    note = (f"抓取段 [{i0}:{i1}]（{i1 - i0 + 1}/{len(wps)} 路点）平移 "
            f"({off[0] * 100:+.1f},{off[1] * 100:+.1f})cm")
    print(f"[vision] {note}")
    return new_wps, note


@app.route('/api/vision/run', methods=['POST'])
def vision_run():
    data = request.json or {}
    action = data.get("action", "check")
    if action not in ("check", "probe"):
        return jsonify({"success": False, "error": f"未知 action: {action}"}), 400
    live = bool(data.get("live", False)) and action == "check"
    speed = max(0.1, min(1.0, float(data.get("speed", 1.0))))
    label = str(data.get("label", "")).strip()[:40]
    roi = data.get("roi") or VISION_ROI
    try:
        roi = [float(x) for x in roi[:4]]
    except (TypeError, ValueError):
        roi = VISION_ROI

    cmd = ["python3", "vision_trigger_grasp.py", action,
           "--detector", "color", "--roi"] + [str(x) for x in roi]
    if action == "probe":
        if not label:
            return jsonify({"success": False, "error": "probe 需要 label"}), 400
        cmd += ["--label", label]
    try:
        proc = _subprocess.run(
            cmd, cwd=VISION_GRASP_DIR, capture_output=True, text=True,
            timeout=120,
        )
    except FileNotFoundError:
        return jsonify({"success": False,
                        "error": f"找不到视觉脚本目录或 python3: {VISION_GRASP_DIR}"}), 500
    except _subprocess.TimeoutExpired:
        return jsonify({"success": False, "error": "视觉脚本执行超时（120s）"}), 500
    output = (proc.stdout or "") + (proc.stderr or "")
    result = {"success": proc.returncode == 0, "action": action,
              "output": output.strip()[-2000:], "returncode": proc.returncode}

    # 解析视觉脚本的偏移补偿指令（VISION_OFFSET 行，米，基座系）
    offset_xy = None
    for line in output.splitlines():
        if line.startswith("VISION_OFFSET "):
            try:
                vj = json.loads(line[len("VISION_OFFSET "):])
                offset_xy = [float(vj["x"]), float(vj["y"])]
            except (ValueError, KeyError, TypeError):
                offset_xy = None
            break

    # live：视觉判定通过（check 退出码 0）后由本进程直接启动回放（带偏移补偿）
    if live and proc.returncode == 0:
        if _replay_state["running"] or trajectory_running:
            result["replay"] = {"success": False, "error": "回放已在进行"}
        elif not os.path.isfile(REPLAY_TRAJ_FILE):
            result["replay"] = {"success": False,
                                "error": f"找不到精炼轨迹 {REPLAY_TRAJ_FILE}"}
        else:
            _replay_state.update({"running": True, "stop": False, "progress": 0.0,
                                  "stage": "init", "events_done": 0})
            globals()["trajectory_running"] = True
            threading.Thread(target=_replay_thread, args=(speed, offset_xy),
                             daemon=True).start()
            result["replay"] = {"success": True, "speed": speed, "offset_xy": offset_xy}
    return jsonify(result)


# ── 视觉采样数据管理（表格查看/编辑/删除 + 像素映射重建）──
# 采样可能有坏点/输错标签，前端表格可人工校对；任何编辑/删除后删除 pixel_map.json
# 防止旧映射继续生效，由“重建映射”（vision_trigger_grasp.py calmap）重算。
VISION_SAMPLES_FILE = os.path.join(VISION_GRASP_DIR, "sensitivity_samples.json")
VISION_PIXEL_MAP_FILE = os.path.join(VISION_GRASP_DIR, "pixel_map.json")


def _read_vision_samples():
    if not os.path.isfile(VISION_SAMPLES_FILE):
        return []
    try:
        with open(VISION_SAMPLES_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, list) else []
    except (ValueError, OSError):
        return []


def _write_vision_samples(data):
    with open(VISION_SAMPLES_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def _invalidate_pixel_map():
    try:
        if os.path.isfile(VISION_PIXEL_MAP_FILE):
            os.remove(VISION_PIXEL_MAP_FILE)
    except OSError:
        pass


@app.route('/api/vision/samples', methods=['GET'])
def vision_samples_list():
    pmap = None
    if os.path.isfile(VISION_PIXEL_MAP_FILE):
        try:
            with open(VISION_PIXEL_MAP_FILE, "r", encoding="utf-8") as f:
                pmap = json.load(f)
        except (ValueError, OSError):
            pmap = None
    return jsonify({"success": True, "samples": _read_vision_samples(),
                    "pixel_map": pmap})


@app.route('/api/vision/samples/<int:idx>', methods=['POST'])
def vision_samples_update(idx):
    data = _read_vision_samples()
    if idx < 0 or idx >= len(data):
        return jsonify({"success": False, "error": f"索引越界：{idx}（共 {len(data)} 条）"}), 404
    body = request.json or {}
    rec = data[idx]
    if "label" in body:
        label = str(body["label"]).strip()[:40]
        if not label:
            return jsonify({"success": False, "error": "标签不能为空"}), 400
        rec["label"] = label
    if "mean" in body:
        try:
            mean = [float(body["mean"][0]), float(body["mean"][1])]
        except (TypeError, ValueError, IndexError):
            return jsonify({"success": False, "error": "mean 须为 [x, y] 两个数"}), 400
        rec["mean"] = [round(mean[0], 1), round(mean[1], 1)]
    rec["edited"] = True  # 标记人工修改过，便于校对
    _write_vision_samples(data)
    _invalidate_pixel_map()
    return jsonify({"success": True, "record": rec,
                    "note": "已保存并使像素映射失效，编辑完成后请点重建映射"})


@app.route('/api/vision/samples/<int:idx>', methods=['DELETE'])
def vision_samples_delete(idx):
    data = _read_vision_samples()
    if idx < 0 or idx >= len(data):
        return jsonify({"success": False, "error": f"索引越界：{idx}（共 {len(data)} 条）"}), 404
    removed = data.pop(idx)
    _write_vision_samples(data)
    _invalidate_pixel_map()
    return jsonify({"success": True, "removed": removed,
                    "note": "已删除并使像素映射失效"})


@app.route('/api/vision/samples/calmap', methods=['POST'])
def vision_samples_calmap():
    _invalidate_pixel_map()
    try:
        proc = _subprocess.run(
            ["python3", "vision_trigger_grasp.py", "calmap"],
            cwd=VISION_GRASP_DIR, capture_output=True, text=True, timeout=60)
    except (_subprocess.TimeoutExpired, FileNotFoundError) as e:
        return jsonify({"success": False, "error": f"重建映射失败: {e}"}), 500
    output = (proc.stdout or "") + (proc.stderr or "")
    pmap = None
    if proc.returncode == 0 and os.path.isfile(VISION_PIXEL_MAP_FILE):
        try:
            with open(VISION_PIXEL_MAP_FILE, "r", encoding="utf-8") as f:
                pmap = json.load(f)
        except (ValueError, OSError):
            pmap = None
    return jsonify({"success": proc.returncode == 0,
                    "message": output.strip()[-500:], "pixel_map": pmap})



# ── Script execution state ─────────────────────────────────────────
import subprocess as _subprocess
import contextlib
import io
_script_process = None
_script_name = None
_script_stop_event = threading.Event()  # signal to stop in-process script
_backend_config_path = None  # set at startup
SCRIPTS_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__),
    '..', '..', 'panthera_python', 'scripts'))
SCRIPT_LOG_DIR = os.path.join(os.path.dirname(__file__), 'logs')
SCRIPT_LOG_PATH = os.path.join(SCRIPT_LOG_DIR, 'script_runner.log')
_script_output_lock = threading.Lock()
_SCRIPT_OUTPUT_LIMIT = 1000


def _append_script_output(text):
    if text is None:
        return
    text = str(text)
    if not text:
        return
    os.makedirs(SCRIPT_LOG_DIR, exist_ok=True)
    if not text.endswith('\n'):
        text += '\n'
    with _script_output_lock:
        with open(SCRIPT_LOG_PATH, 'a', encoding='utf-8', errors='replace') as log_file:
            log_file.write(text)
            log_file.flush()


def _clear_script_output():
    os.makedirs(SCRIPT_LOG_DIR, exist_ok=True)
    with _script_output_lock:
        with open(SCRIPT_LOG_PATH, 'w', encoding='utf-8'):
            pass


def _get_script_output():
    if not os.path.exists(SCRIPT_LOG_PATH):
        return []
    with _script_output_lock:
        with open(SCRIPT_LOG_PATH, 'r', encoding='utf-8', errors='replace') as log_file:
            lines = log_file.read().splitlines()
    return lines[-_SCRIPT_OUTPUT_LIMIT:]


def _script_is_running():
    return (_script_process is not None and
        ((isinstance(_script_process, _subprocess.Popen) and _script_process.poll() is None) or
         (isinstance(_script_process, threading.Thread) and _script_process.is_alive())))


def _discover_scripts():
    scripts = []

    if not os.path.isdir(SCRIPTS_DIR):
        return scripts

    for filename in sorted(os.listdir(SCRIPTS_DIR)):
        full_path = os.path.normpath(os.path.join(SCRIPTS_DIR, filename))
        if not os.path.isfile(full_path) or not filename.endswith('.py') or filename.startswith('__'):
            continue
        name = filename[:-3]
        label = name.replace('_', ' ')
        scripts.append({
            'name': name,
            'file': filename,
            'label': label,
        })

    scripts.sort(key=lambda item: item['file'])
    return scripts


def _resolve_script_path(script_name):
    script_name = (script_name or '').replace('\\', '/').lstrip('/')
    if not script_name.endswith('.py'):
        script_name += '.py'

    script_path = os.path.normpath(os.path.join(SCRIPTS_DIR, script_name))
    scripts_root = os.path.abspath(SCRIPTS_DIR)
    abs_script_path = os.path.abspath(script_path)
    if os.path.commonpath([scripts_root, abs_script_path]) != scripts_root:
        return None, script_name
    return script_path, script_name


@app.route('/api/scripts', methods=['GET'])
def list_scripts():
    """List available Python scripts from the SDK scripts directory."""
    is_running = _script_is_running()
    scripts = _discover_scripts()
    return jsonify({
        'scripts': scripts,
        'scripts_count': len(scripts),
        'scripts_dir': os.path.abspath(SCRIPTS_DIR),
        'running': is_running,
        'current_script': _script_name
    })


@app.route('/api/scripts/output', methods=['GET'])
def get_script_output():
    """Get captured output from the current or last script run."""
    return jsonify({
        'success': True,
        'running': _script_is_running(),
        'current_script': _script_name,
        'log_path': os.path.abspath(SCRIPT_LOG_PATH),
        'output': _get_script_output()
    })


@app.route('/api/scripts/log', methods=['GET'])
def get_script_log():
    """Read captured script output directly from the log file."""
    lines = _get_script_output()
    return Response('\n'.join(lines), mimetype='text/plain; charset=utf-8')


@app.route('/api/scripts/run', methods=['POST'])
def run_script():
    """Launch a script.  Demo→subprocess(PantheraSim).  Real→in-process thread."""
    global _script_process, _script_name, script_mode
    data = request.json or {}
    script_name = data.get('script', '')

    if not script_name:
        return jsonify({'success': False, 'error': 'No script specified'}), 400

    script_path, script_name = _resolve_script_path(script_name)
    if not script_path:
        return jsonify({'success': False, 'error': f'Invalid script path: {script_name}'}), 403
    if not os.path.exists(script_path):
        return jsonify({'success': False, 'error': f'Script not found: {script_name}'}), 404
    if robot is None and not demo_mode:
        return jsonify({'success': False, 'error': 'Robot is not connected'}), 409

    _clear_script_output()
    _append_script_output(f"[ScriptRunner] Starting {script_name}")

    # Stop any running script first
    if _script_process is not None:
        if isinstance(_script_process, _subprocess.Popen) and _script_process.poll() is None:
            _script_process.terminate()
            try: _script_process.wait(timeout=2)
            except Exception: _script_process.kill()
        elif isinstance(_script_process, threading.Thread) and _script_process.is_alive():
            # In-process thread — we can't force-kill reliably,
            # but setting script_mode=False lets the control loop resume
            script_mode = False
            _script_process = None
            _script_name = None

    # ── Demo / simulation mode: subprocess with PantheraSim ──────────
    if demo_mode or robot is None:
        runner = os.path.join(os.path.dirname(__file__), 'run_script.py')
        cmd = [sys.executable, runner, '--demo', script_name]
        try:
            log_file = open(SCRIPT_LOG_PATH, 'a', encoding='utf-8', buffering=1)
            _script_process = _subprocess.Popen(
                cmd,
                cwd=os.path.dirname(__file__),
                stdout=log_file,
                stderr=log_file,
                text=True,
                bufsize=1,
            )
            _script_name = script_name
            script_mode = True
            threading.Thread(target=_watch_script_process, args=(_script_process, log_file), daemon=True).start()
            return jsonify({'success': True, 'script': script_name, 'pid': _script_process.pid})
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)}), 500

    # ── Real robot mode: run in-process thread to avoid serial-port
    #     conflicts with the backend's existing Panthera instance ─────
    try:
        _script_name = script_name
        script_mode = True
        thr = threading.Thread(
            target=_run_script_in_thread,
            args=(script_path,),
            daemon=True,
        )
        thr.start()
        _script_process = thr
        return jsonify({'success': True, 'script': script_name, 'thread': True})
    except Exception as e:
        script_mode = False
        return jsonify({'success': False, 'error': str(e)}), 500


def _watch_script_process(process, log_file=None):
    """Clear running state when a subprocess exits."""
    global _script_process, _script_name, script_mode

    try:
        return_code = process.wait()
        _append_script_output(f"[ScriptRunner] Script exited with code {return_code}")
    except Exception as e:
        _append_script_output(f"[ScriptRunner] Process watcher error: {e}")
    finally:
        if log_file:
            try:
                log_file.close()
            except Exception:
                pass
        if _script_process is process:
            _script_process = None
            _script_name = None
            script_mode = False


class _ScriptOutputCapture(io.TextIOBase):
    def __init__(self, stream):
        super().__init__()
        self.stream = stream
        self._buffer = ''

    def writable(self):
        return True

    def write(self, text):
        if not text:
            return 0
        self.stream.write(text)
        self.stream.flush()
        self._buffer += text
        while '\n' in self._buffer:
            line, self._buffer = self._buffer.split('\n', 1)
            _append_script_output(line)
        return len(text)

    def flush(self):
        self.stream.flush()
        if self._buffer:
            _append_script_output(self._buffer)
            self._buffer = ''


class _ScriptStop(Exception):
    """Raised inside a script thread to abort execution."""
    pass


def _run_script_in_thread(script_path):
    """Execute a script in-process using the backend's real robot instance."""
    global script_mode, _script_name, _script_process, robot, current_positions, current_velocities
    global current_torques, current_fk, _script_stop_event
    import types

    _script_stop_event.clear()
    real_robot = robot  # NEVER patch this — control/broadcast loops use it

    # ── joint limits for position clamping ──────────────────────────
    arm_config = _arm_joint_config()
    if arm_config:
        jl_lower = np.array([jc['min'] for jc in arm_config])
        jl_upper = np.array([jc['max'] for jc in arm_config])
    else:
        jl_lower = np.array([-3.14]*6)
        jl_upper = np.array([3.14]*6)

    def _check_stop():
        if _script_stop_event.is_set():
            raise _ScriptStop("Script stopped by user")

    def _safe_pos():
        raw = real_robot.get_current_pos()
        if hasattr(raw, 'tolist'):
            raw = raw.tolist()
        raw = np.asarray(raw[:len(jl_lower)])
        return np.clip(raw, jl_lower + 1e-6, jl_upper - 1e-6)

    def _push_state():
        """Sync broadcast variables from real robot so frontend sees live position."""
        try:
            pos = real_robot.get_current_pos()
            vel = real_robot.get_current_vel()
            tqe = real_robot.get_current_torque()
            current_positions[:] = pos.tolist() if hasattr(pos, 'tolist') else list(pos)
            current_velocities[:] = vel.tolist() if hasattr(vel, 'tolist') else list(vel)
            current_torques[:] = tqe.tolist() if hasattr(tqe, 'tolist') else list(tqe)
            try:
                fk = real_robot.forward_kinematics(pos)
                if fk:
                    current_fk['position'] = fk['position'].tolist() if hasattr(fk['position'], 'tolist') else list(fk['position'])
                    Rmat = fk['rotation']
                    current_fk['rotation'] = Rmat.tolist() if hasattr(Rmat, 'tolist') else [list(row) for row in Rmat]
                    from scipy.spatial.transform import Rotation
                    rot = Rotation.from_matrix(np.array(Rmat))
                    zyx = rot.as_euler('ZYX', degrees=True)
                    current_fk['euler'] = [zyx[1], zyx[2], zyx[0]]
            except Exception:
                pass
        except Exception:
            pass

    # ── Build a wrapper around the real robot ───────────────────────
    # The wrapper adds: stop-check, position-clamping, auto-push.
    # The real robot object is never modified.

    class _ScriptRobotWrapper:
        def __init__(self):
            pass

        def __getattr__(self, name):
            # Forward everything to the real robot
            attr = getattr(real_robot, name)
            if not callable(attr):
                return attr

            def checked_method(*args, **kwargs):
                _check_stop()
                result = attr(*args, **kwargs)
                _check_stop()
                return result

            return checked_method

        # ── position getters (clamped) ──────────────────────────
        def get_current_pos(self):
            _check_stop()
            return _safe_pos()

        # ── control methods (checked + auto-push) ───────────────
        def Joint_Pos_Vel(self, *a, **kw):
            _check_stop()
            r = real_robot.Joint_Pos_Vel(*a, **kw)
            _push_state()
            _check_stop()
            return r

        def pos_vel_tqe_kp_kd(self, *a, **kw):
            _check_stop()
            r = real_robot.pos_vel_tqe_kp_kd(*a, **kw)
            _push_state()
            _check_stop()
            return r

        def moveJ(self, *a, **kw):
            _check_stop()
            r = real_robot.moveJ(*a, **kw)
            _push_state()
            _check_stop()
            return r

        def Joint_Vel(self, *a, **kw):
            _check_stop()
            r = real_robot.Joint_Vel(*a, **kw)
            _push_state()
            return r

        def moveL(self, *a, **kw):
            _check_stop()
            r = real_robot.moveL(*a, **kw)
            _push_state()
            return r

    wrapper = _ScriptRobotWrapper()

    # Make sure script imports see the current panthera_python/scripts tree.
    scripts_path = os.path.abspath(SCRIPTS_DIR)
    if scripts_path in sys.path:
        sys.path.remove(scripts_path)
    sys.path.insert(0, scripts_path)

    # Import a fresh Panthera module so static helpers match the updated scripts.
    for module_name in list(sys.modules):
        if module_name == 'Panthera_lib' or module_name.startswith('Panthera_lib.'):
            del sys.modules[module_name]
    from Panthera_lib.Panthera import Panthera as RealPantheraClass

    # Inject Panthera→wrapper into Panthera_lib

    class _ScriptPantheraProxy:
        def __new__(cls, *args, **kwargs):
            return wrapper

    panthera_lib = types.ModuleType('Panthera_lib')
    panthera_lib.Panthera = _ScriptPantheraProxy
    panthera_lib.TrajectoryRecorder = None
    sys.modules['Panthera_lib'] = panthera_lib
    sys.modules['Panthera_lib.Panthera'] = types.ModuleType('Panthera_lib.Panthera')
    sys.modules['Panthera_lib.Panthera'].Panthera = _ScriptPantheraProxy

    for attr_name in dir(RealPantheraClass):
        if attr_name.startswith('__'):
            continue
        attr = getattr(RealPantheraClass, attr_name)
        if isinstance(RealPantheraClass.__dict__.get(attr_name), staticmethod):
            setattr(_ScriptPantheraProxy, attr_name, staticmethod(attr))

    try:
        with open(script_path) as f:
            code = compile(f.read(), script_path, 'exec')
        exec_globals = {'__name__': '__main__', '__file__': script_path}
        capture = _ScriptOutputCapture(sys.stdout)
        with contextlib.redirect_stdout(capture), contextlib.redirect_stderr(capture):
            exec(code, exec_globals)
        capture.flush()
    except _ScriptStop:
        print("[ScriptRunner] Script stopped by user")
        _append_script_output("[ScriptRunner] Script stopped by user")
    except Exception as e:
        import traceback
        print(f"[ScriptRunner] Error: {e}")
        _append_script_output(f"[ScriptRunner] Error: {e}")
        traceback.print_exc()
        _append_script_output(traceback.format_exc())
    finally:
        script_mode = False
        _script_name = None
        _script_process = None
        print("[ScriptRunner] Script finished")
        _append_script_output("[ScriptRunner] Script finished")


@app.route('/api/scripts/stop', methods=['POST'])
def stop_script():
    """Stop the currently running script (subprocess or in-process thread)."""
    global _script_process, _script_name, script_mode

    if _script_process is None:
        _script_name = None
        script_mode = False
        return jsonify({'success': True, 'status': 'not_running'})

    # Subprocess
    if isinstance(_script_process, _subprocess.Popen):
        if _script_process.poll() is not None:
            _script_process = None; _script_name = None; script_mode = False
            return jsonify({'success': True, 'status': 'not_running'})
        try:
            _script_process.terminate()
            try: _script_process.wait(timeout=3)
            except Exception: _script_process.kill(); _script_process.wait(timeout=2)
            _script_process = None; _script_name = None; script_mode = False
            return jsonify({'success': True, 'status': 'stopped'})
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)}), 500

    # In-process thread — signal stop via event
    if isinstance(_script_process, threading.Thread):
        if not _script_process.is_alive():
            _script_process = None; _script_name = None; script_mode = False
            return jsonify({'success': True, 'status': 'not_running'})
        _script_stop_event.set()
        # Give the thread a moment to react
        _script_process.join(timeout=3)
        if _script_process is not None and _script_process.is_alive():
            return jsonify({'success': False, 'status': 'stopping', 'error': 'Script did not stop within timeout'}), 202
        script_mode = False
        _script_process = None
        _script_name = None
        return jsonify({'success': True, 'status': 'stopped'})

    return jsonify({'success': False, 'error': 'Unknown process type'}), 500


@app.route('/api/script_state', methods=['POST'])
def script_state_update():
    """Receive joint state from external simulation scripts (PantheraSim)."""
    global script_mode, script_state, current_positions, current_velocities
    global current_torques, current_fk
    try:
        data = request.json
        if 'positions' in data:
            script_state['positions'] = data['positions']
            current_positions[:] = data['positions']
        if 'velocities' in data:
            script_state['velocities'] = data['velocities']
            current_velocities[:] = data['velocities']
        if 'torques' in data:
            script_state['torques'] = data['torques']
            current_torques[:] = data['torques']
        if 'fk' in data and data['fk'] is not None:
            current_fk = data['fk']
            script_state['fk'] = data['fk']
        script_mode = True
        return jsonify({"success": True})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 400


@app.route('/api/set_zero', methods=['POST'])
def set_zero():
    """Reset encoder positions to zero (set current position as zero reference)"""
    global robot, current_positions, target_positions

    if robot is None:
        return jsonify({"success": False, "error": "Robot not connected"}), 400

    try:
        # Call the robot's set_reset_zero method
        robot.set_reset_zero()
        robot.motor_send_cmd()

        # Reset our tracked positions to zero
        with target_lock:
            current_positions[:] = [0.0] * len(current_positions)
            target_positions[:] = [0.0] * len(target_positions)

        print("[Set Zero] Encoder positions reset to zero")

        # Broadcast the update to all clients
        socketio.emit('joint_positions', {
            'positions': [0.0] * len(current_positions),
            'velocities': [0.0] * len(current_positions),
            'torques': [0.0] * len(current_positions)
        })

        return jsonify({"success": True, "message": "Encoder positions reset to zero"})

    except Exception as e:
        print(f"[Set Zero] Error: {e}")
        return jsonify({"success": False, "error": str(e)}), 500


@app.route('/api/set_velocity', methods=['POST'])
def set_velocity():
    """Set movement velocity"""
    global target_velocity

    with target_lock:
        target_velocity = request.json.get('velocity', 0.5)

    return jsonify({"success": True, "velocity": target_velocity})


@app.route('/api/set_mode', methods=['POST'])
def set_mode():
    """Set control mode: 'position', 'gravity_comp', 'gravity_friction', 'impedance'"""
    data = request.json
    mode = data.get('mode', 'position')

    if mode not in ['position', 'gravity_comp', 'gravity_friction', 'impedance']:
        return jsonify({"success": False, "error": "Invalid mode"}), 400

    with target_lock:
        _set_control_mode(mode)

    print(f"Control mode changed to: {mode}")
    return jsonify({"success": True, "mode": mode})


@app.route('/api/get_mode', methods=['GET'])
def get_mode():
    """Get current control mode and parameters"""
    return jsonify({
        "mode": control_mode,
        "impedance": {
            "K": impedance_K.tolist(),
            "B": impedance_B.tolist(),
            "target": impedance_target.tolist()
        },
        "gravity_comp": {
            "gain": gravity_gain.tolist(),
            "offset": joint_offset.tolist()
        }
    })


@app.route('/api/set_impedance_params', methods=['POST'])
def set_impedance_params():
    """Set impedance control parameters"""
    global impedance_K, impedance_B, impedance_target

    data = request.json

    with target_lock:
        if 'K' in data:
            impedance_K = np.array(data['K'])
        if 'B' in data:
            impedance_B = np.array(data['B'])
        if 'target' in data:
            impedance_target = np.array(data['target'])

    return jsonify({
        "success": True,
        "K": impedance_K.tolist(),
        "B": impedance_B.tolist(),
        "target": impedance_target.tolist()
    })


@app.route('/api/set_impedance_target', methods=['POST'])
def set_impedance_target():
    """Set impedance control target position"""
    global impedance_target

    data = request.json

    with target_lock:
        if 'target' in data:
            impedance_target = np.array(data['target'])
        elif 'joint' in data and 'position' in data:
            # Set single joint target
            joint_index = data['joint']
            impedance_target[joint_index] = data['position']

    return jsonify({"success": True, "target": impedance_target.tolist()})


# ============== Waypoint API Routes ==============

@app.route('/api/waypoints', methods=['GET'])
def get_waypoints():
    """Get all waypoints"""
    return jsonify({
        "success": True,
        "waypoints": waypoints,
        "max_waypoints": MAX_WAYPOINTS,
        "trajectory_running": trajectory_running
    })


@app.route('/api/waypoints/add', methods=['POST'])
def add_waypoint():
    """Add current position as a new waypoint"""
    global waypoints

    if len(waypoints) >= MAX_WAYPOINTS:
        return jsonify({"success": False, "error": f"Maximum {MAX_WAYPOINTS} waypoints allowed"}), 400

    data = request.json

    if 'positions' in data:
        # Use provided positions
        positions = data['positions']
    else:
        # Use current robot positions
        positions = current_positions.copy()

    duration = data.get('duration', 1.0)

    waypoint = {
        'positions': positions,
        'gripper': _clamp_gripper_position(data.get('gripper', _gripper_target)),
        'duration': duration,
        'settle': max(0.0, min(10.0, float(data.get('settle', 0.8)))),
        'index': len(waypoints)
    }
    waypoints.append(waypoint)

    print(f"Added waypoint {len(waypoints)}: {positions}")
    return jsonify({"success": True, "waypoint": waypoint, "total": len(waypoints)})


@app.route('/api/waypoints/update', methods=['POST'])
def update_waypoint():
    """Update a waypoint"""
    global waypoints

    data = request.json
    index = data.get('index')

    if index is None or index < 0 or index >= len(waypoints):
        return jsonify({"success": False, "error": "Invalid waypoint index"}), 400

    if 'positions' in data:
        waypoints[index]['positions'] = data['positions']
    if 'gripper' in data:
        waypoints[index]['gripper'] = _clamp_gripper_position(data['gripper'])
    if 'duration' in data:
        waypoints[index]['duration'] = data['duration']
    if 'settle' in data:
        waypoints[index]['settle'] = max(0.0, min(10.0, float(data['settle'])))

    return jsonify({"success": True, "waypoint": waypoints[index]})


@app.route('/api/waypoints/delete', methods=['POST'])
def delete_waypoint():
    """Delete a waypoint"""
    global waypoints

    data = request.json
    index = data.get('index')

    if index is None or index < 0 or index >= len(waypoints):
        return jsonify({"success": False, "error": "Invalid waypoint index"}), 400

    deleted = waypoints.pop(index)

    # Update indices
    for i, wp in enumerate(waypoints):
        wp['index'] = i

    return jsonify({"success": True, "deleted": deleted, "total": len(waypoints)})


@app.route('/api/waypoints/clear', methods=['POST'])
def clear_waypoints():
    """Clear all waypoints"""
    global waypoints
    waypoints = []
    return jsonify({"success": True})


@app.route('/api/waypoints/go_to', methods=['POST'])
def go_to_waypoint():
    """Move robot to a specific waypoint"""
    data = request.json
    index = data.get('index')

    if index is None or index < 0 or index >= len(waypoints):
        return jsonify({"success": False, "error": "Invalid waypoint index"}), 400

    positions = waypoints[index]['positions']
    gripper = waypoints[index].get('gripper', _gripper_target)

    switched_to_position = False
    with target_lock:
        switched_to_position = control_mode in ['gravity_comp', 'gravity_friction']
        if switched_to_position:
            _set_control_mode('position')
        _start_smooth_position_move(positions, gripper_target=gripper, with_handoff=switched_to_position)

    if switched_to_position:
        socketio.emit('mode_changed', {'mode': 'position'})

    return jsonify({"success": True, "target": positions})


@app.route('/api/trajectory/run', methods=['POST'])
def run_trajectory():
    """Execute trajectory through all waypoints"""
    global trajectory_running

    if trajectory_running:
        return jsonify({"success": False, "error": "Trajectory already running"}), 400

    if len(waypoints) < 2:
        return jsonify({"success": False, "error": "Need at least 2 waypoints"}), 400

    data = request.json
    control_rate = data.get('control_rate', 100)

    waypoint_list, durations, gripper_list, holds = _prepare_trajectory_from_current_pose()

    # Start trajectory in separate thread
    traj_thread = threading.Thread(
        target=execute_trajectory_thread,
        args=(waypoint_list, durations, gripper_list, holds, control_rate),
        daemon=True
    )
    traj_thread.start()

    return jsonify({"success": True, "message": "Trajectory started"})


@app.route('/api/trajectory/stop', methods=['POST'])
def stop_trajectory():
    """Stop running trajectory"""
    global trajectory_running

    trajectory_running = False
    return jsonify({"success": True})


@app.route('/api/trajectory/status', methods=['GET'])
def trajectory_status():
    """Get trajectory execution status"""
    return jsonify({
        "running": trajectory_running,
        "progress": trajectory_progress,
        "task": active_task_name,
        "task_progress": task_progress
    })


# ============== Reusable Task API ==============

def _safe_task_name(name):
    """Keep task names human-readable while preventing path traversal."""
    name = str(name or '').strip()
    name = re.sub(r'[^\w\- 一-鿿]', '_', name)
    return name[:80].strip(' ._')


def _task_path(name):
    safe = _safe_task_name(name)
    return os.path.join(TASKS_DIR, f'{safe}.json') if safe else None


def _validate_task(task):
    if not isinstance(task, dict):
        raise ValueError('任务格式无效')
    name = _safe_task_name(task.get('name'))
    if not name:
        raise ValueError('请输入任务名称')
    steps = task.get('steps')
    if not isinstance(steps, list) or not steps:
        raise ValueError('任务至少需要一个步骤')
    normalized = []
    for step in steps:
        kind = step.get('type', 'move') if isinstance(step, dict) else ''
        duration = max(0.1, min(300.0, float(step.get('duration', 1.0))))
        settle = max(0.0, min(10.0, float(step.get('settle', 0.5))))
        if kind == 'move':
            positions = step.get('positions')
            if not isinstance(positions, list) or len(positions) < ARM_JOINT_COUNT:
                raise ValueError('移动步骤必须包含6个关节位置')
            positions = _clamp_arm_positions([float(v) for v in positions[:ARM_JOINT_COUNT]])
            gripper = _clamp_gripper_position(step.get('gripper', _gripper_target))
            normalized.append({'type': 'move', 'positions': positions, 'gripper': gripper, 'duration': duration, 'settle': settle})
        elif kind == 'gripper':
            normalized.append({'type': 'gripper', 'position': _clamp_gripper_position(step.get('position', _gripper_target)), 'duration': duration, 'settle': settle})
        else:
            raise ValueError(f'不支持的步骤类型: {kind}')
    return {'name': name, 'version': 1, 'steps': normalized}


def _load_task_file(name):
    path = _task_path(name)
    if not path or not os.path.isfile(path):
        return None
    with open(path, 'r', encoding='utf-8') as f:
        return _validate_task(json.load(f))


def _task_worker(task):
    global active_task_name, task_progress
    try:
        positions = _clamp_arm_positions(_current_arm_positions())
        gripper = _clamp_gripper_position(_read_gripper_position())
        points = [positions]
        grippers = [gripper]
        durations = []
        holds = []
        for step in task['steps']:
            if step['type'] == 'move':
                next_positions = step['positions']
                next_gripper = step['gripper']
            else:
                next_positions = list(points[-1])
                next_gripper = step['position']
            points.append(next_positions)
            grippers.append(next_gripper)
            durations.append(step['duration'])
            holds.append(step.get('settle', 0.5))
        active_task_name = task['name']
        task_progress = 0.0
        execute_trajectory_thread(points, durations, grippers, holds, control_rate=100)
    finally:
        active_task_name = None
        task_progress = 1.0 if not trajectory_running else task_progress


@app.route('/api/tasks', methods=['GET'])
def list_tasks():
    result = []
    for filename in sorted(os.listdir(TASKS_DIR)):
        if not filename.endswith('.json'):
            continue
        try:
            with open(os.path.join(TASKS_DIR, filename), 'r', encoding='utf-8') as f:
                task = _validate_task(json.load(f))
            result.append({'name': task['name'], 'steps': len(task['steps']), 'version': task['version']})
        except Exception:
            continue
    return jsonify({'success': True, 'tasks': result, 'active': active_task_name, 'running': trajectory_running})


@app.route('/api/tasks/<path:name>', methods=['GET'])
def get_task(name):
    task = _load_task_file(name)
    if task is None:
        return jsonify({'success': False, 'error': '任务不存在'}), 404
    return jsonify({'success': True, 'task': task})


@app.route('/api/tasks', methods=['POST'])
def save_task():
    try:
        data = request.get_json(silent=True) or {}
        # Accept either a complete task or the currently displayed waypoints.
        if 'steps' not in data:
            data['steps'] = [
                {'type': 'move', 'positions': wp['positions'], 'gripper': wp.get('gripper', _gripper_target), 'duration': wp.get('duration', 1.0), 'settle': wp.get('settle', 0.5)}
                for wp in waypoints
            ]
        task = _validate_task(data)
        path = _task_path(task['name'])
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(task, f, ensure_ascii=False, indent=2)
        return jsonify({'success': True, 'task': task})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 400


@app.route('/api/tasks/<path:name>', methods=['DELETE'])
def delete_task(name):
    path = _task_path(name)
    if not path or not os.path.isfile(path):
        return jsonify({'success': False, 'error': '任务不存在'}), 404
    os.remove(path)
    return jsonify({'success': True})


@app.route('/api/tasks/<path:name>/run', methods=['POST'])
def run_task(name):
    global trajectory_running
    if trajectory_running:
        return jsonify({'success': False, 'error': '已有动作正在执行'}), 409
    task = _load_task_file(name)
    if task is None:
        return jsonify({'success': False, 'error': '任务不存在'}), 404
    if not demo_mode and robot is None:
        return jsonify({'success': False, 'error': '机器人未连接'}), 400
    with target_lock:
        if control_mode in ['gravity_comp', 'gravity_friction']:
            _set_control_mode('position')
    threading.Thread(target=_task_worker, args=(task,), daemon=True).start()
    return jsonify({'success': True, 'task': task['name']})


@app.route('/api/tasks/<path:name>/load', methods=['POST'])
def load_task_into_waypoints(name):
    global waypoints
    task = _load_task_file(name)
    if task is None:
        return jsonify({'success': False, 'error': '任务不存在'}), 404
    loaded = []
    for i, step in enumerate(task['steps']):
        if step['type'] == 'move':
            loaded.append({'positions': step['positions'], 'gripper': step['gripper'], 'duration': step['duration'], 'settle': step.get('settle', 0.5), 'index': i})
        elif loaded:
            # Represent a gripper-only step as a waypoint at the previous arm pose.
            loaded.append({'positions': list(loaded[-1]['positions']), 'gripper': step['position'], 'duration': step['duration'], 'settle': step.get('settle', 0.5), 'index': i})
    if len(loaded) > MAX_WAYPOINTS:
        return jsonify({'success': False, 'error': f'任务步骤超过当前轨迹点上限 {MAX_WAYPOINTS}'}), 400
    waypoints = loaded
    socketio.emit('waypoints_updated', {'waypoints': waypoints})
    return jsonify({'success': True, 'task': task, 'waypoints': waypoints})


@app.route('/api/tasks/stop', methods=['POST'])
def stop_task():
    global trajectory_running
    trajectory_running = False
    return jsonify({'success': True})


@app.route('/api/tasks/status', methods=['GET'])
def task_status():
    return jsonify({'success': True, 'running': trajectory_running, 'task': active_task_name, 'progress': trajectory_progress})


# ============== WebSocket Events ==============

@socketio.on('connect')
def handle_connect():
    """Handle new WebSocket connection"""
    connected_clients.add(request.sid)
    print(f"Client connected: {request.sid} (total: {len(connected_clients)})")

    # Send initial config
    emit('config', {
        "robot_name": robot_config['robot']['name'] if robot_config else "Unknown",
        "joints": JOINT_CONFIG,
        "demo_mode": demo_mode,
        "connected": robot is not None or demo_mode,
        "control_mode": control_mode,
        "end_effector_link": robot_config.get('urdf', {}).get('end_effector_link') if robot_config else None,
        "gripper_limits": _gripper_limits_list(),
        "impedance": {
            "K": impedance_K.tolist(),
            "B": impedance_B.tolist(),
            "target": impedance_target.tolist()
        },
        "end_effector_offset": END_EFFECTOR_OFFSET
    })


@socketio.on('disconnect')
def handle_disconnect():
    """Handle WebSocket disconnection"""
    connected_clients.discard(request.sid)
    print(f"Client disconnected: {request.sid} (total: {len(connected_clients)})")


@socketio.on('move_joint')
def handle_move_joint(data):
    """Handle joint movement command via WebSocket"""
    global target_positions

    joint_index = data.get('joint')
    position = data.get('position')

    if joint_index is not None and position is not None:
        joint_index = int(joint_index)
        if _is_gripper_index(joint_index):
            _cancel_reset_profile()
            _set_gripper_target(position)
            return

        arm_config = _arm_joint_config()
        if arm_config and joint_index < len(arm_config):
            jc = arm_config[joint_index]
            position = max(jc['min'], min(jc['max'], position))

        if joint_index < len(target_positions):
            with target_lock:
                _cancel_reset_profile()
                target_positions[joint_index] = position


@socketio.on('move_all')
def handle_move_all(data):
    """Handle all joints movement via WebSocket"""
    global target_positions, target_velocity

    with target_lock:
        gripper_velocity = data.get('velocity', 0.3)
        _cancel_reset_profile()
        if 'positions' in data:
            positions = list(data['positions'])
            target_positions[:] = _clamp_arm_positions(positions)
            gripper_cfg = _gripper_config()
            if gripper_cfg and len(positions) > gripper_cfg["index"]:
                _set_gripper_target(positions[gripper_cfg["index"]], velocity=gripper_velocity)

        if 'gripper' in data:
            _set_gripper_target(data['gripper'], velocity=gripper_velocity)

        if 'velocity' in data:
            target_velocity = data['velocity']


@socketio.on('home')
def handle_home():
    """Handle home command via WebSocket"""
    with target_lock:
        _start_smooth_position_reset()


@socketio.on('reset_all')
def handle_reset_all():
    """Handle smooth reset command via WebSocket"""
    with target_lock:
        _start_smooth_position_reset()


@socketio.on('stop')
def handle_stop():
    """Handle stop command via WebSocket"""
    global target_positions

    with target_lock:
        _cancel_reset_profile()
        target_positions[:] = current_positions.copy()


@socketio.on('set_zero')
def handle_set_zero():
    """Handle set zero command via WebSocket - reset encoder positions to zero"""
    global robot, current_positions, target_positions

    if robot is None:
        emit('set_zero_result', {'success': False, 'error': 'Robot not connected'})
        return

    try:
        # Call the robot's set_reset_zero method
        robot.set_reset_zero()
        robot.motor_send_cmd()

        # Reset our tracked positions to zero
        with target_lock:
            _cancel_reset_profile()
            current_positions[:] = [0.0] * len(current_positions)
            target_positions[:] = [0.0] * len(target_positions)

        print("[Set Zero] Encoder positions reset to zero")

        # Broadcast the update to all clients
        socketio.emit('joint_positions', {
            'positions': [0.0] * len(current_positions),
            'velocities': [0.0] * len(current_positions),
            'torques': [0.0] * len(current_positions)
        })

        emit('set_zero_result', {'success': True, 'message': 'Encoder positions reset to zero'})

    except Exception as e:
        print(f"[Set Zero] Error: {e}")
        emit('set_zero_result', {'success': False, 'error': str(e)})


@socketio.on('set_mode')
def handle_set_mode(data):
    """Handle control mode change via WebSocket"""
    mode = data.get('mode', 'position')

    if mode in ['position', 'gravity_comp', 'gravity_friction', 'impedance']:
        with target_lock:
            _set_control_mode(mode)

        print(f"Control mode changed to: {mode}")

        # Broadcast mode change to all clients
        socketio.emit('mode_changed', {'mode': mode})


@socketio.on('set_impedance_target')
def handle_set_impedance_target(data):
    """Handle impedance target change via WebSocket"""
    global impedance_target

    with target_lock:
        if 'target' in data:
            impedance_target = np.array(data['target'])
        elif 'joint' in data and 'position' in data:
            joint_index = data['joint']
            impedance_target[joint_index] = data['position']


@socketio.on('set_impedance_params')
def handle_set_impedance_params(data):
    """Handle impedance parameter change via WebSocket"""
    global impedance_K, impedance_B

    with target_lock:
        if 'K' in data:
            impedance_K = np.array(data['K'])
        if 'B' in data:
            impedance_B = np.array(data['B'])


@socketio.on('add_waypoint')
def handle_add_waypoint(data):
    """Add waypoint via WebSocket"""
    global waypoints

    if len(waypoints) >= MAX_WAYPOINTS:
        emit('waypoint_error', {'error': f'Maximum {MAX_WAYPOINTS} waypoints allowed'})
        return

    if 'positions' in data:
        positions = data['positions']
    else:
        positions = current_positions.copy()

    duration = data.get('duration', 1.0)

    waypoint = {
        'positions': positions,
        'gripper': _clamp_gripper_position(data.get('gripper', _gripper_target)),
        'duration': duration,
        'settle': max(0.0, min(10.0, float(data.get('settle', 0.8)))),
        'index': len(waypoints)
    }
    waypoints.append(waypoint)

    # Broadcast to all clients
    socketio.emit('waypoints_updated', {'waypoints': waypoints})


@socketio.on('delete_waypoint')
def handle_delete_waypoint(data):
    """Delete waypoint via WebSocket"""
    global waypoints

    index = data.get('index')
    if index is None or index < 0 or index >= len(waypoints):
        emit('waypoint_error', {'error': 'Invalid waypoint index'})
        return

    waypoints.pop(index)

    # Update indices
    for i, wp in enumerate(waypoints):
        wp['index'] = i

    socketio.emit('waypoints_updated', {'waypoints': waypoints})


@socketio.on('clear_waypoints')
def handle_clear_waypoints():
    """Clear all waypoints via WebSocket"""
    global waypoints
    waypoints = []
    socketio.emit('waypoints_updated', {'waypoints': waypoints})


@socketio.on('update_waypoint_duration')
def handle_update_waypoint_duration(data):
    """Update waypoint duration via WebSocket"""
    global waypoints

    index = data.get('index')
    duration = data.get('duration')

    if index is None or index < 0 or index >= len(waypoints):
        return

    if duration is not None:
        waypoints[index]['duration'] = duration
        socketio.emit('waypoints_updated', {'waypoints': waypoints})


@socketio.on('update_waypoint_settle')
def handle_update_waypoint_settle(data):
    """Update the post-arrival settle time for a waypoint."""
    global waypoints
    index = data.get('index')
    if index is None or index < 0 or index >= len(waypoints):
        return
    try:
        waypoints[index]['settle'] = max(0.0, min(10.0, float(data.get('settle', 0.8))))
        socketio.emit('waypoints_updated', {'waypoints': waypoints})
    except (TypeError, ValueError):
        return


@socketio.on('run_trajectory')
def handle_run_trajectory(data):
    """Run trajectory via WebSocket"""
    global trajectory_running

    if trajectory_running:
        emit('trajectory_error', {'error': 'Trajectory already running'})
        return

    if len(waypoints) < 2:
        emit('trajectory_error', {'error': 'Need at least 2 waypoints'})
        return

    control_rate = data.get('control_rate', 100)

    waypoint_list, durations, gripper_list, holds = _prepare_trajectory_from_current_pose()

    traj_thread = threading.Thread(
        target=execute_trajectory_thread,
        args=(waypoint_list, durations, gripper_list, holds, control_rate),
        daemon=True
    )
    traj_thread.start()


@socketio.on('stop_trajectory')
def handle_stop_trajectory():
    """Stop trajectory via WebSocket"""
    global trajectory_running
    trajectory_running = False


@socketio.on('go_to_waypoint')
def handle_go_to_waypoint(data):
    """Move to waypoint via WebSocket"""
    index = data.get('index')
    if index is None or index < 0 or index >= len(waypoints):
        return

    positions = waypoints[index]['positions']
    gripper = waypoints[index].get('gripper', _gripper_target)

    switched_to_position = False
    with target_lock:
        switched_to_position = control_mode in ['gravity_comp', 'gravity_friction']
        if switched_to_position:
            _set_control_mode('position')
        _start_smooth_position_move(positions, gripper_target=gripper, with_handoff=switched_to_position)

    if switched_to_position:
        socketio.emit('mode_changed', {'mode': 'position'})


# ============== Keyboard Control ==============

_key_state = {}
_key_lock = threading.Lock()
_valid_keys = {'w','s','a','d','q','e','i','k','j','l','u','o','z','x'}
_cmd_queue = []
_cmd_queue_lock = threading.Lock()
KEY_STEP = 0.005      # rad per control cycle at 200Hz  (~1 rad/s max)
GRIPPER_STEP = 0.02   # rad per control cycle
_gripper_target = 0.0


@socketio.on('key_down')
def handle_key_down(data):
    key = data.get('key', '').lower()
    if key in _valid_keys:
        with _key_lock:
            _key_state[key] = True


@socketio.on('key_up')
def handle_key_up(data):
    key = data.get('key', '').lower()
    if key in _valid_keys:
        with _key_lock:
            _key_state[key] = False


@socketio.on('command')
def handle_command(data):
    action = data.get('action', '')
    if action in ('home', 'zero_ft', 'print_pose'):
        with _cmd_queue_lock:
            _cmd_queue.append(action)


def _process_keyboard():
    """Apply active keyboard keys to control targets.
    Call from control loop (200 Hz) when in position or impedance mode."""
    global target_positions, impedance_target, _gripper_target, current_positions

    with _key_lock:
        keys = dict(_key_state)

    with _cmd_queue_lock:
        cmds = list(_cmd_queue)
        _cmd_queue.clear()

    for cmd in cmds:
        if cmd == 'home':
            _start_smooth_position_reset()
            impedance_target[:] = [0.0] * 6
            _gripper_target = 0.0
        elif cmd == 'print_pose':
            print(f"[KB] pos={current_positions}  imp_target={impedance_target}")

    if not keys:
        return

    if control_mode == 'position':
        with target_lock:
            t = target_positions
            if keys.get('w'): t[0] += KEY_STEP
            if keys.get('s'): t[0] -= KEY_STEP
            if keys.get('a'): t[1] += KEY_STEP
            if keys.get('d'): t[1] -= KEY_STEP
            if keys.get('q'): t[2] += KEY_STEP
            if keys.get('e'): t[2] -= KEY_STEP
            if keys.get('i'): t[3] += KEY_STEP
            if keys.get('k'): t[3] -= KEY_STEP
            if keys.get('j'): t[4] += KEY_STEP
            if keys.get('l'): t[4] -= KEY_STEP
            if keys.get('u'): t[5] += KEY_STEP
            if keys.get('o'): t[5] -= KEY_STEP
            arm_config = _arm_joint_config()
            if arm_config:
                for i in range(min(len(t), len(arm_config))):
                    t[i] = max(arm_config[i]['min'], min(arm_config[i]['max'], t[i]))

    elif control_mode == 'impedance':
        t = impedance_target
        if keys.get('w'): t[0] += KEY_STEP
        if keys.get('s'): t[0] -= KEY_STEP
        if keys.get('a'): t[1] += KEY_STEP
        if keys.get('d'): t[1] -= KEY_STEP
        if keys.get('q'): t[2] += KEY_STEP
        if keys.get('e'): t[2] -= KEY_STEP
        if keys.get('i'): t[3] += KEY_STEP
        if keys.get('k'): t[3] -= KEY_STEP
        if keys.get('j'): t[4] += KEY_STEP
        if keys.get('l'): t[4] -= KEY_STEP
        if keys.get('u'): t[5] += KEY_STEP
        if keys.get('o'): t[5] -= KEY_STEP
        arm_config = _arm_joint_config()
        if arm_config:
            for i in range(min(len(t), len(arm_config))):
                t[i] = max(arm_config[i]['min'], min(arm_config[i]['max'], t[i]))

    # Gripper keys (work in any mode)
    if keys.get('z') or keys.get('x'):
        if keys.get('z'): _gripper_target += GRIPPER_STEP
        if keys.get('x'): _gripper_target -= GRIPPER_STEP
        # Clamp
        gl = _gripper_config()
        if gl:
            _gripper_target = max(gl['min'], min(gl['max'], _gripper_target))
        if robot is not None and not demo_mode:
            try:
                robot.gripper_control(_gripper_target, 0.3, 0.5)
            except Exception:
                pass


# ============== Main ==============

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Digital Twin Backend Server')
    parser.add_argument('--config', '-c', type=str,
                        default=LOCAL_CONFIG_PATH,
                        help='Path to robot config YAML')
    parser.add_argument('--demo', action='store_true',
                        help='Run in demo mode without robot connection')
    parser.add_argument('--port', '-p', type=int, default=5000,
                        help='Server port (default: 5000)')
    args = parser.parse_args()

    demo_mode = args.demo

    print("=" * 50)
    print("Digital Twin Backend Server")
    print("=" * 50)

    # Load configuration
    config_path = args.config
    if not os.path.isabs(config_path):
        config_path = os.path.join(os.getcwd(), config_path)
    _backend_config_path = config_path

    print(f"\n1. Loading configuration from: {config_path}")
    try:
        load_config(config_path)
    except Exception as e:
        print(f"Failed to load config: {e}")
        print("Using default joint configuration")
        JOINT_CONFIG = [
            {"name": f"Joint {i+1}", "index": i, "min": -3.14, "max": 3.14}
            for i in range(6)
        ]

    # Initialize robot
    print("\n2. Initializing robot...")
    init_robot(config_path)

    # Start control loops
    print("\n3. Starting control loops...")
    start_loops()

    # Start server
    print(f"\n4. Starting server on port {args.port}...")
    print(f"\n   Backend API: http://localhost:{args.port}")
    print(f"   WebSocket:   ws://localhost:{args.port}")
    print("=" * 50)

    socketio.run(app, host='0.0.0.0', port=args.port, debug=False,
                 allow_unsafe_werkzeug=True)
