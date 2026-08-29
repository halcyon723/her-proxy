#!/usr/bin/env python3
"""
Digital Twin Backend — Cartesian Impedance Control + Force-Torque Sensing

基于 7_keyboard_ee_control_cart_impedance_polar.py，增加 Flask+SocketIO 后端，
将机器人状态（关节角、末端位姿、外力估计）通过 WebSocket 广播给 Three.js 前端 3D 可视化。

架构：三线程 + Flask
  控制线程 (1500Hz) ── FK + Jacobian + 笛卡尔阻抗 + 外力估计 → 关节力矩
  广播线程 (30Hz)   ── 通过 WebSocket 向浏览器推送 robot_state
  主线程            ── 键盘控制（极坐标）来自 pygame 窗口 和/或 浏览器前端

控制律：
  τ = J^T(JJ^T+λ²I)^{-1}(K*e_x - B*ẋ) + G(q) + C(q,dq)*dq + f(dq) - B_joint*dq

外力估计：
  τ_ext = τ_measured - (G(q) + C(q,dq)*dq + f(dq))
  F_ext = pinv(J^T) @ τ_ext → LPF

Usage:
  cd Panthera_digital_twin-main/backend
  python app_cartesian.py
  # 另一个终端: cd ../frontend && npm run dev
  # 浏览器: http://localhost:3000 → Connect → 用键盘控制
"""
import sys
import os
import time
import threading
import logging
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import pinocchio as pin

# ─── pygame (optional) ────────────────────────────────────────
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    pygame = None

# ─── Flask + SocketIO ──────────────────────────────────────────
from flask import Flask, jsonify, send_from_directory
from flask_socketio import SocketIO, emit
from flask_cors import CORS

# Suppress Flask request logs
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# ─── SDK 路径 ──────────────────────────────────────────────────
SDK_PATH = os.path.join(os.path.dirname(__file__), '..', '..', 'panthera_python')
sys.path.append(SDK_PATH)
sys.path.append(os.path.join(SDK_PATH, 'scripts'))
from Panthera_lib.Panthera import Panthera

# ─── Flask app ─────────────────────────────────────────────────
app = Flask(__name__, static_folder='../frontend/dist', static_url_path='')
CORS(app, origins="*")
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Serve Panthera-HT URDF + meshes as "arm_description"
ARM_DESCRIPTION_PATH = os.path.join(os.path.dirname(__file__), '..', '..', 'panthera_python', 'xlerobot-HT_description_gripper')

# ─── 控制参数（复用 script 7）──────────────────────────────────
POS_STEP        = 0.001
ROT_STEP        = 0.2
POLAR_ANGLE_MAX = np.radians(0.5)
CTRL_FREQ_IMP   = 2000
CTRL_FREQ_KEY   = 200
BROADCAST_FREQ  = 30

MAX_TORQUE = [21.0, 36.0, 36.0, 30.0, 10.0, 10.0]
JOINT_VEL  = [0.5] * 6

K_POS = np.array([20.0, 20.0, 60.0])
K_ROT = np.array([20.0, 40.0, 40.0])
K_CART = np.concatenate([K_POS, K_ROT])
K_CART_DEFAULT = K_CART.copy()

B_POS = np.array([0.0, 0.0, 0.0])
B_ROT = np.array([0.0, 0.0, 0.0])
B_CART = np.concatenate([B_POS, B_ROT])

LAMBDA_DAMP = 0.05
TAU_LIMIT = np.array([20.0, 30.0, 30.0, 20.0, 10.0, 10.0])
JOINT_DAMPING = np.array([1.2, 2, 2, 2, 0.8, 0.6])

IMP_Fc         = np.array([0.05, 0.05, 0.05, 0.05, 0.01, 0.01])
IMP_Fv         = np.array([0.03, 0.03, 0.03, 0.03, 0.01, 0.01])
IMP_VEL_THRESH = 0.02

# 力矩输出低通滤波截止频率 (Hz)
TOR_LPF_CUTOFF = 500.0

TOOL_OFFSET = np.array([0.165, 0.0, 0.0])

GRIPPER_STEP        = 0.02
GRIPPER_KP          = 8.0
GRIPPER_KD          = 0.5
GRIPPER_MIN_DEFAULT = 0.0
GRIPPER_MAX_DEFAULT = 1.6

FT_CUTOFF_FREQ = 5.0
FT_WARN_LO = 1.0
FT_WARN_HI = 5.0

JOINT_LIMIT_MARGIN = 0.1
JOINT_LIMIT_WARN_DURATION = 1.0

HOME_POS       = [0.24, 0.0, 0.15]
HOME_ROT_EULER = (0.0, np.pi / 2, 0.0)

SERVER_PORT = 5000


# ─── 数学工具 ──────────────────────────────────────────────────

def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def orientation_error_axis_angle(R_des, R_cur):
    R_err = R_cur.T @ R_des
    rot = Rot.from_matrix(R_err)
    rotvec = rot.as_rotvec()
    return R_cur @ rotvec


def _build_pin_q(robot, joint_angles):
    q = np.zeros(robot.model.nq)
    for i, name in enumerate(robot.joint_names):
        jid = robot.model.getJointId(name)
        q[robot.model.joints[jid].idx_q] = joint_angles[i]
    return q


def compute_fk_and_jacobian(robot, data, joint_angles):
    q = _build_pin_q(robot, joint_angles)
    pin.computeJointJacobians(robot.model, data, q)

    last_jid = robot.model.getJointId(robot.joint_names[-1])
    T_last = data.oMi[last_jid]
    R_last = T_last.rotation
    p_last = T_last.translation

    r_world = R_last @ TOOL_OFFSET
    p_tcp = (p_last + r_world).copy()
    R_tcp = R_last.copy()

    J_full = pin.getJointJacobian(
        robot.model, data, last_jid,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    )

    J_tcp = J_full.copy()
    J_tcp[:3, :] -= skew(r_world) @ J_full[3:, :]

    cols = [robot.model.joints[robot.model.getJointId(n)].idx_v
            for n in robot.joint_names]
    J6 = J_tcp[:, cols]

    return p_tcp, R_tcp, J6


def print_pose(pos, rot, label="末端位姿"):
    euler = Rot.from_matrix(rot).as_euler('xyz', degrees=True)
    print(f"\n[{label}]")
    print(f"  位置 (m):  X={pos[0]:+.4f}  Y={pos[1]:+.4f}  Z={pos[2]:+.4f}")
    print(f"  姿态 (°):  Roll={euler[0]:+.1f}  Pitch={euler[1]:+.1f}  Yaw={euler[2]:+.1f}")


# ─── Flask 路由 ────────────────────────────────────────────────

@app.route('/')
def index():
    return send_from_directory(app.static_folder, 'index.html')


@app.route('/arm_description/<path:filename>')
def serve_arm_description(filename):
    return send_from_directory(ARM_DESCRIPTION_PATH, filename)


@app.route('/api/arm_description_files')
def get_arm_description_files():
    files = {}
    def scan_directory(path, prefix=''):
        for entry in os.scandir(path):
            rel_path = os.path.join(prefix, entry.name) if prefix else entry.name
            if entry.is_file():
                files[rel_path] = f'/arm_description/{rel_path}'
            elif entry.is_dir():
                scan_directory(entry.path, rel_path)
    try:
        scan_directory(ARM_DESCRIPTION_PATH)
        return jsonify({"success": True, "files": files, "base_url": "/arm_description"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


# These will be set after robot init
_gripper_limits = [0.0, 1.6]  # [min, max] in motor rad, updated in main()
_broadcast_state = {
    'positions': [0.0] * 6,
    'velocities': [0.0] * 6,
    'torques': [0.0] * 6,
    'ee_position': [0.0, 0.0, 0.0],
    'ee_euler': [0.0, 0.0, 0.0],
    'cartesian_error': [0.0] * 6,
    'external_wrench': [0.0] * 6,
    'target_ee_position': [0.0, 0.0, 0.0],
    'control_torques': [0.0] * 6,
    'timestamp': 0.0
}
_broadcast_lock = threading.Lock()


@app.route('/api/config')
def get_config():
    return jsonify({
        "robot_name": "Panthera-HT (Cartesian Impedance)",
        "joints": [
            {"name": f"joint{i+1}", "index": i, "min": -3.14, "max": 3.14}
            for i in range(6)
        ],
        "demo_mode": False,
        "connected": True,
        "control_mode": "cartesian_impedance",
        "end_effector_offset": float(np.linalg.norm(TOOL_OFFSET))
    })


@app.route('/api/status')
def get_status():
    with _broadcast_lock:
        state = _broadcast_state.copy()
    return jsonify(state)


# ─── WebSocket 事件 ────────────────────────────────────────────

connected_clients = set()

# Web keyboard state (from browser keydown/keyup)
_web_key_state = {}
_web_key_lock = threading.Lock()
_command_queue = []
_command_queue_lock = threading.Lock()
VALID_WEB_KEYS = {'w', 's', 'a', 'd', 'q', 'e', 'i', 'k', 'j', 'l', 'u', 'o', 'z', 'x'}


@socketio.on('connect')
def handle_connect():
    from flask import request as flask_request
    connected_clients.add(flask_request.sid)
    print(f"[WS] Client connected: {flask_request.sid} (total: {len(connected_clients)})")
    emit('config', {
        "robot_name": "Panthera-HT (Cartesian Impedance)",
        "joints": [
            {"name": f"joint{i+1}", "index": i, "min": -3.14, "max": 3.14}
            for i in range(6)
        ],
        "demo_mode": False,
        "connected": True,
        "control_mode": "cartesian_impedance",
        "end_effector_offset": float(np.linalg.norm(TOOL_OFFSET)),
        "impedance_kp": K_CART.tolist(),
        "gripper_limits": _gripper_limits
    })


@socketio.on('disconnect')
def handle_disconnect():
    from flask import request as flask_request
    connected_clients.discard(flask_request.sid)
    print(f"[WS] Client disconnected: {flask_request.sid} (total: {len(connected_clients)})")
    # Clear web key state when last client disconnects
    if not connected_clients:
        with _web_key_lock:
            _web_key_state.clear()


@socketio.on('key_down')
def handle_key_down(data):
    key = data.get('key', '').lower()
    if key in VALID_WEB_KEYS:
        with _web_key_lock:
            _web_key_state[key] = True


@socketio.on('key_up')
def handle_key_up(data):
    key = data.get('key', '').lower()
    if key in VALID_WEB_KEYS:
        with _web_key_lock:
            _web_key_state[key] = False


@socketio.on('command')
def handle_command(data):
    action = data.get('action', '')
    if action in ('home', 'zero_ft', 'print_pose'):
        with _command_queue_lock:
            _command_queue.append(action)


@socketio.on('set_impedance_kp')
def handle_set_impedance_kp(data):
    kp = data.get('kp')
    if isinstance(kp, list) and len(kp) == 6:
        kp = [max(0.0, min(200.0, float(v))) for v in kp]
        K_CART[:] = kp
        print(f"[WS] Impedance Kp updated: {kp}")


@socketio.on('reset_impedance_kp')
def handle_reset_impedance_kp(data=None):
    K_CART[:] = K_CART_DEFAULT
    print(f"[WS] Impedance Kp reset to default: {K_CART_DEFAULT.tolist()}")
    emit('impedance_kp_reset', {'kp': K_CART_DEFAULT.tolist()})


# ─── 统一按键查询 ────────────────────────────────────────────

# Pygame key name → pygame constant mapping (built lazily)
_PYGAME_KEY_MAP = None

def _build_pygame_key_map():
    global _PYGAME_KEY_MAP
    if not PYGAME_AVAILABLE:
        _PYGAME_KEY_MAP = {}
        return
    _PYGAME_KEY_MAP = {
        'w': pygame.K_w, 's': pygame.K_s, 'a': pygame.K_a, 'd': pygame.K_d,
        'q': pygame.K_q, 'e': pygame.K_e, 'i': pygame.K_i, 'k': pygame.K_k,
        'j': pygame.K_j, 'l': pygame.K_l, 'u': pygame.K_u, 'o': pygame.K_o,
        'z': pygame.K_z, 'x': pygame.K_x,
    }


def _is_key_active(pygame_keys, key_name):
    """Check if key is pressed from either pygame or web browser."""
    if _PYGAME_KEY_MAP is None:
        _build_pygame_key_map()
    # Check pygame
    if pygame_keys is not None and key_name in _PYGAME_KEY_MAP:
        if pygame_keys[_PYGAME_KEY_MAP[key_name]]:
            return True
    # Check web
    with _web_key_lock:
        return _web_key_state.get(key_name, False)


# ─── 主程序 ────────────────────────────────────────────────────

def main():
    print("""
╔══════════════════════════════════════════════════════════════╗
║  笛卡尔阻抗控制 + 数字孪生 (Digital Twin + Cartesian Imp.)  ║
╠══════════════════════════════════════════════════════════════╣
║  位置控制 (水平极坐标):                                      ║
║    W / S   →  径向 远离 / 靠近 基座                          ║
║    A / D   →  绕基座Z轴 逆时针 / 顺时针（朝向同步）          ║
║    Q / E   →  Z轴 上升 / 下降                               ║
╠══════════════════════════════════════════════════════════════╣
║  姿态控制 (末端坐标系):                                      ║
║    I / K   →  绕Y轴 俯仰 (Pitch) +/-                       ║
║    J / L   →  绕Z轴 偏航 (Yaw)   +/-                       ║
║    U / O   →  绕X轴 横滚 (Roll)  +/-                       ║
╠══════════════════════════════════════════════════════════════╣
║  其他:                                                       ║
║    Z / X   →  夹爪 关闭 / 张开                              ║
║    M       →  外力传感器归零（以当前值为零点）               ║
║    Space   →  打印当前笛卡尔目标位姿                         ║
║    R       →  回到初始位置                                   ║
║    Esc     →  退出程序                                       ║
╚══════════════════════════════════════════════════════════════╝
""")

    # ── 初始化 pygame（可选）────────────────────────────────────
    if PYGAME_AVAILABLE:
        pygame.init()
        screen = pygame.display.set_mode((600, 210))
        pygame.display.set_caption("Cartesian Impedance (Polar) + Digital Twin")
        font = pygame.font.SysFont(None, 22)
        clock = pygame.time.Clock()
        print("【pygame 窗口已打开】键盘控制来源：pygame 窗口 + 浏览器前端")
    else:
        screen = font = clock = None
        print("【无 pygame】键盘控制仅通过浏览器前端")

    print(f"  浏览器打开 http://localhost:3000 → Connect → 键盘控制\n")

    # ── 初始化机器人 ─────────────────────────────────────────
    robot = Panthera()
    zero_pos = [0.0] * robot.motor_count
    _z6 = [0.0] * 6

    ctrl_data = robot.model.createData()

    # 1. 回零位
    print("初始化: 关节回零位...")
    robot.Joint_Pos_Vel(zero_pos, JOINT_VEL, MAX_TORQUE, iswait=True)
    time.sleep(0.5)

    # 2. 移动到初始位姿
    home_rot = robot.rotation_matrix_from_euler(*HOME_ROT_EULER)
    init_joints = robot.inverse_kinematics(HOME_POS, home_rot, robot.get_current_pos())
    if init_joints is None:
        print("[警告] 初始位置 IK 失败，保持零位")
    else:
        print(f"移动到初始关节位置，末端目标: {HOME_POS}")
        robot.moveJ(init_joints, duration=3.0, max_tqu=MAX_TORQUE, iswait=True)
    time.sleep(0.5)

    # 3. 末端沿 Z 轴上升 0.1m
    fk_after_home = robot.forward_kinematics()
    p_lift = np.array(fk_after_home['position'])
    R_lift = np.array(fk_after_home['rotation'])
    p_lift[2] += 0.1

    print(f"末端沿 Z 轴上升 0.1m，目标 Z = {p_lift[2]:.3f} m ...")
    success = robot.moveL(
        target_position=p_lift,
        target_rotation=R_lift,
        duration=2.0,
        use_spline=True
    )
    if not success:
        print("[警告] moveL 上升失败，以当前位置作为笛卡尔控制起点")
    time.sleep(0.5)

    # 4. 读取当前位姿作为笛卡尔目标初值
    q0 = np.array(robot.get_current_pos())
    p0, R0, _ = compute_fk_and_jacobian(robot, robot.data, q0)
    print_pose(p0, R0, "笛卡尔阻抗控制起始位姿（轴角法）")

    # 夹爪限位
    global _gripper_limits
    gripper_min = robot.gripper_limits['lower'] if robot.gripper_limits else GRIPPER_MIN_DEFAULT
    gripper_max = robot.gripper_limits['upper'] if robot.gripper_limits else GRIPPER_MAX_DEFAULT
    _gripper_limits[:] = [float(gripper_min), float(gripper_max)]
    print(f"夹爪限位: [{gripper_min:.2f}, {gripper_max:.2f}] rad")

    # 关节限位
    jl_lower = np.array(robot.joint_limits['lower']) + JOINT_LIMIT_MARGIN
    jl_upper = np.array(robot.joint_limits['upper']) - JOINT_LIMIT_MARGIN
    print(f"关节限位 (含 {JOINT_LIMIT_MARGIN} rad 安全裕量):")
    for i in range(len(jl_lower)):
        print(f"  J{i+1}: [{jl_lower[i]:+.2f}, {jl_upper[i]:+.2f}] rad")

    # ── 线程间共享状态 ───────────────────────────────────────
    lock = threading.Lock()
    x_des_pos = p0.copy()
    x_des_rot = R0.copy()
    gripper_des = float(np.clip(robot.get_current_pos_gripper(), gripper_min, gripper_max))
    ctrl_enabled = threading.Event()
    stop_event = threading.Event()
    ctrl_enabled.set()

    last_feasible_pos = p0.copy()
    last_feasible_rot = R0.copy()
    joint_limit_warn = ""
    joint_limit_warn_time = 0.0

    disp_err = np.zeros(6)
    disp_gripper = np.zeros(2)
    disp_wrench = np.zeros(6)
    wrench_bias = np.zeros(6)
    disp_lock = threading.Lock()

    # ── 笛卡尔阻抗控制线程 ──────────────────────────────────
    def impedance_loop():
        nonlocal last_feasible_pos, last_feasible_rot
        nonlocal joint_limit_warn, joint_limit_warn_time
        dt = 1.0 / CTRL_FREQ_IMP
        F_ext_filtered = np.zeros(6)
        alpha_ft = (2 * np.pi * FT_CUTOFF_FREQ * dt) / (1 + 2 * np.pi * FT_CUTOFF_FREQ * dt)
        # 力矩输出低通滤波状态
        tor_filtered = np.zeros(6)
        alpha_tor = (2 * np.pi * TOR_LPF_CUTOFF * dt) / (1 + 2 * np.pi * TOR_LPF_CUTOFF * dt)
        tor_initialized = False
        while not stop_event.is_set():
            t0 = time.time()

            if ctrl_enabled.is_set():
                q = np.array(robot.get_current_pos())
                dq = np.array(robot.get_current_vel())

                # 关节限位检测
                violated = (q < jl_lower) | (q > jl_upper)
                if np.any(violated):
                    parts = []
                    for i in range(len(q)):
                        if violated[i]:
                            side = "下限" if q[i] < jl_lower[i] else "上限"
                            parts.append(f"J{i+1}{side}({q[i]:+.2f})")
                    with lock:
                        joint_limit_warn = " ".join(parts)
                        joint_limit_warn_time = time.time()
                        x_des_pos[:] = last_feasible_pos
                        x_des_rot[:] = last_feasible_rot
                else:
                    with lock:
                        last_feasible_pos[:] = x_des_pos
                        last_feasible_rot[:] = x_des_rot

                # FK + Jacobian
                p_cur, R_cur, J = compute_fk_and_jacobian(robot, ctrl_data, q)

                with lock:
                    p_des = x_des_pos.copy()
                    R_des = x_des_rot.copy()
                    g_des = gripper_des

                # 笛卡尔误差
                e_pos = p_des - p_cur
                e_rot = orientation_error_axis_angle(R_des, R_cur)
                e_x = np.concatenate([e_pos, e_rot])

                dx = J @ dq
                F = K_CART * e_x - B_CART * dx

                # DLS 映射
                JJT = J @ J.T
                alpha = np.linalg.solve(JJT + LAMBDA_DAMP**2 * np.eye(6), F)
                tor_cart = J.T @ alpha

                tor_joint_damp = -JOINT_DAMPING * dq

                # 重力 + 摩擦补偿（科氏力已关闭，dq 噪声敏感）
                tor_gra = np.array(robot.get_Gravity())
                tor_fri = np.array(robot.get_friction_compensation(
                    dq, IMP_Fc, IMP_Fv, IMP_VEL_THRESH))

                tor_raw = np.clip(tor_cart + tor_joint_damp + tor_gra + tor_fri,
                                  -TAU_LIMIT, TAU_LIMIT)

                # 力矩输出低通滤波
                if not tor_initialized:
                    tor_filtered[:] = tor_raw
                    tor_initialized = True
                else:
                    tor_filtered[:] = alpha_tor * tor_raw + (1 - alpha_tor) * tor_filtered
                tor = tor_filtered

                # 发送力矩命令
                robot.Motors[robot.gripper_id - 1].pos_vel_tqe_kp_kd(
                    g_des, 0.0, 0.0, GRIPPER_KP, GRIPPER_KD)
                robot.pos_vel_tqe_kp_kd(_z6, _z6, tor.tolist(), _z6, _z6)

                # 外力估计
                tau_measured = np.array(robot.get_current_torque())
                tau_model = tor_gra + tor_fri
                tau_ext = tau_measured - tau_model

                JJT_ft = J @ J.T
                F_ext_raw = J @ np.linalg.solve(
                    JJT_ft + LAMBDA_DAMP**2 * np.eye(6), tau_ext)

                F_ext_filtered[:] = alpha_ft * F_ext_raw + (1 - alpha_ft) * F_ext_filtered

                # 更新显示缓冲
                g_actual = robot.get_current_pos_gripper()
                with disp_lock:
                    disp_err[:] = e_x
                    disp_gripper[:] = [g_des, g_actual]
                    disp_wrench[:] = F_ext_filtered - wrench_bias

                # 更新广播缓冲
                euler_deg = Rot.from_matrix(R_cur).as_euler('xyz', degrees=True)
                with _broadcast_lock:
                    _broadcast_state['positions'] = q.tolist()
                    _broadcast_state['velocities'] = dq.tolist()
                    _broadcast_state['torques'] = tau_measured.tolist()
                    _broadcast_state['ee_position'] = p_cur.tolist()
                    _broadcast_state['ee_euler'] = euler_deg.tolist()
                    _broadcast_state['cartesian_error'] = e_x.tolist()
                    _broadcast_state['external_wrench'] = (F_ext_filtered - wrench_bias).tolist()
                    _broadcast_state['target_ee_position'] = p_des.tolist()
                    _broadcast_state['control_torques'] = tor.tolist()
                    _broadcast_state['impedance_kp'] = K_CART.tolist()
                    _broadcast_state['gripper_position'] = float(g_actual)
                    _broadcast_state['timestamp'] = time.time()

            elapsed = time.time() - t0
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

    ctrl_thread = threading.Thread(target=impedance_loop, daemon=True)
    ctrl_thread.start()

    # ── 状态广播线程 ─────────────────────────────────────────
    def broadcast_loop():
        dt = 1.0 / BROADCAST_FREQ
        while not stop_event.is_set():
            t0 = time.time()
            if connected_clients:
                with _broadcast_lock:
                    state = _broadcast_state.copy()
                socketio.emit('robot_state', state)
            elapsed = time.time() - t0
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

    broadcast_thread = threading.Thread(target=broadcast_loop, daemon=True)
    broadcast_thread.start()

    # ── Flask+SocketIO 后台启动 ──────────────────────────────
    print(f"\n[Server] Starting on port {SERVER_PORT}...")
    print(f"  Backend API: http://localhost:{SERVER_PORT}")
    print(f"  Frontend:    http://localhost:3000 (npm run dev)")

    server_thread = threading.Thread(
        target=lambda: socketio.run(app, host='0.0.0.0', port=SERVER_PORT,
                                     debug=False, use_reloader=False,
                                     log_output=False),
        daemon=True
    )
    server_thread.start()

    # ── 键盘主循环（pygame + web 统一处理）─────────────────────
    print("\n开始键盘控制...\n")

    rot_step_rad = np.radians(ROT_STEP)
    KEY_ACTION_MAP = {
        'q': (np.array([0, 0, POS_STEP]), None),
        'e': (np.array([0, 0, -POS_STEP]), None),
        'i': (np.zeros(3), (0, rot_step_rad, 0)),
        'k': (np.zeros(3), (0, -rot_step_rad, 0)),
        'j': (np.zeros(3), (0, 0, rot_step_rad)),
        'l': (np.zeros(3), (0, 0, -rot_step_rad)),
        'u': (np.zeros(3), (rot_step_rad, 0, 0)),
        'o': (np.zeros(3), (-rot_step_rad, 0, 0)),
    }

    try:
        running = True
        while running:
            pygame_keys = None

            # ── pygame 事件处理（如果有）──────────────────────
            if PYGAME_AVAILABLE:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            running = False
                        elif event.key == pygame.K_SPACE:
                            with _command_queue_lock:
                                _command_queue.append('print_pose')
                        elif event.key == pygame.K_r:
                            with _command_queue_lock:
                                _command_queue.append('home')
                        elif event.key == pygame.K_m:
                            with _command_queue_lock:
                                _command_queue.append('zero_ft')
                pygame_keys = pygame.key.get_pressed()

            # ── 处理命令队列（来自 pygame 或 web）──────────────
            with _command_queue_lock:
                cmds = list(_command_queue)
                _command_queue.clear()

            for cmd in cmds:
                if cmd == 'print_pose':
                    with lock:
                        p_print = x_des_pos.copy()
                        R_print = x_des_rot.copy()
                    print_pose(p_print, R_print, "当前笛卡尔目标位姿")

                elif cmd == 'home':
                    ctrl_enabled.clear()
                    print("\n回到初始位置...")
                    home_q = init_joints if init_joints is not None else zero_pos
                    robot.moveJ(home_q, duration=2.0, max_tqu=MAX_TORQUE, iswait=True)
                    q_now = np.array(robot.get_current_pos())
                    p_now, R_now, _ = compute_fk_and_jacobian(
                        robot, robot.data, q_now)
                    with lock:
                        x_des_pos[:] = p_now
                        x_des_rot[:] = R_now
                    print_pose(p_now, R_now, "已回到初始位置")
                    ctrl_enabled.set()

                elif cmd == 'zero_ft':
                    with disp_lock:
                        wrench_bias[:] = disp_wrench + wrench_bias
                    print("[M] 外力传感器已归零")

            # ── 持续按键 → 更新笛卡尔目标（统一 pygame + web）──
            if ctrl_enabled.is_set():
                combined_pos = np.zeros(3)
                first_rot_euler = None
                any_pressed = False

                for key_name, (dp, rot_euler) in KEY_ACTION_MAP.items():
                    if _is_key_active(pygame_keys, key_name):
                        combined_pos += dp
                        if rot_euler is not None and first_rot_euler is None:
                            first_rot_euler = rot_euler
                        any_pressed = True

                # 极坐标处理
                with lock:
                    px, py = x_des_pos[0], x_des_pos[1]
                r_horiz = np.hypot(px, py)

                if _is_key_active(pygame_keys, 'a') or _is_key_active(pygame_keys, 'd'):
                    dtheta = POS_STEP / max(r_horiz, 0.01)
                    dtheta = min(dtheta, POLAR_ANGLE_MAX)
                    if _is_key_active(pygame_keys, 'd'):
                        dtheta = -dtheta
                    c, s = np.cos(dtheta), np.sin(dtheta)
                    Rz = np.array([[c, -s, 0.0],
                                   [s,  c, 0.0],
                                   [0.0, 0.0, 1.0]])
                    with lock:
                        x_des_pos[:] = Rz @ x_des_pos
                        x_des_rot[:] = Rz @ x_des_rot
                    any_pressed = True

                if _is_key_active(pygame_keys, 'w') or _is_key_active(pygame_keys, 's'):
                    if r_horiz > 0.001:
                        radial_dir = np.array([px, py, 0.0]) / r_horiz
                    else:
                        radial_dir = np.array([1.0, 0.0, 0.0])
                    dr = POS_STEP if _is_key_active(pygame_keys, 'w') else -POS_STEP
                    with lock:
                        x_des_pos += dr * radial_dir
                    any_pressed = True

                if any_pressed:
                    with lock:
                        x_des_pos += combined_pos
                        if first_rot_euler is not None:
                            dR = robot.rotation_matrix_from_euler(*first_rot_euler)
                            new_rot = x_des_rot @ dR
                            U, _, Vt = np.linalg.svd(new_rot)
                            x_des_rot[:] = U @ Vt

                # 夹爪控制
                if _is_key_active(pygame_keys, 'z') or _is_key_active(pygame_keys, 'x'):
                    with lock:
                        if _is_key_active(pygame_keys, 'z'):
                            gripper_des = max(gripper_min, gripper_des - GRIPPER_STEP)
                        if _is_key_active(pygame_keys, 'x'):
                            gripper_des = min(gripper_max, gripper_des + GRIPPER_STEP)

            # ── pygame 窗口刷新（如果有）─────────────────────
            if PYGAME_AVAILABLE:
                screen.fill((20, 20, 30))
                screen.blit(font.render(
                    "W/S=radial  A/D=orbit  Q/E=Z  I/K/J/L/U/O  Z/X  M=zero  R  Esc",
                    True, (160, 190, 220)), (10, 6))

                with disp_lock:
                    err_snap = disp_err.copy()
                    g_snap = disp_gripper.copy()
                    wrench_snap = disp_wrench.copy()
                ep_mm = err_snap[:3] * 1000.0
                er_deg = np.degrees(err_snap[3:])
                ep_n = np.linalg.norm(ep_mm)
                er_n = np.linalg.norm(er_deg)

                def _err_color(val, lo, hi):
                    if val < lo:  return (80, 220, 80)
                    if val < hi:  return (220, 200, 60)
                    return (220, 70, 70)

                screen.blit(font.render(
                    f"Pos err(mm) X:{ep_mm[0]:+6.1f}  Y:{ep_mm[1]:+6.1f}  Z:{ep_mm[2]:+6.1f}  |e|:{ep_n:5.1f}",
                    True, _err_color(ep_n, 3, 15)), (10, 36))
                screen.blit(font.render(
                    f"Rot err(deg) X:{er_deg[0]:+6.1f}  Y:{er_deg[1]:+6.1f}  Z:{er_deg[2]:+6.1f}  |e|:{er_n:5.1f}",
                    True, _err_color(er_n, 2, 8)), (10, 58))
                with lock:
                    g_des_disp = gripper_des
                    polar_pos = x_des_pos.copy()
                    warn_text = joint_limit_warn
                    warn_t = joint_limit_warn_time
                g_pct = (g_des_disp - gripper_min) / max(gripper_max - gripper_min, 1e-6) * 100
                polar_r = np.hypot(polar_pos[0], polar_pos[1])
                polar_theta = np.degrees(np.arctan2(polar_pos[1], polar_pos[0]))
                screen.blit(font.render(
                    f"Gripper des:{g_des_disp:.3f} act:{g_snap[1]:.3f} {g_pct:.0f}%"
                    f"   Polar r={polar_r:.3f}m  \u03b8={polar_theta:+.1f}\u00b0  z={polar_pos[2]:.3f}m",
                    True, (160, 200, 240)), (10, 80))
                screen.blit(font.render(
                    f"ctrl={'ON ' if ctrl_enabled.is_set() else 'OFF'}   "
                    f"K_pos=[{K_POS[0]:.0f},{K_POS[1]:.0f},{K_POS[2]:.0f}]  "
                    f"K_rot=[{K_ROT[0]:.0f},{K_ROT[1]:.0f},{K_ROT[2]:.0f}]  "
                    f"DT:{len(connected_clients)}cli",
                    True, (130, 130, 150)), (10, 103))

                # 外力估计显示
                f_force = wrench_snap[:3]
                f_torque = wrench_snap[3:]
                f_norm = np.linalg.norm(f_force)
                ft_color = _err_color(f_norm, FT_WARN_LO, FT_WARN_HI)
                screen.blit(font.render(
                    f"F_ext(N) X:{f_force[0]:+5.2f} Y:{f_force[1]:+5.2f} Z:{f_force[2]:+5.2f} |F|:{f_norm:5.2f}"
                    f"  M(Nm) X:{f_torque[0]:+5.2f} Y:{f_torque[1]:+5.2f} Z:{f_torque[2]:+5.2f}",
                    True, ft_color), (10, 125))

                # 关节限位报警
                if warn_text and (time.time() - warn_t) < JOINT_LIMIT_WARN_DURATION:
                    blink = int(time.time() * 4) % 2 == 0
                    if blink:
                        screen.blit(font.render(
                            f"JOINT LIMIT  {warn_text}",
                            True, (255, 60, 60)), (10, 147))

                pygame.display.flip()
                clock.tick(CTRL_FREQ_KEY)
            else:
                # Headless: maintain 250Hz loop rate
                time.sleep(1.0 / CTRL_FREQ_KEY)

    finally:
        stop_event.set()
        ctrl_thread.join(timeout=1.0)
        if PYGAME_AVAILABLE:
            pygame.quit()
        print("\n\n程序已退出（电机将保持当前位置直至超时掉电）")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n程序被中断")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
