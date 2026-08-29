#!/usr/bin/env python3
"""
PantheraSim - Simulated Panthera robot for web visualization.
Drops into example scripts as a drop-in replacement for Panthera_lib.Panthera.
Uses Pinocchio for kinematics/dynamics and pushes joint state to the
digital-twin backend so the 3D frontend visualizes it in real time.
"""

import time
import os
import sys
import json
import yaml
import numpy as np
import urllib.request

try:
    import pinocchio as pin
    from scipy.spatial.transform import Rotation as R
    from scipy.interpolate import CubicSpline
    HAS_PINOCCHIO = True
except ImportError:
    HAS_PINOCCHIO = False
    print("[PantheraSim] pinocchio / scipy not available — FK/IK disabled")


# ── tiny MotorState placeholder ──────────────────────────────────────────
class _MotorState:
    __slots__ = ('position', 'velocity', 'torque')
    def __init__(self, position=0.0, velocity=0.0, torque=0.0):
        self.position = position
        self.velocity = velocity
        self.torque = torque


class _FakeMotor:
    """Simulates a single motor, delegates state to the parent PantheraSim."""
    def __init__(self, index, robot):
        self._index = index
        self._robot = robot

    def get_current_motor_state(self):
        if self._index < self._robot.motor_count:
            return _MotorState(
                self._robot._pos[self._index],
                self._robot._vel[self._index],
                self._robot._tqe[self._index])
        else:
            return self._robot._gripper

    def get_motor_id(self): return self._index + 1
    def get_motor_enum_type(self): return "MotorType.SIM"
    def get_motor_name(self):
        return f"joint{self._index}" if self._index < self._robot.motor_count else "gripper"


class PantheraSim:
    """Drop-in replacement for Panthera_lib.Panthera that simulates the
    robot locally and streams joint state to the digital-twin backend."""

    # Default backend URL – can be overridden via env var
    BACKEND_URL = os.environ.get("PANTHERA_BACKEND_URL", "http://127.0.0.1:5000")

    def __init__(self, config_path=None):
        self.motor_count = 6
        self.gripper_id = 6

        # ── resolve config path ─────────────────────────────────────────
        if config_path is None:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.normpath(
                os.path.join(script_dir, "..", "robot_param", "xlb.yaml"))

        self.config_dir = os.path.dirname(os.path.abspath(config_path))
        print(f"[PantheraSim] config_path = {config_path}")

        # ── load YAML ───────────────────────────────────────────────────
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        print(f"[PantheraSim] Config loaded: {self.config.get('robot', {}).get('name', 'unknown')}")

        # ── joint limits ────────────────────────────────────────────────
        rcfg = self.config.get('robot', {})
        jl = rcfg.get('joint_limits', {})
        self.joint_limits = {
            'lower': np.array(jl.get('lower', [-3.14]*self.motor_count)),
            'upper': np.array(jl.get('upper', [ 3.14]*self.motor_count)),
        }
        self.gripper_limits = {
            'lower': rcfg.get('gripper_limits', {}).get('lower', 0.0),
            'upper': rcfg.get('gripper_limits', {}).get('upper', 2.0),
        }

        # ── motor params (simulated defaults) ───────────────────────────
        self.max_torque = np.array(rcfg.get('max_torque',
            [21.0, 36.0, 36.0, 21.0, 10.0, 10.0]))
        self.velocity_limits = np.array(rcfg.get('velocity_limits',
            [3.14]*self.motor_count))
        self.acceleration_limits = np.array(rcfg.get('acceleration_limits',
            [10.0]*self.motor_count))

        # moveit params
        mc = self.config.get('moveit_cartesian', {})
        self.eef_step = mc.get('eef_step', 0.01)
        self.jump_threshold = mc.get('jump_threshold', 0.5)
        self.resample_dt = mc.get('resample_dt', 0.01)

        # ── URDF / Pinocchio ────────────────────────────────────────────
        self.model = None
        self.data = None
        self.joint_names = []
        self.joint_ids = []
        self._load_urdf()

        # ── internal state ──────────────────────────────────────────────
        self._pos = np.zeros(self.motor_count)
        self._vel = np.zeros(self.motor_count)
        self._tqe = np.zeros(self.motor_count)
        self._gripper = _MotorState(0.0)
        self._last_push = 0.0

        # Fake Motors list (7 elements: 6 joints + 1 gripper)
        self.Motors = [_FakeMotor(i, self) for i in range(self.motor_count + 1)]

    # ── URDF loading ────────────────────────────────────────────────────
    def _load_urdf(self):
        if not HAS_PINOCCHIO:
            return
        try:
            urdf_rel = self.config['urdf']['file_path']
            urdf_path = os.path.normpath(os.path.join(self.config_dir, urdf_rel))
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            self.joint_names = self.config['kinematics']['joint_names']
            for name in self.joint_names:
                if self.model.existJointName(name):
                    self.joint_ids.append(self.model.getJointId(name))
            print(f"[PantheraSim] URDF loaded: {urdf_path}  ({len(self.joint_ids)} joints)")
        except Exception as e:
            print(f"[PantheraSim] URDF load failed: {e}")

    # ── internal helpers ────────────────────────────────────────────────
    def _sim_step(self, target_pos, target_vel, dt=0.005):
        """Crude first-order simulation: move position toward target."""
        alpha = 0.3  # smoothing factor per step (tunable)
        self._pos = self._pos + alpha * (np.asarray(target_pos) - self._pos)
        self._vel = np.asarray(target_vel)
        self._tqe = np.zeros(self.motor_count)  # simplified
        self._push_to_backend()

    def _push_to_backend(self):
        """POST current joint state to the digital-twin backend."""
        now = time.time()
        if now - self._last_push < 0.02:  # throttle to ~50 Hz
            return
        self._last_push = now
        try:
            payload = json.dumps({
                "positions": self._pos.tolist(),
                "velocities": self._vel.tolist(),
                "torques": self._tqe.tolist(),
                "fk": self._compute_fk_dict(),
            }).encode('utf-8')
            req = urllib.request.Request(
                f"{self.BACKEND_URL}/api/script_state",
                data=payload,
                headers={"Content-Type": "application/json"})
            urllib.request.urlopen(req, timeout=0.3)
        except Exception as e:
            if os.environ.get("PANTHERA_DEBUG"):
                print(f"[PantheraSim] push error: {e}")

    def _compute_fk_dict(self):
        """Return FK dict with plain Python types (JSON-serializable)."""
        fk = self.forward_kinematics(self._pos)
        if fk is None:
            return None
        Rmat = fk['rotation']
        pos = fk['position']
        return {
            'position': pos.tolist() if hasattr(pos, 'tolist') else list(pos),
            'rotation': Rmat.tolist() if hasattr(Rmat, 'tolist') else [list(r) for r in Rmat],
            'euler': self._rotation_to_euler(Rmat),
        }

    @staticmethod
    @staticmethod
    def _rotation_to_euler(rot_matrix):
        try:
            rot = R.from_matrix(np.array(rot_matrix))
            zyx = rot.as_euler('ZYX', degrees=True)
            return [zyx[1], zyx[2], zyx[0]]  # roll,pitch,yaw  (matching backend convention)
        except Exception:
            return [0, 0, 0]

    # ═══════════════════════════════════════════════════════════════════
    #  Hardware-command stubs  (no-ops in simulation)
    # ═══════════════════════════════════════════════════════════════════
    def send_get_motor_state_cmd(self): pass
    def motor_send_cmd(self): pass
    def set_stop(self): pass
    def set_reset_zero(self): pass

    # ═══════════════════════════════════════════════════════════════════
    #  State getters
    # ═══════════════════════════════════════════════════════════════════
    def get_current_pos(self):
        return self._pos.copy()

    def get_current_vel(self):
        return self._vel.copy()

    def get_current_torque(self):
        return self._tqe.copy()

    def get_current_state_gripper(self):
        return self._gripper

    def get_current_pos_gripper(self):
        return self._gripper.position

    def get_current_vel_gripper(self):
        return self._gripper.velocity

    def get_current_torque_gripper(self):
        return self._gripper.torque

    # ═══════════════════════════════════════════════════════════════════
    #  Joint control
    # ═══════════════════════════════════════════════════════════════════
    def Joint_Pos_Vel(self, pos, vel, max_tqu=None, iswait=False,
                       tolerance=0.1, timeout=15.0):
        pos = np.asarray(pos)
        vel_arr = np.asarray(vel)
        # Clamp to limits
        pos = np.clip(pos, self.joint_limits['lower'], self.joint_limits['upper'])
        if iswait:
            # Simulate movement at the requested velocity
            start = self._pos.copy()
            dist = np.max(np.abs(pos - start))
            avg_vel = np.mean(np.abs(vel_arr))
            duration = dist / avg_vel if avg_vel > 0.01 else 1.0
            duration = min(duration, timeout)  # cap at timeout
            steps = max(20, int(duration * 50))  # ~50 fps simulation
            for i in range(steps):
                t = (i + 1) / steps
                s = 3*t*t - 2*t*t*t
                self._pos = start + s * (pos - start)
                self._vel = vel_arr * (1 - s)
                self._tqe = np.zeros(self.motor_count)
                self._push_to_backend()
                time.sleep(duration / steps)
        else:
            self._sim_step(pos, vel_arr)
        return True

    def Joint_Vel(self, vel):
        vel = np.asarray(vel)
        vel = np.clip(vel, -self.velocity_limits, self.velocity_limits)
        self._vel = vel
        self._pos = self._pos + vel * 0.005
        self._push_to_backend()
        return True

    def moveJ(self, pos, duration, max_tqu=None, iswait=False,
              tolerance=0.1, timeout=15.0):
        pos = np.asarray(pos)
        pos = np.clip(pos, self.joint_limits['lower'], self.joint_limits['upper'])
        current = self.get_current_pos()
        vel = (pos - current) / duration
        return self.Joint_Pos_Vel(pos, vel, max_tqu, iswait, tolerance, timeout)

    def pos_vel_tqe_kp_kd(self, pos, vel, tqe, kp, kd):
        """MIT 5-param control – simulated as position control."""
        pos = np.asarray(pos)
        pos = np.clip(pos, self.joint_limits['lower'], self.joint_limits['upper'])
        # Simple PD emulation: move toward target pos
        err = pos - self._pos
        sim_vel = np.asarray(kp) * err  # approximate
        sim_vel = np.clip(sim_vel, -self.velocity_limits, self.velocity_limits)
        self._sim_step(pos, sim_vel)
        return True

    def Joints_Sync_Arrival(self, pos, duration=2.0):
        """All joints arrive simultaneously."""
        return self.moveJ(pos, duration, iswait=True)

    # ═══════════════════════════════════════════════════════════════════
    #  Gripper control
    # ═══════════════════════════════════════════════════════════════════
    def gripper_control(self, pos, vel, max_tqu=0.5):
        pos = max(self.gripper_limits['lower'],
                  min(self.gripper_limits['upper'], pos))
        self._gripper = _MotorState(pos, vel, 0.0)
        return True

    def gripper_control_MIT(self, pos, vel, tqe, kp, kd):
        pos = max(self.gripper_limits['lower'],
                  min(self.gripper_limits['upper'], pos))
        self._gripper = _MotorState(pos, vel, tqe)
        return True

    def gripper_open(self, pos=1.6, vel=0.5, max_tqu=0.5):
        return self.gripper_control(pos, vel, max_tqu)

    def gripper_close(self, pos=0.0, vel=0.5, max_tqu=0.5):
        return self.gripper_control(pos, vel, max_tqu)

    # ═══════════════════════════════════════════════════════════════════
    #  Kinematics
    # ═══════════════════════════════════════════════════════════════════
    def forward_kinematics(self, joint_angles=None):
        if self.model is None:
            return None
        if joint_angles is None:
            joint_angles = self._pos
        q = np.zeros(self.model.nq)
        for i, name in enumerate(self.joint_names):
            if i < len(joint_angles):
                jid = self.model.getJointId(name)
                idx = self.model.joints[jid].idx_q
                q[idx] = joint_angles[i]
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        last_name = self.joint_names[-1]
        last_jid = self.model.getJointId(last_name)
        T_last = self.data.oMi[last_jid]
        tool_offset = np.array([0.165, 0.0, 0.0])
        position = T_last.translation + T_last.rotation.dot(tool_offset)
        rotation = T_last.rotation.copy()
        T = np.eye(4)
        T[:3, :3] = rotation
        T[:3, 3] = position
        return {'position': np.asarray(position),
                'rotation': np.asarray(rotation),
                'transform': T,
                'joint_angles': np.asarray(joint_angles)}

    def inverse_kinematics(self, target_position, target_rotation=None,
                           init_q=None, max_iter=1000, eps=1e-3, damping=1e-2,
                           adaptive_damping=True, multi_init=True, num_attempts=8):
        if self.model is None:
            return None
        if target_rotation is None:
            target_rotation = np.eye(3)
        tool_offset = np.array([0.165, 0.0, 0.0])
        last_joint_target = np.array(target_position) - np.array(target_rotation).dot(tool_offset)
        oMdes = pin.SE3(np.array(target_rotation), last_joint_target)
        if init_q is None:
            init_q = self._pos.copy()
        q = np.zeros(self.model.nq)
        for i, name in enumerate(self.joint_names):
            if i < len(init_q):
                jid = self.model.getJointId(name)
                idx = self.model.joints[jid].idx_q
                q[idx] = init_q[i]
        last_name = self.joint_names[-1]
        joint_id = self.model.getJointId(last_name)
        lower = self.joint_limits['lower']
        upper = self.joint_limits['upper']
        dt = 1e-1
        for it in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            iMd = self.data.oMi[joint_id].actInv(oMdes)
            err = pin.log(iMd).vector
            if np.linalg.norm(err) < eps:
                result = np.array([q[self.model.joints[self.model.getJointId(n)].idx_q]
                                   for n in self.joint_names])
                return result
            J = pin.computeJointJacobian(self.model, self.data, q, joint_id)
            J = -np.dot(pin.Jlog6(iMd.inverse()), J)
            lam = damping * (1.0 + 1.0 / (np.linalg.norm(err) + 0.1)) if adaptive_damping else damping
            JJT = J.dot(J.T)
            try:
                alpha = np.linalg.solve(JJT + lam**2 * np.eye(6), err)
            except np.linalg.LinAlgError:
                return None
            v = -J.T.dot(alpha)
            v_norm = np.linalg.norm(v)
            if v_norm > 10.0:
                v *= 10.0 / v_norm
            q_new = pin.integrate(self.model, q, v * dt)
            q_check = np.array([q_new[self.model.joints[self.model.getJointId(n)].idx_q]
                                for n in self.joint_names])
            if np.any(q_check < lower) or np.any(q_check > upper):
                return None
            q = q_new
        print(f"[PantheraSim] IK did not converge after {max_iter} iters")
        return None

    @staticmethod
    def rotation_matrix_from_euler(roll, pitch, yaw):
        return R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

    # ═══════════════════════════════════════════════════════════════════
    #  Dynamics
    # ═══════════════════════════════════════════════════════════════════
    def get_Gravity(self, q=None):
        if self.model is None:
            return np.zeros(self.motor_count)
        if q is None:
            q = self._pos
        q = np.asarray(q)
        orig_gravity = self.model.gravity.copy()
        self.model.gravity.linear = np.array([0.0, 0.0, -9.81])
        G = pin.computeGeneralizedGravity(self.model, self.data, q)
        self.model.gravity.linear = orig_gravity.linear
        return G

    def get_Coriolis(self, q=None, v=None):
        if self.model is None:
            return np.zeros((self.motor_count, self.motor_count))
        if q is None: q = self._pos
        if v is None: v = self._vel
        return pin.computeCoriolisMatrix(self.model, self.data, np.asarray(q), np.asarray(v))

    def get_Coriolis_vector(self, q=None, v=None):
        C = self.get_Coriolis(q, v)
        vv = np.asarray(self._vel if v is None else v)
        return C.dot(vv)

    def get_Mass_Matrix(self, q=None):
        if self.model is None:
            return np.eye(self.motor_count)
        if q is None: q = self._pos
        M = pin.crba(self.model, self.data, np.asarray(q))
        return M[:len(q), :len(q)]

    def get_Inertia_Terms(self, q=None, a=None):
        if q is None: q = self._pos
        if a is None: a = np.zeros(self.motor_count)
        M = self.get_Mass_Matrix(q)
        return M.dot(np.asarray(a))

    def get_Dynamics(self, q=None, v=None, a=None):
        if self.model is None:
            return np.zeros(self.motor_count)
        if q is None: q = self._pos
        if v is None: v = self._vel
        if a is None: a = np.zeros(self.model.nv)
        return pin.rnea(self.model, self.data, np.asarray(q),
                        np.asarray(v), np.asarray(a))

    @staticmethod
    def get_friction_compensation(vel=None, Fc=None, Fv=None, vel_threshold=0.01):
        if vel is None:
            return np.zeros(6)
        vel = np.asarray(vel)
        Fc = np.asarray(Fc)
        Fv = np.asarray(Fv)
        full = Fc * np.sign(vel) + Fv * vel
        low = Fv * vel
        return np.where(np.abs(vel) < vel_threshold, low, full)

    # ═══════════════════════════════════════════════════════════════════
    #  Trajectory helpers
    # ═══════════════════════════════════════════════════════════════════
    @staticmethod
    def septic_interpolation(start_pos, end_pos, duration, current_time):
        start = np.asarray(start_pos)
        end = np.asarray(end_pos)
        if current_time <= 0:
            return start, np.zeros_like(start), np.zeros_like(start)
        if current_time >= duration:
            return end, np.zeros_like(end), np.zeros_like(end)
        t = current_time / duration
        t2, t3 = t*t, t*t*t
        t4, t5 = t3*t, t4*t
        t6, t7 = t5*t, t6*t
        a0 = 1 - 35*t4 + 84*t5 - 70*t6 + 20*t7
        a1 = 35*t4 - 84*t5 + 70*t6 - 20*t7
        da0 = -140*t3 + 420*t4 - 420*t5 + 140*t6
        da1 = 140*t3 - 420*t4 + 420*t5 - 140*t6
        dda0 = -420*t2 + 1680*t3 - 2100*t4 + 840*t5
        dda1 = 420*t2 - 1680*t3 + 2100*t4 - 840*t5
        pos = a0*start + a1*end
        vel = (da0*start + da1*end) / duration
        acc = (dda0*start + dda1*end) / (duration*duration)
        return pos, vel, acc

    @staticmethod
    def septic_interpolation_with_velocity(start_pos, end_pos, start_vel,
                                            end_vel, duration, current_time):
        start = np.asarray(start_pos); end = np.asarray(end_pos)
        sv = np.asarray(start_vel); ev = np.asarray(end_vel)
        if current_time <= 0:
            return start, sv, np.zeros_like(start)
        if current_time >= duration:
            return end, ev, np.zeros_like(end)
        t = current_time / duration
        t2, t3 = t*t, t*t*t
        t4, t5 = t3*t, t4*t
        t6, t7 = t5*t, t6*t
        # 7th-order polynomial with velocity boundary conditions
        a0 = 1 - 20*t4 + 45*t5 - 36*t6 + 10*t7
        a1 = duration*(t - 6*t4 + 8*t5 - 3*t6)
        a2 = 20*t4 - 45*t5 + 36*t6 - 10*t7
        a3 = duration*(-4*t4 + 7*t5 - 3*t6)
        pos = a0*start + a1*sv + a2*end + a3*ev
        da0 = -80*t3 + 225*t4 - 216*t5 + 70*t6
        da1 = duration*(1 - 24*t3 + 40*t4 - 18*t5)
        da2 = 80*t3 - 225*t4 + 216*t5 - 70*t6
        da3 = duration*(-16*t3 + 35*t4 - 18*t5)
        vel = (da0*start + da1*sv + da2*end + da3*ev) / duration
        dda0 = -240*t2 + 900*t3 - 1080*t4 + 420*t5
        dda1 = duration*(-72*t2 + 160*t3 - 90*t4)
        dda2 = 240*t2 - 900*t3 + 1080*t4 - 420*t5
        dda3 = duration*(-48*t2 + 140*t3 - 90*t4)
        acc = (dda0*start + dda1*sv + dda2*end + dda3*ev) / (duration*duration)
        return pos, vel, acc

    # ═══════════════════════════════════════════════════════════════════
    #  Cartesian path helpers  (for moveL)
    # ═══════════════════════════════════════════════════════════════════
    def compute_cartesian_path(self, waypoints, avoid_collisions=False):
        if len(waypoints) < 2:
            return None, 0.0
        joint_trajectory = [self._pos.copy()]
        current_q = self._pos.copy()
        for i in range(len(waypoints) - 1):
            start_pose = waypoints[i]
            end_pose = waypoints[i + 1]
            segment_traj, success = self._interpolate_segment(
                start_pose, end_pose, current_q)
            if not success or segment_traj is None:
                frac = i / (len(waypoints) - 1)
                return joint_trajectory, frac
            if len(joint_trajectory) > 0 and len(segment_traj) > 0:
                if np.allclose(joint_trajectory[-1], segment_traj[0], atol=1e-4):
                    joint_trajectory.extend(segment_traj[1:])
                else:
                    joint_trajectory.extend(segment_traj)
            else:
                joint_trajectory.extend(segment_traj)
            current_q = joint_trajectory[-1]
        return joint_trajectory, 1.0

    def _interpolate_segment(self, start_pose, end_pose, current_q):
        num_steps = max(2, int(np.linalg.norm(
            np.array(end_pose['position']) - np.array(start_pose['position']))
            / self.eef_step))
        joint_path = []
        for step in range(num_steps + 1):
            t = step / num_steps
            s = 3*t*t - 2*t*t*t
            interp_pos = (np.array(start_pose['position']) * (1 - s)
                          + np.array(end_pose['position']) * s)
            interp_rot = self._slerp(start_pose['rotation'],
                                      end_pose['rotation'], s)
            ik_result = self.inverse_kinematics(interp_pos, interp_rot, current_q)
            if ik_result is None:
                return None, False
            joint_path.append(ik_result)
            current_q = ik_result
        return joint_path, True

    @staticmethod
    def _slerp(R1, R2, t):
        """Spherical linear interpolation between two rotation matrices."""
        R1, R2 = np.array(R1), np.array(R2)
        dR = R2 @ R1.T
        theta = np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))
        if abs(theta) < 1e-6:
            return R1
        sin_theta = np.sin(theta)
        return (np.sin((1-t)*theta)/sin_theta)*R1 + (np.sin(t*theta)/sin_theta)*R2

    def compute_time_parameterization(self, joint_trajectory, duration=None):
        if duration is not None:
            n = len(joint_trajectory)
            return np.linspace(0, duration, n).tolist()
        # Auto-calculate based on joint distances
        timestamps = [0.0]
        for i in range(1, len(joint_trajectory)):
            dist = np.linalg.norm(
                np.array(joint_trajectory[i]) - np.array(joint_trajectory[i-1]))
            dt = max(0.01, dist / 1.0)  # max 1 rad/s
            timestamps.append(timestamps[-1] + dt)
        return timestamps

    def smooth_trajectory_spline(self, joint_trajectory, timestamps):
        """Apply cubic spline smoothing and resample."""
        joint_trajectory = np.array(joint_trajectory)
        timestamps = np.array(timestamps)
        n_pts = len(timestamps)
        try:
            cs = CubicSpline(timestamps, joint_trajectory, axis=0, bc_type='clamped')
        except Exception:
            return joint_trajectory.tolist(), timestamps.tolist(), \
                   np.gradient(joint_trajectory, axis=0).tolist()
        resample_ts = np.arange(timestamps[0], timestamps[-1], self.resample_dt)
        if resample_ts[-1] < timestamps[-1]:
            resample_ts = np.append(resample_ts, timestamps[-1])
        smoothed = cs(resample_ts)
        velocities = cs(resample_ts, 1)
        return smoothed.tolist(), resample_ts.tolist(), velocities.tolist()

    def _execute_trajectory(self, joint_trajectory, timestamps, velocities, max_tqu=None):
        """Execute pre-computed trajectory in simulation."""
        start_time = time.perf_counter()
        for i in range(len(joint_trajectory)):
            target_time = timestamps[i]
            while (time.perf_counter() - start_time) < target_time:
                time.sleep(0.0001)
            self._pos = np.array(joint_trajectory[i])
            self._vel = np.array(velocities[i]) if i < len(velocities) else np.zeros(self.motor_count)
            self._push_to_backend()
        return True

    def moveL(self, target_position, target_rotation=None, duration=None,
              use_spline=True, max_tqu=None):
        print("[PantheraSim] moveL start")
        fk = self.forward_kinematics()
        if fk is None:
            return False
        start_pose = {'position': fk['position'], 'rotation': fk['rotation']}
        if target_rotation is None:
            target_rotation = start_pose['rotation']
        end_pose = {'position': target_position, 'rotation': target_rotation}
        traj, fraction = self.compute_cartesian_path([start_pose, end_pose])
        if traj is None or len(traj) == 0:
            return False
        timestamps = self.compute_time_parameterization(traj, duration)
        if use_spline:
            traj, timestamps, velocities = self.smooth_trajectory_spline(traj, timestamps)
        else:
            velocities = []
            for j in range(len(traj) - 1):
                dt = timestamps[j+1] - timestamps[j]
                v = (np.array(traj[j+1]) - np.array(traj[j])) / dt
                velocities.append(v)
            velocities.append(velocities[-1] if velocities else np.zeros(self.motor_count))
        return self._execute_trajectory(traj, timestamps, velocities, max_tqu)
