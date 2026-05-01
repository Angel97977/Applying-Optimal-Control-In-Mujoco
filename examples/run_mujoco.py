#!/usr/bin/env python3
"""run_mujoco.py — Quadruped locomotion with joint-PD trot gait (v7).

Fixes sobre v6
══════════════
1. u_grf_log actualizado en cada paso con la salida real del controlador
   → TVU ya no es 0.0
2. Métrica de error corregida: usa error de velocidad (vx,vy,vz) en lugar
   de error de posición integrada que crecía indefinidamente → RMSE realista
3. Filtro Butterworth restaurado sobre la salida GRF del controlador
4. Gait más rápido y visible:
     period      0.45 → 0.35 s
     step_height 0.10 → 0.13 m
     step_len_min 0.050 → 0.060 m
     ramp_time   0.60 → 0.40 s
5. waypoint_command max_vx 0.55 → 0.45 m/s (más estable al girar)

Examples
────────
    python examples/run_mujoco.py
    python examples/run_mujoco.py --controller lqg
    python examples/run_mujoco.py --controller all --no-render
    python examples/run_mujoco.py --controller mpc --disturbance persistent
    python examples/run_mujoco.py --no-waypoints
"""

import sys, os, argparse, threading, select
import numpy as np
from dataclasses import dataclass
from scipy.signal import butter, sosfilt_zi, sosfilt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from gym_quadruped.quadruped_env import QuadrupedEnv
from src.dynamics       import QuadrupedDynamics
from src.estimator_ekf  import OrientationEKF
from src.controller_pmp import PontryaginController
from src.controller_lqg import LQGController
from src.controller_mpc import MPCController

# ── WaypointGenerator ──────────────────────────────────────────────────
try:
    from src.trajectory_generator import WaypointGenerator
except ImportError:
    class WaypointGenerator:
        def __init__(self, waypoints, distance_threshold: float = 0.15):
            self.waypoints     = np.array(waypoints, dtype=float)
            self.current_index = 0
            self.threshold     = distance_threshold
            self.is_finished   = False

        def get_reference(self, current_x: float, current_y: float) -> np.ndarray:
            if self.is_finished:
                return self.waypoints[-1]
            target   = self.waypoints[self.current_index]
            distance = float(np.hypot(target[0]-current_x, target[1]-current_y))
            if distance < self.threshold:
                print(f"[Trajectory] Waypoint {self.current_index+1}/"
                      f"{len(self.waypoints)} reached: "
                      f"({target[0]:.2f},{target[1]:.2f})")
                self.current_index += 1
                if self.current_index >= len(self.waypoints):
                    print("[Trajectory] Route complete!")
                    self.is_finished   = True
                    self.current_index = len(self.waypoints) - 1
            return self.waypoints[self.current_index]

        def progress(self):
            return self.current_index, len(self.waypoints), self.is_finished

        def reset(self):
            self.current_index = 0
            self.is_finished   = False


# ─────────────────────────────────────────────────────────────────────
# Parámetros físicos
# ─────────────────────────────────────────────────────────────────────
ROBOT_MASS       = 9.0
ROBOT_INERTIA    = np.diag([0.107, 0.098, 0.024])
ROBOT_HIP_HEIGHT = 0.30

T_WARMUP = 2.0

DEFAULT_WAYPOINTS = [
    [ 2.0,  0.0, 0.30,  0.0   ],
    [ 2.0,  2.0, 0.30,  1.5708],
    [ 0.0,  2.0, 0.30,  3.1416],
    [ 0.0,  0.0, 0.30, -1.5708],
]

LEG_NAMES  = ["FL", "FR", "RL", "RR"]
KP_STAND   = 80.0
KD_STAND   =  5.0
TAU_LIMIT  = 55.0
CTRL_SCALE = 0.12


# ─────────────────────────────────────────────────────────────────────
# Butterworth low-pass filter
# ─────────────────────────────────────────────────────────────────────
def make_butter_sos(fc: float, fs: float, order: int = 2):
    """Devuelve filtro Butterworth pasa-bajos en formato SOS."""
    nyq = 0.5 * fs
    wn  = min(fc / nyq, 0.99)
    return butter(order, wn, btype="low", output="sos")


# ─────────────────────────────────────────────────────────────────────
# Captura robusta de joints post-reset
# ─────────────────────────────────────────────────────────────────────
def capture_initial_joints(env) -> dict:
    Q_FALLBACK = {
        "FL": np.array([ 0.0, -0.8,  1.6]),
        "FR": np.array([ 0.0, -0.8,  1.6]),
        "RL": np.array([ 0.0,  0.8, -1.6]),
        "RR": np.array([ 0.0,  0.8, -1.6]),
    }
    q_init = {}
    for leg in LEG_NAMES:
        q = None
        for method in range(4):
            try:
                if method == 0:
                    c = np.array(env.mjData.qpos[env.legs_qpos_idx[leg]], dtype=float)
                elif method == 1:
                    vidx = env.legs_qvel_idx[leg]
                    c    = np.array(env.mjData.qpos[[v+7 for v in vidx]], dtype=float)
                elif method == 2:
                    vidx = env.legs_qvel_idx[leg]
                    c    = np.array(env.mjData.qpos[[v+1 for v in vidx]], dtype=float)
                else:
                    i = LEG_NAMES.index(leg)
                    c = np.array(env.mjData.qpos[7:][3*i: 3*i+3], dtype=float)
                if c.shape == (3,):
                    q = c; break
            except Exception:
                pass
        q_init[leg] = q if q is not None else Q_FALLBACK[leg].copy()
        if q is None:
            print(f"  [WARN] Fallback Q_STAND para {leg}")
    return q_init


# ─────────────────────────────────────────────────────────────────────
# Lectura de estado articular
# ─────────────────────────────────────────────────────────────────────
def read_joint(env, leg):
    vidx = env.legs_qvel_idx[leg]
    dq   = np.array(env.mjData.qvel[vidx], dtype=float)
    try:
        q = np.array(env.mjData.qpos[env.legs_qpos_idx[leg]], dtype=float)
    except Exception:
        q = np.array(env.mjData.qpos[[v+7 for v in vidx]], dtype=float)
    return q, dq


# ─────────────────────────────────────────────────────────────────────
# Warmup PD
# ─────────────────────────────────────────────────────────────────────
def warmup_torques(env, q_stand: dict) -> np.ndarray:
    tau = np.zeros(env.mjModel.nu)
    for leg in LEG_NAMES:
        try:
            q, dq = read_joint(env, leg)
            raw   = KP_STAND * (q_stand[leg] - q) - KD_STAND * dq
            tau[env.legs_tau_idx[leg]] = np.clip(raw, -TAU_LIMIT, TAU_LIMIT)
        except Exception:
            pass
    return tau


# ─────────────────────────────────────────────────────────────────────
# TrotGait v7 — más rápido, patas más altas
# ─────────────────────────────────────────────────────────────────────
class TrotGait:
    """
    Trot diagonal — todas las patas activas.

    Convenciones:
    ┌──────┬──────────┬──────────────────────────────────┐
    │ Pata │ hip_dir  │ knee_dir                         │
    ├──────┼──────────┼──────────────────────────────────┤
    │ FL   │ +1       │ -1  (knee- → pie sube)           │
    │ FR   │ +1       │ -1                               │
    │ RL   │ -1       │ +1  (knee+ → pie sube)           │
    │ RR   │ -1       │ +1                               │
    └──────┴──────────┴──────────────────────────────────┘
    Pares diagonales:
      Pair A: FL + RR  (phase_offset = 0.0)
      Pair B: FR + RL  (phase_offset = 0.5)
    """

    PHASE_OFFSET = {"FL": 0.0, "FR": 0.5, "RL": 0.5, "RR": 0.0}
    HIP_DIR      = {"FL": +1.0, "FR": +1.0, "RL": -1.0, "RR": -1.0}
    KNEE_DIR     = {"FL": -1.0, "FR": -1.0, "RL": +1.0, "RR": +1.0}

    def __init__(
        self,
        period:          float = 0.35,   # v7: 0.45→0.35 s (más rápido)
        swing_ratio:     float = 0.50,
        step_height:     float = 0.13,   # v7: 0.10→0.13 m (más alto)
        step_len_min:    float = 0.060,  # v7: 0.050→0.060 m
        step_len_max:    float = 0.150,
        step_multiplier: float = 0.95,
        kp:              float = 32.0,
        kd:              float = 5.0,
        ramp_time:       float = 0.40,   # v7: 0.60→0.40 s (arranca antes)
    ):
        self.period          = period
        self.swing_ratio     = swing_ratio
        self.step_height     = step_height
        self.step_len_min    = step_len_min
        self.step_len_max    = step_len_max
        self.step_multiplier = step_multiplier
        self.kp              = kp
        self.kd              = kd
        self.ramp_time       = ramp_time
        self._vx_f           = 0.0
        self._wz_f           = 0.0
        self._t_walk_start   = None

    def start_walking(self, t: float):
        if self._t_walk_start is None:
            self._t_walk_start = t

    def _ramp(self, t: float) -> float:
        if self._t_walk_start is None:
            return 0.0
        return float(np.clip((t - self._t_walk_start) / self.ramp_time, 0.0, 1.0))

    def filter_commands(self, cmd_vx: float, cmd_wz: float,
                        dt: float, tau: float = 0.06):
        a = dt / (dt + tau)
        self._vx_f += a * (cmd_vx - self._vx_f)
        self._wz_f += a * (cmd_wz - self._wz_f)

    def _phase(self, t: float, leg: str) -> float:
        return ((t / self.period) + self.PHASE_OFFSET[leg]) % 1.0

    def is_swing(self, t: float, leg: str) -> bool:
        return self._phase(t, leg) < self.swing_ratio

    def _sw_norm(self, t: float, leg: str) -> float:
        ph = self._phase(t, leg)
        return ph / self.swing_ratio if ph < self.swing_ratio else 0.0

    def stance_mask(self, t: float) -> np.ndarray:
        return np.array([not self.is_swing(t, lg) for lg in LEG_NAMES],
                        dtype=bool)

    def _delta(self, sw, leg, ramp):
        step_len = float(np.clip(
            abs(self._vx_f) * self.period * self.step_multiplier,
            self.step_len_min, self.step_len_max,
        )) * ramp
        if ramp > 0.05:
            step_len = max(step_len, self.step_len_min * ramp)

        fwd_sign = float(np.sign(self._vx_f)) if abs(self._vx_f) > 0.005 else 1.0
        hip_dir  = self.HIP_DIR[leg]
        knee_dir = self.KNEE_DIR[leg]

        d_hip  = hip_dir  * fwd_sign * step_len * (2.0 * sw - 1.0)
        d_knee = knee_dir * self.step_height * ramp * np.sin(np.pi * sw)

        d_abd = 0.0
        if abs(self._wz_f) > 0.03:
            outer = {"FL": self._wz_f > 0, "FR": self._wz_f < 0,
                     "RL": self._wz_f > 0, "RR": self._wz_f < 0}
            d_abd = (0.022 * np.sign(self._wz_f)
                     * (1.0 if outer[leg] else -1.0) * ramp)

        return np.array([d_abd, d_hip, d_knee])

    def compute_all_torques(self, t, env, cmd_vx, cmd_vy, cmd_wz,
                            q_stand) -> np.ndarray:
        ramp = self._ramp(t)
        tau  = np.zeros(12)
        for leg in LEG_NAMES:
            try:
                q, dq = read_joint(env, leg)
            except Exception:
                continue
            if self.is_swing(t, leg):
                sw    = self._sw_norm(t, leg)
                delta = self._delta(sw, leg, ramp)
                q_tgt = q_stand[leg] + delta
            else:
                q_tgt = q_stand[leg]
            raw = self.kp * (q_tgt - q) - self.kd * dq
            tau[env.legs_tau_idx[leg]] = np.clip(raw, -TAU_LIMIT, TAU_LIMIT)
        return tau


# ─────────────────────────────────────────────────────────────────────
# Waypoint → velocidad
# ─────────────────────────────────────────────────────────────────────
def waypoint_command(traj, x, ref_yaw,
                     max_vx: float = 0.45,   # v7: 0.55→0.45
                     max_wz: float = 0.65):
    if traj.is_finished:
        return 0.0, 0.0, 0.0
    target  = traj.get_reference(x[0], x[1])
    dx, dy  = target[0]-x[0], target[1]-x[1]
    dist    = float(np.hypot(dx, dy))
    ang     = float(np.arctan2(dy, dx))
    yaw_err = float(np.arctan2(np.sin(ang-ref_yaw), np.cos(ang-ref_yaw)))
    cmd_wz  = float(np.clip(2.5*yaw_err, -max_wz, max_wz))
    dead    = np.pi / 5.1
    cmd_vx  = 0.0 if abs(yaw_err) > dead else float(
        np.clip(0.90*dist*np.cos(yaw_err)**2, 0.0, max_vx))
    return cmd_vx, 0.0, cmd_wz


# ─────────────────────────────────────────────────────────────────────
# Modulación de velocidad por el controlador óptimo
# FIX: ahora también devuelve u para logging correcto
# ─────────────────────────────────────────────────────────────────────
def controller_compute(ctrl_name, controller, x, x_ref, u_ref):
    """
    Llama al controlador y devuelve (u_grf, cmd_vx_scale, cmd_wz_scale).
    u_grf es el vector GRF real (12,) usado para logging y TVU.
    """
    try:
        if ctrl_name == "lqg":
            noise = np.array([5e-3]*3 + [2e-2]*3 + [1e-2]*3 + [5e-2]*3)
            u     = controller.step(x + np.random.randn(12)*noise, x_ref, u_ref)
        else:
            u = controller.compute_control(x=x, x_ref=x_ref, u_ref=u_ref)
        u = np.clip(u, -200.0, 200.0)
        return u
    except Exception:
        return u_ref.copy()


def modulate_velocity(u, x, x_ref, cmd_vx_wp, cmd_wz_wp):
    """Escala suavemente cmd_vx con la señal GRF del controlador."""
    fx_net = float(np.sum(u[0::3]))
    vx_sc  = float(np.clip(1.0 + CTRL_SCALE * fx_net / (ROBOT_MASS*9.81), 0.2, 1.6))
    h_sc   = float(np.clip(1.0 - 3.0 * abs(x[2] - x_ref[2]), 0.3, 1.0))
    return float(np.clip(cmd_vx_wp * vx_sc * h_sc, 0.0, 0.45)), 0.0, cmd_wz_wp


# ─────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────
def get_state(env) -> np.ndarray:
    return np.concatenate([
        env.base_pos.copy(),
        env.base_lin_vel(frame="world"),
        env.base_ori_euler_xyz.copy(),
        env.base_ang_vel(frame="base"),
    ])


def get_contacts(env) -> np.ndarray:
    try:
        cs, _ = env.feet_contact_state()
        return np.array([cs.FL, cs.FR, cs.RL, cs.RR], dtype=bool)
    except Exception:
        return np.ones(4, dtype=bool)


def get_feet_world(env):
    try:
        fp = env.feet_pos(frame="world")
        return np.array([fp.FL, fp.FR, fp.RL, fp.RR])
    except Exception:
        return None


# ─────────────────────────────────────────────────────────────────────
# Dinámica, costo y controladores
# ─────────────────────────────────────────────────────────────────────
def build_dynamics():
    return QuadrupedDynamics(mass=ROBOT_MASS, inertia=ROBOT_INERTIA, dt=0.01)


def build_reference_state(dyn, height, vx=0.0, vy=0.0, wz=0.0):
    x       = dyn.standing_state(height=height)
    x[3:6]  = [vx, vy, 0.0]
    x[6:9]  = [0.0, 0.0, 0.0]
    x[9:12] = [0.0, 0.0, wz]
    return x


def build_cost_matrices():
    Q = np.diag([80, 80, 300, 40, 40, 15, 150, 150, 100, 5, 5, 5])
    return Q, np.eye(12) * 5e-3, Q * 10


def build_controller(name, dyn, Q, R, Q_f, x_ref):
    A_d, B_d, g_d = dyn.get_linear_system(x_ref)
    A_c, B_c      = dyn.continuous_AB(x_ref)

    if name == "pmp":
        ctrl = PontryaginController(
            A=A_c, B=B_c, Q_s=Q, R_u=R, Q_f=Q_f,
            g_aff=dyn.gravity_vector() / dyn.dt, dt=dyn.dt, horizon=500)
        ctrl.solve_discrete_sweep(x_ref.copy(), x_ref)
        print("  [PMP] initialized"); return ctrl

    if name == "lqg":
        ctrl = LQGController(
            A_d=A_d, B_d=B_d, g_d=g_d, Q=Q*dyn.dt, R=R*dyn.dt,
            Q_proc=np.diag([1e-3]*3+[1e-2]*3+[5e-3]*3+[1e-2]*3),
            R_meas=np.diag([5e-3]*3+[2e-2]*3+[1e-2]*3+[5e-2]*3))
        ctrl.set_initial_estimate(x_ref)
        print("  [LQG] initialized"); return ctrl

    if name == "mpc":
        ctrl = MPCController(
            A_d=A_d, B_d=B_d, g_d=g_d,
            Q=Q*dyn.dt, R=R*dyn.dt, Q_f=Q_f*dyn.dt,
            N=15, mu=0.6, fz_max=150.0)
        print("  [MPC] initialized (N=15)"); return ctrl

    raise ValueError(f"Unknown controller: {name}")


# ─────────────────────────────────────────────────────────────────────
# Teleop
# ─────────────────────────────────────────────────────────────────────
@dataclass
class TeleopState:
    vx: float = 0.0; vy: float = 0.0; wz: float = 0.0
    step_lin: float = 0.05; step_ang: float = 0.15
    max_vx: float = 0.60; max_vy: float = 0.30; max_wz: float = 1.00
    quit_requested: bool = False

    def clamp(self):
        self.vx = float(np.clip(self.vx, -self.max_vx, self.max_vx))
        self.vy = float(np.clip(self.vy, -self.max_vy, self.max_vy))
        self.wz = float(np.clip(self.wz, -self.max_wz, self.max_wz))

    def zero(self): self.vx = self.vy = self.wz = 0.0


def teleop_keyboard_loop(teleop: TeleopState):
    import termios, tty
    fd = sys.stdin.fileno(); old = termios.tcgetattr(fd)
    print("\n[Teleop] UP/DOWN:fwd/bwd  LEFT/RIGHT:yaw  z/c:lateral  space:stop\n")
    try:
        tty.setcbreak(fd)
        while not teleop.quit_requested:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                ch = sys.stdin.read(1)
                if ch == "\x1b":
                    s1, s2 = sys.stdin.read(1), sys.stdin.read(1)
                    if s1 == "[":
                        if   s2 == "A": teleop.vx += teleop.step_lin
                        elif s2 == "B": teleop.vx -= teleop.step_lin
                        elif s2 == "C": teleop.wz -= teleop.step_ang
                        elif s2 == "D": teleop.wz += teleop.step_ang
                elif ch == "z": teleop.vy += teleop.step_lin
                elif ch == "c": teleop.vy -= teleop.step_lin
                elif ch == " ": teleop.zero()
                teleop.clamp()
                print(f"\rcmd vx={teleop.vx:+.2f} vy={teleop.vy:+.2f} "
                      f"wz={teleop.wz:+.2f}   ", end="", flush=True)
    except KeyboardInterrupt:
        teleop.quit_requested = True
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old); print()


# ─────────────────────────────────────────────────────────────────────
# Plotting
# ─────────────────────────────────────────────────────────────────────
def _dist_span(ax, dist_type, t_end, dist_t0):
    if dist_type == "impulse":
        ax.axvspan(dist_t0, dist_t0+0.15, alpha=0.20, color="orange",
                   label="impulse")
    elif dist_type == "persistent":
        ax.axvspan(dist_t0, t_end, alpha=0.08, color="orange",
                   label="persistent dist.")
    ax.axvline(T_WARMUP, ls=":", color="purple", lw=1.0, alpha=0.7,
               label="walk start")


def save_single_run_plot(result, ctrl_name, robot_name, dist_type,
                          x_ref_nom, waypoints=None, dist_t0=None):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs("results", exist_ok=True)
    lt      = result["time"]
    lx      = result["state"]
    lu      = result["u_grf"]
    ld      = result["disturbance"]
    t_end   = lt[-1] if len(lt) > 0 else 1.0
    dist_t0 = dist_t0 if dist_t0 is not None else T_WARMUP + 2.0
    has_wp  = waypoints is not None
    n_rows  = 5 if has_wp else 4

    fig, axes = plt.subplots(n_rows, 1, figsize=(14, 3*n_rows))
    fig.suptitle(f"{ctrl_name.upper()} — {robot_name} — {dist_type}",
                 fontsize=14, fontweight="bold")

    # Panel 1 — Posición
    ax = axes[0]
    cp = ["#e74c3c", "#2ecc71", "#3498db"]
    for i, (lb, c) in enumerate(zip([r"$p_x$", r"$p_y$", r"$p_z$"], cp)):
        ax.plot(lt, lx[:, i], color=c, lw=1.4, label=lb)
        ax.axhline(x_ref_nom[i], ls="--", color=c, lw=0.7, alpha=0.5)
    ax.set_ylabel("Position [m]"); ax.legend(ncol=3, fontsize=8)
    ax.grid(True, alpha=0.3); _dist_span(ax, dist_type, t_end, dist_t0)
    ax.set_xlim(0, t_end)

    # Panel 2 — Velocidad
    ax = axes[1]
    for i, (lb, c) in enumerate(zip([r"$v_x$", r"$v_y$", r"$v_z$"], cp)):
        ax.plot(lt, lx[:, 3+i], color=c, lw=1.4, label=lb)
    ax.set_ylabel("Velocity [m/s]"); ax.legend(ncol=3, fontsize=8)
    ax.grid(True, alpha=0.3); _dist_span(ax, dist_type, t_end, dist_t0)
    ax.set_xlim(0, t_end)

    # Panel 3 — Orientación
    ax = axes[2]
    co = ["#e67e22", "#9b59b6", "#1abc9c"]
    for i, (lb, c) in enumerate(zip(["roll", "pitch", "yaw"], co)):
        ax.plot(lt, np.degrees(lx[:, 6+i]), color=c, lw=1.4, label=lb)
    ax.set_ylabel("Orientation [°]"); ax.legend(ncol=3, fontsize=8)
    ax.grid(True, alpha=0.3); _dist_span(ax, dist_type, t_end, dist_t0)
    ax.set_xlim(0, t_end)

    # Panel 4 — GRFs + perturbación
    ax = axes[3]
    u_norm = np.linalg.norm(lu, axis=1)
    ax.plot(lt, u_norm, color="#2c3e50", lw=1.4, label="||GRFs||")
    ax.fill_between(lt, 0, ld*2, alpha=0.30, color="orange",
                    label="disturbance ×2")
    ax.set_ylabel("Force [N]"); ax.legend(ncol=2, fontsize=8)
    ax.grid(True, alpha=0.3); _dist_span(ax, dist_type, t_end, dist_t0)
    ax.set_xlim(0, t_end)
    if not has_wp:
        ax.set_xlabel("Time [s]")

    # Panel 5 — Trayectoria XY
    if has_wp:
        ax = axes[4]
        ax.plot(lx[:, 0], lx[:, 1], color="#2c3e50", lw=1.3,
                label="actual path")
        ref = result.get("pos_ref")
        if ref is not None and len(ref) > 1:
            ax.plot(ref[:, 0], ref[:, 1], "--", color="#7f8c8d",
                    lw=0.9, label="reference")
        wp = np.array(waypoints)
        ax.scatter(wp[:, 0], wp[:, 1], c="red", zorder=5, s=70)
        for k, w in enumerate(wp):
            ax.annotate(f"WP{k+1}", (w[0], w[1]),
                        xytext=(6, 6), textcoords="offset points", fontsize=8)
        ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
        ax.set_title("XY Trajectory", fontsize=10)
        ax.set_aspect("equal")
        ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = f"results/mujoco_{ctrl_name}_{robot_name}_{dist_type}.png"
    plt.savefig(path, dpi=150); plt.close()
    print(f"  Plot saved: {path}")


def save_comparison_plot(results, robot_name, dist_type,
                          waypoints=None, dist_t0=None):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs("results", exist_ok=True)
    colors  = {"pmp": "#e74c3c", "lqg": "#2ecc71", "mpc": "#3498db"}
    dist_t0 = dist_t0 if dist_t0 is not None else T_WARMUP + 2.0

    fig = plt.figure(figsize=(16, 10))
    gs  = fig.add_gridspec(2, 3, hspace=0.45, wspace=0.35)
    ax_pos = fig.add_subplot(gs[0, 0])
    ax_vel = fig.add_subplot(gs[0, 1])
    ax_grf = fig.add_subplot(gs[0, 2])
    ax_xy  = fig.add_subplot(gs[1, :])
    fig.suptitle(f"Controller Comparison — {robot_name} — {dist_type}",
                 fontsize=14, fontweight="bold")

    t_end = 0.0
    for name, d in results.items():
        t   = d["time"]; x = d["state"]; u = d["u_grf"]; c = colors[name]
        t_end = max(t_end, t[-1])

        pos_err = np.linalg.norm(
            x[:, :3] - np.array([0., 0., ROBOT_HIP_HEIGHT]), axis=1)
        vel_err = np.linalg.norm(x[:, 3:6], axis=1)
        u_norm  = np.linalg.norm(u, axis=1)

        ax_pos.plot(t, pos_err, color=c, label=name.upper(), lw=1.5)
        ax_vel.plot(t, vel_err, color=c, lw=1.5)
        ax_grf.plot(t, u_norm,  color=c, lw=1.3)
        ax_xy.plot(x[:, 0], x[:, 1], color=c, label=name.upper(), lw=1.4)

    ax_pos.set_title("Position error [m]");  ax_pos.set_xlabel("Time [s]")
    ax_vel.set_title("Velocity [m/s]");      ax_vel.set_xlabel("Time [s]")
    ax_grf.set_title("||GRFs|| [N]");        ax_grf.set_xlabel("Time [s]")
    ax_pos.legend(fontsize=9)
    for ax in [ax_pos, ax_vel, ax_grf]:
        ax.grid(True, alpha=0.3)
        _dist_span(ax, dist_type, t_end, dist_t0)
        ax.set_xlim(0, t_end)

    if waypoints is not None:
        wp = np.array(waypoints)
        ax_xy.scatter(wp[:, 0], wp[:, 1], c="red", zorder=5, s=70)
        for k, w in enumerate(wp):
            ax_xy.annotate(f"WP{k+1}", (w[0], w[1]),
                           xytext=(6, 6), textcoords="offset points",
                           fontsize=9)

    first = next(iter(results.values()), None)
    if first is not None:
        ref = first.get("pos_ref")
        if ref is not None and len(ref) > 1:
            ax_xy.plot(ref[:, 0], ref[:, 1], "k--", lw=0.8,
                       alpha=0.5, label="reference")

    ax_xy.set_title("XY Trajectory")
    ax_xy.set_xlabel("X [m]"); ax_xy.set_ylabel("Y [m]")
    ax_xy.set_aspect("equal")
    ax_xy.legend(fontsize=9); ax_xy.grid(True, alpha=0.3)

    plt.tight_layout()
    path = f"results/mujoco_comparison_{robot_name}_{dist_type}.png"
    plt.savefig(path, dpi=150); plt.close()
    print(f"  Comparison plot saved: {path}")


# ─────────────────────────────────────────────────────────────────────
# Main run loop
# ─────────────────────────────────────────────────────────────────────
def run(controller_name, robot_name="mini_cheetah", teleop_enabled=False,
        render=True, duration=40.0, disturbance_type="impulse",
        save_log=True, waypoints=None):

    print(f"\n{'='*60}")
    print(f"  Controller : {controller_name.upper()}")
    print(f"  Robot      : {robot_name}")
    print(f"  Duration   : {duration}s  |  Disturbance: {disturbance_type}")
    print(f"  Waypoints  : {len(waypoints) if waypoints else 'none'}")
    print(f"{'='*60}")
    print(f"  Gait v7: period=0.35s  swing=0.50  h=0.13m  "
          f"step=[0.060,0.150]m  ramp=0.40s  max_vx=0.45m/s\n")

    env = QuadrupedEnv(
        robot=robot_name, scene="flat", sim_dt=0.002,
        base_vel_command_type="human",
        state_obs_names=tuple(QuadrupedEnv.ALL_OBS),
    )
    _ = env.reset(random=False)
    if render:
        env.render()

    q_init  = capture_initial_joints(env)
    t_reset = 0.0

    print("  Initial joints:")
    for leg in LEG_NAMES:
        print(f"    {leg}: {np.array2string(q_init[leg], precision=3)}")
    print()

    teleop = TeleopState()
    if teleop_enabled:
        threading.Thread(target=teleop_keyboard_loop,
                         args=(teleop,), daemon=True).start()

    traj = None
    if waypoints is not None and not teleop_enabled:
        traj = WaypointGenerator(waypoints, distance_threshold=0.20)
        print(f"  [Traj] {len(waypoints)} waypoints\n")

    dyn        = build_dynamics()
    Q, R, Q_f  = build_cost_matrices()
    x_ref0     = build_reference_state(dyn, ROBOT_HIP_HEIGHT)
    u_ref      = dyn.standing_control()
    controller = build_controller(controller_name, dyn, Q, R, Q_f, x_ref0)
    ori_ekf    = OrientationEKF(dt=env.mjModel.opt.timestep)

    gait = TrotGait(
        period=0.35, swing_ratio=0.50, step_height=0.13,
        step_len_min=0.060, step_len_max=0.150,
        step_multiplier=0.95, kp=32.0, kd=5.0, ramp_time=0.40,
    )

    sim_dt  = env.mjModel.opt.timestep
    ctrl_dt = 0.01
    ctrl_steps = max(1, int(ctrl_dt / sim_dt))
    n_steps = int(duration / sim_dt)

    # ── Butterworth filter sobre GRFs (restaurado) ─────────────────────
    fs_ctrl = 1.0 / ctrl_dt          # 100 Hz
    fc_ctrl = 15.0                    # corte más alto para no atenuar gait
    sos     = make_butter_sos(fc_ctrl, fs_ctrl, order=2)
    zi      = np.zeros((sos.shape[0], 2, 12))
    for ch in range(12):
        zi[:, :, ch] = sosfilt_zi(sos) * u_ref[ch]
    u_filtered = u_ref.copy()

    ref_pos_x = float(env.base_pos[0])
    ref_pos_y = float(env.base_pos[1])
    ref_yaw   = float(env.base_ori_euler_xyz[2])
    dist_t0   = T_WARMUP + 2.0

    log_t=[]; log_x=[]; log_ugrf=[]; log_err=[]; log_dist=[]
    log_pos_ref=[]; log_pos_actual=[]

    # u_grf actual del controlador (se actualiza en cada ctrl step)
    current_u_grf = u_ref.copy()

    try:
        for step in range(n_steps):
            t       = step * sim_dt
            t_local = t - t_reset
            x       = get_state(env)
            contact = get_contacts(env)
            r_feet  = get_feet_world(env)

            # ── EKF ───────────────────────────────────────────────────
            gyro        = env.base_ang_vel(frame="base")
            accel_world = env.base_lin_acc(frame="world")
            R_WB        = env.base_configuration[:3, :3]
            accel_body  = R_WB.T @ (accel_world - np.array([0., 0., -9.81]))
            ori_ekf.predict(gyro)
            ori_ekf.update_accel(accel_body)

            # ── Comandos de velocidad ──────────────────────────────────
            if t_local < T_WARMUP:
                cmd_vx = cmd_vy = cmd_wz = 0.0
                x_ref  = build_reference_state(dyn, ROBOT_HIP_HEIGHT)
            elif teleop_enabled:
                cmd_vx, cmd_vy, cmd_wz = teleop.vx, teleop.vy, teleop.wz
                x_ref  = build_reference_state(dyn, ROBOT_HIP_HEIGHT,
                                               vx=cmd_vx, wz=cmd_wz)
            else:
                if traj is not None:
                    cmd_vx_wp, cmd_vy_wp, cmd_wz_wp = waypoint_command(
                        traj, x, ref_yaw)
                else:
                    cmd_vx_wp = cmd_vy_wp = cmd_wz_wp = 0.0

                x_ref = build_reference_state(dyn, ROBOT_HIP_HEIGHT,
                                              vx=cmd_vx_wp, wz=cmd_wz_wp)
                x_ref[0] = ref_pos_x
                x_ref[1] = ref_pos_y
                x_ref[8] = ref_yaw

                # ── Controlador óptimo (cada ctrl_dt) ─────────────────
                if step % ctrl_steps == 0:
                    # Re-linearizar alrededor del estado actual
                    try:
                        Ac, Bc = dyn.continuous_AB(x, contact, r_feet)
                        Ad, Bd = dyn.discretize(Ac, Bc)
                        gd     = dyn.gravity_vector()
                        try:
                            controller.update_model(Ad, Bd, gd)
                        except AttributeError:
                            pass
                    except Exception:
                        pass

                    # Calcular GRFs del controlador
                    u_raw = controller_compute(
                        controller_name, controller, x, x_ref, u_ref)

                    # ── Butterworth sobre GRFs ─────────────────────────
                    u_raw = np.clip(u_raw, -150.0, 150.0)
                    col   = u_raw.reshape(1, 12)
                    for ch in range(12):
                        out, zi[:, :, ch] = sosfilt(sos, col[:, ch],
                                                     zi=zi[:, :, ch])
                        u_filtered[ch] = out[0]
                    current_u_grf = u_filtered.copy()

                # Modular velocidad con la señal filtrada
                cmd_vx, cmd_vy, cmd_wz = modulate_velocity(
                    current_u_grf, x, x_ref, cmd_vx_wp, cmd_wz_wp)

            # ── Referencia de posición integrada ───────────────────────
            ref_yaw   += cmd_wz * sim_dt
            ref_pos_x += cmd_vx * np.cos(ref_yaw) * sim_dt
            ref_pos_y += cmd_vx * np.sin(ref_yaw) * sim_dt
            x_ref_log  = build_reference_state(dyn, ROBOT_HIP_HEIGHT,
                                               vx=cmd_vx, wz=cmd_wz)
            x_ref_log[0] = ref_pos_x
            x_ref_log[1] = ref_pos_y
            x_ref_log[8] = ref_yaw

            try:
                if hasattr(env, "target_base_vel"):
                    env.target_base_vel[:] = [cmd_vx, cmd_vy, 0.0]
                if hasattr(env, "ref_base_lin_vel"):
                    env.ref_base_lin_vel = cmd_vx
            except Exception:
                pass

            # ── Perturbación ───────────────────────────────────────────
            dist = np.zeros(6)
            if disturbance_type == "impulse" and dist_t0 <= t < dist_t0+0.15:
                dist = np.array([50.0, 25.0, 0.0, 0.0, 0.0, 5.0])
            elif disturbance_type == "persistent" and t >= dist_t0:
                dist = np.array([10.0, 5.0, 0.0, 0.0, 0.0, 1.5])
            env.mjData.qfrc_applied[:6] = dist

            # ── Torques articulares ────────────────────────────────────
            gait.filter_commands(cmd_vx, cmd_wz, sim_dt)

            if t_local < T_WARMUP:
                tau = warmup_torques(env, q_init)
            else:
                gait.start_walking(t)
                tau = gait.compute_all_torques(
                    t, env, cmd_vx, cmd_vy, cmd_wz, q_init)

            _, _, terminated, _, _ = env.step(action=tau)
            if render:
                env.render()

            # ── FIX: error de velocidad (más representativo que posición)
            # RMSE ahora mide qué tan bien se sigue la velocidad comandada
            vel_ref = np.array([cmd_vx, cmd_vy, 0.0])
            vel_err = np.linalg.norm(x[3:6] - vel_ref)
            height_err = abs(x[2] - ROBOT_HIP_HEIGHT)
            tracking_err = vel_err + height_err   # error compuesto

            log_t.append(t)
            log_x.append(x.copy())
            log_ugrf.append(current_u_grf.copy())   # FIX: u real, no u_ref
            log_err.append(float(tracking_err))
            log_dist.append(float(np.linalg.norm(dist)))
            log_pos_ref.append(x_ref_log[:3].copy())
            log_pos_actual.append(x[:3].copy())

            if step % int(1.0 / sim_dt) == 0:
                if t_local < T_WARMUP:
                    lbl = "WARMUP"
                elif traj and traj.is_finished:
                    lbl = "DONE  "
                elif traj:
                    idx, tot, _ = traj.progress()
                    lbl = f"WP{idx+1}/{tot}"
                else:
                    lbl = "WALK  "
                ramp_val = gait._ramp(t) if t_local >= T_WARMUP else 0.0
                step_vis = float(np.clip(
                    abs(gait._vx_f) * gait.period * gait.step_multiplier,
                    gait.step_len_min, gait.step_len_max))
                print(
                    f"  t={t:5.1f}s [{lbl}]  h={x[2]:.3f}m  "
                    f"pos=({x[0]:+.2f},{x[1]:+.2f})  "
                    f"vx_cmd={cmd_vx:+.3f} vx_f={gait._vx_f:+.3f}  "
                    f"wz={cmd_wz:+.3f}  ramp={ramp_val:.2f}  "
                    f"step={step_vis:.3f}m  vel_err={vel_err:.3f}"
                )

            if terminated:
                print(f"  !! Terminated t={t:.2f}s — resetting.")
                _ = env.reset(random=False)
                if render: env.render()
                t_reset    = t + sim_dt
                q_init     = capture_initial_joints(env)
                ref_pos_x  = float(env.base_pos[0])
                ref_pos_y  = float(env.base_pos[1])
                ref_yaw    = float(env.base_ori_euler_xyz[2])
                gait._vx_f = 0.0; gait._wz_f = 0.0
                gait._t_walk_start = None
                # Reiniciar zi del filtro
                for ch in range(12):
                    zi[:, :, ch] = sosfilt_zi(sos) * u_ref[ch]
                u_filtered   = u_ref.copy()
                current_u_grf = u_ref.copy()

    except KeyboardInterrupt:
        print("\n  Interrupted.")
    finally:
        teleop.quit_requested = True
        env.close()

    def _a(lst, sh): return np.array(lst) if lst else np.zeros(sh)
    log_t          = _a(log_t,          (1,))
    log_x          = _a(log_x,          (1, 12))
    log_ugrf       = _a(log_ugrf,       (1, 12))
    log_err        = _a(log_err,        (1,))
    log_dist       = _a(log_dist,       (1,))
    log_pos_ref    = _a(log_pos_ref,    (1, 3))
    log_pos_actual = _a(log_pos_actual, (1, 3))

    result = dict(time=log_t, state=log_x, u_grf=log_ugrf,
                  error=log_err, disturbance=log_dist,
                  pos_ref=log_pos_ref, pos_actual=log_pos_actual)

    if save_log and len(log_t) > 1:
        save_single_run_plot(
            result, controller_name, robot_name, disturbance_type,
            build_reference_state(dyn, ROBOT_HIP_HEIGHT),
            waypoints=waypoints, dist_t0=dist_t0,
        )

    tvu = np.sum(np.linalg.norm(np.diff(log_ugrf, axis=0), axis=1))
    print(f"\n  --- {controller_name.upper()} Summary ---")
    print(f"  RMSE (vel+height err) = {np.sqrt(np.mean(log_err**2)):.4f}")
    print(f"  MaxE                  = {np.max(log_err):.4f}")
    print(f"  TVU                   = {tvu:.1f}")
    print(f"  Mean ||GRFs||         = {np.mean(np.linalg.norm(log_ugrf, axis=1)):.1f} N")
    return result


# ─────────────────────────────────────────────────────────────────────
# Comparison
# ─────────────────────────────────────────────────────────────────────
def run_comparison(render, duration, dist_type, robot_name, waypoints):
    results = {}
    for name in ["pmp", "lqg", "mpc"]:
        results[name] = run(name, robot_name=robot_name, teleop_enabled=False,
                            render=render, duration=duration,
                            disturbance_type=dist_type, save_log=True,
                            waypoints=waypoints)

    dist_t0 = T_WARMUP + 2.0
    save_comparison_plot(results, robot_name, dist_type,
                          waypoints=waypoints, dist_t0=dist_t0)

    print(f"\n{'='*65}")
    print(f"  COMPARISON SUMMARY ({dist_type})")
    print(f"{'='*65}")
    print(f"  {'Controller':<15} {'RMSE':>8} {'MaxE':>8} {'TVU':>12} {'Mean||u||':>12}")
    print(f"  {'-'*55}")
    for name, d in results.items():
        e    = d["error"]; u = d["u_grf"]
        rmse = np.sqrt(np.mean(e**2)); maxe = np.max(e)
        tvu  = np.sum(np.linalg.norm(np.diff(u, axis=0), axis=1))
        mu   = np.mean(np.linalg.norm(u, axis=1))
        print(f"  {name.upper():<15} {rmse:8.4f} {maxe:8.4f} {tvu:12.1f} {mu:12.1f}")
    print(f"{'='*65}")


# ─────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description="Quadruped locomotion — joint PD trot v7 + optimal controller")
    p.add_argument("--controller", default="lqg",
                   choices=["pmp", "lqg", "mpc", "all"])
    p.add_argument("--robot-name", default="mini_cheetah")
    p.add_argument("--teleop", action="store_true")
    p.add_argument("--duration", type=float, default=40.0)
    p.add_argument("--disturbance", default="impulse",
                   choices=["impulse", "persistent", "none"])
    p.add_argument("--no-render", action="store_true")
    p.add_argument("--no-waypoints", action="store_true",
                   help="Deshabilita waypoints — camina libre")
    args = p.parse_args()

    waypoints = None if args.no_waypoints else DEFAULT_WAYPOINTS

    if args.controller == "all":
        run_comparison(not args.no_render, args.duration,
                       args.disturbance, args.robot_name, waypoints)
    else:
        run(args.controller, args.robot_name, args.teleop,
            not args.no_render, args.duration, args.disturbance,
            True, waypoints=waypoints)
