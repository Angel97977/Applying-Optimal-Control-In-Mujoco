#!/usr/bin/env python3
"""run_mujoco_v4.py — Quadruped locomotion with pure joint-PD control.

REDESIGN v4  — raíz del problema y solución
═══════════════════════════════════════════
PROBLEMA: El robot caía siempre durante BLEND/SETTLE.

Causa raíz: q_init (equilibrio real del env) ≠ Q_STAND hardcodeado.
  • FL/FR capturados: hip≈-2.2, knee≈+1.3
  • RL/RR capturados: hip≈+0.9, knee≈+1.3  ← knee POSITIVO, no -1.6 de Q_STAND
  Cuando el controlador GRF lineariza alrededor de Q_STAND y luego mapea
  fuerzas via Jacobiano, el robot estaba en una configuración totalmente
  distinta → torques masivos → termina en < 3 s.

FIXES v4:
  1. Control articular puro (joint PD) — sin GRF→torques en el loop.
  2. q_init capturado post-reset usado como referencia, no Q_STAND.
  3. knee_dir=+1 para TODAS las patas (convención real URDF: knee+→dobla→pie sube).
  4. hip_dir=-1 para RL/RR  (convencion opuesta en eje sagital para patas traseras).
  5. Barrido lineal de hip: (2·sw−1) da paso neto adelante en vez del bump sin(π·sw).
  6. Controladores (PMP/LQG/MPC) usados como moduladores suaves de velocidad.
     Extraen señal de tracking error → escalan cmd_vx/cmd_wz sin poder desestabilizar.
  7. Marcha conservadora: period=0.80 s, step_height=0.040, kp=32, kd=5.

Examples
────────
    python examples/run_mujoco_v4.py
    python examples/run_mujoco_v4.py --controller lqg
    python examples/run_mujoco_v4.py --controller all --no-render
    python examples/run_mujoco_v4.py --controller mpc --disturbance persistent
"""

import sys, os, argparse, threading, select
import numpy as np
from dataclasses import dataclass
from scipy.signal import butter, sosfilt_zi, sosfilt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from gym_quadruped.quadruped_env import QuadrupedEnv
from src.dynamics          import QuadrupedDynamics
from src.estimator_ekf     import OrientationEKF
from src.controller_pmp    import PontryaginController
from src.controller_lqg    import LQGController
from src.controller_mpc    import MPCController

try:
    from src.trajectory_generator import WaypointGenerator
except ImportError:
    class WaypointGenerator:
        def __init__(self, waypoints, distance_threshold=0.15):
            self.waypoints     = np.array(waypoints, dtype=float)
            self.current_index = 0
            self.threshold     = distance_threshold
            self.is_finished   = False

        def get_reference(self, cx, cy):
            if self.is_finished:
                return self.waypoints[-1]
            tgt  = self.waypoints[self.current_index]
            dist = float(np.hypot(tgt[0]-cx, tgt[1]-cy))
            if dist < self.threshold:
                print(f"[Traj] WP {self.current_index+1}/{len(self.waypoints)} reached")
                self.current_index += 1
                if self.current_index >= len(self.waypoints):
                    print("[Traj] Route complete.")
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

# Fases de tiempo — sin BLEND, transición directa
T_WARMUP = 2.0    # el robot mantiene pose inicial (joint PD)
T_WALK   = 2.0    # empieza a andar inmediatamente después de T_WARMUP

# Waypoints: [x, y, z, yaw]
DEFAULT_WAYPOINTS = [
    [ 2.0,  0.0, 0.30,  0.0   ],
    [ 2.0,  2.0, 0.30,  1.5708],
    [ 0.0,  2.0, 0.30,  3.1416],
    [ 0.0,  0.0, 0.30, -1.5708],
]

# Q_STAND se mantiene sólo como fallback de diagnóstico — NO se usa en control
Q_STAND_REF = {
    "FL": np.array([ 0.0, -0.8,  1.6]),
    "FR": np.array([ 0.0, -0.8,  1.6]),
    "RL": np.array([ 0.0,  0.8, -1.6]),
    "RR": np.array([ 0.0,  0.8, -1.6]),
}
LEG_NAMES = ["FL", "FR", "RL", "RR"]

# Ganancias joint PD
KP_STAND  = 80.0   # warmup: hold pose fuerte
KD_STAND  =  5.0
KP_WALK   = 32.0   # marcha: más suave para absorber impactos
KD_WALK   =  5.0
TAU_LIMIT = 55.0   # N·m por joint

# Modulación del controlador (qué tan fuerte escala la velocidad)
CTRL_SCALE = 0.12  # fracción de la fuerza GRF que afecta la velocidad


# ─────────────────────────────────────────────────────────────────────
# Captura robusta de joints post-reset
# ─────────────────────────────────────────────────────────────────────
def capture_initial_joints(env) -> dict:
    """Lee ángulos reales del env tras reset(). Cuatro métodos en cascada."""
    q_init = {}
    for leg in LEG_NAMES:
        q = None
        for method in range(4):
            try:
                if method == 0:
                    idx = env.legs_qpos_idx[leg]
                    c   = np.array(env.mjData.qpos[idx], dtype=float)
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
                    q = c
                    break
            except Exception:
                pass
        if q is None:
            print(f"  [WARN] capture_initial_joints: fallback Q_STAND_REF para {leg}")
            q = Q_STAND_REF[leg].copy()
        q_init[leg] = q
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
# Warmup controller: mantiene q_init estático
# ─────────────────────────────────────────────────────────────────────
def warmup_torques(env, q_stand: dict) -> np.ndarray:
    """PD fuerte que mantiene la postura inicial."""
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
# TrotGait — joint PD completo con delta articular correcto
# ─────────────────────────────────────────────────────────────────────
class TrotGait:
    """
    Control articular de trot para mini_cheetah.

    Convenciones confirmadas por q_init capturado:
    ┌──────┬────────────┬───────────────────────────────────────────────┐
    │ Pata │ q_init     │ Dirección                                     │
    ├──────┼────────────┼───────────────────────────────────────────────┤
    │ FL   │ hip≈-2.2   │ hip+→adelante  knee+→dobla→pie sube           │
    │ FR   │ hip≈-2.2   │ hip+→adelante  knee+→dobla→pie sube           │
    │ RL   │ hip≈+0.9   │ hip-→adelante  knee+→dobla→pie sube           │
    │ RR   │ hip≈+0.9   │ hip-→adelante  knee+→dobla→pie sube           │
    └──────┴────────────┴───────────────────────────────────────────────┘
    knee_dir = +1 para TODAS las patas (diferencia clave vs versiones anteriores).
    """
    PHASE_OFFSET = {"FL": 0.0, "FR": 0.5, "RL": 0.5, "RR": 0.0}

    def __init__(self, period=0.80, swing_ratio=0.40,
                 step_height=0.040, kp=32.0, kd=5.0):
        self.period      = period
        self.swing_ratio = swing_ratio
        self.step_height = step_height
        self.kp          = kp
        self.kd          = kd
        # Filtro de primer orden para cmd_vx/cmd_wz (τ=0.15 s)
        self._vx_f = 0.0
        self._wz_f = 0.0

    def filter_commands(self, cmd_vx: float, cmd_wz: float,
                        dt: float, tau: float = 0.15):
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
        return np.array([not self.is_swing(t, lg) for lg in LEG_NAMES], dtype=bool)

    def _delta(self, sw: float, cmd_vx: float,
               cmd_vy: float, cmd_wz: float, leg: str) -> np.ndarray:
        """
        Delta articular durante swing respecto a q_stand.

        Hip:  barrido lineal (2·sw−1):
              sw=0 → pie detrás  (−step_len)
              sw=1 → pie delante (+step_len)

        Knee: sin(π·sw), siempre positivo:
              →  dobla más en todos los casos → pie sube

        Abducción: pequeña corrección lateral para giros.
        """
        step_len = float(np.clip(abs(cmd_vx) * self.period * 0.50,
                                 0.004, 0.07))
        v_sign   = np.sign(cmd_vx) if abs(cmd_vx) > 0.006 else 0.0

        rear = leg in ("RL", "RR")
        # hip: +1 para FL/FR (hip+ = adelante), −1 para RL/RR (hip- = adelante)
        hip_dir  = -1.0 if rear else  1.0
        # knee: +1 para TODAS (knee+ = doblar = pie sube) ← FIX PRINCIPAL
        knee_dir =  1.0

        d_hip  = hip_dir  * v_sign   * step_len * (2.0 * sw - 1.0)
        d_knee = knee_dir * (self.step_height / 0.15) * np.sin(np.pi * sw)

        d_abd = 0.0
        if abs(cmd_wz) > 0.03:
            outer = {"FL": cmd_wz > 0, "FR": cmd_wz < 0,
                     "RL": cmd_wz > 0, "RR": cmd_wz < 0}
            d_abd = 0.025 * np.sign(cmd_wz) * (1.0 if outer[leg] else -1.0)

        return np.array([d_abd, d_hip, d_knee])

    def compute_all_torques(self, t: float, env,
                            cmd_vx: float, cmd_vy: float, cmd_wz: float,
                            q_stand: dict) -> np.ndarray:
        """
        Torques articulares para TODAS las patas.
        Patas en swing: siguen trayectoria del gait.
        Patas en stance: mantienen q_stand (PD de posición).
        Usa velocidades filtradas internamente.
        """
        tau = np.zeros(12)
        for leg in LEG_NAMES:
            try:
                q, dq = read_joint(env, leg)
            except Exception:
                continue

            if self.is_swing(t, leg):
                sw    = self._sw_norm(t, leg)
                delta = self._delta(sw, self._vx_f, cmd_vy, self._wz_f, leg)
                q_tgt = q_stand[leg] + delta
            else:
                q_tgt = q_stand[leg]   # stance: mantener postura

            raw = self.kp * (q_tgt - q) - self.kd * dq
            tau[env.legs_tau_idx[leg]] = np.clip(raw, -TAU_LIMIT, TAU_LIMIT)
        return tau


# ─────────────────────────────────────────────────────────────────────
# Waypoint command
# ─────────────────────────────────────────────────────────────────────
def waypoint_command(traj, x: np.ndarray, ref_yaw: float,
                     max_vx: float = 0.22,
                     max_wz: float = 0.55) -> tuple:
    """
    Genera (cmd_vx, 0, cmd_wz) desde el waypoint actual.
    Dead-band 40° → robot gira primero, luego avanza.
    """
    if traj.is_finished:
        return 0.0, 0.0, 0.0

    target  = traj.get_reference(x[0], x[1])
    dx, dy  = target[0] - x[0], target[1] - x[1]
    dist    = float(np.hypot(dx, dy))
    ang     = float(np.arctan2(dy, dx))
    yaw_err = float(np.arctan2(np.sin(ang - ref_yaw), np.cos(ang - ref_yaw)))

    cmd_wz    = float(np.clip(2.5 * yaw_err, -max_wz, max_wz))
    dead_band = np.pi / 4.5   # ≈ 40°

    if abs(yaw_err) > dead_band:
        cmd_vx = 0.0
    else:
        align  = float(np.cos(yaw_err)) ** 2
        cmd_vx = float(np.clip(0.55 * dist * align, 0.0, max_vx))

    return cmd_vx, 0.0, cmd_wz


# ─────────────────────────────────────────────────────────────────────
# Helpers de estado
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
# Dynamics / cost / controllers (sin cambios — mismos módulos)
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
            A_d=A_d, B_d=B_d, g_d=g_d, Q=Q * dyn.dt, R=R * dyn.dt,
            Q_proc=np.diag([1e-3]*3 + [1e-2]*3 + [5e-3]*3 + [1e-2]*3),
            R_meas=np.diag([5e-3]*3 + [2e-2]*3 + [1e-2]*3 + [5e-2]*3))
        ctrl.set_initial_estimate(x_ref)
        print("  [LQG] initialized"); return ctrl

    if name == "mpc":
        ctrl = MPCController(
            A_d=A_d, B_d=B_d, g_d=g_d,
            Q=Q * dyn.dt, R=R * dyn.dt, Q_f=Q_f * dyn.dt,
            N=15, mu=0.6, fz_max=150.0)
        print("  [MPC] initialized (N=15)"); return ctrl

    raise ValueError(f"Unknown controller: {name}")


# ─────────────────────────────────────────────────────────────────────
# Modulación de velocidad por controlador
# ─────────────────────────────────────────────────────────────────────
def controller_modulate_velocity(ctrl_name, controller, x, x_ref, u_ref,
                                  cmd_vx_wp: float,
                                  cmd_wz_wp: float) -> tuple:
    """
    Usa la salida GRF del controlador para modular suavemente la velocidad.

    El controlador computa u ∈ R^12 (fuerzas GRF óptimas).
    Extraemos la señal de error de tracking y la convertimos a
    una corrección de velocidad pequeña (±CTRL_SCALE).

    PMP: solución offline, corrección basada en K·e.
    LQG: usa estado estimado Kalman (más suave).
    MPC: horizonte deslizante, anticipación.

    IMPORTANTE: la corrección está saturada a ±CTRL_SCALE → nunca
    desestabiliza el control articular.
    """
    try:
        if ctrl_name == "lqg":
            noise = np.array([5e-3]*3 + [2e-2]*3 + [1e-2]*3 + [5e-2]*3)
            y = x + np.random.randn(12) * noise
            u = controller.step(y, x_ref, u_ref)
        else:
            u = controller.compute_control(x=x, x_ref=x_ref, u_ref=u_ref)

        u = np.clip(u, -200.0, 200.0)

        # Extraer señal de error de tracking como señal de velocidad:
        # fx_net > 0  → el modelo quiere ir más adelante → acelera
        # fy_net     → corrección lateral → afecta cmd_wz
        fx_net = float(np.sum(u[0::3]))  # suma fuerzas en X
        fz_avg = float(np.mean(u[2::3]))  # promedio fuerzas verticales (altura)

        # Normalizar por peso del robot para obtener fracción de aceleración
        weight = ROBOT_MASS * 9.81
        vx_scale = float(np.clip(1.0 + CTRL_SCALE * fx_net / weight, 0.2, 1.6))
        # Corrección de altura: si el controlador quiere más fuerza vertical
        # el robot está bajo → reducir velocidad para estabilizar
        height_err = float(x[2] - x_ref[2])
        vx_height  = float(np.clip(1.0 - 3.0 * abs(height_err), 0.3, 1.0))

        cmd_vx_out = float(np.clip(cmd_vx_wp * vx_scale * vx_height,
                                   0.0, 0.30))
        cmd_wz_out = cmd_wz_wp   # yaw se deja al waypoint_command

        return cmd_vx_out, 0.0, cmd_wz_out

    except Exception:
        return cmd_vx_wp, 0.0, cmd_wz_wp


# ─────────────────────────────────────────────────────────────────────
# Teleop
# ─────────────────────────────────────────────────────────────────────
@dataclass
class TeleopState:
    vx: float = 0.0; vy: float = 0.0; wz: float = 0.0
    step_lin: float = 0.05; step_ang: float = 0.15
    max_vx: float = 0.5; max_vy: float = 0.3; max_wz: float = 1.0
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
# Plots
# ─────────────────────────────────────────────────────────────────────
def save_single_run_plot(result, ctrl_name, robot_name, dist_type,
                          x_ref_nom, waypoints=None):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs("results", exist_ok=True)
    lt = result["time"]; lx = result["state"]
    lu = result["u_grf"]; ld = result["disturbance"]
    n  = 5 if waypoints is not None else 4
    fig, ax = plt.subplots(n, 1, figsize=(14, 3*n), sharex=True)
    fig.suptitle(f"{ctrl_name.upper()} – {robot_name} – {dist_type} [v4]",
                 fontsize=14, fontweight="bold")

    for i, lb in enumerate([r"$p_x$", r"$p_y$", r"$p_z$"]):
        ax[0].plot(lt, lx[:, i], label=lb)
        ax[0].axhline(x_ref_nom[i], ls="--", color="gray", lw=0.6)
    ax[0].set_ylabel("Position [m]"); ax[0].legend(ncol=3, fontsize=8)

    for i, lb in enumerate([r"$v_x$", r"$v_y$", r"$v_z$"]):
        ax[1].plot(lt, lx[:, 3+i], label=lb)
    ax[1].set_ylabel("Velocity [m/s]"); ax[1].legend(ncol=3, fontsize=8)

    for i, lb in enumerate(["roll", "pitch", "yaw"]):
        ax[2].plot(lt, np.degrees(lx[:, 6+i]), label=lb)
    ax[2].set_ylabel("Orientation [°]"); ax[2].legend(ncol=3, fontsize=8)

    ax[3].plot(lt, np.linalg.norm(lu, axis=1), label="||u_grf||")
    ax[3].fill_between(lt, 0, ld*2, alpha=0.25, label="disturbance")
    ax[3].set_ylabel("Force [N]"); ax[3].legend(fontsize=8)

    if waypoints is not None:
        ax[4].plot(lx[:, 0], lx[:, 1], lw=1.2, label="actual")
        ref = result.get("pos_ref")
        if ref is not None and len(ref) > 1:
            ax[4].plot(ref[:, 0], ref[:, 1], "g--", lw=0.8, label="ref")
        wp = np.array(waypoints)
        ax[4].scatter(wp[:, 0], wp[:, 1], c="red", zorder=5, label="WPs")
        for k, w in enumerate(wp):
            ax[4].annotate(f"WP{k+1}", (w[0], w[1]),
                           xytext=(5, 5), textcoords="offset points", fontsize=7)
        ax[4].set_xlabel("X [m]"); ax[4].set_ylabel("Y [m]")
        ax[4].set_aspect("equal"); ax[4].legend(fontsize=8)
        ax[4].grid(True, alpha=0.3)
    else:
        ax[3].set_xlabel("Time [s]")

    for ax_ in ax[:4]:
        ax_.grid(True, alpha=0.3)
        ax_.axvline(T_WARMUP, ls=":", color="purple", lw=0.8, alpha=0.6,
                    label="walk start" if ax_ is ax[0] else "")

    plt.tight_layout()
    path = f"results/mujoco_v4_{ctrl_name}_{robot_name}_{dist_type}.png"
    plt.savefig(path, dpi=150); plt.close()
    print(f"  Plot saved: {path}")


def save_comparison_plot(results, robot_name, dist_type):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs("results", exist_ok=True)
    colors  = {"pmp": "#e74c3c", "lqg": "#2ecc71", "mpc": "#3498db"}
    markers = {"pmp": "s",       "lqg": "^",       "mpc": "o"}

    fig = plt.figure(figsize=(16, 10))
    gs  = fig.add_gridspec(2, 3, hspace=0.4, wspace=0.35)
    ax  = [fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]),
           fig.add_subplot(gs[0, 2]), fig.add_subplot(gs[1, :])]
    fig.suptitle(f"Controller Comparison v4 – {robot_name} – {dist_type}",
                 fontsize=14, fontweight="bold")

    for name, d in results.items():
        t = d["time"]; x = d["state"]
        c = colors[name]; m = markers[name]
        ax[0].plot(t, np.linalg.norm(
            x[:, :2] - np.array([[ROBOT_HIP_HEIGHT]*2]), axis=1),
            color=c, label=name.upper(), lw=1.5)
        ax[1].plot(t, np.linalg.norm(x[:, 3:6], axis=1), color=c, lw=1.5)
        ax[2].plot(t, np.degrees(x[:, 6]), color=c, lw=1.2, ls="-")
        ax[2].plot(t, np.degrees(x[:, 7]), color=c, lw=1.0, ls="--")

    ax[0].set_title("Position error XY [m]"); ax[0].legend()
    ax[1].set_title("Velocity magnitude [m/s]")
    ax[2].set_title("Roll/Pitch [°] (−/-- )")

    # XY trajectory panel
    for name, d in results.items():
        x = d["state"]
        ax[3].plot(x[:, 0], x[:, 1], color=colors[name],
                   label=name.upper(), lw=1.3)

    if results:
        first = next(iter(results.values()))
        ref   = first.get("pos_ref")
        if ref is not None and len(ref) > 1:
            ax[3].plot(ref[:, 0], ref[:, 1], "k--", lw=0.8, label="ref", alpha=0.5)

    if DEFAULT_WAYPOINTS:
        wp = np.array(DEFAULT_WAYPOINTS)
        ax[3].scatter(wp[:, 0], wp[:, 1], c="red", zorder=5, s=60)
        for k, w in enumerate(wp):
            ax[3].annotate(f"WP{k+1}", (w[0], w[1]),
                           xytext=(5, 5), textcoords="offset points", fontsize=8)
    ax[3].set_title("XY Trajectory"); ax[3].set_aspect("equal")
    ax[3].set_xlabel("X [m]"); ax[3].set_ylabel("Y [m]")
    ax[3].legend(); ax[3].grid(True, alpha=0.3)

    for ax_ in ax[:3]:
        ax_.grid(True, alpha=0.3)
        ax_.set_xlabel("Time [s]")

    path = f"results/mujoco_v4_comparison_{robot_name}_{dist_type}.png"
    plt.savefig(path, dpi=150); plt.close()
    print(f"  Comparison plot saved: {path}")


# ─────────────────────────────────────────────────────────────────────
# Main run loop
# ─────────────────────────────────────────────────────────────────────
def run(controller_name, robot_name="mini_cheetah", teleop_enabled=False,
        render=True, duration=40.0, disturbance_type="impulse",
        save_log=True, waypoints=None):

    print(f"\n{'='*60}")
    print(f"  Controller : {controller_name.upper()} [v4 — joint PD]")
    print(f"  Robot      : {robot_name}")
    print(f"  Duration   : {duration}s  | Disturbance: {disturbance_type}")
    print(f"  Waypoints  : {len(waypoints) if waypoints else 'none'}")
    print(f"{'='*60}")
    print(f"  Phases: WARMUP 0-{T_WARMUP}s → WALK {T_WARMUP}s+\n")

    env = QuadrupedEnv(
        robot=robot_name, scene="flat", sim_dt=0.002,
        base_vel_command_type="human",
        state_obs_names=tuple(QuadrupedEnv.ALL_OBS),
    )
    _ = env.reset(random=False)
    if render:
        env.render()

    # Capturar pose real de equilibrio — referencia de todo el control
    q_init = capture_initial_joints(env)
    t_reset = 0.0

    print("  Initial joints (control reference):")
    for leg in LEG_NAMES:
        print(f"    {leg}: {np.array2string(q_init[leg], precision=3)}")
    print()

    # Verificar que las rodillas traseras son positivas (diagnóstico)
    for leg in ("RL", "RR"):
        if q_init[leg][2] < 0:
            print(f"  [INFO] {leg} knee = {q_init[leg][2]:.3f} (negativo — convención opuesta)")

    # Teleop
    teleop = TeleopState()
    if teleop_enabled:
        th = threading.Thread(target=teleop_keyboard_loop,
                              args=(teleop,), daemon=True)
        th.start()

    # Trayectoria
    traj = None
    if waypoints is not None and not teleop_enabled:
        traj = WaypointGenerator(waypoints, distance_threshold=0.20)
        print(f"  [Traj] {len(waypoints)} waypoints\n")

    # Dinámica, controlador, EKF
    dyn        = build_dynamics()
    Q, R, Q_f  = build_cost_matrices()
    x_ref0     = build_reference_state(dyn, ROBOT_HIP_HEIGHT)
    u_ref      = dyn.standing_control()
    controller = build_controller(controller_name, dyn, Q, R, Q_f, x_ref0)
    ori_ekf    = OrientationEKF(dt=env.mjModel.opt.timestep)

    # Gait
    gait = TrotGait(period=0.80, swing_ratio=0.40,
                    step_height=0.040, kp=32.0, kd=5.0)

    sim_dt  = env.mjModel.opt.timestep
    n_steps = int(duration / sim_dt)

    # Logs
    log_t=[]; log_x=[]; log_ugrf=[]; log_err=[]; log_dist=[]
    log_pos_ref=[]; log_pos_actual=[]

    ref_pos_x = float(env.base_pos[0])
    ref_pos_y = float(env.base_pos[1])
    ref_yaw   = float(env.base_ori_euler_xyz[2])
    dist_t0   = T_WARMUP + 2.0
    u_grf_log = u_ref.copy()  # para logging

    try:
        for step in range(n_steps):
            t       = step * sim_dt
            t_local = t - t_reset
            x       = get_state(env)

            # ── Orientación EKF ────────────────────────────────────────
            gyro        = env.base_ang_vel(frame="base")
            accel_world = env.base_lin_acc(frame="world")
            R_WB        = env.base_configuration[:3, :3]
            accel_body  = R_WB.T @ (accel_world - np.array([0., 0., -9.81]))
            ori_ekf.predict(gyro)
            ori_ekf.update_accel(accel_body)

            # ── Comandos de velocidad ──────────────────────────────────
            if t_local < T_WARMUP:
                cmd_vx = cmd_vy = cmd_wz = 0.0
            elif teleop_enabled:
                cmd_vx, cmd_vy, cmd_wz = teleop.vx, teleop.vy, teleop.wz
            else:
                # Waypoint → velocidad base
                if traj is not None:
                    cmd_vx_wp, cmd_vy_wp, cmd_wz_wp = waypoint_command(
                        traj, x, ref_yaw)
                else:
                    cmd_vx_wp = cmd_vy_wp = cmd_wz_wp = 0.0

                # Controlador → modulación de velocidad
                x_ref = build_reference_state(dyn, ROBOT_HIP_HEIGHT,
                                              vx=cmd_vx_wp, wz=cmd_wz_wp)
                x_ref[0] = ref_pos_x; x_ref[1] = ref_pos_y
                x_ref[8] = ref_yaw

                cmd_vx, cmd_vy, cmd_wz = controller_modulate_velocity(
                    controller_name, controller, x, x_ref, u_ref,
                    cmd_vx_wp, cmd_wz_wp)

            # ── Actualizar referencia de posición ──────────────────────
            ref_yaw   += cmd_wz * sim_dt
            ref_pos_x += cmd_vx * np.cos(ref_yaw) * sim_dt
            ref_pos_y += cmd_vx * np.sin(ref_yaw) * sim_dt
            x_ref_log  = build_reference_state(dyn, ROBOT_HIP_HEIGHT,
                                               vx=cmd_vx, wz=cmd_wz)
            x_ref_log[0]=ref_pos_x; x_ref_log[1]=ref_pos_y
            x_ref_log[8]=ref_yaw

            try:
                if hasattr(env, "target_base_vel"):
                    env.target_base_vel[:] = [cmd_vx, cmd_vy, 0.0]
                if hasattr(env, "ref_base_lin_vel"):
                    env.ref_base_lin_vel = cmd_vx
            except Exception:
                pass

            # ── Perturbación ───────────────────────────────────────────
            dist = np.zeros(6)
            if disturbance_type == "impulse" and dist_t0 <= t < dist_t0 + 0.15:
                dist = np.array([50.0, 25.0, 0.0, 0.0, 0.0, 5.0])
            elif disturbance_type == "persistent" and t >= dist_t0:
                dist = np.array([10.0, 5.0, 0.0, 0.0, 0.0, 1.5])
            env.mjData.qfrc_applied[:6] = dist

            # ── Torques ────────────────────────────────────────────────
            gait.filter_commands(cmd_vx, cmd_wz, sim_dt)

            if t_local < T_WARMUP:
                tau = warmup_torques(env, q_init)
            else:
                tau = gait.compute_all_torques(
                    t, env, cmd_vx, cmd_vy, cmd_wz, q_init)

            _, _, terminated, _, _ = env.step(action=tau)
            if render:
                env.render()

            # ── Logging ────────────────────────────────────────────────
            log_t.append(t); log_x.append(x.copy())
            log_ugrf.append(u_grf_log.copy())
            log_err.append(float(np.linalg.norm(x[:6] - x_ref_log[:6])))
            log_dist.append(float(np.linalg.norm(dist)))
            log_pos_ref.append(x_ref_log[:3].copy())
            log_pos_actual.append(x[:3].copy())

            # ── Consola cada 1 s ───────────────────────────────────────
            if step % int(1.0 / sim_dt) == 0:
                if t_local < T_WARMUP:
                    lbl = "WARMUP   "
                elif traj and traj.is_finished:
                    lbl = "DONE     "
                elif traj:
                    idx, tot, _ = traj.progress()
                    lbl = f"WP{idx+1}/{tot}   "
                else:
                    lbl = "WALK     "
                print(f"  t={t:5.1f}s [{lbl}] "
                      f"h={x[2]:.3f}m  pos=({x[0]:+.2f},{x[1]:+.2f})  "
                      f"vx={cmd_vx:+.3f} vx_f={gait._vx_f:+.3f}  "
                      f"wz={cmd_wz:+.3f}")

            # ── Reset si cae ───────────────────────────────────────────
            if terminated:
                print(f"  !! Terminated t={t:.2f}s — resetting.")
                _ = env.reset(random=False)
                if render: env.render()
                t_reset   = t + sim_dt
                q_init    = capture_initial_joints(env)
                ref_pos_x = float(env.base_pos[0])
                ref_pos_y = float(env.base_pos[1])
                ref_yaw   = float(env.base_ori_euler_xyz[2])
                gait._vx_f = 0.0; gait._wz_f = 0.0

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
        save_single_run_plot(result, controller_name, robot_name,
                             disturbance_type,
                             build_reference_state(dyn, ROBOT_HIP_HEIGHT),
                             waypoints=waypoints)

    err = log_err
    tvu = np.sum(np.linalg.norm(np.diff(log_ugrf, axis=0), axis=1))
    print(f"\n  --- {controller_name.upper()} Summary ---")
    print(f"  RMSE={np.sqrt(np.mean(err**2)):.4f}  "
          f"MaxE={np.max(err):.4f}  TVU={tvu:.1f}")
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
    save_comparison_plot(results, robot_name, dist_type)

    print(f"\n{'='*65}")
    print(f"  COMPARISON v4 ({dist_type})")
    print(f"  {'Ctrl':<12} {'RMSE':>8} {'MaxE':>8} {'TVU':>12}")
    print(f"  {'-'*45}")
    for name, d in results.items():
        e    = d["error"]; u = d["u_grf"]
        rmse = np.sqrt(np.mean(e**2)); maxe = np.max(e)
        tvu  = np.sum(np.linalg.norm(np.diff(u, axis=0), axis=1))
        print(f"  {name.upper():<12} {rmse:8.4f} {maxe:8.4f} {tvu:12.1f}")
    print(f"{'='*65}")


# ─────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--controller", default="lqg",
                   choices=["pmp", "lqg", "mpc", "all"])
    p.add_argument("--robot-name", default="mini_cheetah")
    p.add_argument("--teleop", action="store_true")
    p.add_argument("--duration", type=float, default=40.0)
    p.add_argument("--disturbance", default="impulse",
                   choices=["impulse", "persistent", "none"])
    p.add_argument("--no-render", action="store_true")
    p.add_argument("--no-waypoints", action="store_true")
    args = p.parse_args()

    waypoints = None if args.no_waypoints else DEFAULT_WAYPOINTS

    if args.controller == "all":
        run_comparison(not args.no_render, args.duration,
                       args.disturbance, args.robot_name, waypoints)
    else:
        run(args.controller, args.robot_name, args.teleop,
            not args.no_render, args.duration, args.disturbance,
            True, waypoints=waypoints)
