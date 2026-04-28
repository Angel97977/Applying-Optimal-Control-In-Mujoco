# Controller Implementation Guide

## Detailed Guide to Each Controller

This document provides implementation-level details for understanding and modifying each controller.

---

## Table of Contents

1. [PMP - Pontryagin Maximum Principle](#pmp-pontryagin-maximum-principle)
2. [LQG - Linear Quadratic Gaussian](#lqg-linear-quadratic-gaussian)
3. [MPC - Model Predictive Control](#mpc-model-predictive-control)
4. [Parameter Tuning Guide](#parameter-tuning-guide)
5. [Adding Custom Controllers](#adding-custom-controllers)

---

## PMP - Pontryagin Maximum Principle

### Overview

The PMP controller computes the theoretically optimal control law via backward integration of the costate (adjoint) differential equation. This is done **offline**, then applied as a feedback law in **real-time**.

### Implementation structure (`src/controller_pmp.py`)

```python
class PontryaginController:
    def __init__(self, A, B, Q_s, R_u, Q_f, g_aff, dt, horizon):
        """
        Args:
            A: Continuous-time state matrix (12×12)
            B: Continuous-time control matrix (12×12)
            Q_s: Running state cost (12×12)
            R_u: Running control cost (12×12)
            Q_f: Terminal state cost (12×12)
            g_aff: Affine term (gravity, 12×1)
            dt: Time step (0.002 s)
            horizon: Number of steps to plan (500)
        """
        self.A, self.B = A, B
        self.Q_s, self.R_u, self.Q_f = Q_s, R_u, Q_f
        self.horizon = horizon
        self.dt = dt
        
        # Discretize continuous dynamics
        self.A_d, self.B_d = self._discretize_AB(A, B, dt)
        self.g_d = dt * g_aff
        
        # Storage for gain matrices
        self.K = [np.zeros((12, 12)) for _ in range(horizon)]
        self.k = [np.zeros(12) for _ in range(horizon)]
```

### Key Methods

#### 1. Discretization (`_discretize_AB`)

```python
def _discretize_AB(self, A, B, dt):
    """Convert continuous (A, B) to discrete via Euler or matrix exponential"""
    # Simple Euler:
    A_d = I + dt * A
    B_d = dt * B
    
    # Or more accurate (matrix exponential):
    from scipy.linalg import expm
    A_d = expm(A * dt)
    B_d = np.linalg.solve(A, (A_d - I)) @ B  # Exact discrete B
    
    return A_d, B_d
```

#### 2. Offline Optimal Control Computation (`solve_discrete_sweep`)

```python
def solve_discrete_sweep(self, x_ref, x_f):
    """
    Backward Riccati integration to compute optimal gains K[k]
    
    Args:
        x_ref: Reference trajectory (not used in LQ case, but for nonlinear)
        x_f: Final/terminal state
    
    Algorithm:
        Initialize P[N] = Q_f  # Terminal cost
        For k = N-1 down to 0:
            # Riccati update
            P_bar = B_d.T @ P[k+1] @ B_d + R_u
            K[k] = -np.linalg.inv(P_bar) @ B_d.T @ P[k+1] @ A_d
            P[k] = Q_s + A_d.T @ P[k+1] @ A_d + A_d.T @ P[k+1] @ B_d @ K[k]
    """
    P = self.Q_f.copy()
    
    for k in range(self.horizon - 1, -1, -1):
        # Intermediate matrix
        P_bar = self.B_d.T @ P @ self.B_d + self.R_u
        
        # Gain computation
        self.K[k] = -np.linalg.inv(P_bar) @ self.B_d.T @ P @ self.A_d
        
        # Backward propagation of cost-to-go
        P = self.Q_s + self.A_d.T @ P @ self.A_d + self.A_d.T @ P @ self.B_d @ self.K[k]
```

#### 3. Real-Time Control (`compute_control`)

```python
def compute_control(self, x, x_ref, u_ref):
    """
    Apply precomputed feedback law:
    u = u_ref + K @ (x - x_ref) + k
    
    Args:
        x: Current state (12×1)
        x_ref: Reference state (12×1)
        u_ref: Reference control (12×1)
    
    Returns:
        u: Optimal control (12×1)
    """
    # Assume we're at step k=0 (or use time-varying K[k] if needed)
    u = u_ref + self.K[0] @ (x - x_ref) + self.k[0]
    
    return np.clip(u, -200, 200)  # Force saturation
```

### Advantages & Disadvantages

**Advantages:**
- ✓ Theoretically optimal for linear-quadratic problems
- ✓ Real-time execution (just matrix-vector multiply)
- ✓ Stable for well-conditioned systems
- ✓ No iterative solving needed

**Disadvantages:**
- ✗ Offline computation required
- ✗ Cannot handle constraints directly
- ✗ Assumes system linearity
- ✗ Sensitive to measurement noise

### Tuning Parameters

```python
# In examples/run_mujoco.py:

# Cost matrices (higher = stricter control)
Q = np.diag([80, 80, 300, 40, 40, 15, 150, 150, 100, 5, 5, 5])
# Q[0:3]   = position costs (80 on x,y; 300 on z for altitude)
# Q[3:6]   = velocity costs (40 on linear, 15 on vertical)
# Q[6:9]   = orientation costs (150 on roll/pitch, 100 on yaw)
# Q[9:12]  = angular velocity costs (5 each)

R = np.eye(12) * 5e-3  # Control effort cost (smaller = more control freedom)
Q_f = Q * 10           # Terminal cost (stronger terminal constraint)

# Horizon for planning
horizon = 500  # 500 steps * 0.002 s = 1 second planning horizon
```

### How to Modify PMP

**To make control tighter:**
```python
Q = Q * 2  # Double all position/velocity costs
R = R * 0.5  # Loosen control cost
# Result: More aggressive control, higher forces
```

**To make control smoother:**
```python
R = R * 2  # Double control cost
# Result: Smoother, gentler control, less responsive
```

---

## LQG - Linear Quadratic Gaussian

### Overview

LQG combines **LQR** (optimal state feedback) with a **Kalman Filter** (optimal state estimation). It's the standard choice for noisy systems and real-time control.

### Architecture

```
Measurement y
    ↓
[Kalman Filter] ← estimates x̂ from y with process/measurement noise models
    ↓
   x̂
    ↓
[LQR Gain K] ← u = u_ref - K(x̂ - x_ref)
    ↓
Control u
    ↓
[System Dynamics]
    ↓
Next state x
```

### Implementation Structure (`src/controller_lqg.py`)

```python
class LQGController:
    def __init__(self, A_d, B_d, g_d, Q, R, Q_proc, R_meas):
        """
        Args:
            A_d, B_d, g_d: Discrete dynamics
            Q, R: Running cost matrices
            Q_proc: Process noise covariance (model uncertainty)
            R_meas: Measurement noise covariance (sensor uncertainty)
        """
        self.A_d, self.B_d = A_d, B_d
        self.g_d = g_d
        
        # Solve discrete-time algebraic Riccati equation for LQR
        self.P = self._solve_dare(A_d, B_d, Q, R)
        self.K = self._compute_lqr_gain(self.P, A_d, B_d, R)
        
        # Kalman filter state and covariance
        self.x_hat = np.zeros(12)
        self.P_kf = np.eye(12) * 0.01  # Initial estimate covariance
        self.Q_proc = Q_proc
        self.R_meas = R_meas
```

### Key Methods

#### 1. Discrete-Time Riccati Solver (`_solve_dare`)

```python
def _solve_dare(self, A, B, Q, R, max_iter=100, tol=1e-6):
    """
    Solve discrete algebraic Riccati equation:
    P = Q + A'PA - A'PB(R + B'PB)^(-1)B'PA
    
    Implementation (iterative):
        P[0] = Q
        P[n+1] = Q + A'P[n]A - A'P[n]B(R + B'P[n]B)^(-1)B'P[n]A
    """
    P = Q.copy()
    
    for iteration in range(max_iter):
        P_old = P.copy()
        
        # Riccati step
        S = np.linalg.inv(R + B.T @ P @ B)
        P = Q + A.T @ P @ A - A.T @ P @ B @ S @ B.T @ P @ A
        
        # Check convergence
        if np.linalg.norm(P - P_old) < tol:
            break
    
    return P
```

#### 2. LQR Gain Computation (`_compute_lqr_gain`)

```python
def _compute_lqr_gain(self, P, A, B, R):
    """
    Compute feedback gain:
    K = (R + B'PB)^(-1) B'PA
    """
    S = np.linalg.inv(R + B.T @ P @ B)
    K = S @ B.T @ P @ A
    
    return K
```

#### 3. Kalman Filter Step (`step`)

```python
def step(self, y, x_ref, u_ref):
    """
    Execute one Kalman filter + LQR step
    
    Args:
        y: Measurement/observation (12×1) - noisy state
        x_ref: Reference state
        u_ref: Reference control
    
    Returns:
        u: Computed control action
    """
    # ========== PREDICTION PHASE ==========
    # Predict next state based on dynamics
    x_hat_predict = self.A_d @ self.x_hat + self.B_d @ self.u_last + self.g_d
    P_predict = self.A_d @ self.P_kf @ self.A_d.T + self.Q_proc
    
    # ========== UPDATE PHASE ==========
    # Kalman gain
    innovation_cov = self.C @ P_predict @ self.C.T + self.R_meas  # Measurement residual covariance
    K_kf = P_predict @ self.C.T @ np.linalg.inv(innovation_cov)
    
    # State update
    innovation = y - self.C @ x_hat_predict  # Measurement residual
    self.x_hat = x_hat_predict + K_kf @ innovation
    
    # Covariance update
    self.P_kf = (np.eye(12) - K_kf @ self.C) @ P_predict
    
    # ========== CONTROL COMPUTATION ==========
    u = u_ref - self.K @ (self.x_hat - x_ref)
    
    self.u_last = u.copy()
    return np.clip(u, -200, 200)
```

### Noise Covariance Tuning

```python
# Process noise (model uncertainty)
Q_proc = np.diag([
    1e-3, 1e-3, 1e-3,  # Position uncertainty (meters)
    1e-2, 1e-2, 1e-2,  # Velocity uncertainty (m/s)
    5e-3, 5e-3, 5e-3,  # Orientation uncertainty (radians)
    1e-2, 1e-2, 1e-2   # Angular velocity uncertainty (rad/s)
])

# Measurement noise (sensor uncertainty)
R_meas = np.diag([
    5e-3, 5e-3, 5e-3,  # IMU accelerometer noise
    2e-2, 2e-2, 2e-2,  # Joint encoder noise
    1e-2, 1e-2, 1e-2,  # Vision/contact noise
    5e-2, 5e-2, 5e-2   # Gyro noise
])
```

**Tuning strategy:**
- **High Q_proc**: More trust in measurements, faster response to disturbance
- **High R_meas**: More trust in model, smoother control, more lag
- **Typical ratio**: R_meas ≈ 5-10× Q_proc

### How to Modify LQG

**For faster response to disturbances:**
```python
Q_proc = Q_proc * 10  # Trust measurements more
# Result: Filter responds quicker, but noisier control
```

**For smoother control:**
```python
R_meas = R_meas * 2  # Trust model more
# Result: Smoother, but slower to respond
```

---

## MPC - Model Predictive Control

### Overview

MPC solves a **constrained finite-horizon optimization problem** at each control step. This allows explicit handling of friction, force limits, and other physical constraints.

### QP Formulation

```
min  (1/2) u'Hu + q'u
s.t. A_ub @ u ≤ b_ub    (inequality: friction pyramid, force bounds)
     A_eq @ u = b_eq    (equality: dynamics constraints)
```

### Implementation Structure (`src/controller_mpc.py`)

```python
class MPCController:
    def __init__(self, A_d, B_d, g_d, Q, R, Q_f, N=15, mu=0.6, fz_max=150.0):
        """
        Args:
            A_d, B_d, g_d: Discrete dynamics
            Q, R, Q_f: Cost matrices
            N: Horizon length (10-15 typical)
            mu: Friction coefficient (0.6 typical)
            fz_max: Max vertical force per leg (150 N typical)
        """
        self.A_d, self.B_d = A_d, B_d
        self.g_d = g_d
        self.Q, self.R, self.Q_f = Q, R, Q_f
        self.N = N
        self.mu = mu
        self.fz_max = fz_max
        
        # Initialize OSQP solver
        self.solver = osqp.OSQP()
        
        # Precompute dynamics matrices for horizon
        self.Jx, self.Ju = self._build_horizon_matrices()
```

### Key Methods

#### 1. Horizon Dynamics Matrices (`_build_horizon_matrices`)

```python
def _build_horizon_matrices(self):
    """
    Construct matrices that map control sequence to state trajectory
    
    x_0 = x0 (given)
    x_1 = A @ x_0 + B @ u_0 + g
    x_2 = A @ x_1 + B @ u_1 + g = A²@x_0 + A@B@u_0 + B@u_1 + A@g + g
    ...
    x_N = A^N @ x_0 + Σ(A^(N-1-i) @ B @ u_i) + Σ(A^(N-1-i) @ g)
    
    This is rewritten as:
    [x_0]   [I    0    0  ...  0  ]   [x_0]   [  0  ]
    [x_1] = [A    0    0  ...  0  ]   [  ] + [...] + [B 0  0 ...][u_0]
    [x_2]   [A²   0    0  ...  0  ]   [x_0]   [...] + [AB B  0 ...][u_1]
    [...    [...]]                             + [...] + [...........][...]
    [x_N]
    
    Resulting in: X = Jx @ x0 + g_sum + Ju @ U
    """
    Jx = np.zeros((12 * (self.N + 1), 12))
    Ju = np.zeros((12 * (self.N + 1), 12 * self.N))
    
    # Fill Jx (effect of initial state)
    Jx[0:12, :] = np.eye(12)
    A_power = self.A_d.copy()
    for k in range(1, self.N + 1):
        Jx[12*k:12*(k+1), :] = A_power
        A_power = self.A_d @ A_power
    
    # Fill Ju (effect of control sequence)
    A_power = np.eye(12)
    for k in range(self.N):
        for j in range(k + 1):
            Ju[12*(k+1):12*(k+2), 12*j:12*(j+1)] = A_power @ self.B_d
        A_power = self.A_d @ A_power
    
    return Jx, Ju
```

#### 2. QP Construction (`_formulate_qp`)

```python
def _formulate_qp(self, x, x_ref, u_ref):
    """
    Construct QP parameters for current state.
    Called at every control step to get time-varying QP.
    """
    # ========== HESSIAN & GRADIENT ==========
    # Hessian: 2 * blkdiag(R, R, ..., R)
    H = spa.block_diag([2 * self.R for _ in range(self.N)])
    
    # Gradient: Based on reference trajectory
    q = np.zeros(12 * self.N)
    for k in range(self.N):
        q[12*k:12*(k+1)] = -2 * self.R @ u_ref
    
    # Account for state costs via Lagrangian manipulation
    # ... (detailed derivation involves chain rule through dynamics)
    
    # ========== EQUALITY CONSTRAINTS (DYNAMICS) ==========
    # x_{k+1} = A @ x_k + B @ u_k + g
    # Rearranged: A @ x_k + B @ u_k - x_{k+1} + g = 0
    
    A_eq = np.zeros((12 * self.N, 12 * self.N))
    b_eq = np.zeros(12 * self.N)
    
    for k in range(self.N):
        # Row for constraint: x_{k+1} = A @ x_k + B @ u_k + g
        # ... construct rows
    
    # ========== INEQUALITY CONSTRAINTS (FRICTION & FORCES) ==========
    # For each leg i and time step k:
    #   -mu * f_z ≤ f_x ≤ mu * f_z
    #   -mu * f_z ≤ f_y ≤ mu * f_z
    #    0 ≤ f_z ≤ f_z_max
    
    A_ub_list = []
    b_ub_list = []
    
    for k in range(self.N):
        for i in range(4):  # 4 legs
            f_idx = 12*k + 3*i  # Index of [fx, fy, fz] for leg i at step k
            
            # Friction cone inequalities (4 per leg)
            # +f_x - mu*f_z ≤ 0
            A_row = np.zeros(12 * self.N)
            A_row[f_idx] = 1
            A_row[f_idx + 2] = -self.mu
            A_ub_list.append(A_row)
            b_ub_list.append(0)
            
            # -f_x - mu*f_z ≤ 0
            A_row = np.zeros(12 * self.N)
            A_row[f_idx] = -1
            A_row[f_idx + 2] = -self.mu
            A_ub_list.append(A_row)
            b_ub_list.append(0)
            
            # ... (similar for f_y)
            
            # Force bounds (2 per leg)
            # f_z ≤ f_z_max  →  f_z ≤ f_z_max
            A_row = np.zeros(12 * self.N)
            A_row[f_idx + 2] = 1
            A_ub_list.append(A_row)
            b_ub_list.append(self.fz_max)
            
            # -f_z ≤ 0  →  f_z ≥ 0
            A_row = np.zeros(12 * self.N)
            A_row[f_idx + 2] = -1
            A_ub_list.append(A_row)
            b_ub_list.append(0)
    
    A_ub = np.array(A_ub_list)
    b_ub = np.array(b_ub_list)
    
    return H, q, A_eq, b_eq, A_ub, b_ub
```

#### 3. Solving and Control (`compute_control`)

```python
def compute_control(self, x, x_ref, u_ref):
    """
    Solve MPC problem and return first control action.
    
    Args:
        x: Current state (12×1)
        x_ref: Reference state (12×1)
        u_ref: Reference control (12×1)
    
    Returns:
        u: Constrained optimal control (12×1)
    """
    # Formulate QP for current state
    H, q, A_eq, b_eq, A_ub, b_ub = self._formulate_qp(x, x_ref, u_ref)
    
    # Update OSQP problem
    self.solver.update(
        P=spa.csc_matrix(H),
        A=spa.csc_matrix(np.vstack([A_eq, A_ub])),
        l=np.hstack([b_eq, np.full(A_ub.shape[0], -np.inf)]),
        u=np.hstack([b_eq, b_ub])
    )
    
    # Solve
    results = self.solver.solve()
    
    # Extract first control action (other N-1 are discarded - receding horizon)
    if results.info.status == 'solved':
        u_optimal_sequence = results.x
        u = u_optimal_sequence[0:12]  # First action
    else:
        # Fall back to reference if solver fails
        u = u_ref.copy()
    
    return np.clip(u, -200, 200)
```

### Constraint Parameters

```python
# Friction coefficient (typical: 0.5-0.7 for rubber on concrete/grass)
mu = 0.6

# Max vertical force per leg (9 kg robot ≈ 22.5 N per leg when standing)
# Conservative max allows for jumping/accelerating
fz_max = 150.0  # N (about 7× standing load)

# Horizon (steps into future to plan)
N = 15  # 15 steps * 0.002 s = 0.03 s preview
# Typical range: 8-20 steps
# Longer horizon: Better planning but slower solve
# Shorter horizon: Faster but less foresight
```

### How to Modify MPC

**For tighter constraints:**
```python
mu = 0.4  # More conservative friction model
fz_max = 100  # Lower force limits
# Result: Safer but slower, requires more aggressive planning
```

**For faster planning horizon:**
```python
N = 10  # Shorter preview → faster QP solve
# Result: Quicker computation but less foresight
```

**For emphasis on constraint satisfaction:**
```python
# Increase R (control cost) or decrease Q (state cost)
R = R * 2  # Penalize large forces
# Result: Stays within constraints better, but slower response
```

---

## Parameter Tuning Guide

### Cost Function Weights

The cost function trades off:
1. **Tracking accuracy** (through Q)
2. **Control effort** (through R)
3. **Terminal control** (through Q_f)

```python
# Position costs (indices 0-2)
# z cost (index 2) is higher because height is critical for stability
Q[0] = 80      # x position cost
Q[1] = 80      # y position cost
Q[2] = 300     # z position cost (height - critical!)

# Velocity costs (indices 3-5)
Q[3] = 40      # vx cost
Q[4] = 40      # vy cost
Q[5] = 15      # vz cost (less critical)

# Orientation costs (indices 6-8)
Q[6] = 150     # roll cost (very important for stability)
Q[7] = 150     # pitch cost (very important)
Q[8] = 100     # yaw cost (less critical, mainly for turning)

# Angular velocity costs (indices 9-11)
Q[9] = 5       # roll rate
Q[10] = 5      # pitch rate
Q[11] = 5      # yaw rate

# Control cost (penalizes force)
R = 5e-3 * I_12  # 5e-3 per force component
```

### Tuning Workflow

**Step 1: Understand baseline performance**
```bash
python examples/run_mujoco.py --controller lqg --no-render
# Check RMSE, MaxE, TVU values
```

**Step 2: Identify problem**
- If falling or unstable → Increase Q[2] (height), Q[6,7] (orientation)
- If too slow/sluggish → Decrease R
- If jerky/oscillating → Increase R, decrease Q
- If drifting laterally → Increase Q[0,1]

**Step 3: Implement change**
```python
# Edit examples/run_mujoco.py, find cost matrix construction (~line 540)
Q = np.diag([80, 80, 300, ...])  # Modify desired element
```

**Step 4: Test and compare**
```bash
python examples/run_mujoco.py --controller lqg --duration 20 --no-render
# Check new metrics
```

### Common Tuning Scenarios

#### Scenario 1: Robot Falls Immediately

**Diagnosis:**
```
t=0.1s Terminated at t=0.002s  # Falls immediately
```

**Fix:**
```python
Q[2] = 1000     # Increase height cost
Q[6] = 500      # Increase roll cost  
Q[7] = 500      # Increase pitch cost
R = R * 0.5     # Allow more control effort
```

#### Scenario 2: Jerky, Unstable Walking

**Diagnosis:**
```
Robot shakes, noisy control output
TVU = 500+  # Very high variation
```

**Fix:**
```python
R = R * 5  # Penalize control more heavily
# Or reduce Q somewhat to relax tracking requirements
Q = Q * 0.5
```

#### Scenario 3: Slow Response, Can't Keep Up with Waypoints

**Diagnosis:**
```
Robot lags behind waypoints, max_vx=0.22 but actual vx << 0.22
```

**Fix:**
```python
R = R * 0.1  # Allow higher forces
# Or modify gait to be faster:
# GAIT_PERIOD = 0.60  (was 0.80)
```

---

## Adding Custom Controllers

### Template for New Controller

```python
# In src/controller_custom.py

class CustomController:
    """Your custom controller implementation"""
    
    def __init__(self, A_d, B_d, g_d, Q, R, **kwargs):
        """
        Initialize controller with dynamics and cost matrices
        
        Args:
            A_d: Discrete state matrix (12×12)
            B_d: Discrete control matrix (12×12)
            g_d: Affine term (12×1)
            Q: State cost matrix (12×12)
            R: Control cost matrix (12×12)
            **kwargs: Any custom parameters
        """
        self.A_d = A_d
        self.B_d = B_d
        self.g_d = g_d
        self.Q = Q
        self.R = R
    
    def compute_control(self, x, x_ref, u_ref):
        """
        Compute control action
        
        Args:
            x: Current state (12×1)
            x_ref: Reference state (12×1)
            u_ref: Reference control (12×1)
        
        Returns:
            u: Control action (12×1)
        """
        # Implement your control law
        u = u_ref  # Placeholder
        
        return np.clip(u, -200, 200)  # Always clip to feasible range
```

### Integration Steps

1. **Create module**: `src/controller_custom.py`
2. **Import in runner**: Add to `examples/run_mujoco.py`
   ```python
   from src.controller_custom import CustomController
   ```
3. **Add to factory**: In `build_controller()` function
   ```python
   if name == "custom":
       ctrl = CustomController(A_d=A_d, B_d=B_d, ...)
       return ctrl
   ```
4. **Test**:
   ```bash
   python examples/run_mujoco.py --controller custom
   ```

---

## Performance Profiling

```python
import time

# Time a controller step
t0 = time.time()
u = controller.compute_control(x, x_ref, u_ref)
elapsed = (time.time() - t0) * 1000  # ms

print(f"Control compute time: {elapsed:.2f} ms")
```

**Typical times:**
- PMP: ~0.1 ms (just matrix multiply)
- LQG: ~0.5 ms (Kalman filter + gain)
- MPC: 1-5 ms (QP solver depends on options)

---

**For theoretical details, see [MATHEMATICAL_FRAMEWORK.md](MATHEMATICAL_FRAMEWORK.md)**
