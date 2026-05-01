# Mathematical Framework

## Complete Derivation of Control Strategies

This document provides the full mathematical foundation for the three control approaches implemented in this project.

---

## Table of Contents

1. [System Dynamics](#system-dynamics)
2. [Pontryagin Maximum Principle (PMP)](#pontryagin-maximum-principle-pmp)
3. [Linear Quadratic Gaussian (LQG)](#linear-quadratic-gaussian-lqg)
4. [Model Predictive Control (MPC)](#model-predictive-control-mpc)
5. [Numerical Methods](#numerical-methods)

---

## System Dynamics

### State Representation

**Continuous-time floating-base state:**

$$\mathbf{x} = \begin{bmatrix} \mathbf{p} \\ \mathbf{v} \\ \boldsymbol{\theta} \\ \boldsymbol{\omega} \end{bmatrix} \in \mathbb{R}^{12}$$

Where:
- $\mathbf{p} \in \mathbb{R}^3$ = Base center-of-mass position in world frame
- $\mathbf{v} \in \mathbb{R}^3$ = Base linear velocity
- $\boldsymbol{\theta} = [\phi, \theta, \psi]^T$ = Euler angles (roll, pitch, yaw)
- $\boldsymbol{\omega} \in \mathbb{R}^3$ = Angular velocity in body frame

### Control Input

**Ground reaction forces (GRF):**

$$\mathbf{u} = \begin{bmatrix} \mathbf{f}_{FL} \\ \mathbf{f}_{FR} \\ \mathbf{f}_{RL} \\ \mathbf{f}_{RR} \end{bmatrix} \in \mathbb{R}^{12}$$

Each $\mathbf{f}_i \in \mathbb{R}^3$ represents the 3D contact force from leg $i \in \{FL, FR, RL, RR\}$.

### Continuous-Time Dynamics

**Floating-base equations of motion:**

$$\dot{\mathbf{v}} = \mathbf{g} + \frac{1}{m}\sum_{i=1}^{4} \mathbf{f}_i$$

$$\dot{\boldsymbol{\omega}} = \mathbf{I}^{-1} \left( \sum_{i=1}^{4} \mathbf{r}_i \times \mathbf{f}_i - \boldsymbol{\omega} \times \mathbf{I}\boldsymbol{\omega} \right)$$

Where:
- $m = 9.0$ kg = Robot mass
- $\mathbf{g} = [0, 0, -9.81]^T$ m/s² = Gravity
- $\mathbf{I} = \text{diag}(I_x, I_y, I_z)$ = Inertia matrix
- $\mathbf{r}_i$ = Moment arm from CoM to foot $i$

**First-order kinematics:**

$$\dot{\mathbf{p}} = \mathbf{v}$$

$$\dot{\boldsymbol{\theta}} = R_x(\phi)^{-1} \begin{bmatrix} 0 & \sin\phi\tan\theta & \cos\phi\tan\theta \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi/\cos\theta & \cos\phi/\cos\theta \end{bmatrix} \boldsymbol{\omega}$$

### Discretization

**Forward Euler with time step $\Delta t = 0.002$ s:**

$$\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \dot{\mathbf{x}}(\mathbf{x}_k, \mathbf{u}_k)$$

**More formally (linearized):**

$$\mathbf{x}_{k+1} = A_k \mathbf{x}_k + B_k \mathbf{u}_k + \mathbf{g}_k + \mathbf{w}_k$$

Where:
- $A_k = I_{12} + \Delta t \frac{\partial \dot{\mathbf{x}}}{\partial \mathbf{x}} \bigg|_{(\mathbf{x}_k, \mathbf{u}_k)}$ (state derivative Jacobian)
- $B_k = \Delta t \frac{\partial \dot{\mathbf{x}}}{\partial \mathbf{u}} \bigg|_{(\mathbf{x}_k, \mathbf{u}_k)}$ (control derivative Jacobian)
- $\mathbf{g}_k$ = Gravity and nonlinear terms (for force mapping via Jacobian from kinematics)
- $\mathbf{w}_k \sim \mathcal{N}(0, Q_{proc})$ = Process noise

### Linearization Points

Controllers linearize around **reference state**:

$$\mathbf{x}_{ref} = \begin{bmatrix} 0 \\ 0 \\ h_{hip} \\ 0 \\ 0 \\ 0 \\ 0 \\ 0 \\ \psi_{cmd} \\ 0 \\ 0 \\ \omega_z \end{bmatrix}$$

With **reference control:**

$$\mathbf{u}_{ref} = \begin{bmatrix} 0 \\ 0 \\ mg/4 \\ 0 \\ 0 \\ mg/4 \\ 0 \\ 0 \\ mg/4 \\ 0 \\ 0 \\ mg/4 \end{bmatrix}$$

(Equal weight distribution to all four legs in neutral standing posture)

---

## Pontryagin Maximum Principle (PMP)

### Optimal Control Problem

**Minimize:**

$$J = \int_0^T L(\mathbf{x}(t), \mathbf{u}(t), t) \, dt + \phi(\mathbf{x}(T))$$

Where cost is:

$$L(\mathbf{x}, \mathbf{u}, t) = \frac{1}{2}(\mathbf{x} - \mathbf{x}_{ref})^T Q (\mathbf{x} - \mathbf{x}_{ref}) + \frac{1}{2}(\mathbf{u} - \mathbf{u}_{ref})^T R (\mathbf{u} - \mathbf{u}_{ref})$$

$$\phi(\mathbf{x}(T)) = \frac{1}{2}(\mathbf{x}(T) - \mathbf{x}_{ref})^T Q_f (\mathbf{x}(T) - \mathbf{x}_{ref})$$

**Subject to:**

$$\dot{\mathbf{x}}(t) = \mathbf{f}(\mathbf{x}(t), \mathbf{u}(t), t)$$

### Hamiltonian Formulation

Define the **Hamiltonian:**

$$H(\mathbf{x}, \mathbf{u}, \boldsymbol{\lambda}, t) = L(\mathbf{x}, \mathbf{u}, t) + \boldsymbol{\lambda}^T \mathbf{f}(\mathbf{x}, \mathbf{u}, t)$$

Where $\boldsymbol{\lambda} \in \mathbb{R}^{12}$ = **costate (adjoint state)**.

### Optimality Conditions (Pontryagin's Maximum Principle)

**1. State dynamics:**
$$\dot{\mathbf{x}} = \nabla_{\boldsymbol{\lambda}} H = \frac{\partial H}{\partial \boldsymbol{\lambda}} = \mathbf{f}(\mathbf{x}, \mathbf{u})$$

**2. Costate dynamics:**
$$\dot{\boldsymbol{\lambda}} = -\nabla_{\mathbf{x}} H = -\frac{\partial H}{\partial \mathbf{x}} = -\left( \nabla_{\mathbf{x}} L + \left(\nabla_{\mathbf{x}} \mathbf{f}\right)^T \boldsymbol{\lambda} \right)$$

**3. Optimal control:**
$$\mathbf{u}^* = \arg\min_{\mathbf{u}} H = -R^{-1} B^T \boldsymbol{\lambda}$$

(Assuming quadratic cost and linear control input in $\mathbf{u}$)

**4. Terminal condition:**
$$\boldsymbol{\lambda}(T) = \nabla_{\mathbf{x}} \phi(\mathbf{x}(T)) = Q_f(\mathbf{x}(T) - \mathbf{x}_{ref})$$

### Linear-Quadratic Special Case

For linear dynamics $\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u} + \mathbf{g}$ and quadratic cost:

**Costate evolution simplifies:**
$$\dot{\boldsymbol{\lambda}} = -Q(\mathbf{x} - \mathbf{x}_{ref}) - A^T\boldsymbol{\lambda}$$

**Optimal control becomes:**
$$\mathbf{u}^* = \mathbf{u}_{ref} - R^{-1}B^T\boldsymbol{\lambda}$$

### Discrete-Time Implementation

**Backward Riccati Equation:**

Starting from terminal condition $P_N = Q_f$, integrate backward:

$$P_k = Q + A^T P_{k+1} A - A^T P_{k+1} B (R + B^T P_{k+1} B)^{-1} B^T P_{k+1} A$$

**Feedback gain:**
$$K_k = (R + B^T P_{k+1} B)^{-1} B^T P_{k+1} A$$

**Optimal control law:**
$$\mathbf{u}_k^* = \mathbf{u}_{ref} - K_k (\mathbf{x}_k - \mathbf{x}_{ref})$$

### Implementation in Code

```python
# See src/controller_pmp.py
# 1. solve_discrete_sweep() computes backward Riccati
# 2. compute_control() applies feedback law
```

---

## Linear Quadratic Gaussian (LQG)

### LQR (Linear Quadratic Regulator)

For linear system:
$$\mathbf{x}_{k+1} = A \mathbf{x}_k + B \mathbf{u}_k + \mathbf{g}$$

Minimize quadratic cost:
$$J = \sum_{k=0}^{\infty} (\mathbf{x}_k - \mathbf{x}_{ref})^T Q (\mathbf{x}_k - \mathbf{x}_{ref}) + (\mathbf{u}_k - \mathbf{u}_{ref})^T R (\mathbf{u}_k - \mathbf{u}_{ref})$$

### Value Function and Riccati Equation

**Value function (infinite horizon):**
$$V(\mathbf{x}_k) = (\mathbf{x}_k - \mathbf{x}_{ref})^T P (\mathbf{x}_k - \mathbf{x}_{ref})$$

**Discrete-time algebraic Riccati equation:**
$$P = Q + A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A$$

### Optimal Gain

**LQR gain matrix:**
$$K = (R + B^T P B)^{-1} B^T P A$$

**Optimal feedback control:**
$$\mathbf{u}_k = \mathbf{u}_{ref} - K(\mathbf{x}_k - \mathbf{x}_{ref})$$

### Kalman Filter for State Estimation

For system with measurements:
$$\mathbf{y}_k = C \mathbf{x}_k + \mathbf{n}_k$$

With process noise $\mathbf{w}_k \sim \mathcal{N}(0, Q_{proc})$ and measurement noise $\mathbf{n}_k \sim \mathcal{N}(0, R_{meas})$.

**Prediction step:**
$$\hat{\mathbf{x}}^-_k = A \hat{\mathbf{x}}_{k-1} + B \mathbf{u}_{k-1} + \mathbf{g}$$
$$P^-_k = A P_{k-1} A^T + Q_{proc}$$

**Update step:**
$$K_{kf} = P^-_k C^T (C P^-_k C^T + R_{meas})^{-1}$$
$$\hat{\mathbf{x}}_k = \hat{\mathbf{x}}^-_k + K_{kf} (\mathbf{y}_k - C \hat{\mathbf{x}}^-_k)$$
$$P_k = (I - K_{kf} C) P^-_k$$

### LQG Control Law

**Using estimated state:**
$$\mathbf{u}_k = \mathbf{u}_{ref} - K(\hat{\mathbf{x}}_k - \mathbf{x}_{ref})$$

Where $\hat{\mathbf{x}}_k$ is the Kalman filter estimate.

### Typical Noise Covariances

**Process noise** (model uncertainty):
$$Q_{proc} = \text{diag}(10^{-3}, 10^{-3}, 10^{-3}, 10^{-2}, 10^{-2}, 10^{-2}, 5 \times 10^{-3}, 5 \times 10^{-3}, 5 \times 10^{-3}, 10^{-2}, 10^{-2}, 10^{-2})$$

**Measurement noise** (sensor uncertainty):
$$R_{meas} = \text{diag}(5 \times 10^{-3}, 5 \times 10^{-3}, 5 \times 10^{-3}, 2 \times 10^{-2}, 2 \times 10^{-2}, 2 \times 10^{-2}, 10^{-2}, 10^{-2}, 10^{-2}, 5 \times 10^{-2}, 5 \times 10^{-2}, 5 \times 10^{-2})$$

### Implementation in Code

```python
# See src/controller_lqg.py
# 1. __init__() solves Riccati equation, initializes Kalman filter
# 2. step() performs prediction + update + control law
```

---

## Model Predictive Control (MPC)

### Receding Horizon Problem

Over horizon $N$ steps (typically 10-15), minimize:

$$\min_{\mathbf{u}_0, ..., \mathbf{u}_{N-1}} \sum_{k=0}^{N-1} \left[ \|\mathbf{x}_k - \mathbf{x}_{ref}\|_Q^2 + \|\mathbf{u}_k - \mathbf{u}_{ref}\|_R^2 \right] + \|\mathbf{x}_N - \mathbf{x}_{ref}\|_{Q_f}^2$$

### Constraints

**1. Dynamics constraint:**
$$\mathbf{x}_{k+1} = A \mathbf{x}_k + B \mathbf{u}_k + \mathbf{g}, \quad \forall k \in [0, N-1]$$

**2. Friction pyramid (each force component):**
$$|f_{x,i}| \leq \mu f_{z,i}, \quad |f_{y,i}| \leq \mu f_{z,i}, \quad \forall i \in [1, 4]$$

With friction coefficient $\mu = 0.6$.

**3. Normal force bounds:**
$$0 \leq f_{z,i} \leq f_{z,max}, \quad \forall i \in [1, 4]$$

With $f_{z,max} = 150$ N.

### Quadratic Program Formulation

Rearranging as a **standard QP**:

$$\min_{\mathbf{z}} \frac{1}{2} \mathbf{z}^T P \mathbf{z} + \mathbf{q}^T \mathbf{z}$$

Subject to:
$$A_{ub} \mathbf{z} \leq \mathbf{b}_{ub} \quad \text{(inequality constraints)}$$
$$A_{eq} \mathbf{z} = \mathbf{b}_{eq} \quad \text{(equality constraints)}$$

Where:
- Decision vector: $\mathbf{z} = [\mathbf{u}_0^T, ..., \mathbf{u}_{N-1}^T]^T \in \mathbb{R}^{12N}$
- Hessian: $P = \text{blkdiag}(R, ..., R)$ (block diagonal)
- Gradient: $\mathbf{q}$ encodes cost function
- Equality constraints: Dynamics constraints
- Inequality constraints: Friction pyramid + force bounds

### Cost Matrix Construction

**Hessian (for control cost):**
$$P = 2\text{blkdiag}(R, ..., R) \text{ with } R = \text{diag}(5 \times 10^{-3}, 5 \times 10^{-3}, 5 \times 10^{-3})$$

**Gradient (for state and terminal cost):**

The predicted states are:
$$\mathbf{x}_k = A^k \mathbf{x}_0 + \sum_{j=0}^{k-1} A^{k-1-j} (B \mathbf{u}_j + \mathbf{g})$$

These are substituted into the cost function to express it purely in terms of $\mathbf{u}_0, ..., \mathbf{u}_{N-1}$.

### Inequality Constraints (Friction Pyramid)

For each leg $i$ and time step $k$:

**Two-cone approximation (linearized friction):**
$$|f_{x,i}| \leq \mu f_{z,i} \quad \Rightarrow \quad -\mu f_{z,i} \leq f_{x,i} \leq \mu f_{z,i}$$
$$|f_{y,i}| \leq \mu f_{z,i} \quad \Rightarrow \quad -\mu f_{z,i} \leq f_{y,i} \leq \mu f_{z,i}$$

**Normal force:**
$$0 \leq f_{z,i} \leq 150 \text{ N}$$

### Solver

Uses **OSQP** (Operator Splitting Quadratic Program) solver:
- Interior-point method
- Primal-dual algorithm
- Real-time capable for $N \approx 10-15$
- Guaranteed convergence for QPs

### Implementation in Code

```python
# See src/controller_mpc.py
# 1. _formulate_qp() constructs P, q, A_ub, b_ub, A_eq, b_eq
# 2. solver.solve() from OSQP solves the QP
# 3. compute_control() extracts first control action
```

---

## Numerical Methods

### Linearization & Jacobians

**Discrete nonlinear system:**
$$\mathbf{x}_{k+1} = \mathbf{f}_d(\mathbf{x}_k, \mathbf{u}_k)$$

**Linearization around reference** $(\mathbf{x}_{ref}, \mathbf{u}_{ref})$:

$$A_k = \frac{\partial \mathbf{f}_d}{\partial \mathbf{x}} \bigg|_{(\mathbf{x}_{ref}, \mathbf{u}_{ref})}, \quad B_k = \frac{\partial \mathbf{f}_d}{\partial \mathbf{u}} \bigg|_{(\mathbf{x}_{ref}, \mathbf{u}_{ref})}$$

**Taylor expansion:**
$$\mathbf{x}_{k+1} \approx A_k (\mathbf{x}_k - \mathbf{x}_{ref}) + B_k (\mathbf{u}_k - \mathbf{u}_{ref}) + \mathbf{x}_{ref} + \text{affine terms}$$

### Numerical Integration

**Forward Euler (order 1):**
$$\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \, \dot{\mathbf{x}}(\mathbf{x}_k, \mathbf{u}_k)$$

**RK4 (Runge-Kutta order 4 - more accurate but slower):**
$$k_1 = \dot{\mathbf{x}}(\mathbf{x}_k, \mathbf{u}_k)$$
$$k_2 = \dot{\mathbf{x}}(\mathbf{x}_k + \frac{\Delta t}{2}k_1, \mathbf{u}_k)$$
$$k_3 = \dot{\mathbf{x}}(\mathbf{x}_k + \frac{\Delta t}{2}k_2, \mathbf{u}_k)$$
$$k_4 = \dot{\mathbf{x}}(\mathbf{x}_k + \Delta t k_3, \mathbf{u}_k)$$
$$\mathbf{x}_{k+1} = \mathbf{x}_k + \frac{\Delta t}{6}(k_1 + 2k_2 + 2k_3 + k_4)$$

### Stability Conditions

**LQR stability:**
If $A, B$ controllable and $Q, R$ positive definite, then:
- Riccati equation has unique positive-definite solution $P$
- Closed-loop system $\mathbf{x}_{k+1} = (A - BK)\mathbf{x}_k$ is asymptotically stable
- All eigenvalues of $(A - BK)$ satisfy $|\lambda_i| < 1$

**MPC stability:**
- Constraint-admissible set: $X = \{\mathbf{x} : \exists \mathbf{u} \text{ satisfying constraints}\}$
- If terminal cost $Q_f$ appropriately chosen, can guarantee $\mathbf{x} \in X$ remains in $X$

---

## Comparison Summary

| Property | PMP | LQG | MPC |
|----------|-----|-----|-----|
| **Optimality** | Global | Local (linear) | Finite horizon |
| **Constraints** | Implicit | Implicit | Explicit |
| **Real-time** | ✗ (offline) | ✓ (fast) | ~ (QP solve) |
| **Robustness** | Low | High (Kalman) | Medium |
| **Computation** | O(N³) backward sweep | O(n³) Riccati | O((12N)³) QP |
| **Stability Guaranteed** | No | Yes (LDS) | Conditional |

---

**For detailed implementation, see:**
- `src/controller_pmp.py` - PMP implementation
- `src/controller_lqg.py` - LQG implementation
- `src/controller_mpc.py` - MPC implementation
