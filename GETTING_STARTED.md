# Getting Started Guide

## 5-Minute Quick Start

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```

### 2. Run Your First Simulation
```bash
python examples/run_mujoco.py --controller lqg
```

You should see:
- A MuJoCo visualization window with a Mini Cheetah robot
- Console output showing simulation progress
- After ~40 seconds, a plot saved to `results/`

**Congratulations! Your installation works!**

---

## Detailed Installation Steps

### Prerequisites Check

Before installing, verify your system has:

```bash
# Check Python version (need 3.8+)
python --version

# Check pip is installed
pip --version

# Check you can create a virtual environment (recommended)
python -m venv --help
```

### Step 1: Clone Repository

```bash
git clone https://github.com/Angel97977/Applying-Optimal-Control-In-Mujoco.git
cd Applying-Optimal-Control-In-Mujoco
```

### Step 2: Create Virtual Environment

**Using venv (recommended):**
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

**Or using conda:**
```bash
conda create -n quadruped python=3.10
conda activate quadruped
```

**Or using pyenv:**
```bash
pyenv install 3.10.0
pyenv local 3.10.0
python -m venv venv
source venv/bin/activate
```

### Step 3: Verify Virtual Environment

```bash
which python  # Should show path in venv/
pip --version  # Should reference venv
```

### Step 4: Install Core Dependencies

```bash
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt
```

**Expected output:**
```
Successfully installed numpy scipy matplotlib mujoco gymnasium osqp websockets
```

### Step 5: Install gym-quadruped (if provided separately)

If you have the gym-quadruped package:

```bash
# Navigate to gym-quadruped directory
cd /path/to/gym-quadruped-master
pip install -e .

# Return to main directory
cd /path/to/Applying-Optimal-Control-In-Mujoco
```

### Step 6: Verify Installation

```bash
# Test all imports work
python -c "
import numpy as np
import scipy
import matplotlib
import mujoco
import gymnasium
import osqp
import websockets
from src.dynamics import QuadrupedDynamics
from src.estimator_ekf import OrientationEKF
from src.controller_pmp import PontryaginController
from src.controller_lqg import LQGController
from src.controller_mpc import MPCController
print('✓ All dependencies and modules imported successfully!')
"
```

---

## Common Installation Issues & Solutions

### Issue 1: "ModuleNotFoundError: No module named 'mujoco'"

**Problem:** MuJoCo not installed or virtual environment not activated

**Solutions:**
```bash
# Activate virtual environment
source venv/bin/activate

# Reinstall mujoco
pip install --upgrade mujoco

# Verify
python -c "import mujoco; print(mujoco.__version__)"
```

### Issue 2: "ModuleNotFoundError: No module named 'gymnasium'"

**Problem:** Gymnasium not installed (renamed from gym)

**Solution:**
```bash
pip install --upgrade gymnasium
```

### Issue 3: "ModuleNotFoundError: No module named 'osqp'"

**Problem:** Quadratic programming solver not installed (needed for MPC)

**Solution:**
```bash
pip install osqp
```

### Issue 4: Permission Error on Linux/Mac

**Problem:** Cannot write to default pip location

**Solution:**
```bash
pip install --user -r requirements.txt
# Or use virtual environment (recommended)
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Issue 5: "No module named 'gym_quadruped'"

**Problem:** gym-quadruped environment not installed

**Solution:**
```bash
# Check if it's in a separate directory and install it
cd /path/to/gym-quadruped-master
pip install -e .

# Or if you don't have it, the environment should handle missing robot gracefully
python examples/run_mujoco.py --robot-name mini_cheetah  # Uses default
```

### Issue 6: MuJoCo License Error

**Problem:** "License file not found" error

**Solution:**
```bash
# Download license (free for academic)
# From: https://www.deepmind.com/mujoco
# Place in: ~/.mujoco/mjkey.txt (Linux/Mac) or %USERPROFILE%/.mujoco/mjkey.txt (Windows)

# Or install mujoco with pip (handles license automatically)
pip install --upgrade mujoco
```

### Issue 7: "No module 'src.controller_*'"

**Problem:** Python path not set correctly

**Solutions:**
```bash
# Option A: Run from project root directory
cd /path/to/Applying-Optimal-Control-In-Mujoco
python examples/run_mujoco.py

# Option B: Add path explicitly
export PYTHONPATH="${PYTHONPATH}:/path/to/Applying-Optimal-Control-In-Mujoco"
```

### Issue 8: "DISPLAY cannot be opened" (SSH/headless)

**Problem:** Running on server without X11 forwarding

**Solution:**
```bash
# Use headless mode (no rendering)
python examples/run_mujoco.py --no-render

# Or set up X11 forwarding
ssh -X user@server
```

---

## First Simulation Walkthrough

### Step 1: Run LQG Controller (Simplest)

```bash
python examples/run_mujoco.py --controller lqg --duration 10 --no-render
```

**Expected console output:**
```
============================================================
  Controller : LQG [v4 — joint PD]
  Robot      : mini_cheetah
  Duration   : 10.0s  | Disturbance: impulse
  Waypoints  : 4
============================================================
  Phases: WARMUP 0-2.0s → WALK 2.0s+

  Initial joints (control reference):
    FL: [0.0, -2.2, 1.3]
    FR: [0.0, -2.2, 1.3]
    RL: [0.0, 0.9, 1.3]
    RR: [0.0, 0.9, 1.3]

  [Traj] 4 waypoints

  t= 0.0s [WARMUP   ] h=0.305m  pos=(+0.00,+0.00)  vx=+0.000 vx_f=+0.000  wz=+0.000
  t= 1.0s [WARMUP   ] h=0.305m  pos=(+0.00,+0.00)  vx=+0.000 vx_f=+0.000  wz=+0.000
  t= 2.0s [WALK     ] h=0.305m  pos=(+0.00,+0.00)  vx=+0.000 vx_f=+0.000  wz=+0.000
  ...
  
  --- LQG Summary ---
  RMSE=0.0165  MaxE=0.0923  TVU=62.1
```

### Step 2: Understand the Output

| Field | Meaning |
|-------|---------|
| `t=X.Xs` | Simulation time in seconds |
| `[WARMUP/WALK]` | Current phase |
| `h=0.305m` | Robot height (should stay ~0.30-0.31m) |
| `pos=(x,y)` | XY position in meters |
| `vx=±0.XXX` | Commanded velocity |
| `vx_f` | Filtered velocity (internal) |
| `wz=±0.XXX` | Yaw angular velocity |

### Step 3: Check Results File

```bash
# List generated plots
ls results/

# View the plot (on Linux with display)
display results/mujoco_lqg_mini_cheetah_impulse.png

# Or open with image viewer
open results/mujoco_lqg_mini_cheetah_impulse.png  # Mac
xdg-open results/mujoco_lqg_mini_cheetah_impulse.png  # Linux
```

### Step 4: Interpret the Plot

The output plot has 4 subplots:

**Subplot 1 - Position:**
- X, Y, Z markers showing position over time
- Dashed lines showing reference
- Should track smoothly

**Subplot 2 - Velocity:**
- X, Y, Z linear velocity components
- Should show desired velocity during walking

**Subplot 3 - Orientation:**
- Roll, pitch, yaw angles
- Should stay near zero for level walking

**Subplot 4 - Control and Disturbance:**
- Blue line: norm of ground reaction forces
- Gray shaded area: disturbance profile
- Shows when control effort is applied

---

## Progressive Examples

### Basic: Single Controller, No Disturbance

```bash
python examples/run_mujoco.py \
    --controller lqg \
    --disturbance none \
    --duration 15 \
    --no-render
```

**What to see:**
- Robot walks smoothly without perturbation
- Velocity increases gradually
- Reaches waypoints smoothly
- Very low control effort

---

### Intermediate: Compare Controllers

```bash
python examples/run_mujoco.py \
    --controller all \
    --duration 20 \
    --no-render
```

**What to see:**
- 3 individual plots (one per controller)
- Comparison plot showing overlay
- Summary table in console

**Analysis:**
- LQG: Smooth control, good stability
- MPC: Tightest tracking, highest effort
- PMP: Good overall, may have numerical issues

---

### Advanced: Custom Parameters

Create a modified `run_mujoco.py` with different:
- Gait period: `GAIT_PERIOD = 0.60` (faster walking)
- Step height: `STEP_HEIGHT = 0.06` (higher steps)
- Cost matrices: Modify `Q`, `R` for different control priorities
- Robot: `--robot-name go2` or `aliengo`

Example:
```bash
# Edit examples/run_mujoco.py, find line ~190:
# GAIT_PERIOD = 0.80  →  GAIT_PERIOD = 0.60

# Then run:
python examples/run_mujoco.py --controller mpc --duration 30
```

---

## Performance Tuning

### Faster Simulations

```bash
# Skip rendering (biggest time saver)
python examples/run_mujoco.py --no-render --duration 60

# Use headless Python (even faster)
python -O examples/run_mujoco.py --no-render
```

### Better Stability

```bash
# Reduce walking speed in run_mujoco.py:
# Line ~195: max_vx = 0.22  →  max_vx = 0.15

# Or increase joint control gains:
# Line ~158: KP_WALK = 32.0  →  KP_WALK = 40.0
```

### Better Tracking

```bash
# Increase position cost in cost function:
# Line ~540: Q = np.diag([80, 80, 300, ...])  →  Q = np.diag([150, 150, 300, ...])

# Decrease control cost:
# Line ~541: R = ... * 5e-3  →  R = ... * 1e-3
```

---

## Comparison Workflow

### Complete Controller Comparison Study

```bash
# 1. Run baseline (LQG)
python examples/run_mujoco.py --controller lqg --duration 15 --no-render

# 2. Run alternative (MPC)
python examples/run_mujoco.py --controller mpc --duration 15 --no-render

# 3. Run research baseline (PMP)
python examples/run_mujoco.py --controller pmp --duration 15 --no-render

# 4. Run all together for automatic comparison
python examples/run_mujoco.py --controller all --duration 15 --no-render

# 5. Analyze results
ls -lh results/  # Check file sizes
```

### Extract Metrics

```bash
# From console output, note down:
# For LQG: RMSE, MaxE, TVU
# For MPC: RMSE, MaxE, TVU  
# For PMP: RMSE, MaxE, TVU

# Create comparison table (manually or with script):
echo "Controller | RMSE   | MaxE   | TVU"
echo "-----------|--------|--------|-------"
echo "LQG        | 0.0165 | 0.0923 | 62.1"
echo "MPC        | 0.0128 | 0.0748 | 145.2"
echo "PMP        | 0.0142 | 0.0856 | 87.3"
```

---

## Troubleshooting Simulations

### Simulation Crashes Immediately

**Symptom:** "Terminated" message at t=0.1s

**Causes:**
1. Robot configuration issue
2. Gravity or physics settings wrong
3. Control gains too high

**Solutions:**
```bash
# Check with no control (debug):
# Edit run_mujoco.py, line ~750:
# tau = gait.compute_all_torques(...)  →  tau = np.zeros(12)

# Run again, should stay upright with gravity only
```

### Jerky Motions / High Acceleration

**Symptom:** Robot shakes or moves stiffly

**Causes:**
1. Control gains too high
2. Cost function too aggressive

**Solutions:**
```bash
# Reduce joint PD gains in run_mujoco.py, line ~158:
KP_WALK = 32.0  →  KP_WALK = 20.0
KD_WALK = 5.0   →  KD_WALK = 3.0

# Increase control cost (line ~541):
R = np.eye(12) * 5e-3  →  R = np.eye(12) * 1e-2
```

### Robot Can't Walk / Stays Still

**Symptom:** Velocity commands appear but robot doesn't move

**Causes:**
1. Waypoint distance too large
2. Friction coefficient too low
3. Gait parameters wrong

**Solutions:**
```bash
# Reduce waypoint distance in run_mujoco.py:
# Line ~183: distance_threshold=0.20  →  distance_threshold=0.10

# Reduce friction if overly conservative:
# Check MPC controller (src/controller_mpc.py): mu = 0.6
```

### NaN or Inf in Outputs

**Symptom:** "RuntimeWarning: invalid value encountered"

**Causes:**
1. Numerical instability in controller
2. State estimate diverged
3. Solver failed

**Solutions:**
```bash
# Use LQG instead (most numerically stable)
python examples/run_mujoco.py --controller lqg

# Or check cost matrices (line ~540)
# Ensure Q, R are positive definite:
# Q = np.diag([...])  # All diagonal elements > 0

# Reduce horizon for MPC (src/controller_mpc.py):
# N = 15  →  N = 10
```

---

## Next Steps

Once you've run a few basic simulations:

1. **Read [MATHEMATICAL_FRAMEWORK.md](MATHEMATICAL_FRAMEWORK.md)**
   - Understand the theory behind controllers
   - Learn how discretization and linearization work

2. **Read [CONTROLLER_GUIDE.md](CONTROLLER_GUIDE.md)**
   - Deep dive into PMP implementation
   - Understand LQR gain calculation
   - Learn MPC problem formulation

3. **Explore [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**
   - Understand module interactions
   - Learn how to extend the system

4. **Modify Parameters**
   - Change `Q` and `R` cost matrices
   - Adjust gait parameters
   - Test different robots

5. **Run Research Studies**
   - Compare controllers systematically
   - Test different disturbances
   - Measure scaling with robot size

---

## Quick Reference: Common Commands

```bash
# Basic tests
python examples/run_mujoco.py                           # Default (LQG)
python examples/run_mujoco.py --controller mpc          # Try MPC
python examples/run_mujoco.py --controller all          # Compare all

# With options
python examples/run_mujoco.py --duration 20 --no-render # Faster
python examples/run_mujoco.py --robot-name go2          # Different robot
python examples/run_mujoco.py --teleop                  # Interactive control
python examples/run_mujoco.py --disturbance persistent  # Harder test

# Combinations
python examples/run_mujoco.py --controller mpc --robot-name aliengo --disturbance persistent --duration 30 --no-render

# Check environment
python -c "import mujoco; print(f'MuJoCo {mujoco.__version__}')"
python -c "import gymnasium; print('Gymnasium OK')"
python -c "from src.controller_lqg import LQGController; print('Controllers OK')"
```

---

## Support Resources

| Issue | Resource |
|-------|----------|
| Math questions | [MATHEMATICAL_FRAMEWORK.md](MATHEMATICAL_FRAMEWORK.md) |
| Code questions | [API_REFERENCE.md](API_REFERENCE.md) |
| Architecture | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) |
| Gait control | [docs/GAIT_CONTROL.md](docs/GAIT_CONTROL.md) |
| Errors | [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) |
| Metrics | [docs/PERFORMANCE_ANALYSIS.md](docs/PERFORMANCE_ANALYSIS.md) |

---

**You're all set! Start with the basic commands and explore from there.**
