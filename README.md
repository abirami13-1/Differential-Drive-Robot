# Differential Drive Robot — Modeling, Control, and Planning Simulation

This repository contains a simulation framework for a **2-wheeled differential drive robot**, covering:

- **Kinematic modeling** (global ↔ robot ↔ wheel frames)
- **Dynamic modeling** (mass/inertia/damping effects)
- **Sliding Mode Control (SMC)** for robust trajectory tracking
- **Adaptive Control** with online parameter estimation
- **A* (A-star) motion planning** on a grid map + obstacle avoidance simulation

---

## Features

### 1) Kinematics
Implements standard differential-drive kinematics under nonholonomic assumptions:
- Planar motion, pure rolling, no slip
- No lateral velocity in robot frame: \(v_y = 0\)
- Frame mappings:
  - **Global → Robot frame**
  - **Robot → Wheel velocities (inverse kinematics)**

Includes kinematic test cases such as:
- **Semi-circular path tracking**
- **In-place rotation**

---

### 2) Dynamics
Implements a physically realistic dynamic model that captures inertia and actuation effects.
Typical structure:
\[
M(q)\dot{v} + C(q,v)v + Dv = B\tau
\]
Where:
- \(M\): inertia / mass matrix  
- \(C\): Coriolis / coupling terms  
- \(D\): damping/friction model  
- \(\tau\): wheel torques / inputs  

Dynamic simulation demonstrates stable tracking of curved trajectories (e.g., figure-8) with realistic torque and velocity behavior.

---

### 3) Sliding Mode Control (SMC)
Implements SMC for robust tracking under disturbances and modeling uncertainty.
- Uses a **sliding surface** combining lateral and heading errors
- Drives \(s(t)\to 0\) to ensure convergence robustness

Typical observed behavior:
- Faster convergence and smaller steady-state tracking error
- Higher control effort with oscillations / chattering-like behavior (reduced using saturation/boundary layers)

---

### 4) Adaptive Control
Implements an adaptive controller that estimates unknown dynamic parameters online.

**Unknown/estimated parameters:**
- Mass \(m\)
- Yaw inertia \(I_z\)
- Damping \(b\)

Uses adaptation gains (learning rates) for each estimate:
- \(\gamma_m, \gamma_I, \gamma_b\)

Typical observed behavior:
- Smoother control than SMC (no chattering)
- Gradual reduction in tracking error as parameters converge
- Parameter estimates stabilize as excitation persists (e.g., figure-8 trajectory)

---

### 5) Motion Planning — A* Algorithm
Implements **A*** search on a 2D grid:
- Grid representation: free = 0, obstacle = 1
- Cost: path length
- Heuristic: Manhattan or Euclidean distance
- Output path → waypoints → reference trajectory for the controller

Obstacle avoidance simulation supports:
- **Static obstacles** (blocked grid cells)
- **Dynamic obstacles** (moving objects)
- Reactive avoidance behavior based on proximity + short-horizon prediction

---


---

## How to Run

### Option 1: Python scripts
1. Create and activate a virtual environment (recommended)
2. Install dependencies:
```bash
pip install -r requirements.txt


Run the desired Module
python src/kinematics.py
python src/dynamics.py
python src/smc_controller.py
python src/adaptive_controller.py
python src/astar_planner.py


