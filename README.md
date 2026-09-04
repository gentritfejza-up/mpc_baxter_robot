# mpc_baxter_robot

ROS Noetic development scaffold for Baxter MPC work with CUDA/NVIDIA GPU support.

## What is included

- Docker-based development environment
- NVIDIA CUDA base image and GPU runtime wiring
- Docker Compose service for local dev
- Taskfile commands for common workflows
- Baxter Lagrange-Euler dynamics package under `src/baxter_dynamics`
- Minimal `baxter_interface_cpp` limb client for torque command runtime wiring
- Model Predictive Controller (MPC) package under `src/mpc_controller`
- Submodule manifest placeholder

## Quick Start

```bash
task docker:build
task docker:up
task docker:shell
```

## Structure

- `docker/Dockerfile` builds the development image
- `docker/docker-compose.yml` defines the local dev service
- `Taskfile.yml` wraps the common commands
- `src/baxter_dynamics` contains the Lagrange-Euler dynamics library
- `src/baxter_dynamics/config/dynamics_params.yaml` provides the Baxter model and torque limits
- `src/baxter_dynamics/launch/lagrange_euler.launch` runs the computed-torque node
- `src/baxter_interface_cpp` provides the C++ limb interface used by the node
- `src/mpc_controller` contains the Model Predictive Controller for trajectory planning
- `.gitmodules` records submodule configuration when added

---

## Running Standalone Nodes

### Lagrange-Euler Dynamics Node

The Lagrange-Euler node computes robot dynamics using the Lagrangian mechanics formulation.

#### Launch with default parameters (left arm):
```bash
roslaunch baxter_dynamics lagrange_euler.launch
```

#### Launch for right arm:
```bash
roslaunch baxter_dynamics lagrange_euler.launch limb:=right
```

#### Direct execution:
```bash
rosrun baxter_dynamics run_lagrange_euler left
rosrun baxter_dynamics run_lagrange_euler right
```

#### Parameters (in `config/dynamics_params.yaml`):

**Robot Model Parameters:**
```yaml
robot.limb:
  segment_names         # Segment identifiers for the kinematic chain
  left/right           # Joint names for left/right limbs
  segment_lengths      # DH segment lengths (l1, d1, l2, d2, l3, d3, l4, d4)
  angle_limits         # Joint position constraints per joint (s0-s1, e0-e1, w0-w2)
  dh_parameters        # Denavit-Hartenberg parameters for each joint
```

**Inertia Parameters:**
```yaml
robot.limb.inertia:
  Ixx, Iyy, Izz       # Principal moments of inertia (7 values, one per joint)
  Ixy, Iyz, Izx       # Products of inertia (7 values each)
```

**Center of Mass and Link Properties:**
```yaml
robot.limb:
  com_x, com_y, com_z  # Center of mass position for each link (7 values)
  com_w                # COM weights (typically 1.0)
  link_masses          # Mass of each link (7 values)
  Q_j                  # Joint axis definitions (16 values: fixed 0,-1 pattern)
```

**Torque Limits:**
```yaml
robot.torque_limits:
  shoulder_elbow_max   # Max torque for shoulder/elbow joints: 50.0 N⋅m
  shoulder_elbow_min   # Min torque for shoulder/elbow joints: -50.0 N⋅m
  wrist_max           # Max torque for wrist joints: 20.0 N⋅m
  wrist_min           # Min torque for wrist joints: -20.0 N⋅m
```

#### Example: Running with custom config
```bash
# Load custom dynamics parameters
rosparam load custom_dynamics.yaml
rosrun baxter_dynamics run_lagrange_euler left
```

---

### MPC Controller Node

The MPC controller implements Model Predictive Control for joint-space trajectory planning using CasADi and IPOPT.

#### Launch with default parameters (left arm):
```bash
roslaunch mpc_controller mpc_controller.launch
```

#### Launch for right arm:
```bash
roslaunch mpc_controller mpc_controller.launch limb:=right
```

#### Direct execution:
```bash
rosrun mpc_controller run_mpc left
rosrun mpc_controller run_mpc right
```

#### Parameters (in `config/config.yaml`):

**Solver Configuration:**
```yaml
robot.solver_mode         # "qc" (quadratic cost) or "time_scaling" (time-optimal)
```

**Robot Dynamics Limits:**
```yaml
robot:
  acceleration_max        # Max acceleration per joint [rad/s²] (7 values)
  acceleration_min        # Min acceleration per joint [rad/s²] (7 values)
  jerk_max               # Max jerk per joint [rad/s³] (7 values)
  jerk_min               # Min jerk per joint [rad/s³] (7 values)
```

**Safety Constraints:**
```yaml
robot.safety:
  ee_z_constraint_enabled      # bool - Enable end-effector z-floor constraint
  ee_z_min_m                   # float - Minimum z height in meters (e.g., -0.20)
  ee_z_constraint_steps        # int - Number of prediction steps to enforce constraint
  ee_base_link                 # string - Base link for kinematics ("base")
```

#### Dynamic Reconfiguration

Tune parameters in real-time without stopping the node:

```bash
rqt_reconfigure
```

Then select `mpc_controller_node_left` or `mpc_controller_node_right` (depending on your limb).

**Tunable Parameters:**

```
Position Weights:
  pos_weight_0 to pos_weight_6     # Weight for tracking each joint's position (0.0 - 100.0)

Acceleration Weights:
  accel_weight_0 to accel_weight_6 # Regularization weight for acceleration (0.0 - 0.1)

Reference Configuration:
  ref_0 to ref_6                   # Target joint angle for each joint (within joint limits)
```

#### Solver Modes Explained

**Quadratic Cost Mode (qc):**
- Fixed sampling time: 0.1 s
- Minimizes: weighted tracking error + acceleration regularization
- Faster computation, predictable execution
- Use for: realtime trajectory tracking with fixed timing

**Time-Scaling Mode:**
- Variable total trajectory time (decision variable)
- Minimizes: trajectory time + tracking + acceleration costs
- Slower but generates time-optimal paths
- Use for: offline motion planning, minimum-time trajectories

#### Example: Real-time Parameter Tuning
```bash
# Terminal 1: Start the MPC controller
roslaunch mpc_controller mpc_controller.launch limb:=left

# Terminal 2: Open GUI for tuning
rqt_reconfigure
# Adjust pos_weight_X, accel_weight_X, and ref_X values in real-time
```

---

## Performance Notes

### Lagrange-Euler Node
- Lightweight computation (mostly parameter lookups and model formation)
- No external solver dependencies
- Suitable for high-frequency dynamics queries

### MPC Controller Node
- Typical cycle time: 50-200 ms (solver-dependent)
- Uses IPOPT with MUMPS linear solver
- Asynchronous ROS spinner (10 threads) for non-blocking communication
- Optimized with -O3 and -march=native flags

---