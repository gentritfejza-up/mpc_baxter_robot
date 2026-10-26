# mpc_baxter_robot

ROS Noetic development scaffold for Baxter MPC work with CUDA/NVIDIA GPU support.

## What is included

- Docker-based development environment
- NVIDIA CUDA base image and GPU runtime wiring
- Docker Compose service for local dev
- Taskfile commands for common workflows
- Baxter Lagrange-Euler dynamics package under `src/baxter_dynamics` (DH model, verified against KDL)
- URDF-based KDL inverse dynamics torque loop under `src/baxter_kdl_dynamics`
- Minimal `baxter_interface_cpp` limb client for torque command runtime wiring
- Model Predictive Controller (MPC) package under `src/baxter_mpc_controller`
- Submodule manifest placeholder

## Quick Start

```bash
task docker:build
task docker:up
task docker:shell
```

Full simulation stack (headless Gazebo, KDL computed-torque loop, MPC, 20 body-clearance-checked
test poses, then a tracking summary):

```bash
task run:validate POSES=20 DWELL=6 GUI=true   # GUI=false for headless
```

## Control architecture

```
reference pose (dynamic_reconfigure ref_i)
        |
   MPC node (50 Hz, OSQP QP, 1.5 s horizon)   -> acceleration_command: a, q0, v0 (measured at solve time)
        |
   torque loop (800 Hz, KDL or Lagrange-Euler) -> qdd = a [+ Kp (q_ref - q) + Kd (v_ref - v)]
        |                                          q_ref = q0 + v0 t + a t^2 / 2  (t = command age)
        |                                          PD off in Gazebo (inner_pd:=false, default),
        |                                          on for the real robot (inner_pd:=true)
        |                                          tau = M qdd + C + G (+ URDF damping)
   /robot/limb/<side>/joint_command (TORQUE_MODE)
```

- Joint position and velocity limits in the MPC are **soft** (slack variables), so a measured
  state outside the limits (contact impact, overspeed) never makes the QP infeasible.
  Acceleration and jerk limits stay hard.
- If a solve still fails, the MPC commands a braking acceleration instead of going silent, so
  the torque loop never times out into position mode mid-motion. A solution that misses the
  cycle deadline is still commanded (late beats a torque-loop timeout).
- The base state (q0, v0) carried with each acceleration is propagated through the MPC's own
  solve latency before publishing, so the torque loop's PD reference never lags the plant
  (otherwise the PD fights the feedforward and the arm reaches only ~40 % of the commanded
  acceleration).
- The MPC cycle runs on the ROS clock, so the model step matches simulated time under Gazebo.
- Gazebo/ros_control enforce the URDF joint velocity limits (1.5 rad/s shoulders/elbows,
  4 rad/s wrists) by clamping the effort to one sign; the MPC therefore keeps its own velocity
  bounds at 80 % of those limits and penalises velocity slightly so the model stays valid.

## Structure

- `docker/Dockerfile` builds the development image
- `docker/docker-compose.yml` defines the local dev service
- `Taskfile.yml` wraps the common commands
- `src/baxter_dynamics` contains the Lagrange-Euler dynamics library
- `src/baxter_dynamics/config/dynamics_params.yaml` provides the Baxter model and torque limits
- `src/baxter_dynamics/launch/lagrange_euler.launch` runs the computed-torque node
- `src/baxter_interface_cpp` provides the C++ limb interface used by the node
- `src/baxter_mpc_controller` contains the Model Predictive Controller for trajectory planning
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

**Computed-torque PD (optional, zero gains = pure feedforward):**
```yaml
robot.lagrange_euler:
  kp, kd                           # one gain per joint around the MPC trajectory
  extrapolate_reference            # track q0 + v0 t + a t^2/2 from the MPC sample time
  max_reference_extrapolation_sec
```

#### Model notes

- The YAML stores URDF inertia tensors (about each link's centre of mass, URDF sign
  convention) rotated into the DH link frames. `inertia_mat()` shifts them to the DH frame
  origin (parallel-axis theorem) and negates the products of inertia to form the
  Lagrange-Euler pseudo-inertia matrix. The rostest `lagrange_euler_kdl.test` checks M, C and
  G against KDL built from the live URDF (agreement ~1e-3).
- Link 7 lumps wrist + hand + gripper base (fixed joints). The electric gripper *finger*
  links in the URDF carry a `0.01 kg m²` inertia each (0.02 kg bodies, ~1000x too large
  physically). Do not "fix" them for simulation: with realistic values the simulated w2
  inertia becomes so small that ODE joint damping and stop friction dominate and w2 no longer
  tracks (validated). The KDL backend lumps the fingers into its tip (matching the simulated
  plant); the Lagrange-Euler model does not (matching the real robot).

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
roslaunch baxter_mpc_controller mpc_controller.launch
```

#### Launch for right arm:
```bash
roslaunch baxter_mpc_controller mpc_controller.launch limb:=right
```

#### Direct execution:
```bash
rosrun baxter_mpc_controller run_mpc left
rosrun baxter_mpc_controller run_mpc right
```

#### Parameters (in `config/config.yaml`):

**Solver Configuration:**
```yaml
robot.solver_mode                 # "qc" (quadratic cost) or "time_scaling" (time-optimal)
robot.solver_backend              # qc: osqp | qpoases | ipopt
robot.solver.soft_state_bounds    # slack on joint position/velocity limits (qc only)
robot.solver.slack_linear_weight / slack_quadratic_weight   # keep linear at 0 (OSQP stalls otherwise)
robot.solver.velocity_weight_ratio   # velocity penalty as a fraction of the position weight
robot.solver.feasibility_tolerance
robot.solver.osqp.{eps_abs,eps_rel,max_iter,polishing,warm_start}
robot.control.mpc_rate_hz / prediction_duration_sec / prediction_horizon / control_horizon
robot.control.braking_fallback    # command a stop instead of nothing when a solve fails
robot.control.publish_on_deadline_miss / max_latency_compensation_sec
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
  pos_weight_0 to pos_weight_6     # Weight for tracking each joint's position (0.0 - 1000.0)

Acceleration Weights:
  accel_weight_0 to accel_weight_6 # Regularization weight for acceleration (0.0 - 1.0)

Reference Configuration:
  ref_0 to ref_6                   # Target joint angle for each joint (within joint limits)
```

#### Solver Modes Explained

**Quadratic Cost Mode (qc):**
- First prediction step = one control period, remaining steps spread over `prediction_duration_sec`
  (1.5 s by default: the horizon must cover a full stop from the velocity limit)
- Minimizes: weighted tracking error + acceleration regularization + slack penalties
- Convex QP solved by OSQP in ~2 ms
- Use for: realtime trajectory tracking with fixed timing

**Time-Scaling Mode:**
- Variable total trajectory time (decision variable)
- Minimizes: trajectory time + tracking + acceleration costs
- Slower but generates time-optimal paths
- Use for: offline motion planning, minimum-time trajectories

#### Example: Real-time Parameter Tuning
```bash
# Terminal 1: Start the MPC controller
roslaunch baxter_mpc_controller mpc_controller.launch limb:=left

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
- Typical cycle time: 3-5 ms with OSQP (qc mode), 50-200 ms with IPOPT (time_scaling)
- Asynchronous ROS spinner (10 threads) for non-blocking communication
- Optimized with -O3 and -march=native flags

### Simulation validation

`roslaunch baxter_mpc_controller baxter_mpc.launch` starts Gazebo, the KDL torque loop, the MPC,
enables the robot and sends `pose_count` deterministic test poses. The pose generator rejects
poses (and the predicted transit from the previous pose, each joint moving time-optimally within its limits) that would put the arm inside the torso/head column, the parked other arm, or the floor
(`send_sim_references.py`, `ArmClearance`); without it ~10 % of random poses self-collide and
stall. `record_control_diagnostics.py` writes a JSON-lines log that
`summarize_control_diagnostics.py` turns into settle times, overshoot, final errors and solver
statistics.

### Real robot

The real Baxter adds its own gravity compensation in torque mode, so launch the KDL loop with
`add_gravity_compensation:=false` there (keep it `true` in Gazebo). Keep the URDF s1 effort
limit at 50 Nm on the real robot; the simulator submodule raises it to 100 Nm only because the
simulated joint has to carry gravity itself.

---