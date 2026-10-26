# MPC Controller

A Model Predictive Controller (MPC) for joint-space trajectory planning on the Baxter robot using CasADi and IPOPT.

## Overview

This package implements a discrete-time, finite-horizon MPC for trajectory planning. The controller optimizes a cost function over a prediction horizon while respecting joint position, velocity, and acceleration constraints.

### Features

- **Two solver modes**:
  - `time_scaling`: Optimizes total trajectory time along with accelerations
  - `qc` (Quadratic Cost): Uses fixed sampling time with quadratic cost minimization

- **Dynamic reconfiguration**: Real-time tuning of:
  - Position and acceleration weights
  - Reference joint angles

- **Safety constraints**:
  - Joint position bounds (soft, slack-penalised)
  - Joint velocity bounds (soft, slack-penalised)
  - Acceleration and jerk limits (hard)
  - Braking fallback command when a solve fails

- **Limb-agnostic**: Can control either left or right arm

## Dependencies

- ROS (Noetic or compatible)
- CasADi (C++ interface)
- IPOPT (nonlinear solver backend)
- baxter_interface_cpp (for robot control)
- dynamic_reconfigure (for parameter tuning)

## Configuration

### Robot Parameters (`config/config.yaml`)

```yaml
robot:
  solver_mode: "qc"                    # "qc" or "time_scaling"
  acceleration_max: [...]              # Max acceleration per joint [rad/s²]
  acceleration_min: [...]              # Min acceleration per joint [rad/s²]
  jerk_max: [...]                      # Max jerk per joint [rad/s³]
  jerk_min: [...]                      # Min jerk per joint [rad/s³]
```

### Dynamic Reconfigure Parameters

Via `rqt_reconfigure` or command line:

- `pos_weight_X`: Position tracking weight for joint X
- `accel_weight_X`: Acceleration regularization weight for joint X
- `ref_X`: Target angle reference for joint X

## Usage

### Launch with default (left arm):
```bash
roslaunch baxter_mpc_controller mpc_controller.launch
```

### Launch with right arm:
```bash
roslaunch baxter_mpc_controller mpc_controller.launch limb:=right
```

### Direct execution:
```bash
rosrun baxter_mpc_controller run_mpc left
rosrun baxter_mpc_controller run_mpc right
```

### Tune parameters:
```bash
rqt_reconfigure  # Then select mpc_controller_node_left or mpc_controller_node_right
```

## Implementation Details

### Solver Modes

#### Time-Scaling Mode
- Adds a decision variable `T` (total trajectory time)
- Minimizes `T` plus weighted tracking and acceleration costs
- Useful for time-optimal trajectories

#### Quadratic Cost Mode
- First step = control period (20 ms at 50 Hz), remaining steps spread over the prediction
  duration (1.5 s)
- Minimizes weighted tracking error plus acceleration regularization plus a quadratic slack penalty (an L1 slack term makes OSQP stall on large reference jumps)
- Convex QP, solved by OSQP with warm start in ~2 ms

### Constraint Formulation

The problem is formulated as discrete-time double-integrator dynamics with:
- **Prediction Horizon**: 25 steps over 1.5 s
- **Control Horizon**: 12 steps (acceleration after step 12 is held constant)
- **Dynamics**: exact double-integrator update per step
- **Cost**: quadratic in position error and control acceleration
- **State bounds**: joint position/velocity limits with one non-negative slack per joint per
  step, so a measured state outside the limits never makes the QP infeasible
- **Reference to the torque loop**: the measured state at solve time plus the first
  acceleration; the torque loop integrates it over the command age

## Performance Optimization

- **Compiler flags**: O3 optimization, native architecture (-march=native)
- **Solver settings**: MUMPS linear solver, moderate verbosity
- **Asynchronous ROS**: 10 thread async spinner for non-blocking communication

Typical cycle time: 50-200 ms (solver-dependent)

## Future Improvements

- [x] Warm-start with previous solution
- [ ] Disturbance observer for model mismatch
- [ ] Integration with trajectory libraries
- [ ] Multi-objective optimization (safety weight scheduling)

## Author

Maintained by Gentrit Fejza (gentritfejza@gmail.com)

## License

MIT
