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
  - Joint position bounds
  - Joint velocity bounds
  - Acceleration and jerk limits
  - Optional end-effector z-axis floor constraint (via linearized kinematics)

- **Limb-agnostic**: Can control either left or right arm

## Dependencies

- ROS (Noetic or compatible)
- CasADi (C++ interface)
- IPOPT (nonlinear solver backend)
- baxter_kinematics (for optional EE constraint)
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
  safety:
    ee_z_constraint_enabled: false      # Enable EE z-floor constraint
    ee_z_min_m: -0.20                  # Min z height in meters
    ee_z_constraint_steps: 8           # Number of steps to enforce constraint
    ee_base_link: "base"                # Base link for kinematics
```

### Dynamic Reconfigure Parameters

Via `rqt_reconfigure` or command line:

- `pos_weight_X`: Position tracking weight for joint X
- `accel_weight_X`: Acceleration regularization weight for joint X
- `ref_X`: Target angle reference for joint X

## Usage

### Launch with default (left arm):
```bash
roslaunch mpc_controller mpc_controller.launch
```

### Launch with right arm:
```bash
roslaunch mpc_controller mpc_controller.launch limb:=right
```

### Direct execution:
```bash
rosrun mpc_controller run_mpc left
rosrun mpc_controller run_mpc right
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
- Fixed sampling time (0.1 s by default)
- Minimizes weighted tracking error plus regularization
- Faster computation, more predictable execution time

### Constraint Formulation

The NLP is formulated as discrete-time dynamics with:
- **Prediction Horizon**: 20 steps
- **Control Horizon**: 15 steps (acceleration after step 15 is held constant)
- **Dynamics**: Standard Euler integration
- **Cost**: Quadratic in position error and control acceleration

### End-Effector Constraint

When enabled, maintains z >= z_min via a linearized Jacobian-based constraint:

```
z_gate * (J_z * q - z_offset) >= 0
```

Recomputed every control cycle to track changing linearization point.

## Performance Optimization

- **Compiler flags**: O3 optimization, native architecture (-march=native)
- **Solver settings**: MUMPS linear solver, moderate verbosity
- **Asynchronous ROS**: 10 thread async spinner for non-blocking communication

Typical cycle time: 50-200 ms (solver-dependent)

## Future Improvements

- [ ] Warm-start with previous solution
- [ ] Disturbance observer for model mismatch
- [ ] Integration with trajectory libraries
- [ ] Multi-objective optimization (safety weight scheduling)

## Author

Maintained by Gentrit Fejza (gentritfejza@gmail.com)

## License

MIT
