# mpc_baxter_robot

ROS Noetic development scaffold for Baxter MPC work with CUDA/NVIDIA GPU support.

## What is included

- Docker-based development environment
- NVIDIA CUDA base image and GPU runtime wiring
- Docker Compose service for local dev
- Taskfile commands for common workflows
- Baxter Lagrange-Euler dynamics package under `src/baxter_dynamics`
- Minimal `baxter_interface_cpp` limb client for torque command runtime wiring
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
- `.gitmodules` records submodule configuration when added