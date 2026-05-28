# mpc_baxter_robot

ROS Noetic development scaffold for Baxter MPC work with CUDA/NVIDIA GPU support.

## What is included

- Docker-based development environment
- NVIDIA CUDA base image and GPU runtime wiring
- Docker Compose service for local dev
- Taskfile commands for common workflows
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
- `.gitmodules` records submodule configuration when added