# Rover Docker environments

Run from the repository root:

```bash
docker compose -f infrastructure/docker/docker-compose.yml config --quiet
docker compose -f infrastructure/docker/docker-compose.yml run --build --rm ci
```

The `ci` service builds the workspace and runs the clock/node-topic smoke command.
`dev` provides the same Humble/Gazebo build environment interactively. `runtime`
is a minimal ROS base shell, not a prebuilt complete rover runtime. There is no
`sim` service and no privileged hardware access in the simulation command.

See [runtime boundaries](../../docs/correctness-and-runtime.md). Compose configuration
validation does not prove image builds, ROS runtime behavior, motion or determinism.
