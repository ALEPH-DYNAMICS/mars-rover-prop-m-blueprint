# Mars Rover PROP-M Blueprint

Simulation-first Mars rover research stack inspired by the Soviet PrOP-M concept and rebuilt around a modern ROS 2 workspace.

This repository currently contains:
- Robot description assets (URDF/Xacro)
- Control and ros2_control configuration
- State-estimation bringup
- Navigation configuration for a modern mode
- ROS-clock mission phase-label publication (no drive/measure/transmit actions)
- Gazebo simulation assets
- Dataset and metrics tooling
- Core CI for the non-Gazebo workspace

---

## Mission Philosophy

PROP-M was simple:
Drive -> Stop -> Measure -> Transmit -> Repeat.

This project preserves that deterministic philosophy while enabling modern autonomy extensions.

Two intended mode profiles (the capabilities below are conceptual unless verified):

1. **PROP-M Mode**
   - Tether-limited radius (~15 m)
   - Cyclic movement logic
   - Contact-triggered obstacle avoidance
   - Deterministic mission cadence

2. **Modern Mode**
   - Full Nav2 navigation
   - SLAM-ready
   - Recovery behaviors
   - Modular autonomy stack

---

## System Architecture

ROS 2 workspace (`ros_ws/`):

- `rover_description` -> Robot model, inertials, transmissions
- `rover_control` -> Simulation command shaping; drive-controller wiring is incomplete
- `rover_estimation` -> Odometry + state fusion
- `rover_navigation` -> Nav2 configuration
- `rover_mission_bt` -> Mission-cycle runner and authored mission trees
- `rover_sim_gazebo` -> Simulation environments
- `rover_tools` -> Dataset export + metrics

---

## Verification boundary

The bounded correctness pass adds pure-Python regressions for control shaping,
metrics, schema validation and packaging. It provides a separate simulator smoke
command that requires observed `/clock`, `/mission/state` and `/cmd_vel_safe` messages.
A successful core package build does not establish a running simulator.

The full ROS/Gazebo runtime was not available in the repair environment. Controller
plugin/spawner wiring, `/cmd_vel_safe` actuation, odometry and mission actions remain
unverified/incomplete. See [the exact executable subset and checks](docs/correctness-and-runtime.md).

---

## Current Maturity

This is an early engineering repository, not a finished flight or field stack.

- Control shaping has software regressions; estimation/navigation runtime integration remains unverified.
- Generic CI validates the core workspace on ROS 2 Humble without the Gazebo package set.
- Simulation bringup is present; the separate integration workflow must be assessed by its actual result.
- The mission runner publishes phase labels on the ROS clock. It neither executes the XML tree nor performs drive, measure or transmit actions.
- The hardware bringup path is an explicit placeholder until a validated driver stack exists.
- Dynamics, inertials, and terramechanics parameters still include first-pass estimates pending calibration.
- Reproducibility depends on a ROS 2 / Gazebo environment with the declared package set available.

## What CI Proves

The `core-build (humble)` workflow is intended to prove one narrow thing well:

- the core ROS 2 workspace installs dependencies, builds, and reaches test-result reporting cleanly on Ubuntu 22.04 / ROS 2 Humble

It does not prove:

- Gazebo simulation fidelity
- hardware-driver readiness
- calibrated rover dynamics
- a full end-to-end mission autonomy runtime

That boundary is intentional. The repository is stronger when CI claims less and proves it consistently.

## Quickstart (Simulation)

From the repository root, with Docker Engine and Compose available:

```bash
docker compose -f infrastructure/docker/docker-compose.yml config --quiet
docker compose -f infrastructure/docker/docker-compose.yml run --build --rm ci
```

The `ci` service builds the ROS workspace and runs the clock/node-topic smoke test.
It returns failure when required observations are absent; a timeout is not success.
This command has no privileged hardware access. A local Docker/ROS runtime is
required; configuration validation alone is not evidence of simulation execution.
Actual rover motion is outside the currently verified subset.

---

## Scenarios

| Scenario | Purpose |
|----------|----------|
| mars_flat | Kinematics + control validation |
| mars_rocks | Obstacle avoidance stress |
| mars_dunes | Slip + terrain interaction |
| lander_tether_site | PROP-M tether realism |

Scenario assets and regression workflow definitions are included in the repository. Their value depends on the local or CI environment being able to build and run the ROS 2 stack.

---

## Reproducibility

The intended run artifact set is:

- MCAP log
- run_metadata.json
- metrics.json

The packaging entry points validate the full installed JSON Schema before
publishing an output directory. The explicit `--gate --thresholds` evaluator
rejects partial evidence and missing/failed bounds. Existing scenario YAMLs have
no quantitative thresholds: acceptance remains unconfigured until justified
values are supplied. Exploratory metrics are not an acceptance pass.

---

## Engineering Standards

- Simulation-first development
- No feature without metrics
- No autonomy without logging
- No merge without CI passing
- No undocumented parameter

---

## Licensing

Software: Apache 2.0  
Hardware designs: CERN-OHL  
Datasets: CC-BY 4.0  

See `/LICENSES` for details.

---

## Roadmap

v0.1 – Simulation MVP  
v0.2 – Navigation + scenario regression  
v0.3 – Full PROP-M mission loop  
v1.0 – Terramechanics + calibrated dynamics  

---

## Research Intent

_This repository is a technical reconstruction and modernization of early planetary surface mobility concepts, implemented as a ROS 2 research prototype with explicit room for calibration and integration hardening._
