# Correctness contracts and runtime boundary — 6 September 2026

The repair concerns existing simulation research software. Unit-test fixtures are
synthetic; none are reported as recorded bags, motion measurements or hardware evidence.

## Executable subset

The control node shapes `/cmd_vel` into `/cmd_vel_safe`; the mission node publishes
phase labels. The XML tree is a traceability asset, not an executed mission tree.
The Gazebo extension Xacro is still a placeholder: no Gazebo ros2_control system
plugin or controller spawners connect the configured drive controller to
`/cmd_vel_safe`. The phase runner sends no drive, measurement or transmission action.
Estimation lacks established live sensor/odometry evidence. Filling this complete
integration is larger than this correctness pass; no hardware driver is added.

`python3 scripts/sim_smoke.py --out analysis/sim-smoke.json` observes an advancing
Gazebo `/clock`, mission labels and finite shaped commands, and fails on missing
observations or early launch exit. This is a clock/node-topic smoke test only.
It does not verify entity spawn, drive motion, odometry, mission completion or
physics fidelity. Launch errors and observations are saved alongside the report.
The repair environment has no ROS/Gazebo or Docker Engine, so runtime checks are
blocked there. A core package build cannot replace them.

## Control, mode and clock

Only `modern` and `prop_m` are valid mode values. Bringup selects exactly one mode
YAML; its `limits.*.<mode>` parameter names are the names consumed by control.
Finite positive velocity/acceleration limits and timeouts are required; deadbands
are finite and nonnegative. Invalid configuration inhibits output. Invalid commands
are discarded and the output ramps toward zero using the existing acceleration bounds.
When slip containment is enabled, absent, invalid, out-of-range or stale slip evidence
also requests a ramp to zero. Slip age defaults to 0.5 simulated seconds and is configurable
as `slip.timeout_s`; it is a simulation policy, not a certified stopping guarantee.

The node and phase runner use their ROS clocks, including `use_sim_time`. The pure
shaper takes an injected clock: paused time does not advance ramps; backward time
clears command/slip freshness and output. The phase runner resets to STOP_MEASURE
on a backward clock jump. Large shaper steps are capped at 0.2 seconds.

The batch seed is forwarded through bringup to Gazebo's documented `seed` launch
argument ([upstream gzserver launch](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_ros/launch/gzserver.launch.py)).
Control and the phase runner have no RNG. Nav2 has no seed wiring in this checkout.
Metadata records requested configuration provenance, not measured physics or proof
of determinism. The recorder uses `/clock` stamps but runs for a wall-time duration;
pause/rate differences can change bag coverage. No end-to-end repeatability is claimed.

## Evidence and acceptance

Install and test the Python subset without ROS:

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements-test.txt ./ros_ws/src/rover_tools ./ros_ws/src/rover_control
python -m unittest discover -s tests -v
python scripts/run_sim_batch.py --help
python scripts/evaluate_metrics.py --help
(cd /tmp && rover_metrics --help && rover_tools metrics --help)
```

Use Python 3.12 for this pinned verification environment. The installed evaluator
and schema have no dependency on locating a source checkout. The existing
`datasets/schemas/run_metadata.schema.json` path links to the single packaged schema.
Draft 2020-12 validation includes RFC3339 `date-time` checks and rejects nonfinite
JSON numbers and boolean seeds. Invalid metadata is rejected before a bag is copied;
copy/validation failures remove the staging directory. A completed dataset is made
visible by renaming its staging directory on the same filesystem.

`integrity.metadata_sha256` hashes canonical JSON, excluding only that field:
UTF-8, sorted keys, compact comma/colon separators, ASCII escaping, no NaN. It is
not a hash of saved-file bytes. `mcap_sha256` hashes the final bag bytes. Independent
readback tests verify both. Root Xacro provenance is explicitly labeled as a source
hash in batch notes; it is not the hash of a runtime-expanded robot model.

Exploratory `rover_metrics DATASET` writes partial output when ROS bag reading is
unavailable. For acceptance, run:

```bash
rover_metrics DATASET --gate --thresholds scenarios/mars_flat/scenario.yaml
```

`thresholds` is a nonempty mapping from a metric path (for example
`mission.stop_measure_max_drift_m`) to numerical `min` and/or `max` bounds. Bounds
are inclusive. Unknown metrics, nonfinite values, invalid bounds, missing artifacts,
partial coverage and threshold violations return nonzero. The scenario name must
match dataset metadata. **No existing scenario defines quantitative thresholds**;
the gate therefore cannot pass until justified acceptance values are supplied.
No default values are invented or lowered by this repair.

Stop drift is the maximum sampled displacement from each continuous STOP_MEASURE
interval's start, combining repeated state publications. Interval boundaries use
linear interpolation between recorded odometry samples, never extrapolation.
Every stop requires two usable in-window samples and bracketing boundary coverage;
missing, unclosed or nonfinite windows yield insufficient evidence. Sampling can
miss motion between samples; this is an odometry-derived metric, not ground truth.

For exploratory batch capture on a sourced ROS installation:

```bash
python3 scripts/run_sim_batch.py --scenario mars_flat --backend gazebo --mode modern --runs 1 --seed 42 --duration 45 --no-nav2 --report analysis/regression-batch.json
python3 scripts/gate_batch.py analysis/regression-batch.json --thresholds scenarios/mars_flat/scenario.yaml
```

The report names the exact run directories; the gate never guesses the newest bag.
Batch failures return nonzero. Recording accepts one MCAP file, not split bags.
Without the missing actuation/odometry integration, complete mobility evidence is
not expected. A partial report does not satisfy acceptance.

## CI and dependency failure

The inspected [16 June job](https://github.com/ELION-DYNAMICS/mars-rover-prop-m-blueprint/actions/runs/27601122600/job/81602083894)
failed because `libgoogle-glog-dev` could not satisfy `libunwind-dev` / `libunwind7-dev`.
The repair uses a clean `ros:humble-ros-base-jammy` container and explicitly installs
`libunwind-dev`, avoiding dependency on the hosted image's development-package set.
The exact prior package conflict and successful installation in the new container
still require a live run; a static edit is not a verified resolution.

Core CI includes runtime dependencies but still skips building `rover_sim_gazebo`.
A separate Python job tests behavior, installed CLIs and Compose configuration.
The simulator job builds all packages, observes the limited smoke subset, records
actual evidence and runs explicit acceptance. Missing prerequisites and skipped
steps remain failures/blocked checks, never successful simulation evidence.

GitHub had automatically disabled the old scheduled simulation workflow for
inactivity. The PR correctness workflow calls that same definition as a reusable
job, so repair-commit simulation checks can run without re-enabling the dormant
nightly schedule or duplicating its implementation.
