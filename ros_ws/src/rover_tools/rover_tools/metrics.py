#!/usr/bin/env python3
"""
scripts/evaluate_metrics.py

Exploratory evidence extraction and explicit acceptance evaluation for a packaged dataset.

Inputs:
  datasets/<run_id>/
    run.mcap
    run_metadata.json

Outputs:
  datasets/<run_id>/metrics.json

Design intent:
- Deterministic, auditable metrics derived from recorded evidence (MCAP).
- Hard fail on missing required artifacts.
- Degrade explicitly if rosbag2_py is unavailable (no silent success).

Metrics (Phase 0):
- run duration (s)
- message counts per required topic
- mission phase durations (DRIVE / STOP_MEASURE / TRANSMIT_LOG / REPEAT / DONE)
- commanded distance proxy (integral of cmd_vel_safe linear.x)
- achieved distance proxy (odom path length)
- slip proxy: 1 - achieved/commanded (clamped)
- stop-measure drift (max displacement during STOP_MEASURE)

This script is intentionally conservative: if it cannot compute a metric, it records
"status": "partial" and lists why.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


# -----------------------------
# Utilities
# -----------------------------

def read_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, obj: Dict[str, Any]) -> None:
    path.write_text(json.dumps(obj, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def stamp_to_sec(stamp_msg: Any) -> float:
    # builtin_interfaces/msg/Time
    return float(stamp_msg.sec) + float(stamp_msg.nanosec) * 1e-9


# -----------------------------
# Topic names (program defaults)
# -----------------------------

TOPIC_CMD = "/cmd_vel_safe"
TOPIC_ODOM = "/odometry/filtered"
TOPIC_STATE = "/mission/state"


# -----------------------------
# Optional rosbag2 parsing
# -----------------------------

@dataclass
class BagSample:
    t: float
    msg: Any


def try_import_rosbag() -> Tuple[bool, str]:
    try:
        import rosbag2_py  # noqa: F401
        import rclpy.serialization  # noqa: F401
        import rosidl_runtime_py.utilities  # noqa: F401
        return True, ""
    except Exception as e:
        return False, str(e)


def load_mcap_samples(mcap_path: Path, topics_of_interest: List[str]) -> Tuple[Dict[str, List[BagSample]], List[str]]:
    """
    Read MCAP with rosbag2_py SequentialReader. Returns samples and warnings.
    """
    warnings: List[str] = []
    samples: Dict[str, List[BagSample]] = {t: [] for t in topics_of_interest}

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except Exception as e:
        raise RuntimeError(f"rosbag2_py stack not available: {e}")

    if not mcap_path.exists():
        raise FileNotFoundError(f"MCAP not found: {mcap_path}")

    storage_options = rosbag2_py.StorageOptions(uri=str(mcap_path), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Map topic -> ROS message type string
    topic_types: Dict[str, str] = {}
    for t in reader.get_all_topics_and_types():
        topic_types[t.name] = t.type

    missing = [t for t in topics_of_interest if t not in topic_types]
    if missing:
        warnings.append(f"topics missing from bag: {missing}")

    # Pre-build message classes for topics we can parse
    msg_classes: Dict[str, Any] = {}
    for t in topics_of_interest:
        if t in topic_types:
            try:
                msg_classes[t] = get_message(topic_types[t])
            except Exception as e:
                warnings.append(f"cannot resolve msg type for {t} ({topic_types[t]}): {e}")

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in samples:
            continue
        if topic not in msg_classes:
            continue

        try:
            msg = deserialize_message(data, msg_classes[topic])
        except Exception as e:
            warnings.append(f"deserialize failed for {topic}: {e}")
            continue

        t = float(t_ns) * 1e-9
        samples[topic].append(BagSample(t=t, msg=msg))

    # Sort (should already be ordered, but enforce)
    for t in topics_of_interest:
        samples[t].sort(key=lambda s: s.t)

    return samples, warnings


# -----------------------------
# Metric computations
# -----------------------------

def compute_phase_durations(state_samples: List[BagSample]) -> Dict[str, float]:
    """
    From std_msgs/String mission state.
    Computes total time spent in each phase.
    """
    durations: Dict[str, float] = {}
    if len(state_samples) < 2:
        return durations

    # state at time i applies until next sample
    for i in range(len(state_samples) - 1):
        s0 = str(getattr(state_samples[i].msg, "data", "")).strip()
        t0 = state_samples[i].t
        t1 = state_samples[i + 1].t
        dt = max(0.0, t1 - t0)
        if not s0:
            s0 = "UNKNOWN"
        durations[s0] = durations.get(s0, 0.0) + dt

    return durations


def integrate_cmd_distance(cmd_samples: List[BagSample]) -> float:
    """
    Integrate commanded forward distance proxy: ∫ max(vx,0) dt
    Uses geometry_msgs/Twist linear.x
    """
    if len(cmd_samples) < 2:
        return 0.0

    dist = 0.0
    for i in range(len(cmd_samples) - 1):
        v = float(cmd_samples[i].msg.linear.x)
        v = max(0.0, v)  # forward-only proxy; reverse is a different story
        dt = max(0.0, cmd_samples[i + 1].t - cmd_samples[i].t)
        dist += v * dt
    return dist


def odom_path_length(odom_samples: List[BagSample]) -> float:
    """
    Compute achieved distance proxy from odometry pose positions.
    """
    if len(odom_samples) < 2:
        return 0.0

    def xy(sample: BagSample) -> Tuple[float, float]:
        p = sample.msg.pose.pose.position
        return float(p.x), float(p.y)

    length = 0.0
    x0, y0 = xy(odom_samples[0])
    for s in odom_samples[1:]:
        x1, y1 = xy(s)
        dx, dy = x1 - x0, y1 - y0
        length += math.hypot(dx, dy)
        x0, y0 = x1, y1
    return length


def stop_measure_drift(odom_samples: List[BagSample], state_samples: List[BagSample]) -> Optional[float]:
    """
    Max displacement during STOP_MEASURE windows, based on odom positions.
    Returns None if insufficient data.
    """
    if len(odom_samples) < 2 or len(state_samples) < 2:
        return None

    # Repeated STOP_MEASURE publications describe one continuous interval.
    intervals = []
    start = None
    for sample in state_samples:
        phase = str(getattr(sample.msg, "data", "")).strip()
        if phase == "STOP_MEASURE" and start is None:
            start = sample.t
        elif phase != "STOP_MEASURE" and start is not None:
            intervals.append((start, sample.t))
            start = None
    if start is not None or not intervals:
        return None  # An unclosed stop does not establish full interval coverage.

    def pos(sample):
        p = sample.msg.pose.pose.position
        return float(p.x), float(p.y)

    def interpolate(t):
        # Linear interpolation is only between recorded samples, never extrapolation.
        for left, right in zip(odom_samples, odom_samples[1:]):
            if left.t <= t <= right.t and right.t > left.t:
                alpha = (t - left.t) / (right.t - left.t)
                x0, y0 = pos(left)
                x1, y1 = pos(right)
                return x0 + alpha * (x1 - x0), y0 + alpha * (y1 - y0)
        return None

    max_drift = 0.0
    for t0, t1 in intervals:
        window = [sample for sample in odom_samples if t0 <= sample.t <= t1]
        origin, end = interpolate(t0), interpolate(t1)
        if t1 <= t0 or len(window) < 2 or origin is None or end is None:
            return None
        points = [pos(sample) for sample in window] + [end]
        if not all(math.isfinite(v) for point in [origin] + points for v in point):
            return None
        max_drift = max(max_drift, max(math.hypot(x - origin[0], y - origin[1]) for x, y in points))
    return max_drift


def acceptance_failures(metrics, scenario):
    """Compare complete evidence with explicit bounds; never invent defaults.

    scenario['thresholds'] maps metric paths to {'min': number, 'max': number}
    (at least one bound). Missing thresholds are an acceptance configuration error.
    """
    failures = []
    if metrics.get("status") != "ok":
        failures.append("evidence status is not ok")
    for name in ("has_cmd_vel_safe", "has_odom_filtered", "has_mission_state"):
        if metrics.get("quality", {}).get(name) is not True:
            failures.append(f"missing required coverage: {name}")
    for topic in (TOPIC_CMD, TOPIC_ODOM, TOPIC_STATE):
        count = metrics.get("topics", {}).get(topic, {}).get("count")
        if type(count) is not int or count < 2:
            failures.append(f"insufficient topic samples: {topic}")
    required = ("time.duration_s", "mission.stop_measure_max_drift_m",
                "mobility.commanded_distance_m_proxy", "mobility.achieved_distance_m_proxy", "mobility.slip_proxy")
    def lookup(path):
        value = metrics
        for part in path.split("."):
            if not isinstance(value, dict) or part not in value:
                return None
            value = value[part]
        return value
    def finite(value):
        return type(value) in (int, float) and math.isfinite(value)
    for name in required:
        if not finite(lookup(name)):
            failures.append(f"missing/nonfinite metric: {name}")
    if finite(lookup("time.duration_s")) and lookup("time.duration_s") <= 0:
        failures.append("run duration must be positive")
    thresholds = scenario.get("thresholds") if isinstance(scenario, dict) else None
    if not isinstance(thresholds, dict) or not thresholds:
        return failures + ["scenario has no explicit quantitative thresholds; acceptance is unconfigured"]
    for path, bounds in thresholds.items():
        if not isinstance(path, str) or not isinstance(bounds, dict) or not bounds or set(bounds) - {"min", "max"}:
            failures.append(f"invalid threshold configuration: {path}")
            continue
        if not all(finite(value) for value in bounds.values()):
            failures.append(f"nonfinite/nonnumeric threshold: {path}")
            continue
        if bounds.get("min", -math.inf) > bounds.get("max", math.inf):
            failures.append(f"reversed bounds: {path}")
            continue
        value = lookup(path)
        if not finite(value):
            failures.append(f"missing/nonfinite metric: {path}")
        elif value < bounds.get("min", -math.inf) or value > bounds.get("max", math.inf):
            failures.append(f"threshold failed: {path}={value}, bounds={bounds}")
    return failures


# -----------------------------
# Main
# -----------------------------

def evaluate(dataset_dir: Path, *, out_path: Optional[Path] = None, strict: bool = True) -> Path:
    run_meta_path = dataset_dir / "run_metadata.json"
    mcap_path = dataset_dir / "run.mcap"

    if strict:
        if not dataset_dir.exists():
            raise FileNotFoundError(f"dataset dir not found: {dataset_dir}")
        if not run_meta_path.exists():
            raise FileNotFoundError(f"missing run_metadata.json: {run_meta_path}")
        if not mcap_path.exists():
            raise FileNotFoundError(f"missing run.mcap: {mcap_path}")

    meta = read_json(run_meta_path) if run_meta_path.exists() else {}
    if strict:
        from .schema_validate import validate_minimal
        valid, reasons = validate_minimal(meta)
        if not valid:
            raise ValueError("Invalid metadata: " + "; ".join(reasons))
    run_id = str(meta.get("run_id", dataset_dir.name))

    metrics: Dict[str, Any] = {
        "schema_version": "0.1",
        "run_id": run_id,
        "status": "ok",
        "notes": [],
        "topics": {},
        "time": {},
        "mission": {},
        "mobility": {},
        "quality": {},
    }

    # Attempt to parse bag
    ok_rosbag, why = try_import_rosbag()
    if not ok_rosbag:
        metrics["status"] = "partial"
        metrics["notes"].append(
            "rosbag2_py not available; cannot compute evidence-based metrics from MCAP. "
            f"Install rosbag2_py stack. Import error: {why}"
        )
        # Still write a metrics file (honest partial)
        out = out_path or (dataset_dir / "metrics.json")
        write_json(out, metrics)
        return out

    samples, warnings = load_mcap_samples(mcap_path, [TOPIC_CMD, TOPIC_ODOM, TOPIC_STATE])
    for w in warnings:
        metrics["notes"].append(w)
    if warnings:
        metrics["status"] = "partial"
    # Invalid samples are unusable evidence, not zero motion or benign slip.
    for topic, values in samples.items():
        last = -math.inf
        for sample in values:
            fields = [sample.t]
            if topic == TOPIC_CMD:
                fields += [float(sample.msg.linear.x), float(sample.msg.angular.z)]
            elif topic == TOPIC_ODOM:
                fields += [float(sample.msg.pose.pose.position.x), float(sample.msg.pose.pose.position.y)]
            if not all(math.isfinite(value) for value in fields) or sample.t < last:
                raise ValueError(f"Nonfinite or decreasing samples: {topic}")
            last = sample.t
        if len(values) < 2:
            metrics["status"] = "partial"
            metrics["notes"].append(f"insufficient samples: {topic}")

    # Message counts
    for t, lst in samples.items():
        metrics["topics"][t] = {"count": len(lst)}

    # Time coverage (use all samples)
    all_times: List[float] = []
    for lst in samples.values():
        all_times += [s.t for s in lst]
    all_times.sort()

    if len(all_times) >= 2:
        metrics["time"]["start_s"] = all_times[0]
        metrics["time"]["end_s"] = all_times[-1]
        metrics["time"]["duration_s"] = max(0.0, all_times[-1] - all_times[0])
    else:
        metrics["status"] = "partial"
        metrics["notes"].append("insufficient timestamps to compute run duration")

    # Mission phase durations
    phase_durs = compute_phase_durations(samples[TOPIC_STATE])
    metrics["mission"]["phase_durations_s"] = phase_durs

    # Mobility proxies
    cmd_dist = integrate_cmd_distance(samples[TOPIC_CMD])
    odom_dist = odom_path_length(samples[TOPIC_ODOM])
    metrics["mobility"]["commanded_distance_m_proxy"] = cmd_dist
    metrics["mobility"]["achieved_distance_m_proxy"] = odom_dist

    slip_proxy = None
    if cmd_dist > 1e-6:
        slip_proxy = clamp(1.0 - (odom_dist / cmd_dist), 0.0, 1.0)
        metrics["mobility"]["slip_proxy"] = slip_proxy
    else:
        metrics["status"] = "partial"
        metrics["notes"].append("commanded distance proxy is ~0; slip proxy undefined")

    drift = stop_measure_drift(samples[TOPIC_ODOM], samples[TOPIC_STATE])
    if drift is not None:
        metrics["mission"]["stop_measure_max_drift_m"] = drift
    else:
        metrics["status"] = "partial"
        metrics["notes"].append("cannot compute stop_measure drift (missing states or odom windows)")

    # Basic data quality flags
    metrics["quality"]["has_cmd_vel_safe"] = len(samples[TOPIC_CMD]) > 0
    metrics["quality"]["has_odom_filtered"] = len(samples[TOPIC_ODOM]) > 0
    metrics["quality"]["has_mission_state"] = len(samples[TOPIC_STATE]) > 0

    if strict:
        if not metrics["quality"]["has_cmd_vel_safe"]:
            metrics["status"] = "partial"
            metrics["notes"].append(f"missing required topic samples: {TOPIC_CMD}")
        if not metrics["quality"]["has_odom_filtered"]:
            metrics["status"] = "partial"
            metrics["notes"].append(f"missing required topic samples: {TOPIC_ODOM}")
        if not metrics["quality"]["has_mission_state"]:
            metrics["status"] = "partial"
            metrics["notes"].append(f"missing required topic samples: {TOPIC_STATE}")

    out = out_path or (dataset_dir / "metrics.json")
    write_json(out, metrics)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Evaluate Phase-0 metrics from a packaged dataset.")
    ap.add_argument("dataset_dir", type=str, help="Path to datasets/<run_id>/")
    ap.add_argument("--out", type=str, default="", help="Optional output path for metrics.json")
    ap.add_argument("--gate", action="store_true", help="Fail on partial evidence or failed/missing thresholds")
    ap.add_argument("--thresholds", type=Path, help="Scenario YAML containing explicit thresholds")
    ap.add_argument("--non-strict", action="store_true", help="Do not hard-fail on missing artifacts")

    args = ap.parse_args()
    if args.gate and (args.non_strict or args.thresholds is None):
        ap.error("--gate requires --thresholds and strict artifact validation")
    dataset_dir = Path(args.dataset_dir).resolve()
    out_path = Path(args.out).resolve() if args.out else None
    strict = not args.non_strict

    try:
        out = evaluate(dataset_dir, out_path=out_path, strict=strict)
        if args.gate:
            import yaml
            scenario = yaml.safe_load(args.thresholds.read_text())
            metrics = read_json(out)
            if not isinstance(scenario, dict) or scenario.get("name") != read_json(dataset_dir / "run_metadata.json").get("scenario"):
                raise ValueError("Threshold scenario name must match dataset metadata")
            failures = acceptance_failures(metrics, scenario)
            metrics["acceptance"] = {"passed": not failures, "failures": failures}
            write_json(out, metrics)
            if failures:
                print("[evaluate_metrics] acceptance failed: " + "; ".join(failures), file=sys.stderr)
                raise SystemExit(1)
    except Exception as e:
        print(f"[evaluate_metrics] ERROR: {e}", file=sys.stderr)
        raise SystemExit(2)

    print(str(out))


if __name__ == "__main__":
    main()
