#!/usr/bin/env python3
"""
scripts/run_sim_batch.py

Batch runner for simulation scenarios.

This script launches best-effort simulation runs for a given scenario and mode,
records bags, packages datasets, and optionally evaluates metrics.

It is intentionally strict:
- Every run has a run_id.
- Every run produces evidence under /datasets/<run_id>/.
- Failures are recorded with diagnostics; no silent skipping.

Expected repo layout:
  mars-rover-prop-m-blueprint/
    ros_ws/
    datasets/
    datasets/schemas/run_metadata.schema.json
    scenarios/<scenario_name>/scenario.yaml

Requires:
- ROS 2 environment sourced
- rover_sim_gazebo installed (for Gazebo backend)
- rover_tools installed OR scripts/package_dataset.py equivalent (we package here directly)

NOTE:
This runner does not assume your full bringup.launch exists yet.
It launches the common simulation bringup and optional Nav2; the bringup controls phase-label publication.
As the program matures, replace these launch calls with a single bringup.

Phase 0 default:
- start Gazebo world
- (optional) start Nav2
- (optional) start mission bt
- record MCAP for N seconds
- stop everything
- package dataset
- evaluate metrics (optional)

"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
import tempfile
import math
import hashlib
import xml.etree.ElementTree as ET
import yaml
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros_ws/src/rover_tools"))
from rover_tools.dataset_package import package_dataset as validated_package_dataset

# Repo-relative defaults
DEFAULT_TOPICS_CFG = "ros_ws/src/rover_tools/config/record_topics_minimal.yaml"

# Conservative topic list fallback if YAML parsing fails
FALLBACK_TOPICS = [
    "/tf",
    "/tf_static",
    "/joint_states",
    "/imu/data",
    "/cmd_vel",
    "/cmd_vel_safe",
    "/odometry/filtered",
    "/mission/state",
]

# Where outputs go
DATASETS_DIR = "datasets"


# -----------------------------
# Small helpers (no dependencies)
# -----------------------------

def now_utc_compact() -> str:
    # ISO-ish compact: YYYYMMDD_HHMMSS
    return time.strftime("%Y%m%d_%H%M%S", time.gmtime())


def run(cmd: List[str], *, env: Optional[Dict[str, str]] = None, cwd: Optional[Path] = None) -> subprocess.Popen:
    # File-backed output cannot fill a pipe while the simulation is running.
    log = tempfile.TemporaryFile(mode="w+")
    try:
        process = subprocess.Popen(cmd, cwd=str(cwd) if cwd else None, env=env,
                                   stdout=log, stderr=subprocess.STDOUT, text=True,
                                   start_new_session=True)
    except Exception:
        log.close()
        raise
    process.run_log = log
    return process


def kill_proc_group(p: subprocess.Popen, timeout_s: float = 10.0) -> None:
    if p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
    except Exception:
        pass

    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if p.poll() is not None:
            return
        time.sleep(0.1)

    try:
        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
    except Exception:
        pass


def drain_output(p: subprocess.Popen, max_lines: int = 200) -> List[str]:
    log = getattr(p, "run_log", None)
    if log is None:
        return []
    log.seek(0)
    lines = log.read().splitlines()[-max_lines:]
    log.close()
    return lines


def read_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, obj: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def read_yaml_topics_minimal(path: Path) -> List[str]:
    """
    Minimal YAML parser for:
      topics:
        - /a
        - /b
    """
    try:
        txt = path.read_text(encoding="utf-8")
    except Exception:
        return []
    topics: List[str] = []
    in_topics = False
    for raw in txt.splitlines():
        ln = raw.strip()
        if not ln or ln.startswith("#"):
            continue
        if ln.startswith("topics:"):
            in_topics = True
            continue
        if in_topics and ln.startswith("-"):
            topics.append(ln.split("-", 1)[1].strip())
    return topics


def git_short_hash(repo_root: Path) -> str:
    try:
        out = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=str(repo_root), text=True).strip()
        return out
    except Exception:
        return "nogit"


def git_dirty(repo_root: Path) -> bool:
    try:
        out = subprocess.check_output(["git", "status", "--porcelain"], cwd=str(repo_root), text=True)
        return len(out.strip()) > 0
    except Exception:
        return False


# -----------------------------
# Scenario loading
# -----------------------------

def load_scenario(repo_root: Path, scenario_name: str) -> Dict[str, Any]:
    scenario_path = repo_root / "scenarios" / scenario_name / "scenario.yaml"
    if not scenario_path.exists():
        raise FileNotFoundError(f"scenario.yaml not found: {scenario_path}")

    data = yaml.safe_load(scenario_path.read_text())
    return {"name": data["name"], "version": data.get("version", "unknown"),
            "world_gazebo": data["defaults"]["world_gazebo"], "scenario_path": str(scenario_path)}


# -----------------------------
# Bag recording
# -----------------------------

def record_bag(out_dir: Path, topics: List[str], duration_s: float) -> Path:
    """
    Record MCAP using ros2 bag record. Returns MCAP path.
    """
    if out_dir.exists():
        raise FileExistsError(out_dir)
    out_dir.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["ros2", "bag", "record", "--use-sim-time", "--storage", "mcap", "-o", str(out_dir)] + topics
    p = run(cmd)
    try:
        # This bounds recording in wall seconds, not deterministic simulation steps.
        time.sleep(duration_s)
        if p.poll() is not None:
            raise RuntimeError(f"bag recorder exited early ({p.returncode})")
    finally:
        kill_proc_group(p, timeout_s=10.0)
        logs = drain_output(p)
        (out_dir.parent / "recorder.log").write_text("\n".join(logs))
    mcaps = sorted(out_dir.glob("*.mcap"))
    if len(mcaps) != 1:
        raise RuntimeError(f"Expected one MCAP in {out_dir}; found {len(mcaps)} (split bags unsupported)")
    return mcaps[0]


# -----------------------------
# Dataset packaging (lightweight)
# -----------------------------

def package_dataset(repo_root: Path, run_id: str, mcap_path: Path, run_metadata: Dict[str, Any]) -> Path:
    return validated_package_dataset(repo_root, run_id, mcap_path, run_metadata)


# -----------------------------
# Launch orchestration
# -----------------------------

@dataclass
class RunConfig:
    scenario: str
    mode: str
    backend: str
    duration_s: float
    seed: int
    start_nav2: bool
    start_mission: bool
    topics_cfg: Path


def launch_gazebo_world(repo_root: Path, world_file: str, cfg: RunConfig, seed: int) -> subprocess.Popen:
    cmd = ["ros2", "launch", "rover_bringup", "sim_bringup.launch.py",
           f"world:={Path(world_file).name}", f"mode:={cfg.mode}", f"seed:={seed}",
           f"start_mission:={str(cfg.start_mission).lower()}", "gui:=false"]
    return run(cmd, cwd=repo_root)


def launch_nav2(repo_root: Path, mode: str) -> subprocess.Popen:
    cmd = ["ros2", "launch", "rover_navigation", "nav2.launch.py", f"mode:={mode}", "use_sim_time:=true"]
    return run(cmd, cwd=repo_root)


# -----------------------------
# Run metadata
# -----------------------------

def build_run_metadata(repo_root: Path, cfg: RunConfig, scenario_info: Dict[str, Any], topics: List[str]) -> Dict[str, Any]:
    world = repo_root / "ros_ws/src/rover_sim_gazebo/worlds" / Path(scenario_info["world_gazebo"]).name
    physics = ET.parse(world).getroot().find("world/physics")
    urdf = repo_root / "ros_ws/src/rover_description/urdf/rover.urdf.xacro"
    return {
        "schema_version": "0.1", "run_id": "",  # caller assigns the run ID
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "mode": "simulation", "backend": cfg.backend, "scenario": cfg.scenario, "seed": cfg.seed,
        "git": {"commit": git_short_hash(repo_root), "dirty": git_dirty(repo_root)},
        "clock": {"use_sim_time": True, "time_source": "sim_clock"},
        "physics": {"engine": physics.attrib["type"], "time_step_s": float(physics.findtext("max_step_size")),
                    "real_time_factor": float(physics.findtext("real_time_factor")), "deterministic": None},
        "robot": {"name": "rover", "base_frame": "base_link", "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest()},
        "params": {"control_config": f"ros_ws/src/rover_bringup/params/modes/{cfg.mode}.yaml",
                   "estimator_config": f"ros_ws/src/rover_estimation/config/ekf_{cfg.mode}.yaml",
                   "terramechanics_config": "ros_ws/src/rover_sim_gazebo/config/terrain_presets.yaml",
                   "nav2_profile": f"ros_ws/src/rover_navigation/params/nav2_{cfg.mode}.yaml" if cfg.start_nav2 else None},
        "topics": {"recorded": topics}, "artifacts": {"mcap": "run.mcap", "metrics_json": None},
        "notes": "Configuration provenance, not measured dynamics: physics values are read from the selected world XML; "
                 "urdf_sha256 hashes the root Xacro source, not its expanded model/includes. "
                 "Seed is forwarded to Gazebo gzserver; no stochastic RNG is used by the phase runner/control shaper. "
                 "Nav2 has no seed wiring here. Recording duration is wall time; determinism and motion are unverified.",
    }


# -----------------------------
# Main
# -----------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description="Batch runner for simulation scenarios with dataset packaging.")
    ap.add_argument("--scenario", required=True, help="Scenario name under scenarios/")
    ap.add_argument("--mode", default="modern", choices=["modern", "prop_m"], help="Mode profile")
    ap.add_argument("--backend", default="gazebo", choices=["gazebo"], help="Simulation backend (Phase 0: gazebo only)")
    ap.add_argument("--runs", type=int, default=3, help="Number of runs")
    ap.add_argument("--duration", type=float, default=30.0, help="Record duration per run (wall-clock seconds)")
    ap.add_argument("--seed", type=int, default=0, help="Base seed (incremented per run)")
    ap.add_argument("--topics-cfg", default=DEFAULT_TOPICS_CFG, help="YAML with topic list for recording")
    ap.add_argument("--no-nav2", action="store_true", help="Do not launch Nav2")
    ap.add_argument("--no-mission", action="store_true", help="Do not launch mission BT")
    ap.add_argument("--report", type=Path, help="Exact batch-report output path for downstream evaluation")
    ap.add_argument("--evaluate-metrics", action="store_true", help="Run scripts/evaluate_metrics.py after packaging")

    args = ap.parse_args()

    if args.runs < 1 or args.seed < 0 or not math.isfinite(args.duration) or args.duration <= 0:
        ap.error("runs must be positive, seed nonnegative, duration finite and positive")
    if Path(args.scenario).name != args.scenario or args.scenario in (".", ".."):
        ap.error("scenario must name a directory directly under scenarios")
    repo_root = Path(__file__).resolve().parents[1]
    cfg = RunConfig(
        scenario=args.scenario,
        mode=args.mode,
        backend=args.backend,
        duration_s=float(args.duration),
        seed=int(args.seed),
        start_nav2=not args.no_nav2,
        start_mission=not args.no_mission,
        topics_cfg=(repo_root / args.topics_cfg).resolve(),
    )

    scenario_info = load_scenario(repo_root, cfg.scenario)
    world_file = scenario_info.get("world_gazebo")
    if not world_file:
        print("[run_sim_batch] ERROR: scenario.yaml missing defaults.world_gazebo", file=sys.stderr)
        raise SystemExit(2)

    # Topics
    topics = read_yaml_topics_minimal(cfg.topics_cfg)
    if not topics:
        topics = FALLBACK_TOPICS

    batch_report: Dict[str, Any] = {
        "schema_version": "0.1",
        "scenario": cfg.scenario,
        "mode": cfg.mode,
        "backend": cfg.backend,
        "runs_requested": args.runs,
        "runs": [],
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }

    for i in range(int(args.runs)):
        run_seed = cfg.seed + i
        run_id = f"{now_utc_compact()}_{cfg.scenario}_{cfg.mode}_{git_short_hash(repo_root)}_{i:02d}"
        print(f"[run_sim_batch] RUN {i+1}/{args.runs} -> {run_id}")

        diagnostics: Dict[str, Any] = {
            "run_id": run_id,
            "status": "ok",
            "notes": [],
            "logs": {},
        }

        p_gz: Optional[subprocess.Popen] = None
        p_nav2: Optional[subprocess.Popen] = None
        p_mission: Optional[subprocess.Popen] = None

        try:
            # Launch simulator
            p_gz = launch_gazebo_world(repo_root, world_file, cfg, run_seed)
            time.sleep(5.0)  # allow gazebo to come up

            # Launch Nav2
            if cfg.start_nav2:
                p_nav2 = launch_nav2(repo_root, cfg.mode)
                time.sleep(3.0)

            for name, process in (("bringup", p_gz), ("nav2", p_nav2)):
                if process is not None and process.poll() is not None:
                    raise RuntimeError(f"{name} exited before recording ({process.returncode})")

            # Record bag
            tmp_bag_base = repo_root / "analysis" / "batch_tmp" / run_id / "bag"
            tmp_bag_base.parent.mkdir(parents=True, exist_ok=True)
            mcap = record_bag(tmp_bag_base, topics, cfg.duration_s)

            for name, process in (("bringup", p_gz), ("nav2", p_nav2)):
                if process is not None and process.poll() is not None:
                    raise RuntimeError(f"{name} exited during recording ({process.returncode})")

            # Package dataset
            bag_info = yaml.safe_load((mcap.parent / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
            recorded = [item["topic_metadata"]["name"] for item in bag_info["topics_with_message_count"] if item["message_count"] > 0]
            meta = build_run_metadata(repo_root, cfg, scenario_info, recorded)
            meta["run_id"] = run_id
            meta["seed"] = int(run_seed)

            dataset_dir = package_dataset(repo_root, run_id, mcap, meta)

            # Evaluate metrics (optional)
            if args.evaluate_metrics:
                eval_script = repo_root / "scripts" / "evaluate_metrics.py"
                if eval_script.exists():
                    p = subprocess.run(
                        [sys.executable, str(eval_script), str(dataset_dir)],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        text=True,
                        check=False,
                    )
                    diagnostics["logs"]["evaluate_metrics"] = p.stdout.splitlines()[-200:]
                    if p.returncode != 0:
                        diagnostics["status"] = "partial"
                        diagnostics["notes"].append("metrics evaluation failed (see logs)")

            diagnostics["dataset_dir"] = str(dataset_dir)

        except Exception as e:
            diagnostics["status"] = "error"
            diagnostics["notes"].append(str(e))

        finally:
            for name, process in (("mission", p_mission), ("nav2", p_nav2), ("gazebo", p_gz)):
                if process is not None:
                    kill_proc_group(process)
                    diagnostics["logs"][name + "_tail"] = drain_output(process)

        batch_report["runs"].append(diagnostics)

    # Write batch report
    out = repo_root / "analysis" / "batch_reports" / f"batch_{now_utc_compact()}_{cfg.scenario}_{cfg.mode}.json"
    out = args.report.resolve() if args.report else out
    write_json(out, batch_report)
    print(str(out))
    if any(run["status"] != "ok" for run in batch_report["runs"]):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
