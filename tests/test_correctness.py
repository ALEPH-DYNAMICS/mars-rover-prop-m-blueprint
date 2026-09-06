"""Synthetic unit fixtures only: these tests are not rover runtime evidence."""
import copy
import hashlib
import importlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
for package in ("rover_tools", "rover_control"):
    sys.path.insert(0, str(ROOT / "ros_ws/src" / package))
from rover_tools import metrics
from rover_tools.schema_validate import validate_minimal
from rover_tools.dataset_package import package_dataset
from rover_control.shaping import CommandShaper, Limits, ShapingConfig, clamp, validate_mode


def state(t, phase):
    return metrics.BagSample(t, NS(data=phase))


def odom(t, x):
    return metrics.BagSample(t, NS(pose=NS(pose=NS(position=NS(x=x, y=0.0)))))


def metadata():
    return {
        "schema_version": "0.1", "run_id": "fixture1", "created_utc": "2026-09-06T00:00:00Z",
        "mode": "simulation", "backend": "gazebo", "scenario": "unit_fixture", "seed": 0,
        "git": {"commit": "1234567", "dirty": False},
        "clock": {"use_sim_time": True, "time_source": "sim_clock"},
        "physics": {"engine": "fixture", "time_step_s": 0.01, "real_time_factor": 1.0},
        "robot": {"name": "fixture", "urdf_sha256": "0" * 64, "base_frame": "base_link"},
        "params": {"control_config": "fixture", "estimator_config": "fixture", "terramechanics_config": "fixture", "nav2_profile": None},
        "topics": {"recorded": ["/fixture"]}, "artifacts": {"mcap": "run.mcap", "metrics_json": None},
    }


def complete_metrics():
    return {
        "status": "ok", "topics": {topic: {"count": 3} for topic in (metrics.TOPIC_CMD, metrics.TOPIC_ODOM, metrics.TOPIC_STATE)},
        "quality": {"has_cmd_vel_safe": True, "has_odom_filtered": True, "has_mission_state": True},
        "time": {"duration_s": 2.0}, "mission": {"stop_measure_max_drift_m": 0.05},
        "mobility": {"commanded_distance_m_proxy": 1.0, "achieved_distance_m_proxy": 0.9, "slip_proxy": 0.1},
    }


class MetricsTests(unittest.TestCase):
    def test_out_and_back_peak(self):
        self.assertEqual(metrics.stop_measure_drift([odom(0, 0), odom(1, 1), odom(2, 0)],
                         [state(0, "STOP_MEASURE"), state(2, "DRIVE")]), 1.0)

    def test_repeated_states_do_not_reset_origin(self):
        self.assertEqual(metrics.stop_measure_drift([odom(0, 0), odom(1, 1), odom(2, 2)],
                         [state(0, "STOP_MEASURE"), state(1, "STOP_MEASURE"), state(2, "DRIVE")]), 2.0)

    def test_empty_or_uncovered_or_unclosed_window(self):
        for odometry, states in [
            ([odom(4, 0), odom(5, 1)], [state(0, "STOP_MEASURE"), state(2, "DRIVE")]),
            ([odom(.5, 0), odom(1.5, 1)], [state(0, "STOP_MEASURE"), state(2, "DRIVE")]),
            ([odom(0, 0), odom(1, 0)], [state(0, "STOP_MEASURE"), state(1, "STOP_MEASURE")]),
        ]:
            self.assertIsNone(metrics.stop_measure_drift(odometry, states))

    def test_interval_boundary_interpolation_and_nonfinite(self):
        states = [state(.5, "STOP_MEASURE"), state(2.5, "DRIVE")]
        self.assertEqual(metrics.stop_measure_drift([odom(0, 0), odom(1, 1), odom(2, 2), odom(3, 3)], states), 2)
        self.assertIsNone(metrics.stop_measure_drift([odom(0, 0), odom(1, math.nan), odom(2, 0)],
                          [state(0, "STOP_MEASURE"), state(2, "DRIVE")]))

    def test_gate_accepts_boundaries_and_rejects_failures(self):
        scenario = {"thresholds": {"mission.stop_measure_max_drift_m": {"max": .05}, "time.duration_s": {"min": 2}}}
        self.assertEqual(metrics.acceptance_failures(complete_metrics(), scenario), [])
        for key, value in [("status", "partial"), ("quality", {}), ("topics", {}), ("mobility", {}), ("mission", {"stop_measure_max_drift_m": .051})]:
            m = complete_metrics(); m[key] = value
            self.assertTrue(metrics.acceptance_failures(m, scenario))

    def test_gate_rejects_missing_bad_or_unknown_thresholds(self):
        for config in [{}, {"thresholds": {}}, {"thresholds": {"no.such.metric": {"max": 1}}},
                       {"thresholds": {"time.duration_s": {"min": math.nan}}},
                       {"thresholds": {"time.duration_s": {"max": True}}},
                       {"thresholds": {"time.duration_s": {"min": 5, "max": 1}}}]:
            self.assertTrue(metrics.acceptance_failures(complete_metrics(), config))

    def test_reporting_partial_is_distinct_from_cli_gate(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp); (p / "run_metadata.json").write_text(json.dumps(metadata()))
            (p / "run.mcap").write_bytes(b"SYNTHETIC, not a recorded bag")
            scenario = p / "scenario.yaml"; scenario.write_text("name: unit_fixture\nthresholds:\n  time.duration_s: {min: 1}\n")
            with patch.object(metrics, "try_import_rosbag", return_value=(False, "unit fixture")):
                out = metrics.evaluate(p)
                self.assertEqual(json.loads(out.read_text())["status"], "partial")
                with patch.object(sys, "argv", ["rover_metrics", str(p), "--gate", "--thresholds", str(scenario)]):
                    with self.assertRaises(SystemExit) as exit:
                        metrics.main()
                    self.assertEqual(exit.exception.code, 1)

    def test_phase_transition_publications_can_share_a_clock_stamp(self):
        samples = {
            metrics.TOPIC_CMD: [metrics.BagSample(t, NS(linear=NS(x=1.), angular=NS(z=0.))) for t in (0, 3)],
            metrics.TOPIC_ODOM: [odom(0, 0), odom(1, 1), odom(2, 1), odom(3, 2)],
            metrics.TOPIC_STATE: [state(0, "DRIVE"), state(1, "DRIVE"), state(1, "STOP_MEASURE"),
                                  state(2, "STOP_MEASURE"), state(2, "DRIVE"), state(3, "DONE")],
        }
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp); (p / "run_metadata.json").write_text(json.dumps(metadata()))
            (p / "run.mcap").write_bytes(b"SYNTHETIC; parsing replaced by unit samples")
            with patch.object(metrics, "try_import_rosbag", return_value=(True, "")), patch.object(metrics, "load_mcap_samples", return_value=(samples, [])):
                result = json.loads(metrics.evaluate(p).read_text())
            self.assertEqual(result["status"], "ok")
            self.assertEqual(result["mission"]["stop_measure_max_drift_m"], 0)


class DatasetTests(unittest.TestCase):
    def test_schema_rejects_invalid_nested_fields_and_nonfinite(self):
        self.assertEqual(validate_minimal(metadata()), (True, []))
        for key, value in [("git", {}), ("clock", {}), ("physics", {}), ("robot", {}), ("params", {}),
                           ("topics", {}), ("topics", {"recorded": []}), ("seed", True),
                           ("created_utc", "not-a-date"), ("seed", -1),
                           ("physics", {"engine": "x", "time_step_s": math.inf, "real_time_factor": 1})]:
            m = metadata(); m[key] = value
            self.assertFalse(validate_minimal(m)[0], (key, value))

    def test_atomic_package_and_independent_hash_readback(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp); bag = p / "test.mcap"; bag.write_bytes(b"SYNTHETIC packaging fixture")
            original = metadata()
            out = package_dataset(p, "fixture1", bag, original)
            saved = json.loads((out / "run_metadata.json").read_text())
            digest = saved["integrity"].pop("metadata_sha256")
            payload = json.dumps(saved, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False).encode("utf-8")
            self.assertEqual(digest, hashlib.sha256(payload).hexdigest())
            self.assertEqual(saved["integrity"]["mcap_sha256"], hashlib.sha256(bag.read_bytes()).hexdigest())
            self.assertNotIn("integrity", original)
            with self.assertRaises(FileExistsError): package_dataset(p, "fixture1", bag, original)

    def test_failed_packaging_creates_no_dataset(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp); bag = p / "test.mcap"; bag.write_bytes(b"SYNTHETIC")
            invalid = metadata(); invalid["seed"] = True
            with self.assertRaises(ValueError): package_dataset(p, "fixture1", bag, invalid)
            self.assertFalse((p / "datasets/fixture1").exists())
            with patch("rover_tools.dataset_package.shutil.copy2", side_effect=OSError("injected copy failure")):
                with self.assertRaises(OSError): package_dataset(p, "fixture1", bag, metadata())
            self.assertEqual(list((p / "datasets").iterdir()), [])
            with self.assertRaises(ValueError): package_dataset(p, "../escape", bag, metadata())

    def test_schema_path_is_the_installed_canonical_resource(self):
        from importlib.resources import files
        self.assertEqual(json.loads((ROOT / "datasets/schemas/run_metadata.schema.json").read_text()),
                         json.loads(files("rover_tools").joinpath("run_metadata.schema.json").read_text()))


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.time = 0.0
        self.shaper = CommandShaper(lambda: self.time)
        self.cfg = ShapingConfig(Limits(.2, .6, .15, .8), slip_enabled=False)

    def test_nan_clamp_and_invalid_configuration(self):
        with self.assertRaises(ValueError): clamp(math.nan, -.2, .2)
        for v in (math.nan, math.inf, -1, 0, True):
            with self.assertRaises(ValueError): Limits(v, .6, .15, .8)
        for kw in ({"timeout": math.nan}, {"slip_gamma": 2}, {"slip_threshold": .6}, {"slip_timeout": 0}):
            with self.assertRaises(ValueError): ShapingConfig(self.cfg.limits, **kw)
        with self.assertRaises(ValueError): validate_mode("typo")

    def test_ramp_and_watchdog_with_injected_clock(self):
        self.shaper.command_received(100, 100)
        self.time = .1
        self.assertEqual(self.shaper.step(self.cfg), (.015, .08000000000000002))
        self.time = 1
        self.assertEqual(self.shaper.step(self.cfg), (0, 0))

    def test_invalid_command_inhibits_and_cannot_accelerate(self):
        self.shaper.command_received(.2, .2); self.time = .1; self.shaper.step(self.cfg)
        self.assertFalse(self.shaper.command_received(math.nan, .1))
        self.time = .2; self.assertEqual(self.shaper.step(self.cfg), (0, 0))

    def test_stale_missing_invalid_and_hard_slip_ramp_to_zero(self):
        cfg = ShapingConfig(self.cfg.limits, slip_timeout=.1)
        self.shaper.command_received(.2, .2); self.time = .05
        self.assertEqual(self.shaper.step(cfg), (0, 0))
        self.shaper.slip_received(0); self.time = .1; self.assertGreater(self.shaper.step(cfg)[0], 0)
        self.time = .2; self.assertEqual(self.shaper.step(cfg), (0, 0))
        for slip in (math.nan, math.inf, True, "0.1"): self.assertFalse(self.shaper.slip_received(slip))
        self.shaper.slip_received(.6); self.time = .3; self.assertEqual(self.shaper.step(cfg), (0, 0))

    def test_signed_slip_preserves_the_documented_ratio_contract(self):
        cfg = ShapingConfig(self.cfg.limits)
        self.shaper.command_received(.2, .2)
        self.assertTrue(self.shaper.slip_received(-.1))  # Documented braking slip.
        self.time = .1
        self.assertGreater(self.shaper.step(cfg)[0], 0)
        self.assertTrue(self.shaper.slip_received(1.1))
        self.time = .2
        self.assertEqual(self.shaper.step(cfg), (0, 0))  # Above hard threshold.

    def test_pause_rate_and_clock_reset(self):
        self.shaper.command_received(.2, .2)
        self.time = .1; output = self.shaper.step(self.cfg)
        for _ in range(10): self.assertEqual(self.shaper.step(self.cfg), output)
        self.time = -.1; self.assertEqual(self.shaper.step(self.cfg), (0, 0))
        self.assertIsNone(self.shaper.command_time)
        # Equal elapsed simulated time, independent of how fast the caller runs.
        for step in (.01, .02):
            self.time = 0; s = CommandShaper(lambda: self.time); s.command_received(.2, .2)
            for i in range(1, round(.2 / step) + 1): self.time = i * step; s.step(self.cfg)
            self.assertAlmostEqual(s.output[0], .03)

    def test_both_mode_profiles_supply_the_consumed_parameter_names(self):
        def flatten(obj, prefix=""):
            result = {}
            for key, value in obj.items():
                path = prefix + key
                if isinstance(value, dict): result.update(flatten(value, path + "."))
                else: result[path] = value
            return result
        for mode, speed, turn in [("modern", .2, .6), ("prop_m", .05, .3)]:
            values = flatten(yaml.safe_load((ROOT / f"ros_ws/src/rover_bringup/params/modes/{mode}.yaml").read_text())["/**"]["ros__parameters"])
            self.assertEqual(validate_mode(values["mode"]), mode)
            self.assertEqual(values[f"limits.v_max_mps.{mode}"], speed)
            self.assertEqual(values[f"limits.omega_max_rps.{mode}"], turn)


if __name__ == "__main__":
    unittest.main()
