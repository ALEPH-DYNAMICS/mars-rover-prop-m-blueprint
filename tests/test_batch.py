"""Synthetic orchestration tests; no bags or simulated motion are fabricated."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import run_sim_batch as batch
from rover_tools.schema_validate import validate_minimal


class BatchTests(unittest.TestCase):
    def config(self, mode="modern"):
        return batch.RunConfig("mars_flat", mode, "gazebo", 1, 73, False, True, ROOT / batch.DEFAULT_TOPICS_CFG)

    def test_seed_and_mode_are_forwarded_to_the_actual_launch(self):
        with patch.object(batch, "run") as run:
            batch.launch_gazebo_world(ROOT, "mars_flat.sdf", self.config("prop_m"), 73)
            args = run.call_args.args[0]
            self.assertIn("seed:=73", args)
            self.assertIn("mode:=prop_m", args)
            self.assertIn("sim_bringup.launch.py", args)

    def test_generated_configuration_metadata_satisfies_schema(self):
        for mode in ("modern", "prop_m"):
            meta = batch.build_run_metadata(ROOT, self.config(mode), batch.load_scenario(ROOT, "mars_flat"), ["/synthetic-unit-test"])
            meta["run_id"] = "fixture1"
            self.assertEqual(validate_minimal(meta), (True, []))
            self.assertIsNone(meta["physics"]["deterministic"])
            for field in ("control_config", "estimator_config", "terramechanics_config"):
                self.assertTrue((ROOT / meta["params"][field]).is_file())

    def test_workflow_arguments_parse_and_failed_run_exits_nonzero(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = Path(tmp) / "batch.json"
            args = ["run_sim_batch", "--scenario", "mars_flat", "--backend", "gazebo", "--seed", "42", "--duration", "45", "--runs", "1", "--no-nav2", "--report", str(report)]
            with patch.object(sys, "argv", args), patch.object(batch, "launch_gazebo_world", side_effect=RuntimeError("synthetic launch failure")):
                with self.assertRaises(SystemExit) as result: batch.main()
            self.assertEqual(result.exception.code, 1)
            self.assertEqual(json.loads(report.read_text())["runs"][0]["status"], "error")


if __name__ == "__main__": unittest.main()
