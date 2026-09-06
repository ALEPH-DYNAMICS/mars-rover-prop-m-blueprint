#!/usr/bin/env python3
"""Gate exactly the datasets named by a successful batch report."""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("--thresholds", type=Path, required=True)
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    runs = report.get("runs", [])
    if not runs or len(runs) != report.get("runs_requested") or any(run.get("status") != "ok" for run in runs):
        parser.exit(1, "Batch is missing runs or contains failed/partial evidence\n")
    for run in runs:
        if not run.get("dataset_dir"):
            parser.exit(1, "Batch run has no dataset directory\n")
        result = subprocess.run([sys.executable, str(Path(__file__).with_name("evaluate_metrics.py")),
                                 run["dataset_dir"], "--gate", "--thresholds", str(args.thresholds)])
        if result.returncode:
            raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
