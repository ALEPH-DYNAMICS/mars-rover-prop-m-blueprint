"""Compatibility wrapper for the installed evaluator."""
from pathlib import Path
from .metrics import evaluate

def evaluate_metrics(dataset_dir: Path, *, strict: bool = True) -> Path:
    return evaluate(Path(dataset_dir), strict=strict)
