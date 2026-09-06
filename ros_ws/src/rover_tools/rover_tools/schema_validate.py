"""Canonical Draft 2020-12 validation, including RFC3339 timestamps.

The datasets/schemas path links to this installed resource: one schema in both
source checkouts and wheel installations. validate_minimal keeps its API name.
"""
import json
from importlib.resources import files
from pathlib import Path
from jsonschema import Draft202012Validator, FormatChecker


def validate_minimal(meta):
    try:
        json.dumps(meta, allow_nan=False)
    except (ValueError, TypeError) as error:
        return False, [f"Metadata must be finite JSON: {error}"]
    schema = json.loads(files("rover_tools").joinpath("run_metadata.schema.json").read_text())
    Draft202012Validator.check_schema(schema)
    validator = Draft202012Validator(schema, format_checker=FormatChecker())
    reasons = [f"{'/'.join(map(str, error.absolute_path)) or '/'}: {error.message}"
               for error in sorted(validator.iter_errors(meta), key=lambda e: str(list(e.absolute_path)))]
    return not reasons, reasons


def main_validate(metadata_path: Path):
    return validate_minimal(json.loads(metadata_path.read_text(encoding="utf-8")))
