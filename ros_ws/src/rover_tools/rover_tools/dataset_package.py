from __future__ import annotations
import copy
import hashlib
import json
import os
import re
import shutil
import tempfile
from pathlib import Path
from .utils import sha256_file, write_json
from .schema_validate import validate_minimal


def metadata_digest(meta):
    """Hash sorted compact ASCII-escaped UTF-8 JSON, excluding the hash itself.

    This is a canonical payload hash, never the hash of saved file bytes.
    """
    payload = copy.deepcopy(meta)
    payload.get("integrity", {}).pop("metadata_sha256", None)
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"),
                         ensure_ascii=True, allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def package_dataset(repo_root, run_id, bag_file, run_metadata, *, plots_dir=None):
    if not isinstance(run_id, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{7,}", run_id):
        raise ValueError("run_id must be a simple filename identifier of at least 8 characters")
    ok, reasons = validate_minimal(run_metadata)
    if not ok:
        raise ValueError("run_metadata.json failed validation: " + "; ".join(reasons))
    if run_metadata["run_id"] != run_id:
        raise ValueError("run_id must match metadata")
    if run_metadata["artifacts"]["mcap"] != "run.mcap":
        raise ValueError("Packaged MCAP artifact must be run.mcap")
    if not Path(bag_file).is_file():
        raise FileNotFoundError(bag_file)
    datasets = Path(repo_root) / "datasets"
    target = datasets / run_id
    if target.exists():
        raise FileExistsError(target)
    datasets.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=".packaging-", dir=datasets))
    try:
        shutil.copy2(bag_file, staging / "run.mcap")
        if plots_dir is not None:
            shutil.copytree(plots_dir, staging / "plots")
        meta = copy.deepcopy(run_metadata)
        meta.setdefault("integrity", {})["mcap_sha256"] = sha256_file(staging / "run.mcap")
        meta["integrity"]["metadata_sha256"] = metadata_digest(meta)
        write_json(staging / "run_metadata.json", meta)
        saved = json.loads((staging / "run_metadata.json").read_text())
        ok, reasons = validate_minimal(saved)
        if not ok or metadata_digest(saved) != saved["integrity"]["metadata_sha256"]:
            raise ValueError(f"Packaged metadata readback failed: {reasons}")
        if target.exists():
            raise FileExistsError(target)
        os.rename(staging, target)
        return target
    finally:
        if staging.exists():
            shutil.rmtree(staging)
