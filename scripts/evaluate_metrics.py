#!/usr/bin/env python3
"""Checkout wrapper; implementation is installed with rover_tools."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros_ws/src/rover_tools"))
from rover_tools.metrics import main
if __name__ == "__main__":
    main()
