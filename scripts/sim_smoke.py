#!/usr/bin/env python3
"""Observe actual Gazebo clock and node topics; no motion claim."""
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=60)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("timeout must be finite and positive")
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String

    args.out.parent.mkdir(parents=True, exist_ok=True)
    report = {"scope": "clock/node topics only; entity spawning, motion and mission actions unverified", "passed": False}
    with args.out.with_suffix(".log").open("w") as log:
        process = subprocess.Popen(["ros2", "launch", "rover_bringup", "sim_bringup.launch.py", "gui:=false"],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = rclpy.create_node("rover_sim_smoke_observer")
        clock_values, phases, commands = [], [], []
        node.create_subscription(Clock, "/clock", lambda m: clock_values.append(m.clock.sec + m.clock.nanosec * 1e-9), qos_profile_sensor_data)
        node.create_subscription(String, "/mission/state", lambda m: phases.append(m.data), 10)
        node.create_subscription(Twist, "/cmd_vel_safe", lambda m: commands.append((m.linear.x, m.angular.z)), 10)
        deadline = time.monotonic() + args.timeout
        try:
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f"bringup exited early: {process.returncode}")
                rclpy.spin_once(node, timeout_sec=.1)
                if len(clock_values) >= 2 and clock_values[-1] > clock_values[0] and len(phases) >= 2 and len(commands) >= 2:
                    report["passed"] = all(math.isfinite(v) for pair in commands for v in pair)
                    break
            report.update(clock_samples=len(clock_values), phase_samples=len(phases), command_samples=len(commands))
        except Exception as error:
            report["error"] = str(error)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    import os
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            node.destroy_node()
            rclpy.shutdown()
            args.out.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    if not report["passed"]:
        raise SystemExit("Simulator smoke failed; inspect the report and launch log")


if __name__ == "__main__":
    main()
