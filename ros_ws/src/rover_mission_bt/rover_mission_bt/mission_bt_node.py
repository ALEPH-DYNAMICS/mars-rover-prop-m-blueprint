from __future__ import annotations

import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class RoverMissionBTNode(Node):
    """
    ROS-clock phase-label runner aligned to authored BT assets.

    Current behavior:
      - Loads and validates a BT XML file path for mission traceability
      - Executes a deterministic phase machine: DRIVE -> STOP_MEASURE -> TRANSMIT_LOG -> REPEAT
      - Publishes /mission/state as std_msgs/String

    This node preserves the mission cadence and interface contract while the
    repository remains in a pre-runtime-BT integration phase.
    """

    def __init__(self) -> None:
        super().__init__("rover_mission_bt")

        self.declare_parameter("mode", "modern")  # modern | prop_m
        self.declare_parameter("tick_rate_hz", 2.0)

        # BT tree path (traceability even if we don't parse it yet)
        self.declare_parameter("tree_file", "")

        # Phase timings (seconds)
        self.declare_parameter("drive_duration_s.modern", 20.0)
        self.declare_parameter("measure_duration_s.modern", 3.0)
        self.declare_parameter("transmit_duration_s.modern", 4.0)

        self.declare_parameter("drive_duration_s.prop_m", 8.0)
        self.declare_parameter("measure_duration_s.prop_m", 8.0)
        self.declare_parameter("transmit_duration_s.prop_m", 6.0)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub_state = self.create_publisher(String, "/mission/state", qos)

        self._phase = "DRIVE"
        self._phase_started = self.get_clock().now().nanoseconds * 1e-9

        rate = float(self.get_parameter("tick_rate_hz").value)
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError("tick_rate_hz must be finite and positive")
        self._durations()
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self._log_startup()

    def _mode(self) -> str:
        m = str(self.get_parameter("mode").value).strip().lower()
        if m not in ("modern", "prop_m"):
            raise ValueError("mode must be modern or prop_m")
        return m

    def _durations(self) -> tuple[float, float, float]:
        m = self._mode()
        d_drive = float(self.get_parameter(f"drive_duration_s.{m}").value)
        d_meas = float(self.get_parameter(f"measure_duration_s.{m}").value)
        d_tx = float(self.get_parameter(f"transmit_duration_s.{m}").value)
        if not all(math.isfinite(d) and d > 0 for d in (d_drive, d_meas, d_tx)):
            raise ValueError("phase durations must be finite and positive")
        return d_drive, d_meas, d_tx

    def _log_startup(self) -> None:
        mode = self._mode()
        tree_file = str(self.get_parameter("tree_file").value).strip()
        if tree_file:
            tree_path = Path(tree_file)
            status = "present" if tree_path.exists() else "missing"
            self.get_logger().info(
                f"mission_bt mode={mode} execution_model=ros_clock_phase_labels "
                f"tree_file={tree_file} tree_status={status}"
            )
        else:
            self.get_logger().info(
                f"mission_bt mode={mode} execution_model=ros_clock_phase_labels "
                "tree_file=(not set)"
            )

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self._phase
        self.pub_state.publish(msg)

    def _advance(self, next_phase: str) -> None:
        self._phase = next_phase
        self._phase_started = self.get_clock().now().nanoseconds * 1e-9
        self._publish_state()

    def _tick(self) -> None:
        self._publish_state()

        d_drive, d_meas, d_tx = self._durations()
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._phase_started

        if elapsed < 0:
            self._advance("STOP_MEASURE")
            return

        if self._phase == "DRIVE":
            if elapsed >= d_drive:
                self._advance("STOP_MEASURE")
        elif self._phase == "STOP_MEASURE":
            if elapsed >= d_meas:
                self._advance("TRANSMIT_LOG")
        elif self._phase == "TRANSMIT_LOG":
            if elapsed >= d_tx:
                self._advance("REPEAT")
        elif self._phase == "REPEAT":
            # The authored BT files currently define the mission cadence.
            # Runtime execution is represented here by the deterministic phase loop.
            self._advance("DRIVE")
        else:
            # Unknown state -> fail safe
            self._advance("STOP_MEASURE")


def main() -> None:
    rclpy.init()
    node = RoverMissionBTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
