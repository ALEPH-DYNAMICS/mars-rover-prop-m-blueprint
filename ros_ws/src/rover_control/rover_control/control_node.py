from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


from .shaping import Limits, ShapingConfig, CommandShaper, validate_mode


class RoverControlNode(Node):
    """
    Simulation command-shaping node.

    Contract:
      /cmd_vel      (in)  geometry_msgs/Twist
      /cmd_vel_safe (out) geometry_msgs/Twist

    Enforces:
      - SI units only
      - deadband
      - velocity clamps
      - acceleration (ramp) limits
      - watchdog timeout (stop on stale command)
      - optional slip-based scaling
    """

    def __init__(self) -> None:
        super().__init__("rover_control")

        # Parameters (safe defaults; override via YAML)
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel_safe")
        self.declare_parameter("cmd_timeout_s", 0.5)

        self.declare_parameter("deadband_v_mps", 0.005)
        self.declare_parameter("deadband_omega_rps", 0.01)

        # Mode (modern | prop_m). Bringup can set this.
        self.declare_parameter("mode", "modern")

        # Limits per mode
        self.declare_parameter("limits.v_max_mps.modern", 0.20)
        self.declare_parameter("limits.v_max_mps.prop_m", 0.05)
        self.declare_parameter("limits.omega_max_rps.modern", 0.60)
        self.declare_parameter("limits.omega_max_rps.prop_m", 0.30)
        self.declare_parameter("limits.a_max_mps2.modern", 0.15)
        self.declare_parameter("limits.a_max_mps2.prop_m", 0.05)
        self.declare_parameter("limits.alpha_max_rps2.modern", 0.80)
        self.declare_parameter("limits.alpha_max_rps2.prop_m", 0.30)

        # Slip containment (optional)
        self.declare_parameter("slip.enabled", True)
        self.declare_parameter("slip.topic", "/wheel/slip_estimate")
        self.declare_parameter("slip.s_threshold.modern", 0.30)
        self.declare_parameter("slip.s_threshold.prop_m", 0.20)
        self.declare_parameter("slip.s_hard_limit.modern", 0.60)
        self.declare_parameter("slip.s_hard_limit.prop_m", 0.40)
        self.declare_parameter("slip.speed_scale_gamma.modern", 0.80)
        self.declare_parameter("slip.speed_scale_gamma.prop_m", 0.70)

        self.declare_parameter("slip.timeout_s", 0.5)

        # Internal state
        self._configuration()  # Reject invalid mode/configuration at startup.
        self._shaper = CommandShaper(lambda: self.get_clock().now().nanoseconds * 1e-9)
        self._slip_enabled = self.get_parameter("slip.enabled").value

        # QoS: commands should be RELIABLE
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Slip can be BEST_EFFORT in practice; keep it RELIABLE for now.
        slip_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)

        self._sub_cmd = self.create_subscription(Twist, in_topic, self._on_cmd, cmd_qos)
        self._pub_safe = self.create_publisher(Twist, out_topic, cmd_qos)

        if self._slip_enabled:
            slip_topic = str(self.get_parameter("slip.topic").value)
            self._sub_slip = self.create_subscription(Float64, slip_topic, self._on_slip, slip_qos)

        # Control loop tick
        self._timer = self.create_timer(0.02, self._tick)  # 50 Hz safety shaping

        self.get_logger().info(f"rover_control online: {in_topic} -> {out_topic}")

    def _mode(self) -> str:
        return validate_mode(self.get_parameter("mode").value)

    def _limits(self) -> Limits:
        m = self._mode()
        v_max = self.get_parameter(f"limits.v_max_mps.{m}").value
        w_max = self.get_parameter(f"limits.omega_max_rps.{m}").value
        a_max = self.get_parameter(f"limits.a_max_mps2.{m}").value
        alpha_max = self.get_parameter(f"limits.alpha_max_rps2.{m}").value
        return Limits(v_max=v_max, w_max=w_max, a_max=a_max, alpha_max=alpha_max)

    def _slip_params(self) -> tuple[float, float, float]:
        m = self._mode()
        s_th = self.get_parameter(f"slip.s_threshold.{m}").value
        s_hard = self.get_parameter(f"slip.s_hard_limit.{m}").value
        gamma = self.get_parameter(f"slip.speed_scale_gamma.{m}").value
        return s_th, s_hard, gamma

    def _configuration(self):
        threshold, hard, gamma = self._slip_params()
        return ShapingConfig(
            limits=self._limits(),
            timeout=self.get_parameter("cmd_timeout_s").value,
            deadband_v=self.get_parameter("deadband_v_mps").value,
            deadband_w=self.get_parameter("deadband_omega_rps").value,
            slip_enabled=self.get_parameter("slip.enabled").value,
            slip_timeout=self.get_parameter("slip.timeout_s").value,
            slip_threshold=threshold, slip_hard=hard, slip_gamma=gamma,
        )

    def _on_cmd(self, msg: Twist) -> None:
        if not self._shaper.command_received(msg.linear.x, msg.angular.z):
            self.get_logger().warning("Invalid command; ramping toward zero")

    def _on_slip(self, msg: Float64) -> None:
        if not self._shaper.slip_received(msg.data):
            self.get_logger().warning("Invalid slip evidence; ramping toward zero")

    def _tick(self) -> None:
        try:
            v, w = self._shaper.step(self._configuration())
        except (TypeError, ValueError, OverflowError) as error:
            v, w = self._shaper.inhibit()
            self.get_logger().error(f"Invalid control configuration: {error}")
        out = Twist()
        out.linear.x, out.angular.z = float(v), float(w)
        self._pub_safe.publish(out)


def main() -> None:
    rclpy.init()
    node = RoverControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
