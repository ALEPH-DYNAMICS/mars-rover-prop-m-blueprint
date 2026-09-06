"""Unit-test the actual node adapters with ROS transport replaced by test doubles.

These check parameter/clock wiring, not a running ROS system.
"""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros_ws/src/rover_control"))


class FakeNode:
    overrides = {}
    def __init__(self, name):
        self.parameters = dict(self.overrides)
        self.now = 0.0
        self.published = []
    def declare_parameter(self, name, default): self.parameters.setdefault(name, default)
    def get_parameter(self, name): return NS(value=self.parameters[name])
    def get_clock(self): return NS(now=lambda: NS(nanoseconds=int(self.now * 1e9)))
    def create_publisher(self, *args): return NS(publish=self.published.append)
    def create_subscription(self, *args): return args
    def create_timer(self, period, callback): return (period, callback)
    def get_logger(self): return NS(info=lambda x: None, warning=lambda x: None, error=lambda x: None)


def load_node(path, name):
    modules = {
        "rclpy": NS(), "rclpy.node": NS(Node=FakeNode),
        "rclpy.qos": NS(QoSProfile=lambda **kw: kw, ReliabilityPolicy=NS(RELIABLE=1), HistoryPolicy=NS(KEEP_LAST=1)),
        "geometry_msgs.msg": NS(Twist=lambda: NS(linear=NS(x=0.0), angular=NS(z=0.0))),
        "std_msgs.msg": NS(Float64=NS, String=NS),
    }
    with patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module


def flatten(values, prefix=""):
    result = {}
    for key, value in values.items():
        if isinstance(value, dict): result.update(flatten(value, prefix + key + "."))
        else: result[prefix + key] = value
    return result


class NodeAdapterTests(unittest.TestCase):
    def tearDown(self): FakeNode.overrides = {}

    def test_effective_parameters_for_each_mode_in_actual_control_node(self):
        module = load_node(ROOT / "ros_ws/src/rover_control/rover_control/control_node.py", "rover_control.adapter_fixture")
        for mode, v, w in [("modern", .2, .6), ("prop_m", .05, .3)]:
            FakeNode.overrides = flatten(yaml.safe_load((ROOT / f"ros_ws/src/rover_bringup/params/modes/{mode}.yaml").read_text())["/**"]["ros__parameters"])
            node = module.RoverControlNode()
            self.assertEqual(node._mode(), mode)
            self.assertEqual((node._configuration().limits.v_max, node._configuration().limits.w_max), (v, w))
        FakeNode.overrides = {"mode": "invalid"}
        with self.assertRaises(ValueError): module.RoverControlNode()

    def test_control_callbacks_and_timer_use_ros_clock(self):
        module = load_node(ROOT / "ros_ws/src/rover_control/rover_control/control_node.py", "rover_control.adapter_fixture")
        FakeNode.overrides = {"slip.enabled": False}
        node = module.RoverControlNode()
        node._on_cmd(NS(linear=NS(x=.2), angular=NS(z=0)))
        node.now = .1; node._tick()
        self.assertAlmostEqual(node.published[-1].linear.x, .015)
        node._tick(); self.assertAlmostEqual(node.published[-1].linear.x, .015)
        node.parameters["limits.v_max_mps.modern"] = float("nan")
        node.now = .2; node._tick()
        self.assertEqual(node.published[-1].linear.x, 0)

    def test_mission_pause_elapsed_time_and_backward_jump(self):
        module = load_node(ROOT / "ros_ws/src/rover_mission_bt/rover_mission_bt/mission_bt_node.py", "mission_adapter_fixture")
        node = module.RoverMissionBTNode()
        for _ in range(20): node._tick()
        self.assertEqual(node._phase, "DRIVE")
        node.now = 20; node._tick(); self.assertEqual(node._phase, "STOP_MEASURE")
        node.now = 23; node._tick(); self.assertEqual(node._phase, "TRANSMIT_LOG")
        node.now = 1; node._tick(); self.assertEqual(node._phase, "STOP_MEASURE")


if __name__ == "__main__": unittest.main()
