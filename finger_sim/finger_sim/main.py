"""
finger_test_scene.py

Drake and drake_ros test scene with meshcat renderer

dip_flexion and left/right_bar_joint have no actuators, they're driven entirely
by geometric loop closure with ball constraints

Torque commands (/finger/torque_commands) are 3-element vectors:
  [mcp_splay, mcp_flexion, pip_flexion]
"""

import sys
import rclpy
from rclpy.node import Node

from rclpy.type_support import check_for_type_support
from drake_ros.core import PySerializer
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# drake_ros imports
from drake_ros.core import (
    init,
    shutdown,
)

# Custom imports
from finger_sim.build_simulator import build_and_run

# ── ROS2 node wrapper ─────────────────────────────────────────────────────────
class FingerSimNode(Node):
    def __init__(self):
        super().__init__("finger_sim")
        self.declare_parameter("sim_duration", 10.0)
        self.declare_parameter("mesh_ext", "gltf")

    @property
    def sim_duration(self) -> float:
        return self.get_parameter("sim_duration").get_parameter_value().double_value

    @property
    def mesh_ext(self) -> str:
        return self.get_parameter("mesh_ext").get_parameter_value().string_value


def main(args=None):
    rclpy.init(args=args)
    init(sys.argv if args is None else args)

    node = FingerSimNode()
    duration = node.sim_duration
    mesh_ext = node.mesh_ext
    node.get_logger().info(
        f"Starting finger test scene (sim_duration={duration} s)"
    )

    # Create all serializers in top-level scope for drake_ros
    check_for_type_support(JointState)
    joint_state_serializer = PySerializer(JointState)
    check_for_type_support(Float64MultiArray)
    torque_serializer = PySerializer(Float64MultiArray)

    try:
        build_and_run(duration, mesh_ext, joint_state_serializer, torque_serializer)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        shutdown()


if __name__ == "__main__":
    main()