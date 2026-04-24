"""
finger_test_scene.py

Drake and drake_ros test scene with meshcat renderer

dip_flexion and left/right_bar_joint have no actuators, they're driven entirely
by geometric loop closure with ball constraints

Motor torque commands (/finger/motor_torque_commands) are 3-element vectors:
    [motor_0, motor_1, motor_2]
"""

import sys
import os

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


def main(args=None):
    argv = sys.argv if args is None else args
    init(argv)
    sim_duration = float(os.environ.get("SIM_DURATION", "120.0"))
    mesh_ext = os.environ.get("MESH_EXT", "gltf")
    print(f"[finger_sim] Starting finger test scene (sim_duration={sim_duration} s")

    # Create all serializers in top-level scope for drake_ros
    check_for_type_support(JointState)
    joint_state_serializer = PySerializer(JointState)
    check_for_type_support(Float64MultiArray)
    torque_serializer = PySerializer(Float64MultiArray)

    try:
        build_and_run(sim_duration, mesh_ext, joint_state_serializer, torque_serializer)
    finally:
        shutdown()


if __name__ == "__main__":
    main()