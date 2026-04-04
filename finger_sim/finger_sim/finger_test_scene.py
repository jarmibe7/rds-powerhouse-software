"""
finger_test_scene.py

Drake and drake_ros test scene with meshcat renderer
"""

import sys
import os

import rclpy
from rclpy.node import Node

# Drake imports
from pydrake.systems.primitives import Demultiplexer
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem, BasicVector, EventStatus
from pydrake.math import RigidTransform
from pydrake.common.value import AbstractValue # type: ignore
from pydrake.geometry import (
    Box,
    Meshcat,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
)

# drake_ros imports
from drake_ros.core import RosInterfaceSystem, RosSubscriberSystem, RosPublisherSystem, PySerializer, init, shutdown
from drake_ros.tf2 import SceneTfBroadcasterSystem, SceneTfBroadcasterParams
from sensor_msgs.msg import JointState
from pydrake.systems.framework import TriggerType
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.type_support import check_for_type_support


#
# Helpers
#

def resolve_package_path(package_name: str, relative_path: str) -> str:
    """Return the absolute path to a file inside a ROS 2 package share dir."""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(package_name), relative_path)


def process_xacro(xacro_path: str, mesh_ext: str) -> str:
    """
    Run xacro on the given file with mesh_ext as a mapping argument.
    Returns the processed URDF as a string.
    """
    import xacro
    doc = xacro.process_file(
        xacro_path,
        mappings={"mesh_ext": mesh_ext},
    )
    return doc.toprettyxml(indent="  ")

class MultiArrayToVector(LeafSystem):
    """Converts std_msgs/Float64MultiArray to a vector output."""
    def __init__(self, size):
        super().__init__()
        self.size = size

        # abstract input: provide a default value of the expected type
        self.DeclareAbstractInputPort("msg", AbstractValue.Make(Float64MultiArray()))

        # vector output
        self.DeclareVectorOutputPort("vector", BasicVector(size), self.calc_output)

    def calc_output(self, context, output):
        msg = self.get_input_port(0).Eval(context)
        # Copy Float64MultiArray data into vector (truncate/pad to match size)
        output.SetFromVector(list(msg.data[:self.size]) + [0.0]*(self.size - len(msg.data)))


class FingerJointStatePublisher(LeafSystem):
    """
    Publishes a ROS JointState message for the actuated finger joints.
    """
    def __init__(self, plant, model_instance):
        super().__init__()
        self.plant = plant
        self.model_instance = model_instance

        # List only the actuated finger joints
        self.active_joints = [
            "mcp_splay", "mcp_flexion", "pip_flexion", "dip_flexion"
        ]

        # Map joint names to indices in the plant state vector
        self.joint_name_to_pos_index = {}
        self.joint_name_to_vel_index = {}

        for name in self.active_joints:
            joint = self.plant.GetJointByName(name, self.model_instance)
            if joint.num_positions() == 1:
                self.joint_name_to_pos_index[name] = joint.position_start()
                self.joint_name_to_vel_index[name] = joint.velocity_start()

        # Keep only joints that exist
        self.active_joints = [name for name in self.active_joints if name in self.joint_name_to_pos_index]

        self.pos_indices = [self.joint_name_to_pos_index[name] for name in self.active_joints]
        self.vel_indices = [self.joint_name_to_vel_index[name] for name in self.active_joints]

        self.zero_effort = [0.0] * len(self.active_joints)

        # Declare input ports: full joint positions/velocities, plus current time
        num_positions = self.plant.num_positions(self.model_instance)
        num_velocities = self.plant.num_velocities(self.model_instance)
        self.DeclareVectorInputPort("joint_positions", num_positions)
        self.DeclareVectorInputPort("joint_velocities", num_velocities)

        # Declare output port
        self.DeclareAbstractOutputPort(
            "joint_state_msg",
            lambda: AbstractValue.Make(JointState()),
            self._calc_output
        )

    def _calc_output(self, context, output):
        positions = self.get_input_port(0).Eval(context)
        velocities = self.get_input_port(1).Eval(context)
        t = context.get_time()  # ← get time directly from context

        msg = JointState()
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int((t % 1.0) * 1e9)
        msg.name = self.active_joints
        msg.position = [positions[i] for i in self.pos_indices]
        msg.velocity = [velocities[i] for i in self.vel_indices]
        msg.effort = self.zero_effort

        output.set_value(msg)

# 
# Main scene builder
#

def build_and_run(sim_duration: float, mesh_ext: str) -> None:
    # These need to go here to keep joint state serializer in higher scope
    check_for_type_support(JointState)
    joint_state_serializer = PySerializer(JointState)

    builder = DiagramBuilder()

    # MultibodyPlant + SceneGraph
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)

    parser = Parser(plant)
    parser.package_map().PopulateFromEnvironment("AMENT_PREFIX_PATH")

    # Load and process xacro
    xacro_path = resolve_package_path(
        "finger_description",
        "urdf/powerhouse_finger.urdf.xacro",
    )
    urdf_string = process_xacro(xacro_path, mesh_ext)

    # AddModelsFromString needs a base URL so package:// URIs resolve
    # correctly relative to the finger_description share directory.
    package_dir = resolve_package_path("finger_description", "")
    models = parser.AddModelsFromString(urdf_string, "urdf")
    print(f"Loaded {len(models)} models")
    finger_model = models[0]

    # Weld base_link to world
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("base_link", finger_model),
        RigidTransform(),
    )

    # TODO: Do joint coupling instead of independent
    plant.AddJointActuator("mcp_splay", plant.GetJointByName("mcp_splay", finger_model))
    plant.AddJointActuator("mcp_flexion", plant.GetJointByName("mcp_flexion", finger_model))
    plant.AddJointActuator("pip_flexion", plant.GetJointByName("pip_flexion", finger_model))
    plant.AddJointActuator("dip_flexion", plant.GetJointByName("dip_flexion", finger_model))

    # Ground
    ground_friction = CoulombFriction(static_friction=0.7, dynamic_friction=0.5)
    plant.RegisterCollisionGeometry(
        plant.world_body(),
        RigidTransform(p=[0, 0, -0.01]),
        Box(2.0, 2.0, 0.02),
        "ground_collision",
        ground_friction,
    )
    plant.RegisterVisualGeometry(
        plant.world_body(),
        RigidTransform(p=[0, 0, -0.01]),
        Box(2.0, 2.0, 0.02),
        "ground_visual",
        [0.5, 0.5, 0.5, 1.0],
    )

    plant.Finalize()
    print("Number of actuators:", plant.num_actuators())

    # Meshcat
    meshcat = Meshcat(port=7000)
    MeshcatVisualizer.AddToBuilder(
        builder, scene_graph, meshcat,
        MeshcatVisualizerParams(),
    )
    print(f"[finger_sim] Meshcat running at: {meshcat.web_url()}")

    # Broadcast tf from drake_ros
    ros_interface_system = builder.AddSystem(RosInterfaceSystem("finger_sim_node"))
    drake_ros = ros_interface_system.get_ros_interface()

    tf_broadcaster = builder.AddSystem(
        SceneTfBroadcasterSystem(
            drake_ros,
            params=SceneTfBroadcasterParams(
                publish_triggers={TriggerType.kPeriodic},
                publish_period=0.05,
            ),
        )
    )
    tf_broadcaster.RegisterMultibodyPlant(plant)
    builder.Connect(
        scene_graph.get_query_output_port(),
        tf_broadcaster.get_graph_query_input_port(),
    )

    # FingerJointStatePublisher
    joint_state_src = builder.AddSystem(FingerJointStatePublisher(plant, finger_model))

    # Split plant state into positions and velocities
    num_q = plant.num_positions(finger_model)
    num_v = plant.num_velocities(finger_model)

    demux = builder.AddSystem(Demultiplexer([num_q, num_v]))
    builder.Connect(plant.get_state_output_port(finger_model),
                    demux.get_input_port(0))

    # Connect demux outputs → FingerJointStatePublisher
    builder.Connect(demux.get_output_port(0), joint_state_src.get_input_port(0))  # positions
    builder.Connect(demux.get_output_port(1), joint_state_src.get_input_port(1))  # velocities

    # 4) ROS publisher with QoS
    joint_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

    joint_state_pub = builder.AddSystem(
        RosPublisherSystem(
            joint_state_serializer,
            "/joint_states",
            joint_qos,
            drake_ros,
            {TriggerType.kPeriodic},
            0.01  # 100 Hz pub freq
        )
    )

    # Connect FingerJointStatePublisher → ROS publisher
    builder.Connect(
        joint_state_src.get_output_port(0),
        joint_state_pub.get_input_port(0)
    )


    # Build and simulate
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(0.01)

    print(f"[finger_sim] mesh_ext={mesh_ext!r}  sim_duration={sim_duration} s")
    print(f"[finger_sim] Simulating ...  (Ctrl-C to stop early)")
    try:
        simulator.AdvanceTo(sim_duration)
    except KeyboardInterrupt:
        pass

    print("[finger_sim] Done.")


# ROS2 node wrapper for simulation
class FingerSimNode(Node):
    def __init__(self):
        super().__init__("finger_sim")
        self.declare_parameter("sim_duration", 10.0)
        self.declare_parameter("mesh_ext", "gltf")   # Default to gltf for Drake

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
        f"Starting finger test scene (sim_duration={duration} s, mesh_ext={mesh_ext!r})"
    )

    try:
        build_and_run(duration, mesh_ext)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        shutdown()


if __name__ == "__main__":
    main()