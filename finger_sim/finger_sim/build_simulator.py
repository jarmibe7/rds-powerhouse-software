"""
Helper functions for building the Drake simulation of the powerhouse finger
"""
import os

import yaml # type: ignore
import numpy as np # type: ignore

# Drake imports
from pydrake.systems.primitives import Demultiplexer
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.math import RigidTransform
from pydrake.geometry import (
    Box,
    Meshcat,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
)

# drake_ros imports
from drake_ros.core import (
    RosInterfaceSystem,
    RosSubscriberSystem,
    RosPublisherSystem,
    PySerializer,
)
from drake_ros.tf2 import SceneTfBroadcasterSystem, SceneTfBroadcasterParams
from sensor_msgs.msg import JointState
from pydrake.systems.framework import TriggerType
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.type_support import check_for_type_support

# Custom imports
from finger_sim.drake_core_systems import MultiArrayToVector
from finger_sim.drake_ros_systems import FingerJointStatePublisher

# ── Helpers ───────────────────────────────────────────────────────────────────
def resolve_package_path(package_name: str, relative_path: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(package_name), relative_path)

def process_xacro(xacro_path: str, mesh_ext: str) -> str:
    import xacro
    doc = xacro.process_file(xacro_path, mappings={"mesh_ext": mesh_ext})
    return doc.toprettyxml(indent="  ")

def load_pin_offsets(axes_yaml_path: str):
    """
    Made entirely by Claude!

    Read joint_axes.yaml and compute body-frame pin offsets for the two
    DIP-side loop closure constraints.

    Explanation
    --------
    The Blender export script stores world-space positions for every joint and for the two DIP-side pin

    Because Drake places each link's body frame at its parent joint origin,
    the body-frame offset of any point P on body B is:

        P_B = P_world - joint_B_world

    where joint_B_world is the world position of B's parent joint Empty.

    Constraints
    -----------
      left_bar  body frame origin  = left_bar_joint  world origin
      right_bar body frame origin  = right_bar_joint world origin
      distal    body frame origin  = dip_flexion      world origin

    Returns
    -------
    (P_LeftBar_DipPin, P_RightBar_DipPin, P_Distal_LeftPin, P_Distal_RightPin)
    All as np.ndarray shape (3,), in meters.
    """
    with open(axes_yaml_path) as f:
        data = yaml.safe_load(f)

    def jw(name):
        """World origin of a joint Empty, as np.array."""
        d = data["joints"][name]["world_origin"]
        return np.array([d["x"], d["y"], d["z"]])

    def ew(name):
        """World origin of an extra_frames Empty, as np.array."""
        d = data["extra_frames"][name]["world_origin"]
        return np.array([d["x"], d["y"], d["z"]])

    left_bar_pip_world   = jw("left_bar_joint")
    right_bar_pip_world  = jw("right_bar_joint")
    dip_flex_world       = jw("dip_flexion")
    left_bar_dip_world   = ew("left_bar_dip")
    right_bar_dip_world  = ew("right_bar_dip")

    P_LeftBar_DipPin  = left_bar_dip_world  - left_bar_pip_world
    P_RightBar_DipPin = right_bar_dip_world - right_bar_pip_world
    P_Distal_LeftPin  = left_bar_dip_world  - dip_flex_world
    P_Distal_RightPin = right_bar_dip_world - dip_flex_world

    return P_LeftBar_DipPin, P_RightBar_DipPin, P_Distal_LeftPin, P_Distal_RightPin


# ── Main Building Functions ───────────────────────────────────────────────────────────────────
def build_plant(builder, mesh_ext):
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)

    parser = Parser(plant)
    parser.package_map().PopulateFromEnvironment("AMENT_PREFIX_PATH")

    # ── Load URDF ─────────────────────────────────────────────────────────────
    xacro_path = resolve_package_path(
        "finger_description",
        "urdf/powerhouse_finger.urdf.xacro",
    )
    urdf_string = process_xacro(xacro_path, mesh_ext)
    models = parser.AddModelsFromString(urdf_string, "urdf")   # Need to turn into raw URDF for drake
    print(f"Loaded {len(models)} models")
    finger_model = models[0]

    # ── Weld base to world ────────────────────────────────────────────────────
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("base_link", finger_model),
        RigidTransform(),
    )

    # ── Actuators ─────────────────────────────────────────────────────────────
    # 3 actuators, DIP is driven by loop closure constraints
    # left_bar_joint and right_bar_joint are also passive (effort=0 in URDF).
    plant.AddJointActuator("mcp_splay",   plant.GetJointByName("mcp_splay",   finger_model))
    plant.AddJointActuator("mcp_flexion", plant.GetJointByName("mcp_flexion", finger_model))
    plant.AddJointActuator("pip_flexion", plant.GetJointByName("pip_flexion", finger_model))

    # ── 4-bar loop closure constraints ────────────────────────────────────────
    # Load DIP-side pin offsets from joint_axes.yaml
    axes_yaml_path = resolve_package_path("finger_description", "meshes/joint_axes.yaml")
    (
        P_LeftBar_DipPin,
        P_RightBar_DipPin,
        P_Distal_LeftPin,
        P_Distal_RightPin,
    ) = load_pin_offsets(axes_yaml_path)

    print(f"[finger_sim] Left  bar DIP pin offset in bar frame:     {P_LeftBar_DipPin}")
    print(f"[finger_sim] Right bar DIP pin offset in bar frame:     {P_RightBar_DipPin}")
    print(f"[finger_sim] Left  DIP pin offset in distal frame:      {P_Distal_LeftPin}")
    print(f"[finger_sim] Right DIP pin offset in distal frame:      {P_Distal_RightPin}")

    left_bar_body  = plant.GetBodyByName("left_bar",       finger_model)
    right_bar_body = plant.GetBodyByName("right_bar",      finger_model)
    distal_body    = plant.GetBodyByName("distal_phalanx", finger_model)

    # Close the left bar loop with ball constraints to mimic pin joints
    plant.AddBallConstraint(
        body_A=left_bar_body,
        p_AP=P_LeftBar_DipPin,
        body_B=distal_body,
        p_BQ=P_Distal_LeftPin,
    )

    plant.AddBallConstraint(
        body_A=right_bar_body,
        p_AP=P_RightBar_DipPin,
        body_B=distal_body,
        p_BQ=P_Distal_RightPin,
    )

    # ── Ground plane ──────────────────────────────────────────────────────────
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
    print(f"[finger_sim] Actuators: {plant.num_actuators()}")
    return plant, scene_graph, finger_model

def build_ros(builder, plant, scene_graph, joint_state_serializer, torque_serializer, finger_model):
    # ── tf broadcaster ─────────────────────────────────────────────────────────
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

    # ── Joint state publisher ─────────────────────────────────────────────────
    joint_state_src = builder.AddSystem(FingerJointStatePublisher(plant, finger_model))

    num_q = plant.num_positions(finger_model)
    num_v = plant.num_velocities(finger_model)

    demux = builder.AddSystem(Demultiplexer([num_q, num_v]))
    builder.Connect(plant.get_state_output_port(finger_model), demux.get_input_port(0))
    builder.Connect(demux.get_output_port(0), joint_state_src.get_input_port(0))  # positions
    builder.Connect(demux.get_output_port(1), joint_state_src.get_input_port(1))  # velocities

    joint_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )

    joint_state_pub = builder.AddSystem(
        RosPublisherSystem(
            joint_state_serializer,
            "/joint_states",
            joint_qos,
            drake_ros,
            {TriggerType.kPeriodic},
            0.01,
        )
    )
    builder.Connect(joint_state_src.get_output_port(0), joint_state_pub.get_input_port(0))

    # ── Torque command subscriber ─────────────────────────────────────────────
    # Commands are 3-element: [mcp_splay, mcp_flexion, pip_flexion]
    

    torque_sub = builder.AddSystem(
        RosSubscriberSystem(
            torque_serializer,
            "/finger/torque_commands",
            joint_qos,
            drake_ros,
        )
    )

    torque_converter = builder.AddSystem(MultiArrayToVector(plant.num_actuators()))
    builder.Connect(torque_sub.get_output_port(0),      torque_converter.get_input_port(0))
    builder.Connect(torque_converter.get_output_port(0), plant.get_actuation_input_port())

def build_and_run(sim_duration, mesh_ext, joint_state_serializer, torque_serializer):
    """
    Main function for building the Drake finger simulation.

    Args:
        sim_duration: How many seconds to run the sim for
        mesh_ext: What mesh file type extension to pass into the robot .xacro
        *_serializer: ROS2 data type serializers to work with drake_ros
    """

    builder = DiagramBuilder()
    plant, scene_graph, finger_model = build_plant(builder, mesh_ext)

    # ── Meshcat ───────────────────────────────────────────────────────────────
    meshcat = Meshcat(port=7000)
    MeshcatVisualizer.AddToBuilder(
        builder, scene_graph, meshcat,
        MeshcatVisualizerParams(),
    )
    print(f"[finger_sim] Meshcat running at: {meshcat.web_url()}")

    build_ros(builder, plant, scene_graph, joint_state_serializer, torque_serializer, finger_model)

    # ── Build and simulate ────────────────────────────────────────────────────
    diagram  = builder.Build()
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    # simulator.AdvanceTo(0.01)

    print(f"[finger_sim] sim_duration={sim_duration} s")
    print(f"[finger_sim] Simulating ...  (Ctrl-C to stop early)")
    try:
        simulator.AdvanceTo(sim_duration)
    except KeyboardInterrupt:
        pass

    print("[finger_sim] Done.")