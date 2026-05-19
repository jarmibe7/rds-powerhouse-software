"""
Helper functions for building the Drake simulation of the powerhouse finger
"""
import os

import yaml # type: ignore
import numpy as np # type: ignore
import graphviz # type: ignore

# Drake imports
from pydrake.systems.primitives import Demultiplexer
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, CoulombFriction, DiscreteContactApproximation, ContactModel
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
from pydrake.geometry import (
    Box,
    Cylinder,
    Mesh,
    Meshcat,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    ProximityProperties,
    Role,
    Rgba,
    AddRigidHydroelasticProperties,
)

from pydrake.multibody.tree import PrismaticJoint, SpatialInertia, UnitInertia

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
from finger_sim.drake_core_systems import (
    FingertipContactForceReporter,
    MultiArrayToVector,
    MotorTorqueToJointTorque,
    TendonTensionToStress,
    VectorToMultiArray,
)
from finger_sim.drake_ros_systems import FingerJointStatePublisher

# Hard-coded demo selection. Options: 'none', 'table', 'holes', 'hammer', or 'weight'.
DEMO_NAME = "weight"

# ── Helpers ───────────────────────────────────────────────────────────────────
def resolve_package_path(package_name, relative_path):
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(package_name), relative_path)

def process_xacro(xacro_path, mesh_ext):
    import xacro
    doc = xacro.process_file(xacro_path, mappings={"mesh_ext": mesh_ext})
    return doc.toprettyxml(indent="  ")

def load_pin_offsets(axes_yaml_path):
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

    right_bar_dip_from_bar    = right_bar_pip_world  + P_RightBar_DipPin
    right_bar_dip_from_distal = dip_flex_world       + P_Distal_RightPin

    print(f"[finger_sim] Right DIP pin from bar frame:    {right_bar_dip_from_bar}")
    print(f"[finger_sim] Right DIP pin from distal frame: {right_bar_dip_from_distal}")
    print(f"[finger_sim] Constraint gap at rest:          {right_bar_dip_from_bar - right_bar_dip_from_distal}")

    return P_LeftBar_DipPin, P_RightBar_DipPin, P_Distal_LeftPin, P_Distal_RightPin


# ── Main Building Functions ───────────────────────────────────────────────────────────────────
def build_plant(builder, mesh_ext):
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)

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
        RigidTransform(RotationMatrix(RollPitchYaw(np.radians(180.0), np.radians(0.0), np.radians(0.0))), p=[0.0,0.0,0.0]),
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


    # Optional demo setup (create board, nails, etc.)
    demo_name = DEMO_NAME
    if demo_name and demo_name.lower() != "none":
        setup_demo(demo_name.lower(), builder, plant, scene_graph, finger_model, mesh_ext)

    # Set contact model to hydroelastic with fallback for non-convex mesh collision
    plant.set_contact_model(ContactModel.kHydroelasticWithFallback)
    plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap)
    plant.set_sap_near_rigid_threshold(0.1e-3)

    plant.Finalize()
    print(f"[finger_sim] Actuators: {plant.num_actuators()}")
    return plant, scene_graph, finger_model


# Store initial poses for free (dynamic) demo objects that need placement after finalize
_DEMO_FREE_BODY_INIT_POSES = []  # list of (body_name, RigidTransform)
_DEMO_MODEL_INSTANCE = None
_DEMO_JOINT_INIT_POSITIONS = []  # list of (joint_name, position)


def setup_demo(demo_name, builder, plant, scene_graph, finger_model, mesh_ext):
    """Setup the demo specified by demo name"""
    func_name = f"setup_{demo_name}"
    fn = globals().get(func_name)
    if not fn:
        print(f"[finger_sim] No demo named '{demo_name}' available")
        return
    print(f"[finger_sim] Setting up demo: {demo_name}")
    fn(builder, plant, scene_graph, finger_model, mesh_ext)


def _add_board(plant, center=[0.15, 0.0, 0.05], rpy_deg=None, friction=None):
    """
    Add the board to the simulator with rigid hydroelastic contact.
    This preserves the original mesh geometry (including holes) without convex-hulling.
    
    Args:
        plant: MultibodyPlant instance
        center: [x, y, z] position of board center
        rpy_deg: [roll, pitch, yaw] rotation in degrees, or None for identity rotation
        friction: CoulombFriction object (used for visual only; hydroelastic ignores it)
    """
    if friction is None:
        friction = CoulombFriction(static_friction=0.9, dynamic_friction=0.5)
    
    # Build rotation matrix from roll-pitch-yaw if provided
    if rpy_deg is None:
        R = RotationMatrix()  # Identity rotation
    else:
        # Convert degrees to radians
        roll_rad = np.radians(rpy_deg[0])
        pitch_rad = np.radians(rpy_deg[1])
        yaw_rad = np.radians(rpy_deg[2])
        rpy = RollPitchYaw(roll_rad, pitch_rad, yaw_rad)
        R = RotationMatrix(rpy)
    
    pose = RigidTransform(R=R, p=center)
    table_mesh_path = resolve_package_path("finger_description", "meshes/table.obj")
    
    # Register visual geometry (standard mesh)
    plant.RegisterVisualGeometry(plant.world_body(), pose, Mesh(table_mesh_path), 
                                 "demo_board_visual", [0.6, 0.3, 0.2, 1.0])
    
    # Register collision geometry with rigid hydroelastic contact
    # This preserves the actual mesh topology (including holes) instead of convex-hulling
    proximity_props = ProximityProperties()
    AddRigidHydroelasticProperties(properties=proximity_props)
    plant.RegisterCollisionGeometry(plant.world_body(), pose, Mesh(table_mesh_path),
                                    "demo_board_collision", proximity_props)


def _add_table(plant, center=[0.15, 0.0, 0.05], rpy_deg=None, size=[0.5, 0.5, 0.02], friction=None):
    """
    Add a simple box-shaped table to act as the ground plane.

    Args:
        plant: MultibodyPlant instance
        center: [x, y, z] position of the box center
        rpy_deg: [roll, pitch, yaw] rotation in degrees, or None for identity rotation
        size: [x_size, y_size, z_size] full extents of the box (meters)
        friction: CoulombFriction (optional)
    """
    if friction is None:
        friction = CoulombFriction(static_friction=1.5, dynamic_friction=1.0)

    # Build rotation matrix from roll-pitch-yaw if provided
    if rpy_deg is None:
        R = RotationMatrix()  # Identity rotation
    else:
        roll_rad = np.radians(rpy_deg[0])
        pitch_rad = np.radians(rpy_deg[1])
        yaw_rad = np.radians(rpy_deg[2])
        rpy = RollPitchYaw(roll_rad, pitch_rad, yaw_rad)
        R = RotationMatrix(rpy)

    pose = RigidTransform(R=R, p=center)

    # Create box shape for visual and collision
    box = Box(size[0], size[1], size[2])
    plant.RegisterVisualGeometry(plant.world_body(), pose, box,
                                 "demo_table_visual", [0.6, 0.3, 0.2, 1.0])

    # Register collision geometry with rigid hydroelastic contact
    proximity_props = ProximityProperties()
    AddRigidHydroelasticProperties(properties=proximity_props)
    plant.RegisterCollisionGeometry(plant.world_body(), pose, box,
                                    "demo_table_collision", proximity_props)


def _add_nail(plant, name, world_pos, weld=True, mass=0.1, friction=None, model_instance=None):
    if friction is None:
        friction = CoulombFriction(static_friction=0.3, dynamic_friction=0.2)

    # Estimate inertia from nail mesh (0.0556m long, 0.00318m radius cylinder)
    length = 0.0556
    radius = 0.00318
    inertia = SpatialInertia(mass, np.zeros(3), UnitInertia.SolidCylinder(radius, length, np.array([0.0, 0.0, 1.0])))

    global _DEMO_MODEL_INSTANCE
    if model_instance is None:
        if _DEMO_MODEL_INSTANCE is None:
            _DEMO_MODEL_INSTANCE = plant.AddModelInstance("demo_objects")
        body = plant.AddRigidBody(name, _DEMO_MODEL_INSTANCE, inertia)
    else:
        body = plant.AddRigidBody(name, model_instance, inertia)

    geom_pose = RigidTransform()
    nail_mesh_path = resolve_package_path("finger_description", "meshes/nail.obj")
    # For collision, use a simple cylinder that matches the OBJ geometry
    # Use rigid hydroelastic for consistency with the board
    proximity_props = ProximityProperties()
    AddRigidHydroelasticProperties(properties=proximity_props)
    plant.RegisterCollisionGeometry(body, geom_pose, Cylinder(radius, length), f"{name}_collision", proximity_props)
    plant.RegisterVisualGeometry(body, geom_pose, Mesh(nail_mesh_path), f"{name}_visual", [0.8, 0.8, 0.8, 1.0])

    # Position nail at world_pos (assumes world_pos is the base position)
    X_WB = RigidTransform(p=world_pos)

    if weld:
        plant.WeldFrames(plant.world_frame(), body.body_frame(), X_WB)
    else:
        # Nail is a free body; save its initial pose to be applied after finalize
        _DEMO_FREE_BODY_INIT_POSES.append((name, X_WB))

def setup_table(builder, plant, scene_graph, finger_model, mesh_ext):
    _add_table(plant)

def setup_holes(builder, plant, scene_graph, finger_model, mesh_ext):
    """Create a board with nails already inserted (welded)."""
    board_center = [0.15, 0.0, 0.05]
    _add_board(plant, board_center)

    # Create 3 nails spaced 0.25m apart in y, symmetric about origin
    nail_y_positions = [-0.125, 0.0, 0.125]
    for i, y in enumerate(nail_y_positions):
        nail_world = [board_center[0], board_center[1] + y, board_center[2]]
        _add_nail(plant, f"nail_{i}", nail_world, weld=True)


def setup_hammer(builder, plant, scene_graph, finger_model, mesh_ext):
    """Create a board and nails that are dynamic (so the finger can hammer them)."""
    board_center = [0.15, 0.0, 0.05]
    _add_board(plant, board_center)

    # Create 3 dynamic nails spaced 0.25m apart in y, symmetric about origin
    nail_y_positions = [-0.125, 0.0, 0.125]
    for i, y in enumerate(nail_y_positions):
        nail_world = [board_center[0], board_center[1] + y, board_center[2] + 0.2]
        _add_nail(plant, f"nail_{i}", nail_world, weld=False)


def setup_weight(builder, plant, scene_graph, finger_model, mesh_ext, center=None, rpy_deg=None, weld=False):
    """Load the weight assembly SDF into the scene."""
    table_center = [0.9, 0.0, -0.29]
    _add_table(plant, center=table_center, size=[2.0, 2.0, 0.2])

    if center is None:
        center = table_center + np.array([0.0, 0.0, 0.8])

    if rpy_deg is None:
        R = RotationMatrix()
    else:
        rpy = RollPitchYaw(np.radians(rpy_deg[0]), np.radians(rpy_deg[1]), np.radians(rpy_deg[2]))
        R = RotationMatrix(rpy)

    weight_sdf_path = resolve_package_path("finger_description", "urdf/weight.sdf")

    parser = Parser(plant)
    parser.package_map().PopulateFromEnvironment("AMENT_PREFIX_PATH")
    models = parser.AddModels(weight_sdf_path)
    if not models:
        print("[finger_sim] Warning: no models were loaded from weight.sdf")
        return

    weight_model = models[0]
    X_WB = RigidTransform(R=R, p=center)
    if weld:
        weight_body = plant.GetBodyByName("weight_body", weight_model)
        plant.WeldFrames(plant.world_frame(), weight_body.body_frame(), X_WB)
    else:
        _DEMO_FREE_BODY_INIT_POSES.append(("weight_body", X_WB))


def build_ros(
    builder,
    plant,
    scene_graph,
    joint_state_serializer,
    torque_serializer,
    stress_serializer,
    tension_serializer,
    finger_model,
):
    # ── tf broadcaster ─────────────────────────────────────────────────────────
    ros_interface_system = builder.AddSystem(RosInterfaceSystem("finger_sim"))
    drake_ros = ros_interface_system.get_ros_interface()

    # tf_broadcaster = builder.AddSystem(
    #     SceneTfBroadcasterSystem(
    #         drake_ros,
    #         params=SceneTfBroadcasterParams(
    #             publish_triggers={TriggerType.kPeriodic},
    #             publish_period=0.05,
    #         ),
    #     )
    # )
    # tf_broadcaster.RegisterMultibodyPlant(plant)
    # builder.Connect(
    #     scene_graph.get_query_output_port(),
    #     tf_broadcaster.get_graph_query_input_port(),
    # )

    # ── Joint state publisher ─────────────────────────────────────────────────
    joint_state_src = builder.AddSystem(FingerJointStatePublisher(plant, finger_model))

    num_q = plant.num_positions(finger_model)
    num_v = plant.num_velocities(finger_model)
    pip_position_index = plant.GetJointByName("pip_flexion", finger_model).position_start()
    jacobian_csv_path = resolve_package_path("finger_control", "config/jacobian_transposes.csv")

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

    # ── Motor torque subscriber + tendon mapping ─────────────────────────────
    # Commands are 4-element: [motor_0, motor_1, motor_2, motor_3]
    torque_sub = builder.AddSystem(
        RosSubscriberSystem(
            torque_serializer,
            "/finger/motor_torque_commands",
            joint_qos,
            drake_ros,
        )
    )

    # Convert ROS message to 4-element vector
    torque_converter = builder.AddSystem(MultiArrayToVector(4))
    # Map 4 motor torques through tendons to 3 joint torques
    tendon_map = builder.AddSystem(
        MotorTorqueToJointTorque(jacobian_csv_path, num_q, pip_position_index)
    )
    tendon_stress = builder.AddSystem(TendonTensionToStress())
    tendon_tension_converter = builder.AddSystem(VectorToMultiArray(4))
    fingertip_force = builder.AddSystem(FingertipContactForceReporter(plant))

    builder.Connect(torque_sub.get_output_port(0), torque_converter.get_input_port(0))
    builder.Connect(torque_converter.get_output_port(0), tendon_map.get_input_port(0))
    builder.Connect(demux.get_output_port(0), tendon_map.get_input_port(1))
    builder.Connect(tendon_map.get_output_port(0), plant.get_actuation_input_port())
    builder.Connect(plant.get_contact_results_output_port(), fingertip_force.get_input_port(0))
    builder.Connect(scene_graph.get_query_output_port(), fingertip_force.get_input_port(1))

    tendon_stress_pub = builder.AddSystem(
        RosPublisherSystem(
            stress_serializer,
            "/finger/tendon_stress",
            joint_qos,
            drake_ros,
            {TriggerType.kPeriodic},
            0.01,
        )
    )
    tendon_tension_pub = builder.AddSystem(
        RosPublisherSystem(
            tension_serializer,
            "/finger/tendon_tension",
            joint_qos,
            drake_ros,
            {TriggerType.kPeriodic},
            0.01,
        )
    )
    builder.Connect(tendon_map.get_output_port(1), tendon_stress.get_input_port(0))
    builder.Connect(tendon_stress.get_output_port(0), tendon_stress_pub.get_input_port(0))
    builder.Connect(tendon_map.get_output_port(1), tendon_tension_converter.get_input_port(0))
    builder.Connect(tendon_tension_converter.get_output_port(0), tendon_tension_pub.get_input_port(0))

    fingertip_force_pub = builder.AddSystem(
        RosPublisherSystem(
            torque_serializer,
            "/finger/fingertip_contact_force",
            joint_qos,
            drake_ros,
            {TriggerType.kPeriodic},
            0.01,
        )
    )
    builder.Connect(fingertip_force.get_output_port(0), fingertip_force_pub.get_input_port(0))

def build_and_run(
    sim_duration,
    mesh_ext,
    joint_state_serializer,
    torque_serializer,
    stress_serializer,
    tension_serializer,
):
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
        MeshcatVisualizerParams(role=Role.kIllustration, prefix="visual"),
    )
    MeshcatVisualizer.AddToBuilder(
        builder, scene_graph, meshcat,
        MeshcatVisualizerParams(
            role=Role.kProximity,
            prefix="collision",
            default_color=Rgba(1.0, 0.0, 0.0, 0.5),
        ),
    )

    build_ros(
        builder,
        plant,
        scene_graph,
        joint_state_serializer,
        torque_serializer,
        stress_serializer,
        tension_serializer,
        finger_model,
    )

    # ── Build and render diagram vis ────────────────────────────────────────────────────
    diagram  = builder.Build()

    # Save to the source directory's image folder
    src_image_dir = os.path.join(
        os.environ["RDS_SRC"],
        "finger_sim",
        "image"
    )
    os.makedirs(src_image_dir, exist_ok=True)
    os.makedirs(src_image_dir, exist_ok=True)

    # Render as SVG using graphviz
    dot_source = diagram.GetGraphvizString()
    graph = graphviz.Source(dot_source)
    graph.render(
        filename="diagram",
        directory=src_image_dir,
        format="svg",
        cleanup=True,
    )
    print(f"[finger_sim] Diagram saved to {src_image_dir}/diagram.svg")

    # ── Simulate ────────────────────────────────────────────────────
    simulator = Simulator(diagram)
    simulator.Initialize()
    meshcat.SetProperty("collision", "visible", False)
    simulator.set_target_realtime_rate(1.0)
    initial_context = simulator.get_context().Clone()

    # Apply poses to dynamic demo objects
    if _DEMO_FREE_BODY_INIT_POSES:
        plant_context = diagram.GetMutableSubsystemContext(plant, initial_context)
        for body_name, X_WB in _DEMO_FREE_BODY_INIT_POSES:
            try:
                body = plant.GetBodyByName(body_name)
                plant.SetFreeBodyPose(plant_context, body, X_WB)
                print(f"[finger_sim] Placed demo body '{body_name}' at {X_WB.translation()}")
            except Exception as e:
                print(f"[finger_sim] Warning: could not place demo body '{body_name}': {e}")

    if _DEMO_JOINT_INIT_POSITIONS:
        plant_context = diagram.GetMutableSubsystemContext(plant, initial_context)
        for joint_name, position in _DEMO_JOINT_INIT_POSITIONS:
            try:
                joint = plant.GetJointByName(joint_name)
                joint.set_translation(plant_context, position)
                print(f"[finger_sim] Placed demo joint '{joint_name}' at z={position}")
            except Exception as e:
                print(f"[finger_sim] Warning: could not place demo joint '{joint_name}': {e}")

    meshcat.AddButton("Reset Simulation")
    reset_clicks = 0

    print(f"[finger_sim] sim_duration={sim_duration} s")
    print(f"[finger_sim] Simulating ...  (Ctrl-C to stop early)")
    try:
        dt = 1e-4
        while simulator.get_context().get_time() < sim_duration:
            t_now = simulator.get_context().get_time()
            try:
                simulator.AdvanceTo(min(sim_duration, t_now + dt))
                # plant_context = diagram.GetSubsystemContext(plant, simulator.get_context())
                # state = plant.GetPositionsAndVelocities(plant_context)
                # print(state)
            except Exception as e:
                plant_context = diagram.GetSubsystemContext(plant, simulator.get_context())
                state = plant.GetPositionsAndVelocities(plant_context)
                print("[finger_sim] AdvanceTo failed at t=", simulator.get_context().get_time())
                print(f"[finger_sim] Exception: {e}")
                print("[finger_sim] Plant state at failure:")
                print(state)
                if _DEMO_FREE_BODY_INIT_POSES:
                    print("[finger_sim] Demo body poses at failure:")
                    for body_name, _ in _DEMO_FREE_BODY_INIT_POSES:
                        try:
                            body = plant.GetBodyByName(body_name)
                            X_WB = plant.GetFreeBodyPose(plant_context, body)
                            print(f"[finger_sim]   {body_name}: translation={X_WB.translation()}")
                        except Exception as body_error:
                            print(f"[finger_sim]   {body_name}: could not read pose ({body_error})")
                raise

            plant_context = diagram.GetSubsystemContext(plant, simulator.get_context())
            state = plant.GetPositionsAndVelocities(plant_context)
            if not np.all(np.isfinite(state)):
                print("[finger_sim] NON-FINITE PLANT STATE at t=", simulator.get_context().get_time())
                print(state)
                break
            clicks = meshcat.GetButtonClicks("Reset Simulation")
            if clicks > reset_clicks:
                simulator.get_mutable_context().SetTimeStateAndParametersFrom(initial_context)
                simulator.Initialize()
                meshcat.SetProperty("collision", "visible", False)
                reset_clicks = clicks
                print("[finger_sim] Reset Simulation clicked: state restored")
    except KeyboardInterrupt:
        pass
    finally:
        meshcat.DeleteButton("Reset Simulation")

    print("[finger_sim] Done.")