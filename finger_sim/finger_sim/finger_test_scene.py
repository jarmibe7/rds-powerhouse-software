"""
finger_test_scene.py
--------------------
Minimal Drake + drake_ros test scene for the RDS powerhouse finger.

What it does
~~~~~~~~~~~~
1. Initialises drake_ros and a Drake DiagramBuilder.
2. Loads the finger xacro, processing it with the mesh_ext parameter.
3. Adds a flat ground plane.
4. Wires up Meshcat for in-browser 3D visualisation.
5. Publishes tf2 transforms over ROS 2 via drake_ros.
6. Runs the simulator at real-time rate for a configurable duration.

Run
~~~
  ros2 run finger_sim finger_test_scene

Override sim duration or mesh format:
  ros2 run finger_sim finger_test_scene --ros-args -p sim_duration:=30.0 -p mesh_ext:=glb

Or via the launch file:
  ros2 launch finger_sim finger_test_scene.launch.xml sim_duration:=9999.0 mesh_ext:=glb

Visualise
~~~~~~~~~
  Open the Meshcat URL printed to the console in your browser.
"""

import sys
import os
import tempfile

import rclpy
from rclpy.node import Node

# Drake core
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

# drake_ros
from drake_ros.core import RosInterfaceSystem, init, shutdown
from drake_ros.tf2 import SceneTfBroadcasterSystem, SceneTfBroadcasterParams
from pydrake.systems.framework import TriggerType


# ── helpers ───────────────────────────────────────────────────────────────────

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


# ── main scene builder ────────────────────────────────────────────────────────

def build_and_run(sim_duration: float, mesh_ext: str) -> None:
    builder = DiagramBuilder()

    # ── MultibodyPlant + SceneGraph ───────────────────────────────────────────
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)

    parser = Parser(plant)
    parser.package_map().PopulateFromEnvironment("AMENT_PREFIX_PATH")

    # ── Load and process xacro ────────────────────────────────────────────────
    xacro_path = resolve_package_path(
        "finger_description",
        "urdf/powerhouse_finger.urdf.xacro",
    )
    urdf_string = process_xacro(xacro_path, mesh_ext)

    # Drake's AddModelsFromString needs a base URL so package:// URIs resolve
    # correctly relative to the finger_description share directory.
    package_dir = resolve_package_path("finger_description", "")
    models = parser.AddModelsFromString(urdf_string, "urdf")
    print(f"Loaded {len(models)} models")
    finger_model = models[0]

    # Weld base_link to world so the finger stays fixed in space
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("base_link", finger_model),
        RigidTransform(),
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

    # ── Meshcat visualiser ────────────────────────────────────────────────────
    meshcat = Meshcat(port=7000)
    MeshcatVisualizer.AddToBuilder(
        builder, scene_graph, meshcat,
        MeshcatVisualizerParams(),
    )
    print(f"[finger_sim] Meshcat running at: {meshcat.web_url()}")

    # ── drake_ros: tf2 broadcaster ────────────────────────────────────────────
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

    # ── Build & simulate ──────────────────────────────────────────────────────
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


# ── ROS 2 node wrapper ────────────────────────────────────────────────────────

class FingerSimNode(Node):
    def __init__(self):
        super().__init__("finger_sim")
        self.declare_parameter("sim_duration", 10.0)
        self.declare_parameter("mesh_ext", "gltf")   # default to gltf for Drake

    @property
    def sim_duration(self) -> float:
        return self.get_parameter("sim_duration").get_parameter_value().double_value

    @property
    def mesh_ext(self) -> str:
        return self.get_parameter("mesh_ext").get_parameter_value().string_value


def main(args=None):
    init(sys.argv if args is None else args)
    rclpy.init(args=args)

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