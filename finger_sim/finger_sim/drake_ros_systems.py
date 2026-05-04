"""
Drake LeafSystems for drake_ros interface
"""
# Drake imports
from pydrake.systems.framework import LeafSystem
from pydrake.common.value import AbstractValue # type: ignore

# drake_ros imports
from sensor_msgs.msg import JointState


class FingerJointStatePublisher(LeafSystem):
    """
    Publishes a ROS JointState message for the main finger joints.

    Publishes mcp_splay, mcp_flexion, pip_flexion, dip_flexion.
    The bar joints are internal to the linkage and not published.
    """
    def __init__(self, plant, model_instance):
        super().__init__()
        self.plant = plant
        self.model_instance = model_instance

        self.active_joints = [
            "mcp_splay", "mcp_flexion", "pip_flexion", "dip_flexion"
        ]

        self.joint_name_to_pos_index = {}
        self.joint_name_to_vel_index = {}

        for name in self.active_joints:
            joint = self.plant.GetJointByName(name, self.model_instance)
            if joint.num_positions() == 1:
                self.joint_name_to_pos_index[name] = joint.position_start()
                self.joint_name_to_vel_index[name] = joint.velocity_start()

        self.active_joints = [
            name for name in self.active_joints
            if name in self.joint_name_to_pos_index
        ]
        self.pos_indices  = [self.joint_name_to_pos_index[n] for n in self.active_joints]
        self.vel_indices  = [self.joint_name_to_vel_index[n] for n in self.active_joints]
        self.zero_effort  = [0.0] * len(self.active_joints)

        num_positions  = self.plant.num_positions(self.model_instance)
        num_velocities = self.plant.num_velocities(self.model_instance)
        self.DeclareVectorInputPort("joint_positions",  num_positions)
        self.DeclareVectorInputPort("joint_velocities", num_velocities)
        self.DeclareAbstractOutputPort(
            "joint_state_msg",
            lambda: AbstractValue.Make(JointState()),
            self._calc_output,
        )

    def _calc_output(self, context, output):
        positions  = self.get_input_port(0).Eval(context)
        velocities = self.get_input_port(1).Eval(context)
        t = context.get_time()

        msg = JointState()
        msg.header.stamp.sec      = int(t)
        msg.header.stamp.nanosec  = int((t % 1.0) * 1e9)
        msg.name     = self.active_joints
        msg.position = [positions[i]  for i in self.pos_indices]
        msg.velocity = [velocities[i] for i in self.vel_indices]
        msg.effort   = self.zero_effort
        output.set_value(msg)