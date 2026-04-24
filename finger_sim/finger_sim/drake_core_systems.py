"""
Core Drake LeafSystems for simulation
"""
# Drake imports
from pydrake.systems.framework import LeafSystem, BasicVector
from pydrake.common.value import AbstractValue # type: ignore
import numpy as np # type: ignore

from std_msgs.msg import Float64MultiArray

class MultiArrayToVector(LeafSystem):
    """Converts std_msgs/Float64MultiArray to a Drake vector output."""
    def __init__(self, size):
        super().__init__()
        self.size = size
        self.DeclareAbstractInputPort("msg", AbstractValue.Make(Float64MultiArray()))
        self.DeclareVectorOutputPort("vector", BasicVector(size), self.calc_output)

    def calc_output(self, context, output):
        msg = self.get_input_port(0).Eval(context)
        output.SetFromVector(
            list(msg.data[:self.size]) + [0.0] * (self.size - len(msg.data))
        )


class MotorTorqueToJointTorque(LeafSystem):
    """Maps motor torques -> tendon tensions -> joint torques."""

    def __init__(self):
        super().__init__()
        self.DeclareVectorInputPort("motor_torque", BasicVector(4))
        self.DeclareVectorOutputPort("joint_torque", BasicVector(3), self.calc_output)

        # Simple proxy model:
        #   tension = motor_torque / radius
        #   joint_torque = J * tension
        # J is 3x4: 3 joints, 4 tendons
        self._radius = np.array([1.0, 1.0, 1.0, 1.0], dtype=float)
        self._J = np.array([
            [1.0, 0.0, 0.0, 0.5],
            [0.0, 1.0, 0.0, 0.5],
            [0.0, 0.0, 1.0, 0.0]
        ], dtype=float)

    def calc_output(self, context, output):
        motor_tau = np.array(self.get_input_port(0).Eval(context), dtype=float)
        tension = np.divide(motor_tau, self._radius)
        joint_tau = self._J @ tension
        output.SetFromVector(joint_tau.tolist())