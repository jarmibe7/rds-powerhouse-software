"""
Core Drake LeafSystems for simulation
"""
# Drake imports
from pydrake.systems.framework import LeafSystem, BasicVector
from pydrake.common.value import AbstractValue # type: ignore

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