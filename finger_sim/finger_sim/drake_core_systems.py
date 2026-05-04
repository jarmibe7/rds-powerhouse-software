"""
Core Drake LeafSystems for simulation
"""
import csv

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


class VectorToMultiArray(LeafSystem):
    """Converts a Drake vector to std_msgs/Float64MultiArray."""

    def __init__(self, size):
        super().__init__()
        self.size = int(size)
        self.DeclareVectorInputPort("vector", BasicVector(self.size))
        self.DeclareAbstractOutputPort(
            "msg",
            lambda: AbstractValue.Make(Float64MultiArray()),
            self.calc_output,
        )

    def calc_output(self, context, output):
        vec = np.array(self.get_input_port(0).Eval(context), dtype=float)
        msg = Float64MultiArray()
        msg.data = vec.tolist()
        output.set_value(msg)


class MotorTorqueToJointTorque(LeafSystem):
    """Maps motor torques -> tendon tensions -> joint torques."""

    def __init__(self, jacobian_csv_path, num_positions, pip_position_index):
        super().__init__()
        self.DeclareVectorInputPort("motor_torque", BasicVector(4))
        self.DeclareVectorInputPort("joint_positions", BasicVector(num_positions))
        self.DeclareVectorOutputPort("joint_torque", BasicVector(3), self.calc_output)
        self.DeclareVectorOutputPort("tendon_tension", BasicVector(4), self.calc_tendon_tension)

        self._pip_position_index = int(pip_position_index)
        self._min_degree, self._max_degree, self._jacobians = self._load_jacobians(jacobian_csv_path)

        # Simple proxy model:
        #   tension = motor_torque / radius
        #   joint_torque = J(q_pip) * tension
        self._radius = np.array([1.0, 1.0, 1.0, 1.0], dtype=float)

    def _calc_tension_and_joint_torque(self, context):
        motor_tau = np.array(self.get_input_port(0).Eval(context), dtype=float)
        q = np.array(self.get_input_port(1).Eval(context), dtype=float)

        pip_deg = int(round(np.rad2deg(q[self._pip_position_index])))
        pip_deg = int(np.clip(pip_deg, self._min_degree, self._max_degree))
        J = self._jacobians[pip_deg - self._min_degree]

        tension = np.divide(motor_tau, self._radius)
        joint_tau = J @ tension
        return tension, joint_tau

    def _load_jacobians(self, csv_path):
        """Loads Jacobian matrices from the CSV file."""
        rows = []
        with open(csv_path, newline="", encoding="utf-8") as csvfile:
            reader = csv.reader(csvfile)
            header = next(reader, None)
            if header is None:
                raise RuntimeError(f"Jacobian CSV is empty: {csv_path}")

            for row in reader:
                if not row:
                    continue
                if len(row) != 13:
                    raise RuntimeError("Malformed Jacobian CSV row: expected 13 columns")

                values = [float(v) for v in row]
                degree = int(round(values[0]))
                jt = np.array(values[1:], dtype=float).reshape(3, 4)
                rows.append((degree, jt))

        if not rows:
            raise RuntimeError(f"Jacobian CSV has no data rows: {csv_path}")

        rows.sort(key=lambda item: item[0])
        min_degree = rows[0][0]
        max_degree = rows[-1][0]
        jacobians = [np.zeros((3, 4), dtype=float) for _ in range(max_degree - min_degree + 1)]

        for degree, jt in rows:
            jacobians[degree - min_degree] = jt

        return min_degree, max_degree, jacobians

    def calc_output(self, context, output):
        _, joint_tau = self._calc_tension_and_joint_torque(context)
        output.SetFromVector(joint_tau.tolist())

    def calc_tendon_tension(self, context, output):
        tension, _ = self._calc_tension_and_joint_torque(context)
        output.SetFromVector(tension.tolist())


class TendonTensionToStress(LeafSystem):
    """Converts tendon tensions to nominal tendon stresses."""

    def __init__(self, tendon_area=1.0e-6):
        super().__init__()
        if tendon_area <= 0.0:
            raise ValueError("tendon_area must be positive")

        self._tendon_area = float(tendon_area)
        self.DeclareVectorInputPort("tendon_tension", BasicVector(4))
        self.DeclareAbstractOutputPort(
            "tendon_stress",
            lambda: AbstractValue.Make(Float64MultiArray()),
            self.calc_output,
        )

    def calc_output(self, context, output):
        tension = np.array(self.get_input_port(0).Eval(context), dtype=float)
        stress = tension / self._tendon_area
        msg = Float64MultiArray()
        msg.data = stress.tolist()
        output.set_value(msg)