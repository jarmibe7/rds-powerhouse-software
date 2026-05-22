#!/usr/bin/env python3
"""Simple slider GUI for fingertip position targets."""

import tkinter as tk

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

AXES = ["x", "y", "z"]
P_MIN = [0.08, -0.03, 0.04]
P_MAX = [0.20, 0.03, 0.14]
P_HOME = [0.1675, 0.0033, 0.0983]


class FingertipTargetGUI(Node):

    def __init__(self):
        super().__init__("fingertip_target_gui")
        self.pub = self.create_publisher(PointStamped, "/finger/fingertip_target", 10)
        self.target_from_controller = None

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.target_state_sub = self.create_subscription(
            PointStamped,
            "/finger/gui_target_feedback",
            self._on_target_state,
            state_qos,
        )

    def _on_target_state(self, msg):
        self.target_from_controller = [msg.point.x, msg.point.y, msg.point.z]

    def publish(self, position):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.point.x = float(position[0])
        msg.point.y = float(position[1])
        msg.point.z = float(position[2])
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FingertipTargetGUI()

    root = tk.Tk()
    root.title("Fingertip Position Target")

    sliders = []
    for i, axis in enumerate(AXES):
        tk.Label(root, text=f"{axis} [m]").grid(row=i, column=0, padx=8, pady=4, sticky="w")
        slider = tk.Scale(
            root,
            from_=P_MIN[i],
            to=P_MAX[i],
            resolution=0.001,
            orient=tk.HORIZONTAL,
            length=320,
        )
        slider.set(P_HOME[i])
        slider.grid(row=i, column=1, padx=8)
        sliders.append(slider)

    def home():
        for i, slider in enumerate(sliders):
            slider.set(P_HOME[i])

    tk.Button(root, text="Home", command=home).grid(
        row=len(AXES), column=0, columnspan=2, pady=8
    )

    sliders_initialized = False

    def tick():
        nonlocal sliders_initialized

        rclpy.spin_once(node, timeout_sec=0)

        if not sliders_initialized:
            if node.target_from_controller is None:
                root.after(20, tick)
                return

            for i, slider in enumerate(sliders):
                slider.set(node.target_from_controller[i])
            sliders_initialized = True

        position = [slider.get() for slider in sliders]
        node.publish(position)
        root.after(20, tick)  # 50 Hz

    root.after(20, tick)
    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
