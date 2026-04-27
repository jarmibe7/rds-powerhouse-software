#!/usr/bin/env python3
"""Simple slider GUI for finger joint targets."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tkinter as tk

JOINTS = ["mcp_splay", "mcp_flexion", "pip_flexion"]
Q_MIN  = [-0.175, -0.785, 0.0]
Q_MAX  = [ 0.175, 0.785, 1.57]

class FingerTargetGUI(Node):
    def __init__(self):
        super().__init__("finger_joint_target_gui")
        self.pub = self.create_publisher(JointState, "/finger/joint_targets", 10)

    def publish(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = positions
        msg.velocity = [0.0] * len(JOINTS)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FingerTargetGUI()

    root = tk.Tk()
    root.title("Finger Joint Targets")

    sliders = []
    for i, name in enumerate(JOINTS):
        tk.Label(root, text=name).grid(row=i, column=0, padx=8, pady=4, sticky="w")
        s = tk.Scale(root, from_=Q_MIN[i], to=Q_MAX[i],
                     resolution=0.01, orient=tk.HORIZONTAL,
                     length=300)
        s.grid(row=i, column=1, padx=8)
        sliders.append(s)

    def tick():
        positions = [s.get() for s in sliders]
        node.publish(positions)
        rclpy.spin_once(node, timeout_sec=0)
        root.after(1, tick)    # 50 Hz

    root.after(20, tick)
    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()