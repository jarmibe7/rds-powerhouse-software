#!/usr/bin/env python3
"""Simple slider GUI for finger joint torques."""

import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

JOINTS = ['mcp_splay', 'mcp_flexion', 'pip_flexion']
TAU_MIN = [-0.5, -7.0, -0.5]
TAU_MAX = [0.5, 7.0, 0.5]


class FingerTorqueGUI(Node):

    def __init__(self):
        super().__init__('finger_torque_gui')
        self.pub = self.create_publisher(Float64MultiArray, '/finger/desired_joint_torques', 10)

    def publish(self, torques):
        msg = Float64MultiArray()
        msg.data = torques
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FingerTorqueGUI()

    root = tk.Tk()
    root.title('Finger Joint Torques')

    sliders = []
    for i, name in enumerate(JOINTS):
        tk.Label(root, text=name).grid(row=i, column=0, padx=8, pady=4, sticky='w')
        s = tk.Scale(root, from_=TAU_MIN[i], to=TAU_MAX[i],
                     resolution=0.01, orient=tk.HORIZONTAL,
                     length=300)
        s.grid(row=i, column=1, padx=8)
        sliders.append(s)

    def zero_all():
        for s in sliders:
            s.set(0.0)

    tk.Button(root, text='Zero All', command=zero_all).grid(
        row=len(JOINTS), column=0, columnspan=2, pady=8
    )

    def tick():
        torques = [s.get() for s in sliders]
        node.publish(torques)
        rclpy.spin_once(node, timeout_sec=0)
        root.after(20, tick)  # 50 Hz

    root.after(20, tick)
    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
