#!/usr/bin/env python3
"""Publish a fingertip target PointStamped on /finger/fingertip_target.

Edit the `desired_position(t)` function to specify your own trajectory.
"""

from math import sin, pi
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


def desired_position(t: float):
    """Return (x, y, z) fingertip target in meters for time t (seconds).

    Example trajectory: small circle in XY plane and sinusoidal Z motion.
    Edit this function to whatever open-loop function you want.
    """
    f = 0.5  # Hz
    amp = 1.5  # 1.5 
    conv_to_meters = 0.01
    x_center = 0.12
    y_center = 0.0
    z_center = 0.05

    x = x_center + amp*sin(2.0*pi*f*t)*conv_to_meters
    y = y_center*conv_to_meters
    z = z_center + amp*sin(4.0*pi*f*t + (3.0*pi/4.0))*conv_to_meters
    return (x, y, z)


class FingertipTrajectoryNode(Node):
    def __init__(self):
        super().__init__('fingertip_trajectory')
        self.pub = self.create_publisher(PointStamped, '/finger/fingertip_target', 10)
        self.start_time = self.get_clock().now()
        freq = 50  # Hz    
        timer_period = 1 / freq
        self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9
        pos = desired_position(t)

        msg = PointStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.point.x = float(pos[0])
        msg.point.y = float(pos[1])
        msg.point.z = float(pos[2])
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FingertipTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
