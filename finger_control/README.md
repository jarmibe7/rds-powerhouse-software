# Finger Control
#### Authors: Cole Abbott, Heinrich Asbury, Jared Berry, Evan Bulatek, and Benji Sobeloff-Gittes

This Python ROS2 package contains functionality for wrapping `fingerlib` controller implementations in ROS2 nodes for the Drake simulation.

#### Commands
To command a raw torque:
```
ros2 topic pub /finger/torque_commands std_msgs/msg/Float64MultiArray   "data: [0.0, 0.0, 5.0]" --rate 1000
```

#### Software Structure
* src/finger_pd_control.cpp - Wrapper for simple PD controller.
* scripts/finger_target_gui.py - GUI for setting desired joint positions.