# Finger Control
#### Authors: Cole Abbott, Heinrich Asbury, Jared Berry, Evan Bulatek, and Benji Sobeloff-Gittes

This Python ROS2 package contains functionality for wrapping `fingerlib` controller implementations in ROS2 nodes for the Drake simulation.

#### Software Structure
* src/finger_pd_control.cpp - Wrapper for simple PD controller.
* scripts/finger_target_gui.py - GUI for setting desired joint positions.