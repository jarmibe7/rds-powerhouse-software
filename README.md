# RDS Powerhouse Software
## Authors: Cole Abbott, Heinrich Asbury, Jared Berry, Evan Bulatek, and Benji Sobeloff-Gittes

This project was associated with MECH_ENG 472: Robot Design Studio at Northwestern University.

This repository contains packages for interfacing with our electronics/hardware, controlling the finger, and running a Drake simulation with ROS2. *Simulation documentation, demos, and architecture description can be found in the `finger_sim` package.*

Generative AI was used for ROS2 node skeletons and Drake API access, with manual adjustments being made for our specific needs.

## Features
* A pure C++ library containing most of the control stack allows us to use the same exact controllers in simulation and hardware. This has been very useful for testing.
* A Drake + ROS2 simulation of the Powerhouse finger, with fully simulated contacts and linkage, and Meshcat visualization.
* Fully simulated tendons that allow motor commands to actuate the finger.
* Joint torque + joint position controllers.
* GUIs to visualize and control the finger with each controller.
* An Onshape -> URDF pipeline using Blender.
    - We chose to use URDF for the robot description, since that is how it is done in public Drake code, as well as internal references.

## Packages
* `core` - A regular C++ package for interfacing with hardware and controlling the finger.
* `finger_description` - A ROS2 C++ package for visualizing the finger in `rviz2`.
* `finger_sim` - A ROS2 C++ package for running simulations of the finger.
* `fingerlib` - A pure C++ library for housing controller code that can be used for both firmware and sim.
* `finger_control` - A ROS2 C++ package for finger controllers, wrapping controller implementations in `fingerlib`.

## Simulation Build Instructions
1. Set up your computer for Ubuntu 24.04, ROS2 Kilted, etc.
    * [Instructions](https://nu-msr.github.io/hackathon/computer_setup.html)
2. Install Drake and drake_ros
    * [Instructions](https://claude.ai/share/4f9408f1-f397-481d-bf8b-3826e7e798fb)
3. Use the following command to build the drake_ros ROS2 workspace.
    ```bash
    bash --norc --noprofile << 'EOF'
    source /opt/ros/kilted/setup.bash
    export CC=gcc-13
    export CXX=g++-13
    colcon build --packages-select drake_ros \
    --cmake-args \
        -DCMAKE_PREFIX_PATH=/opt/drake \
        -Dpybind11_DIR=/opt/drake/lib/cmake/pybind11 \
        -DCMAKE_C_COMPILER=gcc-13 \
        -DCMAKE_CXX_COMPILER=g++-13 \
        -DBUILD_TESTING=OFF
    ```
4. Clone this git repository into the `src/` repository of a new ROS2 colcon workspace, and build it.
5. Follow the instructions in the README in `finger_sim` to run the simulation.

## Firmware Build Instructions
* To build without ROS2 (no visualization/simulator) use platform.io.

## References
* [drake_ros](https://github.com/RobotLocomotion/drake-ros)