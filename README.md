# RDS Powerhouse Software
#### Authors: Cole Abbott, Heinrich Asbury, Jared Berry, Evan Bulatek, and Benji Sobeloff-Gittes

This project was associated with MECH_ENG 472: Robot Design Studio at Northwestern University.

This repository contains packages for interfacing with our electronics/hardware, controlling the finger, and running a Drake simulation with ROS2.

Generative AI was used for ROS2 node skeletons and Drake API access, with manual adjustments being made for our specific needs.

#### Design Decisions/Features
* A Drake + ROS2 simulation of the Powerhouse finger, with fully simulated contacts and linkage, and Meshcat visualization.
* A torque controller for directly controlling joint torques.
* A simple PD position controller that doesn't work very well.
* GUIs to visualize and control the finger with each controller.
* We chose to stick with URDF for the robot description for now, since that is how we have seen it be done in public Drake code, as well as internal references.
    - If this ends up being an issue, we will switch to SDF.
    - Inertial geometries are interpreted from collision geometries.
* To see changes from Onshape assembly to simulated meshes, look at the README in finger_description.
* To run the simulation, you'll need to go through an extensive and painful Drake and drake_ros install with ROS2 Kilted as detailed above.

For your convenience, a video of the simulation in action can be seen below. The video uses a simple torque controller for controlling the joint torques.

[Drake demo video link](https://drive.google.com/file/d/1iBwN4o5fzTkN7qK0gGYoEYKLPsBD8gBn/view?usp=sharing)

#### Packages
* `core` - A regular C++ package for interfacing with hardware and controlling the finger.
* `finger_description` - A ROS2 C++ package for visualizing the finger in `rviz2`.
* `finger_sim` - A ROS2 C++ package for running simulations of the finger.
* `fingerlib` - A pure C++ library for housing controller code that can be used for both firmware and sim.
* `finger_control` - A ROS2 C++ package for finger controllers, wrapping controller implementations in `fingerlib`.

#### Simulation Build Instructions
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
5. Follow the instructions in the README in finger_sim to run the simulation.

#### Firmware Build Instructions
* To build without ROS2 (no visualization/simulator) use platform.io.

#### References
* [drake_ros](https://github.com/RobotLocomotion/drake-ros)