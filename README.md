# RDS Powerhouse Software
#### Authors: Cole Abbott, Heinrich Asbury, Jared Berry, Evan Bulatek, and Benji Sobeloff-Gittes

This project was associated with MECH_ENG 472: Robot Design Studio at Northwestern University.

This repository contains packages for interfacing with our electronics/hardware, controlling the finger, and running a simulation in ROS2.

#### Build Instructions
* To build without ROS2 (no visualization/simulator):
    ```
    git clone git@github.com:jarmibe7/rds-powerhouse-software.git
    cd core
    mkdir build
    cd build
    cmake ..
    cmake --build .
    ```

* To build with ROS2:
    1. Ensure you have Ubuntu 24.04 or higher.
    2. Ensure you have ROS2 Kilted. 
        * See [ROS2 Kilted Kaiju documentation](https://docs.ros.org/en/kilted/index.html).
    3. Make and build a ROS2 workspace.
    ```
    mkdir ws_<your_ws_name>
    cd ws_<your_ws_name>
    colcon build
    ```
    4. Clone this repository into the `src/` directory.
    ```
    mkdir src
    cd src
    git clone git@github.com:jarmibe7/rds-powerhouse-software.git
    ```
    5. You should now be able to build the entire project using `colcon`.
    ```
    cd /path/to/your_ws_name
    colcon build
    ```

#### Packages
* `core` - A regular C++ package for interfacing with hardware and controlling the finger.
* `finger_description` - A ROS2 C++ package for visualizing the finger in `rviz2`.
* `finger_sim` - A ROS2 C++ package for running simulations of the finger.
* `fingerlib` - A pure C++ library for housing controller code that can be used for both firmware and sim.
* `finger_control` - A ROS2 C++ package for finger controllers, wrapping controller implementations in `fingerlib`.