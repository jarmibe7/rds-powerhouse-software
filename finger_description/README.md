# Finger Description
#### Authors: Cole Abbott, Heinrich Asbury, Jared Berry, Evan Bulatek, and Benji Sobeloff-Gittes

This C++ ROS2 package contains functionality visualizing the finger and loading it into `rviz2`.

#### Software Structure
* config/ - Contains visualization param and `rviz2` config files.
* launch/ - Contains all launch files.
* meshes/ - Finger meshes
* urdf/ - Finger URDF files for visualization.

#### Links
* [Onshape-to-robot](https://onshape-to-robot.readthedocs.io/en/latest/config.html)

#### TODO:
* Determine correct vals for jointMaxEffort and jointMaxVelocity in onshape_exports/config.json