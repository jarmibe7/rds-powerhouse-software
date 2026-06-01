# fingerlib 
## Authors: Cole Abbott, Heinrich Asbury, Jared Berry, Evan Bulatek, and Benji Sobeloff-Gittes

This C++ library contains functions for controlling the figure, that can be used by both the firmware packages and the ROS2 packages. This enables rapid controller prototyping, adjustment, and testing.

## Software Structure
* `kinematics.hpp` - Functions for calculating tendon tensions and forward/inverse kinematics.
* `jacobian_lookup.hpp` - Class for containing the routing jacobian lookup table.
* `nnls.hpp` - Non-negative least squares solver for computing tendon tensions from desired joint torques.
* `constants.hpp` - Contains geometric constants and simple utility functions.

## Utilities
* `precompute_jacobian` - Generates the Jacobian transpose lookup table used by the runtime code.
* `workspace_sweep` - Samples fingertip workspace over a joint grid, writes a CSV point cloud, prints axis-aligned bounds, and can clamp a desired point to the nearest sampled workspace point.