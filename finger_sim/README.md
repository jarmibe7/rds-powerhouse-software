# Finger Sim
#### Authors: Cole Abbott, Heinrich Asbury, Jared Berry, Evan Bulatek, and Benji Sobeloff-Gittes

This Python ROS2 package contains functionality for running a Drake simulation of the finger.

#### Commands
First add the RDS workspace source to your ~/.bashrc
```
export RDS_SRC="/home/jarmibe7/ws_rds/src/rds-powerhouse-software"
```

To run the simulation:
```
ros2 launch finger_sim sim.launch.xml sim_duration:=9999.0
```

To run the simulation with simulated tendon PD control:
```
ros2 launch finger_sim sim_tendon.launch.xml
```

The simulator publishes tendon stress estimates on `/finger/tendon_stress` as a `std_msgs/Float64MultiArray`. Use the following command to visualize:
```
rqt_plot /finger/tendon_stress/data[0] /finger/tendon_stress/data[1] /finger/tendon_stress/data[2] /finger/tendon_stress/data[3]

rqt_plot "/finger/tendon_tension/data[0]" "/finger/tendon_tension/data[1]" "/finger/tendon_tension/data[2]" "/finger/tendon_tension/data[3]"
```

```
ros2 run rqt_plot rqt_plot /finger/fingertip_target/point/x /finger/fingertip_target/point/y /finger/fingertip_target/point/z/z
```

#### Software Structure
* launch/ - Contains all launch files.
* finger_sim/ - Python package containing all of the simulation code.
* image/ - Contains visualized Drake diagram.