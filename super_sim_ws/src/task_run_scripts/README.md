# task_run_scripts

ROS1 utility package for task execution and flight-control support.

## Contents

- `src/realman_monitor.cpp`: a displacement monitor node that watches MAVROS odometry and can switch the flight mode to `STABILIZED` when motion exceeds a threshold.
- `scripts/`: helper Python scripts for odom forwarding, logging, and QGC / RViz support.
- `launch/`: launch files for PX4, ArduPilot, and combined startup flows — organized per radar type in `odin/` and `fastlivo2/` (decoupled, each radar uses its own files).
- `config/`: runtime parameters for the monitor node.

## Typical usage

```bash
catkin_make
source devel/setup.bash
roslaunch task_run_scripts realman.launch
```

The package expects a ROS1 environment with `mavros`, `geometry_msgs`, `tf`, `tf2`, `roscpp`, and `rospy` available.
