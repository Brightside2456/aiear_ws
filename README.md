# ROS 2 aiear_ws

A standard ROS 2 workspace for the `aiear` robot project.

## Structure

- `src/`: Contains source code for packages.
  - `aiear_robot`: Main robot package.
  - `rplidar_ros`: LiDAR driver package.
  - `uros/`: micro-ROS components.
  - `micro_ros_setup`: micro-ROS setup scripts.

## Building

To build the workspace, run:

```bash
colcon build
```

## Setup

Source the workspace:

```bash
source install/setup.bash
```
