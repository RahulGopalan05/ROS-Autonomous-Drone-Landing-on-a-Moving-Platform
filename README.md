# Autonomous Drone Landing on a Moving Platform — How to Run

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic (gazebo11)
- Required ROS packages:
  ```bash
  sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-msgs ros-humble-tf2-ros ros-humble-rviz2
  ```

## Build

```bash
cd ~/Desktop/ROS-6S
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Run

### Launch everything (Gazebo + RViz + all nodes)

```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/ROS-6S
source install/setup.bash
ros2 launch drone_landing simulation_launch.py
```

### Launch without RViz (lighter)

```bash
ros2 launch drone_landing simulation_launch.py use_rviz:=false
```

## What Happens

1. Gazebo opens with a quadrotor drone at 5m altitude and a green helipad platform on the ground
2. After 3s — drone starts tracking the moving platform (TRACKING)
3. Once aligned for 1.5s — drone begins descent (DESCENDING)
4. At 0.35m above platform — slow final approach (LANDING)
5. At 0.08m — touchdown (LANDED)
6. Drone locks onto the platform and rides it indefinitely

Total landing time: ~22 seconds from launch.

## Monitor (open a second terminal)

```bash
source /opt/ros/humble/setup.bash

# Watch landing status
ros2 topic echo /landing/status

# Watch drone pose
ros2 topic echo /drone/pose

# Watch height above platform
ros2 topic echo /drone/height_above_platform

# List all topics
ros2 topic list

# List all nodes
ros2 node list

# View topic graph (GUI)
rqt_graph
```

## Topics Published

| Topic                        | Type                          | Description                    |
|------------------------------|-------------------------------|--------------------------------|
| /drone/pose                  | geometry_msgs/PoseStamped     | Drone position in world frame  |
| /platform/pose               | geometry_msgs/PoseStamped     | Platform position in world     |
| /relative_pose               | geometry_msgs/PoseStamped     | Platform minus drone position  |
| /drone/height_above_platform | std_msgs/Float32              | Vertical distance drone→platform|
| /landing/status              | std_msgs/String               | Current state + error metrics  |
| /tf                          | tf2_msgs/TFMessage            | TF: map→drone/base_link, map→platform |

## Nodes

| Node                | Role                                      |
|---------------------|-------------------------------------------|
| platform_mover      | Moves platform in sinusoidal XY trajectory|
| state_estimator     | Extracts poses from Gazebo, publishes TF  |
| landing_controller  | PID controller + state machine for landing|

## Troubleshooting

**Gazebo won't start / nodes can't find services:**
```bash
killall -9 gzserver gzclient
pkill -9 -f drone_landing
# Then relaunch
```

**Build errors after code changes:**
```bash
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```
