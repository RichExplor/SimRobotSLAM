# sim_robot_slam

Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic four-wheel mecanum simulation. The
robot has a 16-channel 360-degree GPU LiDAR, an IMU, wheel odometry, and ROS
bridges for its sensors and drive command.

## Install dependencies

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-teleop-twist-keyboard
```

## Build and launch

From the ROS 2 workspace root:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/guofeng/work/code/SuperSLAM
colcon build --packages-select sim_robot_slam
source install/setup.bash
ros2 launch sim_robot_slam simulation.launch.py
```

Gazebo opens the approximately 50 × 50 m demo world and starts the robot at the
origin. Its open-top indoor layout has a connected cross-shaped main corridor
and eight rooms: a meeting room, library, workshop, server lab, kitchen, lounge,
clinic, and warehouse. The rooms contain furniture, shelving, equipment, crates,
and other obstacles for LiDAR mapping and navigation. To use another SDF world,
pass its path with `world:=/path/to/world.sdf`.

## ROS topics

| Topic | ROS type | Direction | Description |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS to Gazebo | Planar mecanum velocity command (forward, lateral, yaw) |
| `/wheel/odometry` | `nav_msgs/msg/Odometry` | Gazebo to ROS | Wheel odometry; `odom` to `base_footprint` |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo to ROS | Odometry transform |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo to ROS | Wheel joint positions and velocities |
| `/imu/data` | `sensor_msgs/msg/Imu` | Gazebo to ROS | IMU data in `imu_link` |
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` | Gazebo to ROS | 16-channel point cloud in `lidar_link` |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo to ROS | Simulation time |

The four drive joints use a lightweight anisotropic-friction approximation for
the mecanum rollers. Individual roller spin joints are not modeled, so this
setup is intended for navigation and SLAM simulation rather than detailed
roller-contact dynamics.

The IMU applies zero-mean Gaussian white noise independently to each axis:
0.05 m/s² standard deviation for linear acceleration and 0.1°/s (0.001745 rad/s)
for angular velocity. It also has small zero-mean turn-on bias (standard deviation
0.005 m/s² for acceleration and 0.01°/s for angular velocity) and slow correlated
bias drift with a 3600 s correlation time. These bias values are deliberately
small compared with the BMI088 datasheet's zero-offset limits.

The LiDAR is configured for 1024 horizontal samples per revolution, 16 vertical
channels from -15 to +15 degrees, 10 Hz, and a 0.2 to 60 m range. These are
simulation parameters and can be changed in `urdf/sim_robot.urdf.xacro`.
Gazebo publishes the packed point cloud on `/lidar/points/points`; the bridge
maps it to the ROS topic `/lidar/points`.

## Drive and inspect

### Keyboard control

With the simulation running, open a second terminal, source ROS and the
workspace, then run the keyboard teleoperation node:

```bash
source /opt/ros/jazzy/setup.bash
source /home/guofeng/work/code/SuperSLAM/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=false -r cmd_vel:=/cmd_vel
```

Keep this terminal focused while driving. The keys are:

| Key | Motion |
| --- | --- |
| `i` / `,` | Forward / backward |
| `j` / `l` | Turn left / right |
| Hold `Shift`, then `J` / `L` | Strafe left / right |
| Hold `Shift`, then `I` / `<` | Forward / backward |
| `k` | Stop |
| `q` / `z` | Increase / decrease both speed limits |
| `w` / `x` | Increase / decrease linear speed |
| `e` / `c` | Increase / decrease turn speed |

Press `Ctrl-C` to exit. The node publishes `geometry_msgs/msg/Twist` on
`/cmd_vel`, matching the Gazebo mecanum drive plugin.

### Publish velocity commands directly

In another sourced terminal, publish a forward command. `linear.x` is forward
speed in m/s; `angular.z` is turn rate in rad/s. Positive turn rate rotates
counterclockwise, and negative values reverse the direction.

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10
```

Turn in place:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" -r 10
```

Strafe left:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {y: 0.2}}" -r 10
```

To stop the robot, publish a zero command (or stop the repeated command
publisher; the Gazebo drive plugin times out stale commands):

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Inspect the sensor and odometry data with:

```bash
ros2 topic echo /wheel/odometry
ros2 topic echo /imu/data
ros2 topic echo /lidar/points --once
ros2 topic hz /lidar/points
```

In RViz2, set **Global Options → Fixed Frame** to `odom`, add a **PointCloud2**
display for `/lidar/points`, and use **Best Effort** reliability. The LiDAR
publishes in `lidar_link`; the robot state publisher provides its transform.

The GPU LiDAR uses Gazebo's Ogre2 sensor renderer. A working graphics driver and
OpenGL support are needed for point cloud generation.
