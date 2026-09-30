# sim_robot_slam

Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic four-wheel mecanum simulation. The
robot has a 16-channel 360-degree GPU LiDAR, an IMU, wheel odometry, and ROS
bridges for its sensors and drive command.

## Install dependencies

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

## Build and launch

From the ROS 2 workspace root:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/guofeng/work/code/SuperSLAM
colcon build --packages-up-to sim_robot_slam super_slam
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
| `/wheel/odometry` | `nav_msgs/msg/Odometry` | Gazebo to ROS | Wheel odometry; its TF is bridged in simulation-only mode |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo or ROS to ROS | Gazebo wheel TF in simulation-only mode; SuperSLAM odometry in Nav2 mode |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | ROS transforms | Robot links; Nav2 launch adds `map -> world` and `imu -> base_footprint` |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo to ROS | Wheel joint positions and velocities |
| `/imu/data` | `sensor_msgs/msg/Imu` | Gazebo to ROS | IMU data in `imu_link` |
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` | Gazebo to ROS | 16-channel point cloud in `lidar_link` |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo to ROS | Simulation time |

The four drive joints use a lightweight anisotropic-friction approximation for
the mecanum rollers. Individual roller spin joints are not modeled, so this
setup is intended for navigation and SLAM simulation rather than detailed
roller-contact dynamics.

Roller friction directions are expressed in `base_footprint`, so they stay fixed
relative to the chassis while the wheel joints rotate. Traction acts along
`[1, -1, 0]` for the front-left and rear-right wheels and `[1, 1, 0]` for the other
two wheels; the perpendicular direction is free to slide. Defining these
directions in the spinning wheel frame causes incorrect lateral and yaw motion,
even when the wheel odometry reports the commanded velocity.

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

In another sourced terminal, publish a forward command. Commands use the robot
body frame: `linear.x` is forward speed, `linear.y` is leftward speed (both in
m/s), and `angular.z` is turn rate in rad/s. Positive turn rate rotates
counterclockwise. The commands are not velocities in the map frame.

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

To stop the robot, publish a zero command. The Gazebo MecanumDrive plugin retains
its last command, so stopping a command publisher alone does not stop the robot:

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

In RViz2, set **Global Options → Fixed Frame** to `odom` when running only the
simulation, or `map` while SuperSLAM is publishing localization transforms.
Add a **PointCloud2** display for `/lidar/points` and use **Best Effort**
reliability. The LiDAR publishes in `lidar_link`; the robot state publisher
and the fixed IMU-to-base transform provide its path to the robot frame.

## Nav2 navigation with SuperSLAM

Nav2 uses the 2D OccupancyGrid map you provide, while SuperSLAM uses its saved
3D point-cloud map for global relocalization. The included `maps/map.yaml` and
`maps/map.pgm` cover the demo world. The Nav2 launch publishes an identity
`map -> world` transform, so the OccupancyGrid and SuperSLAM's saved global map
must use the same origin and axes. If you replace the 2D map with one built in a
different coordinate frame, calibrate that transform before navigation.
The YAML `origin` locates the image grid inside the `map` frame; map_server
already applies it, so do not copy it into the TF translation.

SuperSLAM's automatic Scan Context relocation uses
`super_slam/map_sim/loop/map.pcd` and its matching `sc_db.bin`, `sc_poses.bin`,
and `sc_manifest.txt`. The current bundle is already present in the workspace.

To launch the simulation, SuperSLAM relocation, Nav2, and the Nav2 RViz view:

```bash
source /opt/ros/jazzy/setup.bash
source /home/guofeng/work/code/SuperSLAM/install/setup.bash
ros2 launch sim_robot_slam sim_navigation.launch.py
```

The launch waits for the map, a valid `/lio/localization_state`, and the complete
`map -> base_footprint` TF chain, then starts Nav2 automatically. No initial pose
is needed for SuperSLAM's global relocalization. After the **Navigation 2** panel
is ready, use **Nav2 Goal** to send a goal. Set `autostart:=false` to leave Nav2
inactive and activate it manually from RViz. To use another OccupancyGrid, pass
its YAML file with `map:=/path/to/map.yaml`.

The default navigation view uses `config/nav2.rviz`. It shows only the map,
robot model, global/local costmaps, and planned path, with the Navigation 2 panel
and Nav2 Goal tool. The robot model is enabled by default and reads the latched
`/robot_description` topic. Sensor/debug displays and AMCL tools are omitted.

Navigation forward speed is limited to 0.30 m/s in both MPPI and the velocity
smoother. Backup recovery is limited to 0.25 m/s by the smoother; docking uses
0.15 m/s. These are linear speed limits, separate from angular rates in rad/s;
keyboard commands sent directly to `/cmd_vel` use the keyboard node's limits.
Global and local inflation radii are 0.40 m with `cost_scaling_factor: 10.0` for
a narrower, faster-decaying obstacle cost field. The radius is measured from
obstacles, not added outside the robot footprint. The padded footprint's
circumscribed radius is about 0.373 m; reassess inflation if its dimensions change.

To monitor startup, check `/lio/localization_state` for `valid: true` and inspect
the transform with:

```bash
ros2 topic echo /lio/localization_state
ros2 run tf2_ros tf2_echo map base_footprint
```

If Gazebo and SuperSLAM are already running, launch only Nav2:

```bash
ros2 launch sim_robot_slam nav2.launch.py map:=/path/to/map.yaml rviz:=true
```

This split Nav2 launch uses the same readiness gate and expects SuperSLAM to be
running. For simulation, start `simulation.launch.py` with
`bridge_config:=/path/to/SimRobotSLAM/config/bridge_nav2.yaml`. This disables
the Gazebo `odom -> base_footprint` TF so SuperSLAM can own the localization
chain without giving `base_footprint` two TF parents. The integrated
`sim_navigation.launch.py` selects this bridge configuration automatically.

Do not run keyboard teleoperation and Nav2 at the same time: both publish
velocity commands to `/cmd_vel`.

Nav2 uses `RotationShimController` around MPPI's `DiffDrive` motion model.
For a new path with heading error above 0.10 rad (about 5.7 degrees), the shim
commands rotation without translation until the error falls below 0.05 rad
(about 2.9 degrees), then MPPI drives forward and steers along the path.
`vy_max` and the velocity smoother's lateral limits are zero, so navigation
does not strafe. MPPI does not reverse during path following; the backup recovery
behavior can still move backward. At the goal, the shim rotates to the goal
orientation selected in RViz. NavFn supplies positions, so the initial heading
is derived from path geometry instead of intermediate pose orientations.
These constraints affect Nav2; keyboard control still supports the physical
Mecanum base's lateral motion. When transitioning from motion to rotation, the
velocity smoother decelerates existing forward speed before it reaches zero.

Collision Monitor excludes point cloud returns within 0.30 m of the LiDAR using
`collision_monitor.pointcloud.min_range`. The downward beams otherwise hit the
chassis inside the robot footprint and cause `FootprintApproach` to stop the
robot. This filter only affects Collision Monitor; `/lidar/points` remains
unchanged for SuperSLAM and the costmaps. Revisit this range if the robot or
LiDAR mounting geometry changes.

The GPU LiDAR uses Gazebo's Ogre2 sensor renderer. A working graphics driver and
OpenGL support are needed for point cloud generation.
