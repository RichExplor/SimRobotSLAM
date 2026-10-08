# SimRobotSLAM 仿真工程

本工程基于 Ubuntu 24.04、ROS 2 Jazzy 和 Gazebo Harmonic，提供四轮麦克纳姆轮机器人仿真。机器人包含 16 线 360° GPU 激光雷达、惯性测量单元（IMU）、轮式里程计，以及传感器和运动指令的 ROS 桥接。

## 目录

- [环境与构建](#环境与构建)
- [快速启动](#快速启动)
- [分终端启动](#分终端启动)
- [场景与地图](#场景与地图)
- [机器人模型与传感器](#机器人模型与传感器)
- [ROS 话题与手动控制](#ros-话题与手动控制)
- [Nav2 与 SuperSLAM 配置](#nav2-与-superslam-配置)
- [故障排查提示](#故障排查提示)

## 环境与构建

### 安装依赖

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

### 构建工作区

```bash
cd /home/guofeng/work/code/SuperSLAM
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to sim_robot_slam super_slam --symlink-install
source install/setup.bash
```

修改 launch、地图或配置后，重新构建相关软件包并重新加载工作区环境。每个新终端都需要 source ROS 和工作区的 setup 文件。

## 快速启动

### 室内建图

```bash
ros2 launch sim_robot_slam indoor_mapping.launch.py
```

该 launch 会启动约 50 × 50 m 的室内场景，在原点生成机器人并运行 SuperSLAM 建图。场景为顶部开放布局，包含相连的十字形主通道和八个房间，房间内设有家具、货架和设备等障碍物。建图结果保存在工作区根目录下的 `map_sim/indoor`。

### 室外建图

```bash
ros2 launch sim_robot_slam outdoor_mapping.launch.py
```

该 launch 会启动约 40 × 25 m（约 1000 m²）的室外石油化工场景并运行 SuperSLAM 建图。室外场景包含封闭的外围巡检道路，起点和终点可沿道路闭环。建图结果保存在 `map_sim/outdoor`。

### 室内导航

```bash
ros2 launch sim_robot_slam indoor_navigation.launch.py rviz:=true
```

### 室外导航

```bash
ros2 launch sim_robot_slam outdoor_navigation.launch.py rviz:=true
```

导航 launch 会启动仿真、SuperSLAM 全局重定位、Nav2 和两个 RViz 窗口：一个用于导航操作，一个用于查看 SuperSLAM 重定位。SuperSLAM 自动执行全局重定位，无需设置初始位姿。Nav2 等待地图、有效的 `/lio/localization_state` 和完整的 `map -> base_footprint` TF 链就绪后自动激活。Navigation 2 面板就绪后，使用 RViz 工具栏中的 Nav2 Goal 发送目标点。传入 `autostart:=false` 可延迟激活 Nav2。

## 分终端启动

以下流程适用于需要分别启停仿真、SuperSLAM 和 Nav2 的情况。每个终端都先执行：

```bash
cd /home/guofeng/work/code/SuperSLAM
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

地图目录参数 `map_dir` 相对于工作区根目录。启动前确认对应先验地图已经保存到 `map_sim/indoor` 或 `map_sim/outdoor`。

### 独立运行建图

建图时先在终端 1 启动仿真和传感器桥接。室内场景会由 launch 生成机器人，使用普通建图桥接配置：

```bash
ros2 launch sim_robot_slam gazebo_simulation.launch.py \
  world:=/home/guofeng/work/code/SuperSLAM/src/SimRobotSLAM/worlds/indoor_world.sdf \
  bridge_config:=/home/guofeng/work/code/SuperSLAM/src/SimRobotSLAM/config/bridge.yaml \
  spawn_robot:=true spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0
```

室外 SDF 已经包含机器人模型，设置 `spawn_robot:=false`：

```bash
ros2 launch sim_robot_slam gazebo_simulation.launch.py \
  world:=/home/guofeng/work/code/SuperSLAM/src/SimRobotSLAM/worlds/outdoor_world.sdf \
  bridge_config:=/home/guofeng/work/code/SuperSLAM/src/SimRobotSLAM/config/bridge.yaml \
  spawn_robot:=false
```

终端 2 启动 SuperSLAM 建图。该 launch 会同时启动建图节点和建图 RViz。

室内：

```bash
ros2 launch super_slam SimMapping.py use_sim_time:=true rviz:=true map_dir:=map_sim/indoor
```

室外：

```bash
ros2 launch super_slam SimMapping.py use_sim_time:=true rviz:=true map_dir:=map_sim/outdoor
```

完成采集后，在建图终端按 `Ctrl-C` 结束 launch，让 SuperSLAM 保存地图和回环数据库。

### 独立运行重定位与导航

终端 1 启动仿真时，导航模式需要使用 `bridge_nav2.yaml`。该配置关闭 Gazebo 的 `odom -> base_footprint` TF，避免它与 SuperSLAM 的定位 TF 冲突。

室内：

```bash
ros2 launch sim_robot_slam gazebo_simulation.launch.py \
  world:=/home/guofeng/work/code/SuperSLAM/src/SimRobotSLAM/worlds/indoor_world.sdf \
  bridge_config:=/home/guofeng/work/code/SuperSLAM/src/SimRobotSLAM/config/bridge_nav2.yaml \
  spawn_robot:=true spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0
```

室外：

```bash
ros2 launch sim_robot_slam gazebo_simulation.launch.py \
  world:=/home/guofeng/work/code/SuperSLAM/src/SimRobotSLAM/worlds/outdoor_world.sdf \
  bridge_config:=/home/guofeng/work/code/SuperSLAM/src/SimRobotSLAM/config/bridge_nav2.yaml \
  spawn_robot:=false
```

终端 2 启动重定位。该 launch 会同时启动 SuperSLAM 重定位节点和 RViz。

室内：

```bash
ros2 launch super_slam SimRelocation.py use_sim_time:=true rviz:=true map_dir:=map_sim/indoor
```

室外：

```bash
ros2 launch super_slam SimRelocation.py use_sim_time:=true rviz:=true map_dir:=map_sim/outdoor
```

终端 3 启动 Nav2。室内：

```bash
ros2 launch sim_robot_slam nav2.launch.py \
  map:=/home/guofeng/work/code/SuperSLAM/src/SimRobotSLAM/maps/indoor.yaml \
  use_sim_time:=true autostart:=true rviz:=true
```

室外：

```bash
ros2 launch sim_robot_slam nav2.launch.py \
  map:=/home/guofeng/work/code/SuperSLAM/src/SimRobotSLAM/maps/outdoor.yaml \
  use_sim_time:=true autostart:=true rviz:=true
```

Nav2 launch 会启动地图服务器、固定 TF、生命周期管理器、定位就绪门控和 Nav2 RViz。定位门控会在地图、SuperSLAM 定位状态和 TF 链就绪后激活 Nav2。Nav2 RViz 与 SuperSLAM 重定位 RViz 是两个独立窗口。仿真和重定位已运行时，只需启动终端 3 的 Nav2 命令。

建图与重定位应按当前任务选择其一运行，避免同时启动两个 SuperSLAM 节点。键盘遥控也不要与 Nav2 同时运行，因为二者都会向 `/cmd_vel` 发布速度指令。

## 场景与地图

### 室内场景

室内世界约为 50 × 50 m，包含可连通的十字形主通道和八个房间。场景采用顶部开放布局，放置了家具、货架、设备、箱体等物体，供激光雷达建图和导航使用。机器人默认在原点生成。

### 室外石油化工场景

室外世界文件为 `worlds/outdoor_world.sdf`，尺寸约 40 × 25 m，面积约 1000 m²。场景包含外围巡检道路、储罐、围堰、精馏塔、反应器、高架管廊、公用设施、火炬、装卸区、建筑、消防设备和车辆出入口。室外 SDF 内置机器人模型，初始位姿为 Gazebo 坐标 `(0, -9, 0)`。采集地图和执行重定位时保持该起始位姿，以便使用一致的坐标系。

### 地图文件与坐标系

工程 `maps` 目录提供二维占据栅格地图：

| 场景 | YAML | 图像 |
| --- | --- | --- |
| 室内 | `maps/indoor.yaml` | `maps/indoor.pgm` |
| 室外 | `maps/outdoor.yaml` | `maps/outdoor.pgm` |

Nav2 的二维地图和 SuperSLAM 的三维先验地图必须具有一致的坐标轴和原点。地图 YAML 中的 `origin` 由 map_server 自动应用，不要重复写入 TF 平移量。若替换地图，应确认二维地图与对应的 SuperSLAM 地图坐标系一致；坐标系不同则先标定 `map -> world` 变换。

### Gazebo Transport 分区

仿真辅助 launch `gazebo_simulation.launch.py` 会启动 Gazebo、机器人状态发布器和 ROS-Gazebo 桥接。每次启动会创建独立的 Gazebo Transport 分区，避免上次遗留的 Gazebo 服务端提供旧模型的传感器数据或处理本次生成请求。需要直接使用 Gazebo CLI 检查时，给 launch 传入 `gz_partition:=sim_robot_debug`，并在 CLI 终端设置 `GZ_PARTITION=sim_robot_debug`。ROS 话题名称不变。

## 机器人模型与传感器

### 实测尺寸与坐标

以下坐标以 `base_footprint` 为原点，原点位于底盘中心在地面的投影处，+X 朝前、+Y 朝左、+Z 朝上。传感器位置表示各自坐标系原点，传感器间旋转按单位阵处理。激光雷达外壳绘制在安装坐标系上方，外壳底部高度为 0.539 m；仿真扫描使用相同的坐标系原点，没有额外的光学原点偏移。

| 参数 | 数值（m） |
| --- | --- |
| 前后轮中心距 | 0.580 |
| 左右轮中心距 | 0.454 |
| 全向轮半径 / 宽度 | 0.125 / 0.050 |
| 轮中心位置 | X = ±0.290，Y = ±0.227，Z = 0.125 |
| 激光雷达原点 | (0.343, 0, 0.539) |
| IMU 原点 | (-0.1955, 0, 0.368) |
| IMU 坐标系下的激光雷达原点 | (0.5385, 0, 0.171) |
| IMU 坐标系下的底盘原点 | (0.1955, 0, -0.368) |

机器人外壳的准确尺寸和质量尚未提供。仿真模型沿用原有外壳比例，尺寸缩放为 0.60 × 0.40 × 0.15 m，外壳中心高度为 Z = 0.2125 m。外壳质量为 8 kg，每个车轮质量为 0.6 kg；惯量按新几何形状更新。车轮包络尺寸为 0.830 × 0.504 m。Nav2 足迹边界为 X = ±0.415 m、Y = ±0.252 m，并额外留出 0.02 m 裕量。

### 麦克纳姆轮模型

四个驱动关节用各向异性摩擦近似麦克纳姆轮滚子的运动效果。模型没有独立滚子关节，适合导航和 SLAM 仿真，不用于精确分析滚子接触动力学。

滚子摩擦方向以 `base_footprint` 表示，因此相对车身固定并随车轮转动。左前轮和右后轮沿 `[1, -1, 0]` 方向产生牵引，其余两个车轮沿 `[1, 1, 0]` 方向产生牵引，垂直方向允许滑动。若将摩擦方向设在旋转的车轮坐标系中，仿真横移和转向会异常，即使轮式里程计显示的速度正确。

### IMU 与激光雷达

IMU 每个轴均叠加均值为零的高斯白噪声：加速度标准差为 0.05 m/s²，角速度标准差为 0.1°/s（0.001745 rad/s）。此外还有较小的启动零偏（加速度标准差 0.005 m/s²、角速度标准差 0.01°/s）和相关时间为 3600 s 的缓慢相关偏置漂移。偏置量相对 BMI088 数据手册中的零偏限值较小。

激光雷达设置为每圈 1024 个水平采样点、16 个垂直通道（-15° 至 +15°）、10 Hz 扫描频率，量程为 0.2 至 60 m。参数可在 `urdf/sim_robot.urdf.xacro` 中调整。Gazebo 在 `/lidar/points/points` 发布打包点云，ROS 桥接后话题为 `/lidar/points`。

## ROS 话题与手动控制

### 主要话题

| 话题 | ROS 类型 | 数据方向 | 说明 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS 到 Gazebo | 麦克纳姆底盘速度指令：前进、横移和转向 |
| `/wheel/odometry` | `nav_msgs/msg/Odometry` | Gazebo 到 ROS | 轮式里程计；仅仿真桥接模式会桥接其 TF |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo 或 ROS | 仿真模式使用 Gazebo 车轮 TF；导航模式使用 SuperSLAM 定位 TF |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | ROS | 机器人连杆 TF；导航 launch 还添加 `map -> world` 和 `imu -> base_footprint` |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo 到 ROS | 车轮关节位置和速度 |
| `/imu/data` | `sensor_msgs/msg/Imu` | Gazebo 到 ROS | `imu_link` 坐标系下的 IMU 数据 |
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` | Gazebo 到 ROS | `lidar_link` 坐标系下的 16 线点云 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo 到 ROS | 仿真时钟 |

### 键盘遥控

仿真运行时，在另一个终端加载 ROS 和工作区环境，启动键盘节点：

```bash
source /opt/ros/jazzy/setup.bash
source /home/guofeng/work/code/SuperSLAM/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=false -r cmd_vel:=/cmd_vel
```

控制时保持键盘终端处于焦点状态：

| 按键 | 动作 |
| --- | --- |
| `i` / `,` | 前进 / 后退 |
| `j` / `l` | 左转 / 右转 |
| 按住 Shift 再按 `J` / `L` | 左移 / 右移 |
| 按住 Shift 再按 `I` / `<` | 前进 / 后退 |
| `k` | 停止 |
| `q` / `z` | 同时提高 / 降低线速度和转向速度上限 |
| `w` / `x` | 提高 / 降低线速度 |
| `e` / `c` | 提高 / 降低转向速度 |

按 `Ctrl-C` 退出。键盘节点在 `/cmd_vel` 发布 `geometry_msgs/msg/Twist`，与 Gazebo 驱动插件输入一致。

### 直接发布速度指令

速度指令使用机器人本体坐标系：`linear.x` 为前进速度，`linear.y` 为左移速度，单位均为 m/s；`angular.z` 为转向角速度，单位为 rad/s。正角速度表示逆时针旋转。

前进：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10
```

原地转向：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" -r 10
```

向左横移：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.2}}" -r 10
```

停止时发布零速度。Gazebo 驱动插件会保留最近一次指令，仅停止发布器不会让机器人停下：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

查看传感器和里程计：

```bash
ros2 topic echo /wheel/odometry
ros2 topic echo /imu/data
ros2 topic echo /lidar/points --once
ros2 topic hz /lidar/points
```

仅运行仿真时，RViz2 固定坐标系设为 `odom`；SuperSLAM 发布定位变换后设为 `map`。添加 `PointCloud2` 显示并订阅 `/lidar/points`，可靠性策略设为 `Best Effort`。机器人状态发布器和 IMU 到底盘的固定变换提供点云从 `lidar_link` 到机器人坐标系的 TF 路径。

## Nav2 与 SuperSLAM 配置

### 地图与定位坐标系

Nav2 使用二维占据栅格地图，SuperSLAM 使用保存的三维点云地图进行全局重定位。导航 launch 发布单位变换 `map -> world`，所以二维地图和 SuperSLAM 地图必须具有相同的原点和坐标轴。地图 YAML 的 `origin` 由 map_server 自动应用，不应重复写入 TF 平移量。若替换的二维地图来自其他坐标系，应先标定 `map -> world` 变换。

室内先验地图位于 `map_sim/indoor/loop/map.pcd`，室外先验地图位于 `map_sim/outdoor`。SuperSLAM 会在回环地图旁保存固定名称的 Scan Context 附属文件，因此室内、室外地图必须保存在不同目录。

### 导航专用点云

SuperSLAM 额外发布供局部代价地图使用的点云：

| 话题 | 坐标系 | 用途 |
| --- | --- | --- |
| `/lio/cloud_nav` | `base_footprint` | 去除地面点后标记障碍物 |
| `/lio/cloud_nav_clearing` | `lidar_link` | 使用当前帧射线清障，保留地面回波 |

两个话题类型为 `sensor_msgs/PointCloud2`，包含 x/y/z/intensity 字段并使用 Best Effort QoS。SuperSLAM 按相对机器人地面投影的高度过滤点云，只保留高度 0.05–1.5 m、距雷达 6 m 内的点，避免把倾斜世界中的地面误认为障碍物。原始 `/lidar/points` 继续供 SLAM 和 Collision Monitor 使用；Collision Monitor 会在机器人坐标系中单独过滤高度。

在 `super_slam/config/SimMapping.yaml` 和 `SimRelocation.yaml` 中，`lio.navigation.accumulation_frames: 3` 和 `max_age: 0.3` 保留短时点云历史。历史帧通过世界位姿对齐到当前底盘坐标系，再执行 0.05 m 体素滤波。动态环境可将累计帧数改为 1，以减少移动物体拖影。

清障只使用单帧点云和真实雷达原点，不使用累计点云生成射线。定位丢失或搜索、时钟重置、超过 max_age 的时间间隔，或相邻帧位姿跳变超过 1 m / 0.5 rad 时，历史点云会清空。点云按处理后的雷达频率发布，与可视化设置无关。

有效跟踪时，SuperSLAM 会在激光雷达帧之间根据 IMU 积分发布 `world -> imu` TF，支持 Nav2 查询中间时刻的位姿。局部点云源以消息坐标系为原点：障碍物标记以底盘中心为原点，清障以雷达原点为原点。点云有效期为 1.0 s，用于容纳处理抖动并在观测过期时停止控制。

局部体素地图覆盖世界 Z 范围 [-0.5, 1.9] m；这是体素层范围，不是地面过滤阈值。地形超出此范围时应调整体素范围。低于 `lio.navigation.min_height` 的点也会被过滤，可按低矮障碍检测需求调整。修改后需重启 SuperSLAM 和 Nav2；重启 Nav2 会创建新的局部代价地图。也可以调用服务清空当前局部地图：

```bash
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap '{}'
```

### 速度、足迹与控制器

MPPI 和速度平滑器的最大前进速度为 0.80 m/s。航向对准最大角速度为 0.60 rad/s；MPPI、原地旋转恢复行为和速度平滑器的转向角速度上限均为 ±0.60 rad/s。速度平滑器限制后退恢复速度为 0.25 m/s，停靠行为速度为 0.15 m/s。线速度单位为 m/s，角速度单位为 rad/s；键盘直接向 `/cmd_vel` 发布的速度仍由键盘节点限速。

全局和局部膨胀半径均为 0.55 m，`cost_scaling_factor: 10.0` 使障碍物代价更快衰减。膨胀半径从障碍物边缘开始计算，不会额外加在机器人足迹之外。当前带裕量的足迹外接圆半径约为 0.513 m；修改机器人尺寸后应重新评估膨胀参数。

Nav2 使用 `RotationShimController` 包装 MPPI 的 `DiffDrive` 运动模型。新路径初始航向误差大于 0.10 rad（约 5.7°）时，先原地转向；误差低于 0.05 rad（约 2.9°）后，MPPI 才开始前进并沿路径转向。MPPI 路径跟踪不倒车，但后退恢复行为仍可倒车。`vy_max` 和速度平滑器的横向速度上限均为零，因此 Nav2 不会横移；键盘控制仍可使用麦克纳姆底盘横移。到达目标点时，机器人会转向 RViz 中设置的目标朝向。NavFn 只提供路径位置，初始航向根据路径几何计算。从前进切换为原地旋转时，速度平滑器先降低前进速度再将其归零。

默认导航 RViz 配置为 `config/nav2.rviz`，显示地图、机器人模型、全局和局部代价地图及规划路径，并提供 Navigation 2 面板和 Nav2 Goal 工具。机器人模型默认启用，从 latched 话题 `/robot_description` 读取。默认视图不包含传感器调试显示和 AMCL 工具。

### SuperSLAM 外参与旧地图兼容性

SuperSLAM 的 `SimMapping.yaml` 和 `SimRelocation.yaml` 使用以下外参：

- `lio.extrinsic.lidar_imu` 平移为 `[0.5385, 0.0, 0.171]`，旋转为单位阵。
- `lio.extrinsic.odom_robo` 为 `[-0.1955, 0.0, 0.368, 0.0, 0.0, 0.0]`。
- `lio.loop.sc_lidar_height` 为 `0.539`。

全局重定位前，应使用这些参数重新建立 SuperSLAM 先验地图和 Scan Context 数据库。地图档案包含传感器外参和高度信息，旧数据库不能直接复用。只有现有二维地图与新点云地图的原点和坐标轴一致时才能继续使用；否则应标定 `map -> world` 变换或重新生成二维栅格地图。

Collision Monitor 的 `pointcloud.min_range` 为 0.20 m，与激光雷达最小测距一致。按当前简化外壳和安装位置，向下的激光束从机器人自身碰撞几何上方通过，因此不再使用旧的 0.30 m 自身回波裁剪距离。此设置只影响 Collision Monitor，原始 `/lidar/points` 保持不变。若外壳或雷达安装位置发生变化，应重新评估自身回波过滤。

## 故障排查提示

### 检查重定位与 TF

查看 SuperSLAM 定位状态，并检查地图到机器人底盘的 TF：

```bash
ros2 topic echo /lio/localization_state
ros2 run tf2_ros tf2_echo map base_footprint
```

Nav2 未激活时，先确认 `/lio/localization_state` 有效、地图已发布，并且 `map -> base_footprint` TF 可查询。Nav2 定位就绪门控会在这些条件满足后激活导航。

### RViz 中没有地图或机器人模型

确认地图 YAML 和对应 PGM 文件路径有效，地图坐标系与 SuperSLAM 先验地图一致。地图文件变化后重新构建 `sim_robot_slam` 并重新 source 工作区。默认导航 RViz 从 `/robot_description` 获取机器人模型，检查机器人状态发布器是否运行以及该话题是否存在。

### 点云没有显示

GPU 激光雷达使用 Gazebo Ogre2 传感器渲染器，生成点云需要可用的图形驱动和 OpenGL 支持。检查 `/lidar/points` 是否持续发布；在 RViz 中将固定坐标系设为 `map` 或 `odom`，并将 PointCloud2 可靠性策略设为 Best Effort。
