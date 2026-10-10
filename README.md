# SimRobotSLAM 仿真工程

基于 Ubuntu 24.04、ROS 2 Jazzy 和 Gazebo Harmonic，提供室内、室外两种场景及四轮麦克纳姆机器人仿真。使用 SuperSLAM 建图与全局重定位，使用 Nav2 导航，并支持沿固定路线执行巡检任务。

## 目录

- [1. 环境准备](#1-环境准备)
- [2. 场景与地图](#2-场景与地图)
- [3. 建图与手动控制](#3-建图与手动控制)
- [4. 定位与导航](#4-定位与导航)
- [5. 固定路线巡检](#5-固定路线巡检)
- [6. 巡检点任务](#6-巡检点任务)
- [7. 分开启动各模块](#7-分开启动各模块)
- [8. 模型与关键配置](#8-模型与关键配置)
- [9. 常见问题](#9-常见问题)

## 1. 环境准备

### 安装依赖

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

工作区需包含 `sim_robot_slam` 和 `super_slam` 软件包，SuperSLAM 的编译依赖按其工程说明安装。

### 编译

```bash
cd /home/guofeng/work/code/SuperSLAM
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to sim_robot_slam super_slam --symlink-install
```

仅修改本工程时，可使用 `colcon build --packages-select sim_robot_slam`。修改源码、启动文件或包内地图、配置后，重新编译并重启对应模块。

### 每个终端的公共准备

下文命令均从工作区根目录执行。每个新终端先运行：

```bash
cd /home/guofeng/work/code/SuperSLAM
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

`map_sim/...` 是相对于当前目录的路径，也可以通过 `map_dir` 传入绝对路径。

## 2. 场景与地图

| 项目 | 室内 | 室外 |
| --- | --- | --- |
| 场景文件 | `worlds/indoor_world.sdf` | `worlds/outdoor_world.sdf` |
| 规模 | 约 50 × 50 m | 约 40 × 25 m，面积约 1000 m² |
| 布局 | 十字通道、八个房间、家具与设备 | 石油化工设施、储罐、管廊与外围闭环道路 |
| Gazebo 初始位置 | `(0, 0, 0)` | `(0, -9, 0)` |
| SuperSLAM 地图目录 | `map_sim/indoor` | `map_sim/outdoor` |
| Nav2 二维地图 | `maps/indoor.yaml`、`indoor.pgm` | `maps/outdoor.yaml`、`outdoor.pgm` |
| 固定路线 | `routes/indoor_route.geojson` | `routes/outdoor_route.geojson` |
| 巡检任务 | `config/indoor_patrol_tasks.yaml` | `config/outdoor_patrol_tasks.yaml` |

SuperSLAM 保存三维先验地图及回环数据库；Nav2 使用另行提供的二维 PGM 和 YAML 文件。室内、室外先验地图应分别保存，保留各目录中的回环地图和附属文件。

二维地图、三维地图和路线图必须使用一致的原点与坐标轴。当前导航发布单位静态变换 `map → world`；地图 YAML 的 `origin` 由地图服务器处理，不要重复加入 TF。若地图坐标系不同，需先标定并调整该变换。

## 3. 建图与手动控制

### 启动建图

按场景选择一条命令，均会启动仿真、SuperSLAM 建图和 RViz：

```bash
# 室内
ros2 launch sim_robot_slam indoor_mapping.launch.py

# 室外
ros2 launch sim_robot_slam outdoor_mapping.launch.py
```

建图默认开启 RViz 和仿真时钟。可追加 `rviz:=false` 关闭 RViz，或用 `map_dir:=/绝对路径/地图目录` 修改保存目录。

在另一个终端启动键盘控制，移动机器人采集场景。完成后先停车、退出键盘节点，再在建图终端按 `Ctrl-C`，等待地图和回环数据库保存完成。

### 键盘控制

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=false -r cmd_vel:=/cmd_vel
```

操作时保持键盘终端有焦点：

| 按键 | 动作 |
| --- | --- |
| `i` / `,` | 前进 / 后退 |
| `j` / `l` | 左转 / 右转 |
| `Shift + J` / `Shift + L` | 左移 / 右移 |
| `k` | 停止 |
| `q` / `z` | 同时增大 / 减小线速度和角速度 |
| `w` / `x` | 增大 / 减小线速度 |
| `e` / `c` | 增大 / 减小角速度 |

导航开始前，按 `k` 停车并用 `Ctrl-C` 退出键盘节点，避免多个节点同时向 `/cmd_vel` 发送指令。

## 4. 定位与导航

### 一键启动

确认对应的三维先验地图、回环数据库和二维地图已准备好，再选择场景：

```bash
# 室内
ros2 launch sim_robot_slam indoor_navigation.launch.py rviz:=true

# 室外
ros2 launch sim_robot_slam outdoor_navigation.launch.py rviz:=true
```

两种启动方式均包含仿真、SuperSLAM 全局重定位、Nav2、巡检节点，以及导航和重定位两个 RViz 窗口。

### 发送导航目标

1. 等待 SuperSLAM 全局重定位成功，无需手动设置初始位姿。
2. Nav2 在二维地图、有效的 `/lio/localization_state` 和 `map → base_footprint` TF 就绪后自动激活。
3. 在导航 RViz 选择 **Nav2 Goal**，在可通行区域按住鼠标左键，拖动设置朝向后释放。
4. 机器人开始执行单目标导航；固定路线巡检按第 5 节操作。

如需先遥控辅助定位，完成后停车并退出键盘节点，再发送导航目标。需要手动激活 Nav2 时，可传入 `autostart:=false`。

### 常用启动参数

| 参数 | 室内默认值 | 室外默认值 | 用途 |
| --- | --- | --- | --- |
| `map` | 包内 `maps/indoor.yaml` | 包内 `maps/outdoor.yaml` | Nav2 二维地图 |
| `map_dir` | `map_sim/indoor` | `map_sim/outdoor` | SuperSLAM 先验地图目录 |
| `route_graph` | 包内 `routes/indoor_route.geojson` | 包内 `routes/outdoor_route.geojson` | 固定路线图 |
| `route_start_id` | `1` | `0` | 巡检起点 |
| `route_goal_id` | `10` | `0` | 巡检终点 |
| `route_via_ids` | 空字符串 | `9` | 按顺序经过的中间节点，逗号分隔 |
| `task_config` | 包内室内任务 YAML | 包内室外任务 YAML | 巡检点任务；空字符串表示不执行任务 |
| `rviz` | `true` | `true` | 启动导航和重定位 RViz |
| `autostart` | `true` | `true` | 就绪后自动激活 Nav2 |
| `use_sim_time` | `true` | `true` | 使用 Gazebo 仿真时钟 |

## 5. 固定路线巡检

### 开始与取消

导航启动文件会加载路线图，但不会自动开始巡检。定位和 Nav2 就绪后，在另一个终端调用：

```bash
# 开始一次巡检
ros2 service call /route_patrol/start std_srvs/srv/Trigger '{}'

# 取消当前巡检
ros2 service call /route_patrol/cancel std_srvs/srv/Trigger '{}'
```

机器人距路线起点超过 0.25 m 时，先导航到起点。之后按图边逐段行驶，到点停稳并完成任务后再执行下一段；每段开始先对齐方向，导航横向速度为零。巡检完成后停止，再次调用开始服务可重新执行。

### 室内路线

路线图为单向链 `0 → 1 → … → 10`，默认从节点 1 出发，到节点 10 完成后结束，没有回程边。

```bash
ros2 launch sim_robot_slam indoor_navigation.launch.py \
  rviz:=true route_start_id:=1 route_goal_id:=10
```

### 室外闭环

路线图已包含 `9 → 0` 回程边，默认行驶 `0 → 1 → … → 9 → 0`，返回节点 0 后结束一圈。

```bash
ros2 launch sim_robot_slam outdoor_navigation.launch.py \
  rviz:=true route_start_id:=0 route_goal_id:=0 route_via_ids:=9
```

起终点同为 0 时，必须指定一个不同的中间节点。巡检节点先规划 `0 → 9` 和 `9 → 0`，两段均成功后才开始行驶。当前图只有单向闭环，指定中间节点 9 即可经过全部节点；以后若加入捷径，可设置 `route_via_ids:="1,2,3,4,5,6,7,8,9"` 明确访问顺序。

### 自定义路线与显示

通过路线绘制工具创建并保存 GeoJSON：

```bash
ros2 launch nav2_rviz_plugins route_tool.launch.py \
  yaml_filename:=/绝对路径/地图.yaml
```

路线图应与地图坐标系一致，拐弯处设置节点。启动导航时加载保存的文件：

```bash
ros2 launch sim_robot_slam indoor_navigation.launch.py \
  route_graph:=/绝对路径/路线.geojson \
  route_start_id:=1 route_goal_id:=10
```

RViz 中绿色 **Patrol Route** 显示 `/route_patrol/path` 的完整巡检路线，蓝色 **Global Path** 显示 `/plan` 的最新导航路径。完整路线在全部规划成功后发布，结束或取消后保留，下一次有效启动时清空旧路线。

沿用旧 RViz 配置时，新增 **Path** 显示项，订阅 `/route_patrol/path`，将持久性（Durability Policy）设为 **Transient Local**、可靠性设为 **Reliable**。这样 RViz 晚启动也能获取完整路线。

## 6. 巡检点任务

任务在室内、室外各自的 `config/*_patrol_tasks.yaml` 中配置，节点编号对应 GeoJSON 的 `properties.id`。同一节点的任务按列表顺序执行，未配置任务的节点停稳后继续行驶。

默认任务为：节点 1 逆时针旋转一周并等待 2 秒，节点 2 等待 5 秒，节点 5 顺时针旋转一周。

```yaml
failure_policy: stop
points:
  1:
    - type: spin
      angle_deg: 360.0
      timeout_sec: 30.0
    - type: wait
      duration_sec: 2.0
  2:
    - type: wait
      duration_sec: 5.0
  5:
    - type: spin
      angle_deg: -360.0
      timeout_sec: 30.0
```

| 任务 | 参数 | 说明 |
| --- | --- | --- |
| `spin` | `angle_deg` | 旋转角度，单位度；正数逆时针，负数顺时针 |
| `spin` | `timeout_sec` | 超时时间，默认 30 秒；旋转速度上限为 0.6 rad/s |
| `wait` | `duration_sec` | 停留时间，单位秒 |
| `wait` | `timeout_sec` | 可选，默认停留时间加 10 秒，必须大于停留时间 |

可指定自定义文件，或关闭任务：

```bash
ros2 launch sim_robot_slam outdoor_navigation.launch.py \
  task_config:=/绝对路径/巡检任务.yaml

ros2 launch sim_robot_slam outdoor_navigation.launch.py task_config:=""
```

每次开始巡检会重新读取任务文件。自定义绝对路径文件可直接修改，下一次巡检生效；修改包内默认配置后重新编译。

执行规则：

- 起点、终点均执行对应任务；闭环节点 0 若配置任务，出发和返回时各执行一次。
- 任务使用仿真时钟，暂停 Gazebo 时暂停计时；收到首帧 `/clock` 后才接受开始请求。
- 停车确认使用 `/wheel/odometry`：线速度不超过 0.02 m/s、角速度不超过 0.02 rad/s，连续保持 0.3 秒，数据不超过 1 秒。默认 5 秒内未停稳则结束巡检。
- 当前仅支持 `failure_policy: stop`。任务失败或超时后停止巡检，不自动重试；取消会等待当前导航或任务结束。
- 配置错误会拒绝开始；不在本次路线中的任务会提示并忽略。

## 7. 分开启动各模块

### Web 定位、路线配置与任务接入

WebVizeSLAM 新增独立网关；编译本包后，在已 source ROS 和工作区的终端运行：

```bash
ros2 run sim_robot_slam web_gateway --ros-args \
  -p use_sim_time:=true -p map_id:="<Web 地图 UUID>" -p version_id:="<版本 UUID>" \
  -p task_data_dir:="$HOME/.local/share/super_slam/web_tasks"
```

网关只监听 `127.0.0.1:8765`，提供定位、TF、地图摘要、巡检状态和不触发运动的
路线规划预览。网页服务设置 `ROBOT_GATEWAY_URL=http://127.0.0.1:8765`。
Nav2 必须加载与 Web 归档完全一致的 PGM/YAML；仅填 UUID 不会绕过栅格校验。
网关使用 `/route_server/set_route_graph` 服务连接 Route Server。状态 JSON 的
`preview_missing` / `preview_reason` 和 `task_missing` / `task_reason` 分别说明
规划与任务接口缺失原因；栅格预检通过仍需这些接口就绪才可下发。
完整配置和阶段范围见 [Web 接入说明](../WebVizeSLAM/system_web/README.md#ros-2-接入与实施进度)。

巡检节点新增 `/route_patrol/status`（JSON 字符串）、`/route_patrol/active_path`
和 `/route_patrol/preview_lock`（SetBool）。预览持有锁时不能开始或取消巡检；
网关恢复原 `graph_filepath` 后释放锁。恢复失败时保持锁定，需要重启导航和巡检。
网页已接通任务下发、到点朝向与顺序等待/旋转、暂停/继续/取消、持久化任务历史。
`task_data_dir` 默认空，仅遥测/预览；指定持久目录才启用任务控制，网关与执行器共用账本。
新增 `/route_patrol/start_web`、`pause`、`resume`、`finish_web` 为 Trigger，动态
`web_mission` 参数仅在空闲执行器被预览锁保留时配置，由网关维护图加载/恢复顺序。
传统 `/route_patrol/start` 与 YAML 点位任务入口保持兼容。

Web 任务先校验新鲜定位/TF、实际地图摘要、全局代价地图和机器人轮廓，再运动。
暂停恢复只执行剩余等待、剩余旋转或剩余路径；轮速确认停稳前不宣告暂停/取消。
定位、地图或代价地图过期会取消动作；超过停车超时保持锁定，恢复原路线图后才空闲。
执行器心跳丢失或节点重启后不重放动作，网关按账本所属 Goal UUID 取消原动作并确认停车；执行器状态恢复后再恢复原图。
状态与安全检查使用单调时钟，点位时长和量测年龄使用 ROS 时间；暂停仿真后持续
断流会触发 Web 任务停止。`web_freshness_sec` 默认 1 秒、`costmap_freshness_sec`
默认 3 秒、`resume_tolerance` 默认 0.5 m。

网关还接入雷达、IMU、轮速、电池和机器人描述；传感器话题可用 `lidar_topic`、
`imu_topic`、`odom_topic`、`battery_topic` 覆盖。`robot_id`、`robot_model`、`robot_serial`
默认空，仅用于实际设备信息，不生成虚构编号。
`/navigation-settings` 的 GET/POST 接入当前激活的 Nav2 控制器和平滑器速度参数；
空闲且确认停稳后锁定执行器、下发、回读，失败时恢复原值；重启导航后恢复 YAML。
当前导航模型不允许横向移动，横向速度为 0。具体字段和手动验收命令见上述 Web 接入说明。
新增测试文件已按用户要求删除；仍需 Gazebo/RViz 与电脑浏览器实际运动联调验收。

用于单独启停仿真、SuperSLAM 和 Nav2。每个终端先完成第 1 节的公共准备，再设置本工程安装目录：

```bash
SIM_SHARE="$(ros2 pkg prefix sim_robot_slam)/share/sim_robot_slam"
```

### 终端 1：仿真

导航使用 `bridge_nav2.yaml`，关闭 Gazebo 的底盘定位 TF，避免与 SuperSLAM 冲突。

```bash
# 室内：由启动文件生成机器人
ros2 launch sim_robot_slam gazebo_simulation.launch.py \
  world:="$SIM_SHARE/worlds/indoor_world.sdf" \
  bridge_config:="$SIM_SHARE/config/bridge_nav2.yaml" spawn_robot:=true

# 室外：场景文件已内置机器人
ros2 launch sim_robot_slam gazebo_simulation.launch.py \
  world:="$SIM_SHARE/worlds/outdoor_world.sdf" \
  bridge_config:="$SIM_SHARE/config/bridge_nav2.yaml" spawn_robot:=false
```

单独建图时，将桥接配置改为 `bridge.yaml`，终端 2 改用建图启动命令，并跳过终端 3、4。

### 终端 2：SuperSLAM 与 RViz

导航时启动重定位，按场景选择地图目录：

```bash
# 室内
ros2 launch super_slam SimRelocation.py \
  use_sim_time:=true rviz:=true map_dir:=map_sim/indoor

# 室外
ros2 launch super_slam SimRelocation.py \
  use_sim_time:=true rviz:=true map_dir:=map_sim/outdoor
```

单独建图时，将 `SimRelocation.py` 改为 `SimMapping.py`，其余参数相同。建图与重定位选择其一运行；两个启动文件都会同时启动对应的 SuperSLAM 节点和 RViz。

### 终端 3：Nav2 与导航 RViz

```bash
# 室内
ros2 launch sim_robot_slam nav2.launch.py \
  map:="$SIM_SHARE/maps/indoor.yaml" \
  route_graph:="$SIM_SHARE/routes/indoor_route.geojson" rviz:=true

# 室外
ros2 launch sim_robot_slam nav2.launch.py \
  map:="$SIM_SHARE/maps/outdoor.yaml" \
  route_graph:="$SIM_SHARE/routes/outdoor_route.geojson" rviz:=true
```

该启动文件默认使用仿真时钟，在定位就绪后激活 Nav2。它只启动导航部分，不启动仿真、重定位或巡检节点；普通目标导航进行到此步骤即可。

### 终端 4：固定路线巡检节点（可选）

```bash
# 室内
ros2 run sim_robot_slam route_patrol --ros-args \
  -p use_sim_time:=true -p start_id:=1 -p goal_id:=10 \
  -p task_config:="$SIM_SHARE/config/indoor_patrol_tasks.yaml"

# 室外闭环；via_ids 是字符串，保留下面的引号
ros2 run sim_robot_slam route_patrol --ros-args \
  -p use_sim_time:=true -p start_id:=0 -p goal_id:=0 -p via_ids:="'9'" \
  -p task_config:="$SIM_SHARE/config/outdoor_patrol_tasks.yaml"
```

在另一个终端调用第 5 节的开始或取消服务。单独运行节点时，参数名称为 `start_id`、`goal_id`、`via_ids`，不带启动文件中的 `route_` 前缀。

## 8. 模型与关键配置

### 机器人与传感器

模型配置为 `urdf/sim_robot.urdf.xacro`。以 `base_footprint` 为地面投影原点，X 向前、Y 向左、Z 向上；IMU 与激光雷达之间旋转为单位阵。

| 参数 | 当前值 |
| --- | --- |
| 前后轮距 / 左右轮距 | 0.580 / 0.454 m |
| 轮半径 / 宽度 | 0.125 / 0.050 m |
| 轮中心 | `(±0.290, ±0.227, 0.125)` m |
| 激光雷达原点 | `(0.343, 0, 0.539)` m |
| IMU 原点 | `(-0.1955, 0, 0.368)` m |
| 激光雷达相对 IMU 的平移 | `(0.5385, 0, 0.171)` m |
| 激光雷达扫描 | 16 线、水平 1024 点、10 Hz、垂直 ±15°、量程 0.2–60 m |
| IMU 白噪声标准差 | 加速度 0.05 m/s²；角速度 0.1°/s |
| IMU 启动零偏标准差 | 加速度 0.005 m/s²；角速度 0.01°/s |
| IMU 偏置漂移相关时间 | 3600 秒 |

外壳为简化模型，尺寸 0.60 × 0.40 × 0.15 m，质量 8 kg，每轮质量 0.6 kg。车轮包络为 0.830 × 0.504 m。麦克纳姆轮使用相对车身固定的各向异性摩擦近似滚子接触。

SuperSLAM 的 `SimMapping.yaml` 和 `SimRelocation.yaml` 应保持一致的外参：`lio.extrinsic.lidar_imu` 平移 `[0.5385, 0, 0.171]`，旋转为单位阵；`lio.extrinsic.odom_robo` 为 `[-0.1955, 0, 0.368, 0, 0, 0]`。更改传感器位置或外参后，重新建立先验地图和回环数据库。

### 导航参数

配置文件为 `config/nav2_params.yaml`，默认 RViz 配置为 `config/nav2.rviz`。

| 参数项 | 当前设置 |
| --- | --- |
| 最大前进速度 | 0.80 m/s |
| 最大转向角速度 | 0.60 rad/s |
| 后退恢复速度上限 | 0.25 m/s |
| 导航横向速度 | 0，禁止横移 |
| 控制器 | 航向对准控制器（RotationShim）与 MPPI，使用 DiffDrive 运动模型 |
| 先转向阈值 / 恢复前进阈值 | 0.10 / 0.05 rad |
| 足迹半长 / 半宽 / 额外裕量 | 0.415 / 0.252 / 0.02 m |
| 全局、局部膨胀半径 | 0.55 m，代价衰减系数 10.0 |

导航速度上限不作用于直接向 `/cmd_vel` 发布的键盘指令。机器人底盘仍支持手动横移。

导航 TF 链为 `map → world → imu → base_footprint → base_link → lidar_link`。其中 `map → world` 为单位静态变换，`world → imu` 由 SuperSLAM 发布，`imu → base_footprint` 静态平移为 `(0.1955, 0, -0.368)` m。

### 主要话题与导航点云

| 话题 | 消息类型 | 用途 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 底盘执行指令：`linear.x` 前进、`linear.y` 左移，单位 m/s；`angular.z` 逆时针转向，单位 rad/s |
| `/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | 平滑后的导航速度，进入碰撞检测 |
| `/wheel/odometry` | `nav_msgs/msg/Odometry` | 轮式里程计，速度字段为 `twist.twist` |
| `/joint_states` | `sensor_msgs/msg/JointState` | 车轮关节位置、速度 |
| `/imu/data` | `sensor_msgs/msg/Imu` | 原始 IMU 数据，坐标系为 `imu_link` |
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` | 原始点云，供 SuperSLAM 和碰撞检测使用 |
| `/lio/cloud_nav` | `sensor_msgs/msg/PointCloud2` | 底盘坐标系内过滤后的点云，标记障碍物 |
| `/lio/cloud_nav_clearing` | `sensor_msgs/msg/PointCloud2` | 雷达坐标系内的当前帧点云，用射线清障 |
| `/route_patrol/path` | `nav_msgs/msg/Path` | 完整巡检路线 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时钟 |

导航点云在 SuperSLAM 仿真配置中按相对 `base_footprint` 的高度过滤：默认保留 0.05–1.5 m、雷达 6 m 范围内的点，累计 3 帧且不超过 0.3 秒，再进行 0.05 m 体素滤波。原始点云不受影响。

标记可使用累计点云，清障只使用当前帧与真实雷达原点。定位丢失、时钟重置或位姿明显跳变时清空历史；动态场景可将 `lio.navigation.accumulation_frames` 设为 1。局部体素层的世界高度范围为 -0.5–1.9 m，需与场景地形适配。

## 9. 常见问题

### 定位完成后 Nav2 仍未激活

检查二维地图、定位状态和 TF，再查看节点生命周期：

```bash
ros2 topic info /map
ros2 topic echo /lio/localization_state
ros2 run tf2_ros tf2_echo map base_footprint
ros2 lifecycle get /controller_server
```

若控制器已退出，先查看退出前的参数或插件报错。局部代价地图的 `width`、`height` 使用整数，例如 `5`，不要写成 `5.0`。

### 规划成功但机器人不动，或巡检报错 105

错误 105 表示未取得足够运动进展。终端反馈的剩余距离是当前路径段的距离。停车时检查以下话题，确认是控制器无速度、碰撞检测拦截还是底盘未执行：

```bash
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /cmd_vel
ros2 topic echo /collision_monitor_state
ros2 lifecycle get /collision_monitor
```

同时检查局部代价地图是否误标地面、定位 TF 是否连续，以及键盘节点是否已经退出。

### RViz 没有地图、模型或完整巡检路线

- 地图：确认 YAML 中的图像路径有效，二维地图与先验地图坐标系一致；分开启动 Nav2 时必须显式指定 `map`。
- 模型：确认 `robot_state_publisher` 正在运行，`/robot_description` 和机器人 TF 存在，启用 **RobotModel**。
- 路线：查看绿色 **Patrol Route**。`/plan` 由多个规划节点更新，只显示最新路径；完整巡检路线使用 `/route_patrol/path` 和 **Transient Local** 持久性。
- 原始点云：检查 `/lidar/points` 是否持续发布；仅运行普通仿真时固定坐标系用 `odom`，导航时用 `map`，点云可靠性设为 **Best Effort**。GPU 激光雷达需要可用的图形驱动与 OpenGL。

### 局部代价地图出现地面圆环

检查 `/lio/cloud_nav` 的地面过滤和清障点云。修改过滤参数后重启 SuperSLAM 与 Nav2；也可清空当前局部地图：

```bash
ros2 service call /local_costmap/clear_entirely_local_costmap \
  nav2_msgs/srv/ClearEntireCostmap '{}'
```

### Gazebo 看不到室外机器人

室外 SDF 已内置机器人，独立启动时使用 `spawn_robot:=false`。在实体树中检查 `sim_robot_slam`，并确认相机位置和启动日志。

每次仿真启动默认创建独立的 Gazebo 通信分区。使用 Gazebo 命令行排查时，可给启动文件传入 `gz_partition:=sim_robot_debug`，并在命令行终端执行 `export GZ_PARTITION=sim_robot_debug`。

### 保存启动日志

```bash
ros2 launch sim_robot_slam outdoor_navigation.launch.py rviz:=true 2>&1 | tee nav2.log
```

ROS 默认日志目录为 `~/.ros/log`。定位进程退出原因时，查看 `process has died` 之前的报错。
