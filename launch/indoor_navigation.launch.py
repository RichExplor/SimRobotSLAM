from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("sim_robot_slam")
    super_slam_share = FindPackageShare("super_slam")
    gazebo_launch = PathJoinSubstitution(
        [package_share, "launch", "gazebo_simulation.launch.py"]
    )
    nav2_launch = PathJoinSubstitution([package_share, "launch", "nav2.launch.py"])
    relocation_rviz_config = PathJoinSubstitution(
        [super_slam_share, "rviz", "Relocation.rviz"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    route_graph = LaunchConfiguration("route_graph")
    route_start_id = LaunchConfiguration("route_start_id")
    route_goal_id = LaunchConfiguration("route_goal_id")
    route_via_ids = LaunchConfiguration("route_via_ids")
    task_config = LaunchConfiguration("task_config")
    map_dir = LaunchConfiguration("map_dir")
    autostart = LaunchConfiguration("autostart")
    rviz = LaunchConfiguration("rviz")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            "world": PathJoinSubstitution(
                [package_share, "worlds", "indoor_world.sdf"]
            ),
            "use_sim_time": use_sim_time,
            "bridge_config": PathJoinSubstitution(
                [package_share, "config", "bridge_nav2.yaml"]
            ),
            "spawn_x": "0.0",
            "spawn_y": "0.0",
            "spawn_yaw": "0.0",
            "spawn_robot": "true",
        }.items(),
    )

    relocation = Node(
        package="super_slam",
        executable="relocation_node",
        name="relocation_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [super_slam_share, "config", "SimRelocation.yaml"]
            ),
            {
                "use_sim_time": use_sim_time,
                "lio.map.save_map_dir": map_dir,
                "lio.map.map_name": "map.pcd",
            },
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    relocation_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="super_slam_relocation_rviz",
        arguments=["-d", relocation_rviz_config, "--ros-args", "--log-level", "warn"],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            "map": map_yaml,
            "route_graph": route_graph,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "rviz": rviz,
        }.items(),
    )

    route_patrol = Node(
        package="sim_robot_slam",
        executable="route_patrol",
        name="route_patrol",
        output="screen",
        parameters=[
            {
                "start_id": ParameterValue(route_start_id, value_type=int),
                "goal_id": ParameterValue(route_goal_id, value_type=int),
                "via_ids": ParameterValue(route_via_ids, value_type=str),
                "task_config": ParameterValue(task_config, value_type=str),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [package_share, "maps", "indoor.yaml"]
                ),
                description="Indoor Nav2 occupancy map YAML file.",
            ),
            DeclareLaunchArgument(
                "map_dir",
                default_value="map_sim/indoor",
                description="Indoor SuperSLAM prior-map directory under the workspace root.",
            ),
            DeclareLaunchArgument(
                "route_graph",
                default_value=PathJoinSubstitution(
                    [package_share, "routes", "indoor_route.geojson"]
                ),
                description="GeoJSON route graph loaded by Nav2 Route Server.",
            ),
            DeclareLaunchArgument(
                "route_via_ids",
                default_value="",
                description="Ordered intermediate node IDs, separated by commas.",
            ),
            DeclareLaunchArgument(
                "task_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "indoor_patrol_tasks.yaml"]
                ),
                description="Inspection tasks keyed by route graph node ID; empty disables tasks.",
            ),
            DeclareLaunchArgument(
                "route_start_id",
                default_value="1",
                description=(
                    "Start node ID in the indoor route graph. Node 1 is near the "
                    "default Gazebo spawn pose."
                ),
            ),
            DeclareLaunchArgument(
                "route_goal_id",
                default_value="10",
                description="Goal node ID in the indoor route graph.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Start Nav2 after SuperSLAM localization and TF are ready.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Show the Nav2 and SuperSLAM RViz interfaces.",
            ),
            simulation,
            relocation,
            relocation_rviz,
            navigation,
            route_patrol,
        ]
    )
