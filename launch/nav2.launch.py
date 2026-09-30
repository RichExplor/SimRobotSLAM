from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("sim_robot_slam")
    nav2_bringup_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
    )
    nav2_rviz_config = PathJoinSubstitution(
        [package_share, "config", "nav2.rviz"]
    )

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    rviz = LaunchConfiguration("rviz")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            params_file,
            {
                "yaml_filename": map_yaml,
                "topic_name": "map",
                "frame_id": "map",
                "use_sim_time": use_sim_time,
            },
        ],
    )

    imu_to_base_footprint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_to_base_footprint",
        arguments=[
            "--x", "0.1955",
            "--y", "0.0",
            "--z", "-0.368",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "imu",
            "--child-frame-id", "base_footprint",
        ],
        output="screen",
    )

    map_to_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_world",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "map",
            "--child-frame-id", "world",
        ],
        output="screen",
    )

    map_server_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
    )

    # IncludeLaunchDescription writes child arguments into the current launch
    # context. Scope autostart:=false to Nav2 bringup so it cannot disable the
    # localization gate below, which uses the parent autostart argument.
    navigation = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup_launch),
                launch_arguments={
                    "slam": "False",
                    "use_localization": "False",
                    "use_namespace": "False",
                    "map": map_yaml,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    # The localization gate starts this lifecycle manager once
                    # map, SuperSLAM validity, and the complete TF chain are ready.
                    "autostart": "false",
                    "use_composition": "False",
                    "use_respawn": "False",
                }.items(),
            )
        ],
    )

    localization_gate = Node(
        package="sim_robot_slam",
        executable="nav2_localization_gate",
        name="nav2_localization_gate",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(autostart),
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="nav2_rviz",
        arguments=["-d", nav2_rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [package_share, "maps", "map.yaml"]
                ),
                description="Path to the OccupancyGrid map YAML file.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "nav2_params.yaml"]
                ),
                description="Nav2 parameters for this Mecanum simulation.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description=(
                    "Start Nav2 after the map, valid SuperSLAM localization, and TF are ready. "
                    "False leaves activation to RViz."
                ),
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Whether to start the Nav2 RViz configuration.",
            ),
            imu_to_base_footprint,
            map_to_world,
            map_server,
            map_server_lifecycle_manager,
            navigation,
            localization_gate,
            rviz_node,
        ]
    )
