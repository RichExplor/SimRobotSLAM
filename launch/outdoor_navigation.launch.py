from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("sim_robot_slam")
    super_slam_share = FindPackageShare("super_slam")
    simulation_launch = PathJoinSubstitution(
        [package_share, "launch", "gazebo_simulation.launch.py"]
    )
    nav2_launch = PathJoinSubstitution([package_share, "launch", "nav2.launch.py"])
    relocation_rviz_config = PathJoinSubstitution(
        [super_slam_share, "rviz", "Relocation.rviz"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    map_dir = LaunchConfiguration("map_dir")
    autostart = LaunchConfiguration("autostart")
    rviz = LaunchConfiguration("rviz")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
        launch_arguments={
            "world": PathJoinSubstitution(
                [package_share, "worlds", "outdoor_world.sdf"]
            ),
            "use_sim_time": use_sim_time,
            "bridge_config": PathJoinSubstitution(
                [package_share, "config", "bridge_nav2.yaml"]
            ),
            "spawn_x": "0.0",
            "spawn_y": "-9.0",
            "spawn_yaw": "0.0",
            "spawn_robot": "false",
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
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "rviz": rviz,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [package_share, "maps", "outdoor.yaml"]
                ),
                description="Required outdoor Nav2 occupancy map YAML file.",
            ),
            DeclareLaunchArgument(
                "map_dir",
                default_value="map_sim/outdoor",
                description="Outdoor SuperSLAM prior-map directory under the workspace root.",
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
        ]
    )
