from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("sim_robot_slam")
    simulation_launch = PathJoinSubstitution(
        [package_share, "launch", "simulation.launch.py"]
    )
    super_slam_relocation_launch = PathJoinSubstitution(
        [FindPackageShare("super_slam"), "launch", "SimRelocation.py"]
    )
    nav2_bridge_config = PathJoinSubstitution(
        [package_share, "config", "bridge_nav2.yaml"]
    )
    nav2_launch = PathJoinSubstitution(
        [package_share, "launch", "nav2.launch.py"]
    )

    world = LaunchConfiguration("world")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    rviz = LaunchConfiguration("rviz")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
        launch_arguments={
            "world": world,
            "use_sim_time": use_sim_time,
            "bridge_config": nav2_bridge_config,
        }.items(),
    )

    # SimRelocation sets rviz:=false internally. Keep that override local so it
    # does not disable the Nav2 RViz launch argument in the parent context.
    super_slam_relocation = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(super_slam_relocation_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "rviz": "true",
                }.items(),
            )
        ],
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
                "world",
                default_value=PathJoinSubstitution(
                    [package_share, "worlds", "demo.sdf"]
                ),
                description="SDF world to launch in Gazebo Sim.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [package_share, "maps", "map.yaml"]
                ),
                description="Path to the Nav2 OccupancyGrid map YAML file.",
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
                    "Automatically start Nav2 after the map, valid SuperSLAM localization, "
                    "and TF are ready. False leaves activation to RViz."
                ),
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Whether to start Nav2 RViz for monitoring and navigation goals.",
            ),
            simulation,
            super_slam_relocation,
            navigation,
        ]
    )
