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
    gazebo_launch = PathJoinSubstitution(
        [package_share, "launch", "gazebo_simulation.launch.py"]
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    map_dir = LaunchConfiguration("map_dir")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            "world": PathJoinSubstitution(
                [package_share, "worlds", "indoor_world.sdf"]
            ),
            "use_sim_time": use_sim_time,
            "bridge_config": PathJoinSubstitution(
                [package_share, "config", "bridge.yaml"]
            ),
            "spawn_x": "0.0",
            "spawn_y": "0.0",
            "spawn_yaw": "0.0",
            "spawn_robot": "true",
        }.items(),
    )

    mapping = Node(
        package="super_slam",
        executable="super_slam_node",
        name="super_slam_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([super_slam_share, "config", "SimMapping.yaml"]),
            {
                "use_sim_time": use_sim_time,
                "lio.map.save_map_dir": map_dir,
                "lio.map.map_name": "map.pcd",
            },
        ],
        arguments=["--ros-args", "--log-level", "info"],
        sigterm_timeout="inf",
        sigkill_timeout="inf",
    )

    slam_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="super_slam_rviz",
        arguments=[
            "-d",
            PathJoinSubstitution([super_slam_share, "rviz", "Lio.rviz"]),
            "--ros-args",
            "--log-level",
            "warn",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Show the SuperSLAM mapping visualization.",
            ),
            DeclareLaunchArgument(
                "map_dir",
                default_value="map_sim/indoor",
                description="Indoor SuperSLAM map directory under the workspace root.",
            ),
            simulation,
            mapping,
            slam_rviz,
        ]
    )
