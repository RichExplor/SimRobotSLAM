from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("sim_robot_slam")
    world_file = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description_file = PathJoinSubstitution(
        [package_share, "urdf", "sim_robot.urdf.xacro"]
    )
    bridge_config_file = LaunchConfiguration("bridge_config")
    gazebo_launch_file = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={"gz_args": ["-r ", world_file]}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", robot_description_file]), value_type=str
                ),
                "use_sim_time": use_sim_time,
            }
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config_file}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "sim_robot_slam",
            "-topic",
            "robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [package_share, "worlds", "demo.sdf"]
                ),
                description="SDF world file to start in Gazebo Sim.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock in ROS nodes.",
            ),
            DeclareLaunchArgument(
                "bridge_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "bridge.yaml"]
                ),
                description="ROS-Gazebo bridge configuration file.",
            ),
            gazebo,
            robot_state_publisher,
            bridge,
            TimerAction(period=3.0, actions=[spawn_robot]),
        ]
    )
