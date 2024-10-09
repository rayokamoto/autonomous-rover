from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")

    # Process the URDF file
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("rover_basic_description"), "urdf", "robot.urdf.xacro"]
    )

    robot_description_config = Command(
        [
            "xacro ",
            xacro_file,
            " use_ros2_control:=",
            use_ros2_control,
            " sim_mode:=",
            use_sim_time,
        ]
    )

    # Create a robot_state_publisher node
    params = {
        "robot_description": robot_description_config,
        "use_sim_time": use_sim_time,
    }
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    # Launch!
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "use_ros2_control",
                default_value="true",
                description="Use ros2_control if true",
            ),
            node_robot_state_publisher,
        ]
    )
