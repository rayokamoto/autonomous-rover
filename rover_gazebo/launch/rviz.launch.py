from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("rover_basic_description"), "rviz", "map.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen"
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_node
    ])
