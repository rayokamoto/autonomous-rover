from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rover_bringup"), "config", "mapper_params_online_async.yaml"]
        ),
        description="Full path to the SLAM toolbox parameters YAML file"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        parameters=[LaunchConfiguration("params_file"), {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen"
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        slam_toolbox_node
    ])
