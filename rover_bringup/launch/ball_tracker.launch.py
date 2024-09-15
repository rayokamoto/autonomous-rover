from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
)

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_name = "rover_bringup"
    sim_mode = LaunchConfiguration("sim_mode")
    sim_mode_dec = DeclareLaunchArgument("sim_mode", default_value="false")

    tracker_params_sim = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "ball_tracker_params_sim.yaml"]
    )
    tracker_params_robot = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "ball_tracker_params_robot.yaml"]
    )

    params_path = PythonExpression(
        [
            '"',
            tracker_params_sim,
            '" if "true" == "',
            sim_mode,
            '" else "',
            tracker_params_robot,
            '"',
        ]
    )

    tracker_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("ball_tracker"), "launch", "ball_tracker.launch.py"]
        ),
        launch_arguments={
            "params_file": params_path,
            "image_topic": "/camera/image_raw",
            "cmd_vel_topic": "/cmd_vel_tracker",
            "enable_3d_tracker": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            sim_mode_dec,
            tracker_launch,
        ]
    )
