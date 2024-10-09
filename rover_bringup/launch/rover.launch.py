import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rover_motor_controller_shared_dir = get_package_share_directory(
        "rover_motor_controller_cpp")
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")

    rover_motor_controller_action_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_motor_controller_shared_dir, "launch", "motor_controller.launch.py"))
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    
    ld.add_action(rover_motor_controller_action_cmd)

    return ld
