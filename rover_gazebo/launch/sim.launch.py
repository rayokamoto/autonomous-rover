from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_package = "rover_bringup"
    model_package = "rover_basic_description"
    gazebo_package = "rover_gazebo"

    # Declare the Gazebo world file argument with a default world as fallback
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution(
            [FindPackageShare(gazebo_package), "worlds", "obstacles.world"]
        ),
        description="Gazebo world file",
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(model_package), "launch", "description.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": "true", "use_ros2_control": "true"}.items(),
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(bringup_package), "launch", "joystick.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    twist_mux_params = PathJoinSubstitution(
        [FindPackageShare(bringup_package), "config", "twist_mux.yaml"]
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": True}],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )

    gazebo_params_file = PathJoinSubstitution(
        [FindPackageShare(gazebo_package), "config", "gazebo_params.yaml"]
    )

    # Include the Gazebo launch file, using the relative world path provided as argument
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "extra_gazebo_args": "--ros-args --params-file "
            + gazebo_params_file.describe(),
        }.items(),
    )

    # Run the spawner node from the gazebo_ros package.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "rover"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Code for delaying a node (I haven't tested how effective it is)
    #
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner

    # Launch them all!
    return LaunchDescription(
        [
            world_arg,
            rsp,
            joystick,
            twist_mux,
            gazebo,
            spawn_entity,
            diff_drive_spawner,
            joint_broad_spawner,
        ]
    )
