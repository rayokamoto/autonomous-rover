from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    pkg_dir = FindPackageShare("rover_navigation")
    launch_dir = PathJoinSubstitution([pkg_dir, "launch"])

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    def run_nav2(
        context: LaunchContext,
        planner: LaunchConfiguration,
        controller: LaunchConfiguration,
    ):
        planner = str(context.perform_substitution(planner))
        controller = str(context.perform_substitution(controller))

        params_file = PathJoinSubstitution(
            [pkg_dir, "params", f"{planner}_{controller}.yaml"]
        )
        param_substitutions = {"use_sim_time": LaunchConfiguration("use_sim_time")}
        configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=LaunchConfiguration("namespace"),
            param_rewrites=param_substitutions,
            convert_types=True,
        )

        # Specify the actions
        return [
            PushRosNamespace(
                condition=IfCondition(LaunchConfiguration("use_namespace")),
                namespace=LaunchConfiguration("namespace"),
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("use_composition")),
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[
                    configured_params,
                    {"autostart": LaunchConfiguration("autostart")},
                ],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/cmd_vel", LaunchConfiguration("cmd_vel_topic")),
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([launch_dir, "navigation.launch.py"]),
                launch_arguments={
                    "namespace": LaunchConfiguration("namespace"),
                    "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    "default_nav_to_pose_bt_xml": LaunchConfiguration(
                        "default_bt_xml_filename"
                    ),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "autostart": LaunchConfiguration("autostart"),
                    "params_file": params_file,
                    "use_composition": LaunchConfiguration("use_composition"),
                    "use_respawn": LaunchConfiguration("use_respawn"),
                    "container_name": "nav2_container",
                }.items(),
            ),
        ]

    # Create the launch configuration variables
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    cmd_vel_topic_cmd = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="cmd_vel",
        description="cmd_vel topic (for remmaping)",
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=PathJoinSubstitution([pkg_dir, "behavior_trees", "rover_bt.xml"]),
        description="Full path to the behavior tree xml file to use",
    )

    planner = LaunchConfiguration("planner")
    planner_cmd = DeclareLaunchArgument(
        "planner",
        default_value="SmacHybrid",
        choices=["SmacHybrid", "SmacLattice"],
        description="Nav2 planner (SmacHybrid or SmacLattice)",
    )

    controller = LaunchConfiguration("controller")
    controller_cmd = DeclareLaunchArgument(
        "controller",
        default_value="MPPI",
        choices=["RPP", "TEB", "MPPI"],
        description="Nav2 controller (RPP or TEB or MPPI)",
    )

    nav2_cmd = OpaqueFunction(function=run_nav2, args=[planner, controller])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(cmd_vel_topic_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(planner_cmd)
    ld.add_action(controller_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(nav2_cmd)

    return ld
