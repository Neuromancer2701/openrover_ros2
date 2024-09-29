import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import (
    EmitEvent,
    RegisterEventHandler,
    LogInfo,
)
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
import lifecycle_msgs.msg
from launch_ros.event_handlers import OnStateTransition


def generate_launch_description():
    package_name = "diff_drive_controller_node"
    node_namespace = LaunchConfiguration("__ns")
    node_name = LaunchConfiguration("__node")
    log_level = LaunchConfiguration("__log_level")
    diff_drive_config = LaunchConfiguration("diff_drive_config")
    node_exe = "diff_drive_controller_node"

    declare_diff_drive_config_cmd = DeclareLaunchArgument(
        "diff_drive_config",
        default_value=os.path.join(
            get_package_share_directory(package_name),
            "config",
            "diff_drive_controller_node.yaml",
        ),
        description="Full path to the ROS2 parameters file to use for Diff Drive Controller configuration",
    )

    declare_ns = DeclareLaunchArgument(
        "__ns",
        default_value=package_name,
        description="Remap the node namespace",
    )

    declare_nn = DeclareLaunchArgument(
        "__node",
        default_value="diff_drive_controller",
        description="Remap the node name",
    )

    declare_log_level = DeclareLaunchArgument(
        "__log_level", default_value="info", description="Log verbosity level"
    )

    diff_drive_controller_node = LifecycleNode(
        package=package_name,
        executable=node_exe,
        namespace=node_namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
            "--",
        ],
        parameters=[
            diff_drive_config,
        ],
        name=node_name,
        output="screen",
        log_cmd=True,
        emulate_tty=True,
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(diff_drive_controller_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=diff_drive_controller_node,
            goal_state="inactive",
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] Diff Drive Controller node is activating."
                ),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(
                            diff_drive_controller_node
                        ),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription(
        [
            declare_ns,
            declare_nn,
            declare_log_level,
            declare_diff_drive_config_cmd,
            diff_drive_controller_node,
            configure_event,
            activate_event
        ]
    )