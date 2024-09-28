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
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_name = "diff_driver_controller_node"
    share_dir = get_package_share_directory(package_name)
    node_namespace = LaunchConfiguration("__ns")
    node_name = LaunchConfiguration("__node")
    log_level = LaunchConfiguration("__log_level")
    params_file = LaunchConfiguration("__params")
    node_exe = "diff_driver_controller_node"

    default_params_file = os.path.join(
        share_dir, "config", "diff_driver_controller_node.yaml"
    )
    configured_diff_drive_params = RewrittenYaml(
        source_file=default_params_file
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

    declare_params = DeclareLaunchArgument(
        "__params",
        default_value=configured_diff_drive_params,
        description="Path to YAML-based node parameterization",
    )

    diff_driver_controller_node = LifecycleNode(
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
            params_file,
        ],
        name=node_name,
        output="screen",
        log_cmd=True,
        emulate_tty=True,
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(diff_driver_controller_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=diff_driver_controller_node,
            goal_state="inactive",
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] Diff Drive Controller node is activating."
                ),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(
                            diff_driver_controller_node
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
            declare_params,
            configure_event,
            activate_event,
            diff_driver_controller_node,
        ]
    )