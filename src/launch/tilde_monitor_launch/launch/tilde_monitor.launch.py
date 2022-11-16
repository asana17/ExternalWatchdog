from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    tilde_aggregator_param_path = LaunchConfiguration("tilde_aggregator_param_file").perform(context)
    with open(tilde_aggregator_param_path, "r") as f:
        tilde_aggregator_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    tilde_aggregator_topic_path = LaunchConfiguration("tilde_aggregator_topic_file").perform(context)
    with open(tilde_aggregator_topic_path, "r") as f:
        tilde_aggregator_topic = yaml.safe_load(f)["/**"]["ros__parameters"]

    tilde_error_monitor_param_path = LaunchConfiguration("tilde_error_monitor_param_file").perform(context)
    with open(tilde_error_monitor_param_path, "r") as f:
        tilde_error_monitor_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    tilde_aggregator_node = Node(
                package='tilde_aggregator',
                executable='tilde_aggregator',
                parameters=[tilde_aggregator_param, tilde_aggregator_topic, ],
                output='screen'
            )

    tilde_error_monitor_node = Node(
                package='tilde_error_monitor',
                executable='tilde_error_monitor',
                parameters=[tilde_error_monitor_param, ],
                output='screen'
            )

    group = GroupAction(
            [
                PushRosNamespace("tilde_monitor"),
                tilde_aggregator_node,
                tilde_error_monitor_node,
            ]
        )

    return [group]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg(
        "tilde_aggregator_param_file",
        [
            FindPackageShare("tilde_aggregator"),
            "/config/tilde_aggregator.param.yaml",
        ],
        "path to the parameter file of tilde_aggregator",
    )
    add_launch_arg(
        "tilde_aggregator_topic_file",
        [
            FindPackageShare("tilde_aggregator"),
            "/config/tilde_message_tracking_tag.param.yaml",
        ],
        "path to the parameter file of tilde_aggregator",
    )
    add_launch_arg(
        "tilde_error_monitor_param_file",
        [
            FindPackageShare("tilde_error_monitor"),
            "/config/tilde_error_monitor.param.yaml",
        ],
        "path to the parameter file of tilde_error_monitor",
    )

    return LaunchDescription(
        launch_arguments
        + [OpaqueFunction(function=launch_setup)]
    )
