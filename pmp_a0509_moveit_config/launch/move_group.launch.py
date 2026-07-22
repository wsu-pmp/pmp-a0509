# octomap point cloud topic names can't be dynamically prefixed by
# PushRosNamespace - ZED wrapper's LoadComposableNodes targets an absolute
# container path.
#
# override the single-stack default in config/sensors_3d.yaml with the
# camera_name param given in order for instance disambiguation to match

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def _launch_setup(context, *args, **kwargs):
    camera_name = context.perform_substitution(LaunchConfiguration("camera_name"))

    moveit_config = MoveItConfigsBuilder(
        "a0509", package_name="pmp_a0509_moveit_config"
    ).to_moveit_configs()

    topic_root = f"/{camera_name}/zed_node"
    sensor = moveit_config.sensors_3d["default_sensor"]
    sensor["point_cloud_topic"] = f"{topic_root}/point_cloud/cloud_registered"
    sensor["filtered_cloud_topic"] = f"{topic_root}/filtered_cloud"

    return generate_move_group_launch(moveit_config).entities


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_name",
                default_value="zed",
                description="ZED camera_name this stack's perception layer was "
                "given (see perception.launch.py); used only to resolve the "
                "octomap updater's point cloud topics",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
