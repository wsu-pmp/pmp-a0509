# ZED sensor layer for one stack. feeds two consumers by topic: the move_group
# octomap updater (point cloud) and the grasp bridge (object detections).
#
# This layer is deliberately NOT inside the stack's PushRosNamespace group.
# zed_camera.launch.py loads its component with LoadComposableNodes against an
# absolute container path ('/' + namespace + '/' + container_name), and
# PushRosNamespace does not rewrite target_container - only the loaded node's own
# namespace - so pushing a namespace over it makes the load target a container
# that does not exist. Its `namespace` arg is no help either: setting it renames
# the node to camera_name, moving every topic off zed_node.
#
# So the ZED is disambiguated by zed_wrapper's own camera_name, which becomes its
# node namespace: /<camera_name>/zed_node/... . bringup.launch.py derives that
# from the stack's single `namespace` arg. camera_name also prefixes the
# wrapper's link names, but that never reaches us - publish_urdf/publish_tf are
# false and our own rsp publishes the ZED links from the robot_description.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "camera_name",
            default_value="zed",
            description="ZED camera_name - becomes the wrapper's node namespace, "
            "so this is what separates two cameras on one domain (see "
            "zed_camera.launch.py). must match what start.launch.py was given, "
            "since move_group resolves its octomap topics from it",
        ),
        DeclareLaunchArgument(
            "camera_model", default_value="zed2i", description="ZED camera model"
        ),
        DeclareLaunchArgument(
            "enable_object_detection",
            default_value="true",
            description="Enable ZED object detection (feeds the grasp bridge)?",
        ),
        DeclareLaunchArgument(
            "enable_body_tracking",
            default_value="true",
            description="Enable ZED body tracking?",
        ),
    ]

    camera_name = LaunchConfiguration("camera_name")
    enable_od = LaunchConfiguration("enable_object_detection")
    enable_bt = LaunchConfiguration("enable_body_tracking")

    zed_wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("zed_wrapper"),
                        "launch",
                        "zed_camera.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "camera_name": camera_name,
            # our rsp already publishes the ZED links as part of the a0509
            # robot_description, so suppress the wrapper's own rsp and TF
            # broadcast - two publishers for the same frames otherwise
            "publish_tf": "false",
            "publish_urdf": "false",
            "camera_model": LaunchConfiguration("camera_model"),
            "object_detection.od_enabled": enable_od,
            "body_tracking.bt_enabled": enable_bt,
            "pos_tracking.pos_tracking_enabled": "true",
        }.items(),
    )

    # zed brings the detectors up disabled; enable them once the node is running.
    # these are CLI calls in a subprocess, so they need the fully-qualified
    # service name - built from camera_name, not the literal "zed" default
    enable_zed_services = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    ["/", camera_name, "/zed_node/enable_obj_det"],
                    "std_srvs/srv/SetBool",
                    "{data: true}",
                ],
                output="screen",
                condition=IfCondition(enable_od),
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    ["/", camera_name, "/zed_node/enable_body_trk"],
                    "std_srvs/srv/SetBool",
                    "{data: true}",
                ],
                output="screen",
                condition=IfCondition(enable_bt),
            ),
        ],
    )

    return LaunchDescription(args + [zed_wrapper, enable_zed_services])
