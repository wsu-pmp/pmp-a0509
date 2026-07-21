# shared ZED sensor layer. feeds two consumers by topic: the move_group octomap
# updater (point cloud) and the grasp bridge (object detections). owns no
# vendor prefix - it only ever concerns the ZED, so args stay unprefixed.

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
            "publish_tf": "false",
            "camera_model": LaunchConfiguration("camera_model"),
            "object_detection.od_enabled": enable_od,
            "body_tracking.bt_enabled": enable_bt,
            "pos_tracking.pos_tracking_enabled": "true",
        }.items(),
    )

    # zed brings the detectors up disabled; enable them once the node is running
    enable_zed_services = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/zed/zed_node/enable_obj_det",
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
                    "/zed/zed_node/enable_body_trk",
                    "std_srvs/srv/SetBool",
                    "{data: true}",
                ],
                output="screen",
                condition=IfCondition(enable_bt),
            ),
        ],
    )

    return LaunchDescription(args + [zed_wrapper, enable_zed_services])
