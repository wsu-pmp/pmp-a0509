# grasp pipeline: perception detections -> planning-scene target -> (future) MTC
# pick. runs the ZED detection bridge and the object selection manager. consumes
# the ZED sensor by topic (perception is composed at the bringup layer)
#
# takes no namespace of its own - bringup.launch.py wraps this in the stack's
# PushRosNamespace group, which is what points its planning_scene topics and
# get_planning_scene client (all relative) at the matching move_group.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    object_id_prefix = LaunchConfiguration("object_id_prefix")
    camera_name = LaunchConfiguration("camera_name")

    args = [
        DeclareLaunchArgument(
            "camera_name",
            default_value="zed",
            description="ZED camera_name the perception layer was given (see "
            "perception.launch.py); used to build objects_topic. absolute, "
            "because the ZED sits outside this stack's namespace",
        ),
        DeclareLaunchArgument(
            "object_id_prefix",
            default_value="zed_object_",
            description="collision-object id prefix shared by the bridge and manager",
        ),
    ]

    bridge = Node(
        package="pmp_a0509_grasp",
        executable="zed_object_bridge",
        name="zed_object_bridge",
        output="screen",
        parameters=[
            {
                "object_id_prefix": object_id_prefix,
                "objects_topic": [
                    "/",
                    camera_name,
                    "/zed_node/obj_det/objects",
                ],
            }
        ],
    )

    manager = Node(
        package="pmp_a0509_grasp",
        executable="object_selection_manager",
        name="object_selection_manager",
        output="screen",
        parameters=[{"object_id_prefix": object_id_prefix}],
    )

    return LaunchDescription(args + [bridge, manager])
