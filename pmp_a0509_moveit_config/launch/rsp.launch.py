# robot_state_publisher for the arg-parametrised description. computes the
# a0509 xacro with the runtime a0509 and rg2 args and publishes a
# single latched robot_description, consumed by both rsp's tf output and the
# controller_manager (via its ~/robot_description topic)
#
# takes no namespace of its own - start.launch.py wraps this in the stack's
# PushRosNamespace group, which is also what puts rsp's /tf onto the stack's own
# TF tree. the description itself is instance-independent (see a0509.urdf.xacro).

from dsr_bringup2.utils import read_update_rate
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "host", default_value="127.0.0.1", description="ROBOT_IP"
        ),
        DeclareLaunchArgument("port", default_value="12345", description="ROBOT_PORT"),
        DeclareLaunchArgument(
            "mode", default_value="virtual", description="OPERATION MODE"
        ),
        DeclareLaunchArgument(
            "rt_host", default_value="192.168.137.50", description="ROBOT_RT_IP"
        ),
        DeclareLaunchArgument(
            "rg2_use_fake_hardware",
            default_value="true",
            description="Use fake hardware for the RG2 gripper?",
        ),
        DeclareLaunchArgument(
            "rg2_ip_address",
            default_value="192.168.1.1",
            description="RG2 Compute Box IP address (tcp connection only).",
        ),
        DeclareLaunchArgument(
            "rg2_port",
            default_value="502",
            description="RG2 Compute Box port (tcp connection only).",
        ),
        DeclareLaunchArgument(
            "camera_model",
            default_value="zed2i",
            description="ZED camera model, selects which mesh/xacro macro "
            "zed_descr.urdf.xacro renders",
        ),
    ]

    update_rate = str(read_update_rate())

    robot_description_path = PathJoinSubstitution(
        [
            FindPackageShare("pmp_a0509_description"),
            "urdf",
            "a0509.urdf.xacro",
        ]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_path,
            " host:=",
            LaunchConfiguration("host"),
            " rt_host:=",
            LaunchConfiguration("rt_host"),
            " port:=",
            LaunchConfiguration("port"),
            " mode:=",
            LaunchConfiguration("mode"),
            " update_rate:=",
            update_rate,
            " rg2_use_fake_hardware:=",
            LaunchConfiguration("rg2_use_fake_hardware"),
            " rg2_ip_address:=",
            LaunchConfiguration("rg2_ip_address"),
            " rg2_port:=",
            LaunchConfiguration("rg2_port"),
            " camera_model:=",
            LaunchConfiguration("camera_model"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    return LaunchDescription(args + [robot_state_pub_node])
