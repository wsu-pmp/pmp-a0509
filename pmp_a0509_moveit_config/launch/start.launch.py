# robot + MoveIt layer, no perception/grasp. thin composition of the component
# launches: the dsr emulator (virtual mode only), rsp (arg'd description),
# robot_control (control_node + spawners + drcf gate), move_group and RViz.
# useful standalone (bring up arm + gripper + MoveIt); also the layer bringup
# composes. every event-handler chain lives inside its component (decision #1).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

MOVEIT_CONFIG = "pmp_a0509_moveit_config"


def _include(package, launch_file, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])]
        ),
        launch_arguments=arguments.items() if arguments else None,
        condition=condition,
    )


def generate_launch_description():
    args = [
        DeclareLaunchArgument("name", default_value="", description="NAME_SPACE"),
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
            "controller_log_level",
            default_value="warn",
            description="ros2_control_node log-level",
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        ),
        DeclareLaunchArgument(
            "onrobot_use_fake_hardware",
            default_value="true",
            description="Use fake hardware for the RG2 gripper?",
        ),
        DeclareLaunchArgument(
            "onrobot_ip_address",
            default_value="192.168.1.1",
            description="RG2 Compute Box IP address (tcp connection only).",
        ),
        DeclareLaunchArgument(
            "onrobot_port",
            default_value="502",
            description="RG2 Compute Box port (tcp connection only).",
        ),
        DeclareLaunchArgument(
            "drcf_ready_timeout_sec",
            default_value="30.0",
            description="max wait for the drcf endpoint (host:port) to accept "
            "connections before control_node starts",
        ),
    ]

    is_virtual = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mode"), "' == 'virtual'"])
    )

    # dsr emulator provides the drcf endpoint in virtual mode; robot_control's
    # gate polls that endpoint regardless of who provides it
    run_emulator = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration("name"),
        parameters=[
            {"name": LaunchConfiguration("name")},
            {"rate": 100},
            {"standby": 5000},
            {"command": True},
            {"host": LaunchConfiguration("host")},
            {"port": LaunchConfiguration("port")},
            {"mode": LaunchConfiguration("mode")},
            {"model": "a0509"},
            {"gripper": "none"},
            {"mobile": "none"},
            {"rt_host": LaunchConfiguration("rt_host")},
        ],
        output="screen",
        condition=is_virtual,
    )

    rsp = _include(
        MOVEIT_CONFIG,
        "rsp.launch.py",
        {
            "name": LaunchConfiguration("name"),
            "host": LaunchConfiguration("host"),
            "port": LaunchConfiguration("port"),
            "mode": LaunchConfiguration("mode"),
            "rt_host": LaunchConfiguration("rt_host"),
            "onrobot_use_fake_hardware": LaunchConfiguration(
                "onrobot_use_fake_hardware"
            ),
            "onrobot_ip_address": LaunchConfiguration("onrobot_ip_address"),
            "onrobot_port": LaunchConfiguration("onrobot_port"),
        },
    )

    robot_control = _include(
        MOVEIT_CONFIG,
        "robot_control.launch.py",
        {
            "name": LaunchConfiguration("name"),
            "host": LaunchConfiguration("host"),
            "port": LaunchConfiguration("port"),
            "controller_log_level": LaunchConfiguration("controller_log_level"),
            "drcf_ready_timeout_sec": LaunchConfiguration("drcf_ready_timeout_sec"),
        },
    )

    move_group = _include(MOVEIT_CONFIG, "move_group.launch.py")

    moveit_rviz = _include(
        MOVEIT_CONFIG,
        "moveit_rviz.launch.py",
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription(
        args + [run_emulator, rsp, robot_control, move_group, moveit_rviz]
    )
