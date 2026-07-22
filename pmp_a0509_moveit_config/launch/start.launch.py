# robot + MoveIt layer, no perception/grasp. thin composition of the component
# launches: the dsr emulator (virtual mode only), rsp (arg'd description),
# robot_control (control_node + spawners + drcf gate), move_group and RViz.
# useful standalone (bring up arm + gripper + MoveIt); also the layer bringup
# composes. every event-handler chain lives inside its component (decision #1).
#
# `namespace` is single param that lets multiple stacks share ROS domain.
# GroupAction pushes namespace and remaps /tf + /tf_static into it, giving each
# stack own node namespace and TF tree, following doosan's multi-robot approach
# (unique name + port, remap_tf) - no tf_prefix/prefix support, controller
# hardcodes joint names, hardware rejects extra params
#
# instanced stacks won't share frames - no planning across both arms; one RViz
# instance per stack; etc.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace, SetRemap
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
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="instance namespace for every node, topic, service and "
            "this stack's own TF tree. set it (per stack) to run more than one "
            "a0509+rg2 stack on one ROS domain",
        ),
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
        DeclareLaunchArgument(
            "camera_name",
            default_value="zed",
            description="ZED camera_name the matching perception layer was given "
            "(see perception.launch.py). the ZED cannot be pushed into our "
            "namespace, so move_group needs this to resolve its octomap topics",
        ),
        DeclareLaunchArgument(
            "drcf_ready_timeout_sec",
            default_value="30.0",
            description="max wait for the drcf endpoint (host:port) to accept "
            "connections before control_node starts",
        ),
    ]

    namespace = LaunchConfiguration("namespace")

    is_virtual = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mode"), "' == 'virtual'"])
    )

    # dsr emulator provides the drcf endpoint in virtual mode; robot_control's
    # gate polls that endpoint regardless of who provides it. `name` is the
    # vendor's own param name and is what run_emulator derives its container name
    # from (see dsr_bringup2/run_emulator.py) - must stay unique per stack
    run_emulator = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        parameters=[
            {"name": namespace},
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
            "host": LaunchConfiguration("host"),
            "port": LaunchConfiguration("port"),
            "mode": LaunchConfiguration("mode"),
            "rt_host": LaunchConfiguration("rt_host"),
            "rg2_use_fake_hardware": LaunchConfiguration("rg2_use_fake_hardware"),
            "rg2_ip_address": LaunchConfiguration("rg2_ip_address"),
            "rg2_port": LaunchConfiguration("rg2_port"),
            "camera_model": LaunchConfiguration("camera_model"),
        },
    )

    robot_control = _include(
        MOVEIT_CONFIG,
        "robot_control.launch.py",
        {
            "namespace": namespace,
            "host": LaunchConfiguration("host"),
            "port": LaunchConfiguration("port"),
            "controller_log_level": LaunchConfiguration("controller_log_level"),
            "drcf_ready_timeout_sec": LaunchConfiguration("drcf_ready_timeout_sec"),
        },
    )

    move_group = _include(
        MOVEIT_CONFIG,
        "move_group.launch.py",
        {"camera_name": LaunchConfiguration("camera_name")},
    )

    moveit_rviz = _include(
        MOVEIT_CONFIG,
        "moveit_rviz.launch.py",
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    # single GroupAction covered by PushRosNamespace, and SetRemaps to put tf2s
    # absolute /tf and /tf_static onto namespaced tf tree
    #
    # robot_control deliberately not in this group: it creates its nodes from an
    # event handler - outlives the group's scope - it takes `namespace` param and
    # applies it itself
    stack = GroupAction(
        [
            PushRosNamespace(namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            run_emulator,
            rsp,
            move_group,
            moveit_rviz,
        ]
    )

    return LaunchDescription(args + [stack, robot_control])
