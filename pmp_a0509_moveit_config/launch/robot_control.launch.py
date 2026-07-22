# ros2_control layer: one ros2_control_node loads the combined dsr arm + rg2
# gripper controllers on a single controller_manager - gated on the drcf
# endpoint being reachable. vendor-combined by necessity: arm and gripper
# share one CM, so they cannot be split into per-vendor launches.
#
# robot_description is taken from the robot_description topic (published by
# rsp.launch.py), so this launch does not recompute the xacro.
#
# applies `namespace` to its own nodes instead of inheriting a PushRosNamespace
# group from start.launch.py, since OnProcessExit handler and returned actions
# execute after the GroupAction's scope has popped

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="instance namespace for control_node and the spawners; "
            "must match the rest of the stack (see start.launch.py)",
        ),
        DeclareLaunchArgument(
            "host", default_value="127.0.0.1", description="ROBOT_IP (drcf endpoint)"
        ),
        DeclareLaunchArgument(
            "port", default_value="12345", description="ROBOT_PORT (drcf endpoint)"
        ),
        DeclareLaunchArgument(
            "controller_log_level",
            default_value="warn",
            description="ros2_control_node log-level",
        ),
        DeclareLaunchArgument(
            "drcf_ready_timeout_sec",
            default_value="30.0",
            description="max wait for the drcf endpoint (host:port) to accept "
            "connections before control_node starts. applies in both real and "
            "virtual modes - only mode:=virtual has meaningful startup latency",
        ),
    ]

    robot_controllers = [
        PathJoinSubstitution(
            [
                FindPackageShare("dsr_controller2"),
                "config",
                "dsr_controller2.yaml",
            ]
        ),
        # layer the RG2 controllers onto the same controller_manager so the
        # gripper shares the Doosan CM
        PathJoinSubstitution(
            [
                FindPackageShare("pmp_a0509_moveit_config"),
                "config",
                "gripper_controllers.yaml",
            ]
        ),
    ]

    # gate control_node on the drcf endpoint (host:port) accepting connections;
    # prevents DRHWInterface throwing if it connects before the emulator starts
    # (mode:=virtual). polls whatever is listening - real robot or emulator
    wait_for_drcf = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution(
                [
                    FindPackageShare("pmp_a0509_moveit_config"),
                    "scripts",
                    "wait_for_tcp_port.py",
                ]
            ),
            LaunchConfiguration("host"),
            LaunchConfiguration("port"),
            LaunchConfiguration("drcf_ready_timeout_sec"),
        ],
        output="screen",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration("namespace"),
        # robot_description comes from rsp's latched robot_description topic;
        # remap the node-relative subscription onto it - both are relative,
        # this resolves inside whatever namespace the stack was pushed into
        remappings=[("~/robot_description", "robot_description")],
        parameters=[*robot_controllers],
        output="both",
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("controller_log_level"),
        ],
    )

    def spawner(controller):
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace=LaunchConfiguration("namespace"),
            arguments=[
                controller,
                "-c",
                "controller_manager",
                "--controller-manager-timeout",
                "15",
            ],
        )

    spawners = [
        spawner("joint_state_broadcaster"),
        spawner("dsr_controller2"),
        spawner("dsr_moveit_controller"),
        spawner("finger_width_controller"),
        spawner("finger_width_effort_controller"),
    ]

    # start control_node and the CM-dependent spawners once the drcf
    # endpoint is confirmed reachable; abort launch on timeout
    def on_wait_for_drcf_exit(event, context):
        if event.returncode == 0:
            return [control_node, *spawners]
        return [
            LogInfo(
                msg="drcf endpoint did not become reachable within "
                "drcf_ready_timeout_sec; aborting bringup, control_node was "
                "never started"
            ),
            Shutdown(reason="drcf endpoint unreachable"),
        ]

    start_control_node_after_drcf_ready = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_drcf,
            on_exit=on_wait_for_drcf_exit,
        )
    )

    return LaunchDescription(
        args + [wait_for_drcf, start_control_node_after_drcf_ready]
    )
