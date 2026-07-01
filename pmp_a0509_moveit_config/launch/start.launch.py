# Modified from
# https://raw.githubusercontent.com/DoosanRobotics/doosan-robot2/refs/heads/humble/dsr_moveit2/dsr_moveit_config_a0509/launch/start.launch.py

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from dsr_bringup2.utils import read_update_rate
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Moveit2
from moveit_configs_utils import MoveItConfigsBuilder


def rviz_node_function(context):
    """Evaluate the model value at launch time, find the package path, and then execute the launch file"""
    # CHANGE: substitue package name and remove dynamic model_value
    # model_value = LaunchConfiguration('model').perform(context)

    # model_value_str = f"{model_value}"
    package_name_str = "pmp_a0509_moveit_config"

    # Get the package path using FindPackageShare
    package_path_str = FindPackageShare(package_name_str).perform(context)

    print("Package name:", package_name_str)
    print("Package path:", package_path_str)

    # CHANGE: use default pmp_a0509_moveit_config MoveItConfigsBuilder
    moveit_config = MoveItConfigsBuilder(
        "a0509", package_name=package_name_str
    ).to_moveit_configs()

    configs_dump_path = Path(
        get_package_share_directory(package_name_str),
        "config",
        "moveit_configs.yaml",
    )

    configs_dump_path.parent.mkdir(parents=True, exist_ok=True)
    with configs_dump_path.open("w") as f:
        yaml.dump({"/**": {"ros__parameters": moveit_config.to_dict()}}, f)

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        # namespace=LaunchConfiguration('name'),
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # RViz
    # CHANGE: rviz config share dir launch -> rviz
    rviz_base = os.path.join(get_package_share_directory(package_name_str), "rviz")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    return [
        run_move_group_node,
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            # namespace=LaunchConfiguration('name'),
            output="log",
            arguments=["-d", rviz_full_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        ),
    ]


def generate_launch_description():
    # CHANGE: remove model launch arg
    #         remove color launch arg
    #         add controller_log_level launch arg
    #         add launch_zed launch arg
    ARGUMENTS = [
        DeclareLaunchArgument("name", default_value="", description="NAME_SPACE"),
        DeclareLaunchArgument(
            "host", default_value="127.0.0.1", description="ROBOT_IP"
        ),
        DeclareLaunchArgument("port", default_value="12345", description="ROBOT_PORT"),
        DeclareLaunchArgument(
            "mode", default_value="virtual", description="OPERATION MODE"
        ),
        # DeclareLaunchArgument('model', default_value = 'm0617',     description = 'ROBOT_MODEL'    ),
        # DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'    ),
        DeclareLaunchArgument("gui", default_value="false", description="Start RViz2"),
        DeclareLaunchArgument(
            "gz", default_value="false", description="USE GAZEBO SIM"
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
            "launch_zed",
            default_value="true",
            description="Launch zed_wrapper (zed2i)?",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        ),
        # OnRobot RG2 gripper integration. Defaults keep the gripper on fake
        # hardware so this launch stays fully virtual/testable without the
        # OnRobot Compute Box. Set onrobot_use_fake_hardware:=false to drive the
        # physical gripper (requires the Compute Box reachable at the address
        # below).
        DeclareLaunchArgument(
            "onrobot_use_fake_hardware",
            default_value="true",
            description="Use fake hardware for the RG2 gripper?",
        ),
        DeclareLaunchArgument(
            "onrobot_connection_type",
            default_value="tcp",
            description="RG2 connection type (tcp for Compute Box, serial for UR Tool I/O).",
            choices=["serial", "tcp"],
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
    ]

    update_rate = str(read_update_rate())  # get update_rate from yaml

    # CHANGE: substitute pmp_a0509_description in control_node
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
            " name:=",
            LaunchConfiguration("name"),
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
            " onrobot_use_fake_hardware:=",
            LaunchConfiguration("onrobot_use_fake_hardware"),
            " onrobot_connection_type:=",
            LaunchConfiguration("onrobot_connection_type"),
            " onrobot_ip_address:=",
            LaunchConfiguration("onrobot_ip_address"),
            " onrobot_port:=",
            LaunchConfiguration("onrobot_port"),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = [
        PathJoinSubstitution(
            [
                FindPackageShare("dsr_controller2"),
                "config",
                "dsr_controller2.yaml",
            ]
        ),
        # CHANGE: layer the RG2 finger_width_controller onto the same
        # controller_manager so the gripper shares the Doosan CM
        PathJoinSubstitution(
            [
                FindPackageShare("pmp_a0509_moveit_config"),
                "config",
                "gripper_controllers.yaml",
            ]
        ),
    ]

    # CHANGE: hardcode model a0509
    run_emulator_node = Node(
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
            # parameters_file_path       # If a parameter is set in both the launch file and a YAML file, the value from the YAML file will be used.
        ],
        output="screen",
    )

    # CHANGE: add controller_log_level arg to control_node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration("name"),
        # NOTE: each controller param file must be a separate element in
        # `parameters`. Nesting them in one sub-list makes launch_ros join the
        # paths into a single (invalid) string, so spread robot_controllers.
        parameters=[robot_description, *robot_controllers],
        output="both",
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("controller_log_level"),
        ],
    )

    # CHANGE: substitute pmp_a0509_description in robot_state_pub_node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=LaunchConfiguration("name"),
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration("name"),
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration("name"),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
    )

    dsr_moveit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("name"),
        arguments=[
            "dsr_moveit_controller",
            "-c",
            "controller_manager",
        ],
    )

    # CHANGE: spawn the RG2 finger_width_controller on the shared CM
    finger_width_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("name"),
        arguments=[
            "finger_width_controller",
            "-c",
            "controller_manager",
        ],
    )

    # Moveit2 config
    rviz_node = OpaqueFunction(function=rviz_node_function)

    # CHANGE: zed_wrapper
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
            "camera_model": "zed2i",
            "object_detection.od_enabled": "true",
            "body_tracking.bt_enabled": "true",
            "pos_tracking.pos_tracking_enabled": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_zed")),
    )

    enable_zed_services = TimerAction(
        period=5.0,  # delay service call
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
            ),
        ],
        condition=IfCondition(LaunchConfiguration("launch_zed")),
    )

    # joint_trajectory_controller_spawner = Node(
    #     package="controller_manager",
    #     # namespace=LaunchConfiguration('name'),
    #     executable="spawner",
    #     arguments=["dsr_joint_trajectory", "-c", "dsr/controller_manager", "-n", "dsr"],
    # )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    # # Delay start of robot_controller after `joint_state_broadcaster`
    # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[robot_controller_spawner],
    #     )
    # )

    # CHANGE: add zed_wrapper
    nodes = [
        run_emulator_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        joint_state_broadcaster_spawner,
        dsr_moveit_controller_spawner,
        finger_width_controller_spawner,
        control_node,
        zed_wrapper,
        enable_zed_services,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
