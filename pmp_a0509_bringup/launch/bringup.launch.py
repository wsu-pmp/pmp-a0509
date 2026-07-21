# top-level orchestration: composes the robot+MoveIt layer (start.launch.py),
# the shared ZED sensor (perception.launch.py), and the one-shot safety-scene
# loader

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
        # dsr arm
        DeclareLaunchArgument("dsr_mode", default_value="virtual"),
        DeclareLaunchArgument("dsr_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("dsr_port", default_value="12345"),
        DeclareLaunchArgument("dsr_rt_host", default_value="192.168.137.50"),
        DeclareLaunchArgument("dsr_controller_log_level", default_value="warn"),
        # rg2 gripper
        DeclareLaunchArgument("rg2_use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("rg2_ip_address", default_value="192.168.1.1"),
        DeclareLaunchArgument("rg2_port", default_value="502"),
        # zed perception
        DeclareLaunchArgument("zed_enabled", default_value="true"),
        DeclareLaunchArgument("zed_camera_model", default_value="zed2i"),
        # composition
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("drcf_ready_timeout_sec", default_value="30.0"),
    ]

    start = _include(
        MOVEIT_CONFIG,
        "start.launch.py",
        {
            "host": LaunchConfiguration("dsr_host"),
            "port": LaunchConfiguration("dsr_port"),
            "mode": LaunchConfiguration("dsr_mode"),
            "rt_host": LaunchConfiguration("dsr_rt_host"),
            "controller_log_level": LaunchConfiguration("dsr_controller_log_level"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "onrobot_use_fake_hardware": LaunchConfiguration("rg2_use_fake_hardware"),
            "onrobot_ip_address": LaunchConfiguration("rg2_ip_address"),
            "onrobot_port": LaunchConfiguration("rg2_port"),
            "drcf_ready_timeout_sec": LaunchConfiguration("drcf_ready_timeout_sec"),
        },
    )

    perception = _include(
        "pmp_a0509_bringup",
        "perception.launch.py",
        {"camera_model": LaunchConfiguration("zed_camera_model")},
        condition=IfCondition(LaunchConfiguration("zed_enabled")),
    )

    # one-shot safety collision objects; sequences itself via wait_for_service on
    # /apply_planning_scene, so it needs no ordering relative to move_group
    safety_scene = PathJoinSubstitution(
        [FindPackageShare(MOVEIT_CONFIG), "config", "safety_scene.yaml"]
    )
    safety_scene_loader = Node(
        package="planning_scene_loader",
        executable="planning_scene_loader_node",
        name="planning_scene_loader",
        output="screen",
        parameters=[{"config_file": safety_scene}],
    )

    return LaunchDescription(args + [start, perception, safety_scene_loader])
