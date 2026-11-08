#!/usr/bin/env python3
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    here = Path(__file__).resolve().parent
    urdf_path = here / "urdf" / "humanoid_gazebo.urdf"
    controller_yaml = here / "humanoid_controllers.yaml"

    for path in (urdf_path, controller_yaml):
        if not path.is_file():
            raise FileNotFoundError(f"Required file not found: {path}")

    # Inject the absolute controller YAML path into the Gazebo ROS 2 control plugin.
    robot_description = urdf_path.read_text().replace(
        "__CONTROLLER_YAML__", str(controller_yaml)
    )

    gazebo = ExecuteProcess(
        cmd=[
            "gazebo", "--verbose", "empty.world",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
        output="screen",
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "ABHI_humanoid",
            "-x", "0", "-y", "0", "-z", "0.05",
        ],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )
    yaw_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "camera_yaw_position_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )
    pitch_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "camera_pitch_position_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        state_publisher,
        TimerAction(period=5.0, actions=[spawn_robot]),
        TimerAction(period=9.0, actions=[joint_state_spawner]),
        TimerAction(period=11.0, actions=[yaw_spawner]),
        TimerAction(period=13.0, actions=[pitch_spawner]),
    ])
