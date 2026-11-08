#!/usr/bin/env python3

from pathlib import Path
import re

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    here = Path(__file__).resolve().parent
    urdf_path = here / "urdf" / "humanoid_gazebo.urdf"
    controller_yaml = here / "humanoid_controllers.yaml"
    world_path = Path("/usr/share/gazebo-11/worlds/empty.world")

    for path in (urdf_path, controller_yaml, world_path):
        if not path.is_file():
            raise FileNotFoundError(f"Required file not found: {path}")

    # Remove XML declaration before publishing robot_description as a string.
    robot_description = urdf_path.read_text(encoding="utf-8-sig")
    robot_description = re.sub(
        r"^\s*<\?xml[^>]*\?>\s*",
        "",
        robot_description,
        count=1,
    )
    robot_description = robot_description.replace(
        "__CONTROLLER_YAML__", str(controller_yaml)
    )

    gazebo = ExecuteProcess(
        cmd=[
            "gazebo", "--verbose", str(world_path),
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

    def controller_spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
        )

    return LaunchDescription([
        gazebo,
        state_publisher,
        TimerAction(period=5.0, actions=[spawn_robot]),
        TimerAction(period=10.0, actions=[
            controller_spawner("joint_state_broadcaster")
        ]),
        TimerAction(period=12.0, actions=[
            controller_spawner("camera_yaw_position_controller")
        ]),
        TimerAction(period=14.0, actions=[
            controller_spawner("camera_pitch_position_controller")
        ]),
        TimerAction(period=16.0, actions=[
            controller_spawner("right_arm_position_controller")
        ]),
        TimerAction(period=18.0, actions=[
            controller_spawner("left_arm_position_controller")
        ]),
    ])
