
#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    here = Path(__file__).resolve().parent

    # New camera-only URDF
    urdf_path = here / "camera_mount.urdf"

    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    robot_description = urdf_path.read_text()

    # Controller configuration
    controller_yaml = Path("/tmp/abhi_camera_controllers.yaml")

    controller_yaml.write_text("""
controller_manager:
  ros__parameters:
    update_rate: 100
    use_sim_time: true

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    camera_yaw_position_controller:
      type: position_controllers/JointGroupPositionController

    camera_pitch_position_controller:
      type: position_controllers/JointGroupPositionController

camera_yaw_position_controller:
  ros__parameters:
    joints:
      - revolute_18
    interface_name: position

camera_pitch_position_controller:
  ros__parameters:
    joints:
      - revolute_19
    interface_name: position
""")

    # Start Gazebo Classic
    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "empty.world",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so"
        ],
        output="screen"
    )

    # Publish robot transforms
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }],
        output="screen"
    )

    # Spawn camera assembly
    spawn_camera = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "abhi_camera",
            "-x", "0",
            "-y", "0",
            "-z", "0.02"
        ],
        output="screen"
    )

    # Joint state broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # Revolute 18 controller
    yaw_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "camera_yaw_position_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # Revolute 19 controller
    pitch_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "camera_pitch_position_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,

        TimerAction(
            period=4.0,
            actions=[spawn_camera]
        ),

        TimerAction(
            period=8.0,
            actions=[joint_state_broadcaster]
        ),

        TimerAction(
            period=10.0,
            actions=[yaw_controller]
        ),

        TimerAction(
            period=12.0,
            actions=[pitch_controller]
        )
    ])


if __name__ == "__main__":
    from launch import LaunchService

    service = LaunchService()
    service.include_launch_description(
        generate_launch_description()
    )
    raise SystemExit(service.run())
