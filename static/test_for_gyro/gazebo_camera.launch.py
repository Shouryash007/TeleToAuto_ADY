from pathlib import Path
import tempfile

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    here = Path(__file__).resolve().parent
    urdf = here / "camera_mount.urdf"

    # Keep the deliverable to three source files: create the controller YAML at launch time.
    controller_yaml = Path("/tmp/gyro_camera_controllers.yaml")
    controller_yaml.write_text("""controller_manager:
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
    joints: [camera_yaw_joint]
    interface_name: position
camera_pitch_position_controller:
  ros__parameters:
    joints: [camera_pitch_joint]
    interface_name: position
""")

    description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", str(urdf)]),
        value_type=str
    )

    gazebo = ExecuteProcess(
        cmd=[
            "gazebo", "--verbose", "empty.world",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen"
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": description, "use_sim_time": True}],
        output="screen"
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "gyro_camera_mount",
                   "-x", "0", "-y", "0", "-z", "0.02"],
        output="screen"
    )

    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
               output="screen")
    yaw = Node(package="controller_manager", executable="spawner",
               arguments=["camera_yaw_position_controller", "--controller-manager", "/controller_manager"],
               output="screen")
    pitch = Node(package="controller_manager", executable="spawner",
                 arguments=["camera_pitch_position_controller", "--controller-manager", "/controller_manager"],
                 output="screen")

    return LaunchDescription([
        gazebo, rsp,
        TimerAction(period=3.0, actions=[spawn]),
        TimerAction(period=6.0, actions=[jsb]),
        TimerAction(period=7.0, actions=[yaw]),
        TimerAction(period=8.0, actions=[pitch]),
    ])


if __name__ == "__main__":
    from launch import LaunchService
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    raise SystemExit(launch_service.run())
