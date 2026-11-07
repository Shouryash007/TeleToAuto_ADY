#!/usr/bin/env python3
"""
so101_gazebo.launch.py -- Spawn ONE SO-101 follower arm in Gazebo (gz sim) with ros2_control
and (optionally) start hand_ik_teleop.py once the controllers are up.

Run (no colcon package needed):
  source /opt/ros/<distro>/setup.bash
  ros2 launch ./so101_gazebo.launch.py urdf_file:=/home/you/path/so101_new_calib.urdf

What it does:
  1. Reads your plain URDF and writes a Gazebo-ready copy to /tmp
       - mesh paths 'assets/xxx.stl' -> absolute file:// URIs (fixes "could not load mesh resource")
       - removes the ROS-1 <transmission> blocks
       - fixes base_link to the world
       - adds the <ros2_control> block + gz_ros2_control plugin
  2. Writes the controller YAML to /tmp
  3. Starts Gazebo, robot_state_publisher, spawns the arm, loads controllers:
       joint_state_broadcaster
       arm_position_controller      -> /arm_position_controller/commands      (5 joints, Float64MultiArray)
       gripper_position_controller  -> /gripper_position_controller/commands  (1 joint,  Float64MultiArray)
  4. Starts hand_ik_teleop.py (same folder as this file) after the controllers are active.

Arguments:
  urdf_file    path to so101_new_calib.urdf (assets/ folder must sit next to it)
  teleop       true|false   start hand_ik_teleop.py automatically (default true)
  gui          true|false   show the Gazebo window (default true)
  world        world file, default empty.sdf
  gz_flavor    auto|gz|ign  gz = Gazebo Harmonic + gz_ros2_control (ROS Jazzy and newer)
                            ign = Gazebo Fortress + ign_ros2_control (ROS Humble)
  gravity      true|false   keep gravity on the arm links (default false: position-held teleop arm)
  camera_index webcam index passed to the teleop node
"""
import os
import re
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER_JOINTS = ["gripper"]
ROBOT_NAME = "so101"
DEFAULT_URDF_CANDIDATES = [
    os.environ.get("SO101_URDF", ""),
    "~/teleToAuto_ADY/static/so101/so101_new_calib.urdf",
    "~/so101/so101_new_calib.urdf",
    "./so101_new_calib.urdf",
]


def detect_flavor():
    distro = os.environ.get("ROS_DISTRO", "")
    return "ign" if distro in ("foxy", "galactic", "humble") else "gz"


def make_gazebo_urdf(text, urdf_dir, yaml_path, flavor, gravity):
    """Return the URDF string with Gazebo + ros2_control additions."""
    # 1. mesh paths -> absolute file:// URIs
    def fix(m):
        path = m.group(1)
        if re.match(r"^(file://|package://|https?://|model://)", path):
            return m.group(0)
        return 'filename="file://%s"' % os.path.normpath(os.path.join(urdf_dir, path))
    text = re.sub(r'filename="([^"]+)"', fix, text)

    # 2. drop ROS-1 transmissions (ros2_control block replaces them)
    text = re.sub(r"<transmission\b.*?</transmission>", "", text, flags=re.DOTALL)

    # 3. discover the root link and every movable joint straight from the file
    root = ET.fromstring(text)
    children = {j.find("child").get("link") for j in root.findall("joint")}
    links = [l.get("name") for l in root.findall("link")]
    base_link = next(l for l in links if l not in children)
    joints = [j.get("name") for j in root.findall("joint") if j.get("type") in ("revolute", "continuous", "prismatic")]

    if flavor == "ign":
        hw_plugin = "ign_ros2_control/IgnitionSystem"
        sim_plugin = ('<plugin filename="ign_ros2_control-system" '
                      'name="ign_ros2_control::IgnitionROS2ControlPlugin">')
    else:
        hw_plugin = "gz_ros2_control/GazeboSimSystem"
        sim_plugin = ('<plugin filename="gz_ros2_control-system" '
                      'name="gz_ros2_control::GazeboSimROS2ControlPlugin">')

    joint_xml = "".join(
        f'\n    <joint name="{j}">'
        '\n      <command_interface name="position"/>'
        '\n      <state_interface name="position"/>'
        '\n      <state_interface name="velocity"/>'
        '\n    </joint>' for j in joints)

    gravity_xml = "" if gravity else "".join(
        f'\n  <gazebo reference="{l}"><gravity>false</gravity></gazebo>' for l in links)

    extra = f"""
  <!-- ===== added by so101_gazebo.launch.py ===== -->
  <link name="world"/>
  <joint name="world_to_{base_link}" type="fixed">
    <parent link="world"/>
    <child link="{base_link}"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
{gravity_xml}
  <ros2_control name="so101_system" type="system">
    <hardware>
      <plugin>{hw_plugin}</plugin>
    </hardware>{joint_xml}
  </ros2_control>

  <gazebo>
    {sim_plugin}
      <parameters>{yaml_path}</parameters>
    </plugin>
  </gazebo>
"""
    idx = text.rfind("</robot>")
    if idx < 0:
        raise RuntimeError("URDF has no closing </robot> tag")
    return text[:idx] + extra + text[idx:], joints


def make_controller_yaml(joints):
    arm = [j for j in ARM_JOINTS if j in joints]
    grip = [j for j in GRIPPER_JOINTS if j in joints]
    return f"""controller_manager:
  ros__parameters:
    update_rate: 100
    use_sim_time: true

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_position_controller:
      type: forward_command_controller/ForwardCommandController

    gripper_position_controller:
      type: forward_command_controller/ForwardCommandController

arm_position_controller:
  ros__parameters:
    joints: {arm}
    interface_name: position

gripper_position_controller:
  ros__parameters:
    joints: {grip}
    interface_name: position
"""


def find_urdf(path):
    candidates = [path] if path else DEFAULT_URDF_CANDIDATES
    for c in candidates:
        if c:
            full = os.path.abspath(os.path.expanduser(c))
            if os.path.isfile(full):
                return full
    raise RuntimeError(
        "Could not find the URDF. Pass urdf_file:=/full/path/to/so101_new_calib.urdf "
        "(tried: %s)" % ", ".join(c for c in candidates if c))


def launch_setup(context):
    urdf_arg = LaunchConfiguration("urdf_file").perform(context)
    flavor = LaunchConfiguration("gz_flavor").perform(context)
    flavor = detect_flavor() if flavor == "auto" else flavor
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    gravity = LaunchConfiguration("gravity").perform(context).lower() == "true"
    teleop = LaunchConfiguration("teleop").perform(context).lower() == "true"
    world = LaunchConfiguration("world").perform(context)
    camera_index = LaunchConfiguration("camera_index").perform(context)

    urdf_path = find_urdf(urdf_arg)
    urdf_dir = os.path.dirname(urdf_path)
    assets = os.path.join(urdf_dir, "assets")
    if not os.path.isdir(assets):
        print(f"[so101_gazebo] WARNING: no 'assets' folder next to the URDF ({assets}); meshes will not load")

    yaml_path = "/tmp/so101_controllers.yaml"
    urdf_out = "/tmp/so101_gazebo.urdf"
    with open(urdf_path, "r") as f:
        text = f.read()
    robot_description, joints = make_gazebo_urdf(text, urdf_dir, yaml_path, flavor, gravity)
    with open(urdf_out, "w") as f:
        f.write(robot_description)
    with open(yaml_path, "w") as f:
        f.write(make_controller_yaml(joints))
    print(f"[so101_gazebo] flavor={flavor}  urdf={urdf_path}  -> {urdf_out}")

    gz_args = f"-r -v 2 {world}" + ("" if gui else " -s")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={"gz_args": gz_args}.items())

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
               parameters=[{"robot_description": robot_description, "use_sim_time": True}])

    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", "robot_description", "-name", ROBOT_NAME, "-x", "0", "-y", "0", "-z", "0.0"])

    clock_type = "ignition.msgs.Clock" if flavor == "ign" else "gz.msgs.Clock"
    clock_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                        arguments=[f"/clock@rosgraph_msgs/msg/Clock[{clock_type}"])

    def spawner(name):
        return Node(package="controller_manager", executable="spawner", output="screen",
                    arguments=[name, "--controller-manager", "/controller_manager", "--controller-manager-timeout", "120"])

    jsb = spawner("joint_state_broadcaster")
    arm = spawner("arm_position_controller")
    grip = spawner("gripper_position_controller")

    actions = [
        gz_sim, clock_bridge, rsp, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, grip])),
    ]

    if teleop:
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand_ik_teleop.py")
        teleop_proc = ExecuteProcess(
            cmd=["python3", script, "--ros-args",
                 "-p", f"urdf_file:={urdf_path}",
                 "-p", f"camera_index:={camera_index}"],
            output="screen")
        actions.append(RegisterEventHandler(OnProcessExit(target_action=grip, on_exit=[teleop_proc])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("urdf_file", default_value="", description="Path to so101_new_calib.urdf"),
        DeclareLaunchArgument("teleop", default_value="true", description="Start hand_ik_teleop.py"),
        DeclareLaunchArgument("gui", default_value="true", description="Show the Gazebo GUI"),
        DeclareLaunchArgument("world", default_value="empty.sdf", description="Gazebo world file"),
        DeclareLaunchArgument("gz_flavor", default_value="auto", description="auto | gz | ign"),
        DeclareLaunchArgument("gravity", default_value="false", description="Keep gravity on the arm links"),
        DeclareLaunchArgument("camera_index", default_value="0", description="Webcam index for the teleop node"),
        OpaqueFunction(function=launch_setup),
    ])
