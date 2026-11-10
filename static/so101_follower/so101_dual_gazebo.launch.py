#!/usr/bin/env python3
"""
so101_dual_gazebo.launch.py -- Spawn TWO SO-101 follower arms (left + right) in Gazebo with
ros2_control and (optionally) start dual_hand_ik_teleop.py once the controllers are active.

Run it with ros2 (NOT with python3), from a terminal where ROS is sourced and NO venv is active:
  source /opt/ros/humble/setup.bash
  ros2 launch ./so101_dual_gazebo.launch.py urdf_file:=$PWD/so101_new_calib.urdf camera_index:=1

What it does:
  1. Reads your single-arm URDF and builds ONE two-arm robot from it in /tmp:
       - every link/joint is duplicated with a  left_  /  right_  prefix
       - left arm at y=+separation/2, right arm at y=-separation/2, both fixed to the world
       - mesh paths 'assets/xxx.stl' -> absolute file:// URIs
       - ROS-1 <transmission> blocks removed, <ros2_control> + Gazebo plugin added
       - right arm is tinted blue so you can tell the arms apart
  2. Starts Gazebo, robot_state_publisher, spawns the robot, then loads these controllers
     in ONE spawner call:
       joint_state_broadcaster
       left_arm_position_controller   left_gripper_position_controller
       right_arm_position_controller  right_gripper_position_controller
  3. If (and only if) the controllers loaded, starts dual_hand_ik_teleop.py
     (it must sit in the same folder as this file).

Arguments:
  urdf_file     path to so101_new_calib.urdf (its assets/ folder must be next to it)
  separation    distance between the two arm bases in metres (default 0.40)
  teleop        true|false  start dual_hand_ik_teleop.py automatically (default true)
  gui           true|false  show the Gazebo window (default true)
  world         world file (default empty.sdf)
  gz_flavor     auto|gz|ign  ign = Gazebo Fortress / ign_ros2_control (ROS Humble)
                             gz  = Gazebo Harmonic / gz_ros2_control (ROS Jazzy+)
  gravity       true|false  keep gravity on the arm links (default false)
  camera_index  webcam index passed to the teleop node
"""
import copy
import os
import re
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER_JOINTS = ["gripper"]
SIDES = ("left", "right")
ROBOT_NAME = "so101_dual"
CONTROLLERS = ["joint_state_broadcaster"] + [
    f"{s}_{kind}_position_controller" for s in SIDES for kind in ("arm", "gripper")]
DEFAULT_URDF_CANDIDATES = [
    os.environ.get("SO101_URDF", ""),
    "~/teleToAuto_ADY/static/so101_follower/so101_new_calib.urdf",
    "~/teleToAuto_ADY/static/so101/so101_new_calib.urdf",
    "./so101_new_calib.urdf",
]


def detect_flavor():
    distro = os.environ.get("ROS_DISTRO", "")
    return "ign" if distro in ("foxy", "galactic", "humble") else "gz"


def make_dual_urdf(text, urdf_dir, yaml_path, flavor, gravity, separation):
    """Build the two-arm Gazebo URDF string from a single-arm URDF. Returns (urdf, joint_names)."""
    # 1. mesh paths -> absolute file:// URIs
    def fix(m):
        path = m.group(1)
        if re.match(r"^(file://|package://|https?://|model://)", path):
            return m.group(0)
        return 'filename="file://%s"' % os.path.normpath(os.path.join(urdf_dir, path))
    text = re.sub(r'filename="([^"]+)"', fix, text)

    # 2. drop ROS-1 transmissions
    text = re.sub(r"<transmission\b.*?</transmission>", "", text, flags=re.DOTALL)

    src = ET.fromstring(text)
    children = {j.find("child").get("link") for j in src.findall("joint")}
    base_link = next(l.get("name") for l in src.findall("link") if l.get("name") not in children)

    out = ET.Element("robot", name=ROBOT_NAME)
    for m in src.findall("material"):
        out.append(copy.deepcopy(m))
    # a blue variant for the right arm
    blue = ET.SubElement(out, "material", name="3d_printed_right")
    ET.SubElement(blue, "color", rgba="0.15 0.45 0.9 1.0")
    ET.SubElement(out, "link", name="world")

    all_links, all_joints = [], []
    for side, y in (("left", +separation / 2.0), ("right", -separation / 2.0)):
        pre = side + "_"
        for el in src:
            if el.tag == "link":
                new = copy.deepcopy(el)
                new.set("name", pre + el.get("name"))
                if side == "right":
                    for mat in new.iter("material"):
                        if mat.get("name") == "3d_printed":
                            mat.set("name", "3d_printed_right")
                out.append(new)
                all_links.append(new.get("name"))
            elif el.tag == "joint":
                new = copy.deepcopy(el)
                new.set("name", pre + el.get("name"))
                new.find("parent").set("link", pre + el.find("parent").get("link"))
                new.find("child").set("link", pre + el.find("child").get("link"))
                out.append(new)
                if new.get("type") in ("revolute", "continuous", "prismatic"):
                    all_joints.append(new.get("name"))
        mount = ET.SubElement(out, "joint", name=f"world_to_{pre}{base_link}", type="fixed")
        ET.SubElement(mount, "parent", link="world")
        ET.SubElement(mount, "child", link=pre + base_link)
        ET.SubElement(mount, "origin", xyz=f"0 {y:.4f} 0", rpy="0 0 0")

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
        '\n    </joint>' for j in all_joints)
    gravity_xml = "" if gravity else "".join(
        f'\n  <gazebo reference="{l}"><gravity>false</gravity></gazebo>' for l in all_links)

    extra = f"""
  <!-- ===== added by so101_dual_gazebo.launch.py ===== -->
{gravity_xml}
  <ros2_control name="so101_dual_system" type="system">
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
    body = ET.tostring(out, encoding="unicode")
    idx = body.rfind("</robot>")
    return '<?xml version="1.0"?>\n' + body[:idx] + extra + body[idx:], all_joints


def make_controller_yaml():
    lines = ["controller_manager:", "  ros__parameters:", "    update_rate: 100", "    use_sim_time: true", "",
             "    joint_state_broadcaster:", "      type: joint_state_broadcaster/JointStateBroadcaster", ""]
    for s in SIDES:
        for kind in ("arm", "gripper"):
            lines += [f"    {s}_{kind}_position_controller:",
                      "      type: forward_command_controller/ForwardCommandController", ""]
    for s in SIDES:
        arm = [f"{s}_{j}" for j in ARM_JOINTS]
        grip = [f"{s}_{j}" for j in GRIPPER_JOINTS]
        lines += [f"{s}_arm_position_controller:", "  ros__parameters:", f"    joints: {arm}",
                  "    interface_name: position", "",
                  f"{s}_gripper_position_controller:", "  ros__parameters:", f"    joints: {grip}",
                  "    interface_name: position", ""]
    return "\n".join(lines) + "\n"


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
    separation = float(LaunchConfiguration("separation").perform(context))

    urdf_path = find_urdf(urdf_arg)
    urdf_dir = os.path.dirname(urdf_path)
    if not os.path.isdir(os.path.join(urdf_dir, "assets")):
        print(f"[so101_dual] WARNING: no 'assets' folder next to the URDF ({urdf_dir}); meshes will not load")

    yaml_path = "/tmp/so101_dual_controllers.yaml"
    urdf_out = "/tmp/so101_dual_gazebo.urdf"
    with open(urdf_path, "r") as f:
        text = f.read()
    robot_description, _joints = make_dual_urdf(text, urdf_dir, yaml_path, flavor, gravity, separation)
    with open(urdf_out, "w") as f:
        f.write(robot_description)
    with open(yaml_path, "w") as f:
        f.write(make_controller_yaml())
    print(f"[so101_dual] flavor={flavor}  urdf={urdf_path}  -> {urdf_out}  (separation {separation} m)")

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

    # One spawner process loads every controller (fewer things that can go wrong).
    spawner = Node(package="controller_manager", executable="spawner", output="screen",
                   arguments=CONTROLLERS + ["--controller-manager", "/controller_manager",
                                            "--controller-manager-timeout", "120"])

    def after_spawn(event, ctx):
        return [spawner]

    actions = [gz_sim, clock_bridge, rsp, spawn,
               RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn))]

    def after_spawner(event, ctx):
        if event.returncode != 0:
            return [LogInfo(msg=(
                "\n[so101_dual] !!! The controller spawner FAILED (exit code %s). The arms will not move.\n"
                "    Read the spawner messages above, then retry by hand:\n"
                "      ros2 run controller_manager spawner %s --controller-manager /controller_manager\n"
                "    Check:  ros2 control list_controllers\n" % (event.returncode, " ".join(CONTROLLERS))))]
        if not teleop:
            return [LogInfo(msg="[so101_dual] Controllers active. Start the teleop node when ready.")]
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dual_hand_ik_teleop.py")
        return [LogInfo(msg="[so101_dual] Controllers active -> starting hand teleop"),
                ExecuteProcess(cmd=["python3", script, "--ros-args",
                                    "-p", f"urdf_file:={urdf_path}",
                                    "-p", f"camera_index:={camera_index}"],
                               output="screen")]

    actions.append(RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=after_spawner)))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("urdf_file", default_value="", description="Path to so101_new_calib.urdf"),
        DeclareLaunchArgument("separation", default_value="0.40", description="Distance between the two arm bases [m]"),
        DeclareLaunchArgument("teleop", default_value="true", description="Start dual_hand_ik_teleop.py"),
        DeclareLaunchArgument("gui", default_value="true", description="Show the Gazebo GUI"),
        DeclareLaunchArgument("world", default_value="empty.sdf", description="Gazebo world file"),
        DeclareLaunchArgument("gz_flavor", default_value="auto", description="auto | gz | ign"),
        DeclareLaunchArgument("gravity", default_value="false", description="Keep gravity on the arm links"),
        DeclareLaunchArgument("camera_index", default_value="0", description="Webcam index for the teleop node"),
        OpaqueFunction(function=launch_setup),
    ])
