#!/usr/bin/env python3
"""ROS 2 UDP-to-JointTrajectory bridge for the Gazebo URDF simulation.

Run this in a ROS 2 environment where rclpy, trajectory_msgs, and the robot's
ros2_control controller are available. It accepts packets from robot1.py on
127.0.0.1:8766 and publishes a short trajectory to the configured controller.

IMPORTANT: adapt JOINT_NAMES and controller topic to the names in your URDF and
ros2_control YAML. Defaults are illustrative SO-101-like names, not guaranteed
URDF names. Physical follower is not used by this bridge.
"""
import json
import math
import os
import queue
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

UDP_HOST = os.getenv("SIMULATION_IPC_HOST", "127.0.0.1")
UDP_PORT = int(os.getenv("SIMULATION_IPC_PORT", "8766"))
CONTROLLER_TOPIC = os.getenv(
    "GAZEBO_TRAJECTORY_TOPIC", "/joint_trajectory_controller/joint_trajectory"
)

# Configure these names to exactly match your URDF/ros2_control controller.
JOINT_NAMES = [name.strip() for name in os.getenv(
    "GAZEBO_JOINT_NAMES",
    "left_shoulder_pan_joint,left_shoulder_lift_joint,left_elbow_flex_joint,left_wrist_flex_joint,left_wrist_roll_joint,left_gripper_joint,right_shoulder_pan_joint,right_shoulder_lift_joint,right_elbow_flex_joint,right_wrist_flex_joint,right_wrist_roll_joint,right_gripper_joint,neck_yaw_joint,neck_pitch_joint"
).split(",") if name.strip()]

# A boolean closed flag maps to these configured simulated gripper positions.
LEFT_GRIPPER_NAME = os.getenv("LEFT_GRIPPER_JOINT", "left_gripper_joint")
RIGHT_GRIPPER_NAME = os.getenv("RIGHT_GRIPPER_JOINT", "right_gripper_joint")
HEAD_YAW_NAME = os.getenv("HEAD_YAW_JOINT", "neck_yaw_joint")
HEAD_PITCH_NAME = os.getenv("HEAD_PITCH_JOINT", "neck_pitch_joint")
GRIPPER_OPEN = float(os.getenv("GRIPPER_OPEN_POSITION", "0.0"))
GRIPPER_CLOSED = float(os.getenv("GRIPPER_CLOSED_POSITION", "0.8"))
TRAJECTORY_DURATION = float(os.getenv("TRAJECTORY_DURATION", "0.12"))

ARM_JOINT_ORDER = (
    ("left", "shoulder_pan"), ("left", "shoulder_lift"),
    ("left", "elbow_flex"), ("left", "wrist_flex"), ("left", "wrist_roll"),
    ("right", "shoulder_pan"), ("right", "shoulder_lift"),
    ("right", "elbow_flex"), ("right", "wrist_flex"), ("right", "wrist_roll"),
)


def safe_float(value):
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except (TypeError, ValueError):
        return None


class GazeboJointBridge(Node):
    def __init__(self):
        super().__init__("teleop_gazebo_joint_bridge")
        self.publisher = self.create_publisher(JointTrajectory, CONTROLLER_TOPIC, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_HOST, UDP_PORT))
        self.sock.settimeout(0.5)
        self.stop_event = threading.Event()
        self.packet_queue = queue.Queue(maxsize=100)
        self.latest_head = {"yaw": 0.0, "pitch": 0.0}
        self.drain_timer = self.create_timer(0.01, self._drain_queue)
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        self.get_logger().info(f"UDP listening at {UDP_HOST}:{UDP_PORT}")
        self.get_logger().info(f"Publishing trajectories to {CONTROLLER_TOPIC}")
        self.get_logger().info(f"Configured joint names: {JOINT_NAMES}")

    def _receive_loop(self):
        while not self.stop_event.is_set():
            try:
                raw, _addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                payload = json.loads(raw.decode("utf-8"))
                try:
                    self.packet_queue.put_nowait(payload)
                except queue.Full:
                    # Prefer the newest controls rather than building up latency.
                    try:
                        self.packet_queue.get_nowait()
                    except queue.Empty:
                        pass
                    self.packet_queue.put_nowait(payload)
            except Exception as exc:
                self.get_logger().warning(f"Dropped invalid UDP packet: {exc}")

    def _drain_queue(self):
        # Called on the ROS executor thread, so publishers and state stay safe.
        for _ in range(20):
            try:
                payload = self.packet_queue.get_nowait()
            except queue.Empty:
                break
            if payload.get("type") == "joint_targets":
                self._publish_joint_targets(payload)
            elif payload.get("type") == "head_orientation":
                orientation = payload.get("orientation", {})
                yaw = safe_float(orientation.get("yaw"))
                pitch = safe_float(orientation.get("pitch"))
                if yaw is not None and pitch is not None and abs(yaw) <= 2 * math.pi and abs(pitch) <= 2 * math.pi:
                    self.latest_head["yaw"] = yaw
                    self.latest_head["pitch"] = pitch
                    self.get_logger().debug(f"Head orientation updated yaw={yaw:.3f}, pitch={pitch:.3f}")
                else:
                    self.get_logger().warning("Ignoring invalid head orientation")

    def _publish_joint_targets(self, payload):
        received_at = safe_float(payload.get("received_at"))
        if received_at is not None and time.time() - received_at > 0.75:
            self.get_logger().warning("Ignoring stale joint packet")
            return
        arms = payload.get("arms", {})
        positions_by_name = {}
        for side, joint in ARM_JOINT_ORDER:
            key = f"{side}_{joint}_joint"
            value = safe_float(arms.get(side, {}).get(joint))
            if value is None or abs(value) > 180.0:
                self.get_logger().warning(f"Ignoring invalid target for {key}")
                return
            positions_by_name[key] = math.radians(value)

        gripper = payload.get("gripper", {})
        positions_by_name[LEFT_GRIPPER_NAME] = GRIPPER_CLOSED if gripper.get("left_closed") else GRIPPER_OPEN
        positions_by_name[RIGHT_GRIPPER_NAME] = GRIPPER_CLOSED if gripper.get("right_closed") else GRIPPER_OPEN
        positions_by_name[HEAD_YAW_NAME] = self.latest_head["yaw"]
        positions_by_name[HEAD_PITCH_NAME] = self.latest_head["pitch"]

        missing = [name for name in positions_by_name if name not in JOINT_NAMES]
        if missing:
            self.get_logger().error(
                "Configured controller joint list is missing names: " + ", ".join(missing)
            )
            return

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [positions_by_name.get(name, 0.0) for name in JOINT_NAMES]
        point.time_from_start.sec = max(0, int(TRAJECTORY_DURATION))
        point.time_from_start.nanosec = int((TRAJECTORY_DURATION % 1.0) * 1_000_000_000)
        msg.points = [point]
        self.publisher.publish(msg)
        self.get_logger().info(f"Published joint target sequence={payload.get('sequence')}")

    def destroy_node(self):
        self.stop_event.set()
        try:
            self.sock.close()
        except OSError:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = GazeboJointBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
