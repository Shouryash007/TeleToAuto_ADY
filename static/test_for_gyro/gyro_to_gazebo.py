#!/usr/bin/env python3
"""Receive gyro JSON over UDP (default port 8766) and command Gazebo camera joints.

Expected packet formats:
  {"type":"head_orientation","orientation":{"yaw":0.2,"pitch":-0.1},"units":"rad"}
  {"type":"head_gyro","gyro":{"gx":12.0,"gy":-4.0},"units":"deg/s"}

The robot script must send its UDP packets to the computer running this node.
"""
import json
import math
import socket
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class GyroToGazebo(Node):
    def __init__(self):
        super().__init__("gyro_to_gazebo")
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("udp_port", 8766)
        self.declare_parameter("gyro_scale", 1.0)
        self.host = str(self.get_parameter("bind_host").value)
        self.port = int(self.get_parameter("udp_port").value)
        self.scale = float(self.get_parameter("gyro_scale").value)
        self.yaw_pub = self.create_publisher(
            Float64MultiArray, "/camera_yaw_position_controller/commands", 10)
        self.pitch_pub = self.create_publisher(
            Float64MultiArray, "/camera_pitch_position_controller/commands", 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.setblocking(False)
        self.yaw = 0.0
        self.pitch = 0.0
        self.last_gyro_t = None
        self.timer = self.create_timer(0.01, self.poll)
        self.get_logger().info(f"Listening for gyro UDP on {self.host}:{self.port}")

    def publish(self):
        yaw_msg = Float64MultiArray()
        yaw_msg.data = [self.yaw]
        pitch_msg = Float64MultiArray()
        pitch_msg.data = [self.pitch]
        self.yaw_pub.publish(yaw_msg)
        self.pitch_pub.publish(pitch_msg)

    def poll(self):
        while True:
            try:
                raw, addr = self.sock.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().error(f"UDP socket error: {exc}")
                return

            try:
                packet = json.loads(raw.decode("utf-8"))
                kind = packet.get("type")
                if kind == "head_orientation":
                    orient = packet.get("orientation") or {}
                    yaw = float(orient.get("yaw", 0.0))
                    pitch = float(orient.get("pitch", 0.0))
                    if not math.isfinite(yaw) or not math.isfinite(pitch):
                        continue
                    self.yaw = clamp(yaw, -1.4, 1.4)
                    self.pitch = clamp(pitch, -1.0, 1.0)
                    self.last_gyro_t = None
                elif kind == "head_gyro":
                    gyro = packet.get("gyro") or {}
                    gx = float(gyro.get("gx", 0.0))
                    gy = float(gyro.get("gy", 0.0))
                    if not math.isfinite(gx) or not math.isfinite(gy):
                        continue
                    now = time.monotonic()
                    dt = 0.0 if self.last_gyro_t is None else min(now - self.last_gyro_t, 0.1)
                    self.last_gyro_t = now
                    self.yaw = clamp(self.yaw + math.radians(gx) * dt * self.scale, -1.4, 1.4)
                    self.pitch = clamp(self.pitch + math.radians(gy) * dt * self.scale, -1.0, 1.0)
                else:
                    continue

                self.publish()
                self.get_logger().info(
                    f"RX {addr[0]}:{addr[1]} {kind}: yaw={self.yaw:+.3f} rad, "
                    f"pitch={self.pitch:+.3f} rad")
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
                self.get_logger().warning(f"Ignoring invalid UDP packet: {exc}")

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = GyroToGazebo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
