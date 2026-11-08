#!/usr/bin/env python3

import json
import math
import socket
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


class GyroToGazebo(Node):

    def __init__(self):
        super().__init__("gyro_to_gazebo")

        self.declare_parameter("bind_host", "0.0.0.0")

        # Matches robot1.py orientation UDP destination.
        self.declare_parameter("udp_port", 8769)

        self.declare_parameter("gyro_scale", 1.0)

        self.host = str(self.get_parameter("bind_host").value)
        self.port = int(self.get_parameter("udp_port").value)
        self.scale = float(self.get_parameter("gyro_scale").value)

        # Revolute_18: camera horizontal rotation.
        self.yaw_pub = self.create_publisher(
            Float64MultiArray,
            "/camera_yaw_position_controller/commands",
            10,
        )

        # Revolute_19: camera vertical rotation.
        # Phone ROLL controls this joint.
        self.pitch_pub = self.create_publisher(
            Float64MultiArray,
            "/camera_pitch_position_controller/commands",
            10,
        )

        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )

        self.sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1,
        )

        self.sock.bind((self.host, self.port))
        self.sock.setblocking(False)

        self.yaw = 0.0
        self.roll = 0.0
        self.last_gyro_time = None
        self.last_log = 0.0

        self.timer = self.create_timer(
            0.01,
            self.receive_packets,
        )

        self.get_logger().info(
            f"Listening for phone yaw/roll orientation "
            f"on {self.host}:{self.port}"
        )

    def publish_commands(self):

        yaw_msg = Float64MultiArray()
        yaw_msg.data = [self.yaw]

        roll_msg = Float64MultiArray()
        roll_msg.data = [self.roll]

        self.yaw_pub.publish(yaw_msg)
        self.pitch_pub.publish(roll_msg)

    def receive_packets(self):

        while True:

            try:
                raw, address = self.sock.recvfrom(65535)

            except BlockingIOError:
                return

            except OSError as error:
                self.get_logger().error(str(error))
                return

            try:

                packet = json.loads(raw.decode("utf-8"))

                if not isinstance(packet, dict):
                    continue

                packet_type = packet.get("type")

                # Absolute phone orientation in radians.
                if packet_type == "head_orientation":

                    orientation = packet.get("orientation") or {}

                    yaw = float(orientation.get("yaw", 0.0))
                    roll = float(orientation.get("roll", 0.0))

                    if (
                        not math.isfinite(yaw)
                        or not math.isfinite(roll)
                    ):
                        continue

                    self.yaw = clamp(yaw, -1.4, 1.4)
                    self.roll = clamp(roll, -1.0, 1.0)

                    self.last_gyro_time = None

                # Integrated gyroscope data in degrees/second.
                elif packet_type == "head_gyro":

                    gyro = packet.get("gyro") or {}

                    gx = float(gyro.get("gx", 0.0))
                    gy = float(gyro.get("gy", 0.0))

                    if (
                        not math.isfinite(gx)
                        or not math.isfinite(gy)
                    ):
                        continue

                    now = time.monotonic()

                    dt = (
                        0.0
                        if self.last_gyro_time is None
                        else min(
                            now - self.last_gyro_time,
                            0.1,
                        )
                    )

                    self.last_gyro_time = now

                    self.yaw = clamp(
                        self.yaw
                        + math.radians(gx) * dt * self.scale,
                        -1.4,
                        1.4,
                    )

                    self.roll = clamp(
                        self.roll
                        + math.radians(gy) * dt * self.scale,
                        -1.0,
                        1.0,
                    )

                else:
                    continue

                self.publish_commands()

                # Log at a low rate to avoid flooding the terminal.
                now = time.monotonic()

                if now - self.last_log > 1.0:

                    self.get_logger().info(
                        f"Phone {address[0]} | "
                        f"Camera yaw: {self.yaw:+.3f} rad | "
                        f"Camera vertical tilt (phone roll): "
                        f"{self.roll:+.3f} rad"
                    )

                    self.last_log = now

            except (
                UnicodeDecodeError,
                json.JSONDecodeError,
                TypeError,
                ValueError,
            ) as error:

                self.get_logger().warning(
                    f"Invalid packet: {error}"
                )

    def destroy_node(self):

        try:
            self.sock.close()
        except OSError:
            pass

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