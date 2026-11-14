#!/usr/bin/env python3
"""
UDP -> ROS 2 bridge for humanoid arm control in Gazebo Classic.

Receives joint_targets on UDP port 8768 and publishes
left and right arm commands.

Phone orientation is handled separately by gyro_to_gazebo.py.
"""

import json
import math
import socket
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


ARM_KEYS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)


# URDF joint limits (rad) for [pan, lift, elbow, wrist_flex, wrist_roll, gripper].
# The gripper limits differ per side (Revolute_28 = right, Revolute_37 = left) and
# are now negative: ~0 is closed, the negative end is open.
_ARM_LIMITS = (
    (-1.91986, 1.91986),
    (-1.74533, 1.74533),
    (-1.69, 1.69),
    (-1.65806, 1.65806),
    (-2.74385, 2.84121),
)
JOINT_LIMITS = {
    "right": _ARM_LIMITS + ((-1.343, 0.137),),
    "left": _ARM_LIMITS + ((-1.7, 0.220),),
}


def clamp(value, low, high):
    return max(low, min(high, value))


def finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None

    return value if math.isfinite(value) else None


class HumanoidArmBridge(Node):

    def __init__(self):
        super().__init__("humanoid_arm_bridge")

        self.declare_parameter("joint_udp_host", "127.0.0.1")
        self.declare_parameter("joint_udp_port", 8768)
        self.declare_parameter("packet_timeout_sec", 1.0)

        self.declare_parameter(
            "right_arm_topic",
            "/right_arm_position_controller/commands",
        )
        self.declare_parameter(
            "left_arm_topic",
            "/left_arm_position_controller/commands",
        )

        # Gripper positions per side, inside the URDF limits above
        # (right: -1.343..0.137, left: -1.7..0.22). Closed ~ 0, open = negative end.
        # The open/closed decision comes from the packet's boolean <side>_closed
        # (robot1.py's gripper_rad assumes the old positive-open range, so it is not used).
        self.declare_parameter("right_gripper_open_rad", -1.2)
        self.declare_parameter("right_gripper_closed_rad", 0.0)
        self.declare_parameter("left_gripper_open_rad", -1.5)
        self.declare_parameter("left_gripper_closed_rad", 0.0)

        # Publish at a fixed rate and limit joint speed so the Gazebo position
        # controllers never receive big steps (which made the joints swing/spin).
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("max_joint_speed_rad_s", 3.0)
        self.declare_parameter("max_gripper_speed_rad_s", 4.0)

        self.right_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter("right_arm_topic").value,
            10,
        )

        self.left_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter("left_arm_topic").value,
            10,
        )

        self.host = str(self.get_parameter("joint_udp_host").value)
        self.port = int(self.get_parameter("joint_udp_port").value)

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

        self.packet_timeout = float(
            self.get_parameter("packet_timeout_sec").value
        )

        self.joint_count = 0
        self.last_log = time.monotonic()

        self.targets = {}   # side -> latest 6 joint targets (rad)
        self.current = {}   # side -> ramped command state (rad)
        self.measured = {}  # side -> latest measured joint state (rad)
        self.joint_name_map = {
            "right": ("Revolute_23", "Revolute_24", "Revolute_25", "Revolute_26", "Revolute_27", "Revolute_28"),
            "left":  ("Revolute_32", "Revolute_33", "Revolute_34", "Revolute_35", "Revolute_36", "Revolute_37"),
        }
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 20
        )
        self.last_step = time.monotonic()

        rate = max(10.0, float(self.get_parameter("publish_rate_hz").value))
        self.max_speed = float(self.get_parameter("max_joint_speed_rad_s").value)
        self.max_grip_speed = float(
            self.get_parameter("max_gripper_speed_rad_s").value
        )

        self.timer = self.create_timer(0.005, self.poll_udp)
        self.publish_timer = self.create_timer(1.0 / rate, self.step_and_publish)

        self.get_logger().info(
            f"Arm bridge listening on {self.host}:{self.port}"
        )
        self.get_logger().info(
            "Publishing left and right arm commands; first target is synchronized "
            "from /joint_states when available."
        )

    def on_joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        for side, names in self.joint_name_map.items():
            values = [finite(positions.get(name)) for name in names]
            if all(value is not None for value in values):
                self.measured[side] = values

    def _fresh(self, packet):
        received = finite(packet.get("received_at"))

        if received is None:
            return False

        age = time.time() - received

        return -0.25 <= age <= self.packet_timeout

    def poll_udp(self):
        latest_packet = None

        while True:
            try:
                raw, _address = self.sock.recvfrom(65535)

            except BlockingIOError:
                break

            except OSError as error:
                self.get_logger().error(str(error))
                break

            try:
                latest_packet = json.loads(raw.decode("utf-8"))

            except (UnicodeDecodeError, json.JSONDecodeError):
                latest_packet = None

        if latest_packet is not None:
            self.handle_joint_packet(latest_packet)

    def handle_joint_packet(self, packet):

        if not isinstance(packet, dict):
            return

        if packet.get("type") != "joint_targets":
            return

        if packet.get("units") != "deg" or not self._fresh(packet):
            self.get_logger().warning(
                "Dropped stale joint packet or unexpected units"
            )
            return

        arms = packet.get("arms")
        gripper = packet.get("gripper")

        if not isinstance(arms, dict) or not isinstance(gripper, dict):
            self.get_logger().warning(
                "Dropped joint packet: arms/gripper missing"
            )
            return

        output = {}

        for side in ("left", "right"):

            values = arms.get(side)

            if not isinstance(values, dict):
                return

            radians = []

            for name in ARM_KEYS:

                degrees = finite(values.get(name))

                if degrees is None or abs(degrees) > 180.0:
                    self.get_logger().warning(
                        f"Dropped invalid {side}.{name}"
                    )
                    return

                radians.append(math.radians(degrees))

            percent = gripper.get(f"{side}_percent")
            if percent is None:
                # Backward compatibility with old binary packets.
                closed = gripper.get(f"{side}_closed")
                if not isinstance(closed, bool):
                    self.get_logger().warning(
                        f"Dropped invalid {side} gripper command"
                    )
                    return
                percent = 100.0 if closed else 0.0

            percent = finite(percent)
            if percent is None or not 0.0 <= percent <= 100.0:
                self.get_logger().warning(
                    f"Dropped invalid {side} gripper percentage"
                )
                return

            open_rad = float(self.get_parameter(
                f"{side}_gripper_open_rad"
            ).value)
            closed_rad = float(self.get_parameter(
                f"{side}_gripper_closed_rad"
            ).value)
            grip_value = open_rad + (percent / 100.0) * (closed_rad - open_rad)

            if not math.isfinite(grip_value):
                return

            # Sixth controller value is the gripper command.
            radians.append(grip_value)

            output[side] = radians

        # Clamp to URDF limits and store as the new target; the publish timer
        # moves toward it at a bounded speed.
        for side, values in output.items():
            self.targets[side] = [
                clamp(v, lo, hi)
                for v, (lo, hi) in zip(values, JOINT_LIMITS[side])
            ]
            if side not in self.current:
                # Start the rate limiter from Gazebo's measured pose when joint
                # states are available, instead of jumping directly to the first
                # hand-derived target. Fall back to the target if /joint_states
                # has not arrived yet.
                measured = self.measured.get(side)
                self.current[side] = list(measured if measured is not None else self.targets[side])

        self.joint_count += 1
        self.periodic_log()

    def step_and_publish(self):
        now = time.monotonic()
        dt = max(0.0, min(0.1, now - self.last_step))
        self.last_step = now

        for side, publisher in (
            ("left", self.left_pub),
            ("right", self.right_pub),
        ):
            target = self.targets.get(side)
            current = self.current.get(side)

            if target is None or current is None:
                continue

            for i, goal in enumerate(target):
                limit = self.max_grip_speed if i == 5 else self.max_speed
                step = limit * dt
                current[i] += clamp(goal - current[i], -step, step)

            self.publish(publisher, current)

    @staticmethod
    def publish(publisher, values):
        message = Float64MultiArray()
        message.data = [float(value) for value in values]
        publisher.publish(message)

    def periodic_log(self):

        now = time.monotonic()

        if now - self.last_log >= 1.0:

            self.get_logger().info(
                f"Forwarded arm joint packets={self.joint_count} "
                f"(UDP {self.port})"
            )

            self.joint_count = 0
            self.last_log = now

    def destroy_node(self):

        try:
            self.sock.close()
        except OSError:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = HumanoidArmBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()