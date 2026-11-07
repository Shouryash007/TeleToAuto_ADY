#!/usr/bin/env python3
"""Latest-frame Gazebo ROS Image -> length-prefixed JPEG TCP stream."""
import socket
import struct
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

HOST, PORT = "127.0.0.1", 8767
IMAGE_TOPIC = "/camera/sim_camera/image_raw"


class CameraBridge(Node):
    def __init__(self):
        super().__init__("gazebo_camera_webrtc_bridge")
        self.bridge = CvBridge()
        self.sock = None
        self.last_warning = 0.0

        # A single latest-frame slot prevents ROS callback backlog.
        self.latest_jpeg = None
        self.frame_lock = threading.Lock()
        self.frame_event = threading.Event()
        self.stop_event = threading.Event()

        self.create_subscription(
            Image, IMAGE_TOPIC, self.on_image, qos_profile_sensor_data
        )
        self.sender = threading.Thread(target=self.send_latest_frames, daemon=True)
        self.sender.start()
        self.get_logger().info(
            f"Subscribing to {IMAGE_TOPIC}; forwarding latest frames to {HOST}:{PORT}"
        )

    def connect(self):
        if self.sock:
            return True
        try:
            sock = socket.create_connection((HOST, PORT), timeout=1.0)
            # Keep the kernel send queue small; stale video is less useful than fresh video.
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 128 * 1024)
            sock.settimeout(0.5)
            self.sock = sock
            self.get_logger().info("Connected to robot1 video receiver")
            return True
        except OSError as exc:
            if time.monotonic() - self.last_warning > 5:
                self.get_logger().warning(f"robot1 receiver not ready: {exc}")
                self.last_warning = time.monotonic()
            return False

    def on_image(self, msg):
        # Keep this callback short. If encoding or sending falls behind,
        # the next callback overwrites this frame instead of queuing old frames.
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60]
            )
            if not ok:
                return
            with self.frame_lock:
                self.latest_jpeg = encoded.tobytes()
            self.frame_event.set()
        except Exception as exc:
            self.get_logger().error(f"Image encoding error: {exc}")

    def send_latest_frames(self):
        while not self.stop_event.is_set():
            self.frame_event.wait(timeout=0.5)
            self.frame_event.clear()
            if self.stop_event.is_set():
                break
            with self.frame_lock:
                payload = self.latest_jpeg
            if not payload or not self.connect():
                continue
            try:
                self.sock.sendall(struct.pack("!I", len(payload)) + payload)
            except OSError as exc:
                self.get_logger().warning(f"Video TCP stalled/reset; reconnecting: {exc}")
                try:
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None

    def destroy_node(self):
        self.stop_event.set()
        self.frame_event.set()
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sender.join(timeout=1.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
