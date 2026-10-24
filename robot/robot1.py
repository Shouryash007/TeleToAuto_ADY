#!/usr/bin/env python3
"""Robot-side WebRTC endpoint for the SO-101 teleoperation simulation.

The physical follower is deliberately disabled. Browser-computed joint targets
and ESP32 head orientation are received over WebRTC DataChannels, validated,
and sent to a local ROS 2/Gazebo UDP bridge. Video is sent from CAMERA_DEVICE
when a camera is available.
"""
import asyncio
import json
import math
import os
import socket
import time

import cv2
import websockets
from av import VideoFrame
from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    RTCConfiguration,
    RTCIceServer,
    VideoStreamTrack,
)

ROBOT_ID = os.getenv("ROBOT_ID", "robot_001")
ROBOT_TOKEN = os.getenv("ROBOT_TOKEN", "robot001secretabc")
SIGNALING_SERVER = os.getenv("SIGNALING_SERVER", "ws://127.0.0.1:8000/ws")
# Physical camera transmission is disabled by default.
# Gazebo camera integration will supply the simulated video separately.
CAMERA_DEVICE = os.getenv("CAMERA_DEVICE", "")
CAMERA_WIDTH = int(os.getenv("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.getenv("CAMERA_HEIGHT", "480"))
CAMERA_FPS = int(os.getenv("CAMERA_FPS", "30"))
ENABLE_PHYSICAL_CAMERA = (
    os.getenv("ENABLE_PHYSICAL_CAMERA", "false").lower() == "true"
)

# Safety invariant: never enable the physical SO-101 follower from this file.
FOLLOWER_ENABLED = False
SIMULATION_ENABLED = os.getenv("SIMULATION_ENABLED", "true").lower() in {"1", "true", "yes"}
SIMULATION_IPC_ADDRESS = (
    os.getenv("SIMULATION_IPC_HOST", "127.0.0.1"),
    int(os.getenv("SIMULATION_IPC_PORT", "8766")),
)
COMMAND_TIMEOUT_SECONDS = float(os.getenv("COMMAND_TIMEOUT_SECONDS", "0.5"))
MAX_ABS_JOINT_DEG = float(os.getenv("MAX_ABS_JOINT_DEG", "180"))

simulation_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

ICE_SERVERS = []
if os.getenv("STUN_URL"):
    ICE_SERVERS.append(RTCIceServer(urls=os.getenv("STUN_URL")))
if os.getenv("TURN_URL"):
    ICE_SERVERS.append(RTCIceServer(
        urls=os.getenv("TURN_URL"),
        username=os.getenv("TURN_USERNAME", ""),
        credential=os.getenv("TURN_PASSWORD", ""),
    ))
RTC_CONFIG = RTCConfiguration(iceServers=ICE_SERVERS)


def finite_number(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


def send_to_gazebo(payload):
    """Send a validated command to the ROS 2 bridge on localhost UDP."""
    if not SIMULATION_ENABLED:
        print("[SAFETY] Simulation output disabled; command not applied")
        return False
    try:
        payload["received_at"] = time.time()
        simulation_socket.sendto(
            json.dumps(payload, separators=(",", ":")).encode("utf-8"),
            SIMULATION_IPC_ADDRESS,
        )
        return True
    except Exception as exc:
        print("[SIM UDP] Send failed:", repr(exc))
        return False


class CameraVideoTrack(VideoStreamTrack):
    def __init__(self, device):
        super().__init__()
        # Integer device indexes and /dev/video paths are both supported.
        source = int(device) if str(device).isdigit() else device
        self.cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        if not self.cap.isOpened():
            self.cap.release()
            raise RuntimeError(f"Cannot open camera: {device}")
        print(f"[CAMERA] Opened {device} at requested {CAMERA_WIDTH}x{CAMERA_HEIGHT}@{CAMERA_FPS}")

    async def recv(self):
        ret, frame = await asyncio.to_thread(self.cap.read)
        if not ret:
            raise RuntimeError("Camera frame read failed")
        video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        pts, time_base = await self.next_timestamp()
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def stop(self):
        if getattr(self, "cap", None) is not None and self.cap.isOpened():
            self.cap.release()
            print("[CAMERA] Released")
        super().stop()


def open_camera_optional():
    """Open a physical camera only when explicitly enabled."""
    if not ENABLE_PHYSICAL_CAMERA:
        print(
            "[CAMERA] Physical camera disabled. "
            "Waiting for Gazebo camera integration."
        )
        return None

    if not CAMERA_DEVICE:
        print("[CAMERA] No physical camera device configured.")
        return None

    try:
        return CameraVideoTrack(CAMERA_DEVICE)
    except Exception as exc:
        print("[CAMERA] No video track; data channels can still work:", repr(exc))
        return None


def validate_joint_targets(data):
    """Validate browser packet and normalize to finite degree values."""
    if data.get("type") != "joint_targets":
        return None
    arms = data.get("arms")
    if not isinstance(arms, dict):
        return None
    # Browser timestamps are epoch milliseconds. Reject stale or far-future packets.
    sent_ms = finite_number(data.get("timestamp_ms"))
    if sent_ms is None or abs((time.time() * 1000.0) - sent_ms) > 1000.0:
        print("[SAFETY] Rejecting stale/invalid joint packet timestamp")
        return None
    normalized = {"left": {}, "right": {}}
    joint_names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
    for side in ("left", "right"):
        source = arms.get(side)
        if not isinstance(source, dict):
            return None
        for name in joint_names:
            number = finite_number(source.get(name))
            if number is None or abs(number) > MAX_ABS_JOINT_DEG:
                print(f"[SAFETY] Rejecting invalid {side}.{name}={source.get(name)!r}")
                return None
            normalized[side][name] = number
    gripper = data.get("gripper", {})
    # Frontend currently sends boolean left_closed/right_closed.
    normalized_gripper = {
        "left_closed": bool(gripper.get("left_closed", False)),
        "right_closed": bool(gripper.get("right_closed", False)),
    }
    return {
        "type": "joint_targets",
        "sequence": data.get("sequence"),
        "timestamp_ms": data.get("timestamp_ms"),
        "units": "deg",
        "arms": normalized,
        "gripper": normalized_gripper,
    }


def validate_head_orientation(data):
    if data.get("type") != "head_orientation":
        return None
    orientation = data.get("orientation")
    if not isinstance(orientation, dict):
        return None
    sent_ms = finite_number(data.get("timestamp_ms"))
    if sent_ms is None or abs((time.time() * 1000.0) - sent_ms) > 1000.0:
        print("[SAFETY] Rejecting stale/invalid IMU packet timestamp")
        return None
    values = {}
    for name in ("yaw", "pitch", "roll"):
        number = finite_number(orientation.get(name))
        # Reject clearly corrupt/unbounded radians.
        if number is None or abs(number) > (2.0 * math.pi):
            print(f"[SAFETY] Rejecting invalid IMU {name}={orientation.get(name)!r}")
            return None
        values[name] = number
    return {
        "type": "head_orientation",
        "sequence": data.get("sequence"),
        "timestamp_ms": data.get("timestamp_ms"),
        "units": "rad",
        "orientation": values,
    }


async def create_webrtc_session(websocket):
    print("[WEBRTC] Creating peer connection")
    pc = RTCPeerConnection(RTC_CONFIG)
    camera = open_camera_optional()
    if camera is not None:
        pc.addTrack(camera)

    # These exact names must match the browser's DataChannel lookups.
    joint_channel = pc.createDataChannel("joint-control")
    imu_channel = pc.createDataChannel("imu-control")
    latest_joint_packet_time = {"value": time.monotonic()}

    @joint_channel.on("open")
    def joint_open():
        print("[DATA] joint-control open")

    @joint_channel.on("close")
    def joint_close():
        print("[DATA] joint-control closed")

    @joint_channel.on("message")
    def joint_message(message):
        try:
            if isinstance(message, bytes):
                message = message.decode("utf-8")
            data = json.loads(message)
            if data.get("type") == "test":
                print("[DATA] Joint channel test:", data)
                return
            packet = validate_joint_targets(data)
            if packet is None:
                print("[DATA] Dropped invalid joint packet")
                return
            latest_joint_packet_time["value"] = time.monotonic()
            print("[JOINT TARGETS] seq=", packet.get("sequence"), packet["arms"], packet["gripper"])
            send_to_gazebo(packet)
        except Exception as exc:
            print("[DATA] Invalid joint-control message:", repr(exc))

    @imu_channel.on("open")
    def imu_open():
        print("[DATA] imu-control open")

    @imu_channel.on("close")
    def imu_close():
        print("[DATA] imu-control closed")

    @imu_channel.on("message")
    def imu_message(message):
        try:
            if isinstance(message, bytes):
                message = message.decode("utf-8")
            data = json.loads(message)
            if data.get("type") == "test":
                print("[DATA] IMU channel test:", data)
                return
            packet = validate_head_orientation(data)
            if packet is None:
                print("[DATA] Dropped invalid IMU packet")
                return
            print("[IMU]", packet["orientation"])
            send_to_gazebo(packet)
        except Exception as exc:
            print("[DATA] Invalid IMU message:", repr(exc))

    @pc.on("connectionstatechange")
    async def connection_state_change():
        print("[WEBRTC] Connection state:", pc.connectionState)
        if pc.connectionState in ("failed", "closed"):
            # No commands should be generated by this peer after closure.
            return

    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)
    deadline = time.monotonic() + 10.0
    while pc.iceGatheringState != "complete" and time.monotonic() < deadline:
        await asyncio.sleep(0.1)
    await websocket.send(json.dumps({
        "type": pc.localDescription.type,
        "sdp": pc.localDescription.sdp,
    }))
    print("[WEBRTC] Offer sent; waiting for browser answer")
    return pc, camera, joint_channel, imu_channel


async def run_robot():
    while True:
        pc = camera = joint_channel = imu_channel = None
        try:
            print("\n" + "=" * 55)
            print(f"[ROBOT] ID: {ROBOT_ID}")
            print(f"[ROBOT] Signaling: {SIGNALING_SERVER}")
            print(f"[ROBOT] Physical follower: DISABLED (enforced)")
            print(f"[ROBOT] Gazebo UDP bridge: {SIMULATION_ENABLED} -> {SIMULATION_IPC_ADDRESS}")
            print("=" * 55)
            async with websockets.connect(
                SIGNALING_SERVER, ping_interval=20, ping_timeout=20, max_size=2**20
            ) as websocket:
                await websocket.send(json.dumps({
                    "role": "robot", "robot_id": ROBOT_ID, "token": ROBOT_TOKEN
                }))
                async for raw in websocket:
                    try:
                        data = json.loads(raw)
                    except json.JSONDecodeError:
                        print("[SERVER] Invalid JSON")
                        continue
                    msg_type = data.get("type")
                    if msg_type == "authenticated":
                        print("[ROBOT] Authenticated with signaling server")
                    elif msg_type == "browser_ready":
                        if pc is not None:
                            await pc.close()
                        if camera is not None:
                            camera.stop()
                        pc, camera, joint_channel, imu_channel = await create_webrtc_session(websocket)
                    elif msg_type == "answer":
                        if pc is None:
                            print("[WEBRTC] Answer received without active peer")
                            continue
                        await pc.setRemoteDescription(RTCSessionDescription(
                            sdp=data["sdp"], type=data.get("type", "answer")
                        ))
                        print("[WEBRTC] Browser answer accepted")
                    elif msg_type == "browser_disconnected":
                        print("[ROBOT] Operator disconnected")
                        if pc is not None:
                            await pc.close()
                        if camera is not None:
                            camera.stop()
                        pc = camera = joint_channel = imu_channel = None
                    else:
                        print("[SERVER] Unhandled signaling message:", msg_type)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            print("[ROBOT] Connection error:", repr(exc))
        finally:
            if pc is not None:
                try:
                    await pc.close()
                except Exception:
                    pass
            if camera is not None:
                try:
                    camera.stop()
                except Exception:
                    pass
        print("[ROBOT] Reconnecting in 3 seconds")
        await asyncio.sleep(3)


if __name__ == "__main__":
    try:
        asyncio.run(run_robot())
    except KeyboardInterrupt:
        print("\n[ROBOT] Shutdown requested")
    finally:
        simulation_socket.close()
