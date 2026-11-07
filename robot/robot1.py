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
import struct
import threading
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
SIGNALING_SERVER = os.getenv("SIGNALING_SERVER", "ws://127.0.0.1:5000/ws")
# Physical camera transmission is disabled by default.
# Gazebo camera integration will supply the simulated video separately.
CAMERA_DEVICE = os.getenv("CAMERA_DEVICE", "")
CAMERA_WIDTH = int(os.getenv("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.getenv("CAMERA_HEIGHT", "480"))
CAMERA_FPS = int(os.getenv("CAMERA_FPS", "30"))
GAZEBO_FRAME_PORT = int(os.getenv("GAZEBO_FRAME_PORT", "8767"))
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

ICE_SERVERS = [
    RTCIceServer(urls=os.getenv("STUN_URL", "stun:stun.l.google.com:19302"))
]
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



_latest_frame = {"bgr": None, "updated": 0.0}
_frame_lock = threading.Lock()
_stats = {"n": 0, "t": time.monotonic()}

def _recv_exact(conn, size):
    data = bytearray()
    while len(data) < size:
        chunk = conn.recv(size - len(data))
        if not chunk:
            return None
        data.extend(chunk)
    return bytes(data)

def _frame_receiver():
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", GAZEBO_FRAME_PORT))
    listener.listen(1)
    print(f"[GAZEBO VIDEO] Listening on 127.0.0.1:{GAZEBO_FRAME_PORT}")
    while True:
        conn, addr = listener.accept()
        print("[GAZEBO VIDEO] ROS bridge connected:", addr)
        try:
            with conn:
                while True:
                    header = _recv_exact(conn, 4)
                    if not header: break
                    length = struct.unpack("!I", header)[0]
                    if length < 1 or length > 4_000_000: break
                    jpeg = _recv_exact(conn, length)
                    if not jpeg: break
                    import numpy as np
                    t0 = time.monotonic()
                    decoded = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    decode_ms = (time.monotonic() - t0) * 1000.0
                    now = time.monotonic()
                    if decoded is not None:
                        with _frame_lock:
                            _latest_frame["bgr"] = decoded
                            _latest_frame["updated"] = now
                        _stats["n"] += 1
                        elapsed = now - _stats["t"]
                        if elapsed >= 2.0:
                            print(
                                f"[GAZEBO VIDEO] in: {_stats['n'] / elapsed:.1f} fps, "
                                f"decode {decode_ms:.1f} ms, {len(jpeg) / 1024:.1f} KB"
                            )
                            _stats["n"] = 0
                            _stats["t"] = now
        except Exception as exc:
            print("[GAZEBO VIDEO] Receiver error:", repr(exc))
        print("[GAZEBO VIDEO] Bridge disconnected")

threading.Thread(target=_frame_receiver, daemon=True).start()

class GazeboCameraVideoTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.last_notice = 0.0
        self.last_age_log = 0.0

    async def recv(self):
        import numpy as np
        pts, time_base = await self.next_timestamp()
        with _frame_lock:
            frame = _latest_frame["bgr"]
            updated = _latest_frame["updated"]
        now = time.monotonic()
        age_ms = (now - updated) * 1000.0 if updated else float("inf")
        if updated and now - self.last_age_log >= 2.0:
            print(f"[GAZEBO VIDEO] outgoing frame age: {age_ms:.1f} ms")
            self.last_age_log = now
        if frame is None or age_ms >= 2000.0:
            frame = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
            if now - self.last_notice > 5:
                print("[GAZEBO VIDEO] Waiting for first Gazebo image frame")
                self.last_notice = now
        result = VideoFrame.from_ndarray(frame, format="bgr24")
        result.pts, result.time_base = pts, time_base
        return result


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
    """Validate two-axis gyroscope angular velocity (gx, gy), in deg/s."""
    if data.get("type") != "head_gyro":
        return None
    gyro = data.get("gyro")
    if not isinstance(gyro, dict):
        return None
    sent_ms = finite_number(data.get("timestamp_ms"))
    if sent_ms is None or abs((time.time() * 1000.0) - sent_ms) > 1000.0:
        print("[SAFETY] Rejecting stale/invalid gyroscope timestamp")
        return None
    values = {}
    for name in ("gx", "gy"):
        number = finite_number(gyro.get(name))
        # Broad sanity limit for angular velocity; tune to the sensor/firmware.
        if number is None or abs(number) > 2000.0:
            print(f"[SAFETY] Rejecting invalid gyro {name}={gyro.get(name)!r}")
            return None
        values[name] = number
    return {
        "type": "head_gyro",
        "sequence": data.get("sequence"),
        "timestamp_ms": data.get("timestamp_ms"),
        "units": "deg/s",
        "gyro": values,
    }


def validate_phone_orientation(data):
    """Validate calibrated relative yaw/pitch angles in radians from the phone page."""
    if data.get("type") != "head_orientation":
        return None
    orientation = data.get("orientation")
    if not isinstance(orientation, dict):
        return None
    sent_ms = finite_number(data.get("timestamp_ms"))
    if sent_ms is None or abs((time.time() * 1000.0) - sent_ms) > 1000.0:
        print("[SAFETY] Rejecting stale/invalid phone orientation timestamp")
        return None
    values = {}
    for name in ("yaw", "pitch", "roll"):
        number = finite_number(orientation.get(name, 0.0))
        if number is None or abs(number) > math.pi:
            print(f"[SAFETY] Rejecting invalid orientation {name}={orientation.get(name)!r}")
            return None
        values[name] = number
    return {
        "type": "head_orientation",
        "sequence": data.get("sequence"),
        "timestamp_ms": data.get("timestamp_ms"),
        "units": "rad",
        "orientation": values,
        "source": "phone_device_orientation",
    }



async def create_webrtc_session(websocket, viewer):
    print(f"[WEBRTC] Creating {viewer} peer")
    pc = RTCPeerConnection(RTC_CONFIG)
    camera = GazeboCameraVideoTrack()
    pc.addTrack(camera)
    joint_channel = imu_channel = None
    if viewer == "browser":
        joint_channel = pc.createDataChannel("joint-control")
        imu_channel = pc.createDataChannel("imu-control")

        @joint_channel.on("message")
        def on_joint(message):
            try:
                if isinstance(message, bytes): message = message.decode()
                packet = validate_joint_targets(json.loads(message))
                if packet: send_to_gazebo(packet)
                else: print("[DATA] Dropped invalid joint packet")
            except Exception as exc: print("[DATA] Joint message error:", repr(exc))

        @imu_channel.on("message")
        def on_imu(message):
            try:
                if isinstance(message, bytes): message = message.decode()
                packet = validate_phone_orientation(json.loads(message))
                if packet: send_to_gazebo(packet)
                else: print("[DATA] Dropped invalid phone orientation")
            except Exception as exc: print("[DATA] IMU message error:", repr(exc))

    @pc.on("connectionstatechange")
    async def state_changed():
        print(f"[WEBRTC] {viewer} state:", pc.connectionState)

    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)
    deadline = time.monotonic() + 10
    while pc.iceGatheringState != "complete" and time.monotonic() < deadline:
        await asyncio.sleep(0.1)
    await websocket.send(json.dumps({
        "type": pc.localDescription.type, "sdp": pc.localDescription.sdp, "viewer": viewer
    }))
    print(f"[WEBRTC] {viewer} offer sent")
    return {"pc": pc, "camera": camera, "joint": joint_channel, "imu": imu_channel}



async def run_robot():
    while True:
        peers = {}
        try:
            print(f"[ROBOT] {ROBOT_ID} | signaling={SIGNALING_SERVER} | physical follower DISABLED")
            async with websockets.connect(SIGNALING_SERVER, ping_interval=20, ping_timeout=20, max_size=2**20) as websocket:
                await websocket.send(json.dumps({"role": "robot", "robot_id": ROBOT_ID, "token": ROBOT_TOKEN}))
                async for raw in websocket:
                    try: data = json.loads(raw)
                    except json.JSONDecodeError: continue
                    kind = data.get("type")
                    if kind == "authenticated":
                        print("[ROBOT] Signaling authentication successful")
                    elif kind in ("browser_ready", "mobile_ready"):
                        viewer = "browser" if kind == "browser_ready" else "mobile"
                        previous = peers.pop(viewer, None)
                        if previous: await previous["pc"].close()
                        peers[viewer] = await create_webrtc_session(websocket, viewer)
                    elif kind in ("head_orientation", "head_gyro"):
                        packet = validate_phone_orientation(data)
                        if packet: send_to_gazebo(packet)
                    elif kind == "answer":
                        viewer = data.get("viewer", "browser")
                        peer = peers.get(viewer)
                        if peer:
                            await peer["pc"].setRemoteDescription(RTCSessionDescription(sdp=data["sdp"], type=data.get("type", "answer")))
                            print(f"[WEBRTC] {viewer} answer accepted")
                        else: print(f"[WEBRTC] No active {viewer} peer for answer")
                    elif kind in ("browser_disconnected", "mobile_disconnected"):
                        viewer = "browser" if kind == "browser_disconnected" else "mobile"
                        peer = peers.pop(viewer, None)
                        if peer: await peer["pc"].close()
                        print(f"[ROBOT] {viewer} viewer disconnected")
                    else:
                        print("[SERVER] Unhandled message:", kind)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            print("[ROBOT] Connection error:", repr(exc))
        finally:
            for peer in peers.values():
                try: await peer["pc"].close()
                except Exception: pass
                try: peer["camera"].stop()
                except Exception: pass
        print("[ROBOT] Reconnecting in 3 seconds")
        await asyncio.sleep(3)


if __name__ == "__main__":
    try:
        asyncio.run(run_robot())
    except KeyboardInterrupt:
        print("\n[ROBOT] Shutdown requested")
    finally:
        simulation_socket.close()
