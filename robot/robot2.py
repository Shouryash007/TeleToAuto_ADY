import asyncio
import json
import os

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


# ============================================================
# CONFIGURATION
# ============================================================

ROBOT_ID = os.getenv(
    "ROBOT_ID",
    "robot_002"
)

ROBOT_TOKEN = os.getenv(
    "ROBOT_TOKEN",
    "robot002secretdef"
)

SIGNALING_SERVER = os.getenv(
    "SIGNALING_SERVER",
    "ws://127.0.0.1:8000/ws"
)

CAMERA_DEVICE = os.getenv(
    "CAMERA_DEVICE",
    "/dev/video3"
)

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30


# ============================================================
# ICE SERVERS
# ============================================================

ICE_SERVERS = []


STUN_URL = os.getenv(
    "STUN_URL",
    ""
)

TURN_URL = os.getenv(
    "TURN_URL",
    ""
)

TURN_USERNAME = os.getenv(
    "TURN_USERNAME",
    ""
)

TURN_PASSWORD = os.getenv(
    "TURN_PASSWORD",
    "")


if STUN_URL:

    ICE_SERVERS.append(
        RTCIceServer(
            urls=STUN_URL
        )
    )


if TURN_URL:

    ICE_SERVERS.append(
        RTCIceServer(
            urls=TURN_URL,
            username=TURN_USERNAME,
            credential=TURN_PASSWORD
        )
    )


RTC_CONFIG = RTCConfiguration(
    iceServers=ICE_SERVERS
)


# ============================================================
# CAMERA
# ============================================================

class CameraVideoTrack(VideoStreamTrack):

    def __init__(self):

        super().__init__()

        print(
            f"[CAMERA] Opening {CAMERA_DEVICE}"
        )

        self.cap = cv2.VideoCapture(
            CAMERA_DEVICE,
            cv2.CAP_V4L2
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            CAMERA_WIDTH
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            CAMERA_HEIGHT
        )

        self.cap.set(
            cv2.CAP_PROP_FPS,
            CAMERA_FPS
        )

        if not self.cap.isOpened():

            raise RuntimeError(
                f"Cannot open {CAMERA_DEVICE}"
            )

        print(
            "[CAMERA] Camera opened"
        )


    async def recv(self):

        ret, frame = self.cap.read()

        if not ret:

            print(
                "[CAMERA] Failed to read frame"
            )

            await asyncio.sleep(
                0.01
            )

            return await self.recv()

        video_frame = (
            VideoFrame.from_ndarray(
                frame,
                format="bgr24"
            )
        )

        pts, time_base = (
            await self.next_timestamp()
        )

        video_frame.pts = pts

        video_frame.time_base = (
            time_base
        )

        return video_frame


    def stop(self):

        print(
            "[CAMERA] Releasing camera"
        )

        if self.cap.isOpened():

            self.cap.release()

        super().stop()


# ============================================================
# WAIT FOR ICE GATHERING
# ============================================================

async def wait_for_ice_gathering(
    pc
):

    while pc.iceGatheringState != "complete":

        await asyncio.sleep(
            0.1
        )


# ============================================================
# WEBRTC SESSION
# ============================================================

async def create_session(
    websocket
):

    print(
        "[ROBOT] Creating WebRTC session"
    )

    # New camera for every session
    camera = CameraVideoTrack()

    pc = RTCPeerConnection(
        RTC_CONFIG
    )

    pc.addTrack(
        camera
    )

    # Create a bidirectional WebRTC DataChannel.
    encoder_channel = pc.createDataChannel("encoder-control")

    last_log_time = 0.0
    msg_count = 0
    first_logged = False

    @encoder_channel.on("open")
    def on_encoder_channel_open():
        print("[DATA] Encoder DataChannel is open")

    @encoder_channel.on("close")
    def on_encoder_channel_close():
        print("[DATA] Encoder DataChannel is closed")

    @encoder_channel.on("message")
    def on_encoder_message(message):
        nonlocal last_log_time, msg_count, first_logged
        try:
            if isinstance(message, bytes):
                message = message.decode("utf-8")

            data = json.loads(message)

            msg_type = data.get("type")
            if msg_type == "test":
                print("[DATA] Test message received:", data)

            elif msg_type == "joint_positions":
                msg_count += 1
                now = asyncio.get_event_loop().time()
                seq = data.get("sequence", "?")
                positions = data.get("positions", {})

                expected_joints = [
                    "shoulder_pan.pos",
                    "shoulder_lift.pos",
                    "elbow_flex.pos",
                    "wrist_flex.pos",
                    "wrist_roll.pos",
                    "gripper.pos"
                ]

                def fmt(val):
                    return f"{val:.1f}" if isinstance(val, (int, float)) else str(val)

                summary = ", ".join(
                    f"{j.replace('.pos', '')}={fmt(positions.get(j, '--'))}"
                    for j in expected_joints
                )

                if not first_logged:
                    first_logged = True
                    last_log_time = now
                    print(f"[DATA] Initial joint_positions received (seq={seq}): {summary}")
                elif now - last_log_time >= 1.0:
                    dt = now - last_log_time
                    rate = msg_count / dt if dt > 0 else 0
                    print(f"[DATA] joint_positions (seq={seq}, rate={rate:.1f}Hz, batch={msg_count}): {summary}")
                    last_log_time = now
                    msg_count = 0

            else:
                print(f"[DATA] Unrecognized message type: {msg_type}")

        except Exception as exc:
            print("[DATA] Invalid message:", repr(exc))

    print(
        "[ROBOT] Camera track added"
    )


    @pc.on("connectionstatechange")
    async def connection_state():

        print(
            "[ROBOT] WebRTC:",
            pc.connectionState
        )


    @pc.on("iceconnectionstatechange")
    async def ice_state():

        print(
            "[ROBOT] ICE:",
            pc.iceConnectionState
        )


    # --------------------------------------------------------
    # CREATE OFFER
    # --------------------------------------------------------

    offer = await pc.createOffer()

    await pc.setLocalDescription(
        offer
    )

    # Wait until STUN/TURN candidates have been gathered
    await wait_for_ice_gathering(
        pc
    )

    print(
        "[ROBOT] ICE gathering complete"
    )

    await websocket.send(
        json.dumps({
            "type":
                pc.localDescription.type,

            "sdp":
                pc.localDescription.sdp
        })
    )

    print(
        "[ROBOT] SDP offer sent"
    )

    return pc, camera


# ============================================================
# MAIN ROBOT LOOP
# ============================================================

async def run_robot():

    while True:

        pc = None
        camera = None

        try:

            print()
            print(
                "===================================="
            )

            print(
                f"[ROBOT] {ROBOT_ID}"
            )

            print(
                "[ROBOT] Connecting to server..."
            )

            print(
                "===================================="
            )


            async with websockets.connect(
                SIGNALING_SERVER
            ) as websocket:

                print(
                    "[ROBOT] Connected"
                )

                # Authenticate robot
                await websocket.send(
                    json.dumps({
                        "role": "robot",
                        "robot_id":
                            ROBOT_ID,
                        "token":
                            ROBOT_TOKEN
                    })
                )


                async for raw in websocket:

                    data = json.loads(
                        raw
                    )

                    message_type = (
                        data.get("type")
                    )

                    print(
                        "[ROBOT] Received:",
                        message_type
                    )


                    # ==========================================
                    # BROWSER READY
                    # ==========================================

                    if message_type == (
                        "browser_ready"
                    ):

                        # Clean old session
                        if pc:

                            await pc.close()

                            pc = None


                        if camera:

                            camera.stop()

                            camera = None


                        # New WebRTC session
                        pc, camera = (
                            await create_session(
                                websocket
                            )
                        )


                    # ==========================================
                    # SDP ANSWER
                    # ==========================================

                    elif message_type == "answer":

                        if pc:

                            answer = (
                                RTCSessionDescription(
                                    sdp=data["sdp"],
                                    type=data["type"]
                                )
                            )

                            await pc.setRemoteDescription(
                                answer
                            )

                            print(
                                "[ROBOT] Remote "
                                "description set"
                            )


                    # ==========================================
                    # BROWSER DISCONNECTED
                    # ==========================================

                    elif message_type == (
                        "browser_disconnected"
                    ):

                        print(
                            "[ROBOT] Browser left"
                        )


                        if pc:

                            await pc.close()

                            pc = None


                        if camera:

                            camera.stop()

                            camera = None


                        print(
                            "[ROBOT] Waiting..."
                        )


        except Exception as e:

            print(
                "[ROBOT] Connection error:",
                repr(e)
            )


        finally:

            if pc:

                try:

                    await pc.close()

                except Exception:
                    pass


            if camera:

                try:

                    camera.stop()

                except Exception:
                    pass


            print(
                "[ROBOT] Reconnecting in 3 seconds..."
            )

            await asyncio.sleep(
                3
            )


# ============================================================
# START
# ============================================================

if __name__ == "__main__":

    try:

        asyncio.run(
            run_robot()
        )

    except KeyboardInterrupt:

        print(
            "[ROBOT] Shutdown"
        )