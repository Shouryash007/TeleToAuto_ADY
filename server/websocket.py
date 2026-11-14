"""
websocket.py - the /ws endpoint: authentication and message routing.

Flow of one connection:
  1. accept, read the FIRST message -> {"role": "robot" | "browser" | "mobile_gyro", ...}
  2. authenticate according to the role (_authenticate_*)
  3. forward messages until the socket closes (_forward_*)
  4. clean up the slot it held (always, in `finally`)

Who talks to whom:
  robot  -> browser or phone   (WebRTC signaling, chosen by the "viewer" field)
  browser -> robot             (signaling, joint targets, ...)
  phone  -> robot              (answer/candidate signaling + head_orientation)
                               and a gx/gy readout mirrored to the browser
The connection tables and slot rules live in robot_manager.py.
"""

import json
import math
import time

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from . import robot_manager as manager
from .auth import check_robot_credentials, user_can_access_robot, username_for_token
from .config import (
    MOBILE_MAX_PACKET_AGE_MS,
    WS_CLOSE_BAD_REQUEST,
    WS_CLOSE_UNAUTHORIZED,
    WS_CLOSE_UNAVAILABLE,
)

router = APIRouter()


async def _send_error(websocket: WebSocket, message: str) -> None:
    await websocket.send_text(json.dumps({"type": "error", "message": message}))


# ============================================================
# AUTHENTICATION (first message)
# Each helper returns True / the username on success. On failure it has
# already informed the client and closed the socket, and returns None/False.
# ============================================================

async def _authenticate_robot(websocket: WebSocket, data: dict) -> bool:
    robot_id = data.get("robot_id")

    rejected = check_robot_credentials(robot_id, data.get("token"))
    if rejected is not None:
        await websocket.close(code=rejected)
        return False

    await manager.register_robot(robot_id, websocket)

    await websocket.send_text(json.dumps({"type": "authenticated", "robot_id": robot_id}))
    return True


async def _authenticate_browser(websocket: WebSocket, data: dict):
    robot_id = data.get("robot_id")

    # --- session ---
    username = username_for_token(data.get("token"))

    if not username:
        print("[AUTH] Invalid browser session")
        await websocket.close(code=WS_CLOSE_UNAUTHORIZED)
        return None

    # --- robot exists / allowed / online ---
    robot = manager.get_robot(robot_id)

    if not robot:
        await _send_error(websocket, "Robot does not exist")
        await websocket.close()
        return None

    if not user_can_access_robot(username, robot):
        await _send_error(websocket, "You are not authorized to access this robot")
        await websocket.close()
        return None

    if not manager.is_online(robot_id):
        await _send_error(websocket, "Robot is offline")
        await websocket.close()
        return None

    # --- only one operator can reserve a robot at a time ---
    ok, _holder = await manager.claim_browser(robot_id, username, websocket)

    if not ok:
        await _send_error(websocket, "Robot is currently being used by another operator")
        await websocket.close(code=WS_CLOSE_UNAVAILABLE)
        return None

    await websocket.send_text(json.dumps({"type": "connected", "robot_id": robot_id}))

    # Tell the robot that the browser is ready.
    await manager.send_to_robot(robot_id, json.dumps({"type": "browser_ready"}))

    print(f"[SIGNAL] Browser ready -> {robot_id}")

    return username


async def _authenticate_mobile(websocket: WebSocket, data: dict):
    # The phone authenticates separately and never replaces the laptop's browser slot.
    robot_id = data.get("robot_id")

    username = username_for_token(data.get("token"))

    if not username:
        await _send_error(websocket, "Invalid mobile session. Please log in again.")
        await websocket.close(code=WS_CLOSE_UNAUTHORIZED)
        return None

    robot = manager.get_robot(robot_id)

    if not robot:
        await _send_error(websocket, "Robot does not exist")
        await websocket.close(code=WS_CLOSE_BAD_REQUEST)
        return None

    if not user_can_access_robot(username, robot):
        await _send_error(websocket, "You are not authorized to access this robot")
        await websocket.close(code=WS_CLOSE_UNAUTHORIZED)
        return None

    if not manager.is_online(robot_id):
        await _send_error(websocket, "Robot is offline")
        await websocket.close(code=WS_CLOSE_UNAVAILABLE)
        return None

    # The phone has its own slot; it does not need a browser reservation.
    ok, holder = await manager.claim_mobile(robot_id, username, websocket)

    if not ok:
        await _send_error(websocket, f"Robot is already controlled by {holder}")
        await websocket.close(code=WS_CLOSE_UNAVAILABLE)
        return None

    await websocket.send_text(json.dumps({
        "type": "mobile_gyro_connected", "robot_id": robot_id, "username": username
    }))
    await manager.send_to_robot(robot_id, json.dumps({"type": "mobile_ready"}))

    print(f"[MOBILE VIDEO] {username} connected to {robot_id}")

    return username


# ============================================================
# MESSAGE FORWARDING
# ============================================================

async def _forward_from_robot(robot_id, data: dict, raw: str) -> None:
    """ROBOT -> BROWSER or PHONE (chosen by the "viewer" field)."""
    viewer = data.get("viewer", "browser")

    if viewer == "mobile":
        target = manager.mobile_connections.get(robot_id)
    else:
        target = manager.browser_connections.get(robot_id)

    if target:
        await target.send_text(raw)
        print(f"[SIGNAL] {robot_id} -> {viewer}: {data.get('type')}")


def _validate_joint_targets(data: dict):
    """Validate browser hand-tracking commands before server-side routing."""
    if data.get("type") != "joint_targets" or data.get("units") != "deg":
        return None

    try:
        sent_ms = float(data.get("timestamp_ms"))
    except (TypeError, ValueError):
        return None
    if not math.isfinite(sent_ms) or abs(time.time() * 1000.0 - sent_ms) > 1000.0:
        return None

    arms = data.get("arms")
    gripper = data.get("gripper")
    if not isinstance(arms, dict) or not isinstance(gripper, dict):
        return None

    joint_names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
    normalized_arms = {"left": {}, "right": {}}
    for side in ("left", "right"):
        side_values = arms.get(side)
        if not isinstance(side_values, dict):
            return None
        for name in joint_names:
            try:
                value = float(side_values.get(name))
            except (TypeError, ValueError):
                return None
            if not math.isfinite(value) or abs(value) > 180.0:
                return None
            normalized_arms[side][name] = value

    normalized_gripper = {}
    for side in ("left", "right"):
        key = f"{side}_percent"
        if key in gripper:
            try:
                percent = float(gripper[key])
            except (TypeError, ValueError):
                return None
        else:
            # Compatibility with older browser clients that sent a boolean.
            legacy = gripper.get(f"{side}_closed", False)
            if not isinstance(legacy, bool):
                return None
            percent = 100.0 if legacy else 0.0
        if not math.isfinite(percent) or not 0.0 <= percent <= 100.0:
            return None
        normalized_gripper[key] = percent

    return {
        "type": "joint_targets",
        "sequence": data.get("sequence"),
        "timestamp_ms": sent_ms,
        "units": "deg",
        "source": data.get("source", "browser_hand_tracking"),
        "arms": normalized_arms,
        "gripper": normalized_gripper,
    }


async def _forward_from_browser(robot_id, data: dict, raw: str) -> None:
    """BROWSER -> ROBOT. Joint targets are validated before forwarding."""
    message_type = data.get("type")

    if message_type == "joint_targets":
        packet = _validate_joint_targets(data)
        if packet is None:
            print(f"[SAFETY] Rejected invalid browser joint packet for {robot_id}")
            return
        raw = json.dumps(packet, separators=(",", ":"))
    elif message_type not in ("answer", "candidate"):
        # Browser is not allowed to spoof server lifecycle or robot messages.
        return

    if await manager.send_to_robot(robot_id, raw):
        print(f"[ROUTING] browser -> server -> {robot_id}: {message_type}")


async def _forward_from_mobile(robot_id, data: dict) -> None:
    """PHONE -> ROBOT (signaling + validated head orientation) and -> browser readout."""
    message_type = data.get("type")

    if message_type in ("answer", "candidate"):
        robot = manager.robot_connections.get(robot_id)
        if robot:
            data["viewer"] = "mobile"
            await robot.send_text(json.dumps(data))
        return

    # Phone uses the same calibrated DeviceOrientation angles as the website.
    if message_type != "head_orientation":
        return

    orientation = data.get("orientation")
    if not isinstance(orientation, dict):
        return

    try:
        yaw = float(orientation.get("yaw"))
        pitch = float(orientation.get("pitch"))
        roll = float(orientation.get("roll", 0.0))
        sent_ms = float(data.get("timestamp_ms"))
    except (TypeError, ValueError):
        return

    if (
        not all(math.isfinite(v) for v in (yaw, pitch, roll, sent_ms))
        or abs((time.time() * 1000.0) - sent_ms) > MOBILE_MAX_PACKET_AGE_MS
        or any(abs(v) > math.pi for v in (yaw, pitch, roll))
    ):
        return

    robot = manager.robot_connections.get(robot_id)
    if robot:
        packet = {
            "type": "head_orientation",
            "sequence": data.get("sequence"),
            "timestamp_ms": data.get("timestamp_ms"),
            "units": "rad",
            "orientation": {"yaw": yaw, "pitch": pitch, "roll": roll},
            "source": "phone_device_orientation",
        }
        await robot.send_text(json.dumps(packet))

    # Also mirror the phone's GX/GY values to the reserved laptop UI.
    # GX is relative yaw and GY is relative pitch, displayed in degrees.
    browser = manager.browser_connections.get(robot_id)
    if browser:
        readout = {
            "type": "mobile_imu_readout",
            "gx": yaw * 180.0 / math.pi,
            "gy": pitch * 180.0 / math.pi,
            "units": "deg",
            "source": "phone_device_orientation",
            "timestamp_ms": sent_ms,
        }
        try:
            await browser.send_text(json.dumps(readout))
        except Exception as exc:
            print(f"[MOBILE GYRO] Laptop readout forwarding failed: {exc}")


async def _message_loop(websocket: WebSocket, role: str, robot_id) -> None:
    while True:
        raw = await websocket.receive_text()
        data = json.loads(raw)

        if role == "mobile_gyro":
            await _forward_from_mobile(robot_id, data)
        elif role == "robot":
            await _forward_from_robot(robot_id, data, raw)
        elif role == "browser":
            await _forward_from_browser(robot_id, data, raw)


# ============================================================
# ENDPOINT
# ============================================================

@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):

    await websocket.accept()

    role = None
    robot_id = None
    username = None

    try:
        # First message = authentication.
        data = json.loads(await websocket.receive_text())

        role = data.get("role")
        robot_id = data.get("robot_id")

        if role == "robot":
            if not await _authenticate_robot(websocket, data):
                return

        elif role == "mobile_gyro":
            username = await _authenticate_mobile(websocket, data)
            if username is None:
                return

        elif role == "browser":
            username = await _authenticate_browser(websocket, data)
            if username is None:
                return

        else:
            await websocket.close(code=WS_CLOSE_BAD_REQUEST)
            return

        await _message_loop(websocket, role, robot_id)

    except WebSocketDisconnect:
        print(f"[WS] {role} disconnected")

    except Exception as e:
        print(f"[WS] Error: {repr(e)}")

    finally:
        # Free whatever slot this socket held (each call checks it still owns it).
        if role == "robot":
            await manager.unregister_robot(robot_id, websocket)
        elif role == "mobile_gyro":
            await manager.release_mobile(robot_id, username, websocket)
        elif role == "browser":
            await manager.release_browser(robot_id, username, websocket)
