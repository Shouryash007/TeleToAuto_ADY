import asyncio
import base64
import hashlib
import hmac
import json
import secrets
import time
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Request
from fastapi.responses import FileResponse
from pydantic import BaseModel


# ============================================================
# CONFIGURATION
# ============================================================

app = FastAPI()

BASE_DIR = Path(__file__).resolve().parent.parent
CLIENT_DIR = BASE_DIR / "client"

CREDENTIALS_DIR = BASE_DIR / "Credentials"

USERS_FILE = CREDENTIALS_DIR / "users.json"
ROBOTS_FILE = CREDENTIALS_DIR / "robots.json"


# ============================================================
# LOAD CONFIGURATION
# ============================================================

with open(USERS_FILE, "r") as f:
    USERS = json.load(f)

with open(ROBOTS_FILE, "r") as f:
    ROBOTS = json.load(f)


# ============================================================
# RUNTIME STATE
# ============================================================

# session_token -> username
sessions = {}

# robot_id -> WebSocket
robot_connections = {}

# robot_id -> WebSocket
browser_connections = {}
browser_users = {}
mobile_connections = {}
mobile_users = {}


# ============================================================
# PASSWORD VERIFICATION
# ============================================================

def verify_password(
    password: str,
    stored_hash: str
) -> bool:

    try:

        algorithm, iterations, salt_b64, hash_b64 = (
            stored_hash.split("$")
        )

        if algorithm != "pbkdf2_sha256":
            return False

        iterations = int(iterations)

        salt = base64.urlsafe_b64decode(
            salt_b64.encode()
        )

        expected = base64.urlsafe_b64decode(
            hash_b64.encode()
        )

        actual = hashlib.pbkdf2_hmac(
            "sha256",
            password.encode(),
            salt,
            iterations
        )

        return hmac.compare_digest(
            actual,
            expected
        )

    except Exception:

        return False


# ============================================================
# LOGIN MODEL
# ============================================================

class LoginRequest(BaseModel):

    username: str
    password: str


# ============================================================
# STATIC CLIENT
# ============================================================

@app.get("/")
async def index():

    return FileResponse(
        CLIENT_DIR / "index.html"
    )


@app.get("/api/health")
async def health():
    """Simple liveness endpoint; it does not expose credentials."""
    return {"ok": True, "online_robots": sorted(robot_connections.keys())}


@app.get("/app.js")
async def app_js():

    return FileResponse(
        CLIENT_DIR / "app.js"
    )


@app.get("/style.css")
async def style_css():

    return FileResponse(
        CLIENT_DIR / "style.css"
    )

# JavaScript feature modules imported by app.js (ES modules).
# Each module must be served from the same / URL namespace because the
# imports in app.js are relative paths such as ./ui-controller.js.
@app.get("/ui-controller.js")
async def ui_controller_js():
    return FileResponse(CLIENT_DIR / "ui-controller.js", media_type="application/javascript")


@app.get("/robot-connection.js")
async def robot_connection_js():
    return FileResponse(CLIENT_DIR / "robot-connection.js", media_type="application/javascript")


@app.get("/hand-tracker.js")
async def hand_tracker_js():
    return FileResponse(CLIENT_DIR / "hand-tracker.js", media_type="application/javascript")


@app.get("/manipulator-controller.js")
async def manipulator_controller_js():
    return FileResponse(CLIENT_DIR / "manipulator-controller.js", media_type="application/javascript")


@app.get("/dual-hand-ik-controller.js")
async def dual_hand_ik_controller_js():
    """Serve the browser-side dual-hand Jacobian/DLS inverse-kinematics module."""
    return FileResponse(
        CLIENT_DIR / "dual-hand-ik-controller.js",
        media_type="application/javascript"
    )


@app.get("/digital-twin.js")
async def digital_twin_js():
    return FileResponse(CLIENT_DIR / "digital-twin.js", media_type="application/javascript")


@app.get("/mobile")
async def mobile_gyro_page():
    """Lightweight phone-only gyroscope controller; separate from laptop UI."""
    return FileResponse(CLIENT_DIR / "mobile-index.html")

@app.get("/mobile-gyro.js")
async def mobile_gyro_js():
    return FileResponse(CLIENT_DIR / "mobile-gyro.js", media_type="application/javascript")


# ============================================================
# LOGIN
# ============================================================

@app.post("/api/login")
async def login(request: LoginRequest):

    username = request.username
    password = request.password

    user = USERS.get(username)

    if not user:

        return {
            "success": False,
            "error": "Invalid username or password"
        }

    if not verify_password(
        password,
        user["password_hash"]
    ):

        return {
            "success": False,
            "error": "Invalid username or password"
        }

    # Create temporary session token
    token = secrets.token_urlsafe(32)

    sessions[token] = username

    print(
        f"[AUTH] User logged in: {username}"
    )

    return {
        "success": True,
        "token": token,
        "username": username
    }


# ============================================================
# ROBOT LIST
# ============================================================

@app.get("/api/robots")
async def get_robots(request: Request):

    # ------------------------------------------------
    # Get browser session token
    # ------------------------------------------------

    token = request.headers.get(
        "Authorization",
        ""
    ).replace("Bearer ", "")

    username = sessions.get(token)

    if not username:
        raise HTTPException(
            status_code=401,
            detail="Invalid browser session"
        )

    # ------------------------------------------------
    # Return only robots this user is allowed to access
    # ------------------------------------------------

    visible_robots = []

    for robot_id, robot_info in ROBOTS.items():

        if username not in robot_info.get(
            "allowed_users",
            []
        ):
            continue

        visible_robots.append({
            "robot_id": robot_id,
            "online": robot_id in robot_connections,
            "busy": robot_id in browser_connections,
            "used_by": browser_users.get(robot_id),
            "mobile_busy": robot_id in mobile_connections,
            "mobile_used_by": mobile_users.get(robot_id)
        })

    return {
        "robots": visible_robots
    }


# ============================================================
# SO-101 LEADER CALIBRATION
# ============================================================

@app.get("/api/calibration")
async def get_calibration(request: Request, id: str = "None"):
    token = request.headers.get(
        "Authorization",
        ""
    ).replace("Bearer ", "")

    username = sessions.get(token)
    if not username:
        raise HTTPException(
            status_code=401,
            detail="Invalid browser session"
        )

    # Search for lerobot calibration files on host
    calib_dirs = [
        Path.home() / ".cache" / "huggingface" / "lerobot" / "calibration" / "teleoperators" / "so_leader",
        BASE_DIR
    ]

    for c_dir in calib_dirs:
        for fname in [f"{id}.json", "None.json", "my_leader_arm.json"]:
            candidate = c_dir / fname
            if candidate.is_file():
                try:
                    with open(candidate, "r") as f:
                        data = json.load(f)
                        return {
                            "success": True,
                            "source": str(candidate),
                            "calibration": data
                        }
                except Exception:
                    pass

    # Verified fallback calibration for SO-101 leader
    return {
        "success": True,
        "source": "default_verified",
        "calibration": {
            "shoulder_pan": {"id": 1, "drive_mode": 0, "homing_offset": -106, "range_min": 816, "range_max": 3543},
            "shoulder_lift": {"id": 2, "drive_mode": 0, "homing_offset": 1016, "range_min": 833, "range_max": 3222},
            "elbow_flex": {"id": 3, "drive_mode": 0, "homing_offset": -5, "range_min": 861, "range_max": 3089},
            "wrist_flex": {"id": 4, "drive_mode": 0, "homing_offset": 143, "range_min": 781, "range_max": 3116},
            "wrist_roll": {"id": 5, "drive_mode": 0, "homing_offset": 7, "range_min": 0, "range_max": 4095},
            "gripper": {"id": 6, "drive_mode": 0, "homing_offset": -133, "range_min": 1535, "range_max": 2709}
        }
    }


# ============================================================
# WEBSOCKET
# ============================================================

@app.websocket("/ws")
async def websocket_endpoint(
    websocket: WebSocket
):

    await websocket.accept()

    role = None
    robot_id = None
    username = None

    try:

        # ====================================================
        # FIRST MESSAGE = AUTHENTICATION
        # ====================================================

        raw = await websocket.receive_text()

        data = json.loads(raw)

        role = data.get("role")

        # ====================================================
        # ROBOT AUTHENTICATION
        # ====================================================

        if role == "robot":

            robot_id = data.get(
                "robot_id"
            )

            robot_token = data.get(
                "token"
            )

            robot = ROBOTS.get(
                robot_id
            )

            if not robot:

                await websocket.close(
                    code=4001
                )

                return

            if not hmac.compare_digest(
                robot_token or "",
                robot["token"]
            ):

                print(
                    f"[AUTH] Robot authentication "
                    f"failed: {robot_id}"
                )

                await websocket.close(
                    code=4003
                )

                return

            # ------------------------------------------------
            # Save robot connection
            # ------------------------------------------------

            old_connection = (
                robot_connections.get(
                    robot_id
                )
            )

            if old_connection:

                try:

                    await old_connection.close(
                        code=4002
                    )

                except Exception:
                    pass

            robot_connections[
                robot_id
            ] = websocket

            print(
                f"[ROBOT] {robot_id} connected"
            )

            await websocket.send_text(
                json.dumps({
                    "type": "authenticated",
                    "robot_id": robot_id
                })
            )

        # ====================================================
        # BROWSER AUTHENTICATION
        # ====================================================

        elif role == "mobile_gyro":
            # Phone sensor connections are independent from the laptop browser session.
            # They authenticate separately and never replace browser_connections.
            session_token = data.get("token")
            robot_id = data.get("robot_id")
            username = sessions.get(session_token)
            if not username:
                await websocket.send_text(json.dumps({
                    "type": "error", "message": "Invalid mobile session. Please log in again."
                }))
                await websocket.close(code=4003)
                return

            robot = ROBOTS.get(robot_id)
            if not robot:
                await websocket.send_text(json.dumps({
                    "type": "error", "message": "Robot does not exist"
                }))
                await websocket.close(code=4001)
                return

            if username not in robot.get("allowed_users", []):
                await websocket.send_text(json.dumps({
                    "type": "error", "message": "You are not authorized to access this robot"
                }))
                await websocket.close(code=4003)
                return

            if robot_id not in robot_connections:
                await websocket.send_text(json.dumps({
                    "type": "error", "message": "Robot is offline"
                }))
                await websocket.close(code=4004)
                return

            # The phone has an independent slot; it does not require a browser reservation.
            existing_mobile = mobile_connections.get(robot_id)
            if existing_mobile is not None and existing_mobile is not websocket:
                if mobile_users.get(robot_id) != username:
                    await websocket.send_text(json.dumps({
                        "type": "error",
                        "message": f"Robot is already controlled by {mobile_users.get(robot_id)}"
                    }))
                    await websocket.close(code=4004)
                    return
                # Same user reconnecting after a page reload: replace the stale socket.
                try:
                    await existing_mobile.close(code=4002)
                except Exception:
                    pass

            mobile_connections[robot_id] = websocket
            mobile_users[robot_id] = username
            await websocket.send_text(json.dumps({
                "type": "mobile_gyro_connected", "robot_id": robot_id, "username": username
            }))
            await robot_connections[robot_id].send_text(json.dumps({"type": "mobile_ready"}))
            print(f"[MOBILE VIDEO] {username} connected to {robot_id}")

        elif role == "browser":

            session_token = data.get(
                "token"
            )

            robot_id = data.get(
                "robot_id"
            )

            # ------------------------------------------------
            # Validate session
            # ------------------------------------------------

            username = sessions.get(
                session_token
            )

            if not username:

                print(
                    "[AUTH] Invalid browser session"
                )

                await websocket.close(
                    code=4003
                )

                return

            # ------------------------------------------------
            # Validate robot
            # ------------------------------------------------

            robot = ROBOTS.get(
                robot_id
            )

            if not robot:

                await websocket.send_text(
                    json.dumps({
                        "type": "error",
                        "message":
                            "Robot does not exist"
                    })
                )

                await websocket.close()

                return

            # ------------------------------------------------
            # Check authorization
            # ------------------------------------------------

            if username not in robot.get(
                "allowed_users",
                []
            ):

                await websocket.send_text(
                    json.dumps({
                        "type": "error",
                        "message":
                            "You are not authorized "
                            "to access this robot"
                    })
                )

                await websocket.close()

                return

            # ------------------------------------------------
            # Check robot online
            # ------------------------------------------------

            if robot_id not in robot_connections:

                await websocket.send_text(
                    json.dumps({
                        "type": "error",
                        "message":
                            "Robot is offline"
                    })
                )

                await websocket.close()

                return

            # ------------------------------------------------
            # Only one operator can reserve a robot at a time.

            existing_browser = browser_connections.get(robot_id)

            if existing_browser is not None and existing_browser is not websocket:
                if browser_users.get(robot_id) != username:
                    await websocket.send_text(json.dumps({
                        "type": "error",
                        "message": "Robot is currently being used by another operator"
                    }))
                    await websocket.close(code=4004)
                    print(
                        f"[ACCESS] {username} denied access to "
                        f"{robot_id} - already in use"
                    )
                    return

                # Same user reconnecting after a page reload: replace stale socket.
                try:
                    await existing_browser.close(code=4002)
                except Exception:
                    pass

            # Reserve the browser slot (or replace this user's stale socket).
            browser_connections[robot_id] = websocket
            browser_users[robot_id] = username

            print(
                f"[USER] {username} connected "
                f"to {robot_id}"
            )

            await websocket.send_text(
                json.dumps({
                    "type": "connected",
                    "robot_id": robot_id
                })
            )

            # Tell robot that browser is ready
            await robot_connections[
                robot_id
            ].send_text(
                json.dumps({
                    "type": "browser_ready"
                })
            )

            print(
                f"[SIGNAL] Browser ready -> "
                f"{robot_id}"
            )

        else:

            await websocket.close(
                code=4001
            )

            return

        # ====================================================
        # MESSAGE FORWARDING
        # ====================================================

        while True:

            raw = await websocket.receive_text()

            data = json.loads(raw)

            message_type = data.get(
                "type"
            )

            # ------------------------------------------------
            # ROBOT -> BROWSER
            # ------------------------------------------------

            if role == "mobile_gyro":
                if message_type in ("answer", "candidate"):
                    robot = robot_connections.get(robot_id)
                    if robot:
                        data["viewer"] = "mobile"
                        await robot.send_text(json.dumps(data))
                    continue
                # Phone uses the same calibrated DeviceOrientation angles as the website.
                if message_type != "head_orientation":
                    continue
                orientation = data.get("orientation")
                if not isinstance(orientation, dict):
                    continue
                try:
                    yaw = float(orientation.get("yaw"))
                    pitch = float(orientation.get("pitch"))
                    roll = float(orientation.get("roll", 0.0))
                    sent_ms = float(data.get("timestamp_ms"))
                except (TypeError, ValueError):
                    continue
                import math
                if (
                    not all(math.isfinite(v) for v in (yaw, pitch, roll, sent_ms))
                    or abs((time.time() * 1000.0) - sent_ms) > 1000.0
                    or any(abs(v) > math.pi for v in (yaw, pitch, roll))
                ):
                    continue
                robot = robot_connections.get(robot_id)
                if robot:
                    packet = {
                        "type": "head_orientation",
                        "sequence": data.get("sequence"),
                        "timestamp_ms": data.get("timestamp_ms"),
                        "units": "rad",
                        "orientation": {"yaw": yaw, "pitch": pitch, "roll": roll},
                        "source": "phone_device_orientation"
                    }
                    await robot.send_text(json.dumps(packet))

                # Also mirror the phone's GX/GY values to the reserved laptop UI.
                # GX is relative yaw and GY is relative pitch, displayed in degrees.
                browser = browser_connections.get(robot_id)
                if browser:
                    readout = {
                        "type": "mobile_imu_readout",
                        "gx": yaw * 180.0 / math.pi,
                        "gy": pitch * 180.0 / math.pi,
                        "units": "deg",
                        "source": "phone_device_orientation",
                        "timestamp_ms": sent_ms
                    }
                    try:
                        await browser.send_text(json.dumps(readout))
                    except Exception as exc:
                        print(f"[MOBILE GYRO] Laptop readout forwarding failed: {exc}")
            elif role == "robot":
                viewer = data.get("viewer", "browser")
                target = mobile_connections.get(robot_id) if viewer == "mobile" else browser_connections.get(robot_id)
                if target:
                    await target.send_text(raw)
                    print(f"[SIGNAL] {robot_id} -> {viewer}: {message_type}")

            # ------------------------------------------------
            # BROWSER -> ROBOT
            # ------------------------------------------------

            elif role == "browser":

                robot = (
                    robot_connections.get(
                        robot_id
                    )
                )

                if robot:

                    await robot.send_text(
                        raw
                    )

                    print(
                        f"[SIGNAL] browser "
                        f"-> {robot_id}: "
                        f"{message_type}"
                    )

    except WebSocketDisconnect:

        print(
            f"[WS] {role} disconnected"
        )

    except Exception as e:

        print(
            f"[WS] Error: {repr(e)}"
        )

    finally:

        # ====================================================
        # ROBOT CLEANUP
        # ====================================================

        if role == "robot":

            if (
                robot_id
                and robot_connections.get(
                    robot_id
                ) is websocket
            ):

                del robot_connections[
                    robot_id
                ]

                print(
                    f"[ROBOT] {robot_id} "
                    f"disconnected"
                )

                browser = (
                    browser_connections.get(
                        robot_id
                    )
                )

                if browser:

                    try:

                        await browser.send_text(
                            json.dumps({
                                "type":
                                    "robot_disconnected"
                            })
                        )

                    except Exception:
                        pass

                mobile = mobile_connections.get(robot_id)
                if mobile:
                    try:
                        await mobile.send_text(json.dumps({
                            "type": "robot_disconnected"
                        }))
                    except Exception:
                        pass

        # ====================================================
        # MOBILE CLEANUP
        # ====================================================
        elif role == "mobile_gyro":
            if robot_id and mobile_connections.get(robot_id) is websocket:
                mobile_connections.pop(robot_id, None)
                mobile_users.pop(robot_id, None)
                robot = robot_connections.get(robot_id)
                if robot:
                    try: await robot.send_text(json.dumps({"type": "mobile_disconnected"}))
                    except Exception: pass
                print(f"[MOBILE VIDEO] {username} disconnected")

        # ====================================================
        # BROWSER CLEANUP
        # ====================================================

        elif role == "browser":

            if (
                robot_id
                and browser_connections.get(robot_id)
                is websocket
            ):
                del browser_connections[robot_id]
                browser_users.pop(robot_id, None)

                print(
                    f"[USER] {username} "
                    f"disconnected from "
                    f"{robot_id}"
                )

                robot = (
                    robot_connections.get(
                        robot_id
                    )
                )

                if robot:

                    try:

                        await robot.send_text(
                            json.dumps({
                                "type":
                                    "browser_disconnected"
                            })
                        )

                    except Exception:
                        pass