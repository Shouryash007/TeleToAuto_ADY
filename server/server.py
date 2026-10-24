import asyncio
import base64
import hashlib
import hmac
import json
import secrets
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

USERS_FILE = BASE_DIR / "users.json"
ROBOTS_FILE = BASE_DIR / "robots.json"


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
            "used_by": browser_users.get(robot_id)
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

            existing_browser = browser_connections.get(
                robot_id
            )

            if existing_browser:

                await websocket.send_text(
                    json.dumps({
                        "type": "error",
                        "message":
                            "Robot is currently being used by another operator"
                    })
                )

                await websocket.close(
                    code=4004
                )

                print(
                    f"[ACCESS] {username} denied access to "
                    f"{robot_id} - already in use"
                )

                return


            # No existing operator -> reserve robot
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

            if role == "robot":

                browser = (
                    browser_connections.get(
                        robot_id
                    )
                )

                if browser:

                    await browser.send_text(
                        raw
                    )

                    print(
                        f"[SIGNAL] {robot_id} "
                        f"-> browser: "
                        f"{message_type}"
                    )

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