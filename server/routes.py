"""
routes.py - HTTP endpoints.

  * the browser app (index.html, app.js, style.css, JS modules, /mobile page)
  * /api/health, /api/login, /api/robots, /api/calibration

WebSocket signaling lives in websocket.py.
"""

import json

from fastapi import APIRouter, Request
from fastapi.responses import FileResponse, Response

from . import robot_manager
from .auth import LoginRequest, login_user, require_user
from .config import (
    CALIBRATION_FALLBACK_FILENAMES,
    CALIBRATION_SEARCH_DIRS,
    CLIENT_DIR,
    DEFAULT_LEADER_CALIBRATION,
)

router = APIRouter()


# ============================================================
# STATIC CLIENT
# ============================================================

@router.get("/")
async def index():
    return FileResponse(CLIENT_DIR / "index.html")


@router.get("/mobile")
async def mobile_gyro_page():
    """Lightweight phone-only gyroscope controller; separate from laptop UI."""
    return FileResponse(CLIENT_DIR / "mobile-index.html")


# Files served from the site root. app.js imports its modules with relative paths
# such as ./ui-controller.js, so every module must live in the same / namespace.
# (url path, file in client/, media type or None to let FastAPI guess)
CLIENT_FILES = [
    ("/app.js", "app.js", None),
    ("/style.css", "style.css", None),
    ("/ui-controller.js", "ui-controller.js", "application/javascript"),
    ("/robot-connection.js", "robot-connection.js", "application/javascript"),
    ("/hand-tracker.js", "hand-tracker.js", "application/javascript"),
    ("/manipulator-controller.js", "manipulator-controller.js", "application/javascript"),
    ("/dual-hand-ik-controller.js", "dual-hand-ik-controller.js", "application/javascript"),
    ("/digital-twin.js", "digital-twin.js", "application/javascript"),
    ("/mobile-gyro.js", "mobile-gyro.js", "application/javascript"),
]


def _client_file_handler(filename: str, media_type):
    async def handler():
        return FileResponse(CLIENT_DIR / filename, media_type=media_type)
    return handler


for _url, _filename, _media_type in CLIENT_FILES:
    router.add_api_route(_url, _client_file_handler(_filename, _media_type), methods=["GET"])


# Browser icon and optional Chrome DevTools discovery request.
# These requests are unrelated to robot control or WebRTC signaling.
@router.get("/favicon.ico", include_in_schema=False)
async def favicon():
    # Small inline SVG avoids requiring an additional image file.
    svg = """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64">
      <rect width="64" height="64" rx="14" fill="#182231"/>
      <circle cx="32" cy="32" r="19" fill="none" stroke="#3d8cff" stroke-width="7"/>
      <circle cx="32" cy="32" r="7" fill="#65e2a2"/>
    </svg>"""
    return Response(content=svg, media_type="image/svg+xml")


@router.get("/.well-known/appspecific/com.chrome.devtools.json", include_in_schema=False)
async def chrome_devtools_discovery():
    # Chrome may probe this optional endpoint. No discovery manifest is used.
    return Response(status_code=204)


# ============================================================
# API
# ============================================================

@router.get("/api/health")
async def health():
    """Simple liveness endpoint; it does not expose credentials."""
    return {"ok": True, "online_robots": robot_manager.online_robot_ids()}


@router.post("/api/login")
async def login(request: LoginRequest):

    token = login_user(request.username, request.password)

    if token is None:
        return {"success": False, "error": "Invalid username or password"}

    return {"success": True, "token": token, "username": request.username}


@router.get("/api/robots")
async def get_robots(request: Request):

    username = require_user(request)

    return {"robots": robot_manager.visible_robots(username)}


@router.get("/api/calibration")
async def get_calibration(request: Request, id: str = "None"):

    require_user(request)

    # Search for lerobot calibration files on the host.
    for calib_dir in CALIBRATION_SEARCH_DIRS:
        for fname in [f"{id}.json", *CALIBRATION_FALLBACK_FILENAMES]:
            candidate = calib_dir / fname
            if candidate.is_file():
                try:
                    with open(candidate, "r") as f:
                        data = json.load(f)
                        return {
                            "success": True,
                            "source": str(candidate),
                            "calibration": data,
                        }
                except Exception:
                    pass

    return {
        "success": True,
        "source": "default_verified",
        "calibration": DEFAULT_LEADER_CALIBRATION,
    }
