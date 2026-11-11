"""
config.py - paths, settings and data files.

Everything that is "configuration" lives here so the other modules stay free of
file paths and constants. Importing this module loads Credentials/users.json and
Credentials/robots.json (the server cannot run without them).
"""

import json
from pathlib import Path


# ============================================================
# PATHS
# ============================================================

BASE_DIR = Path(__file__).resolve().parent.parent      # project root
CLIENT_DIR = BASE_DIR / "client"                       # browser app served by routes.py
CREDENTIALS_DIR = BASE_DIR / "Credentials"

USERS_FILE = CREDENTIALS_DIR / "users.json"
ROBOTS_FILE = CREDENTIALS_DIR / "robots.json"


# ============================================================
# USERS AND ROBOTS (loaded once at start-up)
# ============================================================

def _load_json(path: Path):
    with open(path, "r") as f:
        return json.load(f)


# username -> {"password_hash": "pbkdf2_sha256$...", ...}
USERS = _load_json(USERS_FILE)

# robot_id -> {"token": "...", "allowed_users": [...], ...}
ROBOTS = _load_json(ROBOTS_FILE)


# ============================================================
# WEBSOCKET SETTINGS
# ============================================================

# Phone orientation packets older than this (or from the future) are dropped.
MOBILE_MAX_PACKET_AGE_MS = 1000.0

# WebSocket close codes used by websocket.py
WS_CLOSE_BAD_REQUEST = 4001     # unknown role / unknown robot
WS_CLOSE_REPLACED = 4002        # an older connection was replaced by a new one
WS_CLOSE_UNAUTHORIZED = 4003    # bad token / session / not allowed
WS_CLOSE_UNAVAILABLE = 4004     # robot offline or already in use


# ============================================================
# SO-101 LEADER CALIBRATION (used by GET /api/calibration)
# ============================================================

# Searched in order; first existing file wins.
CALIBRATION_SEARCH_DIRS = [
    Path.home() / ".cache" / "huggingface" / "lerobot" / "calibration" / "teleoperators" / "so_leader",
    BASE_DIR,
]

# File names tried in each folder, after "<id>.json".
CALIBRATION_FALLBACK_FILENAMES = ["None.json", "my_leader_arm.json"]

# Verified fallback calibration for the SO-101 leader.
DEFAULT_LEADER_CALIBRATION = {
    "shoulder_pan": {"id": 1, "drive_mode": 0, "homing_offset": -106, "range_min": 816, "range_max": 3543},
    "shoulder_lift": {"id": 2, "drive_mode": 0, "homing_offset": 1016, "range_min": 833, "range_max": 3222},
    "elbow_flex": {"id": 3, "drive_mode": 0, "homing_offset": -5, "range_min": 861, "range_max": 3089},
    "wrist_flex": {"id": 4, "drive_mode": 0, "homing_offset": 143, "range_min": 781, "range_max": 3116},
    "wrist_roll": {"id": 5, "drive_mode": 0, "homing_offset": 7, "range_min": 0, "range_max": 4095},
    "gripper": {"id": 6, "drive_mode": 0, "homing_offset": -133, "range_min": 1535, "range_max": 2709},
}
