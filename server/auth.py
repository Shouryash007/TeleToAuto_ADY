"""
auth.py - login, password checking and session tokens.

Owns the `sessions` table (session_token -> username) and every "is this
caller allowed?" decision for operators and robots.
"""

import base64
import hashlib
import hmac
import secrets
from typing import Optional

from fastapi import HTTPException, Request
from pydantic import BaseModel

from .config import (
    ROBOTS,
    USERS,
    WS_CLOSE_BAD_REQUEST,
    WS_CLOSE_UNAUTHORIZED,
)


# session_token -> username
sessions = {}


# ============================================================
# PASSWORDS
# ============================================================

def verify_password(password: str, stored_hash: str) -> bool:
    """Check a password against 'pbkdf2_sha256$iterations$salt_b64$hash_b64'."""
    try:
        algorithm, iterations, salt_b64, hash_b64 = stored_hash.split("$")

        if algorithm != "pbkdf2_sha256":
            return False

        iterations = int(iterations)
        salt = base64.urlsafe_b64decode(salt_b64.encode())
        expected = base64.urlsafe_b64decode(hash_b64.encode())

        actual = hashlib.pbkdf2_hmac("sha256", password.encode(), salt, iterations)

        return hmac.compare_digest(actual, expected)

    except Exception:
        return False


# ============================================================
# OPERATOR LOGIN / SESSIONS
# ============================================================

class LoginRequest(BaseModel):
    username: str
    password: str


def login_user(username: str, password: str) -> Optional[str]:
    """Return a new session token if the credentials are valid, else None."""
    user = USERS.get(username)

    if not user:
        return None

    if not verify_password(password, user["password_hash"]):
        return None

    token = secrets.token_urlsafe(32)
    sessions[token] = username

    print(f"[AUTH] User logged in: {username}")

    return token


def username_for_token(token) -> Optional[str]:
    """Username that owns a session token, or None if the token is unknown."""
    return sessions.get(token)


def require_user(request: Request) -> str:
    """HTTP helper: username from the 'Authorization: Bearer <token>' header, else 401."""
    token = request.headers.get("Authorization", "").replace("Bearer ", "")

    username = sessions.get(token)

    if not username:
        raise HTTPException(status_code=401, detail="Invalid browser session")

    return username


def user_can_access_robot(username: str, robot: dict) -> bool:
    """True if `username` is in the robot's allowed_users list."""
    return username in robot.get("allowed_users", [])


# ============================================================
# ROBOT AUTHENTICATION
# ============================================================

def check_robot_credentials(robot_id, robot_token) -> Optional[int]:
    """
    Validate a robot's first WebSocket message.

    Returns None when the robot is authentic, otherwise the WebSocket close
    code to reject it with (4001 unknown robot, 4003 wrong token).
    """
    robot = ROBOTS.get(robot_id)

    if not robot:
        return WS_CLOSE_BAD_REQUEST

    if not hmac.compare_digest(robot_token or "", robot["token"]):
        print(f"[AUTH] Robot authentication failed: {robot_id}")
        return WS_CLOSE_UNAUTHORIZED

    return None
