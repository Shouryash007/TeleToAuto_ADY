"""
robot_manager.py - which robots are online and who is using them.

Owns the live connection tables and the rules for taking/releasing a robot.
It never parses messages: websocket.py decides what to do, this module keeps
the bookkeeping (and notifies the other side when a slot changes).
"""

import json
from typing import Optional, Tuple

from .auth import user_can_access_robot
from .config import ROBOTS, WS_CLOSE_REPLACED


# ============================================================
# RUNTIME STATE
# ============================================================

# robot_id -> WebSocket of the robot program
robot_connections = {}

# robot_id -> WebSocket / username of the operator's browser
browser_connections = {}
browser_users = {}

# robot_id -> WebSocket / username of the operator's phone (gyro)
mobile_connections = {}
mobile_users = {}


# ============================================================
# QUERIES
# ============================================================

def get_robot(robot_id) -> Optional[dict]:
    """Robot entry from robots.json, or None if it does not exist."""
    return ROBOTS.get(robot_id)


def is_online(robot_id) -> bool:
    return robot_id in robot_connections


def online_robot_ids() -> list:
    return sorted(robot_connections.keys())


def visible_robots(username: str) -> list:
    """Robots `username` may use, with their live online/busy status."""
    visible = []

    for robot_id, robot_info in ROBOTS.items():

        if not user_can_access_robot(username, robot_info):
            continue

        visible.append({
            "robot_id": robot_id,
            "online": robot_id in robot_connections,
            "busy": robot_id in browser_connections,
            "used_by": browser_users.get(robot_id),
            "mobile_busy": robot_id in mobile_connections,
            "mobile_used_by": mobile_users.get(robot_id),
        })

    return visible


async def send_to_robot(robot_id, text: str) -> bool:
    """Send text to a robot's WebSocket. Returns False if the robot is offline."""
    robot = robot_connections.get(robot_id)

    if not robot:
        return False

    await robot.send_text(text)
    return True


async def _notify(websocket, message: dict) -> None:
    """Best-effort JSON notification; a dead socket is ignored."""
    try:
        await websocket.send_text(json.dumps(message))
    except Exception:
        pass


# ============================================================
# ROBOT SLOT
# ============================================================

async def register_robot(robot_id, websocket) -> None:
    """Store a robot connection, closing any older connection for the same robot."""
    old_connection = robot_connections.get(robot_id)

    if old_connection:
        try:
            await old_connection.close(code=WS_CLOSE_REPLACED)
        except Exception:
            pass

    robot_connections[robot_id] = websocket

    print(f"[ROBOT] {robot_id} connected")


async def unregister_robot(robot_id, websocket) -> None:
    """Remove a robot (if this socket still owns the slot) and tell its viewers."""
    if not (robot_id and robot_connections.get(robot_id) is websocket):
        return

    del robot_connections[robot_id]

    print(f"[ROBOT] {robot_id} disconnected")

    browser = browser_connections.get(robot_id)
    if browser:
        await _notify(browser, {"type": "robot_disconnected"})

    mobile = mobile_connections.get(robot_id)
    if mobile:
        await _notify(mobile, {"type": "robot_disconnected"})


# ============================================================
# BROWSER SLOT (one operator per robot)
# ============================================================

async def claim_browser(robot_id, username, websocket) -> Tuple[bool, Optional[str]]:
    """
    Reserve the robot's browser slot for `username`.

    Returns (True, None) on success. Returns (False, current_user) if a
    different operator already holds it. The same user reconnecting (page
    reload) replaces their own stale socket.
    """
    existing = browser_connections.get(robot_id)

    if existing is not None and existing is not websocket:
        if browser_users.get(robot_id) != username:
            print(f"[ACCESS] {username} denied access to {robot_id} - already in use")
            return False, browser_users.get(robot_id)

        try:
            await existing.close(code=WS_CLOSE_REPLACED)
        except Exception:
            pass

    browser_connections[robot_id] = websocket
    browser_users[robot_id] = username

    print(f"[USER] {username} connected to {robot_id}")

    return True, None


async def release_browser(robot_id, username, websocket) -> None:
    """Free the browser slot (if this socket still owns it) and tell the robot."""
    if not (robot_id and browser_connections.get(robot_id) is websocket):
        return

    del browser_connections[robot_id]
    browser_users.pop(robot_id, None)

    print(f"[USER] {username} disconnected from {robot_id}")

    robot = robot_connections.get(robot_id)
    if robot:
        await _notify(robot, {"type": "browser_disconnected"})


# ============================================================
# MOBILE (PHONE) SLOT - independent from the browser slot
# ============================================================

async def claim_mobile(robot_id, username, websocket) -> Tuple[bool, Optional[str]]:
    """Same rules as claim_browser, for the phone gyro connection."""
    existing = mobile_connections.get(robot_id)

    if existing is not None and existing is not websocket:
        if mobile_users.get(robot_id) != username:
            return False, mobile_users.get(robot_id)

        try:
            await existing.close(code=WS_CLOSE_REPLACED)
        except Exception:
            pass

    mobile_connections[robot_id] = websocket
    mobile_users[robot_id] = username

    return True, None


async def release_mobile(robot_id, username, websocket) -> None:
    """Free the phone slot (if this socket still owns it) and tell the robot."""
    if not (robot_id and mobile_connections.get(robot_id) is websocket):
        return

    mobile_connections.pop(robot_id, None)
    mobile_users.pop(robot_id, None)

    robot = robot_connections.get(robot_id)
    if robot:
        await _notify(robot, {"type": "mobile_disconnected"})

    print(f"[MOBILE VIDEO] {username} disconnected")
