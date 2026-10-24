import json
import math
import os
import socket
import time
from pathlib import Path

from lerobot.robots.so_follower import SOFollower
from lerobot.robots.so_follower.config_so_follower import (
    SOFollowerRobotConfig,
)

HOST = "127.0.0.1"
PORT = 8765
FOLLOWER_PORT = "/dev/ttyACM0"
FOLLOWER_ID = "my_follower_arm"

# Keep False until the initial position checks are complete.
MOTION_ENABLED = (
    os.getenv("FOLLOWER_MOTION_ENABLED", "true").lower() == "true"
)

JOINTS = (
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
)

# Accept a command only if all six values are valid.
def validate_positions(positions):
    if not isinstance(positions, dict):
        return False

    for joint in JOINTS:
        value = positions.get(joint)

        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
        ):
            print(f"[FOLLOWER] Invalid value for {joint}")
            return False

        if joint == "gripper.pos":
            if not 0 <= value <= 100:
                print("[FOLLOWER] Gripper must be between 0 and 100%")
                return False
        elif not -180 <= value <= 180:
            print(f"[FOLLOWER] Invalid degree value for {joint}")
            return False

    return True


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))
sock.settimeout(0.1)

follower = None
last_sequence = -1
last_command_time = None
last_sent_time = 0.0

print(f"[FOLLOWER] IPC listening on {HOST}:{PORT}")
print(f"[FOLLOWER] Motion enabled: {MOTION_ENABLED}")

try:
    config = SOFollowerRobotConfig(
        port=FOLLOWER_PORT,
        id=FOLLOWER_ID,
        use_degrees=True,
        max_relative_target=10.0,
    )

    follower = SOFollower(config)

    print("[FOLLOWER] Connecting using saved LeRobot calibration...")
    follower.connect(calibrate=False)
    print("[FOLLOWER] LeRobot connection established.")

    observation = follower.get_observation()
    print("[FOLLOWER] Initial joint positions:")
    for joint in JOINTS:
        print(f"  {joint}: {observation.get(joint, 'unavailable')}")

    print("[FOLLOWER] Waiting for WebRTC joint commands.")

    while True:
        try:
            raw, address = sock.recvfrom(65535)
        except socket.timeout:
            if (
                last_command_time is not None
                and time.monotonic() - last_command_time > 0.5
            ):
                last_command_time = None
                print("[FOLLOWER] Command stream timed out.")
            continue

        try:
            data = json.loads(raw.decode("utf-8"))

            if data.get("type") != "joint_positions":
                continue

            sequence = data.get("sequence")
            positions = data.get("positions")

            if not validate_positions(positions):
                continue

            if isinstance(sequence, int):
                if sequence <= last_sequence:
                    continue
                last_sequence = sequence

            action = {
                joint: float(positions[joint])
                for joint in JOINTS
            }
            print("[DEBUG] Incoming positions:", action)
            last_command_time = time.monotonic()

            # Limit update rate while testing.
            now = time.monotonic()
            if now - last_sent_time < 0.05:
                continue
            last_sent_time = now

            if MOTION_ENABLED:
                follower.send_action(action)
                print(f"[FOLLOWER] Command sent, seq={sequence}")
            else:
                print(f"[DRY RUN] Valid command received, seq={sequence}")

        except (ValueError, TypeError, UnicodeDecodeError) as exc:
            print("[FOLLOWER] Rejected message:", exc)

except KeyboardInterrupt:
    print("\n[FOLLOWER] Stopping controller.")

finally:
    sock.close()

    if follower is not None and follower.is_connected:
        follower.disconnect()

    print("[FOLLOWER] Controller stopped.")