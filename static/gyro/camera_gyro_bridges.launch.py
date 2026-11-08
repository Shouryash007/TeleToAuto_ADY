#!/usr/bin/env python3
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    here = Path(__file__).resolve().parent
    camera_script = here / "gazebo_camera_bridge.py"
    gyro_script = here / "gyro_to_gazebo.py"

    for path in (camera_script, gyro_script):
        if not path.is_file():
            raise FileNotFoundError(f"Bridge script not found: {path}")

    camera_bridge = ExecuteProcess(
        cmd=[sys.executable, str(camera_script)],
        output="screen",
    )
    gyro_bridge = ExecuteProcess(
        cmd=[sys.executable, str(gyro_script)],
        output="screen",
    )

    return LaunchDescription([camera_bridge, gyro_bridge])
