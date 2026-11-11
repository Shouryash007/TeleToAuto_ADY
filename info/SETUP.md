# Setup

Everything is installed with **one script** (`install.sh`) from **one set of requirements files**:

| File | What it is for |
|---|---|
| `install.sh` | The only installer. Pick what to install by name (below). |
| `requirements.txt` | Whole project in one environment: it simply includes the two files below. |
| `requirements_robot.txt` | Robot endpoint (`robot1.py`, `robot2.py`). |
| `requirements_server.txt` | Signaling + login server (`server.py`). |

Requires Ubuntu 22.04 (or similar) with Python 3.10+ and `python3-venv`:
`sudo apt install python3 python3-venv python3-pip`

## What each part installs

### Robot  (`requirements_robot.txt`)
Runs on the machine that has the camera / simulation.
- `aiortc` - WebRTC connection and DataChannels to the operator.
- `av` - video frame objects used by aiortc.
- `numpy`, `opencv-python` - camera capture and frame handling.
- `websockets` - connection to the signaling server.

### Server  (`requirements_server.txt`)
Runs where operators log in and robots register.
- `fastapi` - login API and WebSocket signaling.
- `uvicorn` - the web server that runs it.
- `pydantic` - request validation.
- `websockets` - WebSocket support for uvicorn.

### Client  (no requirements file)
The browser app in `client/`. Nothing to pip-install: MediaPipe Hands loads from a CDN, and you
need a modern browser, a webcam, and HTTPS or `localhost` (browsers block camera access otherwise).

## How to install

```bash
./install.sh                  # everything (robot + server + client notes) into ./venv
./install.sh robot            # ONLY the robot endpoint
./install.sh server           # ONLY the server
./install.sh robot server     # any combination
./install.sh client           # nothing to install; prints how to serve the client
./install.sh --check          # show what is installed
./install.sh --dry-run robot  # preview commands without running them
```

Prefer plain pip? These are equivalent:

```bash
python3 -m venv venv && source venv/bin/activate
pip install -r requirements_robot.txt     # only the robot
pip install -r requirements_server.txt    # only the server
pip install -r requirements.txt           # everything
```

### Example: "I only want the robot"
```bash
./install.sh robot
source venv/bin/activate
python robot/robot1.py
```

### Example: server only (e.g. on a cloud machine)
```bash
./install.sh server
source venv/bin/activate
uvicorn server.server:app --host 127.0.0.1 --port 8000
```

## Optional extras (add to any command)

| Option | Installs | When you need it |
|---|---|---|
| `--hardware` | `pyserial`, `feetech-servo-sdk` | Real SO-101 follower arm (`Others/follower_driver.py`). `Others/follower_controller.py` also needs `lerobot` (large, PyTorch): `pip install lerobot`. |
| `--handtrack` | separate `venv-handtrack/` with `numpy<2`, `opencv-contrib-python<4.12`, `mediapipe==0.10.14` | Python hand teleop `static/so101_follower/dual_hand_ik_teleop.py`. Separate because `numpy<2` conflicts with the core environment. Use Python 3.10/3.11. |
| `--ros` | ROS 2 Humble + Gazebo 11 apt packages (sudo) | Humanoid simulation (`static/gazebo_humanoid.launch.py`, `arm_bridge.py`, `gyro/`). ROS 2 Humble itself must be installed first. |
| `--so101-sim` | `ros_gz` / Ignition apt packages (sudo) | `static/so101_follower/so101_dual_gazebo.launch.py`. |
| `--cloudflared` | `cloudflared` from the `.deb` in this repo (sudo) | Exposing the server publicly. |

Other options: `--venv DIR` (environment folder, default `venv`), `--python BIN` (e.g. `python3.10`).

## Important notes
- **ROS scripts use system Python.** Run `arm_bridge.py`, `gyro/*.py` and the launch files after
  `source /opt/ros/humble/setup.bash`, and do **not** activate `venv/` for them.
- `.env` is created from `.env.example` the first time you install robot or server. Set
  `ROBOT_ID`, `ROBOT_TOKEN` and `SIGNALING_SERVER` in it.
- Your previous run notes and test checklist (formerly inside `requirements.txt`) are in `NOTES.md`.
