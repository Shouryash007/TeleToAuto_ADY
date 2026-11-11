#!/usr/bin/env bash
# ============================================================
# teleToAuto_ADY - dependency installer
#
#   ./install.sh                     install everything (same as: all)
#   ./install.sh robot               only the robot endpoint   (robot/requirements_robot.txt)
#   ./install.sh server              only the server           (server/requirements_server.txt)
#   ./install.sh client              nothing to pip-install; prints how to serve it
#   ./install.sh robot server        any combination
#
#   Extras (add to any command):
#     --hardware      real follower-arm packages (pyserial, Feetech SDK)       [into venv/]
#     --handtrack     Python MediaPipe teleop (separate venv-handtrack/)
#     --ros           ROS 2 Humble + Gazebo 11 apt packages           (needs sudo)
#     --so101-sim     ros_gz / Ignition packages for the SO-101 launch (needs sudo)
#     --cloudflared   install cloudflared from the .deb in this repo   (needs sudo)
#   Other:
#     --check         only verify what is installed
#     --dry-run       print the commands without running them
#     --venv DIR      virtual-environment folder (default: venv)
#     --python BIN    python to use (default: python3, must be 3.10+)
#     -h, --help
# ============================================================
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

PYTHON=python3; VENV=venv
ROBOT=0; SERVER=0; CLIENT=0
HW=0; HAND=0; ROS=0; SO101=0; CF=0; CHECK=0; DRY=0

while [ $# -gt 0 ]; do
  case "$1" in
    robot) ROBOT=1 ;;
    server) SERVER=1 ;;
    client) CLIENT=1 ;;
    all) ROBOT=1; SERVER=1; CLIENT=1 ;;
    --hardware) HW=1 ;;
    --handtrack) HAND=1 ;;
    --ros) ROS=1 ;;
    --so101-sim) SO101=1 ;;
    --cloudflared) CF=1 ;;
    --check) CHECK=1 ;;
    --dry-run) DRY=1 ;;
    --venv) shift; VENV="${1:?--venv needs a folder}" ;;
    --python) shift; PYTHON="${1:?--python needs a value}" ;;
    -h|--help) sed -n '2,26p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown argument: $1 (try --help)"; exit 2 ;;
  esac
  shift
done

# No component given -> install everything, unless only extras / --check were requested.
if [ $((ROBOT+SERVER+CLIENT)) -eq 0 ] && [ $((HW+HAND+ROS+SO101+CF+CHECK)) -eq 0 ]; then
  ROBOT=1; SERVER=1; CLIENT=1
fi

say()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m[warn]\033[0m %s\n' "$*"; }
run()  { echo "+ $*"; [ "$DRY" -eq 1 ] || "$@"; }

check_python() {
  command -v "$PYTHON" >/dev/null || { echo "$PYTHON not found: sudo apt install python3 python3-venv python3-pip"; exit 1; }
  "$PYTHON" -c 'import sys; sys.exit(0 if sys.version_info >= (3,10) else 1)' \
    || { echo "Python 3.10+ required (found $($PYTHON -V))."; exit 1; }
  "$PYTHON" -c 'import venv, ensurepip' 2>/dev/null \
    || { echo "python3-venv missing: sudo apt install python3-venv python3-pip"; exit 1; }
}

# Locate a requirements file: first existing candidate wins.
find_req() {
  for p in "$@"; do [ -f "$p" ] && { echo "$p"; return 0; }; done
  echo "Requirements file not found. Looked for: $*" >&2
  return 1
}

ensure_venv() {  # ensure_venv <dir>
  [ -d "$1" ] || run "$PYTHON" -m venv "$1"
  run "$1/bin/python" -m pip install --upgrade pip
}

check_env() {
  say "Checking installation"
  mod() { if "$1" -c "import $2" 2>/dev/null; then printf '  [ok]      %s\n' "$3"; else printf '  [missing] %s\n' "$3"; fi; }
  if [ -x "$VENV/bin/python" ]; then
    echo "  $VENV/:"
    for m in fastapi uvicorn pydantic; do mod "$VENV/bin/python" "$m" "server: $m"; done
    for m in aiortc av numpy cv2 websockets; do mod "$VENV/bin/python" "$m" "robot:  $m"; done
    mod "$VENV/bin/python" scservo_sdk "hardware: scservo_sdk (optional)"
  else echo "  [missing] $VENV/ (run ./install.sh)"; fi
  [ -x venv-handtrack/bin/python ] && mod venv-handtrack/bin/python mediapipe "venv-handtrack: mediapipe" || echo "  [skip]    venv-handtrack/ (only for --handtrack)"
  command -v ros2 >/dev/null && echo "  [ok]      ros2 (${ROS_DISTRO:-?})" || echo "  [skip]    ros2 not on PATH (source /opt/ros/humble/setup.bash)"
  command -v gazebo >/dev/null && echo "  [ok]      gazebo" || echo "  [skip]    gazebo not found (--ros)"
  command -v cloudflared >/dev/null && echo "  [ok]      cloudflared" || echo "  [skip]    cloudflared not found (--cloudflared)"
}

if [ "$CHECK" -eq 1 ]; then check_env; exit 0; fi

# ---------- Python packages: robot / server (one shared venv) ----------
if [ $((ROBOT+SERVER+HW)) -gt 0 ]; then
  check_python
  say "Python environment ($VENV/)"
  ensure_venv "$VENV"
  REQS=()
  if [ "$ROBOT" -eq 1 ]; then
    ROBOT_REQ=$(find_req robot/requirements_robot.txt robots/requirements_robot.txt requirements_robot.txt) || exit 1
    REQS+=( -r "$ROBOT_REQ" )
  fi
  if [ "$SERVER" -eq 1 ]; then
    SERVER_REQ=$(find_req server/requirements_server.txt requirements_server.txt) || exit 1
    REQS+=( -r "$SERVER_REQ" )
  fi
  [ ${#REQS[@]} -gt 0 ] && run "$VENV/bin/python" -m pip install "${REQS[@]}"
  if [ "$HW" -eq 1 ]; then
    say "Follower-arm hardware packages"
    run "$VENV/bin/python" -m pip install "pyserial>=3.5" "feetech-servo-sdk>=1.0"
    warn "Serial access may need:  sudo usermod -aG dialout \$USER   (then log out/in)"
  fi
  if [ ! -f .env ] && [ -f .env.example ]; then
    say "Creating .env from .env.example (set ROBOT_ID / ROBOT_TOKEN / SIGNALING_SERVER)"
    run cp .env.example .env
  fi
fi

# ---------- client ----------
if [ "$CLIENT" -eq 1 ]; then
  say "Client"
  echo "No Python packages needed - the browser loads MediaPipe from a CDN."
  echo "Serve it locally with:  python3 -m http.server 8080 --directory client"
  echo "then open http://localhost:8080 (camera access needs HTTPS or localhost)."
fi

# ---------- Python hand tracking (so101 simulation) ----------
if [ "$HAND" -eq 1 ]; then
  check_python
  say "Python hand-tracking environment (venv-handtrack/)"
  "$PYTHON" -c 'import sys; sys.exit(0 if sys.version_info < (3,12) else 1)' \
    || warn "mediapipe 0.10.14 needs Python 3.10/3.11 - re-run with --python python3.10"
  ensure_venv venv-handtrack
  # Separate venv on purpose: numpy<2 conflicts with the robot/server environment.
  run venv-handtrack/bin/python -m pip install "numpy<2" "opencv-contrib-python<4.12" "mediapipe==0.10.14"
fi

# ---------- ROS 2 / Gazebo (apt) ----------
if [ "$ROS" -eq 1 ] || [ "$SO101" -eq 1 ]; then
  DISTRO="${ROS_DISTRO:-humble}"
  if [ ! -d "/opt/ros/$DISTRO" ]; then
    warn "ROS 2 '$DISTRO' not found in /opt/ros. Install it first:"
    warn "  https://docs.ros.org/en/$DISTRO/Installation/Ubuntu-Install-Debs.html"
    [ "$DRY" -eq 1 ] || exit 1
  fi
  SUDO=""; command -v sudo >/dev/null && SUDO="sudo"
  APT=( "ros-$DISTRO-robot-state-publisher" "ros-$DISTRO-ros2-control" "ros-$DISTRO-ros2-controllers"
        "ros-$DISTRO-controller-manager" python3-opencv python3-numpy )
  [ "$ROS" -eq 1 ]   && APT+=( "ros-$DISTRO-gazebo-ros-pkgs" "ros-$DISTRO-gazebo-ros2-control" "ros-$DISTRO-cv-bridge" )
  [ "$SO101" -eq 1 ] && APT+=( "ros-$DISTRO-ros-gz-sim" "ros-$DISTRO-ros-gz-bridge" "ros-$DISTRO-ign-ros2-control" )
  say "ROS 2 / Gazebo apt packages"
  run $SUDO apt-get update
  run $SUDO apt-get install -y "${APT[@]}"
  warn "ROS scripts (static/arm_bridge.py, launch files) use SYSTEM Python:"
  warn "  source /opt/ros/$DISTRO/setup.bash   (do NOT activate $VENV for them)"
fi

# ---------- cloudflared ----------
if [ "$CF" -eq 1 ]; then
  say "cloudflared"
  if command -v cloudflared >/dev/null; then echo "already installed"
  elif [ -f cloudflared-linux-amd64.deb ] && [ "$(uname -m)" = "x86_64" ]; then run sudo dpkg -i cloudflared-linux-amd64.deb
  else warn "No matching .deb for $(uname -m); see https://developers.cloudflare.com/cloudflare-one/connections/connect-networks/downloads/"; fi
fi

[ "$DRY" -eq 1 ] || check_env
if [ $((ROBOT+SERVER)) -gt 0 ]; then
  echo; echo "Next:  source $VENV/bin/activate"
  [ "$SERVER" -eq 1 ] && echo "  uvicorn server.main:app --host 127.0.0.1 --port 8000"
  [ "$ROBOT" -eq 1 ]  && echo "  python robot/robot1.py"
fi
exit 0
