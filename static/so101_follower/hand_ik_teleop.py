#!/usr/bin/env python3
"""
hand_ik_teleop.py  --  Mimic your hand with the SO-101 follower arm (ROS 2 + Gazebo).

Pipeline (one loop iteration per webcam frame):

  webcam -> MediaPipe Hands -> hand features (palm centre, hand size, tilt, pinch)
        -> target end-effector position in the robot base frame
        -> damped-least-squares inverse kinematics driven by the analytic
           geometric Jacobian  (dq = J^T (J J^T + lambda^2 I)^-1 * error)
        -> joint-speed limiting + smoothing
        -> Float64MultiArray position commands for the Gazebo controllers

Hand -> robot mapping (robot faces the same way you do; your view is mirrored):
  hand right / left ........ end effector moves right / left   (-y / +y)
  hand up / down ........... end effector moves up / down      (+z / -z)
  hand toward / away camera  end effector reaches out / back   (+x / -x)  [uses apparent hand size]
  hand tilt (roll in image)  wrist_roll
  thumb-index pinch ......... gripper close / open

Keys (click the video window first):
  E  engage / disengage sending commands (also re-centres on your hand)
  C  re-centre: your current hand pose becomes the arm's home pose
  Q / ESC  quit

Setup:
  pip install mediapipe opencv-python numpy      (mediapipe 0.10.x)
  source /opt/ros/<distro>/setup.bash
  python3 hand_ik_teleop.py --ros-args -p urdf_file:=/path/to/so101_new_calib.urdf
"""
import math
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import cv2

try:
    import mediapipe as mp
    _HANDS = mp.solutions.hands
    _DRAW = mp.solutions.drawing_utils
except (ImportError, AttributeError) as exc:  # pragma: no cover
    sys.exit(f"MediaPipe 'solutions' API not available ({exc}). Try: pip install 'mediapipe==0.10.14'")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# ======================================================================
# Kinematics: forward kinematics, analytic Jacobian, damped-least-squares IK
# ======================================================================
# Embedded copy of the SO-101 follower chain (so101_new_calib.urdf).
# (name, type, xyz, rpy, (lower, upper))
DEFAULT_CHAIN = [
    ("shoulder_pan", "revolute", (0.0388353, -8.97657e-09, 0.0624), (3.14159, 4.18253e-17, -3.14159), (-1.91986, 1.91986)),
    ("shoulder_lift", "revolute", (-0.0303992, -0.0182778, -0.0542), (-1.5708, -1.5708, 0.0), (-1.74533, 1.74533)),
    ("elbow_flex", "revolute", (-0.11257, -0.028, 1.73763e-16), (-3.63608e-16, 8.74301e-16, 1.5708), (-1.69, 1.69)),
    ("wrist_flex", "revolute", (-0.1349, 0.0052, 3.62355e-17), (4.02456e-15, 8.67362e-16, -1.5708), (-1.65806, 1.65806)),
    ("wrist_roll", "revolute", (5.55112e-17, -0.0611, 0.0181), (1.5708, 0.0486795, 3.14159), (-2.74385, 2.84121)),
    ("gripper_frame_joint", "fixed", (-0.0079, -0.000218121, -0.0981274), (0.0, 3.14159, 0.0), None),
]


def rpy_to_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def load_chain_from_urdf(path, ee_link="gripper_frame_link"):
    """Walk from ee_link up to the root and return the serial chain."""
    root = ET.parse(path).getroot()
    by_child = {}
    for j in root.findall("joint"):
        origin = j.find("origin")
        xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
        rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
        limit = j.find("limit")
        lim = None
        if limit is not None and "lower" in limit.attrib:
            lim = (float(limit.get("lower")), float(limit.get("upper")))
        by_child[j.find("child").get("link")] = (
            j.get("name"), j.get("type"), tuple(xyz), tuple(rpy), lim, j.find("parent").get("link"))
    chain, link = [], ee_link
    while link in by_child:
        name, jtype, xyz, rpy, lim, parent = by_child[link]
        chain.append((name, jtype, xyz, rpy, lim))
        link = parent
    chain.reverse()
    if not any(c[1] == "revolute" for c in chain):
        raise ValueError(f"No revolute joints found from {ee_link} to the root in {path}")
    return chain


class SO101Kinematics:
    """Serial-chain FK, analytic geometric Jacobian and damped-least-squares IK."""

    def __init__(self, chain=None):
        self.chain = chain or DEFAULT_CHAIN
        self.active = [c for c in self.chain if c[1] == "revolute"]
        self.names = [c[0] for c in self.active]
        self.lower = np.array([c[4][0] for c in self.active])
        self.upper = np.array([c[4][1] for c in self.active])
        self._A = [self._fixed_transform(c[2], c[3]) for c in self.chain]
        # Signs relating each flex joint to the arm's pitch angle (parallel axes).
        _, _, axes, origins = self.fk_full(np.zeros(len(self.active)))
        n = axes[1]
        self.pitch_signs = np.array([round(float(np.dot(n, a))) for a in axes], dtype=float)
        self.pitch_signs[0] = 0.0   # pan
        self.pitch_signs[-1] = 0.0  # roll

    @staticmethod
    def _fixed_transform(xyz, rpy):
        A = np.eye(4)
        A[:3, :3] = rpy_to_matrix(*rpy)
        A[:3, 3] = xyz
        return A

    def fk_full(self, q):
        """Return (p_ee, R_ee, axes[], origins[]) in the base frame."""
        T = np.eye(4)
        axes, origins, k = [], [], 0
        for c, A in zip(self.chain, self._A):
            T = T @ A
            if c[1] == "revolute":
                axes.append(T[:3, 2].copy())
                origins.append(T[:3, 3].copy())
                ct, st = math.cos(q[k]), math.sin(q[k])
                Rz = np.array([[ct, -st, 0, 0], [st, ct, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
                T = T @ Rz
                k += 1
        return T[:3, 3].copy(), T[:3, :3].copy(), axes, origins

    def fk(self, q):
        p, R, _, _ = self.fk_full(q)
        return p, R

    def jacobian(self, q):
        """3 x N position Jacobian: column i = axis_i x (p_ee - origin_i)."""
        p, _, axes, origins = self.fk_full(q)
        return np.stack([np.cross(a, p - o) for a, o in zip(axes, origins)], axis=1)

    def pitch(self, q):
        return float(np.dot(self.pitch_signs, q))

    def solve(self, q0, target, ik_joints, target_pitch=None, pitch_weight=0.3,
              damping=0.05, iters=30, tol=5e-4, max_step=0.25, null_gain=0.05, q_neutral=None):
        """Position IK (optionally with a pitch row) using the Jacobian.

        Only the joints in `ik_joints` (indices) move; the others are held at q0.
        Returns (q, position_error_m, converged).
        """
        q = np.array(q0, dtype=float)
        idx = list(ik_joints)
        n = len(idx)
        err_norm = 1e9
        for _ in range(iters):
            p, _ = self.fk(q)
            e = np.asarray(target, dtype=float) - p
            err_norm = float(np.linalg.norm(e))
            J = self.jacobian(q)[:, idx]
            rows, err = [J], [e]
            pitch_err = 0.0
            if target_pitch is not None and pitch_weight > 0.0:
                pitch_err = target_pitch - self.pitch(q)
                rows.append(pitch_weight * self.pitch_signs[idx][None, :])
                err.append([pitch_weight * pitch_err])
            Jt = np.vstack(rows)
            er = np.concatenate([np.atleast_1d(x) for x in err])
            if err_norm < tol and abs(pitch_err) < 0.02:
                return q, err_norm, True
            JJt = Jt @ Jt.T + (damping ** 2) * np.eye(Jt.shape[0])
            J_pinv = Jt.T @ np.linalg.inv(JJt)
            dq = J_pinv @ er
            if q_neutral is not None and Jt.shape[0] < n:
                N = np.eye(n) - J_pinv @ Jt
                dq = dq + N @ (null_gain * (np.asarray(q_neutral)[idx] - q[idx]))
            m = float(np.max(np.abs(dq)))
            if m > max_step:
                dq *= max_step / m
            q[idx] = np.clip(q[idx] + dq, self.lower[idx], self.upper[idx])
        p, _ = self.fk(q)
        err_norm = float(np.linalg.norm(np.asarray(target) - p))
        return q, err_norm, err_norm < 2e-3


# ======================================================================
# Hand tracking
# ======================================================================
class HandFeatures:
    __slots__ = ("cx", "cy", "size", "roll", "pinch", "landmarks")

    def __init__(self, cx, cy, size, roll, pinch, landmarks):
        self.cx, self.cy, self.size, self.roll, self.pinch, self.landmarks = cx, cy, size, roll, pinch, landmarks


class HandTracker:
    PALM_IDS = (0, 5, 9, 13, 17)

    def __init__(self, model_complexity=0, min_conf=0.6):
        self.hands = _HANDS.Hands(
            static_image_mode=False, max_num_hands=1,
            model_complexity=model_complexity,
            min_detection_confidence=min_conf, min_tracking_confidence=min_conf)

    def process(self, frame_bgr):
        h, w = frame_bgr.shape[:2]
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        result = self.hands.process(rgb)
        if not result.multi_hand_landmarks:
            return None
        lm = result.multi_hand_landmarks[0]
        pts = np.array([[p.x * w, p.y * h] for p in lm.landmark])  # pixels
        palm = pts[list(self.PALM_IDS)].mean(axis=0)
        hand_len = float(np.linalg.norm(pts[0] - pts[9]))
        if hand_len < 1e-3:
            return None
        v = pts[5] - pts[17]                                   # pinky MCP -> index MCP
        roll = math.atan2(v[1], v[0])
        pinch = float(np.linalg.norm(pts[4] - pts[8])) / hand_len
        return HandFeatures(palm[0] / w, palm[1] / h, hand_len / h, roll, pinch, lm)

    @staticmethod
    def draw(frame, feats):
        _DRAW.draw_landmarks(frame, feats.landmarks, _HANDS.HAND_CONNECTIONS)

    def close(self):
        self.hands.close()


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ======================================================================
# ROS node
# ======================================================================
class HandIKTeleop(Node):
    def __init__(self):
        super().__init__("hand_ik_teleop")
        P = self.declare_parameter
        P("urdf_file", "")                      # empty -> embedded SO-101 chain
        P("camera_index", 0)
        P("frame_width", 640)
        P("frame_height", 480)
        P("show_window", True)
        P("auto_engage", False)
        P("arm_topic", "/arm_position_controller/commands")
        P("gripper_topic", "/gripper_position_controller/commands")
        P("joint_states_topic", "/joint_states")
        # Home end-effector position (base frame, metres) and tool pitch (rad, + = tilted down)
        P("home_x", 0.24); P("home_y", 0.0); P("home_z", 0.22); P("home_pitch", 0.35)
        # Half-size of the workspace box around home (metres)
        P("range_x", 0.06); P("range_y", 0.12); P("range_z", 0.06)
        # How far your hand must travel for the full range
        P("hand_span_x", 0.22)   # fraction of image width
        P("hand_span_y", 0.20)   # fraction of image height
        P("depth_span", 0.35)    # relative change of apparent hand size
        P("depth_sign", 1.0)     # -1 flips toward/away
        P("lateral_sign", 1.0)   # -1 flips left/right
        P("roll_gain", 1.0)      # wrist_roll rad per rad of hand tilt (negative flips)
        P("pitch_weight", 0.3)   # 0 = position only (null-space posture), >0 holds tool pitch
        P("ik_damping", 0.05)
        P("max_joint_speed", 2.5)   # rad/s
        P("smoothing", 0.35)        # EMA factor on hand features (0..1, higher = faster)
        P("lost_hold_s", 0.5)
        P("gripper_open", 1.2); P("gripper_closed", 0.0)
        P("pinch_closed", 0.25); P("pinch_open", 0.95)
        P("model_complexity", 0)

        g = lambda n: self.get_parameter(n).value
        self.g = g

        urdf = g("urdf_file")
        chain = load_chain_from_urdf(urdf) if urdf else None
        self.kin = SO101Kinematics(chain)
        if len(self.kin.names) < 5:
            raise RuntimeError(f"Expected a 5-joint arm chain, got {self.kin.names}")
        self.ik_joints = [0, 1, 2, 3]       # pan, lift, elbow, wrist_flex solve the position
        self.roll_idx = 4                   # wrist_roll follows hand tilt directly

        self.home_target = np.array([g("home_x"), g("home_y"), g("home_z")])
        seed = np.array([0.0, -1.0, 0.6, 0.8, 0.0])
        self.target_pitch = g("home_pitch")
        q_home, err, ok = self.kin.solve(seed, self.home_target, self.ik_joints,
                                         target_pitch=self.target_pitch, pitch_weight=1.0,
                                         iters=400, tol=1e-4)
        if not ok:
            self.get_logger().warn(f"Home pose only reached to {err * 1000:.1f} mm; adjust home_x/y/z/pitch")
        self.q_home = q_home
        self.q_cmd = q_home.copy()
        self.pitch_ref = self.kin.pitch(q_home)
        self.get_logger().info("Home joints [rad]: " + ", ".join(f"{n}={v:+.2f}" for n, v in zip(self.kin.names, q_home)))

        self.arm_pub = self.create_publisher(Float64MultiArray, g("arm_topic"), 10)
        self.grip_pub = self.create_publisher(Float64MultiArray, g("gripper_topic"), 10)
        self.joint_state = {}
        self.create_subscription(JointState, g("joint_states_topic"), self._on_joint_state, 10)

        # state
        self.engaged = bool(g("auto_engage"))
        self.refs = None          # (cx0, cy0, size0, roll0)
        self.smoothed = None      # smoothed (cx, cy, size, roll, pinch)
        self.last_seen = 0.0
        self.target = self.home_target.copy()
        self.roll_cmd = 0.0
        self.grip_cmd = g("gripper_open")
        self.last_err = 0.0
        self.last_log = 0.0
        self.last_t = time.monotonic()

    # ------------------------------------------------------------------
    def _on_joint_state(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joint_state[n] = p

    def _current_joints(self):
        if all(n in self.joint_state for n in self.kin.names):
            return np.array([self.joint_state[n] for n in self.kin.names])
        return None

    # ------------------------------------------------------------------
    def recenter(self, feats):
        self.refs = (feats.cx, feats.cy, feats.size, feats.roll)
        self.smoothed = np.array([feats.cx, feats.cy, feats.size, 0.0, feats.pinch])
        self.target = self.home_target.copy()

    def set_engaged(self, value, feats):
        self.engaged = value
        if value:
            q = self._current_joints()
            self.q_cmd = q if q is not None else self.q_home.copy()
            if feats is not None:
                self.recenter(feats)
        self.get_logger().info("ENGAGED" if value else "DISENGAGED")

    # ------------------------------------------------------------------
    def update(self, feats, now):
        """Hand features -> joint command (returns True if a command was produced)."""
        g = self.g
        dt = clamp(now - self.last_t, 1e-3, 0.1)
        self.last_t = now

        if feats is not None:
            self.last_seen = now
            if self.refs is None:
                self.recenter(feats)
            roll_rel = wrap_angle(feats.roll - self.refs[3])
            raw = np.array([feats.cx, feats.cy, feats.size, roll_rel, feats.pinch])
            a = g("smoothing")
            self.smoothed = raw if self.smoothed is None else (1 - a) * self.smoothed + a * raw
        elif self.smoothed is None or now - self.last_seen > g("lost_hold_s"):
            return False  # hold last command

        if feats is not None and self.refs is not None:
            cx, cy, size, roll_rel, pinch = self.smoothed
            cx0, cy0, size0, _ = self.refs
            ux = clamp((cx - cx0) / g("hand_span_x"), -1.0, 1.0) * g("lateral_sign")
            uz = clamp(-(cy - cy0) / g("hand_span_y"), -1.0, 1.0)
            ud = clamp((size / size0 - 1.0) / g("depth_span"), -1.0, 1.0) * g("depth_sign")
            self.target = self.home_target + np.array([
                ud * g("range_x"),      # toward camera -> reach forward
                -ux * g("range_y"),     # hand right -> robot right (-y)
                uz * g("range_z")])
            self.roll_cmd = clamp(g("roll_gain") * roll_rel, self.kin.lower[self.roll_idx], self.kin.upper[self.roll_idx])
            t = clamp((pinch - g("pinch_closed")) / max(1e-6, g("pinch_open") - g("pinch_closed")), 0.0, 1.0)
            self.grip_cmd = g("gripper_closed") + t * (g("gripper_open") - g("gripper_closed"))

        # ---- inverse kinematics (Jacobian / DLS), warm-started from the last command
        q_seed = self.q_cmd.copy()
        q_seed[self.roll_idx] = self.roll_cmd
        q_ik, err, _ = self.kin.solve(
            q_seed, self.target, self.ik_joints,
            target_pitch=self.pitch_ref, pitch_weight=g("pitch_weight"),
            damping=g("ik_damping"), iters=25, q_neutral=self.q_home)
        self.last_err = err

        # ---- joint speed limit
        step = g("max_joint_speed") * dt
        self.q_cmd = np.clip(self.q_cmd + np.clip(q_ik - self.q_cmd, -step, step),
                             self.kin.lower, self.kin.upper)
        return True

    def publish(self):
        arm = Float64MultiArray(); arm.data = [float(v) for v in self.q_cmd]
        grip = Float64MultiArray(); grip.data = [float(self.grip_cmd)]
        self.arm_pub.publish(arm)
        self.grip_pub.publish(grip)

    # ------------------------------------------------------------------
    def overlay(self, frame, feats, now):
        p, _ = self.kin.fk(self.q_cmd)
        state = "ENGAGED" if self.engaged else "PAUSED - press E"
        color = (0, 200, 0) if self.engaged else (0, 165, 255)
        lines = [
            (state, color),
            ("hand: " + ("tracked" if feats is not None else "LOST"), (255, 255, 255) if feats else (0, 0, 255)),
            (f"target  x{self.target[0]:+.3f} y{self.target[1]:+.3f} z{self.target[2]:+.3f} m", (255, 255, 255)),
            (f"ee      x{p[0]:+.3f} y{p[1]:+.3f} z{p[2]:+.3f} m  err {self.last_err * 1000:.1f} mm", (255, 255, 255)),
            ("q " + " ".join(f"{v:+.2f}" for v in self.q_cmd) + f" | grip {self.grip_cmd:.2f}", (200, 200, 200)),
            ("E engage  C recentre  Q quit", (180, 180, 180)),
        ]
        for i, (txt, col) in enumerate(lines):
            cv2.putText(frame, txt, (10, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1, cv2.LINE_AA)
        if self.refs is not None:  # draw the neutral spot
            h, w = frame.shape[:2]
            cv2.drawMarker(frame, (int(self.refs[0] * w), int(self.refs[1] * h)), (255, 255, 0), cv2.MARKER_CROSS, 18, 1)

    # ------------------------------------------------------------------
    def run(self):
        g = self.g
        cap = cv2.VideoCapture(int(g("camera_index")))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(g("frame_width")))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(g("frame_height")))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # always work on the freshest frame (less lag)
        if not cap.isOpened():
            self.get_logger().error("Could not open the webcam (try -p camera_index:=1)")
            return
        tracker = HandTracker(int(g("model_complexity")))
        show = bool(g("show_window"))
        self.get_logger().info("Ready. Show your hand, then press E in the video window to engage.")
        try:
            while rclpy.ok():
                ok, frame = cap.read()
                if not ok:
                    self.get_logger().warn("Camera frame grab failed")
                    time.sleep(0.01)
                    continue
                frame = cv2.flip(frame, 1)               # mirror view
                feats = tracker.process(frame)
                now = time.monotonic()
                rclpy.spin_once(self, timeout_sec=0.0)

                if feats is not None:
                    tracker.draw(frame, feats)
                produced = self.update(feats, now)
                if self.engaged and produced:
                    self.publish()

                if now - self.last_log > 2.0 and self.engaged:
                    self.last_log = now
                    self.get_logger().info(f"IK position error {self.last_err * 1000:.1f} mm")

                if show:
                    self.overlay(frame, feats, now)
                    cv2.imshow("SO-101 hand teleop", frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
                    if key == ord("e"):
                        self.set_engaged(not self.engaged, feats)
                    elif key == ord("c") and feats is not None:
                        self.recenter(feats)
                        self.get_logger().info("Re-centred on current hand pose")
        finally:
            tracker.close()
            cap.release()
            cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = HandIKTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
