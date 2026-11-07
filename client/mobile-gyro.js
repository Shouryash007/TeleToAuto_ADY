// The gyro is given a limitation based tuning
// for left and right (-80_+80) and top to bottom(-60_+60)
(() => {
  "use strict";
  const $ = (id) => document.getElementById(id);

  let token = null, socket = null, listening = false, active = false, sequence = 0;
  let referenceQ = null;
  let pitchAxisWorld = null;
  let pendingCalibration = false;
  let yawContinuous = 0;
  let pitchContinuous = 0;
  let lastSend = 0;

  const SEND_INTERVAL_MS = 33;
  const YAW_LIMIT_DEG = 80;
  const PITCH_LIMIT_DEG = 60;
  const toRad = (d) => d * Math.PI / 180;
  const toDeg = (r) => r * 180 / Math.PI;

  function status(message, activeState = false) {
    $("status").textContent = message;
    $("status").classList.toggle("active", activeState);
  }

  function wsUrl() {
    const scheme = location.protocol === "https:" ? "wss:" : "ws:";
    return `${scheme}//${location.host}/ws`;
  }

  function qMultiply(a, b) {
    return {
      w: a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
      x: a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
      y: a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
      z: a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
  }

  function qConjugate(q) {
    return { w:q.w, x:-q.x, y:-q.y, z:-q.z };
  }

  function qNormalize(q) {
    const n = Math.hypot(q.w, q.x, q.y, q.z);
    if (!Number.isFinite(n) || n < 1e-10) return null;
    return { w:q.w/n, x:q.x/n, y:q.y/n, z:q.z/n };
  }

  function qAxis(axis, angle) {
    const s = Math.sin(angle/2), c = Math.cos(angle/2);
    return {
      w:c,
      x:axis === "x" ? s : 0,
      y:axis === "y" ? s : 0,
      z:axis === "z" ? s : 0
    };
  }

  function qRotate(q, v) {
    const p = { w:0, x:v.x, y:v.y, z:v.z };
    const r = qMultiply(qMultiply(q, p), qConjugate(q));
    return { x:r.x, y:r.y, z:r.z };
  }

  // DeviceOrientation uses intrinsic Z-X'-Y'' rotations:
  // alpha about Z, beta about the rotated X, gamma about the rotated Y.
  function readDeviceQuaternion(event) {
    if (event.alpha == null || event.beta == null || event.gamma == null) return null;
    const a = Number(event.alpha), b = Number(event.beta), g = Number(event.gamma);
    if (![a,b,g].every(Number.isFinite)) return null;

    return qNormalize(qMultiply(
      qMultiply(qAxis("z", toRad(a)), qAxis("x", toRad(b))),
      qAxis("y", toRad(g))
    ));
  }

  function normalizeVector(v) {
    const n = Math.hypot(v.x, v.y, v.z);
    if (!Number.isFinite(n) || n < 1e-8) return null;
    return { x:v.x/n, y:v.y/n, z:v.z/n };
  }

  // Extract only the component of a quaternion that rotates around axis.
  function twistAngle(q, axis) {
    const projection = q.x*axis.x + q.y*axis.y + q.z*axis.z;
    const twist = qNormalize({
      w:q.w,
      x:axis.x*projection,
      y:axis.y*projection,
      z:axis.z*projection
    });
    if (!twist) return 0;
    let angle = 2 * Math.atan2(
      twist.x*axis.x + twist.y*axis.y + twist.z*axis.z,
      twist.w
    );
    // Keep the single-sample result in the principal range.
    if (angle > Math.PI) angle -= 2*Math.PI;
    if (angle < -Math.PI) angle += 2*Math.PI;
    return angle;
  }

  function wrapPi(a) {
    return ((a + Math.PI) % (2*Math.PI) + 2*Math.PI) % (2*Math.PI) - Math.PI;
  }

  function unwrap(previous, principal) {
    return previous + wrapPi(principal - previous);
  }

  function calibrateFrom(q) {
    referenceQ = q;

    // For the user's fixed landscape hold, the phone's physical top axis
    // (local +Y) points to the user's left. Project it onto the horizontal
    // plane: this is the axis for nodding the head up/down.
    const topAxis = qRotate(q, { x:0, y:1, z:0 });
    pitchAxisWorld = normalizeVector({ x:topAxis.x, y:topAxis.y, z:0 });

    if (!pitchAxisWorld) {
      status("Calibration failed: hold phone approximately upright", false);
      referenceQ = null;
      return false;
    }

    yawContinuous = 0;
    pitchContinuous = 0;
    $("yaw").textContent = "0.0°";
    $("pitch").textContent = "0.0°";
    status("Calibrated — turn and nod your head to control", true);
    return true;
  }

  function statusConnected() {
    return socket?.readyState === WebSocket.OPEN;
  }

  function onOrientation(event) {
    const currentQ = readDeviceQuaternion(event);
    if (!currentQ) {
      status("Waiting for complete phone orientation readings");
      return;
    }

    if (pendingCalibration || !referenceQ) {
      pendingCalibration = false;
      calibrateFrom(currentQ);
      return;
    }

    // World-frame change from the calibrated phone orientation.
    let delta = qNormalize(qMultiply(currentQ, qConjugate(referenceQ)));
    if (!delta) return;

    // q and -q represent the same orientation. Select shortest representation.
    if (delta.w < 0) {
      delta = { w:-delta.w, x:-delta.x, y:-delta.y, z:-delta.z };
    }

    // First extract world vertical twist (head turn / yaw).
    const worldUp = { x:0, y:0, z:1 };
    const yawPrincipal = twistAngle(delta, worldUp);
    const yawQ = qNormalize({
      w:Math.cos(yawPrincipal/2),
      x:0, y:0, z:Math.sin(yawPrincipal/2)
    });

    // Remove yaw before extracting nod/pitch around the phone's calibrated
    // horizontal top-axis. This avoids assigning yaw motion to the pitch joint.
    const residual = qNormalize(qMultiply(qConjugate(yawQ), delta));
    if (!residual) return;
    const pitchPrincipal = twistAngle(residual, pitchAxisWorld);

    yawContinuous = unwrap(yawContinuous, yawPrincipal);
    pitchContinuous = unwrap(pitchContinuous, pitchPrincipal);

    const yaw = Math.max(-YAW_LIMIT_DEG, Math.min(YAW_LIMIT_DEG, toDeg(yawContinuous)));
    const pitch = Math.max(-PITCH_LIMIT_DEG, Math.min(PITCH_LIMIT_DEG, toDeg(pitchContinuous)));

    $("yaw").textContent = `${yaw.toFixed(1)}°`;
    $("pitch").textContent = `${pitch.toFixed(1)}°`;

    const now = Date.now();
    if (active && statusConnected() && now - lastSend >= SEND_INTERVAL_MS) {
      lastSend = now;
      socket.send(JSON.stringify({
        type: "head_orientation",
        sequence: ++sequence,
        timestamp_ms: now,
        units: "rad",
        orientation: {
          yaw: toRad(yaw),
          pitch: toRad(pitch),
          roll: 0
        },
        source: "phone_landscape_world_twist"
      }));
    }
  }

  async function login(event) {
    event.preventDefault();
    $("loginError").textContent = "";
    try {
      const response = await fetch("/api/login", {
        method:"POST",
        headers:{"Content-Type":"application/json"},
        body:JSON.stringify({
          username:$("username").value.trim(),
          password:$("password").value
        })
      });
      const data = await response.json();
      if (!response.ok || !data.success) throw new Error(data.error || "Login failed");
      token = data.token;
      sessionStorage.setItem("mobile_session_token", token);
      sessionStorage.setItem("mobile_username", data.username);
      $("loginCard").classList.add("hidden");
      $("sensorCard").classList.remove("hidden");
      await connectSocket();
    } catch (error) {
      $("loginError").textContent = error.message || "Unable to log in";
      status("Login failed");
    }
  }

  function connectSocket() {
    return new Promise((resolve, reject) => {
      socket = new WebSocket(wsUrl());
      const timeout = setTimeout(() => reject(new Error("Signaling server timed out")), 8000);
      socket.onopen = () => socket.send(JSON.stringify({
        role:"mobile_gyro", token, robot_id:$("robotId").value.trim()
      }));
      socket.onmessage = (event) => {
        let data;
        try { data = JSON.parse(event.data); } catch { return; }
        if (data.type === "mobile_gyro_connected") {
          clearTimeout(timeout);
          status(`Connected to ${data.robot_id}. Sensor is off.`, true);
          resolve();
        } else if (data.type === "error") {
          clearTimeout(timeout);
          status(data.message);
          reject(new Error(data.message));
        }
      };
      socket.onerror = () => {
        clearTimeout(timeout);
        status("WebSocket connection error");
        reject(new Error("Could not connect to signaling server"));
      };
      socket.onclose = () => {
        if (active) stop();
        status("Disconnected from server");
      };
    });
  }

  async function start() {
    if (!window.isSecureContext) {
      alert("Phone sensors require HTTPS. Open this page using your HTTPS tunnel URL.");
      return;
    }
    if (!("DeviceOrientationEvent" in window)) {
      alert("This browser does not expose DeviceOrientationEvent.");
      return;
    }
    try {
      if (typeof DeviceOrientationEvent.requestPermission === "function") {
        const permission = await DeviceOrientationEvent.requestPermission();
        if (permission !== "granted") throw new Error("Motion permission was not granted");
      }
      if (!listening) {
        window.addEventListener("deviceorientation", onOrientation, true);
        listening = true;
      }
      referenceQ = null;
      pitchAxisWorld = null;
      pendingCalibration = true;
      active = true;
      $("startButton").disabled = true;
      $("calibrateButton").disabled = false;
      $("stopButton").disabled = false;
      status("Hold phone still in landscape — calibrating neutral position", true);
    } catch (error) {
      status("Sensor could not start");
      alert(error.message || String(error));
    }
  }

  function calibrate() {
    referenceQ = null;
    pitchAxisWorld = null;
    pendingCalibration = true;
    yawContinuous = 0;
    pitchContinuous = 0;
    $("yaw").textContent = "0.0°";
    $("pitch").textContent = "0.0°";
    status("Hold phone still — capturing neutral position", true);
  }

  function stop() {
    active = false;
    if (listening) {
      window.removeEventListener("deviceorientation", onOrientation, true);
      listening = false;
    }
    $("startButton").disabled = false;
    $("calibrateButton").disabled = true;
    $("stopButton").disabled = true;
    status(statusConnected() ? "Connected — sensor stopped" : "Sensor stopped");
  }

  async function logout() {
    stop();
    if (socket) socket.close();
    socket = null;
    sessionStorage.removeItem("mobile_session_token");
    sessionStorage.removeItem("mobile_username");
    token = null;
    $("sensorCard").classList.add("hidden");
    $("loginCard").classList.remove("hidden");
    status("Logged out");
  }

  $("loginForm").addEventListener("submit", login);
  $("startButton").addEventListener("click", start);
  $("calibrateButton").addEventListener("click", calibrate);
  $("stopButton").addEventListener("click", stop);
  $("logoutButton").addEventListener("click", logout);
})();