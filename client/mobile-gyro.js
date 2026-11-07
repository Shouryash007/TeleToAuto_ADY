(() => {
  "use strict";
  const $ = (id) => document.getElementById(id);

  let token = null, socket = null, listening = false, active = false, sequence = 0;
  let peerConnection = null;
  let videoStatsTimer = null;
  const cameraVideo = $("robotCameraVideo");
  const cameraPlaceholder = $("robotCameraPlaceholder");
  const cameraStatus = $("robotCameraStatus");
  // The existing mobile HTML uses gx/gy value IDs; the second channel is Roll.
  document.querySelectorAll(".reading span").forEach((el) => {
    if (/GY|Pitch/i.test(el.textContent)) el.textContent = "GY · Roll";
  });
  document.querySelectorAll(".hint").forEach((el) => {
    el.textContent = "Hold the phone steady in your neutral position and press Calibrate. Turn left/right for yaw; tilt the phone top edge up/down for roll.";
  });
  // Quaternion-based calibrated orientation control with hard travel limits.
  let referenceQ = null;
  let rollAxisWorld = null;
  let pendingCalibration = false;
  let yawContinuous = 0;
  let rollContinuous = 0;
  let smoothedYaw = 0;
  let smoothedRoll = 0;
  let lastFilterTime = 0;
  let lastSend = 0;

  const SEND_INTERVAL_MS = 33; // approximately 30 Hz
  const YAW_LIMIT_DEG = 80;    // GX: +/-80 degrees
  const ROLL_LIMIT_DEG = 60;   // Roll: +/-60 degrees
  const MAX_RATE_DEG_S = 100;  // limits sudden per-frame changes
  const FILTER_ALPHA = 0.22;  // lower = smoother, higher = more responsive
  const DEADBAND_DEG = 0.35;
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

  async function login(event) {
    event.preventDefault();
    $("loginError").textContent = "";
    try {
      const response = await fetch("/api/login", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          username: $("username").value.trim(),
          password: $("password").value
        })
      });
      const data = await response.json();
      if (!response.ok || !data.success) throw new Error(data.error || "Login failed");

      token = data.token;
      sessionStorage.setItem("mobile_session_token", token);
      sessionStorage.setItem("mobile_username", data.username);
      $("loginCard").classList.add("hidden");
      $("sensorCard").classList.remove("hidden");
      $("sensorCardControls").classList.remove("hidden");
      await connectSocket();
    } catch (error) {
      $("loginError").textContent = error.message || "Unable to log in";
      status("Login failed");
    }
  }


  function stopVideoStats() {
    if (videoStatsTimer !== null) {
      clearInterval(videoStatsTimer);
      videoStatsTimer = null;
    }
  }

  function startVideoStats(pc) {
    stopVideoStats();
    videoStatsTimer = setInterval(async () => {
      if (peerConnection !== pc || pc.connectionState === "closed") {
        stopVideoStats();
        return;
      }
      try {
        const stats = await pc.getStats();
        stats.forEach((r) => {
          if (r.type === "inbound-rtp" && (r.kind === "video" || r.mediaType === "video")) {
            const jitterMs = r.jitterBufferEmittedCount
              ? (r.jitterBufferDelay / r.jitterBufferEmittedCount * 1000).toFixed(0)
              : "n/a";
            console.log(
              `[MOBILE WEBRTC] fps ${r.framesPerSecond ?? "n/a"}, ` +
              `jitterBuffer ${jitterMs} ms, dropped ${r.framesDropped ?? "n/a"}, ` +
              `decoded ${r.framesDecoded ?? "n/a"}`
            );
          }
          if (r.type === "candidate-pair" && r.state === "succeeded" && r.nominated) {
            console.log(`[MOBILE WEBRTC] RTT ${(r.currentRoundTripTime * 1000).toFixed(0)} ms`);
          }
        });
      } catch (error) {
        console.debug("[MOBILE WEBRTC] Stats unavailable:", error);
      }
    }, 2000);
  }

  function closeVideoPeer() {
    stopVideoStats();
    if (peerConnection) { try { peerConnection.close(); } catch {} peerConnection = null; }
    if (cameraVideo) cameraVideo.srcObject = null;
    cameraPlaceholder?.classList.remove("hidden");
    if (cameraStatus) { cameraStatus.textContent = "VIDEO DISCONNECTED"; cameraStatus.classList.remove("live"); }
  }
  function waitForIceGathering(pc) {
    if (pc.iceGatheringState === "complete") return Promise.resolve();
    return new Promise((resolve) => {
      const timer = setTimeout(done, 5000);
      function check() { if (pc.iceGatheringState === "complete") done(); }
      function done() { clearTimeout(timer); pc.removeEventListener("icegatheringstatechange", check); resolve(); }
      pc.addEventListener("icegatheringstatechange", check);
    });
  }
  async function handleVideoOffer(data) {
    closeVideoPeer();
    // A public STUN server helps phones on a different network discover
    // a reachable ICE candidate. Configure TURN as well for restrictive NATs.
    const iceServers = [{
      urls: window.TELEOP_STUN_URL || "stun:stun.l.google.com:19302"
    }];
    if (window.TELEOP_TURN_URL && window.TELEOP_TURN_USERNAME && window.TELEOP_TURN_CREDENTIAL) {
      iceServers.push({
        urls: window.TELEOP_TURN_URL,
        username: window.TELEOP_TURN_USERNAME,
        credential: window.TELEOP_TURN_CREDENTIAL
      });
    }
    const pc = new RTCPeerConnection({ iceServers });
    peerConnection = pc;
    startVideoStats(pc);
    pc.ontrack = async (event) => {
      cameraVideo.srcObject = event.streams?.[0] || new MediaStream([event.track]);
      try {
        await cameraVideo.play();
        cameraPlaceholder?.classList.add("hidden");
        cameraStatus.textContent = "LIVE · WEBRTC";
        cameraStatus.classList.add("live");
      } catch (error) { cameraStatus.textContent = "TAP VIDEO TO PLAY"; }
    };
    pc.onconnectionstatechange = () => {
      console.log("[MOBILE VIDEO] connection state:", pc.connectionState);
      if (cameraStatus) {
        cameraStatus.textContent = pc.connectionState === "connected"
          ? "LIVE · WEBRTC"
          : `VIDEO ${pc.connectionState.toUpperCase()}`;
        cameraStatus.classList.toggle("live", pc.connectionState === "connected");
      }
    };
    pc.oniceconnectionstatechange = () => {
      console.log("[MOBILE VIDEO] ICE state:", pc.iceConnectionState);
    };
    await pc.setRemoteDescription({ type: data.type, sdp: data.sdp });
    const answer = await pc.createAnswer();
    await pc.setLocalDescription(answer);
    await waitForIceGathering(pc);
    if (socket?.readyState === WebSocket.OPEN && peerConnection === pc) {
      socket.send(JSON.stringify({ type: pc.localDescription.type, sdp: pc.localDescription.sdp, viewer: "mobile" }));
    }
  }

  function connectSocket() {
    return new Promise((resolve, reject) => {
      socket = new WebSocket(wsUrl());
      const timeout = setTimeout(() => reject(new Error("Signaling server timed out")), 8000);

      socket.onopen = () => socket.send(JSON.stringify({
        role: "mobile_gyro",
        token,
        robot_id: $("robotId").value.trim()
      }));

      socket.onmessage = (event) => {
        let data;
        try { data = JSON.parse(event.data); } catch { return; }
        if (data.type === "mobile_gyro_connected") {
          clearTimeout(timeout);
          status(`Connected to ${data.robot_id}. Sensor is off.`, true);
          resolve();
        } else if (data.type === "offer" && data.viewer === "mobile") {
          handleVideoOffer(data).catch((error) => {
            console.error("[MOBILE VIDEO] Offer failed", error);
            if (cameraStatus) cameraStatus.textContent = "VIDEO SETUP FAILED";
          });
        } else if (data.type === "operator_disconnected") {
          closeVideoPeer();
          status("Laptop operator disconnected");
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

  function qMultiply(a, b) {
    return {
      w: a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
      x: a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
      y: a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
      z: a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
  }
  function qConjugate(q) { return { w:q.w, x:-q.x, y:-q.y, z:-q.z }; }
  function qNormalize(q) {
    const n = Math.hypot(q.w, q.x, q.y, q.z);
    if (!Number.isFinite(n) || n < 1e-10) return null;
    return { w:q.w/n, x:q.x/n, y:q.y/n, z:q.z/n };
  }
  function qAxis(axis, angle) {
    const s = Math.sin(angle/2), c = Math.cos(angle/2);
    return { w:c, x:axis === "x" ? s : 0, y:axis === "y" ? s : 0, z:axis === "z" ? s : 0 };
  }
  function qRotate(q, v) {
    const p = { w:0, x:v.x, y:v.y, z:v.z };
    const r = qMultiply(qMultiply(q, p), qConjugate(q));
    return { x:r.x, y:r.y, z:r.z };
  }
  // Convert the phone's alpha/beta/gamma orientation to a quaternion.
  function readDeviceQuaternion(event) {
    if (event.alpha == null || event.beta == null || event.gamma == null) return null;
    const a = Number(event.alpha), b = Number(event.beta), g = Number(event.gamma);
    if (![a,b,g].every(Number.isFinite)) return null;
    return qNormalize(qMultiply(qMultiply(qAxis("z", toRad(a)), qAxis("x", toRad(b))), qAxis("y", toRad(g))));
  }
  function normalizeVector(v) {
    const n = Math.hypot(v.x, v.y, v.z);
    if (!Number.isFinite(n) || n < 1e-8) return null;
    return { x:v.x/n, y:v.y/n, z:v.z/n };
  }
  function twistAngle(q, axis) {
    const projection = q.x*axis.x + q.y*axis.y + q.z*axis.z;
    const twist = qNormalize({ w:q.w, x:axis.x*projection, y:axis.y*projection, z:axis.z*projection });
    if (!twist) return 0;
    let angle = 2 * Math.atan2(twist.x*axis.x + twist.y*axis.y + twist.z*axis.z, twist.w);
    if (angle > Math.PI) angle -= 2*Math.PI;
    if (angle < -Math.PI) angle += 2*Math.PI;
    return angle;
  }
  function wrapPi(a) { return ((a + Math.PI) % (2*Math.PI) + 2*Math.PI) % (2*Math.PI) - Math.PI; }
  function unwrap(previous, principal) { return previous + wrapPi(principal - previous); }
  function calibrateFrom(q) {
    referenceQ = q;
    // Match the old phone movement design: use the calibrated phone top-edge
    // axis projected onto the horizontal plane for the up/down movement.
    // This channel is named ROLL in this project (not pitch).
    const phoneTopAxis = qRotate(q, { x:0, y:1, z:0 });
    rollAxisWorld = normalizeVector({ x:phoneTopAxis.x, y:phoneTopAxis.y, z:0 });
    if (!rollAxisWorld) {
      status("Calibration failed — hold phone upright and steady, then retry", false);
      referenceQ = null;
      return false;
    }
    yawContinuous = 0;
    rollContinuous = 0;
    smoothedYaw = 0;
    smoothedRoll = 0;
    lastFilterTime = 0;
    $("gx").textContent = "0.0°";
    $("gy").textContent = "0.0°";
    status("Calibrated — turn left/right for yaw; tilt up/down for roll", true);
    return true;
  }
  function onOrientation(event) {
    const currentQ = readDeviceQuaternion(event);
    if (!currentQ) { status("Waiting for complete phone orientation readings"); return; }
    if (pendingCalibration || !referenceQ) {
      pendingCalibration = false;
      calibrateFrom(currentQ);
      return;
    }
    let delta = qNormalize(qMultiply(currentQ, qConjugate(referenceQ)));
    if (!delta) return;
    if (delta.w < 0) delta = { w:-delta.w, x:-delta.x, y:-delta.y, z:-delta.z };

    const worldUp = { x:0, y:0, z:1 };
    const yawPrincipal = twistAngle(delta, worldUp);
    const yawQ = qNormalize({ w:Math.cos(yawPrincipal/2), x:0, y:0, z:Math.sin(yawPrincipal/2) });
    const residual = qNormalize(qMultiply(qConjugate(yawQ), delta));
    if (!residual) return;
    const rollPrincipal = twistAngle(residual, rollAxisWorld);

    yawContinuous = unwrap(yawContinuous, yawPrincipal);
    rollContinuous = unwrap(rollContinuous, rollPrincipal);

    // Clamp raw targets, then rate-limit and low-pass filter them to suppress jumps.
    const targetYaw = Math.max(-YAW_LIMIT_DEG, Math.min(YAW_LIMIT_DEG, toDeg(yawContinuous)));
    const targetRoll = Math.max(-ROLL_LIMIT_DEG, Math.min(ROLL_LIMIT_DEG, toDeg(rollContinuous)));
    const now = Date.now();
    const dt = lastFilterTime ? Math.min((now - lastFilterTime) / 1000, 0.1) : 1 / 30;
    lastFilterTime = now;
    const maxStep = MAX_RATE_DEG_S * dt;
    const limitedYaw = Math.max(smoothedYaw - maxStep, Math.min(smoothedYaw + maxStep, targetYaw));
    const limitedRoll = Math.max(smoothedRoll - maxStep, Math.min(smoothedRoll + maxStep, targetRoll));
    smoothedYaw += (limitedYaw - smoothedYaw) * FILTER_ALPHA;
    smoothedRoll += (limitedRoll - smoothedRoll) * FILTER_ALPHA;

    const yaw = Math.abs(smoothedYaw) < DEADBAND_DEG ? 0 : -smoothedYaw;
    const roll = Math.abs(smoothedRoll) < DEADBAND_DEG ? 0 : -smoothedRoll;
    $("gx").textContent = `${yaw.toFixed(1)}°`;
    $("gy").textContent = `${roll.toFixed(1)}°`;

    if (active && socket?.readyState === WebSocket.OPEN && now - lastSend >= SEND_INTERVAL_MS) {
      lastSend = now;
      socket.send(JSON.stringify({
        type: "head_orientation",
        sequence: ++sequence,
        timestamp_ms: now,
        units: "rad",
        orientation: { yaw: toRad(yaw), pitch: 0, roll: toRad(roll) },
        source: "phone_yaw_roll_old_axis_mapping_smoothed"
      }));
    }
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
      rollAxisWorld = null;
      pendingCalibration = true;
      active = true;
      $("startButton").disabled = true;
      $("calibrateButton").disabled = false;
      $("stopButton").disabled = false;
      status("Hold phone in your usual landscape position, screen facing you; calibrating neutral", true);
    } catch (error) {
      status("Sensor could not start");
      alert(error.message || String(error));
    }
  }

  function calibrate() {
    referenceQ = null;
    rollAxisWorld = null;
    pendingCalibration = true;
    yawContinuous = 0;
    rollContinuous = 0;
    $("gx").textContent = "0.0°";
    $("gy").textContent = "0.0°";
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
    status(socket?.readyState === WebSocket.OPEN ? "Connected — sensor stopped" : "Sensor stopped");
  }

  async function logout() {
    stop();
    closeVideoPeer();
    if (socket) socket.close();
    socket = null;
    sessionStorage.removeItem("mobile_session_token");
    sessionStorage.removeItem("mobile_username");
    token = null;
    $("sensorCard").classList.add("hidden");
    $("sensorCardControls").classList.add("hidden");
    $("loginCard").classList.remove("hidden");
    status("Logged out");
  }

  $("loginForm").addEventListener("submit", login);
  $("startButton").addEventListener("click", start);
  $("calibrateButton").addEventListener("click", calibrate);
  $("stopButton").addEventListener("click", stop);
  $("logoutButton").addEventListener("click", logout);
})();