(() => {
  "use strict";
  const $ = (id) => document.getElementById(id);

  let token = null, socket = null, listening = false, active = false, sequence = 0;
  let latest = { yaw: 0, pitch: 0 };
  let baseline = null;
  let lastSend = 0;
  const SEND_INTERVAL_MS = 33; // about 30 Hz

  const wrapAngle = (angle) => ((angle + 180) % 360 + 360) % 360 - 180;

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

  // Match the existing website's Mobile / VR Scene:
  // DeviceOrientation alpha/beta angles, relative to a calibrated baseline.
  function onOrientation(event) {
    if (event.alpha == null || event.beta == null) {
      status("No phone orientation readings available");
      return;
    }

    const alpha = Number(event.alpha);
    const beta = Number(event.beta);
    if (!Number.isFinite(alpha) || !Number.isFinite(beta)) return;

    if (!baseline) baseline = { alpha, beta };
    const yaw = wrapAngle(alpha - baseline.alpha);
    const pitch = wrapAngle(beta - baseline.beta);

    latest = { yaw, pitch };
    $("gx").textContent = `${yaw.toFixed(1)}°`;
    $("gy").textContent = `${pitch.toFixed(1)}°`;

    const now = Date.now();
    if (active && socket?.readyState === WebSocket.OPEN && now - lastSend >= SEND_INTERVAL_MS) {
      lastSend = now;
      socket.send(JSON.stringify({
        type: "head_orientation",
        sequence: ++sequence,
        timestamp_ms: now,
        units: "rad",
        orientation: {
          yaw: yaw * Math.PI / 180,
          pitch: pitch * Math.PI / 180,
          roll: 0
        },
        source: "phone_device_orientation"
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
      baseline = null;
      active = true;
      $("startButton").disabled = true;
      $("calibrateButton").disabled = false;
      $("stopButton").disabled = false;
      status("Orientation active — sending yaw/pitch", true);
    } catch (error) {
      status("Sensor could not start");
      alert(error.message || String(error));
    }
  }

  function calibrate() {
    // Current relative angle becomes the new neutral orientation.
    // The next orientation event re-establishes the baseline.
    baseline = null;
    latest = { yaw: 0, pitch: 0 };
    $("gx").textContent = "0.0°";
    $("gy").textContent = "0.0°";
    status("Hold phone still — setting neutral position", true);
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