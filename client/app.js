/* Bundled desktop client: GyroscopeInput + MobileHeadScene + application logic. */

/*
 * Bluetooth gyroscope input module.
 * Expected ESP32 BLE service: Nordic UART Service (NUS)
 * Notify characteristic must emit either JSON {"gx": 1.2, "gy": -3.4}\n
 * or CSV: 1.2,-3.4\n
 * Configure window.TELEOP_GYRO_BLE in index.html if your firmware uses
 * different UUIDs. gx/gy are treated as angular velocity in deg/s.
 */
(function () {
    "use strict";

    const DEFAULTS = {
        serviceUuid: "6e400001-b5a3-f393-e0a9-e50e24dcca9e",
        notifyCharacteristicUuid: "6e400003-b5a3-f393-e0a9-e50e24dcca9e",
        deviceNamePrefix: ""
    };

    let device = null;
    let server = null;
    let characteristic = null;
    let onSample = null;
    let textBuffer = "";
    const decoder = new TextDecoder();

    function config() {
        return { ...DEFAULTS, ...(window.TELEOP_GYRO_BLE || {}) };
    }

    function parseLine(line) {
        const text = line.trim();
        if (!text) return null;
        try {
            const data = JSON.parse(text);
            const gx = Number(data.gx ?? data.gyroscope_x);
            const gy = Number(data.gy ?? data.gyroscope_y);
            if (Number.isFinite(gx) && Number.isFinite(gy)) return { gx, gy };
        } catch (_) {
            // Accept simple CSV as an alternative to JSON.
        }
        const parts = text.split(/[;,\s]+/).filter(Boolean);
        if (parts.length >= 2) {
            const gx = Number(parts[0]);
            const gy = Number(parts[1]);
            if (Number.isFinite(gx) && Number.isFinite(gy)) return { gx, gy };
        }
        console.warn("[GYRO] Ignoring unrecognized BLE data:", text);
        return null;
    }

    function handleNotification(event) {
        const bytes = event.target.value;
        const chunk = decoder.decode(bytes);
        textBuffer += chunk;
        if (textBuffer.length > 2048) textBuffer = textBuffer.slice(-2048);
        const lines = textBuffer.split(/\r?\n/);
        textBuffer = lines.pop() || "";
        for (const line of lines) {
            const sample = parseLine(line);
            if (sample && typeof onSample === "function") onSample(sample);
        }
        // Also accept one complete JSON/CSV packet without a newline.
        if (!textBuffer.includes("{") && /[,;]/.test(textBuffer)) {
            const sample = parseLine(textBuffer);
            if (sample && typeof onSample === "function") onSample(sample);
            textBuffer = "";
        }
    }

    async function connect(callback) {
        if (!navigator.bluetooth) {
            throw new Error("Web Bluetooth is unavailable. Use Chrome on Android over HTTPS, or localhost on a supported desktop browser.");
        }
        if (typeof callback !== "function") throw new Error("A gyroscope data callback is required.");
        const cfg = config();
        onSample = callback;
        device = await navigator.bluetooth.requestDevice({
            acceptAllDevices: true,
            optionalServices: [cfg.serviceUuid]
        });
        device.addEventListener("gattserverdisconnected", onDisconnected);
        server = await device.gatt.connect();
        const service = await server.getPrimaryService(cfg.serviceUuid);
        characteristic = await service.getCharacteristic(cfg.notifyCharacteristicUuid);
        characteristic.addEventListener("characteristicvaluechanged", handleNotification);
        await characteristic.startNotifications();
        textBuffer = "";
        console.info("[GYRO] Bluetooth connected:", device.name || "BLE device");
        return { name: device.name || "BLE device" };
    }

    function onDisconnected() {
        console.warn("[GYRO] BLE device disconnected");
        characteristic = null;
        server = null;
        device = null;
    }

    async function disconnect() {
        if (characteristic) {
            characteristic.removeEventListener("characteristicvaluechanged", handleNotification);
            try { await characteristic.stopNotifications(); } catch (_) {}
        }
        if (device?.gatt?.connected) device.gatt.disconnect();
        characteristic = null;
        server = null;
        device = null;
        onSample = null;
        textBuffer = "";
    }

    window.GyroscopeInput = { connect, disconnect };
})();


/* Phone browser head-orientation input for Mobile / VR Scene.
 * Uses DeviceOrientationEvent (relative yaw/pitch angles), not BLE.
 * The page must be opened on the phone over HTTPS (or localhost).
 */
(function () {
  "use strict";
  let listening = false;
  let baseline = null;
  let latest = { alpha: 0, beta: 0 };
  let enabled = false;

  const $ = (id) => document.getElementById(id);
  const wrapAngle = (angle) => ((angle + 180) % 360 + 360) % 360 - 180;

  function renderStatus(text, active = false) {
    const el = $("mobileGyroStatus");
    if (el) { el.textContent = text; el.classList.toggle("active", active); }
  }

  function onOrientation(event) {
    if (event.alpha == null || event.beta == null) {
      renderStatus("NO SENSOR DATA");
      return;
    }
    latest = { alpha: Number(event.alpha), beta: Number(event.beta) };
    if (!baseline) baseline = { ...latest };
    const yaw = wrapAngle(latest.alpha - baseline.alpha);
    const pitch = wrapAngle(latest.beta - baseline.beta);
    window.TeleopMobileBridge?.setOrientation?.({ gx: yaw, gy: pitch });
    if (!enabled) return;
    renderStatus("HEAD TRACKING ACTIVE", true);
  }

  async function start() {
    if (!window.isSecureContext) {
      alert("Phone sensors require HTTPS. Open the website using its HTTPS tunnel address.");
      return;
    }
    if (!window.DeviceOrientationEvent) {
      alert("This browser does not expose device orientation sensors.");
      return;
    }
    try {
      if (typeof DeviceOrientationEvent.requestPermission === "function") {
        const permission = await DeviceOrientationEvent.requestPermission();
        if (permission !== "granted") throw new Error("Motion sensor permission was not granted.");
      }
      if (!listening) {
        window.addEventListener("deviceorientation", onOrientation, true);
        listening = true;
      }
      baseline = null;
      enabled = true;
      window.TeleopMobileBridge?.setActive?.(true);
      $("enableMobileGyroButton").disabled = true;
      $("calibrateMobileGyroButton").disabled = false;
      $("stopMobileGyroButton").disabled = false;
      renderStatus("WAITING FOR SENSOR", true);
    } catch (error) {
      console.error("[MOBILE GYRO] Start failed:", error);
      alert("Could not enable phone orientation: " + (error.message || error));
    }
  }

  function calibrate() {
    if (latest.alpha == null || latest.beta == null) return;
    baseline = { ...latest };
    window.TeleopMobileBridge?.setOrientation?.({ gx: 0, gy: 0 });
    renderStatus("NEUTRAL POSITION SET", true);
  }

  function stop() {
    enabled = false;
    if (listening) { window.removeEventListener("deviceorientation", onOrientation, true); listening = false; }
    window.TeleopMobileBridge?.setActive?.(false);
    if ($("enableMobileGyroButton")) $("enableMobileGyroButton").disabled = false;
    if ($("calibrateMobileGyroButton")) $("calibrateMobileGyroButton").disabled = true;
    if ($("stopMobileGyroButton")) $("stopMobileGyroButton").disabled = true;
    renderStatus("SENSOR OFF");
  }

  function init() {
    $("enableMobileGyroButton")?.addEventListener("click", start);
    $("calibrateMobileGyroButton")?.addEventListener("click", calibrate);
    $("stopMobileGyroButton")?.addEventListener("click", stop);
  }

  window.MobileHeadScene = { init, start, stop, calibrate };
})();


// ============================================================
// OPERATOR FRONT END: HUMAN ARM TRACKING + ESP32 IMU
// -> ROBOT SIMULATION
//
// Network streams:
// 1. joint-control DataChannel: calculated arm joint targets
// 2. imu-control DataChannel: phone head orientation
// 3. robot-camera WebRTC video track: simulation -> browser
//
// Human webcam processing happens locally in the browser.
// No operator webcam video is sent to the robot.
//
// IMPORTANT:
// The current 2D landmark mapping is approximate, not calibrated IK.
// Test in simulation only until the mapping and limits are verified.
// ============================================================

"use strict";

// -----------------------------
// Application state
// -----------------------------
let sessionToken = null;
let username = null;
let websocket = null;
let peerConnection = null;
let selectedRobot = null;

let jointChannel = null;
let imuChannel = null;

// Local-only operator webcam and tracking
let operatorStream = null;
let operatorVideo = null;
let operatorCanvas = null;
let operatorCtx = null;
let trackingActive = false;
let mediaPipeLoaded = false;
let poseModel = null;
let handsModel = null;
let poseBusy = false;
let handsBusy = false;

let latestImu = { gx: 0, gy: 0 }; // gx = relative yaw degrees; gy = relative pitch degrees
let mobileGyroActive = false;
let imuSequence = 0;
let jointSequence = 0;
let lastJointSend = 0;
let lastImuSend = 0;
let lastPoseSeen = 0;
let lastHandSeen = 0;

let gripperClosedBySide = { left: false, right: false };
let lastGripperChangeBySide = { left: 0, right: 0 };
let neutralPose = null;

const CONTROL = {
    sendRateHz: 20,
    imuRateHz: 30,
    stalePoseMs: 500,
    staleHandMs: 700,
    maxJointStepDeg: 8,
    smoothingAlpha: 0.35,
    gripperCloseThreshold: 0.72,
    gripperOpenThreshold: 0.48,
    gripperDebounceMs: 180,

    // Demonstration limits only. Match these to the URDF.
    jointLimitsDeg: {
        shoulder_pan: [-90, 90],
        shoulder_lift: [-75, 75],
        elbow_flex: [-10, 120],
        wrist_flex: [-90, 90],
        wrist_roll: [-90, 90]
    }
};

const JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper"
];

const $ = (id) => document.getElementById(id);

const loginPage = $("loginPage");
const dashboardPage = $("dashboardPage");
const loginForm = $("loginForm");
const loginError = $("loginError");
const loggedUser = $("loggedUser");
const robotList = $("robotList");
const cameraPanel = $("cameraPanel");
const robotSelection = $("robotSelection");
const currentRobot = $("currentRobot");
const connectionStatus = $("connectionStatus");
const cameraState = $("cameraState");
const remoteVideo = $("remoteVideo");
const videoOverlay = $("videoOverlay");

// -----------------------------
// Login
// -----------------------------
if (loginForm) {
    loginForm.addEventListener("submit", async (event) => {
        event.preventDefault();

        if (loginError) loginError.textContent = "";

        const user = $("username")?.value.trim() || "";
        const password = $("password")?.value || "";

        try {
            const response = await fetch("/api/login", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({
                    username: user,
                    password
                })
            });

            const data = await response.json();

            if (!response.ok || !data.success) {
                if (loginError) {
                    loginError.textContent =
                        data.error || "Login failed";
                }
                return;
            }

            sessionToken = data.token;
            username = data.username;

            sessionStorage.setItem(
                "session_token",
                sessionToken
            );
            sessionStorage.setItem("username", username);

            if (loggedUser) {
                loggedUser.textContent = username;
            }

            showDashboard();
            await loadRobots();

        } catch (error) {
            console.error("[AUTH] Login error:", error);

            if (loginError) {
                loginError.textContent =
                    "Unable to connect to server";
            }
        }
    });
}

function showDashboard() {
    loginPage?.classList.add("hidden");
    dashboardPage?.classList.remove("hidden");
}

// -----------------------------
// Robot list and access
// -----------------------------
async function loadRobots() {
    if (!robotList) return;

    robotList.innerHTML = "<p>Loading robots...</p>";

    const token =
        sessionToken ||
        sessionStorage.getItem("session_token");

    if (!token) {
        sessionStorage.clear();
        dashboardPage?.classList.add("hidden");
        loginPage?.classList.remove("hidden");
        return;
    }

    try {
        const response = await fetch("/api/robots", {
            headers: {
                "Authorization": `Bearer ${token}`
            }
        });

        if (response.status === 401) {
            sessionStorage.clear();
            sessionToken = null;
            username = null;

            dashboardPage?.classList.add("hidden");
            loginPage?.classList.remove("hidden");
            return;
        }

        if (!response.ok) {
            throw new Error(
                `Robot request failed: ${response.status}`
            );
        }

        const data = await response.json();
        robotList.innerHTML = "";

        for (const robot of (data.robots || [])) {
            const card = document.createElement("div");
            card.className = "robot-card";

            const status = robot.online
                ? "ONLINE"
                : "OFFLINE";

            const icon = document.createElement("div");
            icon.className =
                "robot-icon " +
                (
                    robot.busy
                        ? "robot-busy"
                        : robot.online
                            ? "robot-online"
                            : "robot-offline"
                );
            icon.textContent = "●";

            const details = document.createElement("div");
            details.className = "robot-details";

            const name = document.createElement("strong");
            name.textContent = robot.robot_id;

            const state = document.createElement("span");
            state.className =
                !robot.online
                    ? "offline"
                    : robot.busy
                        ? "busy"
                        : "online";
            state.textContent = status;

            details.append(name, state);

            if (robot.online) {
                const use = document.createElement("span");
                use.className =
                    robot.busy ? "used-by" : "free";

                use.textContent = robot.busy
                    ? `USED BY: ${robot.used_by || "another operator"}`
                    : "FREE";

                details.appendChild(use);
            }

            const button = document.createElement("button");
            button.className =
                `connect-button ${robot.busy ? "busy-button" : ""}`;

            button.textContent =
                robot.busy ? "IN USE" : "CONNECT";

            button.disabled =
                !robot.online || !!robot.busy;

            if (robot.online && !robot.busy) {
                button.addEventListener("click", () => {
                    connectToRobot(robot.robot_id);
                });
            }

            card.append(icon, details, button);
            robotList.appendChild(card);
        }

    } catch (error) {
        console.error("[CLIENT] Robot list error:", error);
        robotList.innerHTML =
            "<p>Failed to load robots.</p>";
    }
}

// -----------------------------
// WebSocket signaling
// The server handles signaling/authentication.
// The data channels carry the actual control values.
// -----------------------------
function connectToRobot(robotId) {
    selectedRobot = robotId;

    robotSelection?.classList.add("hidden");
    cameraPanel?.classList.remove("hidden");
    // The four-panel console sits beside cameraPanel, so show it explicitly.
    ensureOperatorPanel();
    $("teleopFourPanel")?.classList.remove("hidden");

    if (currentRobot) {
        currentRobot.textContent = robotId;
    }

    if (connectionStatus) {
        connectionStatus.textContent = "Connecting...";
    }

    if (cameraState) {
        cameraState.textContent =
            "Waiting for robot camera";
    }

    videoOverlay?.classList.remove("hidden");

    const protocol =
        window.location.protocol === "https:"
            ? "wss:"
            : "ws:";

    websocket = new WebSocket(
        `${protocol}//${window.location.host}/ws`
    );

    websocket.onopen = () => {
        websocket.send(JSON.stringify({
            role: "browser",
            token:
                sessionToken ||
                sessionStorage.getItem("session_token"),
            robot_id: robotId
        }));
    };

    websocket.onmessage = async (event) => {
        let data;

        try {
            data = JSON.parse(event.data);
        } catch {
            console.warn("[SIGNAL] Invalid JSON from server");
            return;
        }

        if (data.type === "connected") {
            if (connectionStatus) {
                connectionStatus.textContent =
                    "Signaling connected";
            }

        } else if (data.type === "offer") {
            try {
                await handleOffer(data);
            } catch (error) {
                console.error(
                    "[WEBRTC] Offer handling failed:",
                    error
                );

                if (connectionStatus) {
                    connectionStatus.textContent =
                        "WebRTC setup failed";
                }
            }

        } else if (data.type === "robot_disconnected") {
            if (connectionStatus) {
                connectionStatus.textContent =
                    "Robot disconnected";
            }

            if (cameraState) {
                cameraState.textContent = "Robot offline";
            }
            $("robotCameraPlaceholder")?.classList.remove("hidden");
            setStatus("robotCameraStatus", "WAITING");

            stopControlTransmission();
            closePeerConnection();

        } else if (data.type === "error") {
            alert(data.message || "Connection error");
            disconnectFromRobot();

        } else if (data.type === "joint_feedback") {
            renderJointFeedback(data);

        } else if (data.type === "imu_feedback") {
            renderImuFeedback(data);
        }
    };

    websocket.onerror = (error) => {
        console.error("[SIGNAL] WebSocket error:", error);

        if (connectionStatus) {
            connectionStatus.textContent = "Signaling error";
        }
    };

    websocket.onclose = () => {
        console.log("[SIGNAL] WebSocket closed");
    };
}

// -----------------------------
// WebRTC peer connection
// -----------------------------
function createPeerConnection() {
    // Configure real STUN/TURN services for remote use.
    // These values must be configured for your deployment.
    const iceServers = [];

    const stunUrl = window.TELEOP_STUN_URL || "";
    const turnUrl = window.TELEOP_TURN_URL || "";

    if (stunUrl) {
        iceServers.push({ urls: stunUrl });
    }

    if (
        turnUrl &&
        window.TELEOP_TURN_USERNAME &&
        window.TELEOP_TURN_CREDENTIAL
    ) {
        iceServers.push({
            urls: turnUrl,
            username: window.TELEOP_TURN_USERNAME,
            credential: window.TELEOP_TURN_CREDENTIAL
        });
    }

    const pc = new RTCPeerConnection({ iceServers });

    pc.ontrack = async (event) => {
        // The only expected video track is the robot's
        // simulated head-camera view.
        if (!remoteVideo) return;

        const stream =
            event.streams?.[0] ||
            new MediaStream([event.track]);

        remoteVideo.srcObject = stream;
        const mobileVideo = $("mobileRemoteVideo");
        if (mobileVideo) mobileVideo.srcObject = stream;
        $("mobileVideoPlaceholder")?.classList.add("hidden");

        try {
            await remoteVideo.play();

            videoOverlay?.classList.add("hidden");
            $("robotCameraPlaceholder")?.classList.add("hidden");
            setStatus("robotCameraStatus", "LIVE");

            if (cameraState) {
                cameraState.textContent =
                    "LIVE — ROBOT VIEW";
            }

            if (connectionStatus) {
                connectionStatus.textContent =
                    "WebRTC connected";
            }

        } catch (error) {
            console.warn(
                "[VIDEO] Autoplay failed; user interaction may be required.",
                error
            );
        }
    };

    pc.ondatachannel = (event) => {
        const channel = event.channel;

        console.log(
            "[DATA] DataChannel received:",
            channel.label
        );

        if (channel.label === "joint-control") {
            jointChannel = channel;

            channel.onopen = () => {
                setStatus("jointChannelStatus", "OPEN");
                updateControlButtonState();
            };

            channel.onclose = () => {
                setStatus("jointChannelStatus", "CLOSED");
                jointChannel = null;
                stopControlTransmission();
                updateControlButtonState();
            };

            channel.onerror = (error) => {
                console.error(
                    "[DATA] Joint channel error:",
                    error
                );
            };

            channel.onmessage = (messageEvent) => {
                handleRobotData(messageEvent.data, "joint");
            };

        } else if (channel.label === "imu-control") {
            imuChannel = channel;

            channel.onopen = () => {
                setStatus("imuChannelStatus", "OPEN");
                updateControlButtonState();
            };

            channel.onclose = () => {
                setStatus("imuChannelStatus", "CLOSED");
                imuChannel = null;
                stopControlTransmission();
                updateControlButtonState();
            };

            channel.onerror = (error) => {
                console.error(
                    "[DATA] IMU channel error:",
                    error
                );
            };

            channel.onmessage = (messageEvent) => {
                handleRobotData(messageEvent.data, "imu");
            };

        } else {
            console.warn(
                "[DATA] Unknown DataChannel:",
                channel.label
            );

            channel.close();
        }
    };

    pc.onconnectionstatechange = () => {
        console.log(
            "[WEBRTC] Connection state:",
            pc.connectionState
        );

        if (
            pc.connectionState === "failed" ||
            pc.connectionState === "closed"
        ) {
            stopControlTransmission();
        }
    };

    pc.oniceconnectionstatechange = () => {
        console.log(
            "[WEBRTC] ICE state:",
            pc.iceConnectionState
        );
    };

    return pc;
}

async function handleOffer(data) {
    closePeerConnection();

    peerConnection = createPeerConnection();

    await peerConnection.setRemoteDescription({
        type: data.type,
        sdp: data.sdp
    });

    // IMPORTANT:
    // Do NOT add the operator webcam to the peer connection.
    // The webcam is processed locally by MediaPipe.
    // The robot should create the joint-control and imu-control
    // DataChannels and offer its camera video track.

    const answer = await peerConnection.createAnswer();

    await peerConnection.setLocalDescription(answer);
    await waitForIceGathering(peerConnection);

    if (
        !websocket ||
        websocket.readyState !== WebSocket.OPEN
    ) {
        throw new Error("Signaling WebSocket is not open");
    }

    websocket.send(JSON.stringify({
        type: peerConnection.localDescription.type,
        sdp: peerConnection.localDescription.sdp
    }));

    console.log("[WEBRTC] SDP answer sent");
}

function waitForIceGathering(pc) {
    if (pc.iceGatheringState === "complete") {
        return Promise.resolve();
    }

    return new Promise((resolve) => {
        const check = () => {
            if (pc.iceGatheringState === "complete") {
                pc.removeEventListener(
                    "icegatheringstatechange",
                    check
                );
                resolve();
            }
        };

        pc.addEventListener(
            "icegatheringstatechange",
            check
        );
    });
}

function closePeerConnection() {
    stopControlTransmission();

    if (peerConnection) {
        try {
            peerConnection.close();
        } catch (error) {
            console.warn(error);
        }

        peerConnection = null;
    }

    jointChannel = null;
    imuChannel = null;

    if (remoteVideo) {
        remoteVideo.srcObject = null;
    }

    videoOverlay?.classList.remove("hidden");
}// ============================================================
// PART 2: FOUR-PANEL OPERATOR DASHBOARD
// ============================================================

function ensureOperatorPanel() {
    // Do not create the dashboard twice.
    if ($("teleopFourPanel")) return;

    const dashboard = document.createElement("section");
    dashboard.id = "teleopFourPanel";
    dashboard.classList.add("hidden");

    dashboard.innerHTML = `
        <div class="teleop-heading">
            <h2>Teleoperation Console</h2>
            <p>
                Local arm tracking · Robot camera · Joint targets · IMU
            </p>
        </div>

        <div class="scene-mode-bar" role="group" aria-label="Teleoperation scene mode">
            <span class="scene-mode-label">VIEW MODE</span>
            <button id="laptopSceneButton" type="button" class="scene-mode-button active">Laptop Scene</button>
            <button id="mobileSceneButton" type="button" class="scene-mode-button">Mobile / VR Scene</button>
        </div>

        <section id="mobileScenePanel" class="mobile-scene-panel hidden">
            <div class="mobile-scene-topline">
                <div>
                    <p class="eyebrow">HEAD-MOUNTED DISPLAY MODE</p>
                    <h3>Robot Camera View</h3>
                    <p class="mobile-scene-help">Mount your phone securely in the goggles, enter Mobile / VR Scene, then enable head tracking.</p>
                </div>
                <span id="mobileGyroStatus" class="mobile-gyro-status">SENSOR OFF</span>
            </div>
            <div id="mobileVideoMount" class="mobile-video-mount">
                <video id="mobileRemoteVideo" autoplay muted playsinline></video>
                <div id="mobileVideoPlaceholder" class="mobile-video-placeholder">Waiting for robot simulation video…</div>
                <div class="vr-reticle" aria-hidden="true">+</div>
            </div>
            <div class="mobile-gyro-controls">
                <button id="enableMobileGyroButton" type="button" class="scene-action-primary">Enable Head Tracking</button>
                <button id="calibrateMobileGyroButton" type="button" disabled>Calibrate Neutral Position</button>
                <button id="stopMobileGyroButton" type="button" disabled>Stop Tracking</button>
            </div>
            <div class="mobile-gyro-values">
                <div><span>GX · Yaw</span><strong id="mobileGxValue">0.0°</strong></div>
                <div><span>GY · Pitch</span><strong id="mobileGyValue">0.0°</strong></div>
            </div>
            <p class="teleop-note">Phone orientation is measured relative to the calibrated neutral position. Only yaw and pitch are sent to the robot over the existing WebRTC imu-control channel. This mode uses the phone's browser sensors, not Bluetooth or Dabble.</p>
        </section>

        <div id="laptopSceneContent">
        <div class="teleop-grid">

            <section class="teleop-card">
                <h3>1. Human Arm Tracking</h3>
                <div class="teleop-status">
                    Tracking:
                    <span id="trackingStatus">NOT STARTED</span>
                </div>

                <div class="teleop-viewport">
                    <video id="operatorVideo"
                           autoplay
                           muted
                           playsinline></video>
                    <canvas id="operatorCanvas"></canvas>
                </div>

                <div class="teleop-actions">
                    <button id="startTrackingButton" type="button">
                        Start Tracking
                    </button>
                    <button id="stopTrackingButton" type="button" disabled>
                        Stop Tracking
                    </button>
                </div>

                <p class="teleop-note">
                    This preview is processed locally in your browser.
                    It is not transmitted to the robot.
                </p>
            </section>

            <section class="teleop-card">
                <h3>2. Robot Head Camera</h3>
                <div class="teleop-status">
                    Camera:
                    <span id="robotCameraStatus">WAITING</span>
                </div>

                <div id="robotCameraMount" class="teleop-viewport">
                    <div id="robotCameraPlaceholder" class="robot-camera-placeholder">
                        Waiting for simulation video...
                    </div>
                </div>

                <p class="teleop-note">
                    Live video received from the robot simulation.
                </p>
            </section>

            <section class="teleop-card">
                <h3>3. Calculated Joint Targets</h3>
                <div class="teleop-status">
                    Joint channel:
                    <span id="jointChannelStatus">WAITING</span>
                </div>

                <div class="arm-target">
                    <h4>Left Arm</h4>
                    <div id="leftJointTargets"></div>
                </div>

                <div class="arm-target">
                    <h4>Right Arm</h4>
                    <div id="rightJointTargets"></div>
                </div>

                <p class="teleop-note">
                    These are calculated targets, not measured robot positions.
                </p>
            </section>

            <section class="teleop-card">
                <h3>4. IMU / Camera Orientation</h3>
                <div class="teleop-status">
                    IMU channel:
                    <span id="imuChannelStatus">WAITING</span>
                </div>

                <div class="teleop-actions">
                    <button id="connectImuButton" type="button">
                        Connect External IMU (BLE)
                    </button>
                    <button id="disconnectImuButton" type="button" disabled>
                        Disconnect IMU
                    </button>
                </div>

                <div class="orientation-values orientation-two-axis">
                    <div>
                        <span>GX · X rotation rate</span>
                        <strong id="imuGx">0.00 °/s</strong>
                    </div>
                    <div>
                        <span>GY · Y rotation rate</span>
                        <strong id="imuGy">0.00 °/s</strong>
                    </div>
                </div>

                <p class="teleop-note">
                    Only gx and gy are displayed and sent over the WebRTC imu-control channel.
                    External BLE input only. For VR head tracking, use Mobile / VR Scene.
                </p>
            </section>

        </div>

        </div><!-- /laptopSceneContent -->

        <div class="teleop-footer">
            <span>Control transmission:</span>
            <strong id="controlTransmissionStatus">STOPPED</strong>
            <button id="disconnectRobotButton" type="button">
                Disconnect Robot
            </button>
        </div>
    `;

    // Add the dashboard below the existing camera/dashboard content.
    if (cameraPanel && cameraPanel.parentNode) {
        cameraPanel.parentNode.insertBefore(
            dashboard,
            cameraPanel.nextSibling
        );
    } else {
        document.body.appendChild(dashboard);
    }

    addTeleopStyles();

    // Reuse the existing incoming robot video element.
    const robotCameraMount = $("robotCameraMount");

    if (remoteVideo && robotCameraMount) {
        remoteVideo.classList.add("teleop-remote-video");
        robotCameraMount.appendChild(remoteVideo);
        // The original fallback container is no longer needed after moving the video.
        const oldVideoContainer = cameraPanel?.querySelector(".video-container");
        const oldCameraInfo = cameraPanel?.querySelector(".camera-info");
        if (oldVideoContainer) oldVideoContainer.classList.add("hidden");
        if (oldCameraInfo) oldCameraInfo.classList.add("hidden");
    }

    // Operator webcam: visible video with tracking landmarks overlaid.
    operatorVideo = $("operatorVideo");
    operatorCanvas = $("operatorCanvas");

    if (operatorVideo) {
        operatorVideo.style.display = "block";
        operatorVideo.style.position = "absolute";
        operatorVideo.style.inset = "0";
        operatorVideo.style.width = "100%";
        operatorVideo.style.height = "100%";
        operatorVideo.style.opacity = "1";
        operatorVideo.style.objectFit = "contain";
        operatorVideo.style.zIndex = "1";
        operatorVideo.muted = true;
        operatorVideo.autoplay = true;
        operatorVideo.playsInline = true;
        operatorVideo.style.transform = "scaleX(-1)";
    }

    if (operatorCanvas) {
        operatorCtx = operatorCanvas.getContext("2d");
        operatorCanvas.style.position = "absolute";
        operatorCanvas.style.inset = "0";
        operatorCanvas.style.width = "100%";
        operatorCanvas.style.height = "100%";
        operatorCanvas.style.zIndex = "2";
        operatorCanvas.style.pointerEvents = "none";
    }

    $("startTrackingButton")?.addEventListener(
        "click",
        startHandTracking
    );

    $("stopTrackingButton")?.addEventListener(
        "click",
        stopHandTracking
    );

    $("connectImuButton")?.addEventListener(
        "click",
        connectEsp32Imu
    );

    $("disconnectImuButton")?.addEventListener(
        "click",
        disconnectEsp32Imu
    );

    $("disconnectRobotButton")?.addEventListener(
        "click",
        disconnectFromRobot
    );

    initializeJointTargetDisplay();
    setupSceneModeButtons();
    window.MobileHeadScene?.init?.();
    updateControlButtonState();
}

// -----------------------------
// Laptop / mobile scene switching
// -----------------------------
function setupSceneModeButtons() {
    const laptopButton = $("laptopSceneButton");
    const mobileButton = $("mobileSceneButton");
    const laptopContent = $("laptopSceneContent");
    const mobilePanel = $("mobileScenePanel");
    if (!laptopButton || !mobileButton || !laptopContent || !mobilePanel) return;

    const setMode = (mode) => {
        const isMobile = mode === "mobile";
        laptopContent.classList.toggle("hidden", isMobile);
        mobilePanel.classList.toggle("hidden", !isMobile);
        laptopButton.classList.toggle("active", !isMobile);
        mobileButton.classList.toggle("active", isMobile);
        if (isMobile) {
            // Don't request sensors automatically; the user must press Enable.
            const video = $("mobileRemoteVideo");
            if (video && remoteVideo?.srcObject && !video.srcObject) video.srcObject = remoteVideo.srcObject;
        } else {
            window.MobileHeadScene?.stop?.();
        }
    };
    laptopButton.addEventListener("click", () => setMode("laptop"));
    mobileButton.addEventListener("click", () => setMode("mobile"));
}

// Bridge used by mobile-scene.js; keeps the IMU DataChannel in this main file.
window.TeleopMobileBridge = {
    setOrientation({ gx, gy }) {
        if (!Number.isFinite(gx) || !Number.isFinite(gy)) return;
        latestImu = { gx, gy };
        lastImuSend = Date.now();
        setStatus("imuGx", `${gx.toFixed(1)} °`);
        setStatus("imuGy", `${gy.toFixed(1)} °`);
        setStatus("mobileGxValue", `${gx.toFixed(1)}°`);
        setStatus("mobileGyValue", `${gy.toFixed(1)}°`);
        setStatus("imuChannelStatus", channelIsOpen(imuChannel) ? "PHONE ORIENTATION ACTIVE" : "WAITING FOR WEBRTC IMU CHANNEL");
    },
    setActive(active) {
        mobileGyroActive = Boolean(active);
        if (mobileGyroActive) maybeStartControlTransmission();
        else if (!trackingActive) stopControlTransmission();
        updateControlButtonState();
    }
};

// -----------------------------
// Basic styles for the dashboard
// -----------------------------
function addTeleopStyles() {
    if ($("teleopStyles")) return;

    const style = document.createElement("style");
    style.id = "teleopStyles";

    style.textContent = `
        #teleopFourPanel {
            width: 100%;
            margin: 24px auto;
            color: var(--text-color, #e9edf5);
            font-family: Arial, sans-serif;
        }

        .teleop-heading {
            margin-bottom: 18px;
        }

        .teleop-heading h2 {
            margin-bottom: 6px;
        }

        .teleop-heading p,
        .teleop-note {
            color: #9ca8ba;
            font-size: 12px;
            line-height: 1.5;
        }

        .teleop-grid {
            display: grid;
            grid-template-columns: repeat(2, minmax(0, 1fr));
            gap: 16px;
        }

        .teleop-card {
            min-width: 0;
            padding: 16px;
            border: 1px solid #303949;
            border-radius: 12px;
            background: #151b26;
        }

        .teleop-card h3 {
            margin: 0 0 12px;
            font-size: 16px;
        }

        .teleop-card h4 {
            margin: 12px 0 8px;
            font-size: 14px;
        }

        .teleop-status {
            margin-bottom: 10px;
            font-size: 12px;
            color: #aab5c6;
        }

        .teleop-status span {
            font-weight: bold;
            color: #70d6a1;
        }

        .teleop-viewport {
            position: relative;
            width: 100%;
            min-height: 220px;
            overflow: hidden;
            background: #080c12;
            border-radius: 8px;
            aspect-ratio: 4 / 3;
        }

        #operatorCanvas,
        .teleop-remote-video {
            display: block;
            width: 100%;
            height: 100%;
            object-fit: contain;
        }

        #operatorCanvas {
            position: absolute;
            inset: 0;
        }

        .teleop-remote-video {
            position: absolute;
            inset: 0;
        }

        .teleop-actions {
            display: flex;
            flex-wrap: wrap;
            gap: 8px;
            margin-top: 12px;
        }

        .teleop-actions button,
        .teleop-footer button {
            padding: 9px 12px;
            border: 0;
            border-radius: 6px;
            cursor: pointer;
        }

        .teleop-actions button:disabled {
            cursor: not-allowed;
            opacity: 0.5;
        }

        .arm-target {
            margin-top: 10px;
            padding: 10px;
            border-radius: 8px;
            background: #0d131d;
        }

        .joint-row {
            display: grid;
            grid-template-columns: minmax(100px, 1fr) auto;
            gap: 8px;
            padding: 5px 0;
            border-bottom: 1px solid #252d3a;
            font-size: 12px;
        }

        .joint-row:last-child {
            border-bottom: 0;
        }

        .joint-value {
            font-variant-numeric: tabular-nums;
            font-weight: bold;
        }

        .orientation-values {
            display: grid;
            grid-template-columns: repeat(3, minmax(0, 1fr));
            gap: 8px;
            margin-top: 14px;
        }

        .orientation-two-axis { grid-template-columns: repeat(2, minmax(0, 1fr)); }

        .orientation-values div {
            padding: 10px 6px;
            border-radius: 8px;
            background: #0d131d;
            text-align: center;
            min-width: 0;
        }

        .orientation-values span,
        .orientation-values strong {
            display: block;
            overflow-wrap: anywhere;
        }

        .orientation-values span {
            margin-bottom: 6px;
            color: #9ca8ba;
            font-size: 11px;
        }

        .orientation-values strong {
            font-size: 12px;
        }

        .teleop-footer {
            display: flex;
            align-items: center;
            flex-wrap: wrap;
            gap: 12px;
            margin-top: 16px;
            padding: 12px;
            border: 1px solid #303949;
            border-radius: 10px;
        }

        @media (max-width: 760px) {
            .teleop-grid {
                grid-template-columns: 1fr;
            }
        }
    `;

    document.head.appendChild(style);
}

// -----------------------------
// UI helper functions
// -----------------------------
function setStatus(id, value) {
    const element = $(id);
    if (element) element.textContent = value;
}

function initializeJointTargetDisplay() {
    for (const side of ["left", "right"]) {
        const container = $(`${side}JointTargets`);
        if (!container) continue;

        container.innerHTML = "";

        for (const name of JOINT_NAMES) {
            const row = document.createElement("div");
            row.className = "joint-row";

            const label = document.createElement("span");
            label.textContent = name.replaceAll("_", " ");

            const value = document.createElement("span");
            value.className = "joint-value";
            value.id = `${side}_${name}_value`;
            value.textContent = name === "gripper"
                ? "0% open"
                : "0.0°";

            row.append(label, value);
            container.appendChild(row);
        }
    }
}

function updateJointTargetDisplay(targets) {
    for (const side of ["left", "right"]) {
        const arm = targets?.[side];
        if (!arm) continue;

        for (const name of JOINT_NAMES) {
            const element = $(`${side}_${name}_value`);
            if (!element) continue;

            const value = Number(arm[name] ?? 0);

            element.textContent = name === "gripper"
                ? `${value.toFixed(0)}% closed`
                : `${value.toFixed(1)}°`;
        }
    }
}

function updateControlButtonState() {
    const connected =
        jointChannel?.readyState === "open" &&
        imuChannel?.readyState === "open";

    const trackingButton = $("startTrackingButton");

    if (trackingButton) {
        trackingButton.disabled = trackingActive;
    }

    setStatus(
        "controlTransmissionStatus",
        connected && trackingActive
            ? "READY — CHECK INPUT FRESHNESS"
            : "STOPPED / WAITING"
    );
}

// -----------------------------
// Incoming robot feedback
// -----------------------------
function handleRobotData(rawData, channelType) {
    let data;

    try {
        data = JSON.parse(rawData);
    } catch {
        console.warn(
            `[DATA] Invalid ${channelType} message`,
            rawData
        );
        return;
    }

    if (data.type === "joint_feedback") {
        renderJointFeedback(data);
    } else if (
        data.type === "head_orientation_feedback" ||
        data.type === "head_gyro_feedback" ||
        data.type === "imu_feedback"
    ) {
        renderImuFeedback(data);
    } else if (data.type === "robot_status") {
        console.log("[ROBOT STATUS]", data);
    } else {
        console.log(`[DATA] ${channelType} feedback:`, data);
    }
}

function renderJointFeedback(data) {
    // This function intentionally keeps measured feedback separate
    // from the calculated targets displayed in panel 3.
    console.log("[JOINT FEEDBACK]", data);
}

function renderImuFeedback(data) {
    const gyro = data.gyro || data;
    if (Number.isFinite(Number(gyro.gx))) setStatus("imuGx", `${Number(gyro.gx).toFixed(2)} °/s`);
    if (Number.isFinite(Number(gyro.gy))) setStatus("imuGy", `${Number(gyro.gy).toFixed(2)} °/s`);
}// ============================================================
// PART 3: LOCAL WEBCAM + MEDIAPIPE TRACKING + JOINT MAPPING
// ============================================================

// Initial targets. Angles are in degrees.
// Gripper convention: 0 = open, 100 = closed.
let latestTargets = {
    left: {
        shoulder_pan: 0,
        shoulder_lift: 0,
        elbow_flex: 0,
        wrist_flex: 0,
        wrist_roll: 0,
        gripper: 0
    },
    right: {
        shoulder_pan: 0,
        shoulder_lift: 0,
        elbow_flex: 0,
        wrist_flex: 0,
        wrist_roll: 0,
        gripper: 0
    }
};

let smoothedTargets = JSON.parse(JSON.stringify(latestTargets));

let latestPoseLandmarks = null;
let latestHandLandmarks = null;
let trackingLoopRunning = false;

// -----------------------------
// Load MediaPipe scripts
// -----------------------------
function loadScriptOnce(src, globalName) {
    if (window[globalName]) {
        return Promise.resolve();
    }

    return new Promise((resolve, reject) => {
        const existing = [
            ...document.querySelectorAll("script")
        ].find(script => script.src === src);

        if (existing) {
            existing.addEventListener("load", resolve, {
                once: true
            });

            existing.addEventListener("error", reject, {
                once: true
            });

            return;
        }

        const script = document.createElement("script");
        script.src = src;
        script.async = true;

        script.onload = () => {
            if (window[globalName]) {
                resolve();
            } else {
                reject(
                    new Error(
                        `Script loaded but ${globalName} was not found`
                    )
                );
            }
        };

        script.onerror = () => {
            reject(new Error(`Could not load ${src}`));
        };

        document.head.appendChild(script);
    });
}

async function loadMediaPipe() {
    if (mediaPipeLoaded) return;

    const poseUrl =
        "https://cdn.jsdelivr.net/npm/@mediapipe/pose/pose.js";

    const handsUrl =
        "https://cdn.jsdelivr.net/npm/@mediapipe/hands/hands.js";

    await loadScriptOnce(poseUrl, "Pose");
    await loadScriptOnce(handsUrl, "Hands");

    mediaPipeLoaded = true;
}

// -----------------------------
// Start local webcam tracking
// -----------------------------
async function startHandTracking() {
    if (trackingActive) return;

    try {
        if (!navigator.mediaDevices?.getUserMedia) {
            throw new Error(
                "Camera access requires HTTPS or localhost."
            );
        }

        setStatus("trackingStatus", "LOADING MODELS");

        await loadMediaPipe();

        
    operatorStream = await navigator.mediaDevices.getUserMedia({
        audio: false,
        video: {
            width: { ideal: 640 },
            height: { ideal: 480 },
            frameRate: { ideal: 30, max: 30 },
            facingMode: "user"
        }
    });


        if (!operatorVideo || !operatorCanvas) {
            throw new Error(
                "Tracking video/canvas elements are missing."
            );
        }

        operatorVideo.srcObject = operatorStream;
        const cameraTrack = operatorStream.getVideoTracks()[0];

        cameraTrack.addEventListener("ended", () => {
            console.error("[TRACKING] Webcam track ended unexpectedly.");

            if (trackingActive) {
                stopHandTracking();
                setStatus("trackingStatus", "CAMERA DISCONNECTED");
                updateControlButtonState();
            }
        });
        operatorVideo.muted = true;
        operatorVideo.playsInline = true;

        
        // Show the operator webcam in panel 1.
        // The canvas sits above the video to draw tracking landmarks.
        operatorVideo.style.display = "block";
        operatorVideo.style.position = "absolute";
        operatorVideo.style.inset = "0";
        operatorVideo.style.width = "100%";
        operatorVideo.style.height = "100%";
        operatorVideo.style.opacity = "1";
        operatorVideo.style.objectFit = "contain";
        operatorVideo.style.transform = "scaleX(-1)";
        operatorVideo.style.pointerEvents = "none";
        operatorVideo.style.zIndex = "1";

        operatorCanvas.style.position = "absolute";
        operatorCanvas.style.inset = "0";
        operatorCanvas.style.width = "100%";
        operatorCanvas.style.height = "100%";
        operatorCanvas.style.zIndex = "2";
        operatorCanvas.style.pointerEvents = "none";


        await operatorVideo.play();

        // Wait until Droidcam supplies a real, decoded frame.
        await waitForVideoFrames(operatorVideo);

        const videoTrack = operatorStream?.getVideoTracks()[0];

        if (!videoTrack || videoTrack.readyState !== "live") {
            throw new Error("The operator webcam track has ended.");
        }

        console.log("[TRACKING] Camera ready:", {
            width: operatorVideo.videoWidth,
            height: operatorVideo.videoHeight,
            track: videoTrack.label,
            state: videoTrack.readyState
        });

        await initializeMediaPipeModels();

        operatorCanvas.width =
            operatorVideo.videoWidth || 640;

        operatorCanvas.height =
            operatorVideo.videoHeight || 480;

        trackingActive = true;
        trackingLoopRunning = true;

        $("startTrackingButton").disabled = true;
        $("stopTrackingButton").disabled = false;

        setStatus("trackingStatus", "RUNNING LOCALLY");

        startTrackingFrameLoop();
        updateControlButtonState();
        maybeStartControlTransmission();

        console.log("[TRACKING] Local webcam tracking started");

    } catch (error) {
        console.error("[TRACKING] Could not start:", error);

        setStatus("trackingStatus", "ERROR");

        stopHandTracking();

        alert(
            "Could not start hand tracking: " +
            (error.message || error)
        );
    }
}

// -----------------------------
// Initialize Pose and Hands models
// -----------------------------
async function initializeMediaPipeModels() {
    poseModel = new window.Pose({
        locateFile: (file) =>
            `https://cdn.jsdelivr.net/npm/@mediapipe/pose/${file}`
    });

    poseModel.setOptions({
        modelComplexity: 1,
        smoothLandmarks: true,
        enableSegmentation: false,
        minDetectionConfidence: 0.55,
        minTrackingConfidence: 0.55
    });

    poseModel.onResults(handlePoseResults);

    handsModel = new window.Hands({
        locateFile: (file) =>
            `https://cdn.jsdelivr.net/npm/@mediapipe/hands/${file}`
    });

    handsModel.setOptions({
        maxNumHands: 2,
        modelComplexity: 1,
        minDetectionConfidence: 0.55,
        minTrackingConfidence: 0.55
    });

    handsModel.onResults(handleHandResults);
}


async function waitForVideoFrames(video, timeoutMs = 10000) {
    if (!video) {
        throw new Error("Operator video element is missing.");
    }

    // Wait for metadata and a usable video resolution.
    if (video.readyState < HTMLMediaElement.HAVE_METADATA ||
        video.videoWidth < 16 ||
        video.videoHeight < 16) {
        await new Promise((resolve, reject) => {
            const timeout = setTimeout(() => {
                cleanup();
                reject(new Error(
                    "Camera did not provide video frames within 10 seconds."
                ));
            }, timeoutMs);

            const cleanup = () => {
                clearTimeout(timeout);
                video.removeEventListener("loadedmetadata", check);
                video.removeEventListener("loadeddata", check);
                video.removeEventListener("resize", check);
            };

            const check = () => {
                if (
                    video.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA &&
                    video.videoWidth >= 16 &&
                    video.videoHeight >= 16
                ) {
                    cleanup();
                    resolve();
                }
            };

            video.addEventListener("loadedmetadata", check);
            video.addEventListener("loadeddata", check);
            video.addEventListener("resize", check);
            check();
        });
    }

    // Metadata alone is not sufficient: wait for an actual decoded frame.
    if (typeof video.requestVideoFrameCallback === "function") {
        await new Promise((resolve, reject) => {
            const timeout = setTimeout(() => {
                reject(new Error("Camera is not producing decoded frames."));
            }, timeoutMs);

            video.requestVideoFrameCallback(() => {
                clearTimeout(timeout);
                resolve();
            });
        });
    }

    if (video.videoWidth < 16 || video.videoHeight < 16) {
        throw new Error(
            `Invalid camera resolution: ${video.videoWidth}x${video.videoHeight}`
        );
    }
}


// -----------------------------
// Frame processing loop
// -----------------------------
async function startTrackingFrameLoop() {
    if (trackingLoopRunning !== true) return;

    if (
        !trackingActive ||
        !operatorVideo ||
        operatorVideo.readyState < 2
    ) {
        requestAnimationFrame(startTrackingFrameLoop);
        return;
    }

    // Do not start another inference while the previous one
    // is still processing.
    if (!poseBusy) {
        poseBusy = true;

        try {
            await poseModel.send({
                image: operatorVideo
            });
        } catch (error) {
            console.warn("[POSE] Frame processing failed:", error);
        } finally {
            poseBusy = false;
        }
    }

    if (!handsBusy) {
        handsBusy = true;

        try {
            await handsModel.send({
                image: operatorVideo
            });
        } catch (error) {
            console.warn("[HANDS] Frame processing failed:", error);
        } finally {
            handsBusy = false;
        }
    }

    requestAnimationFrame(startTrackingFrameLoop);
}

// -----------------------------
// Pose landmark handling
// -----------------------------
function handlePoseResults(results) {
    if (!trackingActive) return;

    const landmarks = results.poseLandmarks;

    if (!landmarks || landmarks.length < 33) {
        setStatus("trackingStatus", "NO BODY DETECTED");
        drawTrackingCanvas(null, latestHandLandmarks);
        return;
    }

    latestPoseLandmarks = landmarks;
    lastPoseSeen = Date.now();

    setStatus("trackingStatus", "BODY TRACKED");

    calculateArmTargets(landmarks);
    drawTrackingCanvas(landmarks, latestHandLandmarks);
}

// -----------------------------
// Hand landmark handling
// -----------------------------
function handleHandResults(results) {
    if (!trackingActive) return;

    const landmarks = results.multiHandLandmarks || [];
    const handedness = results.multiHandedness || [];

    latestHandLandmarks = landmarks;

    if (landmarks.length > 0) {
        lastHandSeen = Date.now();
    }

    for (let index = 0; index < landmarks.length; index++) {
        const hand = landmarks[index];
        const label =
            handedness[index]?.label?.toLowerCase();

        if (label !== "left" && label !== "right") {
            continue;
        }

        const closed = estimateHandClosed(hand);

        updateGripperState(label, closed);
    }

    drawTrackingCanvas(
        latestPoseLandmarks,
        latestHandLandmarks
    );
}

// -----------------------------
// Math utilities
// -----------------------------
function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function lerp(start, end, alpha) {
    return start + (end - start) * alpha;
}

function radiansToDegrees(radians) {
    return radians * 180 / Math.PI;
}

function distance2D(a, b) {
    const dx = a.x - b.x;
    const dy = a.y - b.y;

    return Math.sqrt(dx * dx + dy * dy);
}

function angleBetween2D(a, b, c) {
    // Angle ABC in degrees.
    const v1 = {
        x: a.x - b.x,
        y: a.y - b.y
    };

    const v2 = {
        x: c.x - b.x,
        y: c.y - b.y
    };

    const mag1 = Math.hypot(v1.x, v1.y);
    const mag2 = Math.hypot(v2.x, v2.y);

    if (mag1 < 1e-6 || mag2 < 1e-6) {
        return 0;
    }

    const dot = v1.x * v2.x + v1.y * v2.y;

    const cosine = clamp(
        dot / (mag1 * mag2),
        -1,
        1
    );

    return radiansToDegrees(Math.acos(cosine));
}

function getLandmark(landmarks, index) {
    const point = landmarks?.[index];

    if (!point || point.visibility < 0.35) {
        return null;
    }

    return point;
}

// -----------------------------
// Approximate arm joint mapping
// -----------------------------
function calculateArmTargets(landmarks) {
    // MediaPipe Pose indices:
    // Left shoulder 11, elbow 13, wrist 15
    // Right shoulder 12, elbow 14, wrist 16
    // Hips 23 and 24 provide a torso reference.

    const leftShoulder = getLandmark(landmarks, 11);
    const leftElbow = getLandmark(landmarks, 13);
    const leftWrist = getLandmark(landmarks, 15);

    const rightShoulder = getLandmark(landmarks, 12);
    const rightElbow = getLandmark(landmarks, 14);
    const rightWrist = getLandmark(landmarks, 16);

    const leftHip = getLandmark(landmarks, 23);
    const rightHip = getLandmark(landmarks, 24);

    if (
        !leftShoulder ||
        !leftElbow ||
        !leftWrist ||
        !rightShoulder ||
        !rightElbow ||
        !rightWrist
    ) {
        setStatus("trackingStatus", "ARM LANDMARKS MISSING");
        return;
    }

    const torsoX = leftHip && rightHip
        ? (leftHip.x + rightHip.x) / 2
        : (leftShoulder.x + rightShoulder.x) / 2;

    const torsoY = leftHip && rightHip
        ? (leftHip.y + rightHip.y) / 2
        : (leftShoulder.y + rightShoulder.y) / 2;

    const leftTarget = mapArmToJoints({
        shoulder: leftShoulder,
        elbow: leftElbow,
        wrist: leftWrist,
        torsoX,
        torsoY,
        side: "left"
    });

    const rightTarget = mapArmToJoints({
        shoulder: rightShoulder,
        elbow: rightElbow,
        wrist: rightWrist,
        torsoX,
        torsoY,
        side: "right"
    });

    latestTargets.left = {
        ...latestTargets.left,
        ...leftTarget,
        gripper: gripperClosedBySide.left ? 100 : 0
    };

    latestTargets.right = {
        ...latestTargets.right,
        ...rightTarget,
        gripper: gripperClosedBySide.right ? 100 : 0
    };

    smoothArmTargets();

    updateJointTargetDisplay(smoothedTargets);
}

// -----------------------------
// Convert tracked human arm to
// approximate SO-101 targets.
// -----------------------------
function mapArmToJoints({
    shoulder,
    elbow,
    wrist,
    torsoX,
    torsoY,
    side
}) {
    const direction = side === "left" ? -1 : 1;

    // Shoulder pan approximation:
    // horizontal displacement of wrist relative to shoulder.
    const horizontal = wrist.x - shoulder.x;

    const shoulderPan = clamp(
        direction * horizontal * 180,
        ...CONTROL.jointLimitsDeg.shoulder_pan
    );

    // Shoulder lift approximation:
    // MediaPipe y increases downward.
    const vertical = shoulder.y - wrist.y;

    const shoulderLift = clamp(
        vertical * 140,
        ...CONTROL.jointLimitsDeg.shoulder_lift
    );

    // Elbow flex approximation from shoulder-elbow-wrist angle.
    const includedAngle = angleBetween2D(
        shoulder,
        elbow,
        wrist
    );

    const elbowFlex = clamp(
        180 - includedAngle,
        ...CONTROL.jointLimitsDeg.elbow_flex
    );

    // Wrist flex approximation from the difference between
    // upper-arm and forearm direction.
    const upperArmAngle = Math.atan2(
        elbow.y - shoulder.y,
        elbow.x - shoulder.x
    );

    const forearmAngle = Math.atan2(
        wrist.y - elbow.y,
        wrist.x - elbow.x
    );

    let wristFlex = radiansToDegrees(
        forearmAngle - upperArmAngle
    );

    // Normalize angle into [-180, 180].
    while (wristFlex > 180) wristFlex -= 360;
    while (wristFlex < -180) wristFlex += 360;

    wristFlex = clamp(
        wristFlex,
        ...CONTROL.jointLimitsDeg.wrist_flex
    );

    // A single 2D camera cannot reliably estimate wrist roll.
    // Keep it neutral until you add depth/rotation estimation.
    const wristRoll = 0;

    return {
        shoulder_pan: shoulderPan,
        shoulder_lift: shoulderLift,
        elbow_flex: elbowFlex,
        wrist_flex: wristFlex,
        wrist_roll: wristRoll
    };
}

// -----------------------------
// Smooth targets and limit
// sudden jumps in angle.
// -----------------------------
function smoothArmTargets() {
    for (const side of ["left", "right"]) {
        for (const name of [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll"
        ]) {
            const desired = Number(
                latestTargets[side][name] || 0
            );

            const previous = Number(
                smoothedTargets[side][name] || 0
            );

            const maxStep = CONTROL.maxJointStepDeg;

            const stepLimited = clamp(
                desired,
                previous - maxStep,
                previous + maxStep
            );

            smoothedTargets[side][name] = lerp(
                previous,
                stepLimited,
                CONTROL.smoothingAlpha
            );
        }

        smoothedTargets[side].gripper =
            latestTargets[side].gripper;
    }
}

// -----------------------------
// Estimate whether the hand is
// clenched or open.
// -----------------------------
function estimateHandClosed(hand) {
    // MediaPipe Hands indices:
    // 0 wrist
    // 4 thumb tip
    // 8 index tip, 12 middle tip,
    // 16 ring tip, 20 pinky tip
    //
    // 5, 9, 13, 17 are the corresponding finger bases.
    // This is a heuristic and should be tuned to camera position.

    if (!hand || hand.length < 21) {
        return null;
    }

    const wrist = hand[0];
    const palmBase = hand[9];

    const palmScale = distance2D(wrist, palmBase);

    if (palmScale < 1e-5) {
        return null;
    }

    const fingerPairs = [
        { tip: 8, base: 5 },
        { tip: 12, base: 9 },
        { tip: 16, base: 13 },
        { tip: 20, base: 17 }
    ];

    let curlSum = 0;
    let count = 0;

    for (const pair of fingerPairs) {
        const tipDistance =
            distance2D(hand[pair.tip], palmBase);

        const baseDistance =
            distance2D(hand[pair.base], palmBase);

        if (baseDistance < 1e-5) continue;

        const ratio = tipDistance / baseDistance;

        // Folded fingertips tend to move toward the palm.
        const curl = clamp(1 - ratio, -1, 1);

        curlSum += curl;
        count++;
    }

    if (count === 0) return null;

    const curlScore = curlSum / count;

    // Hysteresis prevents the gripper flickering around a threshold.
    if (curlScore > 0.18) {
        return true;
    }

    if (curlScore < 0.08) {
        return false;
    }

    return null;
}

// -----------------------------
// Independent left/right gripper
// control with debounce.
// -----------------------------
function updateGripperState(side, closed) {
    if (closed === null) return;

    const now = Date.now();

    if (
        now - lastGripperChangeBySide[side] <
        CONTROL.gripperDebounceMs
    ) {
        return;
    }

    if (gripperClosedBySide[side] === closed) {
        return;
    }

    gripperClosedBySide[side] = closed;
    lastGripperChangeBySide[side] = now;

    latestTargets[side].gripper = closed ? 100 : 0;
    smoothedTargets[side].gripper = closed ? 100 : 0;

    updateJointTargetDisplay(smoothedTargets);

    console.log(
        `[GRIPPER] ${side} hand:`,
        closed ? "CLOSED" : "OPEN"
    );
}

// -----------------------------
// Draw skeleton and hand points
// over the operator preview.
// -----------------------------
function drawTrackingCanvas(poseLandmarks, handLandmarks) {
    if (
        !operatorCanvas ||
        !operatorCtx
    ) {
        return;
    }

    const width = operatorCanvas.width;
    const height = operatorCanvas.height;

    operatorCtx.clearRect(0, 0, width, height);

    // Mirror the visualization for a natural selfie preview.
    operatorCtx.save();
    operatorCtx.translate(width, 0);
    operatorCtx.scale(-1, 1);

    if (poseLandmarks) {
        const links = [
            [11, 13], [13, 15],
            [12, 14], [14, 16],
            [11, 12]
        ];

        operatorCtx.lineWidth = 4;
        operatorCtx.strokeStyle = "#31e6a1";

        for (const [a, b] of links) {
            const p1 = poseLandmarks[a];
            const p2 = poseLandmarks[b];

            if (!p1 || !p2) continue;

            operatorCtx.beginPath();
            operatorCtx.moveTo(
                p1.x * width,
                p1.y * height
            );
            operatorCtx.lineTo(
                p2.x * width,
                p2.y * height
            );
            operatorCtx.stroke();
        }

        for (const index of [11, 12, 13, 14, 15, 16]) {
            const point = poseLandmarks[index];

            if (!point) continue;

            operatorCtx.beginPath();
            operatorCtx.arc(
                point.x * width,
                point.y * height,
                6,
                0,
                Math.PI * 2
            );
            operatorCtx.fillStyle = "#ffcb55";
            operatorCtx.fill();
        }
    }

    if (handLandmarks) {
        for (const hand of handLandmarks) {
            for (const point of hand) {
                operatorCtx.beginPath();
                operatorCtx.arc(
                    point.x * width,
                    point.y * height,
                    3,
                    0,
                    Math.PI * 2
                );
                operatorCtx.fillStyle = "#66b7ff";
                operatorCtx.fill();
            }
        }
    }

    operatorCtx.restore();
}

// -----------------------------
// Stop local tracking safely.
// Does not disconnect the robot.
// -----------------------------
function stopHandTracking() {
    trackingActive = false;
    trackingLoopRunning = false;

    poseBusy = false;
    handsBusy = false;

    if (operatorStream) {
        for (const track of operatorStream.getTracks()) {
            track.stop();
        }

        operatorStream = null;
    }

    if (operatorVideo) {
        operatorVideo.pause();
        operatorVideo.srcObject = null;
    }

    if (operatorCtx && operatorCanvas) {
        operatorCtx.clearRect(
            0,
            0,
            operatorCanvas.width,
            operatorCanvas.height
        );
    }

    latestPoseLandmarks = null;
    latestHandLandmarks = null;

    stopControlTransmission();

    if ($("startTrackingButton")) {
        $("startTrackingButton").disabled = false;
    }

    if ($("stopTrackingButton")) {
        $("stopTrackingButton").disabled = true;
    }

    setStatus("trackingStatus", "STOPPED");
    updateControlButtonState();

    console.log("[TRACKING] Local tracking stopped");
}// ============================================================
// PART 4: GYROSCOPE MODULE + DATA CHANNEL TRANSMISSION
// Bluetooth input is isolated in client/gyroscope.js.
// ============================================================

let jointTransmitTimer = null;
let imuTransmitTimer = null;
let transmissionEnabled = false;
let staleWarningShown = false;

async function connectEsp32Imu() {
    if (!window.GyroscopeInput) {
        alert("gyroscope.js is not loaded. Add it before app.js in index.html.");
        return;
    }

    try {
        await window.GyroscopeInput.connect((sample) => {
            const gx = Number(sample.gx);
            const gy = Number(sample.gy);
            if (!Number.isFinite(gx) || !Number.isFinite(gy)) return;

            latestImu = { gx, gy };
            lastImuSend = Date.now();
            setStatus("imuGx", `${gx.toFixed(2)} °/s`);
            setStatus("imuGy", `${gy.toFixed(2)} °/s`);
            setStatus("imuChannelStatus", "BLUETOOTH IMU CONNECTED");
        });

        $("connectImuButton").disabled = true;
        $("disconnectImuButton").disabled = false;
    } catch (error) {
        console.error("[IMU] Bluetooth connection failed:", error);
        setStatus("imuChannelStatus", "BLUETOOTH ERROR");
        alert(`Could not connect to the IMU: ${error.message || error}`);
    }
}

async function disconnectEsp32Imu() {
    try {
        await window.GyroscopeInput?.disconnect();
    } catch (error) {
        console.warn("[IMU] Disconnect issue:", error);
    }
    lastImuSend = 0;
    if ($("connectImuButton")) $("connectImuButton").disabled = false;
    if ($("disconnectImuButton")) $("disconnectImuButton").disabled = true;
    setStatus("imuChannelStatus", "DISCONNECTED");
    stopControlTransmission();
}

// Begin periodic transmission.
// -----------------------------
function startControlTransmission() {
    if (transmissionEnabled) return;

    transmissionEnabled = true;
    staleWarningShown = false;

    const jointIntervalMs =
        1000 / CONTROL.sendRateHz;

    const imuIntervalMs =
        1000 / CONTROL.imuRateHz;

    jointTransmitTimer = setInterval(
        sendJointTargets,
        jointIntervalMs
    );

    imuTransmitTimer = setInterval(
        sendImuCommand,
        imuIntervalMs
    );

    setStatus(
        "controlTransmissionStatus",
        "TRANSMISSION ENABLED"
    );

    console.log("[CONTROL] Transmission enabled");
}

// -----------------------------
// Stop periodic transmission.
// Does not close the WebRTC peer.
// -----------------------------
function stopControlTransmission() {
    transmissionEnabled = false;

    if (jointTransmitTimer !== null) {
        clearInterval(jointTransmitTimer);
        jointTransmitTimer = null;
    }

    if (imuTransmitTimer !== null) {
        clearInterval(imuTransmitTimer);
        imuTransmitTimer = null;
    }

    setStatus(
        "controlTransmissionStatus",
        "STOPPED"
    );
}

// -----------------------------
// Check whether a DataChannel is
// ready for sending.
// -----------------------------
function channelIsOpen(channel) {
    return channel?.readyState === "open";
}

// -----------------------------
// Send joint targets to robot.
// All angles are in degrees.
// -----------------------------
function sendJointTargets() {
    if (!transmissionEnabled || !trackingActive) {
        return;
    }

    if (!channelIsOpen(jointChannel)) {
        setStatus("jointChannelStatus", "NOT OPEN");
        return;
    }

    const now = Date.now();

    // Never send stale pose estimates.
    if (
        !lastPoseSeen ||
        now - lastPoseSeen > CONTROL.stalePoseMs
    ) {
        if (!staleWarningShown) {
            console.warn(
                "[CONTROL] Pose tracking is stale; joint transmission paused"
            );

            staleWarningShown = true;
        }

        setStatus(
            "controlTransmissionStatus",
            "PAUSED — STALE POSE"
        );

        return;
    }

    staleWarningShown = false;

    const targets = {
        left: { ...smoothedTargets.left },
        right: { ...smoothedTargets.right }
    };

    const packet = {
        type: "joint_targets",
        sequence: ++jointSequence,
        timestamp_ms: now,
        units: "deg",
        source: "browser_hand_tracking",
        arms: {
            left: targets.left,
            right: targets.right
        },
        gripper: {
            left_closed: gripperClosedBySide.left,
            right_closed: gripperClosedBySide.right
        }
    };

    try {
        jointChannel.send(JSON.stringify(packet));

        lastJointSend = now;

        setStatus(
            "controlTransmissionStatus",
            "SENDING JOINT TARGETS"
        );

    } catch (error) {
        console.error("[CONTROL] Joint send failed:", error);
    }
}

// -----------------------------
// Send ESP32 orientation to robot.
// Angles are in radians.
// -----------------------------
function sendImuCommand() {
    if (!transmissionEnabled || !channelIsOpen(imuChannel)) return;

    const now = Date.now();
    if (!lastImuSend || now - lastImuSend > 500) {
        setStatus("imuChannelStatus", "WAITING FOR FRESH IMU");
        return;
    }

    // Phone browser reports orientation angles, not raw angular velocity.
    // Send calibrated relative yaw/pitch in radians for the simulation.
    const packet = {
        type: "head_orientation",
        sequence: ++imuSequence,
        timestamp_ms: now,
        units: "rad",
        orientation: {
            yaw: latestImu.gx * Math.PI / 180,
            pitch: latestImu.gy * Math.PI / 180,
            roll: 0
        },
        source: "phone_device_orientation"
    };

    try {
        imuChannel.send(JSON.stringify(packet));
    } catch (error) {
        console.error("[CONTROL] Gyroscope send failed:", error);
    }
}

// -----------------------------
// Start sending when the channels
// and required input are available.
// -----------------------------
function maybeStartControlTransmission() {
    const channelsReady =
        channelIsOpen(jointChannel) &&
        channelIsOpen(imuChannel);

    if (
        channelsReady &&
        (trackingActive || mobileGyroActive)
    ) {
        startControlTransmission();
    } else {
        updateControlButtonState();
    }
}

// -----------------------------
// Disconnect from robot.
// -----------------------------
function disconnectFromRobot() {
    console.log("[CLIENT] Disconnecting from robot");

    stopHandTracking();
    stopControlTransmission();

    closePeerConnection();

    if (websocket) {
        try {
            websocket.close();
        } catch (error) {
            console.warn("[SIGNAL] Close error:", error);
        }

        websocket = null;
    }

    selectedRobot = null;

    cameraPanel?.classList.add("hidden");
    $("teleopFourPanel")?.classList.add("hidden");
    robotSelection?.classList.remove("hidden");

    setStatus("robotCameraStatus", "WAITING");
    $("robotCameraPlaceholder")?.classList.remove("hidden");
    setStatus("jointChannelStatus", "WAITING");
    setStatus("imuChannelStatus", "WAITING");

    if (remoteVideo) {
        remoteVideo.srcObject = null;
    }

    loadRobots();
}

// -----------------------------
// Ensure the UI reflects the
// current connection state.
// -----------------------------
function updateControlButtonState() {
    const channelsReady =
        channelIsOpen(jointChannel) &&
        channelIsOpen(imuChannel);

    if ($("startTrackingButton")) {
        $("startTrackingButton").disabled =
            trackingActive;
    }

    if ($("stopTrackingButton")) {
        $("stopTrackingButton").disabled =
            !trackingActive;
    }

    if (channelsReady && (trackingActive || mobileGyroActive)) {
        maybeStartControlTransmission();
    } else {
        setStatus(
            "controlTransmissionStatus",
            "WAITING FOR TRACKING + CHANNELS"
        );
    }
}

// -----------------------------
// Update robot-camera status
// whenever the video element
// starts or stops receiving video.
// -----------------------------
if (remoteVideo) {
    remoteVideo.addEventListener("playing", () => {
        setStatus("robotCameraStatus", "LIVE");
    });

    remoteVideo.addEventListener("waiting", () => {
        setStatus("robotCameraStatus", "BUFFERING");
    });

    remoteVideo.addEventListener("emptied", () => {
        setStatus("robotCameraStatus", "WAITING");
    });
}

// -----------------------------
// Re-evaluate transmission when
// the page is hidden.
// -----------------------------
document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
        stopControlTransmission();
    } else {
        maybeStartControlTransmission();
    }
});

// -----------------------------
// Restore saved session if one
// exists when the page is loaded.
// -----------------------------
(async function restoreSession() {
    sessionToken =
        sessionStorage.getItem("session_token");

    username =
        sessionStorage.getItem("username");

    if (sessionToken) {
        if (loggedUser && username) {
            loggedUser.textContent = username;
        }

        showDashboard();
        await loadRobots();
    }
})();