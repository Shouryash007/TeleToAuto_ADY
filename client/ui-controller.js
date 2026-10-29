// ============================================================
// ui-controller.js
//
// Login, robot selection, the four-panel teleoperation console,
// buttons and status text. No robotics calculations here.
//
// User actions are reported through the handlers passed to createUi():
//   onConnectRobot(robotId)
//   onStartTracking() / onStopTracking()
//   onConnectImu()    / onDisconnectImu()
//   onDisconnectRobot()
// ============================================================

import { JOINT_NAMES } from "./manipulator-controller.js";

// Optional deployment overrides (e.g. backend on another origin).
const API_BASE = window.TELEOP_API_BASE || "";

export function createUi(handlers = {}) {
    const $ = (id) => document.getElementById(id);
    const call = (name, ...args) => handlers[name]?.(...args);

    let sessionToken = null;
    let username = null;

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

    // -------------------------
    // Small helpers
    // -------------------------
    function setStatus(id, value) {
        const element = $(id);

        if (element && element.textContent !== value) {
            element.textContent = value;
        }
    }

    function setConnectionStatus(text) {
        if (connectionStatus) connectionStatus.textContent = text;
    }

    function setCameraState(text) {
        if (cameraState) cameraState.textContent = text;
    }

    function showAlert(message) {
        alert(message);
    }

    function showLoginPage() {
        dashboardPage?.classList.add("hidden");
        loginPage?.classList.remove("hidden");
    }

    function showDashboard() {
        loginPage?.classList.add("hidden");
        dashboardPage?.classList.remove("hidden");
        document.getElementById("simulationPanel")?.classList.remove("hidden");
    }

    function getSessionToken() {
        return sessionToken || sessionStorage.getItem("session_token");
    }

    // -------------------------
    // Login
    // -------------------------
    function initLogin() {
        if (!loginForm) return;

        loginForm.addEventListener("submit", async (event) => {
            event.preventDefault();

            if (loginError) loginError.textContent = "";

            const user = $("username")?.value.trim() || "";
            const password = $("password")?.value || "";

            try {
                const response = await fetch(`${API_BASE}/api/login`, {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({ username: user, password })
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

                sessionStorage.setItem("session_token", sessionToken);
                sessionStorage.setItem("username", username);

                if (loggedUser) loggedUser.textContent = username;

                showDashboard();
                document.getElementById("simulationPanel")
                    ?.classList.remove("hidden");
                await loadRobots();

            } catch (error) {
                console.error("[AUTH] Login error:", error);

                if (loginError) {
                    loginError.textContent = "Unable to connect to server";
                }
            }
        });
    }

    // -------------------------
    // Robot list and access
    // -------------------------
    async function loadRobots() {
        if (!robotList) return;

        robotList.innerHTML = "<p>Loading robots...</p>";

        const token = getSessionToken();

        if (!token) {
            sessionStorage.clear();
            showLoginPage();
            return;
        }

        try {
            const response = await fetch(`${API_BASE}/api/robots`, {
                headers: { "Authorization": `Bearer ${token}` }
            });

            if (response.status === 401) {
                sessionStorage.clear();
                sessionToken = null;
                username = null;
                showLoginPage();
                return;
            }

            if (!response.ok) {
                throw new Error(`Robot request failed: ${response.status}`);
            }

            const data = await response.json();
            robotList.innerHTML = "";

            for (const robot of (data.robots || [])) {
                const card = document.createElement("div");
                card.className = "robot-card";

                const icon = document.createElement("div");
                icon.className =
                    "robot-icon " +
                    (robot.busy
                        ? "robot-busy"
                        : robot.online
                            ? "robot-online"
                            : "robot-offline");
                icon.textContent = "●";

                const details = document.createElement("div");
                details.className = "robot-details";

                const name = document.createElement("strong");
                name.textContent = robot.robot_id;

                const state = document.createElement("span");
                state.className = !robot.online
                    ? "offline"
                    : robot.busy
                        ? "busy"
                        : "online";
                state.textContent = robot.online ? "ONLINE" : "OFFLINE";

                details.append(name, state);

                if (robot.online) {
                    const use = document.createElement("span");
                    use.className = robot.busy ? "used-by" : "free";
                    use.textContent = robot.busy
                        ? `USED BY: ${robot.used_by || "another operator"}`
                        : "FREE";

                    details.appendChild(use);
                }

                const button = document.createElement("button");
                button.className =
                    `connect-button ${robot.busy ? "busy-button" : ""}`;
                button.textContent = robot.busy ? "IN USE" : "CONNECT";
                button.disabled = !robot.online || !!robot.busy;

                if (robot.online && !robot.busy) {
                    button.addEventListener("click", () => {
                        call("onConnectRobot", robot.robot_id);
                    });
                }

                card.append(icon, details, button);
                robotList.appendChild(card);
            }

        } catch (error) {
            console.error("[CLIENT] Robot list error:", error);
            robotList.innerHTML = "<p>Failed to load robots.</p>";
        }
    }

    // -------------------------
    // Four-panel console
    // -------------------------
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
                    <div class="teleop-status">
                        ESP32:
                        <span id="imuSerialStatus">NOT CONNECTED</span>
                    </div>

                    <div class="teleop-actions">
                        <button id="connectImuButton" type="button">
                            Connect ESP32
                        </button>
                        <button id="disconnectImuButton" type="button" disabled>
                            Disconnect ESP32
                        </button>
                    </div>

                    <div class="orientation-values">
                        <div>
                            <span>Yaw</span>
                            <strong id="imuYaw">0.000 rad</strong>
                        </div>
                        <div>
                            <span>Pitch</span>
                            <strong id="imuPitch">0.000 rad</strong>
                        </div>
                        <div>
                            <span>Roll</span>
                            <strong id="imuRoll">0.000 rad</strong>
                        </div>
                    </div>

                    <div class="orientation-values">
                        <div>
                            <span>Camera yaw target</span>
                            <strong id="cameraYawTarget">0.0°</strong>
                        </div>
                        <div>
                            <span>Camera pitch target</span>
                            <strong id="cameraPitchTarget">0.0°</strong>
                        </div>
                        <div>
                            <span>Camera roll target</span>
                            <strong id="cameraRollTarget">0.0°</strong>
                        </div>
                    </div>

                    <p class="teleop-note">
                        Orientation values are received from the ESP32.
                        Camera movement depends on the simulation's configured axes.
                    </p>
                </section>

            </div>

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

            // The original fallback container is no longer needed.
            cameraPanel?.querySelector(".video-container")
                ?.classList.add("hidden");
            cameraPanel?.querySelector(".camera-info")
                ?.classList.add("hidden");
        }

        // Operator webcam: visible video with landmarks overlaid.
        const operatorVideo = $("operatorVideo");
        const operatorCanvas = $("operatorCanvas");

        if (operatorVideo) {
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
            operatorVideo.muted = true;
            operatorVideo.autoplay = true;
            operatorVideo.playsInline = true;
        }

        if (operatorCanvas) {
            operatorCanvas.style.position = "absolute";
            operatorCanvas.style.inset = "0";
            operatorCanvas.style.width = "100%";
            operatorCanvas.style.height = "100%";
            operatorCanvas.style.zIndex = "2";
            operatorCanvas.style.pointerEvents = "none";
        }

        $("startTrackingButton")?.addEventListener(
            "click", () => call("onStartTracking")
        );
        $("stopTrackingButton")?.addEventListener(
            "click", () => call("onStopTracking")
        );
        $("connectImuButton")?.addEventListener(
            "click", () => call("onConnectImu")
        );
        $("disconnectImuButton")?.addEventListener(
            "click", () => call("onDisconnectImu")
        );
        $("disconnectRobotButton")?.addEventListener(
            "click", () => call("onDisconnectRobot")
        );

        initializeJointTargetDisplay();
    }

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

    // -------------------------
    // Joint target display
    // -------------------------
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
                    ? "0% closed"
                    : "0.0°";

                row.append(label, value);
                container.appendChild(row);
            }
        }
    }

    function updateJointTargets(targets) {
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

    // -------------------------
    // IMU display
    // -------------------------
    function renderImuReadout(orientation) {
        setStatus("imuYaw", `${orientation.yaw.toFixed(3)} rad`);
        setStatus("imuPitch", `${orientation.pitch.toFixed(3)} rad`);
        setStatus("imuRoll", `${orientation.roll.toFixed(3)} rad`);
    }

    // Camera targets echoed back by the robot (radians -> degrees).
    function renderImuFeedback(data) {
        const orientation = data.orientation || data;

        const show = (id, value) => {
            if (Number.isFinite(Number(value))) {
                setStatus(
                    id,
                    `${(Number(value) * 180 / Math.PI).toFixed(1)}°`
                );
            }
        };

        show("cameraYawTarget", orientation.yaw);
        show("cameraPitchTarget", orientation.pitch);
        show("cameraRollTarget", orientation.roll);
    }

    // -------------------------
    // Button state
    // -------------------------
    function setTrackingActive(active) {
        const start = $("startTrackingButton");
        const stop = $("stopTrackingButton");

        if (start) start.disabled = active;
        if (stop) stop.disabled = !active;
    }

    function setImuConnected(connected) {
        const connect = $("connectImuButton");
        const disconnect = $("disconnectImuButton");

        if (connect) connect.disabled = connected;
        if (disconnect) disconnect.disabled = !connected;
    }

    // -------------------------
    // Robot video
    // -------------------------
    async function attachRobotVideo(stream) {
        if (!remoteVideo) return;

        remoteVideo.srcObject = stream;

        try {
            await remoteVideo.play();

            videoOverlay?.classList.add("hidden");
            $("robotCameraPlaceholder")?.classList.add("hidden");

            setStatus("robotCameraStatus", "LIVE");
            setCameraState("LIVE — ROBOT VIEW");
            setConnectionStatus("WebRTC connected");

        } catch (error) {
            console.warn(
                "[VIDEO] Autoplay failed; user interaction may be required.",
                error
            );
        }
    }

    function clearRobotVideo() {
        if (remoteVideo) remoteVideo.srcObject = null;

        videoOverlay?.classList.remove("hidden");
        $("robotCameraPlaceholder")?.classList.remove("hidden");
        setStatus("robotCameraStatus", "WAITING");
    }

    function initRemoteVideoEvents() {
        if (!remoteVideo) return;

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

    // -------------------------
    // Screen transitions
    // -------------------------
    function showTeleop(robotId) {
        robotSelection?.classList.add("hidden");
        cameraPanel?.classList.remove("hidden");

        ensureOperatorPanel();
        $("teleopFourPanel")?.classList.remove("hidden");

        if (currentRobot) currentRobot.textContent = robotId;

        setConnectionStatus("Connecting...");
        setCameraState("Waiting for robot camera");

        videoOverlay?.classList.remove("hidden");
    }

    function showRobotOffline() {
        setConnectionStatus("Robot disconnected");
        setCameraState("Robot offline");
        clearRobotVideo();
    }

    function showRobotSelection() {
        cameraPanel?.classList.add("hidden");
        $("teleopFourPanel")?.classList.add("hidden");
        robotSelection?.classList.remove("hidden");

        clearRobotVideo();
        setStatus("jointChannelStatus", "WAITING");
        setStatus("imuChannelStatus", "WAITING");

        loadRobots();
    }

    // -------------------------
    // Init
    // -------------------------
    function init() {
        initLogin();
        initRemoteVideoEvents();

        // Restore a saved session if one exists.
        sessionToken = sessionStorage.getItem("session_token");
        username = sessionStorage.getItem("username");

        if (sessionToken) {
            if (loggedUser && username) loggedUser.textContent = username;

            showDashboard();
            loadRobots();
        }
    }

    return {
        init,
        getSessionToken,
        setStatus,
        setConnectionStatus,
        showAlert,

        showTeleop,
        showRobotOffline,
        showRobotSelection,

        getOperatorElements: () => ({
            video: $("operatorVideo"),
            canvas: $("operatorCanvas")
        }),

        updateJointTargets,
        renderImuReadout,
        renderImuFeedback,

        setTrackingActive,
        setImuConnected,

        attachRobotVideo,
        clearRobotVideo
    };
}
