// ============================================================
// GLOBAL STATE
// ============================================================

let sessionToken = null;
let username = null;

let websocket = null;
let peerConnection = null;
let encoderChannel = null;

let selectedRobot = null;


// ============================================================
// DOM
// ============================================================

const loginPage =
    document.getElementById("loginPage");

const dashboardPage =
    document.getElementById("dashboardPage");

const loginForm =
    document.getElementById("loginForm");

const loginError =
    document.getElementById("loginError");

const loggedUser =
    document.getElementById("loggedUser");

const robotList =
    document.getElementById("robotList");

const cameraPanel =
    document.getElementById("cameraPanel");

const robotSelection =
    document.getElementById("robotSelection");

const currentRobot =
    document.getElementById("currentRobot");

const connectionStatus =
    document.getElementById("connectionStatus");

const cameraState =
    document.getElementById("cameraState");

const remoteVideo =
    document.getElementById("remoteVideo");

const videoOverlay =
    document.getElementById("videoOverlay");


// ============================================================
// LOGIN
// ============================================================

loginForm.addEventListener(
    "submit",
    async (event) => {

        event.preventDefault();

        loginError.textContent = "";

        const usernameInput =
            document.getElementById(
                "username"
            ).value.trim();

        const passwordInput =
            document.getElementById(
                "password"
            ).value;


        try {

            const response = await fetch(
                "/api/login",
                {
                    method: "POST",

                    headers: {
                        "Content-Type":
                            "application/json"
                    },

                    body: JSON.stringify({
                        username:
                            usernameInput,

                        password:
                            passwordInput
                    })
                }
            );


            const data =
                await response.json();


            if (!data.success) {

                loginError.textContent =
                    data.error ||
                    "Login failed";

                return;
            }


            sessionToken =
                data.token;

            username =
                data.username;


            // Keep only for this browser tab/session
            sessionStorage.setItem(
                "session_token",
                sessionToken
            );

            sessionStorage.setItem(
                "username",
                username
            );


            loggedUser.textContent =
                username;


            showDashboard();

            await loadRobots();

        } catch (error) {

            console.error(
                "Login error:",
                error
            );

            loginError.textContent =
                "Unable to connect to server";
        }

    }
);


// ============================================================
// SHOW DASHBOARD
// ============================================================

function showDashboard() {

    loginPage.classList.add(
        "hidden"
    );

    dashboardPage.classList.remove(
        "hidden"
    );
}


// ============================================================
// LOAD ROBOTS
// ============================================================

async function loadRobots() {

    robotList.innerHTML =
        "<p>Loading robots...</p>";

    // Get the current session token
    const token =
        sessionToken ||
        sessionStorage.getItem("session_token");

    if (!token) {
        console.error("[AUTH] No session token");

        sessionStorage.clear();

        dashboardPage.classList.add("hidden");
        loginPage.classList.remove("hidden");

        return;
    }

    try {

        const response =
            await fetch(
                "/api/robots",
                {
                    method: "GET",
                    headers: {
                        "Authorization":
                            `Bearer ${token}`
                    }
                }
            );

        // --------------------------------------------
        // Session is invalid
        // --------------------------------------------

        if (response.status === 401) {

            console.error(
                "[AUTH] Session invalid or expired"
            );

            sessionStorage.clear();

            sessionToken = null;
            username = null;

            dashboardPage.classList.add(
                "hidden"
            );

            loginPage.classList.remove(
                "hidden"
            );

            return;
        }

        if (!response.ok) {

            throw new Error(
                `HTTP ${response.status}`
            );
        }

        const data =
            await response.json();

        console.log(
            "[CLIENT] Robots received:",
            data
        );

        // --------------------------------------------
        // Display authorized robots
        // --------------------------------------------

        robotList.innerHTML = "";

        for (
            const robot of data.robots
        ) {

            const card =
                document.createElement(
                    "div"
                );

            card.className =
                "robot-card";

                let status;

                if (!robot.online) {
                    status = "OFFLINE";
                } else {
                    status = "ONLINE";
                }

            card.innerHTML = `

            <div class="robot-icon ${
                robot.busy
                    ? "robot-busy"
                    : robot.online
                        ? "robot-online"
                        : "robot-offline"
            }">
                ●
            </div>

                <div class="robot-details">

                    <strong>
                        ${robot.robot_id}
                    </strong>

                    <span
                        class="${
                            !robot.online
                                ? "offline"
                                : robot.busy
                                    ? "busy"
                                    : "online"
                        }"
                    >
                        ${status}
                    </span>

                    ${
                        robot.online
                            ? robot.busy
                                ? `<span class="used-by">
                                    USED BY: ${robot.used_by}
                                </span>`
                                : `<span class="free">
                                    FREE
                                </span>`
                            : ""
                    }

                </div>

                <button
                    class="connect-button ${
                        robot.busy ? "busy-button" : ""
                    }"
                    ${
                        robot.online && !robot.busy
                            ? ""
                            : "disabled"
                    }
                >
                    ${
                        robot.busy
                            ? "IN USE"
                            : "CONNECT"
                    }
                </button>

            `;

            const button =
                card.querySelector(
                    ".connect-button"
                );

            if (robot.online) {

                button.addEventListener(
                    "click",
                    () => {

                        connectToRobot(
                            robot.robot_id
                        );

                    }
                );

            }

            robotList.appendChild(
                card
            );
        }

    } catch (error) {

        console.error(
            "[CLIENT] Robot list error:",
            error
        );

        robotList.innerHTML =
            "<p>Failed to load robots.</p>";
    }
}


// ============================================================
// CONNECT TO ROBOT
// ============================================================

function connectToRobot(
    robotId
) {

    selectedRobot =
        robotId;


    robotSelection.classList.add(
        "hidden"
    );

    cameraPanel.classList.remove(
        "hidden"
    );


    currentRobot.textContent =
        robotId;


    connectionStatus.textContent =
        "Connecting...";


    cameraState.textContent =
        "Starting";


    videoOverlay.classList.remove(
        "hidden"
    );


    // --------------------------------------------------------
    // WebSocket URL
    // --------------------------------------------------------

    const protocol =
        window.location.protocol ===
        "https:"
            ? "wss:"
            : "ws:";


    const websocketUrl =
        `${protocol}//${window.location.host}/ws`;


    websocket =
        new WebSocket(
            websocketUrl
        );


    // --------------------------------------------------------
    // WebSocket opened
    // --------------------------------------------------------

    websocket.onopen = () => {

        console.log(
            "[CLIENT] Signaling connected"
        );


        websocket.send(
            JSON.stringify({

                role: "browser",

                token:
                    sessionToken,

                robot_id:
                    robotId
            })
        );


        console.log(
            "[CLIENT] Browser authenticated"
        );

    };


    // --------------------------------------------------------
    // Signaling messages
    // --------------------------------------------------------

    websocket.onmessage =
        async (event) => {

            const data =
                JSON.parse(
                    event.data
                );


            console.log(
                "[CLIENT] Message:",
                data.type
            );


            // ==============================================
            // CONNECTED
            // ==============================================

            if (
                data.type ===
                "connected"
            ) {

                connectionStatus.textContent =
                    "Signaling connected";

                cameraState.textContent =
                    "Waiting for video";

            }


            // ==============================================
            // SDP OFFER
            // ==============================================

            else if (
                data.type ===
                "offer"
            ) {

                await handleOffer(
                    data
                );

            }


            // ==============================================
            // ROBOT DISCONNECTED
            // ==============================================

            else if (
                data.type ===
                "robot_disconnected"
            ) {

                connectionStatus.textContent =
                    "Robot disconnected";

                cameraState.textContent =
                    "Robot offline";

                closePeerConnection();

            }


            // ==============================================
            // ERROR
            // ==============================================

            else if (
                data.type ===
                "error"
            ) {

                alert(
                    data.message
                );

                disconnectFromRobot();

            }

        };


    websocket.onerror = (
        error
    ) => {

        console.error(
            "[CLIENT] WebSocket error:",
            error
        );

        connectionStatus.textContent =
            "Connection error";

    };


    websocket.onclose = () => {

        console.log(
            "[CLIENT] Signaling disconnected"
        );

    };
}


// ============================================================
// CREATE WEBRTC PEER
// ============================================================

function createPeerConnection() {

    const iceServers = [];


    // --------------------------------------------------------
    // STUN
    // --------------------------------------------------------

    // Replace with your own STUN server
    // when deploying remotely.

    iceServers.push({
        urls:
            "stun:YOUR_STUN_SERVER:3478"
    });


    // --------------------------------------------------------
    // TURN
    // --------------------------------------------------------

    // Replace these with your TURN credentials.
    //
    // For production, use temporary TURN credentials,
    // not permanent credentials embedded in JavaScript.

    iceServers.push({

        urls:
            "turn:YOUR_TURN_SERVER:3478",

        username:
            "YOUR_TURN_USERNAME",

        credential:
            "YOUR_TURN_PASSWORD"

    });


    const pc =
        new RTCPeerConnection({
            iceServers:
                iceServers
        });


    pc.ontrack = (
        event
    ) => {

        console.log(
            "[CLIENT] Video track received"
        );


        if (
            event.streams &&
            event.streams.length > 0
        ) {

            remoteVideo.srcObject =
                event.streams[0];

        } else {

            remoteVideo.srcObject =
                new MediaStream([
                    event.track
                ]);

        }


        remoteVideo.play()
            .then(() => {

                console.log(
                    "[CLIENT] Video playing"
                );

                videoOverlay.classList.add(
                    "hidden"
                );

                connectionStatus.textContent =
                    "WebRTC connected";

                cameraState.textContent =
                    "LIVE";

            })
            .catch(
                console.error
            );
            
            
            
    };

    pc.ondatachannel = (event) => {
        encoderChannel = event.channel;
    
        console.log(
            "[DATA] DataChannel received:",
            encoderChannel.label
        );
    
        encoderChannel.onopen = () => {
            console.log("[DATA] Encoder channel is open");
            onDataChannelOpen();
        };
    
        encoderChannel.onclose = () => {
            console.log("[DATA] Encoder channel is closed");
            onDataChannelClose();
            encoderChannel = null;
        };
    
        encoderChannel.onerror = (error) => {
            console.error("[DATA] Channel error:", error);
            onDataChannelError(error);
        };
    
        encoderChannel.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
    
                console.log(
                    "[DATA] Message from robot:",
                    data
                );
            } catch (error) {
                console.error(
                    "[DATA] Invalid message:",
                    error
                );
            }
        };
    };


    pc.oniceconnectionstatechange =
        () => {

            console.log(
                "[CLIENT] ICE:",
                pc.iceConnectionState
            );


            if (
                pc.iceConnectionState ===
                "connected" ||
                pc.iceConnectionState ===
                "completed"
            ) {

                connectionStatus.textContent =
                    "WebRTC connected";

            }


            if (
                pc.iceConnectionState ===
                "failed"
            ) {

                connectionStatus.textContent =
                    "WebRTC connection failed";

                cameraState.textContent =
                    "Connection failed";

            }

        };


    pc.onconnectionstatechange =
        () => {

            console.log(
                "[CLIENT] WebRTC:",
                pc.connectionState
            );

        };


    return pc;
}


// ============================================================
// HANDLE OFFER
// ============================================================

async function handleOffer(
    data
) {

    console.log(
        "[CLIENT] Received offer"
    );


    closePeerConnection();


    peerConnection =
        createPeerConnection();


    await peerConnection.setRemoteDescription({
        type:
            data.type,

        sdp:
            data.sdp
    });


    console.log(
        "[CLIENT] Remote description set"
    );


    const answer =
        await peerConnection.createAnswer();


    await peerConnection.setLocalDescription(
        answer
    );


    // --------------------------------------------------------
    // Wait for ICE gathering to finish.
    //
    // This prototype uses non-trickle ICE.
    // --------------------------------------------------------

    await waitForIceGathering(
        peerConnection
    );


    websocket.send(
        JSON.stringify({

            type:
                peerConnection
                    .localDescription
                    .type,

            sdp:
                peerConnection
                    .localDescription
                    .sdp

        })
    );


    console.log(
        "[CLIENT] SDP answer sent"
    );

}


// ============================================================
// WAIT FOR ICE
// ============================================================

function waitForIceGathering(
    pc
) {

    return new Promise(
        (resolve) => {

            if (
                pc.iceGatheringState ===
                "complete"
            ) {

                resolve();

                return;
            }


            const check =
                () => {

                    if (
                        pc.iceGatheringState ===
                        "complete"
                    ) {

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

        }
    );
}


// ============================================================
// CLOSE WEBRTC
// ============================================================

function closePeerConnection() {

    if (peerConnection) {

        try {

            peerConnection.close();

        } catch (error) {

            console.error(
                error
            );

        }

        peerConnection =
            null;
    }


    remoteVideo.srcObject =
        null;


    videoOverlay.classList.remove(
        "hidden"
    );

}


// ============================================================
// DISCONNECT
// ============================================================

function disconnectFromRobot() {

    onRobotDisconnected();

    closePeerConnection();


    if (websocket) {

        try {

            websocket.close();

        } catch (error) {}

        websocket =
            null;
    }


    selectedRobot =
        null;


    cameraPanel.classList.add(
        "hidden"
    );

    robotSelection.classList.remove(
        "hidden"
    );


    loadRobots();

}


// ============================================================
// BUTTONS
// ============================================================

document
    .getElementById(
        "disconnectButton"
    )
    .addEventListener(
        "click",
        disconnectFromRobot
    );


document
    .getElementById(
        "logoutButton"
    )
    .addEventListener(
        "click",
        () => {

            disconnectFromRobot();


            sessionStorage.clear();


            sessionToken =
                null;

            username =
                null;


            dashboardPage.classList.add(
                "hidden"
            );

            loginPage.classList.remove(
                "hidden"
            );

        }
    );


// ============================================================
// RESTORE SESSION
// ============================================================

const savedToken =
    sessionStorage.getItem(
        "session_token"
    );

const savedUsername =
    sessionStorage.getItem(
        "username"
    );


if (
    savedToken &&
    savedUsername
) {

    sessionToken =
        savedToken;

    username =
        savedUsername;

    loggedUser.textContent =
        username;

    showDashboard();

    loadRobots();
}
setInterval(() => {

    if (
        sessionToken &&
        !dashboardPage.classList.contains("hidden")
    ) {
        loadRobots();
    }

}, 3000);


window.sendTestEncoderMessage = () => {
    if (
        !encoderChannel ||
        encoderChannel.readyState !== "open"
    ) {
        console.error(
            "[DATA] Encoder channel is not open"
        );
        return;
    }

    encoderChannel.send(JSON.stringify({
        type: "test",
        sequence: 1,
        message: "Hello from operator browser"
    }));

    console.log("[DATA] Test message sent");
};


// ============================================================
// SO-101 LEADER TELEOPERATION & LIVE JOINT VERIFICATION
// ============================================================

const SO101_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper"
];

const SO101_IDS = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper"
};

// Verified defaults from host LeRobot calibration (None.json)
const DEFAULT_CALIBRATION = {
    shoulder_pan:  { id: 1, range_min: 816,  range_max: 3543, drive_mode: 0 },
    shoulder_lift: { id: 2, range_min: 833,  range_max: 3222, drive_mode: 0 },
    elbow_flex:    { id: 3, range_min: 861,  range_max: 3089, drive_mode: 0 },
    wrist_flex:    { id: 4, range_min: 781,  range_max: 3116, drive_mode: 0 },
    wrist_roll:    { id: 5, range_min: 0,    range_max: 4095, drive_mode: 0 },
    gripper:       { id: 6, range_min: 1535, range_max: 2709, drive_mode: 0 }
};

let activeCalibration = JSON.parse(JSON.stringify(DEFAULT_CALIBRATION));

// Serial port state
let leaderPort = null;
let leaderReader = null;
let leaderWriter = null;
let isLeaderConnected = false;
let isTransmitting = false;
let txSequence = 0;
let leaderPollTimer = null;
let staleCheckTimer = null;

let latestPositions = {
    shoulder_pan: null,
    shoulder_lift: null,
    elbow_flex: null,
    wrist_flex: null,
    wrist_roll: null,
    gripper: null
};

let lastDataTimestamp = 0;
let packetCount = 0;
let lastRateTime = performance.now();

// DOM elements
const leaderStatusBadge = document.getElementById("leaderStatusBadge");
const connectLeaderButton = document.getElementById("connectLeaderButton");
const disconnectLeaderButton = document.getElementById("disconnectLeaderButton");
const toggleTransmitButton = document.getElementById("toggleTransmitButton");
const leaderAlert = document.getElementById("leaderAlert");
const dataFreshnessDot = document.getElementById("dataFreshnessDot");
const leaderLastUpdated = document.getElementById("leaderLastUpdated");
const leaderFrequency = document.getElementById("leaderFrequency");
const dcChannelStatus = document.getElementById("dcChannelStatus");
const dcTxStatus = document.getElementById("dcTxStatus");
const dcLastSeq = document.getElementById("dcLastSeq");

const jointDomMap = {};
for (const joint of SO101_JOINTS) {
    jointDomMap[joint] = {
        row: document.getElementById(`row_${joint}`),
        pos: document.getElementById(`pos_${joint}`),
        status: document.getElementById(`status_${joint}`)
    };
}

// ------------------------------------------------------------
// CALIBRATION FETCH
// ------------------------------------------------------------
async function fetchCalibration() {
    const token = sessionToken || sessionStorage.getItem("session_token");
    if (!token) return;

    try {
        const response = await fetch("/api/calibration", {
            headers: {
                "Authorization": `Bearer ${token}`
            }
        });

        if (response.ok) {
            const data = await response.json();
            if (data.success && data.calibration) {
                activeCalibration = data.calibration;
                console.log("[LEADER] Loaded active calibration:", activeCalibration);
            }
        }
    } catch (err) {
        console.warn("[LEADER] Could not fetch server calibration, using verified defaults:", err);
    }
}

// Fetch calibration on session load
fetchCalibration();

// ------------------------------------------------------------
// STATUS & UI HELPERS
// ------------------------------------------------------------
function setLeaderStatus(state, alertMsg = "") {
    if (!leaderStatusBadge) return;

    leaderStatusBadge.className = "status-badge";

    if (state === "Connected") {
        leaderStatusBadge.classList.add("badge-connected");
        leaderStatusBadge.textContent = "Connected";
        connectLeaderButton.disabled = true;
        disconnectLeaderButton.disabled = false;
        if (encoderChannel && encoderChannel.readyState === "open") {
            toggleTransmitButton.disabled = false;
        }
    } else if (state === "Connecting") {
        leaderStatusBadge.classList.add("badge-connecting");
        leaderStatusBadge.textContent = "Connecting...";
        connectLeaderButton.disabled = true;
        disconnectLeaderButton.disabled = true;
        toggleTransmitButton.disabled = true;
        leaderLastUpdated.textContent = "Connecting to leader...";
    } else if (state === "Error") {
        leaderStatusBadge.classList.add("badge-error");
        leaderStatusBadge.textContent = "Error";
        connectLeaderButton.disabled = false;
        disconnectLeaderButton.disabled = true;
        toggleTransmitButton.disabled = true;
        dataFreshnessDot.className = "freshness-dot dot-red";
    } else { // Disconnected
        leaderStatusBadge.classList.add("badge-disconnected");
        leaderStatusBadge.textContent = "Disconnected";
        connectLeaderButton.disabled = false;
        disconnectLeaderButton.disabled = true;
        toggleTransmitButton.disabled = true;
        dataFreshnessDot.className = "freshness-dot dot-gray";
    }

    if (alertMsg) {
        leaderAlert.textContent = alertMsg;
        leaderAlert.classList.remove("hidden");
    } else {
        leaderAlert.textContent = "";
        leaderAlert.classList.add("hidden");
    }
}

function resetJointTable() {
    for (const joint of SO101_JOINTS) {
        latestPositions[joint] = null;
        if (jointDomMap[joint]) {
            jointDomMap[joint].pos.textContent = "--";
            jointDomMap[joint].status.textContent = "Waiting";
            jointDomMap[joint].status.className = "status-cell status-waiting";
        }
    }
    if (dataFreshnessDot) dataFreshnessDot.className = "freshness-dot dot-gray";
    if (leaderLastUpdated) leaderLastUpdated.textContent = "Waiting for leader connection";
    if (leaderFrequency) leaderFrequency.textContent = "--";
}

// ------------------------------------------------------------
// CONNECT LEADER (WEB SERIAL)
// ------------------------------------------------------------
async function connectLeader() {
    if (!("serial" in navigator)) {
        setLeaderStatus("Error", "Web Serial API is not supported in this browser. Please use Chrome, Edge, or Opera over HTTPS.");
        return;
    }

    try {
        setLeaderStatus("Connecting");

        // Browser native USB device permission prompt
        leaderPort = await navigator.serial.requestPort();

        // 1,000,000 baud as verified against Feetech STS3215 hardware
        await leaderPort.open({
            baudRate: 1000000,
            dataBits: 8,
            stopBits: 1,
            parity: "none"
        });

        leaderWriter = leaderPort.writable.getWriter();
        leaderReader = leaderPort.readable.getReader();
        isLeaderConnected = true;

        setLeaderStatus("Connected");
        console.log("[LEADER] Connected to USB serial leader at 1,000,000 baud");

        // Start serial read stream parser & poll timer
        startSerialReader();
        startSerialPoller();
        startStaleChecker();

    } catch (err) {
        console.error("[LEADER] Connect error:", err);
        if (err.name === "NotFoundError") {
            // User cancelled permission dialog
            setLeaderStatus("Disconnected");
        } else {
            setLeaderStatus("Error", `Serial connection failed: ${err.message || err}`);
            await disconnectLeader();
        }
    }
}

// ------------------------------------------------------------
// DISCONNECT LEADER
// ------------------------------------------------------------
async function disconnectLeader() {
    isLeaderConnected = false;

    if (isTransmitting) {
        stopTransmitting();
    }

    if (leaderPollTimer) {
        clearInterval(leaderPollTimer);
        leaderPollTimer = null;
    }

    if (staleCheckTimer) {
        clearInterval(staleCheckTimer);
        staleCheckTimer = null;
    }

    try {
        if (leaderReader) {
            await leaderReader.cancel();
            leaderReader.releaseLock();
            leaderReader = null;
        }
    } catch (e) {
        console.warn("[LEADER] Reader release warning:", e);
    }

    try {
        if (leaderWriter) {
            leaderWriter.releaseLock();
            leaderWriter = null;
        }
    } catch (e) {
        console.warn("[LEADER] Writer release warning:", e);
    }

    try {
        if (leaderPort) {
            await leaderPort.close();
            leaderPort = null;
        }
    } catch (e) {
        console.warn("[LEADER] Port close warning:", e);
    }

    setLeaderStatus("Disconnected");
    resetJointTable();
}

// ------------------------------------------------------------
// SERIAL STREAM READING & PACKET PARSING
// ------------------------------------------------------------
async function startSerialReader() {
    let rxBuf = [];

    try {
        while (isLeaderConnected && leaderPort && leaderPort.readable) {
            const { value, done } = await leaderReader.read();
            if (done) {
                console.log("[LEADER] Serial readable stream closed");
                break;
            }
            if (!value) continue;

            for (let i = 0; i < value.length; i++) {
                rxBuf.push(value[i]);
            }

            // Feetech Protocol 0 packet parser:
            // Look for header 0xFF, 0xFF
            while (rxBuf.length >= 8) {
                let hIdx = -1;
                for (let i = 0; i < rxBuf.length - 1; i++) {
                    if (rxBuf[i] === 0xFF && rxBuf[i + 1] === 0xFF) {
                        hIdx = i;
                        break;
                    }
                }

                if (hIdx === -1) {
                    // Retain trailing 0xFF if present
                    rxBuf = (rxBuf[rxBuf.length - 1] === 0xFF) ? [0xFF] : [];
                    break;
                }

                if (hIdx > 0) {
                    rxBuf.splice(0, hIdx);
                }

                if (rxBuf.length < 8) break;

                const id = rxBuf[2];
                const len = rxBuf[3];
                const err = rxBuf[4];
                const posLo = rxBuf[5];
                const posHi = rxBuf[6];
                const chk = rxBuf[7];

                // Feetech checksum: ~(id + len + err + posLo + posHi) & 0xFF
                const expectedChk = (~(id + len + err + posLo + posHi)) & 0xFF;

                if (chk === expectedChk && len === 4 && SO101_IDS[id]) {
                    const rawWord = posLo | (posHi << 8);

                    // Decode 15-bit sign-magnitude representation (bit 15 = sign)
                    const signBit = (rawWord >> 15) & 1;
                    const mag = rawWord & 0x7FFF;
                    const signedVal = signBit ? -mag : mag;

                    handleMotorReading(id, signedVal);
                    rxBuf.splice(0, 8);
                } else {
                    // Advance sliding window past invalid header
                    rxBuf.shift();
                }
            }
        }
    } catch (err) {
        if (isLeaderConnected) {
            console.error("[LEADER] Read loop error:", err);
            setLeaderStatus("Error", `Serial read error: ${err.message || err}`);
            disconnectLeader();
        }
    }
}

// ------------------------------------------------------------
// POSITION CONVERSION & TABLE UPDATE
// ------------------------------------------------------------
function handleMotorReading(id, rawVal) {
    const jointName = SO101_IDS[id];
    if (!jointName) return;

    const cal = activeCalibration[jointName] || DEFAULT_CALIBRATION[jointName];
    let converted;

    if (jointName === "gripper") {
        // Gripper: 0-100 range normalized from calibrated min and max
        const bounded = Math.min(cal.range_max, Math.max(cal.range_min, rawVal));
        converted = ((bounded - cal.range_min) / (cal.range_max - cal.range_min)) * 100.0;
    } else {
        // Arm joints 1-5: Degrees normalized around midpoint
        const mid = (cal.range_min + cal.range_max) / 2.0;
        converted = (rawVal - mid) * 360.0 / 4095.0;
    }

    latestPositions[jointName] = converted;
    lastDataTimestamp = Date.now();
    packetCount++;

    const dom = jointDomMap[jointName];
    if (dom) {
        dom.pos.textContent = converted.toFixed(2);
        dom.status.textContent = "Active";
        dom.status.className = "status-cell status-active";
    }

    if (dataFreshnessDot) {
        dataFreshnessDot.className = "freshness-dot dot-green";
    }

    const d = new Date();
    const timeStr = `${d.toTimeString().split(" ")[0]}.${String(d.getMilliseconds()).padStart(3, "0")}`;
    if (leaderLastUpdated) {
        leaderLastUpdated.textContent = timeStr;
    }
}

// ------------------------------------------------------------
// SERIAL POLLING LOOP (SYNC READ)
// ------------------------------------------------------------
function startSerialPoller() {
    // Feetech INST_SYNC_READ (0x82) to broadcast 0xFE, start address 56, length 2, IDs 1..6
    // Verified packet: [0xFF, 0xFF, 0xFE, 10, 0x82, 56, 2, 1, 2, 3, 4, 5, 6, 0x26]
    const syncPacket = new Uint8Array([0xFF, 0xFF, 0xFE, 10, 0x82, 56, 2, 1, 2, 3, 4, 5, 6, 0x26]);

    if (leaderPollTimer) clearInterval(leaderPollTimer);

    // Poll at ~30 Hz (33ms interval)
    leaderPollTimer = setInterval(async () => {
        if (!isLeaderConnected || !leaderWriter) return;

        try {
            await leaderWriter.write(syncPacket);
        } catch (err) {
            console.error("[LEADER] Write error:", err);
            setLeaderStatus("Error", `Serial write error: ${err.message || err}`);
            disconnectLeader();
            return;
        }

        // Update read rate telemetry once per second
        const now = performance.now();
        if (now - lastRateTime >= 1000) {
            const elapsed = (now - lastRateTime) / 1000;
            const hz = (packetCount / 6) / elapsed;
            packetCount = 0;
            lastRateTime = now;
            if (leaderFrequency) {
                leaderFrequency.textContent = `${hz.toFixed(1)} Hz`;
            }
        }

        // Transmit over WebRTC DataChannel if operator started sending
        if (isTransmitting && encoderChannel && encoderChannel.readyState === "open") {
            transmitJointPositions();
        }
    }, 35);
}

// ------------------------------------------------------------
// DATACHANNEL TRANSMISSION (REUSE ENCODER-CONTROL)
// ------------------------------------------------------------
function transmitJointPositions() {
    if (!encoderChannel || encoderChannel.readyState !== "open") return;

    // Guard against building up unbounded queues if buffer fills
    if (encoderChannel.bufferedAmount > 65536) {
        console.warn("[DATA] High DataChannel buffer, dropping frame to avoid backlog");
        return;
    }

    // Ensure all 6 joints have been acquired
    for (const joint of SO101_JOINTS) {
        if (latestPositions[joint] === null || latestPositions[joint] === undefined) {
            return;
        }
    }

    const payload = {
        type: "joint_positions",
        sequence: ++txSequence,
        timestamp: Date.now(),
        positions: {
            "shoulder_pan.pos": Number(latestPositions.shoulder_pan.toFixed(2)),
            "shoulder_lift.pos": Number(latestPositions.shoulder_lift.toFixed(2)),
            "elbow_flex.pos": Number(latestPositions.elbow_flex.toFixed(2)),
            "wrist_flex.pos": Number(latestPositions.wrist_flex.toFixed(2)),
            "wrist_roll.pos": Number(latestPositions.wrist_roll.toFixed(2)),
            "gripper.pos": Number(latestPositions.gripper.toFixed(2))
        }
    };

    encoderChannel.send(JSON.stringify(payload));

    if (dcLastSeq) dcLastSeq.textContent = `#${txSequence}`;
    if (dcTxStatus) {
        dcTxStatus.textContent = "Transmitting";
        dcTxStatus.style.color = "#4ade80";
    }
}

function startTransmitting() {
    if (!encoderChannel || encoderChannel.readyState !== "open") {
        alert("Cannot transmit: WebRTC DataChannel is not open yet. Please wait for robot connection.");
        return;
    }
    if (!isLeaderConnected) {
        alert("Cannot transmit: SO-101 Leader arm is not connected.");
        return;
    }

    isTransmitting = true;
    toggleTransmitButton.textContent = "Stop Sending Data";
    toggleTransmitButton.classList.add("transmitting");
    if (dcTxStatus) {
        dcTxStatus.textContent = "Transmitting";
        dcTxStatus.style.color = "#4ade80";
    }
    console.log("[DATA] Started sending joint positions over DataChannel");
}

function stopTransmitting() {
    isTransmitting = false;
    toggleTransmitButton.textContent = "Start Sending Data";
    toggleTransmitButton.classList.remove("transmitting");
    if (dcTxStatus) {
        dcTxStatus.textContent = "Stopped";
        dcTxStatus.style.color = "#94a3b8";
    }
    console.log("[DATA] Stopped sending joint positions over DataChannel");
}

// ------------------------------------------------------------
// STALE DATA DETECTOR
// ------------------------------------------------------------
function startStaleChecker() {
    if (staleCheckTimer) clearInterval(staleCheckTimer);

    staleCheckTimer = setInterval(() => {
        if (!isLeaderConnected || lastDataTimestamp === 0) return;

        const age = Date.now() - lastDataTimestamp;
        if (age > 1000) {
            if (dataFreshnessDot) dataFreshnessDot.className = "freshness-dot dot-amber";
            for (const joint of SO101_JOINTS) {
                if (jointDomMap[joint]) {
                    jointDomMap[joint].status.textContent = "Stale";
                    jointDomMap[joint].status.className = "status-cell status-stale";
                }
            }
        }
        if (age > 3000) {
            if (dataFreshnessDot) dataFreshnessDot.className = "freshness-dot dot-red";
            if (leaderStatusBadge) {
                leaderStatusBadge.textContent = "Connected (Stale)";
                leaderStatusBadge.className = "status-badge badge-connecting";
            }
        }
    }, 300);
}

// ------------------------------------------------------------
// WEBRTC DATACHANNEL CALLBACK INTEGRATION
// ------------------------------------------------------------
function onDataChannelOpen() {
    if (dcChannelStatus) {
        dcChannelStatus.textContent = "Open";
        dcChannelStatus.style.color = "#4ade80";
    }
    if (isLeaderConnected && toggleTransmitButton) {
        toggleTransmitButton.disabled = false;
    }
}

function onDataChannelClose() {
    if (dcChannelStatus) {
        dcChannelStatus.textContent = "Closed";
        dcChannelStatus.style.color = "#f87171";
    }
    if (isTransmitting) {
        stopTransmitting();
    }
    if (toggleTransmitButton) {
        toggleTransmitButton.disabled = true;
    }
}

function onDataChannelError(error) {
    if (dcChannelStatus) {
        dcChannelStatus.textContent = "Error";
        dcChannelStatus.style.color = "#f87171";
    }
}

function onRobotDisconnected() {
    if (isTransmitting) {
        stopTransmitting();
    }
    if (dcChannelStatus) {
        dcChannelStatus.textContent = "Waiting";
        dcChannelStatus.style.color = "";
    }
    if (dcTxStatus) {
        dcTxStatus.textContent = "Idle";
        dcTxStatus.style.color = "";
    }
    if (toggleTransmitButton) {
        toggleTransmitButton.disabled = true;
    }
}

// ------------------------------------------------------------
// HARDWARE USB UNPLUG EVENT LISTENER
// ------------------------------------------------------------
if ("serial" in navigator) {
    navigator.serial.addEventListener("disconnect", (event) => {
        if (leaderPort && event.target === leaderPort) {
            console.warn("[LEADER] Physical USB disconnect detected");
            setLeaderStatus("Error", "USB device was disconnected.");
            disconnectLeader();
        }
    });
}

// ------------------------------------------------------------
// UI BUTTON LISTENERS
// ------------------------------------------------------------
if (connectLeaderButton) {
    connectLeaderButton.addEventListener("click", connectLeader);
}

if (disconnectLeaderButton) {
    disconnectLeaderButton.addEventListener("click", disconnectLeader);
}

if (toggleTransmitButton) {
    toggleTransmitButton.addEventListener("click", () => {
        if (isTransmitting) {
            stopTransmitting();
        } else {
            startTransmitting();
        }
    });
}

