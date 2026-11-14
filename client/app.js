// Main application coordinator.
// Feature implementations live in their own modules.
import { createUi } from "./ui-controller.js";
import { createRobotConnection } from "./robot-connection.js";
import { createHandTracker } from "./hand-tracker.js";
import { createDualHandIKController } from "./dual-hand-ik-controller.js";
import { createDigitalTwin } from "./digital-twin.js";

const ui = createUi({
    onConnectRobot: connectToRobot,
    onStartTracking: startTracking,
    onStopTracking: stopTracking,
    onConnectImu: connectImu,
    onDisconnectImu: disconnectImu,
    onDisconnectRobot: disconnectRobot
});

const manipulator = createDualHandIKController();
let robot = null;
let tracker = null;
let trackingActive = false;
let jointTimer = null;

function setStatus(id, value) {
    ui.setStatus(id, value);
}

function setTrackingStatus(message) {
    setStatus("trackingStatus", message);
}

function connectToRobot(robotId) {
    stopTracking();
    robot?.disconnect();

    robot = createRobotConnection({
        onSignalingStatus: (message) => ui.setConnectionStatus(message),
        onRemoteStream: (stream) => ui.attachRobotVideo(stream),
        onChannelState: (kind, state) => {
            setStatus(kind === "joint" ? "jointChannelStatus" : "imuChannelStatus",
                state === "open" ? "CONNECTED" : "DISCONNECTED");
            updateTransmission();
        },
        onConnectionState: (state) => {
            ui.setConnectionStatus(`WebRTC ${state}`);
            if (state === "failed" || state === "closed") stopTransmission();
        },
        onRobotMessage: (data) => {
            if (data.type === "imu_feedback") ui.renderImuFeedback(data);
            if (data.type === "mobile_imu_readout") updatePhoneImu(data);
        },
        onRobotDisconnected: () => {
            stopTransmission();
            ui.showRobotOffline();
        },
        onError: (message) => ui.showAlert(message)
    });

    ui.showTeleop(robotId);
    robot.connect(robotId, ui.getSessionToken());
}

async function startTracking() {
    if (trackingActive) return;

    const elements = ui.getOperatorElements();
    try {
        if (!tracker) {
            tracker = createHandTracker({
                // New dual-hand pipeline: MediaPipe Hands -> Jacobian/DLS IK.
                // Pose landmarks are deliberately not used for joint calculation.
                onPose: () => {},
                onHands: (hands, handedness) => manipulator.updateFromHands(hands, handedness),
                onStatus: setTrackingStatus,
                onCameraLost: () => {
                    stopTracking();
                    setTrackingStatus("CAMERA LOST");
                },
                onStartGesture: () => startTracking(),
                onStopGesture: () => stopTracking()
            });
        }

        await tracker.start(elements.video, elements.canvas);
        trackingActive = true;
        ui.setTrackingActive(true);
        setTrackingStatus("TRACKING ACTIVE");
        updateTransmission();
    } catch (error) {
        console.error("[TRACKING] Could not start:", error);
        ui.showAlert(error.message || "Could not start hand tracking.");
        setTrackingStatus("ERROR");
    }
}

function stopTracking() {
    tracker?.stop();
    trackingActive = false;
    manipulator.markStale();
    ui.setTrackingActive(false);
    stopTransmission();
    setTrackingStatus("STOPPED");
}

function connectImu() {
    // The old bundled app contained a BLE reader, but this ZIP has no
    // standalone gyroscope module. Avoid pretending the missing module works.
    ui.showAlert("The ESP32/Bluetooth IMU module is not included in this client folder. Phone orientation is handled separately at /mobile.");
}

function disconnectImu() {
    stopTransmission();
    setStatus("imuChannelStatus", "DISCONNECTED");
}

function disconnectRobot() {
    stopTracking();
    robot?.disconnect();
    robot = null;
    ui.clearRobotVideo();
    ui.showRobotSelection();
}

function channelReady(kind) {
    if (!robot) return false;
    return kind === "joint" ? robot.isJointChannelOpen() : robot.isImuChannelOpen();
}

function updateTransmission() {
    const ready = channelReady("joint");
    if (ready && trackingActive) startTransmission();
    else if (!ready || !trackingActive) stopTransmission();
}

function startTransmission() {
    if (jointTimer) return;
    jointTimer = window.setInterval(sendJointTargets, 50);
    setStatus("controlTransmissionStatus", "TRANSMISSION ENABLED");
}

function stopTransmission() {
    if (jointTimer) clearInterval(jointTimer);
    jointTimer = null;
    setStatus("controlTransmissionStatus", "STOPPED");
}

function sendJointTargets() {
    if (!trackingActive || !robot || !channelReady("joint")) return;
    const targets = manipulator.getTargets();
    if (!manipulator.isFresh()) {
        setStatus("controlTransmissionStatus", "PAUSED — STALE POSE");
        return;
    }
    let gripperPercent;
    if (typeof manipulator.getGripperPercent === "function") {
        gripperPercent = manipulator.getGripperPercent();
    } else {
        const closed = manipulator.getGripperClosed();
        gripperPercent = {
            left: closed.left ? 100 : 0,
            right: closed.right ? 100 : 0
        };
    }
    robot.sendJointTargets(targets, gripperPercent);
    ui.updateJointTargets(targets);
}

function handleVisibility() {
    if (document.hidden) stopTransmission();
    else updateTransmission();
}

document.addEventListener("visibilitychange", handleVisibility);
window.addEventListener("beforeunload", () => {
    stopTransmission();
    tracker?.stop();
    robot?.disconnect();
});

// Receive calibrated phone yaw/pitch forwarded by the signaling server.
function updatePhoneImu(data) {
    const gx = Number(data.gx);
    const gy = Number(data.gy);
    if (!Number.isFinite(gx) || !Number.isFinite(gy)) return;

    // The phone sends orientation directly to the robot through the server.
    // This message is only a mirrored laptop readout; do not retransmit it.
    ui.setStatus("imuGx", `${gx.toFixed(1)}°`);
    ui.setStatus("imuGy", `${gy.toFixed(1)}°`);
    ui.setStatus("imuChannelStatus", "PHONE ORIENTATION ACTIVE");
}

ui.init();

// Digital twin is optional on pages that contain its viewport.
try {
    if (document.getElementById("digitalTwinViewport")) {
        createDigitalTwin();
    }
} catch (error) {
    console.warn("[DIGITAL TWIN] Initialization skipped:", error);
}
