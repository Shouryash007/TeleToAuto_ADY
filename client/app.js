// Main application coordinator.
// Feature implementations live in their own modules.
import { createUi } from "./ui-controller.js";
import { createRobotConnection } from "./robot-connection.js";
import { createHandTracker } from "./hand-tracker.js";
import { createManipulatorController, CONTROL as MANIPULATOR_CONTROL } from "./manipulator-controller.js";
import { createDigitalTwin } from "./digital-twin.js";

const ui = createUi({
    onConnectRobot: connectToRobot,
    onStartTracking: startTracking,
    onStopTracking: stopTracking,
    onConnectImu: connectImu,
    onDisconnectImu: disconnectImu,
    onDisconnectRobot: disconnectRobot
});

const manipulator = createManipulatorController();
let robot = null;
let tracker = null;
let trackingActive = false;
let mobileGyroActive = false;
let latestOrientation = { yaw: 0, pitch: 0, roll: 0 };
let latestOrientationAt = 0;
let jointTimer = null;
let imuTimer = null;

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
                onPose: (landmarks) => {
                    if (landmarks) manipulator.updateFromPose(landmarks);
                    else manipulator.markStale();
                },
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
    latestOrientationAt = 0;
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
    const ready = channelReady("joint") && channelReady("imu");
    if (ready && (trackingActive || mobileGyroActive)) startTransmission();
    else if (!ready || (!trackingActive && !mobileGyroActive)) stopTransmission();
}

function startTransmission() {
    if (jointTimer || imuTimer) return;
    jointTimer = window.setInterval(sendJointTargets, 50);
    imuTimer = window.setInterval(sendOrientation, 1000 / 30);
    setStatus("controlTransmissionStatus", "TRANSMISSION ENABLED");
}

function stopTransmission() {
    if (jointTimer) clearInterval(jointTimer);
    if (imuTimer) clearInterval(imuTimer);
    jointTimer = null;
    imuTimer = null;
    setStatus("controlTransmissionStatus", "STOPPED");
}

function sendJointTargets() {
    if (!trackingActive || !robot || !channelReady("joint")) return;
    const targets = manipulator.getTargets();
    if (!manipulator.isFresh()) {
        setStatus("controlTransmissionStatus", "PAUSED — STALE POSE");
        return;
    }
    robot.sendJointTargets(targets, manipulator.getGripperClosed());
    ui.updateJointTargets(targets);
}

function sendOrientation() {
    if (!robot || !channelReady("imu")) return;
    if (!latestOrientationAt || Date.now() - latestOrientationAt > 500) {
        setStatus("imuChannelStatus", "WAITING FOR FRESH ORIENTATION");
        return;
    }
    robot.sendHeadOrientation(latestOrientation);
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

    latestOrientation = {
        yaw: gx * Math.PI / 180,
        pitch: gy * Math.PI / 180,
        roll: 0
    };
    latestOrientationAt = Date.now();
    mobileGyroActive = true;
    setStatus("imuGx", `${gx.toFixed(1)}°`);
    setStatus("imuGy", `${gy.toFixed(1)}°`);
    setStatus("imuChannelStatus", "PHONE ORIENTATION ACTIVE");
    updateTransmission();
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
