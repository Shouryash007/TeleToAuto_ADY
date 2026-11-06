// ============================================================
// robot-connection.js
//
// WebSocket signaling, WebRTC peer connection, DataChannels and
// robot commands. Uses the existing protocol unchanged:
//
//   joint-control channel: {"type":"joint_targets", ...}   (deg)
//   imu-control channel:   {"type":"head_orientation", ...} (rad)
//   robot video track:     robot camera -> browser
//
// No DOM access here. Everything the UI needs is reported through
// the callbacks passed to createRobotConnection():
//
//   onSignalingStatus(text)
//   onRemoteStream(MediaStream)
//   onChannelState(kind, "open" | "closed")     kind: "joint" | "imu"
//   onConnectionState(pcState)
//   onRobotMessage(data, source)                source: "joint" | "imu" | "signaling"
//   onRobotDisconnected()
//   onError(message)                            server-reported error
// ============================================================

export const STREAM_CONFIG = {
    jointRateHz: 20,
    imuRateHz: 30
};

const ICE_GATHER_TIMEOUT_MS = 5000;

function signalingUrl() {
    if (window.TELEOP_SIGNALING_URL) {
        return window.TELEOP_SIGNALING_URL;
    }

    const protocol =
        window.location.protocol === "https:" ? "wss:" : "ws:";

    return `${protocol}//${window.location.host}/ws`;
}

function waitForIceGathering(pc, timeoutMs = ICE_GATHER_TIMEOUT_MS) {
    if (pc.iceGatheringState === "complete") {
        return Promise.resolve();
    }

    return new Promise((resolve) => {
        const finish = () => {
            clearTimeout(timer);
            pc.removeEventListener("icegatheringstatechange", check);
            resolve();
        };

        const check = () => {
            if (pc.iceGatheringState === "complete") finish();
        };

        // Do not hang forever: send whatever candidates we have.
        const timer = setTimeout(() => {
            console.warn("[WEBRTC] ICE gathering timed out; continuing");
            finish();
        }, timeoutMs);

        pc.addEventListener("icegatheringstatechange", check);
    });
}

export function createRobotConnection(handlers = {}) {
    const notify = (name, ...args) => handlers[name]?.(...args);

    let websocket = null;
    let peerConnection = null;
    let jointChannel = null;
    let imuChannel = null;
    let robotId = null;

    // Sequence numbers keep increasing across reconnects.
    let jointSequence = 0;
    let imuSequence = 0;

    // -------------------------
    // WebRTC
    // -------------------------
    function attachChannel(kind, channel) {
        if (kind === "joint") {
            jointChannel = channel;
        } else {
            imuChannel = channel;
        }

        channel.onopen = () => notify("onChannelState", kind, "open");

        channel.onclose = () => {
            // Ignore events from a channel that was already replaced.
            if (kind === "joint" && jointChannel === channel) {
                jointChannel = null;
            }

            if (kind === "imu" && imuChannel === channel) {
                imuChannel = null;
            }

            notify("onChannelState", kind, "closed");
        };

        channel.onerror = (error) => {
            console.error(`[DATA] ${kind} channel error:`, error);
        };

        channel.onmessage = (messageEvent) => {
            let data;

            try {
                data = JSON.parse(messageEvent.data);
            } catch {
                console.warn(
                    `[DATA] Invalid ${kind} message`,
                    messageEvent.data
                );
                return;
            }

            notify("onRobotMessage", data, kind);
        };

        if (channel.readyState === "open") {
            notify("onChannelState", kind, "open");
        }
    }

    function createPeerConnection() {
        // Configure real STUN/TURN services for remote use.
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

        pc.ontrack = (event) => {
            // The only expected video track is the robot's
            // simulated head-camera view.
            const stream =
                event.streams?.[0] ||
                new MediaStream([event.track]);

            notify("onRemoteStream", stream);
        };

        pc.ondatachannel = (event) => {
            const channel = event.channel;

            console.log("[DATA] DataChannel received:", channel.label);

            if (channel.label === "joint-control") {
                attachChannel("joint", channel);
            } else if (channel.label === "imu-control") {
                attachChannel("imu", channel);
            } else {
                console.warn("[DATA] Unknown DataChannel:", channel.label);
                channel.close();
            }
        };

        pc.onconnectionstatechange = () => {
            console.log("[WEBRTC] Connection state:", pc.connectionState);
            notify("onConnectionState", pc.connectionState);
        };

        pc.oniceconnectionstatechange = () => {
            console.log("[WEBRTC] ICE state:", pc.iceConnectionState);
        };

        return pc;
    }

    async function handleOffer(ws, data) {
        closePeerConnection();

        const pc = createPeerConnection();
        peerConnection = pc;

        await pc.setRemoteDescription({
            type: data.type,
            sdp: data.sdp
        });

        // IMPORTANT: do NOT add the operator webcam to this connection.
        // It is processed locally by MediaPipe. The robot creates the
        // joint-control and imu-control channels and offers its camera.

        const answer = await pc.createAnswer();

        await pc.setLocalDescription(answer);
        await waitForIceGathering(pc);

        if (
            peerConnection !== pc ||
            websocket !== ws ||
            ws.readyState !== WebSocket.OPEN
        ) {
            throw new Error("Signaling WebSocket is not open");
        }

        ws.send(JSON.stringify({
            type: pc.localDescription.type,
            sdp: pc.localDescription.sdp
        }));

        console.log("[WEBRTC] SDP answer sent");
    }

    function closePeerConnection() {
        const pc = peerConnection;

        peerConnection = null;
        jointChannel = null;
        imuChannel = null;

        if (pc) {
            try {
                pc.close();
            } catch (error) {
                console.warn(error);
            }
        }
    }

    // -------------------------
    // Signaling
    // -------------------------
    function connect(id, sessionToken) {
        disconnect();

        robotId = id;

        const ws = new WebSocket(signalingUrl());
        websocket = ws;

        ws.onopen = () => {
            ws.send(JSON.stringify({
                role: "browser",
                token: sessionToken,
                robot_id: id
            }));
        };

        ws.onmessage = async (event) => {
            if (websocket !== ws) return;

            let data;

            try {
                data = JSON.parse(event.data);
            } catch {
                console.warn("[SIGNAL] Invalid JSON from server");
                return;
            }

            if (data.type === "connected") {
                notify("onSignalingStatus", "Signaling connected");

            } else if (data.type === "offer") {
                try {
                    await handleOffer(ws, data);
                } catch (error) {
                    console.error("[WEBRTC] Offer handling failed:", error);
                    notify("onSignalingStatus", "WebRTC setup failed");
                }

            } else if (data.type === "robot_disconnected") {
                notify("onSignalingStatus", "Robot disconnected");
                closePeerConnection();
                notify("onRobotDisconnected");

            } else if (data.type === "error") {
                notify("onError", data.message || "Connection error");

            } else if (
                data.type === "joint_feedback" ||
                data.type === "imu_feedback"
            ) {
                notify("onRobotMessage", data, "signaling");
            }
        };

        ws.onerror = (error) => {
            console.error("[SIGNAL] WebSocket error:", error);

            if (websocket === ws) {
                notify("onSignalingStatus", "Signaling error");
            }
        };

        ws.onclose = () => {
            console.log("[SIGNAL] WebSocket closed");
        };
    }

    function disconnect() {
        closePeerConnection();

        const ws = websocket;
        websocket = null;
        robotId = null;

        if (ws) {
            try {
                ws.close();
            } catch (error) {
                console.warn("[SIGNAL] Close error:", error);
            }
        }
    }

    // -------------------------
    // Channel state
    // -------------------------
    function peerUsable() {
        return (
            !!peerConnection &&
            peerConnection.connectionState !== "failed" &&
            peerConnection.connectionState !== "closed"
        );
    }

    const isJointChannelOpen = () =>
        peerUsable() && jointChannel?.readyState === "open";

    const isImuChannelOpen = () =>
        peerUsable() && imuChannel?.readyState === "open";

    // -------------------------
    // Robot commands (existing packet formats)
    // -------------------------

    // arms:    { left: {...5 joints + gripper}, right: {...} }  degrees
    // gripper: { left: bool, right: bool }  (closed)
    function sendJointTargets(arms, gripperClosed) {
        if (!isJointChannelOpen()) return false;

        const packet = {
            type: "joint_targets",
            sequence: ++jointSequence,
            timestamp_ms: Date.now(),
            units: "deg",
            source: "browser_hand_tracking",
            arms: {
                left: arms.left,
                right: arms.right
            },
            gripper: {
                left_closed: gripperClosed.left,
                right_closed: gripperClosed.right
            }
        };

        try {
            jointChannel.send(JSON.stringify(packet));
            return true;
        } catch (error) {
            console.error("[CONTROL] Joint send failed:", error);
            return false;
        }
    }

    // orientation: { yaw, pitch, roll } in radians
    function sendHeadOrientation(orientation) {
        if (!isImuChannelOpen()) return false;

        const packet = {
            type: "head_orientation",
            sequence: ++imuSequence,
            timestamp_ms: Date.now(),
            units: "rad",
            orientation: {
                yaw: orientation.yaw,
                pitch: orientation.pitch,
                roll: orientation.roll
            }
        };

        try {
            imuChannel.send(JSON.stringify(packet));
            return true;
        } catch (error) {
            console.error("[CONTROL] IMU send failed:", error);
            return false;
        }
    }

    return {
        connect,
        disconnect,
        sendJointTargets,
        sendHeadOrientation,
        isJointChannelOpen,
        isImuChannelOpen,
        getRobotId: () => robotId
    };
}
