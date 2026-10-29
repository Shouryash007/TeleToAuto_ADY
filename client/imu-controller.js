// ============================================================
// imu-controller.js
//
// ESP32 + MPU6050 head orientation over Web Serial.
// Independent of hand tracking and of the robot connection.
//
// The firmware (arduino_code.cpp) streams newline-delimited JSON:
//   {"type":"imu","yaw":-12.45,"pitch":3.21,"roll":0.15,"timestamp_ms":10243}
// with angles in DEGREES. The robot protocol uses RADIANS, so this
// module converts, and wraps angles into [-PI, PI] (yaw is integrated
// from the gyro and can run past +-360 deg).
// ============================================================

export const IMU_CONFIG = {
    baudRate: 115200,
    maxBufferLength: 2048,

    // Units the ESP32 sends. Set to "rad" if the firmware is changed.
    serialAngleUnits: "deg",

    // "imu" is what the firmware sends; "imu_orientation" is accepted
    // too. Messages with no type are accepted. Anything else is ignored.
    acceptedTypes: ["imu", "imu_orientation"],

    // Reject garbage (anything beyond 10 full turns).
    maxAbsAngleRad: 20 * Math.PI,

    // Orientation older than this is not forwarded to the robot.
    staleMs: 500
};

function toRadians(value) {
    return IMU_CONFIG.serialAngleUnits === "deg"
        ? value * Math.PI / 180
        : value;
}

function wrapPi(angle) {
    return Math.atan2(Math.sin(angle), Math.cos(angle));
}

// -----------------------------
// Parse one serial line. Pure function (exported for testing).
// Returns { orientation: {yaw,pitch,roll} } in radians,
// { ignored: true }, or { error: "..." }.
// -----------------------------
export function parseImuLine(line) {
    let message;

    try {
        message = JSON.parse(line);
    } catch {
        return { error: "invalid-json" };
    }

    if (!message || typeof message !== "object") {
        return { error: "invalid-json" };
    }

    if (
        message.type &&
        !IMU_CONFIG.acceptedTypes.includes(message.type)
    ) {
        return { ignored: true };
    }

    const yaw = toRadians(Number(message.yaw));
    const pitch = toRadians(Number(message.pitch));
    const roll = toRadians(Number(message.roll));

    if (
        !Number.isFinite(yaw) ||
        !Number.isFinite(pitch) ||
        !Number.isFinite(roll)
    ) {
        return { error: "missing-values" };
    }

    const limit = IMU_CONFIG.maxAbsAngleRad;

    if (
        Math.abs(yaw) > limit ||
        Math.abs(pitch) > limit ||
        Math.abs(roll) > limit
    ) {
        return { error: "out-of-range" };
    }

    return {
        orientation: {
            yaw: wrapPi(yaw),
            pitch: wrapPi(pitch),
            roll: wrapPi(roll)
        }
    };
}

// -----------------------------
// Controller
//   onStatus(text)
//   onOrientation({yaw,pitch,roll})   radians
//   onConnectionChange(connected)
// -----------------------------
export function createImuController({
    onStatus,
    onOrientation,
    onConnectionChange
} = {}) {
    let port = null;
    let reader = null;
    let readActive = false;
    let readLoopPromise = null;
    let buffer = "";

    let latest = { yaw: 0, pitch: 0, roll: 0 };
    let lastSeen = 0;

    function handleLine(line) {
        const result = parseImuLine(line);

        if (result.ignored) return;

        if (result.error) {
            console.warn(`[IMU] Discarded line (${result.error}):`, line);
            return;
        }

        latest = result.orientation;
        lastSeen = Date.now();

        onOrientation?.(latest);
        onStatus?.("IMU DATA RECEIVED");
    }

    async function readLoop() {
        let localReader = null;

        try {
            localReader = port.readable.getReader();
            reader = localReader;

            const decoder = new TextDecoder();

            while (readActive) {
                const { value, done } = await localReader.read();

                if (done) break;
                if (!value) continue;

                buffer += decoder.decode(value, { stream: true });

                let newlineIndex;

                while ((newlineIndex = buffer.indexOf("\n")) >= 0) {
                    const line = buffer.slice(0, newlineIndex).trim();
                    buffer = buffer.slice(newlineIndex + 1);

                    if (line) handleLine(line);
                }

                // Only the unterminated remainder counts against the limit.
                if (buffer.length > IMU_CONFIG.maxBufferLength) {
                    console.warn("[IMU] Oversized serial line discarded");
                    buffer = "";
                }
            }

        } catch (error) {
            if (readActive) {
                console.error("[IMU] Serial read error:", error);
                onStatus?.("READ ERROR");
            }

        } finally {
            if (localReader) {
                try {
                    localReader.releaseLock();
                } catch (_) {
                    // Already released.
                }
            }

            if (reader === localReader) {
                reader = null;
            }
        }

        // The loop ended without disconnect() being called
        // (device unplugged or stream closed).
        if (readActive) {
            readActive = false;
            await closePort();
            lastSeen = 0;
            onStatus?.("DEVICE DISCONNECTED");
            onConnectionChange?.(false);
        }
    }

    async function closePort() {
        const current = port;
        port = null;

        if (!current) return;

        try {
            await current.close();
        } catch (error) {
            console.warn("[IMU] Port close:", error);
        }
    }

    async function connect() {
        if (!("serial" in navigator)) {
            throw new Error(
                "Web Serial is unavailable in this browser. " +
                "Use a supported Chromium browser over HTTPS or localhost."
            );
        }

        if (port) return;

        try {
            port = await navigator.serial.requestPort();

            await port.open({ baudRate: IMU_CONFIG.baudRate });

            readActive = true;
            buffer = "";
            lastSeen = 0;

            onStatus?.("ESP32 CONNECTED — WAITING FOR DATA");
            onConnectionChange?.(true);
            console.log("[IMU] Serial port opened");

            readLoopPromise = readLoop();

        } catch (error) {
            await disconnect();
            onStatus?.("SERIAL ERROR");
            throw error;
        }
    }

    async function disconnect() {
        readActive = false;

        if (reader) {
            try {
                await reader.cancel();
            } catch (error) {
                console.warn("[IMU] Reader cancellation:", error);
            }
        }

        if (readLoopPromise) {
            try {
                await readLoopPromise;
            } catch (_) {
                // Errors are already handled inside readLoop.
            }

            readLoopPromise = null;
        }

        await closePort();

        reader = null;
        buffer = "";
        lastSeen = 0;

        onStatus?.("DISCONNECTED");
        onConnectionChange?.(false);

        console.log("[IMU] ESP32 disconnected");
    }

    return {
        connect,
        disconnect,
        isConnected: () => !!port && readActive,

        // Latest orientation (radians) or null if nothing fresh.
        getFreshOrientation: (now = Date.now()) =>
            lastSeen > 0 && now - lastSeen <= IMU_CONFIG.staleMs
                ? { ...latest }
                : null
    };
}
