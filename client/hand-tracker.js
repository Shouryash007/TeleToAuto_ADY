
// ============================================================
// hand-tracker.js — dual-hand tracking only (no MediaPipe Pose)
// MediaPipe Hands supplies 21 landmarks per hand for browser-side IK.
// ============================================================

const HANDS_BASE = "https://cdn.jsdelivr.net/npm/@mediapipe/hands";
let mediaPipeLoaded = false;

function loadHandsScript() {
    if (window.Hands) return Promise.resolve();
    if (mediaPipeLoaded) return Promise.resolve();

    return new Promise((resolve, reject) => {
        const src = `${HANDS_BASE}/hands.js`;
        const existing = [...document.querySelectorAll("script")]
            .find(s => s.src === src);

        if (existing) {
            existing.addEventListener("load", resolve, { once: true });
            existing.addEventListener("error", reject, { once: true });
            return;
        }

        const script = document.createElement("script");
        script.src = src;
        script.async = true;

        script.onload = () =>
            window.Hands
                ? resolve()
                : reject(new Error("MediaPipe Hands did not load."));

        script.onerror = () =>
            reject(new Error(`Could not load ${src}`));

        document.head.appendChild(script);

    }).then(() => {
        mediaPipeLoaded = true;
    });
}

async function waitForVideoFrames(video, timeoutMs = 10000) {
    if (!video) {
        throw new Error("Operator video element is missing.");
    }

    if (
        video.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA &&
        video.videoWidth >= 16
    ) {
        return;
    }

    await new Promise((resolve, reject) => {

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

        const timeout = setTimeout(() => {
            cleanup();
            reject(
                new Error(
                    "Camera did not provide video frames within 10 seconds."
                )
            );
        }, timeoutMs);

        video.addEventListener("loadedmetadata", check);
        video.addEventListener("loadeddata", check);
        video.addEventListener("resize", check);

        check();
    });
}

const HAND_CONNECTIONS = [
    [0, 1], [1, 2], [2, 3], [3, 4],
    [0, 5], [5, 6], [6, 7], [7, 8],
    [5, 9], [9, 10], [10, 11], [11, 12],
    [9, 13], [13, 14], [14, 15], [15, 16],
    [13, 17], [17, 18], [18, 19], [19, 20],
    [0, 17]
];

// MediaPipe can report a second "ghost" detection on the same hand (usually with
// the opposite Left/Right label) when only one hand is visible. If two detections
// overlap heavily, keep only the one with the higher handedness score.
function palmCenter(hand) {
    const ids = [0, 5, 9, 13, 17];
    return {
        x: ids.reduce((s, i) => s + hand[i].x, 0) / ids.length,
        y: ids.reduce((s, i) => s + hand[i].y, 0) / ids.length
    };
}

function removeDuplicateHands(hands, handedness) {
    const keep = hands.map(() => true);

    for (let i = 0; i < hands.length; i++) {
        for (let j = i + 1; j < hands.length; j++) {
            if (!keep[i] || !keep[j]) continue;

            const a = palmCenter(hands[i]);
            const b = palmCenter(hands[j]);
            const size = Math.max(
                Math.hypot(hands[i][0].x - hands[i][9].x, hands[i][0].y - hands[i][9].y),
                Math.hypot(hands[j][0].x - hands[j][9].x, hands[j][0].y - hands[j][9].y),
                1e-4
            );

            // Two real hands are never closer than about one palm-length apart.
            if (Math.hypot(a.x - b.x, a.y - b.y) < size * 0.8) {
                const si = handedness[i]?.score ?? 0;
                const sj = handedness[j]?.score ?? 0;
                keep[si >= sj ? j : i] = false;
            }
        }
    }

    return {
        hands: hands.filter((_, i) => keep[i]),
        handedness: handedness.filter((_, i) => keep[i])
    };
}

export function createHandTracker({
    onHands,
    onStatus,
    onCameraLost
} = {}) {

    let stream = null;
    let video = null;
    let canvas = null;
    let ctx = null;

    let active = false;
    let generation = 0;
    let handsModel = null;
    let handsBusy = false;

    function ensureModel() {

        if (handsModel) return;

        handsModel = new window.Hands({
            locateFile: file => `${HANDS_BASE}/${file}`
        });

        handsModel.setOptions({
            maxNumHands: 2,
            modelComplexity: 1,
            minDetectionConfidence: 0.7,
            minTrackingConfidence: 0.6
        });

        handsModel.onResults(handleResults);
    }

    // Receive MediaPipe results and forward landmarks to the IK system.
    function handleResults(results) {

        if (!active) return;

        const { hands, handedness } = removeDuplicateHands(
            results.multiHandLandmarks || [],
            results.multiHandedness || []
        );

        // Send detected landmarks to the dual-hand IK controller.
        onHands?.(hands, handedness);

        // Draw detected hand landmarks.
        drawHands(hands);
    }

    function drawHands(hands) {

        if (!ctx || !canvas) return;

        const w = canvas.width;
        const h = canvas.height;

        ctx.clearRect(0, 0, w, h);

        ctx.save();

        // Mirror landmarks to match the mirrored webcam preview.
        ctx.translate(w, 0);
        ctx.scale(-1, 1);

        hands.forEach((hand, index) => {

            ctx.strokeStyle =
                index === 0 ? "#42d6a4" : "#66b7ff";

            ctx.fillStyle = ctx.strokeStyle;
            ctx.lineWidth = Math.max(2, w / 320);

            for (const [a, b] of HAND_CONNECTIONS) {

                ctx.beginPath();

                ctx.moveTo(
                    hand[a].x * w,
                    hand[a].y * h
                );

                ctx.lineTo(
                    hand[b].x * w,
                    hand[b].y * h
                );

                ctx.stroke();
            }

            for (const point of hand) {

                ctx.beginPath();

                ctx.arc(
                    point.x * w,
                    point.y * h,
                    Math.max(2.5, w / 240),
                    0,
                    Math.PI * 2
                );

                ctx.fill();
            }
        });

        ctx.restore();
    }

    async function runFrameLoop(runId) {

        if (runId !== generation || !active) return;

        if (!video || video.readyState < 2) {
            requestAnimationFrame(() => runFrameLoop(runId));
            return;
        }

        if (!handsBusy) {

            handsBusy = true;

            try {
                await handsModel.send({ image: video });

            } catch (error) {
                console.warn(
                    "[HANDS] Frame processing failed:",
                    error
                );

            } finally {
                handsBusy = false;
            }
        }

        if (runId === generation && active) {
            requestAnimationFrame(() => runFrameLoop(runId));
        }
    }

    async function start(videoElement, canvasElement) {

        if (active) return;

        if (!navigator.mediaDevices?.getUserMedia) {
            throw new Error(
                "Camera access requires HTTPS or localhost."
            );
        }

        if (!videoElement || !canvasElement) {
            throw new Error(
                "Tracking video/canvas elements are missing."
            );
        }

        video = videoElement;
        canvas = canvasElement;
        ctx = canvas.getContext("2d");

        handsBusy = false;

        onStatus?.("LOADING HAND MODEL");

        await loadHandsScript();

        stream = await navigator.mediaDevices.getUserMedia({
            audio: false,
            video: {
                width: { ideal: 640 },
                height: { ideal: 480 },
                frameRate: { ideal: 30, max: 30 },
                facingMode: "user"
            }
        });

        const cameraStream = stream;

        video.srcObject = cameraStream;

        cameraStream.getVideoTracks()[0]?.addEventListener(
            "ended",
            () => {
                if (active && stream === cameraStream) {
                    onCameraLost?.();
                }
            }
        );

        video.muted = true;
        video.playsInline = true;

        await video.play();
        await waitForVideoFrames(video);

        ensureModel();

        canvas.width = video.videoWidth || 640;
        canvas.height = video.videoHeight || 480;

        active = true;

        runFrameLoop(++generation);
    }

    function stop() {

        active = false;
        generation++;

        if (stream) {
            stream.getTracks().forEach(track => track.stop());
            stream = null;
        }

        if (video) {
            video.pause();
            video.srcObject = null;
        }

        if (ctx && canvas) {
            ctx.clearRect(
                0,
                0,
                canvas.width,
                canvas.height
            );
        }

        onHands?.([], []);
    }

    return {
        start,
        stop,
        isActive: () => active
    };
}
