// ============================================================
// hand-tracker.js
// ============================================================

const POSE_BASE = "https://cdn.jsdelivr.net/npm/@mediapipe/pose";
const HANDS_BASE = "https://cdn.jsdelivr.net/npm/@mediapipe/hands";

let mediaPipeLoaded = false;

function loadScriptOnce(src, globalName) {
    if (window[globalName]) return Promise.resolve();

    return new Promise((resolve, reject) => {
        const existing = [...document.querySelectorAll("script")].find(script => script.src === src);
        if (existing) {
            existing.addEventListener("load", resolve, { once: true });
            existing.addEventListener("error", reject, { once: true });
            return;
        }

        const script = document.createElement("script");
        script.src = src;
        script.async = true;

        script.onload = () => {
            if (window[globalName]) resolve();
            else reject(new Error(`Script loaded but ${globalName} was not found`));
        };
        script.onerror = () => reject(new Error(`Could not load ${src}`));
        document.head.appendChild(script);
    });
}

async function loadMediaPipe() {
    if (mediaPipeLoaded) return;
    await loadScriptOnce(`${POSE_BASE}/pose.js`, "Pose");
    await loadScriptOnce(`${HANDS_BASE}/hands.js`, "Hands");
    mediaPipeLoaded = true;
}

async function waitForVideoFrames(video, timeoutMs = 10000) {
    if (!video) throw new Error("Operator video element is missing.");

    if (video.readyState < HTMLMediaElement.HAVE_METADATA || video.videoWidth < 16 || video.videoHeight < 16) {
        await new Promise((resolve, reject) => {
            const timeout = setTimeout(() => {
                cleanup();
                reject(new Error("Camera did not provide video frames within 10 seconds."));
            }, timeoutMs);

            const cleanup = () => {
                clearTimeout(timeout);
                video.removeEventListener("loadedmetadata", check);
                video.removeEventListener("loadeddata", check);
                video.removeEventListener("resize", check);
            };

            const check = () => {
                if (video.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA && video.videoWidth >= 16 && video.videoHeight >= 16) {
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

    if (typeof video.requestVideoFrameCallback === "function") {
        await new Promise((resolve, reject) => {
            const timeout = setTimeout(() => reject(new Error("Camera is not producing decoded frames.")), timeoutMs);
            video.requestVideoFrameCallback(() => {
                clearTimeout(timeout);
                resolve();
            });
        });
    }
}

export function createHandTracker({
    onPose,
    onHands,
    onStatus,
    onCameraLost,
    onStartGesture,
    onStopGesture
} = {}) {
    let stream = null;
    let video = null;
    let canvas = null;
    let ctx = null;

    let active = false;
    let generation = 0;

    let poseModel = null;
    let handsModel = null;
    let poseBusy = false;
    let handsBusy = false;

    let latestPose = null;
    let latestHands = null;
    let lastGestureTime = 0;

    function ensureModels() {
        if (!poseModel) {
            poseModel = new window.Pose({ locateFile: (file) => `${POSE_BASE}/${file}` });
            poseModel.setOptions({
                modelComplexity: 1, smoothLandmarks: true, enableSegmentation: false,
                minDetectionConfidence: 0.55, minTrackingConfidence: 0.55
            });
            poseModel.onResults(handlePoseResults);
        }

        if (!handsModel) {
            handsModel = new window.Hands({ locateFile: (file) => `${HANDS_BASE}/${file}` });
            handsModel.setOptions({
                maxNumHands: 2, modelComplexity: 1, minDetectionConfidence: 0.55, minTrackingConfidence: 0.55
            });
            handsModel.onResults(handleHandResults);
        }
    }

    function handlePoseResults(results) {
        if (!active) return;
        const landmarks = results.poseLandmarks;
        if (!landmarks || landmarks.length < 33) {
            latestPose = null;
            drawTrackingCanvas(null, latestHands);
            onPose?.(null);
            return;
        }
        latestPose = landmarks;
        onPose?.(landmarks);
        drawTrackingCanvas(landmarks, latestHands);
    }

    function handleHandResults(results) {
        if (!active) return;
        const landmarks = results.multiHandLandmarks || [];
        const handedness = results.multiHandedness || [];
        latestHands = landmarks;

        if (landmarks.length === 2) {
            const now = Date.now();
            if (now - lastGestureTime > 1500) {
                let thumbsUpCount = 0;
                let fistCount = 0;

                for (const hand of landmarks) {
                    const thumbUp = hand[4].y < hand[3].y;
                    const indexFolded = hand[8].y > hand[5].y;
                    const middleFolded = hand[12].y > hand[9].y;
                    const ringFolded = hand[16].y > hand[13].y;
                    const pinkyFolded = hand[20].y > hand[17].y;

                    if (thumbUp && indexFolded && middleFolded && ringFolded) {
                        thumbsUpCount++;
                    } else if (indexFolded && middleFolded && ringFolded && pinkyFolded) {
                        fistCount++;
                    }
                }

                if (thumbsUpCount === 2 && onStartGesture) {
                    onStartGesture();
                    lastGestureTime = now;
                } else if (fistCount === 2 && onStopGesture) {
                    onStopGesture();
                    lastGestureTime = now;
                }
            }
        }

        onHands?.(landmarks, handedness);
        drawTrackingCanvas(latestPose, latestHands);
    }

    async function runFrameLoop(runId) {
        if (runId !== generation || !active) return;
        if (!video || video.readyState < 2) {
            requestAnimationFrame(() => runFrameLoop(runId));
            return;
        }

        if (!poseBusy) {
            poseBusy = true;
            try { await poseModel.send({ image: video }); }
            catch (error) { console.warn("[POSE] Frame processing failed:", error); }
            finally { poseBusy = false; }
        }

        if (runId !== generation || !active) return;

        if (!handsBusy) {
            handsBusy = true;
            try { await handsModel.send({ image: video }); }
            catch (error) { console.warn("[HANDS] Frame processing failed:", error); }
            finally { handsBusy = false; }
        }

        requestAnimationFrame(() => runFrameLoop(runId));
    }

    function drawArmSegment(p1, p2, width, height) {
        if (!p1 || !p2) return;
        const x1 = p1.x * width, y1 = p1.y * height;
        const x2 = p2.x * width, y2 = p2.y * height;

        const dx = x2 - x1;
        const dy = y2 - y1;
        const len = Math.hypot(dx, dy);
        if (len < 1e-3) return;

        const nx = -dy / len;
        const ny = dx / len;
        const offset = 30; // Border width

        // Transparent Center region
        ctx.lineWidth = 40;
        ctx.strokeStyle = "rgba(49, 230, 161, 0.2)";
        ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();

        // 2 Outer boundary lines
        ctx.lineWidth = 2;
        ctx.strokeStyle = "#ff4444";
        ctx.beginPath(); ctx.moveTo(x1 + nx * offset, y1 + ny * offset); ctx.lineTo(x2 + nx * offset, y2 + ny * offset); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(x1 - nx * offset, y1 - ny * offset); ctx.lineTo(x2 - nx * offset, y2 - ny * offset); ctx.stroke();

        // 1 Middle tracking line
        ctx.lineWidth = 3;
        ctx.strokeStyle = "#ffffff";
        ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
    }

    function drawTrackingCanvas(poseLandmarks, handLandmarks) {
        if (!canvas || !ctx) return;
        const width = canvas.width;
        const height = canvas.height;

        ctx.clearRect(0, 0, width, height);
        ctx.save();
        ctx.translate(width, 0);
        ctx.scale(-1, 1);

        if (poseLandmarks) {
            drawArmSegment(poseLandmarks[11], poseLandmarks[13], width, height); // L Shoulder -> L Elbow
            drawArmSegment(poseLandmarks[13], poseLandmarks[15], width, height); // L Elbow -> L Wrist
            drawArmSegment(poseLandmarks[12], poseLandmarks[14], width, height); // R Shoulder -> R Elbow
            drawArmSegment(poseLandmarks[14], poseLandmarks[16], width, height); // R Elbow -> R Wrist

            ctx.lineWidth = 4;
            ctx.strokeStyle = "#31e6a1";
            ctx.beginPath();
            ctx.moveTo(poseLandmarks[11].x * width, poseLandmarks[11].y * height);
            ctx.lineTo(poseLandmarks[12].x * width, poseLandmarks[12].y * height);
            ctx.stroke();
        }

        if (handLandmarks) {
            ctx.strokeStyle = "#66b7ff";
            ctx.lineWidth = 3;
            for (const hand of handLandmarks) {
                const fingerPaths = [[0, 1, 2, 3, 4], [0, 5, 6, 7, 8]];
                for (const path of fingerPaths) {
                    ctx.beginPath();
                    for (let i = 0; i < path.length; i++) {
                        const pt = hand[path[i]];
                        if (i === 0) ctx.moveTo(pt.x * width, pt.y * height);
                        else ctx.lineTo(pt.x * width, pt.y * height);
                    }
                    ctx.stroke();
                }
            }
        }
        ctx.restore();
    }

    async function start(videoElement, canvasElement) {
        if (active) return;
        if (!navigator.mediaDevices?.getUserMedia) throw new Error("Camera access requires HTTPS or localhost.");
        if (!videoElement || !canvasElement) throw new Error("Tracking video/canvas elements are missing.");

        video = videoElement;
        canvas = canvasElement;
        ctx = canvas.getContext("2d");
        poseBusy = false;
        handsBusy = false;

        onStatus?.("LOADING MODELS");
        await loadMediaPipe();

        const cameraStream = await navigator.mediaDevices.getUserMedia({
            audio: false,
            video: { width: { ideal: 640 }, height: { ideal: 480 }, frameRate: { ideal: 30, max: 30 }, facingMode: "user" }
        });

        stream = cameraStream;
        video.srcObject = cameraStream;
        const cameraTrack = cameraStream.getVideoTracks()[0];

        cameraTrack?.addEventListener("ended", () => {
            if (active && stream === cameraStream) onCameraLost?.();
        });

        video.muted = true;
        video.playsInline = true;
        await video.play();
        await waitForVideoFrames(video);

        ensureModels();
        canvas.width = video.videoWidth || 640;
        canvas.height = video.videoHeight || 480;
        active = true;
        latestPose = null;
        latestHands = null;

        runFrameLoop(++generation);
    }

    function stop() {
        active = false;
        generation++;
        if (stream) {
            for (const track of stream.getTracks()) track.stop();
            stream = null;
        }
        if (video) {
            video.pause();
            video.srcObject = null;
        }
        if (ctx && canvas) ctx.clearRect(0, 0, canvas.width, canvas.height);
    }

    return { start, stop, isActive: () => active };
}