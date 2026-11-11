// ============================================================
// hand-tracker.js — dual-hand tracking only (no MediaPipe Pose)
// MediaPipe Hands supplies 21 landmarks per hand for browser-side IK.
//
// On top of the raw MediaPipe output this file now adds:
//   1. CONFIDENCE FILTERING  - handedness-score gate (with hysteresis), palm-size sanity,
//                              frame-edge clipping, palm-shape consistency, per-landmark
//                              jump detection -> a 0..1 `quality` per hand.
//   2. TEMPORAL TRACKING     - persistent track IDs (nearest-neighbour + velocity prediction),
//                              label voting (MediaPipe's Left/Right flips a lot), 2-frame
//                              confirmation, One-Euro smoothing of every landmark.
//   3. OCCLUSION HANDLING    - when a hand disappears the track "coasts" on its (damped)
//                              velocity with decaying quality, then goes "lost"; jumpy
//                              landmarks (a finger hidden behind the other hand / an object)
//                              are pulled back toward their prediction; on re-acquisition the
//                              track is flagged `reacquired` so the IK can glide instead of snap.
//
// onHands(hands, handedness) keeps its old signature. Each `handedness[i]` is still
// { label, score } but now also carries:
//   quality, pinchQuality, status ("tracked" | "coasting"), trackId, reacquired
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

// ------------------------------------------------------------
// Tunables (mutable on purpose: createHandTracker({ tracking: {...} }) overrides them)
// Landmark coordinates are normalised image units (0..1); "palm size" = |wrist - middle MCP|.
// ------------------------------------------------------------
export const TRACK_CFG = {
    // --- confidence filtering ---
    minHandednessScore: 0.7,   // needed to START a track
    minTrackScore: 0.5,        // needed to keep feeding an existing track (hysteresis)
    minPalmSize: 0.03,         // reject specks / far-away false positives
    maxPalmSize: 0.7,
    edgeMargin: 0.015,         // landmarks closer than this to the image border count as clipped
    maxEdgeFraction: 0.6,      // reject detections that are mostly outside the frame
    shapeTolerance: 0.5,       // mean relative palm-bone error that maps to shape score 0
    jumpGate: 0.8,             // max one-frame landmark move (in palm sizes) vs. prediction
    outlierBlend: 0.3,         // how much of a "jumped" landmark is accepted (0 = ignore, 1 = trust)
    minOutputQuality: 0.15,    // below this the hand is not reported at all

    // --- temporal tracking ---
    maxHands: 2,
    gateRadius: 2.5,           // association gate in palm sizes
    labelMismatchCost: 0.3,    // extra cost when the MediaPipe label disagrees with the track
    confirmFrames: 2,          // detections needed before a new track is reported
    maxSpeed: 3.0,             // clamp on estimated palm speed (image units / s)
    oneEuro: { minCutoff: 1.5, beta: 8, dCutoff: 1.0 },

    // --- occlusion handling ---
    coastMs: 350,              // keep reporting a predicted hand this long after it vanishes
    lostMs: 1200,              // keep the track alive (for re-association) this long
    velDamping: 4.0,           // 1/s exponential velocity damping while coasting
    qualityDecayPerSec: 3.0,   // quality decay while coasting
    reacquireMinMs: 150        // a gap longer than this makes the next detection a "reacquire"
};

const PALM_IDS = [0, 5, 9, 13, 17];
const PALM_BONES = [[0, 5], [0, 9], [0, 13], [0, 17], [5, 9], [9, 13], [13, 17]];

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const dist3 = (a, b) => Math.hypot(a.x - b.x, a.y - b.y, (a.z || 0) - (b.z || 0));

function palmCenter(hand) {
    return {
        x: PALM_IDS.reduce((s, i) => s + hand[i].x, 0) / PALM_IDS.length,
        y: PALM_IDS.reduce((s, i) => s + hand[i].y, 0) / PALM_IDS.length
    };
}

function palmSize(hand) {
    return Math.hypot(hand[0].x - hand[9].x, hand[0].y - hand[9].y);
}

const inFrame = (p, m = TRACK_CFG.edgeMargin) =>
    p.x >= m && p.x <= 1 - m && p.y >= m && p.y <= 1 - m;

// Scale-free description of the (nearly rigid) palm; compared against the track's own history.
function shapeSignature(hand) {
    const ref = Math.max(dist3(hand[0], hand[9]), 1e-5);
    return PALM_BONES.map(([a, b]) => dist3(hand[a], hand[b]) / ref);
}

// MediaPipe can report a second "ghost" detection on the same hand (usually with
// the opposite Left/Right label) when only one hand is visible. If two detections
// overlap heavily, keep only the one with the higher handedness score.
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

// ------------------------------------------------------------
// One-Euro filter: heavy smoothing when still (kills jitter), light smoothing when fast (no lag).
// ------------------------------------------------------------
class OneEuro {
    constructor(cfg) {
        this.cfg = cfg;
        this.reset();
    }
    reset() { this.x = null; this.dx = 0; this.t = 0; }
    static alpha(cutoff, dt) {
        const tau = 1 / (2 * Math.PI * cutoff);
        return 1 / (1 + tau / dt);
    }
    filter(v, tMs) {
        if (this.x === null) {
            this.x = v; this.t = tMs; this.dx = 0;
            return v;
        }
        const dt = Math.max((tMs - this.t) / 1000, 1e-3);
        this.t = tMs;
        const ad = OneEuro.alpha(this.cfg.dCutoff, dt);
        this.dx = ad * ((v - this.x) / dt) + (1 - ad) * this.dx;
        const cutoff = this.cfg.minCutoff + this.cfg.beta * Math.abs(this.dx);
        const a = OneEuro.alpha(cutoff, dt);
        this.x = a * v + (1 - a) * this.x;
        return this.x;
    }
}

// ------------------------------------------------------------
// A persistent hand track.
// ------------------------------------------------------------
class HandTrack {
    constructor(id, now) {
        this.id = id;
        this.filters = Array.from({ length: 21 }, () =>
            [0, 1, 2].map(() => new OneEuro(TRACK_CFG.oneEuro)));
        this.hand = null;
        this.center = { x: 0, y: 0 };
        this.vel = { x: 0, y: 0 };
        this.size = 0.1;
        this.shape = null;
        this.labelVote = 0;          // >= 0 -> "Left", < 0 -> "Right" (MediaPipe convention)
        this.hits = 0;
        this.lastSeen = now;
        this.lastUpdate = now;
        this.quality = 0;
        this.pinchQuality = 0;
        this.score = 0;
        this.status = "tracked";     // tracked | coasting | lost
        this.confirmed = false;
        this.reacquired = false;
    }

    get label() { return this.labelVote >= 0 ? "Left" : "Right"; }

    predictedCenter(now) {
        const dt = clamp((now - this.lastUpdate) / 1000, 0, 0.25);
        return { x: this.center.x + this.vel.x * dt, y: this.center.y + this.vel.y * dt };
    }

    // Fold a validated detection into the track.
    update(det, now) {
        const cfg = TRACK_CFG;
        const dt = clamp((now - this.lastUpdate) / 1000, 0.001, 0.25);
        const wasAway = this.status !== "tracked";
        this.reacquired = this.hits > 0 && wasAway && (now - this.lastSeen) > cfg.reacquireMinMs;

        // After an occlusion the filters hold stale state: restart them on the fresh measurement.
        if (wasAway) for (const f of this.filters) for (const e of f) e.reset();

        const havePrev = !!this.hand && !wasAway;
        const size = Math.max(palmSize(det.hand), 1e-4);
        const gate = cfg.jumpGate * size;
        let outliers = 0, pinchBad = false;

        const hand = det.hand.map((p, i) => {
            let x = p.x, y = p.y;
            const z = p.z || 0;
            if (havePrev) {
                // A landmark that teleports relative to the palm-velocity prediction is almost
                // always an occlusion artefact (finger behind something) or a track swap.
                const prev = this.hand[i];
                const px = prev.x + this.vel.x * dt, py = prev.y + this.vel.y * dt;
                if (Math.hypot(x - px, y - py) > gate) {
                    outliers++;
                    if (i === 4 || i === 8) pinchBad = true;
                    x = px + (x - px) * cfg.outlierBlend;
                    y = py + (y - py) * cfg.outlierBlend;
                }
            }
            const f = this.filters[i];
            return { x: f[0].filter(x, now), y: f[1].filter(y, now), z: f[2].filter(z, now) };
        });

        // Palm-shape consistency vs. this track's own history.
        const sig = shapeSignature(hand);
        let shapeScore = 1;
        if (this.shape) {
            let err = 0;
            for (let k = 0; k < sig.length; k++) err += Math.abs(sig[k] - this.shape[k]) / Math.max(this.shape[k], 1e-3);
            err /= sig.length;
            shapeScore = clamp(1 - err / cfg.shapeTolerance, 0, 1);
            if (shapeScore > 0.7) for (let k = 0; k < sig.length; k++) this.shape[k] += 0.02 * (sig[k] - this.shape[k]);
        } else {
            this.shape = sig;
        }

        const outlierFrac = outliers / 21;
        const geom = (1 - det.edgeFrac) * (0.5 + 0.5 * shapeScore);
        const qNew = clamp(det.score * geom * (1 - outlierFrac), 0, 1);
        this.quality = this.hits === 0 ? qNew : 0.5 * this.quality + 0.5 * qNew;

        const tipsVisible = inFrame(hand[4]) && inFrame(hand[8]);
        this.pinchQuality = (pinchBad || !tipsVisible) ? 0 : this.quality;

        const c = palmCenter(hand);
        if (havePrev) {
            const vx = (c.x - this.center.x) / dt, vy = (c.y - this.center.y) / dt;
            this.vel.x = 0.5 * this.vel.x + 0.5 * vx;
            this.vel.y = 0.5 * this.vel.y + 0.5 * vy;
            const sp = Math.hypot(this.vel.x, this.vel.y);
            if (sp > cfg.maxSpeed) { this.vel.x *= cfg.maxSpeed / sp; this.vel.y *= cfg.maxSpeed / sp; }
        } else {
            this.vel.x = 0; this.vel.y = 0;
        }

        this.center = c;
        this.size = size;
        this.hand = hand;
        this.score = det.score;
        this.lastSeen = now;
        this.lastUpdate = now;
        this.hits++;
        this.status = "tracked";
        if (this.hits >= cfg.confirmFrames) this.confirmed = true;

        const sign = det.label === "Left" ? 1 : -1;
        this.labelVote = clamp(this.labelVote * 0.9 + sign * det.score, -5, 5);
    }

    // No detection this frame: predict, decay confidence, eventually give up.
    miss(now) {
        const cfg = TRACK_CFG;
        if (!this.confirmed) { this.status = "lost"; return; }
        if (now - this.lastSeen <= cfg.coastMs && this.hand) {
            const dt = clamp((now - this.lastUpdate) / 1000, 0, 0.25);
            this.lastUpdate = now;
            const damp = Math.exp(-cfg.velDamping * dt);
            this.vel.x *= damp; this.vel.y *= damp;
            const dx = this.vel.x * dt, dy = this.vel.y * dt;
            this.hand = this.hand.map(p => ({ x: p.x + dx, y: p.y + dy, z: p.z }));
            this.center = { x: this.center.x + dx, y: this.center.y + dy };
            this.quality *= Math.exp(-cfg.qualityDecayPerSec * dt);
            this.pinchQuality = 0;   // never trust fingertips while coasting -> gripper holds
            this.status = "coasting";
        } else {
            this.status = "lost";
        }
    }
}

// ------------------------------------------------------------
// Track manager: detections in, stable filtered hands out.
// ------------------------------------------------------------
export class HandTrackManager {
    constructor(overrides = {}) {
        const { oneEuro, ...rest } = overrides;
        Object.assign(TRACK_CFG, rest);
        if (oneEuro) TRACK_CFG.oneEuro = { ...TRACK_CFG.oneEuro, ...oneEuro };
        this.reset();
    }

    reset() {
        this.tracks = [];
        this.nextId = 1;
    }

    update(rawHands, rawHandedness, now = performance.now()) {
        const cfg = TRACK_CFG;
        const dedup = removeDuplicateHands(rawHands || [], rawHandedness || []);

        // 1) confidence filtering of raw detections
        const dets = [];
        dedup.hands.forEach((hand, i) => {
            if (!hand || hand.length < 21) return;
            const hd = dedup.handedness[i] || {};
            const score = hd.score ?? 0;
            if (score < cfg.minTrackScore) return;
            const size = palmSize(hand);
            if (size < cfg.minPalmSize || size > cfg.maxPalmSize) return;
            const edgeFrac = hand.filter(p => !inFrame(p)).length / hand.length;
            if (edgeFrac > cfg.maxEdgeFraction) return;
            dets.push({
                hand, score, edgeFrac, size,
                label: hd.label === "Right" ? "Right" : "Left",
                center: palmCenter(hand)
            });
        });

        // 2) greedy nearest-neighbour association (existing tracks incl. coasting/lost ones)
        const pairs = [];
        this.tracks.forEach((t, ti) => {
            const pc = t.predictedCenter(now);
            const gate = cfg.gateRadius * (t.status === "tracked" ? 1 : t.status === "coasting" ? 1.5 : 2);
            dets.forEach((d, di) => {
                let cost = Math.hypot(d.center.x - pc.x, d.center.y - pc.y) / Math.max(t.size, d.size);
                if (d.label !== t.label) cost += cfg.labelMismatchCost;
                if (cost <= gate) pairs.push({ ti, di, cost });
            });
        });
        pairs.sort((a, b) => a.cost - b.cost);

        const usedT = new Set(), usedD = new Set();
        for (const p of pairs) {
            if (usedT.has(p.ti) || usedD.has(p.di)) continue;
            usedT.add(p.ti); usedD.add(p.di);
            this.tracks[p.ti].update(dets[p.di], now);
        }

        // 3) occlusion: unmatched tracks coast, then get lost
        this.tracks.forEach((t, ti) => { if (!usedT.has(ti)) t.miss(now); });

        // 4) new tracks (stricter score than for existing ones)
        dets.forEach((d, di) => {
            if (usedD.has(di) || d.score < cfg.minHandednessScore) return;
            if (this.tracks.length >= cfg.maxHands + 1) return;
            const t = new HandTrack(this.nextId++, now);
            t.update(d, now);
            this.tracks.push(t);
        });

        // 5) housekeeping
        this.tracks = this.tracks.filter(t =>
            (t.confirmed || t.status !== "lost") && (now - t.lastSeen) <= cfg.lostMs);
        const live = this.tracks.filter(t => t.confirmed && t.status !== "lost");
        if (live.length > cfg.maxHands) {
            live.sort((a, b) => b.quality * (b.status === "tracked" ? 1 : 0.5) -
                                a.quality * (a.status === "tracked" ? 1 : 0.5));
            const drop = new Set(live.slice(cfg.maxHands));
            this.tracks = this.tracks.filter(t => !drop.has(t));
        }

        // 6) output
        const out = this.tracks.filter(t =>
            t.confirmed && t.status !== "lost" && t.quality >= cfg.minOutputQuality);
        const labels = new Map(out.map(t => [t, t.label]));
        if (out.length === 2 && labels.get(out[0]) === labels.get(out[1])) {
            // Two hands can never share a label. Raw (un-mirrored) frames: MediaPipe's "Left"
            // is the hand at the smaller x, so resolve by position.
            const [a, b] = [...out].sort((p, q) => p.center.x - q.center.x);
            labels.set(a, "Left"); labels.set(b, "Right");
        }

        return {
            hands: out.map(t => t.hand),
            handedness: out.map(t => ({
                label: labels.get(t),
                score: t.quality,
                quality: t.quality,
                pinchQuality: t.pinchQuality,
                detScore: t.score,
                status: t.status,
                trackId: t.id,
                reacquired: t.reacquired
            }))
        };
    }

    getInfo() {
        return this.tracks.map(t => ({
            id: t.id, label: t.label, status: t.status, quality: t.quality,
            confirmed: t.confirmed, ageMs: performance.now() - t.lastSeen
        }));
    }
}

export function createHandTracker({
    onHands,
    onStatus,
    onCameraLost,
    onTracks,              // optional: (trackInfo[]) => void, once per processed frame
    tracking = {},         // overrides for TRACK_CFG
    modelOptions = {}      // overrides for MediaPipe Hands options
} = {}) {

    let stream = null;
    let video = null;
    let canvas = null;
    let ctx = null;

    let active = false;
    let generation = 0;
    let handsModel = null;
    let handsBusy = false;

    const manager = new HandTrackManager(tracking);

    function ensureModel() {

        if (handsModel) return;

        handsModel = new window.Hands({
            locateFile: file => `${HANDS_BASE}/${file}`
        });

        // Detection/tracking thresholds are a little lower than before on purpose: the
        // confidence filter + temporal tracker above do the real gating, and a lower
        // MediaPipe tracking threshold keeps it from dropping a half-occluded hand.
        handsModel.setOptions({
            maxNumHands: 2,
            modelComplexity: 1,
            minDetectionConfidence: 0.6,
            minTrackingConfidence: 0.5,
            ...modelOptions
        });

        handsModel.onResults(handleResults);
    }

    // Receive MediaPipe results, filter/track them and forward to the IK system.
    function handleResults(results) {

        if (!active) return;

        const { hands, handedness } = manager.update(
            results.multiHandLandmarks || [],
            results.multiHandedness || [],
            performance.now()
        );

        // Send tracked landmarks (+ quality metadata) to the dual-hand IK controller.
        onHands?.(hands, handedness);
        onTracks?.(manager.getInfo());

        drawHands(hands, handedness);
    }

    function drawHands(hands, handedness = []) {

        if (!ctx || !canvas) return;

        const w = canvas.width;
        const h = canvas.height;

        ctx.clearRect(0, 0, w, h);

        ctx.save();

        // Mirror landmarks to match the mirrored webcam preview.
        ctx.translate(w, 0);
        ctx.scale(-1, 1);

        hands.forEach((hand, index) => {

            const meta = handedness[index] || {};
            const coasting = meta.status === "coasting";
            const q = clamp(meta.quality ?? 1, 0, 1);

            ctx.globalAlpha = coasting ? 0.55 : 0.45 + 0.55 * q;
            ctx.strokeStyle = coasting
                ? "#ffb454"                                   // occluded -> predicted
                : meta.label === "Right" ? "#66b7ff" : "#42d6a4";
            ctx.fillStyle = ctx.strokeStyle;
            ctx.lineWidth = Math.max(2, w / 320);
            ctx.setLineDash(coasting ? [6, 4] : []);

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

            ctx.setLineDash([]);

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
        manager.reset();

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

        manager.reset();
        onHands?.([], []);
    }

    return {
        start,
        stop,
        isActive: () => active,
        getTracks: () => manager.getInfo()
    };
}
