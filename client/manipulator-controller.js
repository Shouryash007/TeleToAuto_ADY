// ============================================================
// manipulator-controller.js
// ============================================================

export const CONTROL = {
    stalePoseMs: 500,
    maxJointStepDeg: 8,
    smoothingAlpha: 0.35,

    // Proportional Pinch thresholds
    pinchCloseDistance: 0.04, 
    pinchOpenDistance: 0.10, 

    swapHandedness: false,

    jointLimitsDeg: {
        shoulder_pan: [-90, 90],
        shoulder_lift: [-75, 75],
        elbow_flex: [-10, 120],
        wrist_flex: [-90, 90],
        wrist_roll: [-90, 90]
    }
};

export const ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll"
];

export const JOINT_NAMES = [...ARM_JOINTS, "gripper"];

function clamp(value, min, max) { return Math.max(min, Math.min(max, value)); }
function lerp(start, end, alpha) { return start + (end - start) * alpha; }
function radiansToDegrees(radians) { return radians * 180 / Math.PI; }

function distance2D(a, b) {
    const dx = a.x - b.x;
    const dy = a.y - b.y;
    return Math.sqrt(dx * dx + dy * dy);
}

function angleBetween2D(a, b, c) {
    const v1 = { x: a.x - b.x, y: a.y - b.y };
    const v2 = { x: c.x - b.x, y: c.y - b.y };
    const mag1 = Math.hypot(v1.x, v1.y);
    const mag2 = Math.hypot(v2.x, v2.y);
    if (mag1 < 1e-6 || mag2 < 1e-6) return 0;
    const dot = v1.x * v2.x + v1.y * v2.y;
    const cosine = clamp(dot / (mag1 * mag2), -1, 1);
    return radiansToDegrees(Math.acos(cosine));
}

function getLandmark(landmarks, index) {
    const point = landmarks?.[index];
    if (!point || point.visibility < 0.35) return null;
    return point;
}

export function mapArmToJoints({ shoulder, elbow, wrist, side }) {
    const direction = side === "left" ? -1 : 1;
    const horizontal = wrist.x - shoulder.x;
    const shoulderPan = clamp(direction * horizontal * 180, ...CONTROL.jointLimitsDeg.shoulder_pan);
    const vertical = shoulder.y - wrist.y;
    const shoulderLift = clamp(vertical * 140, ...CONTROL.jointLimitsDeg.shoulder_lift);
    const includedAngle = angleBetween2D(shoulder, elbow, wrist);
    const elbowFlex = clamp(180 - includedAngle, ...CONTROL.jointLimitsDeg.elbow_flex);

    const upperArmAngle = Math.atan2(elbow.y - shoulder.y, elbow.x - shoulder.x);
    const forearmAngle = Math.atan2(wrist.y - elbow.y, wrist.x - elbow.x);
    let wristFlex = radiansToDegrees(forearmAngle - upperArmAngle);
    while (wristFlex > 180) wristFlex -= 360;
    while (wristFlex < -180) wristFlex += 360;
    wristFlex = clamp(wristFlex, ...CONTROL.jointLimitsDeg.wrist_flex);

    return { shoulder_pan: shoulderPan, shoulder_lift: shoulderLift, elbow_flex: elbowFlex, wrist_flex: wristFlex, wrist_roll: 0 };
}

export function estimateProportionalGripper(hand) {
    if (!hand || hand.length < 21) return 0;

    const pinchDistance = distance2D(hand[4], hand[8]);
    
    // Map distance to a 0-100 percentage (0 = open, 100 = closed)
    if (pinchDistance <= CONTROL.pinchCloseDistance) return 100;
    if (pinchDistance >= CONTROL.pinchOpenDistance) return 0;
    
    const range = CONTROL.pinchOpenDistance - CONTROL.pinchCloseDistance;
    const offset = pinchDistance - CONTROL.pinchCloseDistance;
    const percent = 100 - ((offset / range) * 100);
    
    return clamp(percent, 0, 100);
}

function neutralTargets() {
    const arm = () => ({ shoulder_pan: 0, shoulder_lift: 0, elbow_flex: 0, wrist_flex: 0, wrist_roll: 0, gripper: 0 });
    return { left: arm(), right: arm() };
}

export function createManipulatorController() {
    const latestTargets = neutralTargets();
    const smoothedTargets = neutralTargets();
    let lastArmUpdate = 0;

    function smoothArmTargets() {
        for (const side of ["left", "right"]) {
            for (const name of ARM_JOINTS) {
                const desired = Number(latestTargets[side][name] || 0);
                const previous = Number(smoothedTargets[side][name] || 0);
                const stepLimited = clamp(desired, previous - CONTROL.maxJointStepDeg, previous + CONTROL.maxJointStepDeg);
                smoothedTargets[side][name] = lerp(previous, stepLimited, CONTROL.smoothingAlpha);
            }
            // Smooth the gripper proportionally
            smoothedTargets[side].gripper = lerp(smoothedTargets[side].gripper, latestTargets[side].gripper, 0.4);
        }
    }

    function updateFromPose(landmarks) {
        if (!landmarks) return false;

        const leftShoulder = getLandmark(landmarks, 11);
        const leftElbow = getLandmark(landmarks, 13);
        const leftWrist = getLandmark(landmarks, 15);
        const rightShoulder = getLandmark(landmarks, 12);
        const rightElbow = getLandmark(landmarks, 14);
        const rightWrist = getLandmark(landmarks, 16);

        if (!leftShoulder || !leftElbow || !leftWrist || !rightShoulder || !rightElbow || !rightWrist) {
            return false;
        }

        const leftTarget = mapArmToJoints({ shoulder: leftShoulder, elbow: leftElbow, wrist: leftWrist, side: "left" });
        const rightTarget = mapArmToJoints({ shoulder: rightShoulder, elbow: rightElbow, wrist: rightWrist, side: "right" });

        latestTargets.left = { ...latestTargets.left, ...leftTarget, gripper: latestTargets.left.gripper };
        latestTargets.right = { ...latestTargets.right, ...rightTarget, gripper: latestTargets.right.gripper };

        smoothArmTargets();
        lastArmUpdate = Date.now();
        return true;
    }

    function updateFromHands(hands, handedness) {
        let changed = false;

        for (let index = 0; index < hands.length; index++) {
            let label = handedness[index]?.label?.toLowerCase();
            if (label !== "left" && label !== "right") continue;
            if (CONTROL.swapHandedness) label = label === "left" ? "right" : "left";

            latestTargets[label].gripper = estimateProportionalGripper(hands[index]);
            changed = true;
        }
        return changed;
    }

    return {
        updateFromPose,
        updateFromHands,
        getTargets: () => JSON.parse(JSON.stringify(smoothedTargets)),
        getGripperClosed: () => ({ left: smoothedTargets.left.gripper > 50, right: smoothedTargets.right.gripper > 50 }),
        isFresh: (now = Date.now()) => lastArmUpdate > 0 && now - lastArmUpdate <= CONTROL.stalePoseMs,
        markStale: () => { lastArmUpdate = 0; }
    };
}