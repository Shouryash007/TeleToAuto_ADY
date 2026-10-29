// client/digital-twin.js
// Three.js + URDF Loader digital twin.
// Simulation-only: never sends commands to the physical robot.

import * as THREE from "three";
import URDFLoader from "urdf-loader";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const CONFIG = {
    // Actual URDF file path served by backend.
    urdfPath: "/static/ABHI_mobile_humnoid.urdf",

    // Must match the package name used in package:// mesh URLs.
    packageName: "ABHI_mobile_humnoid_description",
    packageRoot: "/static",

    // Actual movable joint names from generated URDF (static/ABHI_mobile_humnoid.urdf).
    joints: {
        right: [
            "Revolute 23",
            "Revolute 24",
            "Revolute 25",
            "Revolute 26",
            "Revolute 27",
            "Revolute 28",
        ],
        left: [
            "Revolute 32",
            "Revolute 33",
            "Revolute 34",
            "Revolute 35",
            "Revolute 36",
            "Revolute 37",
        ],
    },

    // Safe initial UI range in degrees.
    defaultMin: -180,
    defaultMax: 180,
};

export function createDigitalTwin({
    viewportId = "digitalTwinViewport",
    controlsId = "digitalTwinControls",
    statusId = "digitalTwinStatus",
    resetButtonId = "resetDigitalTwin",
} = {}) {
    const viewport = document.getElementById(viewportId);
    const controlsPanel = document.getElementById(controlsId);
    const status = document.getElementById(statusId);
    const resetBtn = document.getElementById(resetButtonId);

    if (!viewport) {
        throw new Error(`Digital twin viewport #${viewportId} was not found.`);
    }

    let robot = null;
    let animationId = null;
    let disposed = false;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x111827);

    const initialWidth = Math.max(viewport.clientWidth, 300);
    const initialHeight = Math.max(viewport.clientHeight, 300);

    const camera = new THREE.PerspectiveCamera(
        45,
        initialWidth / initialHeight,
        0.01,
        100
    );

    camera.position.set(2.2, 2.0, 2.6);

    const renderer = new THREE.WebGLRenderer({
        antialias: true,
        alpha: false,
    });

    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setSize(initialWidth, initialHeight);
    renderer.outputColorSpace = THREE.SRGBColorSpace;

    viewport.replaceChildren(renderer.domElement);
    renderer.domElement.style.display = "block";
    renderer.domElement.style.width = "100%";
    renderer.domElement.style.height = "100%";

    const orbit = new OrbitControls(camera, renderer.domElement);
    orbit.enableDamping = true;
    orbit.dampingFactor = 0.05;
    orbit.target.set(0, 0.4, 0);

    scene.add(new THREE.HemisphereLight(0xffffff, 0x475569, 2.2));

    const keyLight = new THREE.DirectionalLight(0xffffff, 2.5);
    keyLight.position.set(4, 6, 5);
    scene.add(keyLight);

    const fillLight = new THREE.DirectionalLight(0xffffff, 1.2);
    fillLight.position.set(-4, 3, -3);
    scene.add(fillLight);

    const backLight = new THREE.DirectionalLight(0xffffff, 1.0);
    backLight.position.set(0, -3, -4);
    scene.add(backLight);

    const grid = new THREE.GridHelper(4, 40, 0x64748b, 0x334155);
    grid.position.y = 0;
    scene.add(grid);

    const axes = new THREE.AxesHelper(0.35);
    scene.add(axes);

    function setStatus(message) {
        if (status) status.textContent = message;
        console.info(`[DIGITAL TWIN] ${message}`);
    }

    function resize() {
        if (disposed) return;

        const width = viewport.clientWidth;
        const height = viewport.clientHeight;

        if (width <= 0 || height <= 0) return;

        camera.aspect = width / height;
        camera.updateProjectionMatrix();
        renderer.setSize(width, height);
    }

    const resizeObserver = new ResizeObserver(() => {
        resize();
    });
    resizeObserver.observe(viewport);

    window.addEventListener("resize", resize);

    function frameRobot(model) {
        if (!model) return;
        model.updateMatrixWorld(true);

        const bounds = new THREE.Box3().setFromObject(model);
        if (bounds.isEmpty()) return;

        const center = bounds.getCenter(new THREE.Vector3());
        const size = bounds.getSize(new THREE.Vector3());
        const largest = Math.max(size.x, size.y, size.z, 0.5);

        orbit.target.copy(center);
        camera.position.set(
            center.x + largest * 1.5,
            center.y + largest * 1.0,
            center.z + largest * 1.8
        );

        camera.near = Math.max(largest / 1000, 0.001);
        camera.far = largest * 100;
        camera.updateProjectionMatrix();
        orbit.update();
    }

    function onResetClick() {
        if (robot) {
            frameRobot(robot);
            setStatus("Digital twin view reset.");
        }
    }

    if (resetBtn) {
        resetBtn.addEventListener("click", onResetClick);
    }

    function setJointDegrees(jointName, degrees) {
        if (!robot) {
            throw new Error("Robot model has not loaded yet.");
        }

        const joint = robot.joints[jointName];

        if (!joint) {
            throw new Error(`Joint "${jointName}" was not found in the URDF.`);
        }

        if (joint.jointType === "fixed") {
            throw new Error(`Joint "${jointName}" is fixed.`);
        }

        if (!Number.isFinite(degrees)) {
            throw new Error("Joint target must be a finite number.");
        }

        // URDF Loader expects radians for revolute / continuous joints.
        joint.setJointValue(THREE.MathUtils.degToRad(degrees));
    }

    function buildJointControls() {
        if (!controlsPanel) return;

        controlsPanel.replaceChildren();

        for (const [side, names] of Object.entries(CONFIG.joints)) {
            const section = document.createElement("section");
            section.className = "twin-arm-controls";

            const heading = document.createElement("h3");
            heading.textContent =
                side === "right" ? "Right arm" : "Left arm";
            section.appendChild(heading);

            names.forEach((jointName, index) => {
                const row = document.createElement("div");
                row.className = "twin-joint-row";

                const label = document.createElement("label");
                const sliderId = `twin-${side}-${index + 1}`;
                label.htmlFor = sliderId;
                label.textContent = `Motor ${index + 1} · ${jointName}`;

                const valueLabel = document.createElement("output");
                valueLabel.textContent = "0°";

                const slider = document.createElement("input");
                slider.type = "range";
                slider.id = sliderId;
                slider.min = String(CONFIG.defaultMin);
                slider.max = String(CONFIG.defaultMax);
                slider.step = "1";
                slider.value = "0";
                slider.disabled = !robot;

                slider.addEventListener("input", () => {
                    try {
                        const degrees = Number(slider.value);
                        setJointDegrees(jointName, degrees);
                        valueLabel.textContent = `${degrees}°`;
                    } catch (error) {
                        setStatus(error.message);
                    }
                });

                const labelRow = document.createElement("div");
                labelRow.className = "twin-joint-label";
                labelRow.append(label, valueLabel);

                row.append(labelRow, slider);
                section.appendChild(row);
            });

            controlsPanel.appendChild(section);
        }
    }

    function enableControls(enabled) {
        if (!controlsPanel) return;
        controlsPanel.querySelectorAll("input[type=range]").forEach(input => {
            input.disabled = !enabled;
        });
    }

    function resolveMeshUrl(path) {
        if (!path) return path;
        // Extract the filename from file://, package://, or relative paths
        const match = path.match(/([^/\\?#]+\.(?:stl|dae))(?:\?.*)?$/i);
        if (match) {
            return `/static/meshes/${match[1]}`;
        }
        return path;
    }

    function loadRobot() {
        setStatus("Loading URDF and meshes...");

        const manager = new THREE.LoadingManager();
        manager.setURLModifier((url) => {
            if (url.endsWith(".urdf")) return url;
            return resolveMeshUrl(url);
        });

        const loader = new URDFLoader(manager);

        loader.packages = {
            [CONFIG.packageName]: CONFIG.packageRoot,
            "ABHI_mobile_humnoid_description": "/static",
            "static": "/static",
        };

        loader.loadMeshCb = (meshPath, loadingManager, material, done) => {
            const resolvedUrl = resolveMeshUrl(meshPath);
            loader.defaultMeshLoader(resolvedUrl, loadingManager, material, (obj, err) => {
                if (err) {
                    console.error(`[DIGITAL TWIN] Failed to load mesh "${meshPath}" (resolved: "${resolvedUrl}"):`, err);
                    setStatus(`Failed to load mesh: ${meshPath}`);
                }
                done(obj, err);
            });
        };

        loader.load(
            CONFIG.urdfPath,
            (loadedRobot) => {
                if (disposed) return;

                robot = loadedRobot;
                // Rotate to match Three.js Y-up coordinate system from ROS Z-up.
                robot.rotation.x = -Math.PI / 2;
                scene.add(robot);

                const actualJoints = Object.keys(robot.joints);
                console.info("[DIGITAL TWIN] Loaded joints:", actualJoints);

                const configuredNames = [
                    ...CONFIG.joints.right,
                    ...CONFIG.joints.left,
                ];

                const missing = configuredNames.filter(
                    name => !robot.joints[name]
                );

                if (missing.length) {
                    setStatus(
                        `Model loaded, but configured joints are missing: ` +
                        missing.join(", ") +
                        ". Check the browser console and URDF joint names."
                    );
                    enableControls(false);
                    frameRobot(robot);
                    return;
                }

                buildJointControls();
                enableControls(true);
                resize();
                frameRobot(robot);
                setStatus("Robot loaded. Simulation-only controls enabled.");
            },
            (event) => {
                if (event && event.total > 0) {
                    const percent = Math.round(
                        (event.loaded / event.total) * 100
                    );
                    setStatus(`Loading robot: ${percent}%`);
                }
            },
            (error) => {
                console.error("[DIGITAL TWIN] URDF loading failed:", error);
                setStatus(
                    `Failed to load robot at "${CONFIG.urdfPath}". Check URDF URL, mesh URLs, package mapping, and browser Network errors.`
                );
                enableControls(false);
            }
        );
    }

    function animate() {
        if (disposed) return;

        animationId = requestAnimationFrame(animate);
        orbit.update();
        renderer.render(scene, camera);
    }

    buildJointControls();
    loadRobot();
    animate();

    return {
        // Supply angles in degrees from simulation or test.
        setJointDegrees,

        setJointTargets(targets) {
            for (const [jointName, degrees] of Object.entries(targets)) {
                setJointDegrees(jointName, degrees);
            }
        },

        getJointNames() {
            return robot ? Object.keys(robot.joints) : [];
        },

        frameRobot: () => frameRobot(robot),

        dispose() {
            disposed = true;
            if (animationId !== null) cancelAnimationFrame(animationId);

            window.removeEventListener("resize", resize);
            if (resetBtn) {
                resetBtn.removeEventListener("click", onResetClick);
            }

            resizeObserver.disconnect();
            orbit.dispose();
            renderer.dispose();

            if (robot) scene.remove(robot);
            viewport.replaceChildren();
        },
    };
}

// Automatically initialize if the required viewport exists.
let digitalTwin = null;

function initializeDigitalTwin() {
    if (digitalTwin) return;

    if (!document.getElementById("digitalTwinViewport")) return;

    try {
        digitalTwin = createDigitalTwin();
        window.digitalTwin = digitalTwin;
    } catch (error) {
        console.error("[DIGITAL TWIN] Initialization failed:", error);
        const statusEl = document.getElementById("digitalTwinStatus");
        if (statusEl) {
            statusEl.textContent = `Initialization failed: ${error.message}`;
        }
    }
}

if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", initializeDigitalTwin, {
        once: true,
    });
} else {
    initializeDigitalTwin();
}