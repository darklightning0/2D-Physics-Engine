// AntiGravity Sandbox Application Controller

let world = null;
let canvas = null;
let ctx = null;

// UI State
let isPlaying = true;
let timeScale = 1.0;
let solverIterations = 5;
let currentGravity = { x: 0, y: 9.81 }; // default Earth gravity
const PIXELS_PER_METER = 60.0; // 1 meter = 60 pixels

let showGrid = true;
let selectedObj = null;
let selectedJoint = null;
let selectedJointType = ""; // distance, spring, gear

// Interaction modes
let interactionMode = "select"; // select, spawn-circle, spawn-rect, spawn-poly, spawn-gear, link-distance, link-spring, link-gear
let linkStartObject = null;

// Dragging variables
let isDraggingBody = false;
let dragOffset = { x: 0, y: 0 };
let wasStaticBeforeDrag = false;

// Right-click object fling variables
let isSlingshotDraggingObj = false;
let slingshotObjStartPos = null;
let slingshotCurrent = null;

// Camera / Pan state
let cameraX = 0;
let cameraY = 0;
let isPanning = false;
let panStartMouse = { x: 0, y: 0 };
let panStartCamera = { x: 0, y: 0 };

// Spawner Settings
const spawnerSettings = {
    mass: 1.5,
    restitution: 0.2,
    friction: 0.6,
    color: "#3b82f6",
    isStatic: false,
    gearTeeth: 16,
    gearPitch: 0.15,
    gearMotor: false,
    gearMotorSpeed: 2.0
};

// Emscripten runtime initialization check/hook
if (window.wasmReady) {
    initApp();
} else {
    window.initApp = initApp;
    Module.onRuntimeInitialized = () => {
        initApp();
    };
}

function initApp() {
    canvas = document.getElementById("physics-canvas");
    ctx = canvas.getContext("2d");

    // Initialize the WebAssembly World with default gravity scaled to pixels/sec^2
    world = new Module.World(new Module.Vector(currentGravity.x * PIXELS_PER_METER, currentGravity.y * PIXELS_PER_METER));

    // Handle canvas sizing
    resizeCanvas();
    
    // Resize observer ensures backing store matches physical coordinates perfectly
    const observer = new ResizeObserver(() => {
        resizeCanvas();
        if (!isPlaying) {
            render();
        }
    });
    observer.observe(canvas.parentElement);

    // Populate initial default scene
    loadGearsScene();

    // Bind UI controls
    setupEventHandlers();

    // Start simulation/render loop
    requestAnimationFrame(loop);
}

function resizeCanvas() {
    canvas.width = canvas.parentElement.clientWidth;
    canvas.height = canvas.parentElement.clientHeight;
}

// ==========================================
// Setup Input Listeners
// ==========================================
function setupEventHandlers() {
    // Collapsible Sidebar Listeners
    const panelWorld = document.getElementById("panel-world");
    const btnWorldToggle = document.getElementById("btn-world-toggle");
    
    btnWorldToggle.addEventListener("click", () => {
        const collapsed = panelWorld.classList.toggle("collapsed");
        btnWorldToggle.classList.toggle("collapsed", collapsed);
        btnWorldToggle.innerHTML = collapsed ? "&rarr;" : "&larr;";
    });

    const panelInspector = document.getElementById("panel-inspector");
    const btnInspectorToggle = document.getElementById("btn-inspector-toggle");

    btnInspectorToggle.addEventListener("click", () => {
        const collapsed = panelInspector.classList.toggle("collapsed");
        btnInspectorToggle.classList.toggle("collapsed", collapsed);
        btnInspectorToggle.innerHTML = collapsed ? "&larr;" : "&rarr;";
    });

    const btnToggleSettings = document.getElementById("btn-toggle-settings");
    const drawer = document.getElementById("spawn-settings-drawer");

    btnToggleSettings.addEventListener("click", () => {
        const expanded = drawer.classList.toggle("expanded");
        btnToggleSettings.innerHTML = expanded ? "Settings &darr;" : "Settings &uarr;";
    });

    // Play/Pause
    const btnPlay = document.getElementById("btn-play");
    btnPlay.addEventListener("click", () => {
        isPlaying = !isPlaying;
        btnPlay.textContent = isPlaying ? "Pause" : "Play";
        btnPlay.className = isPlaying ? "btn btn-primary" : "btn";
    });

    // Step frame
    document.getElementById("btn-step").addEventListener("click", () => {
        if (!isPlaying && world) {
            world.update(0.016, solverIterations);
            render();
        }
    });

    // Reset
    document.getElementById("btn-reset").addEventListener("click", () => {
        world.clear();
        selectedObj = null;
        selectedJoint = null;
        selectedJointType = "";
        updateInspectorHUD();
        setStatus("World cleared");
    });

    // Sliders
    const slideTimeScale = document.getElementById("slide-time-scale");
    const valTimeScale = document.getElementById("val-time-scale");
    slideTimeScale.addEventListener("input", (e) => {
        timeScale = parseFloat(e.target.value);
        valTimeScale.textContent = timeScale.toFixed(1) + "x";
    });

    const slideIterations = document.getElementById("slide-iterations");
    const valIterations = document.getElementById("val-iterations");
    slideIterations.addEventListener("input", (e) => {
        solverIterations = parseInt(e.target.value);
        valIterations.textContent = solverIterations;
    });

    const slideGravityX = document.getElementById("slide-gravity-x");
    const valGravityX = document.getElementById("val-gravity-x");
    slideGravityX.addEventListener("input", (e) => {
        currentGravity.x = parseFloat(e.target.value);
        valGravityX.textContent = currentGravity.x.toFixed(1) + " m/s²";
        world.setGravity(new Module.Vector(currentGravity.x * PIXELS_PER_METER, currentGravity.y * PIXELS_PER_METER));
    });

    const slideGravityY = document.getElementById("slide-gravity-y");
    const valGravityY = document.getElementById("val-gravity-y");
    slideGravityY.addEventListener("input", (e) => {
        currentGravity.y = parseFloat(e.target.value);
        valGravityY.textContent = currentGravity.y.toFixed(1) + " m/s²";
        world.setGravity(new Module.Vector(currentGravity.x * PIXELS_PER_METER, currentGravity.y * PIXELS_PER_METER));
    });

    document.getElementById("chk-grid").addEventListener("change", (e) => {
        showGrid = e.target.checked;
    });

    // Spawner Config inputs
    document.getElementById("spawn-mass").addEventListener("change", (e) => {
        spawnerSettings.mass = Math.max(0.01, parseFloat(e.target.value) || 1.0);
    });
    document.getElementById("spawn-restitution").addEventListener("input", (e) => {
        spawnerSettings.restitution = parseFloat(e.target.value);
    });
    document.getElementById("spawn-friction").addEventListener("input", (e) => {
        spawnerSettings.friction = parseFloat(e.target.value);
    });
    document.getElementById("spawn-color").addEventListener("input", (e) => {
        spawnerSettings.color = e.target.value;
    });
    document.getElementById("spawn-static").addEventListener("change", (e) => {
        spawnerSettings.isStatic = e.target.checked;
    });

    // Gear inputs
    document.getElementById("gear-teeth").addEventListener("change", (e) => {
        spawnerSettings.gearTeeth = Math.max(8, parseInt(e.target.value) || 16);
    });
    document.getElementById("gear-pitch").addEventListener("change", (e) => {
        spawnerSettings.gearPitch = Math.max(0.01, parseFloat(e.target.value) || 0.15);
    });
    document.getElementById("gear-motor").addEventListener("change", (e) => {
        spawnerSettings.gearMotor = e.target.checked;
    });
    document.getElementById("gear-motor-speed").addEventListener("change", (e) => {
        spawnerSettings.gearMotorSpeed = parseFloat(e.target.value) || 2.0;
    });

    // Mode switchers
    const spawnerButtons = {
        "btn-select-mode": "select",
        "spawn-circle": "spawn-circle",
        "spawn-rect": "spawn-rect",
        "spawn-polygon": "spawn-poly",
        "spawn-gear": "spawn-gear",
        "btn-link-distance": "link-distance",
        "btn-link-spring": "link-spring",
        "btn-link-gear": "link-gear"
    };

    Object.entries(spawnerButtons).forEach(([btnId, mode]) => {
        const btn = document.getElementById(btnId);
        if (!btn) return;
        btn.addEventListener("click", () => {
            document.querySelectorAll(".spawnables-bar .btn").forEach(b => b.classList.remove("active"));
            btn.classList.add("active");
            interactionMode = mode;
            linkStartObject = null;
            
            if (mode === "select") {
                setStatus("Selection Mode");
            } else {
                setStatus(`Mode: ${mode.replace("-", " ").toUpperCase()}`);
                // Automatically open drawer to allow spawn settings modifications
                drawer.classList.add("expanded");
                btnToggleSettings.innerHTML = "Settings &darr;";
            }

            const gearCard = document.getElementById("gear-parameters");
            if (interactionMode === "spawn-gear") {
                gearCard.classList.remove("hidden");
            } else {
                gearCard.classList.add("hidden");
            }
        });
    });

    // Scene Buttons
    document.getElementById("scene-gears").addEventListener("click", loadGearsScene);
    document.getElementById("scene-stack").addEventListener("click", loadStackingScene);
    document.getElementById("scene-washer").addEventListener("click", loadWasherScene);
    document.getElementById("scene-ccd").addEventListener("click", loadCCDScene);

    // Inspector Overlay Controls
    document.getElementById("inspector-close").addEventListener("click", () => {
        selectedObj = null;
        selectedJoint = null;
        selectedJointType = "";
        updateInspectorHUD();
    });

    document.getElementById("btn-inspect-delete").addEventListener("click", () => {
        if (selectedObj) {
            world.removeObject(selectedObj);
            selectedObj = null;
            updateInspectorHUD();
            setStatus("Object deleted");
        } else if (selectedJoint) {
            setStatus("Joint deletion requires deleting one of its anchored bodies.");
        }
    });

    // Modify selected parameters live (Object)
    document.getElementById("inspect-mass").addEventListener("change", (e) => {
        if (selectedObj) {
            selectedObj.mass = Math.max(0.01, parseFloat(e.target.value) || 1.0);
            selectedObj.updateDerivedProperties();
        }
    });
    document.getElementById("inspect-restitution").addEventListener("change", (e) => {
        if (selectedObj) {
            selectedObj.restitution = parseFloat(e.target.value);
        }
    });
    document.getElementById("inspect-friction").addEventListener("change", (e) => {
        if (selectedObj) {
            selectedObj.friction = parseFloat(e.target.value);
        }
    });
    document.getElementById("inspect-angle").addEventListener("change", (e) => {
        if (selectedObj) {
            selectedObj.angle = parseFloat(e.target.value);
            selectedObj.transformUpdateRequired = true;
            selectedObj.aabbUpdateRequired = true;
        }
    });
    document.getElementById("inspect-radius").addEventListener("change", (e) => {
        if (selectedObj && selectedObj.type === 0) {
            selectedObj.radius = Math.max(1, parseFloat(e.target.value) || 10);
            selectedObj.updateDerivedProperties();
        }
    });
    document.getElementById("inspect-width").addEventListener("change", (e) => {
        if (selectedObj && selectedObj.type === 1) {
            selectedObj.width = Math.max(1, parseFloat(e.target.value) || 10);
            selectedObj.updateDerivedProperties();
        }
    });
    document.getElementById("inspect-height").addEventListener("change", (e) => {
        if (selectedObj && selectedObj.type === 1) {
            selectedObj.height = Math.max(1, parseFloat(e.target.value) || 10);
            selectedObj.updateDerivedProperties();
        }
    });
    document.getElementById("inspect-pinned").addEventListener("change", (e) => {
        if (selectedObj) {
            selectedObj.isPinned = e.target.checked;
        }
    });
    document.getElementById("inspect-static").addEventListener("change", (e) => {
        if (selectedObj) {
            selectedObj.isStatic = e.target.checked;
            selectedObj.updateDerivedProperties();
        }
    });
    document.getElementById("inspect-ccd").addEventListener("change", (e) => {
        if (selectedObj) {
            selectedObj.ccdEnabled = e.target.checked;
        }
    });
    document.getElementById("inspect-motor").addEventListener("change", (e) => {
        if (selectedObj) {
            selectedObj.motorEnabled = e.target.checked;
        }
    });
    document.getElementById("inspect-motor-speed").addEventListener("change", (e) => {
        if (selectedObj) {
            selectedObj.motorSpeed = parseFloat(e.target.value) || 0.0;
        }
    });
    document.getElementById("inspect-motor-torque").addEventListener("change", (e) => {
        if (selectedObj) {
            selectedObj.motorMaxTorque = parseFloat(e.target.value) || 0.0;
        }
    });

    // Modify selected parameters live (Joint)
    document.getElementById("inspect-joint-rest").addEventListener("change", (e) => {
        if (selectedJoint) {
            selectedJoint.restLength = parseFloat(e.target.value) || 0;
        }
    });
    document.getElementById("inspect-joint-stiff").addEventListener("change", (e) => {
        if (selectedJoint && selectedJointType === 'spring') {
            selectedJoint.stiffness = parseFloat(e.target.value) || 0;
        }
    });
    document.getElementById("inspect-joint-damp").addEventListener("change", (e) => {
        if (selectedJoint && selectedJointType === 'spring') {
            selectedJoint.damping = parseFloat(e.target.value) || 0;
        }
    });
    document.getElementById("inspect-joint-isbelt").addEventListener("change", (e) => {
        if (selectedJoint && selectedJointType === 'gear') {
            selectedJoint.isBelt = e.target.checked;
        }
    });

    // Canvas Mouse bindings
    canvas.addEventListener("mousedown", handleCanvasMouseDown);
    canvas.addEventListener("mousemove", handleCanvasMouseMove);
    canvas.addEventListener("mouseup", handleCanvasMouseUp);
    canvas.addEventListener("contextmenu", e => e.preventDefault());

    // AI Natural Language Spawner
    document.getElementById("btn-ai-generate").addEventListener("click", handleAIPrompt);
}

// ==========================================
// Predefined Scene Loaders
// ==========================================
function hexToCoreColor(hex) {
    hex = hex.replace("#", "");
    const r = parseInt(hex.substring(0, 2), 16);
    const g = parseInt(hex.substring(2, 4), 16);
    const b = parseInt(hex.substring(4, 6), 16);
    return { r, g, b, a: 255 };
}

function loadGearsScene() {
    world.clear();
    selectedObj = null;
    selectedJoint = null;
    selectedJointType = "";
    updateInspectorHUD();

    // 1. Anchored boundary floor
    const grey = { r: 120, g: 120, b: 120, a: 255 };
    world.createRectangle(1200, 30, new Module.Vector(600, 750), new Module.Vector(0, 0), 0, 0, 10, 0.4, grey, true);

    // 2. Active Motor Gear
    const colA = hexToCoreColor("#3b82f6");
    let gearA = world.createCircle(106.667, new Module.Vector(400, 400), new Module.Vector(0, 0), 0, 0, 8.0, 0.2, colA, false);
    gearA.isGear = true;
    gearA.teethCount = 32;
    gearA.diametralPitch = 0.15;
    gearA.motorEnabled = true;
    gearA.motorSpeed = 2.0;
    gearA.isPinned = true;

    // 3. Meshed Gear B (Teeth: 16)
    const colB = hexToCoreColor("#8b5cf6");
    let gearB = world.createCircle(53.333, new Module.Vector(660, 400), new Module.Vector(0, 0), 0.05, 0, 4.0, 0.2, colB, false);
    gearB.isGear = true;
    gearB.teethCount = 16;
    gearB.diametralPitch = 0.15;
    gearB.isPinned = true;

    // Add Gear Meshing constraint link
    world.createGearJoint(gearA, gearB, false);

    setStatus("Loaded scene: Gears Mesh");
}

function loadStackingScene() {
    world.clear();
    selectedObj = null;
    selectedJoint = null;
    selectedJointType = "";
    updateInspectorHUD();

    // Floor
    const grey = { r: 100, g: 100, b: 100, a: 255 };
    world.createRectangle(1200, 40, new Module.Vector(600, 750), new Module.Vector(0, 0), 0, 0, 10, 0.2, grey, true);

    // Ramp
    world.createRectangle(400, 20, new Module.Vector(250, 500), new Module.Vector(0, 0), -0.4, 0, 10, 0.2, grey, true);

    // Stack boxes
    const orange = hexToCoreColor("#f97316");
    for (let i = 0; i < 6; ++i) {
        world.createRectangle(60, 60, new Module.Vector(700, 700 - i * 65), new Module.Vector(0, 0), 0, 0, 2.0, 0.05, orange, false);
    }

    // Stack circles
    const green = hexToCoreColor("#10b981");
    for (let i = 0; i < 5; ++i) {
        world.createCircle(30, new Module.Vector(850, 700 - i * 65), new Module.Vector(0, 0), 0, 0, 1.5, 0.1, green, false);
    }

    setStatus("Loaded scene: Stacking Test");
}

function loadWasherScene() {
    world.clear();
    selectedObj = null;
    selectedJoint = null;
    selectedJointType = "";
    updateInspectorHUD();

    // Floor container
    const grey = { r: 120, g: 120, b: 120, a: 255 };
    world.createRectangle(1000, 30, new Module.Vector(600, 750), new Module.Vector(0, 0), 0, 0, 10, 0.2, grey, true);
    world.createRectangle(30, 600, new Module.Vector(130, 450), new Module.Vector(0, 0), 0, 0, 10, 0.2, grey, true);
    world.createRectangle(30, 600, new Module.Vector(1070, 450), new Module.Vector(0, 0), 0, 0, 10, 0.2, grey, true);

    // Spawning a cluster of circles falling down (liquid-like stack)
    const colors = ["#3b82f6", "#10b981", "#f97316", "#ef4444", "#8b5cf6"];
    for (let row = 0; row < 6; ++row) {
        for (let col = 0; col < 12; ++col) {
            const hex = colors[(row + col) % colors.length];
            const c = hexToCoreColor(hex);
            const offset = (row % 2) * 15;
            world.createCircle(20, new Module.Vector(350 + col * 45 + offset, 200 + row * 45), new Module.Vector(0, 0), 0, 0, 1.0, 0.1, c, false);
        }
    }

    setStatus("Loaded scene: Washer Stack");
}

function loadCCDScene() {
    world.clear();
    selectedObj = null;
    selectedJoint = null;
    selectedJointType = "";
    updateInspectorHUD();

    // Spawning a super thin wall
    const grey = { r: 160, g: 160, b: 160, a: 255 };
    world.createRectangle(15, 600, new Module.Vector(600, 400), new Module.Vector(0, 0), 0, 0, 10, 0.5, grey, true);

    // Static borders
    world.createRectangle(1200, 20, new Module.Vector(600, 780), new Module.Vector(0, 0), 0, 0, 10, 0.2, grey, true);
    world.createRectangle(1200, 20, new Module.Vector(600, 20), new Module.Vector(0, 0), 0, 0, 10, 0.2, grey, true);

    setStatus("CCD Ready: Launch dynamic bullet circles with Right-Click Drag!");
}

// ==========================================
// Mouse and Raycast selectors
// ==========================================
function getMousePos(e) {
    const rect = canvas.getBoundingClientRect();
    // Convert screen-space mouse position to world-space by subtracting camera offset
    return {
        x: e.clientX - rect.left - cameraX,
        y: e.clientY - rect.top - cameraY
    };
}

function getScreenMousePos(e) {
    const rect = canvas.getBoundingClientRect();
    return {
        x: e.clientX - rect.left,
        y: e.clientY - rect.top
    };
}

function getObjectAt(pos) {
    const list = world.getObjects();
    for (let i = 0; i < list.size(); ++i) {
        const obj = list.get(i);
        if (obj.isBeltSpan) continue;
        if (obj.type === 0) { // circle
            const d = Math.hypot(obj.position.x - pos.x, obj.position.y - pos.y);
            if (d <= obj.radius) return obj;
        } else { // polygon
            const aabb = obj.getAABB();
            if (pos.x >= aabb.min.x && pos.x <= aabb.max.x && pos.y >= aabb.min.y && pos.y <= aabb.max.y) {
                return obj;
            }
        }
    }
    return null;
}

function pointToSegmentDistance(p, a, b) {
    const l2 = Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2);
    if (l2 === 0) return Math.hypot(p.x - a.x, p.y - a.y);
    let t = ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / l2;
    t = Math.max(0, Math.min(1, t));
    return Math.hypot(
        p.x - (a.x + t * (b.x - a.x)),
        p.y - (a.y + t * (b.y - a.y))
    );
}

function getJointAt(pos) {
    const threshold = 12.0; // Click range tolerance
    
    // 1. Distance Joints
    const dJoints = world.getDistanceJoints();
    for (let i = 0; i < dJoints.size(); ++i) {
        const j = dJoints.get(i);
        const pA = getAnchorWorldPos(j.objA, j.localAnchorA);
        const pB = getAnchorWorldPos(j.objB, j.localAnchorB);
        if (pointToSegmentDistance(pos, pA, pB) < threshold) {
            return { type: 'distance', joint: j };
        }
    }
    
    // 2. Spring Joints
    const sJoints = world.getSpringJoints();
    for (let i = 0; i < sJoints.size(); ++i) {
        const j = sJoints.get(i);
        const pA = getAnchorWorldPos(j.objA, j.localAnchorA);
        const pB = getAnchorWorldPos(j.objB, j.localAnchorB);
        if (pointToSegmentDistance(pos, pA, pB) < threshold) {
            return { type: 'spring', joint: j };
        }
    }
    
    // 3. Gear Joints
    const gJoints = world.getGearJoints();
    for (let i = 0; i < gJoints.size(); ++i) {
        const j = gJoints.get(i);
        const pA = j.objA.position;
        const pB = j.objB.position;
        if (pointToSegmentDistance(pos, pA, pB) < threshold) {
            return { type: 'gear', joint: j };
        }
    }
    
    return null;
}

// ==========================================
// Canvas Event Handlers
// ==========================================
function handleCanvasMouseDown(e) {
    const pos = getMousePos(e);
    const screenPos = getScreenMousePos(e);

    // Right-Click Drag — fling an existing object OR pan the viewport
    if (e.button === 2) {
        const hit = getObjectAt(pos);
        if (hit) {
            selectedObj = hit;
            selectedJoint = null;
            selectedJointType = "";
            isSlingshotDraggingObj = true;
            slingshotObjStartPos = { x: hit.position.x, y: hit.position.y };
            slingshotCurrent = pos;
            updateInspectorHUD();
        } else {
            // Right click on empty space starts panning the viewport
            isPanning = true;
            panStartMouse = screenPos;
            panStartCamera = { x: cameraX, y: cameraY };
            canvas.style.cursor = "grabbing";
        }
        return;
    }

    if (e.button !== 0) return;

    if (interactionMode === "select") {
        const hit = getObjectAt(pos);
        if (hit) {
            selectedObj = hit;
            selectedJoint = null;
            selectedJointType = "";
            isDraggingBody = true;
            dragOffset = {
                x: pos.x - hit.position.x,
                y: pos.y - hit.position.y
            };
            wasStaticBeforeDrag = hit.isStatic;
            updateInspectorHUD();
        } else {
            const jointData = getJointAt(pos);
            if (jointData) {
                selectedObj = null;
                selectedJoint = jointData.joint;
                selectedJointType = jointData.type;
                updateInspectorHUD();
            } else {
                // Left click on empty space deselects
                selectedObj = null;
                selectedJoint = null;
                selectedJointType = "";
                updateInspectorHUD();
            }
        }
        return;
    }

    // Joint Links Mode
    if (interactionMode.startsWith("link-")) {
        const obj = getObjectAt(pos);
        if (!obj) return;

        if (!linkStartObject) {
            linkStartObject = obj;
            setStatus(`Link Mode: Select second object...`);
        } else {
            if (linkStartObject === obj) {
                linkStartObject = null;
                setStatus("Link cancelled (same object)");
                return;
            }

            if (interactionMode === "link-distance") {
                const len = Math.hypot(linkStartObject.position.x - obj.position.x, linkStartObject.position.y - obj.position.y);
                world.createDistanceJoint(linkStartObject, obj, new Module.Vector(0, 0), new Module.Vector(0, 0), len, 2.0, 0.7);
                setStatus("Distance Joint linked");
            } else if (interactionMode === "link-spring") {
                const len = Math.hypot(linkStartObject.position.x - obj.position.x, linkStartObject.position.y - obj.position.y);
                world.createSpringJoint(linkStartObject, obj, new Module.Vector(0, 0), new Module.Vector(0, 0), len, 30.0, 1.5);
                setStatus("Spring Joint linked");
            } else if (interactionMode === "link-gear") {
                if (linkStartObject.isGear && obj.isGear) {
                    world.createGearJoint(linkStartObject, obj, false);
                    setStatus("Gear connection linked");
                } else {
                    setStatus("Error: Mesh gears requires both objects to be Gears.");
                }
            }

            linkStartObject = null;
            interactionMode = "select";
            document.querySelectorAll(".spawnables-bar .btn").forEach(b => b.classList.remove("active"));
            document.getElementById("btn-select-mode").classList.add("active");
        }
        return;
    }

    // Spawn Mode
    const color = hexToCoreColor(spawnerSettings.color);
    let newObj = null;

    if (interactionMode === "spawn-circle") {
        newObj = world.createCircle(30, new Module.Vector(pos.x, pos.y), new Module.Vector(0, 0), 0, 0, spawnerSettings.mass, spawnerSettings.restitution, color, spawnerSettings.isStatic);
        newObj.friction = spawnerSettings.friction;
    } else if (interactionMode === "spawn-rect") {
        newObj = world.createRectangle(80, 50, new Module.Vector(pos.x, pos.y), new Module.Vector(0, 0), 0, 0, spawnerSettings.mass, spawnerSettings.restitution, color, spawnerSettings.isStatic);
        newObj.friction = spawnerSettings.friction;
    } else if (interactionMode === "spawn-poly") {
        const r = 40;
        const verts = new Module.Vector2DList();
        for (let i = 0; i < 5; ++i) {
            const angle = (i * 2 * Math.PI) / 5;
            verts.push_back(new Module.Vector(r * Math.cos(angle), r * Math.sin(angle)));
        }
        newObj = world.createPolygon(verts, new Module.Vector(pos.x, pos.y), new Module.Vector(0, 0), 0, 0, spawnerSettings.mass, spawnerSettings.restitution, color, spawnerSettings.isStatic);
        newObj.friction = spawnerSettings.friction;
    } else if (interactionMode === "spawn-gear") {
        const rad = spawnerSettings.gearTeeth / (2.0 * spawnerSettings.gearPitch);
        newObj = world.createCircle(rad, new Module.Vector(pos.x, pos.y), new Module.Vector(0, 0), 0, 0, spawnerSettings.mass, spawnerSettings.restitution, color, spawnerSettings.isStatic);
        newObj.isGear = true;
        newObj.teethCount = spawnerSettings.gearTeeth;
        newObj.diametralPitch = spawnerSettings.gearPitch;
        newObj.friction = spawnerSettings.friction;
        
        if (spawnerSettings.gearMotor) {
            newObj.motorEnabled = true;
            newObj.motorSpeed = spawnerSettings.gearMotorSpeed;
            newObj.isPinned = true;
        }
    }

    if (newObj) {
        setStatus("Object spawned at (" + Math.round(pos.x) + ", " + Math.round(pos.y) + ")");
        selectedObj = newObj;
        updateInspectorHUD();
        // Stay in the current spawn mode so the user can place more objects
        // without having to re-click the spawn button each time.
        // The spawned object is still shown in the inspector for quick inspection.
    }
}

function handleCanvasMouseMove(e) {
    const pos = getMousePos(e);
    const screenPos = getScreenMousePos(e);

    // Camera pan
    if (isPanning) {
        cameraX = panStartCamera.x + (screenPos.x - panStartMouse.x);
        cameraY = panStartCamera.y + (screenPos.y - panStartMouse.y);
        if (!isPlaying) render();
        return; // skip other drag logic while panning
    }

    // Update cursor to suggest panning is available
    if (interactionMode === "select" && !isDraggingBody) {
        canvas.style.cursor = "";
    }
    
    if (isDraggingBody && selectedObj) {
        selectedObj.isStatic = true;
        selectedObj.MoveTo(new Module.Vector(pos.x - dragOffset.x, pos.y - dragOffset.y));
        selectedObj.linearVelocity = new Module.Vector(0, 0);
        selectedObj.angularVelocity = 0.0;
        
        if (!isPlaying) {
            world.updateBeltSpans(0.0);
            render();
        }
    }

    if (isSlingshotDraggingObj) {
        slingshotCurrent = pos;
    }
}

function handleCanvasMouseUp(e) {
    const pos = getMousePos(e);

    if (e.button === 0) {
        if (isDraggingBody) {
            isDraggingBody = false;
            if (selectedObj) {
                selectedObj.isStatic = wasStaticBeforeDrag;
                selectedObj.updateDerivedProperties();
            }
        }
    }

    if (e.button === 2) {
        // Stop panning
        if (isPanning) {
            isPanning = false;
            canvas.style.cursor = "";
        }

        if (isSlingshotDraggingObj && selectedObj) {
            isSlingshotDraggingObj = false;
            const dx = slingshotObjStartPos.x - pos.x;
            const dy = slingshotObjStartPos.y - pos.y;
            selectedObj.linearVelocity = new Module.Vector(dx * 5.0, dy * 5.0);
            setStatus("Launched object!");
        }
    }
}

// ==========================================
// Inspector Data Syncer
// ==========================================
function updateInspectorHUD() {
    const panel = document.getElementById("panel-inspector");
    const btn = document.getElementById("btn-inspector-toggle");

    if (!selectedObj && !selectedJoint) {
        panel.classList.add("collapsed");
        btn.classList.add("collapsed");
        btn.innerHTML = "&larr;";
        return;
    }

    panel.classList.remove("collapsed");
    btn.classList.remove("collapsed");
    btn.innerHTML = "&rarr;";
    
    const objSection = document.getElementById("inspect-object-section");
    const jointSection = document.getElementById("inspect-joint-section");
    
    if (selectedObj) {
        objSection.classList.remove("hidden");
        jointSection.classList.add("hidden");
        document.getElementById("inspector-title").textContent = "Object Inspector";
        
        document.getElementById("inspect-type").textContent = selectedObj.isGear ? "Gear" : (selectedObj.type === 0 ? "Circle" : "Polygon");
        document.getElementById("inspect-pos").textContent = `(${Math.round(selectedObj.position.x)}, ${Math.round(selectedObj.position.y)})`;
        document.getElementById("inspect-vel").textContent = `(${Math.round(selectedObj.linearVelocity.x)}, ${Math.round(selectedObj.linearVelocity.y)})`;
        
        document.getElementById("inspect-mass").value = selectedObj.mass.toFixed(2);
        document.getElementById("inspect-restitution").value = selectedObj.restitution.toFixed(2);
        document.getElementById("inspect-friction").value = selectedObj.friction.toFixed(2);
        document.getElementById("inspect-angle").value = selectedObj.angle.toFixed(2);
        document.getElementById("inspect-pinned").checked = selectedObj.isPinned;
        document.getElementById("inspect-static").checked = selectedObj.isStatic;
        document.getElementById("inspect-ccd").checked = selectedObj.ccdEnabled;
        
        const radiusLbl = document.getElementById("inspect-radius-lbl");
        const widthLbl = document.getElementById("inspect-width-lbl");
        const heightLbl = document.getElementById("inspect-height-lbl");
        
        if (selectedObj.type === 0) { // Circle / Gear
            radiusLbl.classList.remove("hidden");
            widthLbl.classList.add("hidden");
            heightLbl.classList.add("hidden");
            document.getElementById("inspect-radius").value = Math.round(selectedObj.radius);
        } else { // Box / Polygon
            radiusLbl.classList.add("hidden");
            widthLbl.classList.remove("hidden");
            heightLbl.classList.remove("hidden");
            document.getElementById("inspect-width").value = Math.round(selectedObj.width);
            document.getElementById("inspect-height").value = Math.round(selectedObj.height);
        }
        
        const gearSect = document.getElementById("inspect-gear-section");
        if (selectedObj.isGear) {
            gearSect.classList.remove("hidden");
            document.getElementById("inspect-motor").checked = selectedObj.motorEnabled;
            document.getElementById("inspect-motor-speed").value = selectedObj.motorSpeed.toFixed(2);
            document.getElementById("inspect-motor-torque").value = Math.round(selectedObj.motorMaxTorque);
        } else {
            gearSect.classList.add("hidden");
        }
    } else if (selectedJoint) {
        objSection.classList.add("hidden");
        jointSection.classList.remove("hidden");
        document.getElementById("inspector-title").textContent = "Joint Inspector";
        
        document.getElementById("inspect-joint-type").textContent = selectedJointType.toUpperCase() + " Joint";
        document.getElementById("inspect-joint-rest").value = Math.round(selectedJoint.restLength || 0);
        
        const stiffLbl = document.getElementById("inspect-joint-stiff-lbl");
        const dampLbl = document.getElementById("inspect-joint-damp-lbl");
        const beltLbl = document.getElementById("inspect-joint-belt-lbl");
        
        if (selectedJointType === 'distance') {
            stiffLbl.classList.add("hidden");
            dampLbl.classList.add("hidden");
            beltLbl.classList.add("hidden");
        } else if (selectedJointType === 'spring') {
            stiffLbl.classList.remove("hidden");
            dampLbl.classList.remove("hidden");
            beltLbl.classList.add("hidden");
            document.getElementById("inspect-joint-stiff").value = Math.round(selectedJoint.stiffness);
            document.getElementById("inspect-joint-damp").value = selectedJoint.damping.toFixed(2);
        } else if (selectedJointType === 'gear') {
            stiffLbl.classList.add("hidden");
            dampLbl.classList.add("hidden");
            beltLbl.classList.remove("hidden");
            document.getElementById("inspect-joint-isbelt").checked = selectedJoint.isBelt;
        }
    }
}

function setStatus(txt) {
    const badge = document.querySelector("#launcher-hud");
    badge.innerHTML = `<span class="badge">Info</span> ${txt}`;
}

// ==========================================
// AI Text Prompt Parser
// ==========================================
function handleAIPrompt() {
    const text = document.getElementById("ai-prompt").value.trim().toLowerCase();
    if (!text) return;

    world.clear();
    selectedObj = null;
    selectedJoint = null;
    selectedJointType = "";
    updateInspectorHUD();

    // Create container boundary automatically
    const grey = { r: 120, g: 120, b: 120, a: 255 };
    world.createRectangle(1200, 30, new Module.Vector(600, 770), new Module.Vector(0, 0), 0, 0, 10, 0.4, grey, true);

    const cyan = hexToCoreColor("#3b82f6");
    const purple = hexToCoreColor("#8b5cf6");

    const stackMatch = text.match(/stack\s*(\d+)\s*(box|circle|rectangle|block)es?/);
    if (stackMatch) {
        const count = parseInt(stackMatch[1]);
        const shape = stackMatch[2];
        
        for (let i = 0; i < count; i++) {
            const x = 600 + (Math.random() - 0.5) * 5;
            const y = 700 - i * 65;
            if (shape === "circle") {
                world.createCircle(25, new Module.Vector(x, y), new Module.Vector(0, 0), 0, 0, 1.0, 0.1, cyan, false);
            } else {
                world.createRectangle(50, 50, new Module.Vector(x, y), new Module.Vector(0, 0), 0, 0, 1.5, 0.05, purple, false);
            }
        }
        setStatus(`AI build complete: stacked ${count} ${shape}s.`);
        return;
    }

    const gearMatch = text.match(/(spawn|mesh)\s*(\d+)\s*(and\s*(\d+))?\s*teeth\s*gears?/);
    if (gearMatch) {
        const teeth1 = parseInt(gearMatch[2]);
        const teeth2 = gearMatch[4] ? parseInt(gearMatch[4]) : 0;
        
        let speed = 2.0;
        const speedMatch = text.match(/speed\s*(-?\d+(\.\d+)?)/);
        if (speedMatch) speed = parseFloat(speedMatch[1]);

        const pitch = 0.15;
        const r1 = teeth1 / (2.0 * pitch);

        let gearA = world.createCircle(r1, new Module.Vector(450, 400), new Module.Vector(0, 0), 0, 0, 5.0, 0.2, cyan, false);
        gearA.isGear = true;
        gearA.teethCount = teeth1;
        gearA.diametralPitch = pitch;
        gearA.motorEnabled = true;
        gearA.motorSpeed = speed;
        gearA.isPinned = true;

        if (teeth2 > 0) {
            const r2 = teeth2 / (2.0 * pitch);
            let gearB = world.createCircle(r2, new Module.Vector(450 + r1 + r2, 400), new Module.Vector(0, 0), 0.05, 0, 3.0, 0.2, purple, false);
            gearB.isGear = true;
            gearB.teethCount = teeth2;
            gearB.diametralPitch = pitch;
            gearB.isPinned = true;

            world.createGearJoint(gearA, gearB, false);
            setStatus(`AI build complete: Meshed ${teeth1}T & ${teeth2}T gears.`);
        } else {
            setStatus(`AI build complete: Spawner gear ${teeth1}T.`);
        }
        return;
    }

    const cradleMatch = text.match(/cradle\s*of\s*(\d+)/);
    if (cradleMatch) {
        const count = parseInt(cradleMatch[1]);
        const startX = 600 - (count * 40) / 2;
        
        for (let i = 0; i < count; i++) {
            const x = startX + i * 40;
            let anchor = world.createRectangle(20, 20, new Module.Vector(x, 150), new Module.Vector(0, 0), 0, 0, 10.0, 0, grey, true);
            
            const swingX = (i === 0) ? x - 120 : x;
            const swingY = (i === 0) ? 150 + 260 : 150 + 300;
            
            let circle = world.createCircle(19.8, new Module.Vector(swingX, swingY), new Module.Vector(0, 0), 0, 0, 2.5, 0.99, purple, false);
            world.createDistanceJoint(anchor, circle, new Module.Vector(0, 0), new Module.Vector(0, 0), 300.0, 2.0, 0);
        }
        
        setStatus(`AI build complete: Newton's Cradle with ${count} nodes.`);
        return;
    }

    setStatus("AI didn't understand the description. Try: 'stack 8 boxes'");
}

// ==========================================
// Simulation Main Loop & Rendering
// ==========================================
let lastTime = performance.now();
let accumulator = 0.0;
const targetDt = 1.0 / 60.0; // 60Hz physics time stepping

function loop(timestamp) {
    let elapsed = (timestamp - lastTime) / 1000.0;
    lastTime = timestamp;

    // Guard against massive frame drops (spiral of death)
    if (elapsed > 0.1) elapsed = 0.1;

    if (isPlaying && world) {
        accumulator += elapsed * timeScale;
        while (accumulator >= targetDt) {
            // Drag logic: temporarily lock dynamic bodies to static
            let wasStatic = false;
            if (isDraggingBody && selectedObj) {
                wasStatic = wasStaticBeforeDrag;
                selectedObj.isStatic = true;
                selectedObj.linearVelocity = new Module.Vector(0, 0);
                selectedObj.angularVelocity = 0.0;
            }

            world.update(targetDt, solverIterations);

            if (isDraggingBody && selectedObj) {
                selectedObj.isStatic = wasStatic;
            }

            accumulator -= targetDt;
        }
    }

    render();
    requestAnimationFrame(loop);
}

function render() {
    if (!ctx) return;
    
    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw Spatial Hash Grid cells (if enabled) — drawn before camera transform so it tiles naturally
    if (showGrid) {
        drawGridCells();
    }

    // Apply camera transform for all world-space drawing
    ctx.save();
    ctx.translate(cameraX, cameraY);

    // Draw Joint Links
    drawJoints();

    // Draw Objects
    drawObjects();

    // Slingshot object preview indicator (right click dragging body)
    if (isSlingshotDraggingObj && selectedObj && slingshotCurrent) {
        drawObjSlingshotUI();
    }

    // Selected object highlight indicator
    if (selectedObj) {
        drawSelectionHighlight();
    }

    ctx.restore();
}

function drawGridCells() {
    const cellSize = 64;
    ctx.strokeStyle = "rgba(255, 255, 255, 0.03)";
    ctx.lineWidth = 1;

    // Offset the grid lines by the fractional part of the camera offset
    // so the grid tiles seamlessly as the camera pans
    const offsetX = ((cameraX % cellSize) + cellSize) % cellSize;
    const offsetY = ((cameraY % cellSize) + cellSize) % cellSize;
    
    for (let x = offsetX - cellSize; x < canvas.width + cellSize; x += cellSize) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, canvas.height);
        ctx.stroke();
    }
    for (let y = offsetY - cellSize; y < canvas.height + cellSize; y += cellSize) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvas.width, y);
        ctx.stroke();
    }
}

function drawObjects() {
    const list = world.getObjects();
    for (let i = 0; i < list.size(); ++i) {
        const obj = list.get(i);
        if (obj.isBeltSpan) continue;
        
        ctx.fillStyle = `rgba(${obj.color.r}, ${obj.color.g}, ${obj.color.b}, 0.75)`;
        ctx.strokeStyle = `rgb(${obj.color.r}, ${obj.color.g}, ${obj.color.b})`;
        ctx.lineWidth = 2.0;

        // NO NEON GLOWS here (shadowBlur is deactivated)
        ctx.shadowBlur = 0;

        if (obj.type === 0) { // Circle / Gear
            ctx.beginPath();
            ctx.arc(obj.position.x, obj.position.y, obj.radius, 0, 2 * Math.PI);
            ctx.fill();
            ctx.stroke();

            // Draw gear teeth
            if (obj.isGear) {
                drawGearTeeth(obj);
            } else {
                // Radial orientation line
                ctx.beginPath();
                ctx.moveTo(obj.position.x, obj.position.y);
                ctx.lineTo(
                    obj.position.x + obj.radius * Math.cos(obj.angle),
                    obj.position.y + obj.radius * Math.sin(obj.angle)
                );
                ctx.strokeStyle = "rgba(255, 255, 255, 0.8)";
                ctx.lineWidth = 1.5;
                ctx.stroke();
            }
        } else { // Polygon / Box
            const verts = obj.getTransformedVertices();
            if (verts.size() > 0) {
                ctx.beginPath();
                const p0 = verts.get(0);
                ctx.moveTo(p0.x, p0.y);
                for (let v = 1; v < verts.size(); ++v) {
                    const p = verts.get(v);
                    ctx.lineTo(p.x, p.y);
                }
                ctx.closePath();
                ctx.fill();
                ctx.stroke();
            }
        }
    }
}

function drawGearTeeth(obj) {
    const teeth = obj.teethCount;
    const center = obj.position;
    const r = obj.radius;
    const toothHeight = 8;
    const angleStep = (2 * Math.PI) / teeth;

    ctx.fillStyle = ctx.strokeStyle;
    ctx.beginPath();

    for (let i = 0; i < teeth; ++i) {
        const theta = obj.angle + i * angleStep;
        
        // 4 points of trapezoid tooth
        const p1_x = center.x + (r - 2) * Math.cos(theta - angleStep * 0.25);
        const p1_y = center.y + (r - 2) * Math.sin(theta - angleStep * 0.25);
        
        const p2_x = center.x + (r + toothHeight) * Math.cos(theta - angleStep * 0.15);
        const p2_y = center.y + (r + toothHeight) * Math.sin(theta - angleStep * 0.15);
        
        const p3_x = center.x + (r + toothHeight) * Math.cos(theta + angleStep * 0.15);
        const p3_y = center.y + (r + toothHeight) * Math.sin(theta + angleStep * 0.15);
        
        const p4_x = center.x + (r - 2) * Math.cos(theta + angleStep * 0.25);
        const p4_y = center.y + (r - 2) * Math.sin(theta + angleStep * 0.25);

        ctx.moveTo(p1_x, p1_y);
        ctx.lineTo(p2_x, p2_y);
        ctx.lineTo(p3_x, p3_y);
        ctx.lineTo(p4_x, p4_y);
    }
    ctx.fill();
}

function drawJoints() {
    ctx.lineWidth = 2.0;

    // 1. Distance Joints
    const dJoints = world.getDistanceJoints();
    for (let i = 0; i < dJoints.size(); ++i) {
        const j = dJoints.get(i);
        const pA = getAnchorWorldPos(j.objA, j.localAnchorA);
        const pB = getAnchorWorldPos(j.objB, j.localAnchorB);
        
        ctx.strokeStyle = (selectedJoint === j) ? "rgba(255, 255, 255, 0.95)" : "rgba(59, 130, 246, 0.5)";
        ctx.beginPath();
        ctx.moveTo(pA.x, pA.y);
        ctx.lineTo(pB.x, pB.y);
        ctx.stroke();
        
        ctx.fillStyle = (selectedJoint === j) ? "#ffffff" : "#3b82f6";
        ctx.beginPath();
        ctx.arc(pA.x, pA.y, 4, 0, 2*Math.PI);
        ctx.arc(pB.x, pB.y, 4, 0, 2*Math.PI);
        ctx.fill();
    }

    // 2. Spring Joints
    const sJoints = world.getSpringJoints();
    for (let i = 0; i < sJoints.size(); ++i) {
        const j = sJoints.get(i);
        const pA = getAnchorWorldPos(j.objA, j.localAnchorA);
        const pB = getAnchorWorldPos(j.objB, j.localAnchorB);
        
        ctx.strokeStyle = (selectedJoint === j) ? "rgba(255, 255, 255, 0.95)" : "rgba(139, 92, 246, 0.6)";
        drawSpringCoils(pA, pB);
    }

    // 3. Gear / Belt tangent loops
    const gJoints = world.getGearJoints();
    for (let i = 0; i < gJoints.size(); ++i) {
        const j = gJoints.get(i);
        
        const isSel = (selectedJoint === j);
        const col = isSel ? "rgba(255, 255, 255, 0.85)" : "rgba(249, 115, 22, 0.5)";
        ctx.strokeStyle = col;
        
        if (j.isBelt) {
            drawBelttangents(j, isSel);
        } else {
            ctx.beginPath();
            ctx.setLineDash([4, 4]);
            ctx.moveTo(j.objA.position.x, j.objA.position.y);
            ctx.lineTo(j.objB.position.x, j.objB.position.y);
            ctx.stroke();
            ctx.setLineDash([]);
        }
    }
}

function drawBelttangents(gear, isSelected) {
    let objL = gear.objA;
    let objR = gear.objB;
    if (objL.position.x > objR.position.x) {
        objL = gear.objB;
        objR = gear.objA;
    }

    const pL = objL.position;
    const pR = objR.position;
    const rL = objL.radius;
    const rR = objR.radius;

    const dx = pR.x - pL.x;
    const dy = pR.y - pL.y;
    const dist = Math.hypot(dx, dy);
    
    if (dist <= Math.abs(rL - rR)) {
        ctx.beginPath();
        ctx.moveTo(pL.x, pL.y);
        ctx.lineTo(pR.x, pR.y);
        ctx.stroke();
        return;
    }

    const ux = dx / dist;
    const uy = dy / dist;

    const sinAlpha = (rL - rR) / dist;
    const alpha = Math.asin(sinAlpha);
    const cosAlpha = Math.cos(alpha);

    const tx = ux * cosAlpha - uy * sinAlpha;
    const ty = ux * sinAlpha + uy * cosAlpha;
    const px = ty;
    const py = -tx;

    const pL_top = { x: pL.x + px * rL, y: pL.y + py * rL };
    const pR_top = { x: pR.x + px * rR, y: pR.y + py * rR };

    const pL_bot = { x: pL.x - px * rL, y: pL.y - py * rL };
    const pR_bot = { x: pR.x - px * rR, y: pR.y - py * rR };

    ctx.lineWidth = isSelected ? 2.5 : 1.8;
    ctx.strokeStyle = isSelected ? "rgba(255, 255, 255, 0.95)" : "rgba(249, 115, 22, 0.6)";

    ctx.beginPath();
    ctx.moveTo(pL_top.x, pL_top.y);
    ctx.lineTo(pR_top.x, pR_top.y);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(pL_bot.x, pL_bot.y);
    ctx.lineTo(pR_bot.x, pR_bot.y);
    ctx.stroke();

    ctx.beginPath();
    const startAngleL = Math.atan2(-py, -px);
    const endAngleL = Math.atan2(py, px);
    ctx.arc(pL.x, pL.y, rL, startAngleL, endAngleL, false);
    ctx.stroke();

    ctx.beginPath();
    const startAngleR = Math.atan2(py, px);
    const endAngleR = Math.atan2(-py, -px);
    ctx.arc(pR.x, pR.y, rR, startAngleR, endAngleR, false);
    ctx.stroke();

    const animOffset = ((gear.beltAnimationOffset || 0) % 12 + 12) % 12;
    const linkSpacing = 12;
    const rollerCount = Math.floor(dist / linkSpacing);
    
    ctx.fillStyle = isSelected ? "#ffffff" : "rgba(255, 255, 255, 0.5)";
    for (let i = 0; i <= rollerCount; i++) {
        const offset = animOffset + i * linkSpacing;
        if (offset < dist) {
            const t = offset / dist;
            const rx = pL_top.x * (1 - t) + pR_top.x * t;
            const ry = pL_top.y * (1 - t) + pR_top.y * t;
            ctx.beginPath();
            ctx.arc(rx, ry, 2, 0, 2 * Math.PI);
            ctx.fill();

            const bx = pR_bot.x * (1 - t) + pL_bot.x * t;
            const by = pR_bot.y * (1 - t) + pL_bot.y * t;
            ctx.beginPath();
            ctx.arc(bx, by, 2, 0, 2 * Math.PI);
            ctx.fill();
        }
    }
}

function getAnchorWorldPos(obj, localAnchor) {
    if (localAnchor.x === 0 && localAnchor.y === 0) {
        return obj.position;
    }
    const cos = Math.cos(obj.angle);
    const sin = Math.sin(obj.angle);
    return {
        x: obj.position.x + (localAnchor.x * cos - localAnchor.y * sin),
        y: obj.position.y + (localAnchor.x * sin + localAnchor.y * cos)
    };
}

function drawSpringCoils(p1, p2) {
    const dx = p2.x - p1.x;
    const dy = p2.y - p1.y;
    const len = Math.hypot(dx, dy);
    
    if (len < 5) return;

    const angle = Math.atan2(dy, dx);
    const coils = 12;
    const coilWidth = 8;
    
    ctx.save();
    ctx.translate(p1.x, p1.y);
    ctx.rotate(angle);
    
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(10, 0);
    
    const coilStep = (len - 20) / coils;
    for (let i = 0; i < coils; ++i) {
        const cx = 10 + i * coilStep + coilStep / 2;
        const cy = (i % 2 === 0) ? -coilWidth : coilWidth;
        ctx.lineTo(cx, cy);
    }
    
    ctx.lineTo(len - 10, 0);
    ctx.lineTo(len, 0);
    ctx.stroke();
    
    ctx.restore();
}



function drawObjSlingshotUI() {
    const start = slingshotObjStartPos;
    const mouse = slingshotCurrent;
    const r_gizmo = (selectedObj.radius || 30) * 0.7;

    const dx = start.x - mouse.x;
    const dy = start.y - mouse.y;
    const dist = Math.hypot(dx, dy);

    if (dist < 2) return;

    // Draw elastic band
    ctx.strokeStyle = "rgba(180, 50, 50, 0.7)";
    ctx.lineWidth = 2.5;
    
    const ux = dx / dist;
    const uy = dy / dist;
    const px = -uy;
    const py = ux;

    ctx.beginPath();
    ctx.moveTo(start.x + px * r_gizmo, start.y + py * r_gizmo);
    ctx.lineTo(mouse.x, mouse.y);
    ctx.lineTo(start.x - px * r_gizmo, start.y - py * r_gizmo);
    ctx.stroke();

    // Dotted projection line
    ctx.strokeStyle = "rgba(74, 222, 128, 0.5)";
    ctx.lineWidth = 1.5;
    ctx.setLineDash([3, 3]);
    
    ctx.beginPath();
    ctx.moveTo(start.x, start.y);
    ctx.lineTo(start.x + dx * 2, start.y + dy * 2);
    ctx.stroke();
    ctx.setLineDash([]);
}

function drawSelectionHighlight() {
    // Pulse selected gold indicator ring
    const chronometre = performance.now() / 1000;
    const pulseAlpha = 0.5 + 0.25 * Math.sin(chronometre * 4.0);

    ctx.strokeStyle = `rgba(251, 191, 36, ${pulseAlpha})`;
    ctx.lineWidth = 2.0;

    const center = selectedObj.position;
    const radius = selectedObj.radius || 30;

    if (selectedObj.type === 0) { // circle
        ctx.beginPath();
        ctx.arc(center.x, center.y, radius + 6, 0, 2 * Math.PI);
        ctx.stroke();

        ctx.fillStyle = `rgba(251, 191, 36, ${pulseAlpha})`;
        ctx.beginPath();
        ctx.arc(center.x, center.y, 3, 0, 2 * Math.PI);
        ctx.fill();

        const handlePos = {
            x: center.x + Math.cos(selectedObj.angle) * (radius + 6),
            y: center.y + Math.sin(selectedObj.angle) * (radius + 6)
        };
        ctx.beginPath();
        ctx.moveTo(center.x, center.y);
        ctx.lineTo(handlePos.x, handlePos.y);
        ctx.stroke();

        ctx.fillStyle = "#fbbf24";
        ctx.beginPath();
        ctx.arc(handlePos.x, handlePos.y, 4, 0, 2 * Math.PI);
        ctx.fill();
    } else { // polygon / box
        const aabb = selectedObj.getAABB();
        ctx.strokeRect(
            aabb.min.x - 6,
            aabb.min.y - 6,
            (aabb.max.x - aabb.min.x) + 12,
            (aabb.max.y - aabb.min.y) + 12
        );
    }
}
