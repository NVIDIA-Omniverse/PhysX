/**
 * Wall Demolition Demo
 *
 * Showcases the high-level blast-stress-solver/rapier and blast-stress-solver/three
 * APIs: buildDestructibleCore + createDestructibleThreeBundle.
 *
 * Click the viewport to launch projectiles at a destructible brick wall.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import Stats from 'three/addons/libs/stats.module.js';
import { buildDestructibleCore } from 'blast-stress-solver/rapier';
import { createDestructibleThreeBundle, RapierDebugRenderer, applyAutoBondingToScenario, } from 'blast-stress-solver/three';
import { buildWallScenario } from 'blast-stress-solver/scenarios';
// ── Config ────────────────────────────────────────────────────
const CONFIG = {
    wall: {
        span: 6.0,
        height: 3.0,
        thickness: 0.32,
        spanSegments: 12,
        heightSegments: 6,
        layers: 1,
        deckMass: 10000,
        areaScale: 0.05,
        addDiagonals: false,
        diagScale: 0.75,
        normalizeAreas: true,
    },
    projectile: {
        radius: 0.35,
        mass: 15000,
        speed: 20,
    },
    solver: {
        gravity: -9.81,
        materialScale: 1e8,
    },
    physics: {
        debrisCollisionMode: 'all',
        friction: 0.25,
        restitution: 0.0,
        contactForceScale: 30,
        skipSingleBodies: false,
    },
    optimization: {
        smallBodyDampingMode: 'off',
        debrisCleanupMode: 'always',
        debrisTtlMs: 8000,
        maxCollidersForDebris: 2,
    },
    autoBonds: false,
};
// ── Three.js setup ────────────────────────────────────────────
const canvas = document.getElementById('demo-canvas');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(canvas.clientWidth, canvas.clientHeight, false);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0a0d13);
scene.fog = new THREE.FogExp2(0x0a0d13, 0.02);
const camera = new THREE.PerspectiveCamera(55, canvas.clientWidth / canvas.clientHeight, 0.1, 200);
camera.position.set(0, 3, 12);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 1.5, 0);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.update();
// Lights
const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
scene.add(ambientLight);
const dirLight = new THREE.DirectionalLight(0xffeedd, 1.0);
dirLight.position.set(8, 14, 10);
dirLight.castShadow = true;
dirLight.shadow.mapSize.set(2048, 2048);
dirLight.shadow.camera.left = -15;
dirLight.shadow.camera.right = 15;
dirLight.shadow.camera.top = 15;
dirLight.shadow.camera.bottom = -5;
scene.add(dirLight);
// Ground plane
const groundGeo = new THREE.PlaneGeometry(60, 60);
const groundMat = new THREE.MeshStandardMaterial({
    color: 0x1a1e2f,
    roughness: 0.85,
    metalness: 0.1,
});
const groundMesh = new THREE.Mesh(groundGeo, groundMat);
groundMesh.rotation.x = -Math.PI / 2;
groundMesh.position.y = -0.35;
groundMesh.receiveShadow = true;
scene.add(groundMesh);
// ── Stats panel (FPS / MS / MB) ───────────────────────────────
const stats = new Stats();
stats.dom.style.position = 'absolute';
stats.dom.style.top = '0';
stats.dom.style.left = '0';
document.querySelector('.viewport')?.appendChild(stats.dom);
// ── Perf tracking ─────────────────────────────────────────────
let _physicsMs = 0;
let _renderMs = 0;
const EMA = 0.12; // exponential moving-average smoothing factor
function updatePerfStats() {
    const el = (id) => document.getElementById(id);
    el('stat-physics-ms').textContent = _physicsMs.toFixed(1) + ' ms';
    el('stat-render-ms').textContent = _renderMs.toFixed(1) + ' ms';
    el('stat-draw-calls').textContent = String(renderer.info.render.calls);
    el('stat-triangles').textContent = renderer.info.render.triangles.toLocaleString();
}
// ── Status HUD ────────────────────────────────────────────────
function updateStatus(core) {
    const el = (id) => document.getElementById(id);
    el('stat-bodies').textContent = String(core.getRigidBodyCount());
    el('stat-bonds').textContent = String(core.getActiveBondsCount());
    el('stat-projectiles').textContent = String(core.projectiles.length);
    const active = core.chunks.filter((c) => c.active).length;
    const detached = core.chunks.filter((c) => c.detached).length;
    el('stat-chunks').textContent = `${active} / ${detached} detached`;
}
// ── Main ──────────────────────────────────────────────────────
let coreRef = null;
let visualsRef = null;
let rapierDebug = null;
let showDebug = false;
async function initScene() {
    let scenario = buildWallScenario(CONFIG.wall);
    // Attach fragment geometries for auto-bonding support
    const sp = scenario.spacing;
    const fragmentGeometries = scenario.nodes.map(() => new THREE.BoxGeometry(sp.x, sp.y, sp.z));
    scenario = {
        ...scenario,
        parameters: { ...scenario.parameters, fragmentGeometries },
    };
    // Auto-bonding: replace manual grid bonds with geometry-derived bonds
    if (CONFIG.autoBonds) {
        scenario = await applyAutoBondingToScenario(scenario, { mode: 'average', maxSeparation: 0.01 });
    }
    console.log(`Wall: ${scenario.nodes.length} nodes, ${scenario.bonds.length} bonds` +
        (CONFIG.autoBonds ? ' (auto-bonded)' : ' (manual)'));
    console.log('[wall-demolition] buildDestructibleCore config:', {
        debrisCollisionMode: CONFIG.physics.debrisCollisionMode,
        friction: CONFIG.physics.friction,
        restitution: CONFIG.physics.restitution,
        contactForceScale: CONFIG.physics.contactForceScale,
        skipSingleBodies: CONFIG.physics.skipSingleBodies,
        smallBodyDampingMode: CONFIG.optimization.smallBodyDampingMode,
        debrisCleanupMode: CONFIG.optimization.debrisCleanupMode,
    });
    const core = await buildDestructibleCore({
        scenario,
        gravity: CONFIG.solver.gravity,
        materialScale: CONFIG.solver.materialScale,
        friction: CONFIG.physics.friction,
        restitution: CONFIG.physics.restitution,
        contactForceScale: CONFIG.physics.contactForceScale,
        debrisCollisionMode: CONFIG.physics.debrisCollisionMode,
        skipSingleBodies: CONFIG.physics.skipSingleBodies,
        damage: {
            enabled: false,
        },
        smallBodyDamping: {
            mode: CONFIG.optimization.smallBodyDampingMode,
        },
        debrisCleanup: {
            mode: CONFIG.optimization.debrisCleanupMode,
            debrisTtlMs: CONFIG.optimization.debrisTtlMs,
            maxCollidersForDebris: CONFIG.optimization.maxCollidersForDebris,
        },
    });
    const group = new THREE.Group();
    scene.add(group);
    const visuals = createDestructibleThreeBundle({
        core,
        scenario,
        root: group,
        useBatchedMesh: true,
        batchedMeshOptions: { enableBVH: false, bvhMargin: 5 },
        includeDebugLines: true,
    });
    // Rapier collider wireframe overlay
    rapierDebug?.dispose();
    rapierDebug = new RapierDebugRenderer(scene, core.world, { enabled: showDebug });
    coreRef = core;
    visualsRef = visuals;
}
// ── Projectile shooting ───────────────────────────────────────
function shootProjectile(ndcX, ndcY) {
    const core = coreRef;
    if (!core)
        return;
    // Ray from camera through click point
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(new THREE.Vector2(ndcX, ndcY), camera);
    const dir = raycaster.ray.direction.clone().normalize();
    core.enqueueProjectile({
        position: {
            x: camera.position.x,
            y: camera.position.y,
            z: camera.position.z,
        },
        velocity: {
            x: dir.x * CONFIG.projectile.speed,
            y: dir.y * CONFIG.projectile.speed,
            z: dir.z * CONFIG.projectile.speed,
        },
        radius: CONFIG.projectile.radius,
        mass: CONFIG.projectile.mass,
        ttl: 6000,
    });
}
canvas.addEventListener('click', (e) => {
    const rect = canvas.getBoundingClientRect();
    const ndcX = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    const ndcY = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    shootProjectile(ndcX, ndcY);
});
// ── UI wiring ─────────────────────────────────────────────────
document.getElementById('btn-reset')?.addEventListener('click', async () => {
    // Tear down
    visualsRef?.dispose();
    coreRef?.dispose();
    coreRef = null;
    visualsRef = null;
    // Rebuild
    await initScene();
});
document.getElementById('btn-debug')?.addEventListener('click', () => {
    showDebug = !showDebug;
    rapierDebug?.setEnabled(showDebug);
    const btn = document.getElementById('btn-debug');
    btn.textContent = showDebug ? '◈ Hide Debug' : '◇ Show Debug';
});
// Config sliders
function bindSlider(id, obj, key, fmt) {
    const slider = document.getElementById(id);
    const display = document.getElementById(id + '-value');
    if (!slider)
        return;
    slider.value = String(obj[key]);
    if (display)
        display.textContent = fmt ? fmt(obj[key]) : String(obj[key]);
    slider.addEventListener('input', () => {
        const v = parseFloat(slider.value);
        obj[key] = v;
        if (display)
            display.textContent = fmt ? fmt(v) : String(v);
    });
}
function bindSelect(id, obj, key, onChange) {
    const select = document.getElementById(id);
    if (!select)
        return;
    select.value = String(obj[key]);
    select.addEventListener('change', () => {
        obj[key] = select.value;
        onChange?.(select.value);
    });
}
function bindCheckbox(id, obj, key, onChange) {
    const checkbox = document.getElementById(id);
    if (!checkbox)
        return;
    checkbox.checked = !!obj[key];
    checkbox.addEventListener('change', () => {
        obj[key] = checkbox.checked;
        onChange?.(checkbox.checked);
    });
}
bindSlider('cfg-columns', CONFIG.wall, 'spanSegments');
bindSlider('cfg-rows', CONFIG.wall, 'heightSegments');
bindSlider('cfg-area-scale', CONFIG.wall, 'areaScale', (v) => v.toFixed(3));
bindSlider('cfg-total-mass', CONFIG.wall, 'deckMass', (v) => v.toLocaleString());
bindSlider('cfg-proj-radius', CONFIG.projectile, 'radius', (v) => v.toFixed(2));
bindSlider('cfg-proj-mass', CONFIG.projectile, 'mass', (v) => v.toLocaleString());
bindSlider('cfg-proj-speed', CONFIG.projectile, 'speed', (v) => v.toFixed(0));
bindSlider('cfg-gravity', CONFIG.solver, 'gravity', (v) => v.toFixed(1));
// Material scale uses a log slider: slider value is the exponent (log10)
{
    const slider = document.getElementById('cfg-material');
    const display = document.getElementById('cfg-material-value');
    if (slider) {
        const exp = Math.log10(CONFIG.solver.materialScale);
        slider.value = String(exp);
        if (display)
            display.textContent = `1e${exp.toFixed(0)}`;
        slider.addEventListener('input', () => {
            const exp = parseFloat(slider.value);
            CONFIG.solver.materialScale = Math.pow(10, exp);
            if (display)
                display.textContent = `1e${exp.toFixed(1)}`;
        });
    }
}
// Auto-bonds toggle
{
    const checkbox = document.getElementById('cfg-auto-bonds');
    if (checkbox) {
        checkbox.checked = CONFIG.autoBonds;
        checkbox.addEventListener('change', () => {
            CONFIG.autoBonds = checkbox.checked;
        });
    }
}
// Physics controls
bindSelect('cfg-debris-collision', CONFIG.physics, 'debrisCollisionMode', (v) => {
    coreRef?.setDebrisCollisionMode(v);
});
bindSlider('cfg-friction', CONFIG.physics, 'friction', (v) => v.toFixed(2));
bindSlider('cfg-restitution', CONFIG.physics, 'restitution', (v) => v.toFixed(2));
bindSlider('cfg-contact-force', CONFIG.physics, 'contactForceScale', (v) => v.toFixed(0));
bindCheckbox('cfg-skip-single', CONFIG.physics, 'skipSingleBodies');
// Optimization controls
bindSelect('cfg-damping-mode', CONFIG.optimization, 'smallBodyDampingMode', (v) => {
    coreRef?.setSmallBodyDamping({ mode: v });
});
bindSelect('cfg-cleanup-mode', CONFIG.optimization, 'debrisCleanupMode', (v) => {
    coreRef?.setDebrisCleanup({ mode: v, debrisTtlMs: CONFIG.optimization.debrisTtlMs });
});
bindSlider('cfg-debris-ttl', CONFIG.optimization, 'debrisTtlMs', (v) => (v / 1000).toFixed(1) + 's');
bindSlider('cfg-max-debris-colliders', CONFIG.optimization, 'maxCollidersForDebris', (v) => v.toFixed(0));
// ── Render loop ───────────────────────────────────────────────
const clock = new THREE.Clock();
function loop() {
    requestAnimationFrame(loop);
    stats.begin();
    const dt = Math.min(clock.getDelta(), 1 / 30);
    controls.update();
    if (coreRef && visualsRef) {
        const t0 = performance.now();
        coreRef.step(dt);
        _physicsMs += ((performance.now() - t0) - _physicsMs) * EMA;
        visualsRef.update({
            debug: showDebug,
            updateBVH: false,
            updateProjectiles: true,
        });
        rapierDebug?.update();
        updateStatus(coreRef);
    }
    const t1 = performance.now();
    renderer.render(scene, camera);
    _renderMs += ((performance.now() - t1) - _renderMs) * EMA;
    updatePerfStats();
    stats.end();
}
// ── Resize ────────────────────────────────────────────────────
function onResize() {
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
}
window.addEventListener('resize', onResize);
// ── Boot ──────────────────────────────────────────────────────
initScene().then(() => loop());
