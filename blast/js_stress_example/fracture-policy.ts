/**
 * Fracture Policy Demo
 *
 * Demonstrates the progressive fracture system with tunable knobs for
 * balancing realism and performance. A tower is built and can be hit with
 * projectiles; sliders control how fractures are budgeted per frame.
 *
 * Click the viewport to launch projectiles at the tower.
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { buildDestructibleCore } from 'blast-stress-solver/rapier';
import {
  createDestructibleThreeBundle,
  RapierDebugRenderer,
} from 'blast-stress-solver/three';
import { buildTowerScenario } from 'blast-stress-solver/scenarios';

// ── Config ────────────────────────────────────────────────────

const CONFIG = {
  tower: {
    side: 5,
    stories: 14,
    spacing: { x: 0.42, y: 0.42, z: 0.42 },
    totalMass: 8_000,
    areaScale: 0.05,
    addDiagonals: true,
    diagScale: 0.55,
    normalizeAreas: true,
  },
  projectile: {
    radius: 0.35,
    mass: 15_000,
    speed: 30,
  },
  solver: {
    gravity: -9.81,
    materialScale: 1e8,
  },
  fracturePolicy: {
    maxFracturesPerFrame: -1,
    maxNewBodiesPerFrame: -1,
    maxColliderMigrationsPerFrame: -1,
    maxDynamicBodies: -1,
    minChildNodeCount: 1,
  },
};

// ── Three.js setup ────────────────────────────────────────────

const canvas = document.getElementById('demo-canvas') as HTMLCanvasElement;
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(canvas.clientWidth, canvas.clientHeight, false);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0e15);
scene.fog = new THREE.FogExp2(0x0b0e15, 0.015);

const camera = new THREE.PerspectiveCamera(
  55,
  canvas.clientWidth / canvas.clientHeight,
  0.1,
  200,
);
camera.position.set(7, 5, 14);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 2.5, 0);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.update();

// Lights
scene.add(new THREE.AmbientLight(0xffffff, 0.35));
const dirLight = new THREE.DirectionalLight(0xffeedd, 1.0);
dirLight.position.set(10, 18, 8);
dirLight.castShadow = true;
dirLight.shadow.mapSize.set(2048, 2048);
dirLight.shadow.camera.left = -12;
dirLight.shadow.camera.right = 12;
dirLight.shadow.camera.top = 16;
dirLight.shadow.camera.bottom = -4;
scene.add(dirLight);

// Ground
const groundGeo = new THREE.PlaneGeometry(60, 60);
const groundMat = new THREE.MeshStandardMaterial({
  color: 0x1a1e2f,
  roughness: 0.85,
  metalness: 0.1,
});
const groundMesh = new THREE.Mesh(groundGeo, groundMat);
groundMesh.rotation.x = -Math.PI / 2;
groundMesh.position.y = -0.4;
groundMesh.receiveShadow = true;
scene.add(groundMesh);

// ── Status HUD ────────────────────────────────────────────────

// Frame time tracking for FPS display
let frameTimes: number[] = [];
const MAX_FRAME_SAMPLES = 60;

function updateStatus(core: any, stepMs: number) {
  const el = (id: string) => document.getElementById(id);

  el('stat-bodies')!.textContent = String(core.getRigidBodyCount());
  el('stat-bonds')!.textContent = String(core.getActiveBondsCount());
  el('stat-projectiles')!.textContent = String(core.projectiles.length);
  const active = core.chunks.filter((c: any) => c.active).length;
  const detached = core.chunks.filter((c: any) => c.detached).length;
  el('stat-chunks')!.textContent = `${active} / ${detached} det`;

  // Frame time tracking
  frameTimes.push(stepMs);
  if (frameTimes.length > MAX_FRAME_SAMPLES) frameTimes.shift();
  const avg = frameTimes.reduce((a, b) => a + b, 0) / frameTimes.length;
  const sorted = [...frameTimes].sort((a, b) => a - b);
  const p95 = sorted[Math.min(sorted.length - 1, Math.floor(sorted.length * 0.95))];
  const max = sorted[sorted.length - 1];
  el('stat-fps')!.textContent = `${(1000 / avg).toFixed(0)} fps`;
  el('stat-step')!.textContent = `${avg.toFixed(1)}ms`;
  el('stat-p95')!.textContent = `${p95.toFixed(1)}ms`;
  el('stat-max')!.textContent = `${max.toFixed(1)}ms`;
}

// ── Main ──────────────────────────────────────────────────────

let coreRef: Awaited<ReturnType<typeof buildDestructibleCore>> | null = null;
let visualsRef: ReturnType<typeof createDestructibleThreeBundle> | null = null;
let rapierDebug: RapierDebugRenderer | null = null;
let showDebug = false;

async function initScene() {
  const scenario = buildTowerScenario(CONFIG.tower);

  // Attach fragment geometries
  const sp = scenario.spacing!;
  const modScenario = {
    ...scenario,
    parameters: {
      ...scenario.parameters,
      fragmentGeometries: scenario.nodes.map(
        () => new THREE.BoxGeometry(sp.x, sp.y, sp.z),
      ),
    },
  };

  console.log(
    `Tower: ${modScenario.nodes.length} nodes, ${modScenario.bonds.length} bonds`,
  );

  const core = await buildDestructibleCore({
    scenario: modScenario,
    gravity: CONFIG.solver.gravity,
    materialScale: CONFIG.solver.materialScale,
    debrisCollisionMode: 'noDebrisPairs',
    damage: { enabled: false },
    debrisCleanup: {
      mode: 'always',
      debrisTtlMs: 10000,
      maxCollidersForDebris: 2,
    },
    smallBodyDamping: {
      mode: 'always',
      colliderCountThreshold: 3,
      minLinearDamping: 2,
      minAngularDamping: 2,
    },
    fracturePolicy: { ...CONFIG.fracturePolicy },
  });

  const group = new THREE.Group();
  scene.add(group);

  const visuals = createDestructibleThreeBundle({
    core,
    scenario: modScenario,
    root: group,
    useBatchedMesh: true,
    batchedMeshOptions: { enableBVH: false, bvhMargin: 5 },
    includeDebugLines: true,
  });

  rapierDebug?.dispose();
  rapierDebug = new RapierDebugRenderer(scene, core.world as any, { enabled: showDebug });

  coreRef = core;
  visualsRef = visuals;
  frameTimes = [];
}

// ── Projectile shooting ───────────────────────────────────────

function shootProjectile(ndcX: number, ndcY: number) {
  const core = coreRef;
  if (!core) return;

  const raycaster = new THREE.Raycaster();
  raycaster.setFromCamera(new THREE.Vector2(ndcX, ndcY), camera);
  const dir = raycaster.ray.direction.clone().normalize();

  core.enqueueProjectile({
    position: { x: camera.position.x, y: camera.position.y, z: camera.position.z },
    velocity: {
      x: dir.x * CONFIG.projectile.speed,
      y: dir.y * CONFIG.projectile.speed,
      z: dir.z * CONFIG.projectile.speed,
    },
    radius: CONFIG.projectile.radius,
    mass: CONFIG.projectile.mass,
    ttl: 8000,
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
  visualsRef?.dispose();
  coreRef?.dispose();
  coreRef = null;
  visualsRef = null;
  await initScene();
});

document.getElementById('btn-debug')?.addEventListener('click', () => {
  showDebug = !showDebug;
  rapierDebug?.setEnabled(showDebug);
  const btn = document.getElementById('btn-debug')!;
  btn.textContent = showDebug ? '* Hide Debug' : '+ Show Debug';
});

// Config sliders
function bindSlider(id: string, obj: Record<string, any>, key: string, opts?: {
  fmt?: (v: number) => string;
  onChange?: (v: number) => void;
}) {
  const slider = document.getElementById(id) as HTMLInputElement | null;
  const display = document.getElementById(id + '-value');
  if (!slider) return;
  slider.value = String(obj[key]);
  if (display) display.textContent = opts?.fmt ? opts.fmt(obj[key]) : String(obj[key]);
  slider.addEventListener('input', () => {
    const v = parseFloat(slider.value);
    obj[key] = v;
    if (display) display.textContent = opts?.fmt ? opts.fmt(v) : String(v);
    opts?.onChange?.(v);
  });
}

function unlimitedFmt(v: number): string {
  return v < 0 ? 'unlimited' : String(Math.round(v));
}

// Tower config (needs reset)
bindSlider('cfg-side', CONFIG.tower, 'side');
bindSlider('cfg-stories', CONFIG.tower, 'stories');
bindSlider('cfg-total-mass', CONFIG.tower, 'totalMass', { fmt: (v) => v.toLocaleString() });

// Projectile config (immediate)
bindSlider('cfg-proj-radius', CONFIG.projectile, 'radius', { fmt: (v) => v.toFixed(2) });
bindSlider('cfg-proj-mass', CONFIG.projectile, 'mass', { fmt: (v) => v.toLocaleString() });
bindSlider('cfg-proj-speed', CONFIG.projectile, 'speed', { fmt: (v) => v.toFixed(0) });

// Fracture policy config (live — applied immediately via setFracturePolicy)
function applyFracturePolicy() {
  if (coreRef?.setFracturePolicy) {
    coreRef.setFracturePolicy({ ...CONFIG.fracturePolicy });
  }
}

bindSlider('cfg-max-fractures', CONFIG.fracturePolicy, 'maxFracturesPerFrame', {
  fmt: unlimitedFmt,
  onChange: applyFracturePolicy,
});
bindSlider('cfg-max-bodies', CONFIG.fracturePolicy, 'maxNewBodiesPerFrame', {
  fmt: unlimitedFmt,
  onChange: applyFracturePolicy,
});
bindSlider('cfg-max-migrations', CONFIG.fracturePolicy, 'maxColliderMigrationsPerFrame', {
  fmt: unlimitedFmt,
  onChange: applyFracturePolicy,
});
bindSlider('cfg-max-dynamic', CONFIG.fracturePolicy, 'maxDynamicBodies', {
  fmt: unlimitedFmt,
  onChange: applyFracturePolicy,
});
bindSlider('cfg-min-child', CONFIG.fracturePolicy, 'minChildNodeCount', {
  onChange: applyFracturePolicy,
});

// Solver config (needs reset)
bindSlider('cfg-gravity', CONFIG.solver, 'gravity', { fmt: (v) => v.toFixed(1) });
{
  const slider = document.getElementById('cfg-material') as HTMLInputElement | null;
  const display = document.getElementById('cfg-material-value');
  if (slider) {
    const exp = Math.log10(CONFIG.solver.materialScale);
    slider.value = String(exp);
    if (display) display.textContent = `1e${exp.toFixed(0)}`;
    slider.addEventListener('input', () => {
      const exp = parseFloat(slider.value);
      CONFIG.solver.materialScale = Math.pow(10, exp);
      if (display) display.textContent = `1e${exp.toFixed(1)}`;
    });
  }
}

// ── Render loop ───────────────────────────────────────────────

const clock = new THREE.Clock();

function loop() {
  requestAnimationFrame(loop);

  const dt = Math.min(clock.getDelta(), 1 / 30);
  controls.update();

  if (coreRef && visualsRef) {
    const t0 = performance.now();
    coreRef.step(dt);
    const stepMs = performance.now() - t0;

    visualsRef.update({
      debug: showDebug,
      updateBVH: false,
      updateProjectiles: true,
    });
    rapierDebug?.update();
    updateStatus(coreRef, stepMs);
  }

  renderer.render(scene, camera);
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
