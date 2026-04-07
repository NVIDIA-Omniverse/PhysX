/**
 * Tower Collapse Demo
 *
 * Showcases the high-level blast-stress-solver/rapier and blast-stress-solver/three
 * APIs with a tall tower that collapses under its own weight or projectile impacts.
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
    side: 4,
    stories: 16,
    spacing: { x: 0.42, y: 0.42, z: 0.42 },
    totalMass: 5_000,
    areaScale: 0.05,
    addDiagonals: true,
    diagScale: 0.55,
    normalizeAreas: true,
  },
  projectile: {
    radius: 0.35,
    mass: 15_000,
    speed: 22,
  },
  solver: {
    gravity: -9.81,
    materialScale: 1e8,
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
camera.position.set(6, 5, 12);

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

function updateStatus(core: any) {
  const el = (id: string) => document.getElementById(id);
  el('stat-bodies')!.textContent = String(core.getRigidBodyCount());
  el('stat-bonds')!.textContent = String(core.getActiveBondsCount());
  el('stat-projectiles')!.textContent = String(core.projectiles.length);
  const active = core.chunks.filter((c: any) => c.active).length;
  const detached = core.chunks.filter((c: any) => c.detached).length;
  el('stat-chunks')!.textContent = `${active} / ${detached} detached`;
}

// ── Main ──────────────────────────────────────────────────────

let coreRef: Awaited<ReturnType<typeof buildDestructibleCore>> | null = null;
let visualsRef: ReturnType<typeof createDestructibleThreeBundle> | null = null;
let rapierDebug: RapierDebugRenderer | null = null;
let showDebug = false;

async function initScene() {
  const scenario = buildTowerScenario(CONFIG.tower);

  const core = await buildDestructibleCore({
    scenario,
    gravity: CONFIG.solver.gravity,
    materialScale: CONFIG.solver.materialScale,
    debrisCollisionMode: 'noDebrisPairs',
    damage: {
      enabled: true,
      autoDetachOnDestroy: true,
      autoCleanupPhysics: true,
    },
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
  rapierDebug = new RapierDebugRenderer(scene, core.world as any, { enabled: showDebug });

  coreRef = core;
  visualsRef = visuals;

  console.log(
    `Tower built: ${scenario.nodes.length} nodes, ${scenario.bonds.length} bonds`,
  );
}

// ── Projectile shooting ───────────────────────────────────────

function shootProjectile(ndcX: number, ndcY: number) {
  const core = coreRef;
  if (!core) return;

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
  btn.textContent = showDebug ? '◈ Hide Debug' : '◇ Show Debug';
});

// Config sliders
function bindSlider(id: string, obj: Record<string, any>, key: string, fmt?: (v: number) => string) {
  const slider = document.getElementById(id) as HTMLInputElement | null;
  const display = document.getElementById(id + '-value');
  if (!slider) return;
  slider.value = String(obj[key]);
  if (display) display.textContent = fmt ? fmt(obj[key]) : String(obj[key]);
  slider.addEventListener('input', () => {
    const v = parseFloat(slider.value);
    obj[key] = v;
    if (display) display.textContent = fmt ? fmt(v) : String(v);
  });
}

bindSlider('cfg-side', CONFIG.tower, 'side');
bindSlider('cfg-stories', CONFIG.tower, 'stories');
bindSlider('cfg-area-scale', CONFIG.tower, 'areaScale', (v) => v.toFixed(3));
bindSlider('cfg-total-mass', CONFIG.tower, 'totalMass', (v) => v.toLocaleString());
bindSlider('cfg-proj-radius', CONFIG.projectile, 'radius', (v) => v.toFixed(2));
bindSlider('cfg-proj-mass', CONFIG.projectile, 'mass', (v) => v.toLocaleString());
bindSlider('cfg-proj-speed', CONFIG.projectile, 'speed', (v) => v.toFixed(0));
bindSlider('cfg-gravity', CONFIG.solver, 'gravity', (v) => v.toFixed(1));
// Material scale uses a log slider: slider value is the exponent (log10)
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
    coreRef.step(dt);
    visualsRef.update({
      debug: showDebug,
      updateBVH: false,
      updateProjectiles: true,
    });
    rapierDebug?.update();
    updateStatus(coreRef);
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
