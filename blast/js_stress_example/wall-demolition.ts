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
import { buildDestructibleCore } from 'blast-stress-solver/rapier';
import {
  createDestructibleThreeBundle,
  RapierDebugRenderer,
} from 'blast-stress-solver/three';
import type { ScenarioDesc } from 'blast-stress-solver/rapier';

// ── Scenario builder ─────────────────────────────────────────

type WallParams = {
  columns: number;
  rows: number;
  depth: number;
  spacing: { x: number; y: number; z: number };
  bondArea: number;
  mass: number;
};

function buildWallScenario(params: WallParams): ScenarioDesc {
  const { columns, rows, depth, spacing, bondArea, mass } = params;
  const nodes: ScenarioDesc['nodes'] = [];
  const bonds: ScenarioDesc['bonds'] = [];
  const gridCoordinates: Array<{ ix: number; iy: number; iz: number }> = [];

  // Total grid dimensions
  const totalCols = columns;
  const totalRows = rows + 1; // +1 for support row at bottom
  const totalDepth = depth;

  // Index helper
  const idx = (ix: number, iy: number, iz: number) =>
    iz * totalCols * totalRows + iy * totalCols + ix;

  // Create nodes
  for (let iz = 0; iz < totalDepth; iz++) {
    for (let iy = 0; iy < totalRows; iy++) {
      for (let ix = 0; ix < totalCols; ix++) {
        const isSupport = iy === 0;
        const centroid = {
          x: (ix - (totalCols - 1) / 2) * spacing.x,
          y: (iy - 1) * spacing.y, // support row at y below 0
          z: (iz - (totalDepth - 1) / 2) * spacing.z,
        };
        const volume = spacing.x * spacing.y * spacing.z;
        nodes.push({
          centroid,
          mass: isSupport ? 0 : mass,
          volume: isSupport ? 0 : volume,
        });
        gridCoordinates.push({
          ix,
          iy: isSupport ? -1 : iy - 1,
          iz,
        });
      }
    }
  }

  // Create bonds between adjacent nodes (6-connectivity)
  const offsets: [number, number, number, { x: number; y: number; z: number }][] = [
    [1, 0, 0, { x: 1, y: 0, z: 0 }],
    [0, 1, 0, { x: 0, y: 1, z: 0 }],
    [0, 0, 1, { x: 0, y: 0, z: 1 }],
  ];

  for (let iz = 0; iz < totalDepth; iz++) {
    for (let iy = 0; iy < totalRows; iy++) {
      for (let ix = 0; ix < totalCols; ix++) {
        const i = idx(ix, iy, iz);
        for (const [dx, dy, dz, normal] of offsets) {
          const nx = ix + dx;
          const ny = iy + dy;
          const nz = iz + dz;
          if (nx < totalCols && ny < totalRows && nz < totalDepth) {
            const j = idx(nx, ny, nz);
            const c0 = nodes[i].centroid;
            const c1 = nodes[j].centroid;
            bonds.push({
              node0: i,
              node1: j,
              centroid: {
                x: (c0.x + c1.x) / 2,
                y: (c0.y + c1.y) / 2,
                z: (c0.z + c1.z) / 2,
              },
              normal,
              area: bondArea,
            });
          }
        }
      }
    }
  }

  return { nodes, bonds, gridCoordinates, spacing };
}

// ── Config ────────────────────────────────────────────────────

const CONFIG = {
  wall: {
    columns: 12,
    rows: 8,
    depth: 2,
    spacing: { x: 0.5, y: 0.35, z: 0.5 },
    bondArea: 0.04,
    mass: 120,
  },
  projectile: {
    radius: 0.4,
    mass: 8000,
    speed: 25,
  },
  solver: {
    gravity: -9.81,
    materialScale: 1.0,
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
scene.background = new THREE.Color(0x0a0d13);
scene.fog = new THREE.FogExp2(0x0a0d13, 0.02);

const camera = new THREE.PerspectiveCamera(
  55,
  canvas.clientWidth / canvas.clientHeight,
  0.1,
  200,
);
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
  const scenario = buildWallScenario(CONFIG.wall);

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
      debrisTtlMs: 8000,
      maxCollidersForDebris: 2,
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
    `Wall built: ${scenario.nodes.length} nodes, ${scenario.bonds.length} bonds`,
  );
}

// ── Projectile shooting ───────────────────────────────────────

function shootProjectile(ndcX: number, ndcY: number) {
  const core = coreRef;
  if (!core) return;

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

bindSlider('cfg-columns', CONFIG.wall, 'columns');
bindSlider('cfg-rows', CONFIG.wall, 'rows');
bindSlider('cfg-bond-area', CONFIG.wall, 'bondArea', (v) => v.toFixed(3));
bindSlider('cfg-proj-radius', CONFIG.projectile, 'radius', (v) => v.toFixed(2));
bindSlider('cfg-proj-mass', CONFIG.projectile, 'mass', (v) => v.toLocaleString());
bindSlider('cfg-proj-speed', CONFIG.projectile, 'speed', (v) => v.toFixed(0));
bindSlider('cfg-gravity', CONFIG.solver, 'gravity', (v) => v.toFixed(1));
bindSlider('cfg-material', CONFIG.solver, 'materialScale', (v) => v.toFixed(2));

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
