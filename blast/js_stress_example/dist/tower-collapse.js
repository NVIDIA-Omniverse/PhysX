import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { buildDestructibleCore } from "blast-stress-solver/rapier";
import {
  createDestructibleThreeBundle,
  RapierDebugRenderer,
  applyAutoBondingToScenario
} from "blast-stress-solver/three";
import { buildTowerScenario } from "blast-stress-solver/scenarios";
const CONFIG = {
  tower: {
    side: 4,
    stories: 16,
    spacing: { x: 0.42, y: 0.42, z: 0.42 },
    totalMass: 5e3,
    areaScale: 0.05,
    addDiagonals: true,
    diagScale: 0.55,
    normalizeAreas: true
  },
  projectile: {
    radius: 0.35,
    mass: 15e3,
    speed: 22
  },
  solver: {
    gravity: -9.81,
    materialScale: 1e8
  },
  autoBonds: false
};
const canvas = document.getElementById("demo-canvas");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(canvas.clientWidth, canvas.clientHeight, false);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
const scene = new THREE.Scene();
scene.background = new THREE.Color(724501);
scene.fog = new THREE.FogExp2(724501, 0.015);
const camera = new THREE.PerspectiveCamera(
  55,
  canvas.clientWidth / canvas.clientHeight,
  0.1,
  200
);
camera.position.set(6, 5, 12);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 2.5, 0);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.update();
scene.add(new THREE.AmbientLight(16777215, 0.35));
const dirLight = new THREE.DirectionalLight(16772829, 1);
dirLight.position.set(10, 18, 8);
dirLight.castShadow = true;
dirLight.shadow.mapSize.set(2048, 2048);
dirLight.shadow.camera.left = -12;
dirLight.shadow.camera.right = 12;
dirLight.shadow.camera.top = 16;
dirLight.shadow.camera.bottom = -4;
scene.add(dirLight);
const groundGeo = new THREE.PlaneGeometry(60, 60);
const groundMat = new THREE.MeshStandardMaterial({
  color: 1711663,
  roughness: 0.85,
  metalness: 0.1
});
const groundMesh = new THREE.Mesh(groundGeo, groundMat);
groundMesh.rotation.x = -Math.PI / 2;
groundMesh.position.y = -0.4;
groundMesh.receiveShadow = true;
scene.add(groundMesh);
function updateStatus(core) {
  const el = (id) => document.getElementById(id);
  el("stat-bodies").textContent = String(core.getRigidBodyCount());
  el("stat-bonds").textContent = String(core.getActiveBondsCount());
  el("stat-projectiles").textContent = String(core.projectiles.length);
  const active = core.chunks.filter((c) => c.active).length;
  const detached = core.chunks.filter((c) => c.detached).length;
  el("stat-chunks").textContent = `${active} / ${detached} detached`;
}
let coreRef = null;
let visualsRef = null;
let rapierDebug = null;
let showDebug = false;
async function initScene() {
  let scenario = buildTowerScenario(CONFIG.tower);
  const sp = scenario.spacing;
  const fragmentGeometries = scenario.nodes.map(
    () => new THREE.BoxGeometry(sp.x, sp.y, sp.z)
  );
  scenario = {
    ...scenario,
    parameters: { ...scenario.parameters, fragmentGeometries }
  };
  if (CONFIG.autoBonds) {
    scenario = await applyAutoBondingToScenario(scenario, { mode: "average", maxSeparation: 0.01 });
  }
  console.log(
    `Tower: ${scenario.nodes.length} nodes, ${scenario.bonds.length} bonds` + (CONFIG.autoBonds ? " (auto-bonded)" : " (manual)")
  );
  const core = await buildDestructibleCore({
    scenario,
    gravity: CONFIG.solver.gravity,
    materialScale: CONFIG.solver.materialScale,
    debrisCollisionMode: "noDebrisPairs",
    damage: {
      enabled: false
    },
    debrisCleanup: {
      mode: "always",
      debrisTtlMs: 1e4,
      maxCollidersForDebris: 2
    },
    smallBodyDamping: {
      mode: "always",
      colliderCountThreshold: 3,
      minLinearDamping: 2,
      minAngularDamping: 2
    }
  });
  const group = new THREE.Group();
  scene.add(group);
  const visuals = createDestructibleThreeBundle({
    core,
    scenario,
    root: group,
    useBatchedMesh: true,
    batchedMeshOptions: { enableBVH: false, bvhMargin: 5 },
    includeDebugLines: true
  });
  rapierDebug?.dispose();
  rapierDebug = new RapierDebugRenderer(scene, core.world, { enabled: showDebug });
  coreRef = core;
  visualsRef = visuals;
}
function shootProjectile(ndcX, ndcY) {
  const core = coreRef;
  if (!core) return;
  const raycaster = new THREE.Raycaster();
  raycaster.setFromCamera(new THREE.Vector2(ndcX, ndcY), camera);
  const dir = raycaster.ray.direction.clone().normalize();
  core.enqueueProjectile({
    position: {
      x: camera.position.x,
      y: camera.position.y,
      z: camera.position.z
    },
    velocity: {
      x: dir.x * CONFIG.projectile.speed,
      y: dir.y * CONFIG.projectile.speed,
      z: dir.z * CONFIG.projectile.speed
    },
    radius: CONFIG.projectile.radius,
    mass: CONFIG.projectile.mass,
    ttl: 8e3
  });
}
canvas.addEventListener("click", (e) => {
  const rect = canvas.getBoundingClientRect();
  const ndcX = (e.clientX - rect.left) / rect.width * 2 - 1;
  const ndcY = -((e.clientY - rect.top) / rect.height) * 2 + 1;
  shootProjectile(ndcX, ndcY);
});
document.getElementById("btn-reset")?.addEventListener("click", async () => {
  visualsRef?.dispose();
  coreRef?.dispose();
  coreRef = null;
  visualsRef = null;
  await initScene();
});
document.getElementById("btn-debug")?.addEventListener("click", () => {
  showDebug = !showDebug;
  rapierDebug?.setEnabled(showDebug);
  const btn = document.getElementById("btn-debug");
  btn.textContent = showDebug ? "\u25C8 Hide Debug" : "\u25C7 Show Debug";
});
function bindSlider(id, obj, key, fmt) {
  const slider = document.getElementById(id);
  const display = document.getElementById(id + "-value");
  if (!slider) return;
  slider.value = String(obj[key]);
  if (display) display.textContent = fmt ? fmt(obj[key]) : String(obj[key]);
  slider.addEventListener("input", () => {
    const v = parseFloat(slider.value);
    obj[key] = v;
    if (display) display.textContent = fmt ? fmt(v) : String(v);
  });
}
bindSlider("cfg-side", CONFIG.tower, "side");
bindSlider("cfg-stories", CONFIG.tower, "stories");
bindSlider("cfg-area-scale", CONFIG.tower, "areaScale", (v) => v.toFixed(3));
bindSlider("cfg-total-mass", CONFIG.tower, "totalMass", (v) => v.toLocaleString());
bindSlider("cfg-proj-radius", CONFIG.projectile, "radius", (v) => v.toFixed(2));
bindSlider("cfg-proj-mass", CONFIG.projectile, "mass", (v) => v.toLocaleString());
bindSlider("cfg-proj-speed", CONFIG.projectile, "speed", (v) => v.toFixed(0));
bindSlider("cfg-gravity", CONFIG.solver, "gravity", (v) => v.toFixed(1));
{
  const slider = document.getElementById("cfg-material");
  const display = document.getElementById("cfg-material-value");
  if (slider) {
    const exp = Math.log10(CONFIG.solver.materialScale);
    slider.value = String(exp);
    if (display) display.textContent = `1e${exp.toFixed(0)}`;
    slider.addEventListener("input", () => {
      const exp2 = parseFloat(slider.value);
      CONFIG.solver.materialScale = Math.pow(10, exp2);
      if (display) display.textContent = `1e${exp2.toFixed(1)}`;
    });
  }
}
{
  const checkbox = document.getElementById("cfg-auto-bonds");
  if (checkbox) {
    checkbox.checked = CONFIG.autoBonds;
    checkbox.addEventListener("change", () => {
      CONFIG.autoBonds = checkbox.checked;
    });
  }
}
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
      updateProjectiles: true
    });
    rapierDebug?.update();
    updateStatus(coreRef);
  }
  renderer.render(scene, camera);
}
function onResize() {
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener("resize", onResize);
initScene().then(() => loop());
