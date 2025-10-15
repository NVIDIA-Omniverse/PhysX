import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import RAPIER from '@dimforge/rapier3d-compat';
import { RapierDebugRenderer } from './rapier-debug-renderer.js';

/**
 * Minimal “split-on-contact” demo:
 * - Fixed root body with many colliders (the “bridge”)
 * - Click to spawn balls
 * - When a contact event hits a bridge collider, that collider is detached
 *   into its own dynamic rigid body (safe two-phase migration).
 *
 * Aliasing-safe sequence:
 *  - Eventful frame: sweep → world.step(eventQueue) → drain (queue detaches)
 *  - Safe frame(s): world.step() → applyPendingDetaches() → removeDisabled()
 */

// ---------- Tunables ----------
const BRIDGE_SEGMENTS = 32;        // how many rectangular pieces
const SEGMENT_SIZE = { x: 0.6, y: 0.2, z: 2.0 };
const SEGMENT_SPACING = 0.62;      // spacing along X
const BRIDGE_Y = 0.5;              // bridge height
const BALL_RADIUS = 0.35;
const BALL_DROP_Y = 8;
let GRAVITY = -9.81;               // mutable for runtime control
const SAFE_FRAMES_AFTER_DETACH = 2;

// ---------- Global-ish demo state ----------
const state = {
  world: null,
  eventQueue: null,
  scene: null,
  camera: null,
  renderer: null,
  controls: null,
  clock: null,
  debugRenderer: null,

  // Root fixed body that owns all bridge colliders initially
  rootBodyHandle: null,

  // Segments: one mesh per collider
  segments: /** @type {Array<{
    index:number,
    localOffset: THREE.Vector3,
    size: {x:number,y:number,z:number},
    mesh: THREE.Mesh,
    colliderHandle: number,
    bodyHandle: number,
    detached: boolean
  }>} */ ([]),

  // Balls (for rendering / pruning)
  balls: /** @type {Array<{ bodyHandle:number, mesh:THREE.Mesh }>} */ ([]),

  // Tracking sets/maps for contact + migration
  colliderToSegment: /** @type {Map<number, number>} */ (new Map()),
  activeContactColliders: /** @type {Set<number>} */ (new Set()),
  disabledCollidersToRemove: /** @type {Set<number>} */ (new Set()),
  bodiesToRemove: /** @type {Set<number>} */ (new Set()),

  // Detach queue (collider handles to split out of the root body)
  pendingDetaches: /** @type {Set<number>} */ (new Set()),

  // When > 0, do safe step(s) with no events and run migrations
  safeFrames: 0,

  // Raycast helpers
  raycaster: new THREE.Raycaster(),
  ndc: new THREE.Vector2(),
  groundPlane: new THREE.Plane(new THREE.Vector3(0, 1, 0), 0) // y = 0 plane
};

// ---------- Bootstrap ----------
init().catch((e) => {
  // eslint-disable-next-line no-console
  console.error('Demo init failed:', e);
});

async function init() {
  await RAPIER.init();

  // Three.js
  const canvas = ensureCanvas();
  const { scene, camera, renderer, controls } = initThree(canvas);
  Object.assign(state, { scene, camera, renderer, controls, clock: new THREE.Clock() });

  // Rapier world
  const world = new RAPIER.World({ x: 0, y: GRAVITY, z: 0 });
  state.world = world;
  state.eventQueue = new RAPIER.EventQueue(true);

  // Scene: lights + ground
  buildLights(scene);
  buildGround(scene, world);

  // Build bridge (fixed root, many colliders)
  buildBridge(scene, world);

  // Debug renderer
  const debugRenderer = new RapierDebugRenderer(scene, world, { enabled: false });
  state.debugRenderer = debugRenderer;

  // Input: click spawns a ball at XZ under cursor (drops from BALL_DROP_Y)
  canvas.addEventListener('pointerdown', onPointerDown);

  // Wire up controls
  setupControls();

  // Start loop
  loop();
}

// ---------- UI Controls ----------
function setupControls() {
  // Gravity slider
  const gravitySlider = document.getElementById('gravity-slider');
  const gravityValue = document.getElementById('gravity-value');
  if (gravitySlider && gravityValue) {
    gravitySlider.addEventListener('input', (e) => {
      GRAVITY = parseFloat(e.target.value);
      gravityValue.textContent = GRAVITY.toFixed(2);
      if (state.world) {
        state.world.gravity = { x: 0, y: GRAVITY, z: 0 };
      }
    });
  }

  // Reset button
  const resetButton = document.getElementById('reset-bridge');
  if (resetButton) {
    resetButton.addEventListener('click', () => {
      location.reload();
    });
  }

  // Debug toggle
  const debugToggle = document.getElementById('toggle-debug');
  if (debugToggle) {
    debugToggle.addEventListener('click', () => {
      if (state.debugRenderer) {
        const enabled = state.debugRenderer.toggle();
        debugToggle.textContent = enabled ? 'Hide Debug Wireframe' : 'Show Debug Wireframe';
      }
    });
  }
}

// ---------- Scene builders ----------

function ensureCanvas() {
  let canvas = document.getElementById('bridge-canvas');
  if (!canvas) {
    canvas = document.createElement('canvas');
    canvas.id = 'bridge-canvas';
    canvas.style.width = '100vw';
    canvas.style.height = '100vh';
    document.body.style.margin = '0';
    document.body.appendChild(canvas);
  }
  return /** @type {HTMLCanvasElement} */ (canvas);
}

function initThree(canvas) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(devicePixelRatio);
  renderer.setSize(canvas.clientWidth || window.innerWidth, canvas.clientHeight || window.innerHeight, false);
  renderer.shadowMap.enabled = true;

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0a0d13);

  const camera = new THREE.PerspectiveCamera(60, (canvas.clientWidth || window.innerWidth) / (canvas.clientHeight || window.innerHeight), 0.1, 500);
  camera.position.set(0, 8, 18);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, BRIDGE_Y, 0);
  controls.enableDamping = true;

  window.addEventListener('resize', () => {
    const w = canvas.clientWidth || window.innerWidth;
    const h = canvas.clientHeight || window.innerHeight;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  });

  return { scene, camera, renderer, controls };
}

function buildLights(scene) {
  const amb = new THREE.AmbientLight(0xffffff, 0.35);
  scene.add(amb);
  const dir = new THREE.DirectionalLight(0xffffff, 0.9);
  dir.position.set(10, 16, 10);
  dir.castShadow = true;
  dir.shadow.mapSize.set(2048, 2048);
  scene.add(dir);
}

function buildGround(scene, world) {
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(60, 1, 60),
    new THREE.MeshStandardMaterial({ color: 0x1a1e2f, roughness: 0.85, metalness: 0.1 })
  );
  mesh.position.set(0, -2.5, 0);
  mesh.receiveShadow = true;
  scene.add(mesh);

  const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  const col = world.createCollider(
    RAPIER.ColliderDesc.cuboid(30, 0.5, 30).setTranslation(0, -2.5, 0),
    body
  );
  void col;
}

function buildBridge(scene, world) {
  // Fixed root body (bridge frame)
  const root = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, BRIDGE_Y, 0));
  state.rootBodyHandle = root.handle;

  // Visual materials
  const mat = new THREE.MeshStandardMaterial({ color: 0x4b6fe8, roughness: 0.4, metalness: 0.45 });

  // Build a line of segments along X, each as its own collider (on the root body)
  const halfCount = (BRIDGE_SEGMENTS - 1) * 0.5;
  for (let i = 0; i < BRIDGE_SEGMENTS; i++) {
    const x = (i - halfCount) * SEGMENT_SPACING;
    const localOffset = new THREE.Vector3(x, 0, 0);

    const halfX = SEGMENT_SIZE.x * 0.5;
    const halfY = SEGMENT_SIZE.y * 0.5;
    const halfZ = SEGMENT_SIZE.z * 0.5;

    const collider = world.createCollider(
      RAPIER.ColliderDesc.cuboid(halfX, halfY, halfZ)
        .setTranslation(localOffset.x, localOffset.y, localOffset.z)
        .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
        .setContactForceEventThreshold(0.0)
        .setFriction(0.9)
        .setRestitution(0.0),
      root
    );

    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(SEGMENT_SIZE.x, SEGMENT_SIZE.y, SEGMENT_SIZE.z),
      mat.clone()
    );
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    scene.add(mesh);

    state.segments.push({
      index: i,
      localOffset,
      size: { ...SEGMENT_SIZE },
      mesh,
      colliderHandle: collider.handle,
      bodyHandle: root.handle,
      detached: false
    });

    state.colliderToSegment.set(collider.handle, i);
    state.activeContactColliders.add(collider.handle);
  }

  // Position meshes initially
  syncSegmentsMeshes();
}

// ---------- Interaction ----------
function onPointerDown(ev) {
  const { camera, renderer, raycaster, ndc, groundPlane } = state;
  const rect = renderer.domElement.getBoundingClientRect();
  ndc.x = ((ev.clientX - rect.left) / rect.width) * 2 - 1;
  ndc.y = -((ev.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(ndc, camera);

  const point = new THREE.Vector3();
  const ok = raycaster.ray.intersectPlane(groundPlane, point);
  if (!ok) return;

  spawnBall(point.x, point.z);
}

function spawnBall(x, z) {
  const { world, scene } = state;

  const body = world.createRigidBody(
    RAPIER.RigidBodyDesc.dynamic()
      .setTranslation(x, BALL_DROP_Y, z)
      .setCanSleep(false)
      .setLinearDamping(0.01)
      .setAngularDamping(0.01)
  );
  const collider = world.createCollider(
    RAPIER.ColliderDesc.ball(BALL_RADIUS)
      .setFriction(0.6)
      .setRestitution(0.2)
      .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
      .setContactForceEventThreshold(0.0),
    body
  );
  void collider;

  const mesh = new THREE.Mesh(
    new THREE.SphereGeometry(BALL_RADIUS, 20, 20),
    new THREE.MeshStandardMaterial({ color: 0xff9147, emissive: 0x331100 })
  );
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  mesh.position.set(x, BALL_DROP_Y, z);
  scene.add(mesh);

  state.balls.push({ bodyHandle: body.handle, mesh });
}

// ---------- Main loop ----------
function loop() {
  const { world, eventQueue, renderer, scene, camera, controls, clock } = state;
  const delta = clock.getDelta();

  if (state.safeFrames > 0) {
    // SAFE STEP: no event queue, then apply migrations & removals
    try { world.step(); } catch (err) { /* eslint-disable no-console */ console.warn('Safe step failed', err); }
    state.safeFrames -= 1;

    applyPendingDetaches();
    removeDisabledHandles();

    updateMeshes();
    updateStatus();
    if (state.debugRenderer) state.debugRenderer.update();
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(loop);
    return;
  }

  // EVENTFUL STEP: pre-sweep → step(eventQueue) → drain → queue detaches
  preStepSweep();

  let stepped = false;
  try {
    world.step(eventQueue);
    stepped = true;
  } catch (err) {
    // If an eventful step fails, schedule a safe frame and reset the queue.
    // eslint-disable-next-line no-console
    console.warn('Eventful step failed, switching to safe frame', err);
    try { state.eventQueue = new RAPIER.EventQueue(true); } catch {}
    state.safeFrames = Math.max(state.safeFrames, 1);
  }
  if (!stepped) {
    updateMeshes();
    updateStatus();
    if (state.debugRenderer) state.debugRenderer.update();
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(loop);
    return;
  }

  // Drain events: queue detaches (no world mutations here!)
  eventQueue.drainContactForceEvents((ev) => {
    const h1 = ev.collider1?.();
    const h2 = ev.collider2?.();
    queueDetachIfBridgeCollider(h1);
    queueDetachIfBridgeCollider(h2);
  });

  // If any detaches were queued, plan safe frames to migrate
  if (state.pendingDetaches.size > 0) {
    state.safeFrames = Math.max(state.safeFrames, SAFE_FRAMES_AFTER_DETACH);
  }

  // Render
  updateMeshes();
  updateStatus();
  if (state.debugRenderer) state.debugRenderer.update();
  controls.update();
  renderer.render(scene, camera);

  requestAnimationFrame(loop);
}

// ---------- Migration & safety helpers ----------

function queueDetachIfBridgeCollider(handle) {
  if (handle == null) return;
  if (!state.colliderToSegment.has(handle)) return;
  // Already detached? Skip
  const seg = state.segments[state.colliderToSegment.get(handle)];
  if (!seg || seg.detached) return;
  state.pendingDetaches.add(handle);
}

function preStepSweep() {
  const { world, activeContactColliders, colliderToSegment, disabledCollidersToRemove } = state;

  // Drop any colliders that no longer exist, are disabled, or lost their parent.
  for (const h of Array.from(activeContactColliders)) {
    const c = world.getCollider(h);
    const p = c ? c.parent() : undefined;
    const b = p != null ? world.getRigidBody(p) : undefined;
    const enabled = c?.isEnabled ? c.isEnabled() : true;
    if (!c || !enabled || !b) {
      activeContactColliders.delete(h);
      colliderToSegment.delete(h);
    }
  }

  // Also purge dead handles from the map
  for (const [h] of Array.from(colliderToSegment.entries())) {
    const c = world.getCollider(h);
    const p = c ? c.parent() : undefined;
    const b = p != null ? world.getRigidBody(p) : undefined;
    const enabled = c?.isEnabled ? c.isEnabled() : true;
    if (!c || !enabled || !b) {
      colliderToSegment.delete(h);
      activeContactColliders.delete(h);
    }
  }

  // Remove any disabled colliders queued by prior migrations (extra hygiene)
  for (const h of Array.from(disabledCollidersToRemove)) {
    const c = world.getCollider(h);
    if (c) world.removeCollider(c, false);
    disabledCollidersToRemove.delete(h);
  }
}

function applyPendingDetaches() {
  if (state.pendingDetaches.size === 0) return;

  const { world, rootBodyHandle, pendingDetaches, disabledCollidersToRemove, colliderToSegment, activeContactColliders } = state;
  const root = world.getRigidBody(rootBodyHandle);
  const rootTr = root.translation();
  const rootRot = root.rotation();

  for (const h of Array.from(pendingDetaches)) {
    pendingDetaches.delete(h);
    const segIdx = colliderToSegment.get(h);
    if (segIdx == null) continue;

    const segment = state.segments[segIdx];
    if (!segment || segment.detached) continue;

    // Disable old collider on root and queue for removal
    const oldCol = world.getCollider(segment.colliderHandle);
    if (oldCol) oldCol.setEnabled(false);
    disabledCollidersToRemove.add(segment.colliderHandle);
    activeContactColliders.delete(segment.colliderHandle);
    colliderToSegment.delete(segment.colliderHandle);

    // Create a new dynamic body at the root's pose (inherits pose; velocities can be zeroed or inherited)
    const dyn = world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(rootTr.x, rootTr.y, rootTr.z)
        .setRotation(rootRot)
        .setLinearDamping(0.25)
        .setAngularDamping(0.5)
    );

    // Recreate the collider on the new body at the same local offset
    const halfX = segment.size.x * 0.5;
    const halfY = segment.size.y * 0.5;
    const halfZ = segment.size.z * 0.5;
    const newCol = world.createCollider(
      RAPIER.ColliderDesc.cuboid(halfX, halfY, halfZ)
        .setTranslation(segment.localOffset.x, segment.localOffset.y, segment.localOffset.z)
        .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
        .setContactForceEventThreshold(0.0)
        .setFriction(0.9)
        .setRestitution(0.0),
      dyn
    );

    // Update bookkeeping
    segment.bodyHandle = dyn.handle;
    segment.colliderHandle = newCol.handle;
    segment.detached = true;

    colliderToSegment.set(newCol.handle, segIdx);
    activeContactColliders.add(newCol.handle);
  }

  // We remove disabled colliders at the end of this safe frame (or next pre-sweep)
}

function removeDisabledHandles() {
  const { world, disabledCollidersToRemove, bodiesToRemove } = state;

  for (const h of Array.from(disabledCollidersToRemove)) {
    const c = world.getCollider(h);
    if (c) world.removeCollider(c, false);
    disabledCollidersToRemove.delete(h);
  }

  for (const bh of Array.from(bodiesToRemove)) {
    const b = world.getRigidBody(bh);
    if (b) world.removeRigidBody(b);
    bodiesToRemove.delete(bh);
  }
}

// ---------- Rendering sync ----------
function updateMeshes() {
  syncSegmentsMeshes();
  syncBallMeshes();
}

function updateStatus() {
  const attachedCount = state.segments.filter(s => !s.detached).length;
  const detachedCount = state.segments.filter(s => s.detached).length;
  const ballCount = state.balls.length;

  const segmentCountEl = document.getElementById('segment-count');
  const detachedCountEl = document.getElementById('detached-count');
  const ballCountEl = document.getElementById('ball-count');

  if (segmentCountEl) segmentCountEl.textContent = attachedCount;
  if (detachedCountEl) detachedCountEl.textContent = detachedCount;
  if (ballCountEl) ballCountEl.textContent = ballCount;
}

function syncSegmentsMeshes() {
  const { world, segments, rootBodyHandle } = state;

  const root = world.getRigidBody(rootBodyHandle);
  const rt = root.translation();
  const rr = root.rotation();
  const rootPos = new THREE.Vector3(rt.x, rt.y, rt.z);
  const rootQuat = new THREE.Quaternion(rr.x, rr.y, rr.z, rr.w);

  const tmp = new THREE.Vector3();
  const quat = new THREE.Quaternion();

  for (const seg of segments) {
    const body = world.getRigidBody(seg.bodyHandle);
    if (!body) continue;

    const t = body.translation();
    const r = body.rotation();

    const basePos = (seg.detached ? new THREE.Vector3(t.x, t.y, t.z) : rootPos);
    const baseQuat = (seg.detached ? new THREE.Quaternion(r.x, r.y, r.z, r.w) : rootQuat);

    seg.mesh.position.copy(basePos);
    quat.copy(baseQuat);
    seg.mesh.quaternion.copy(quat);

    // Apply local (bridge-local) offset
    tmp.copy(seg.localOffset).applyQuaternion(seg.mesh.quaternion);
    seg.mesh.position.add(tmp);
  }
}

function syncBallMeshes() {
  const { world, balls, scene } = state;
  const keep = [];

  for (const b of balls) {
    const body = world.getRigidBody(b.bodyHandle);
    if (!body) continue;

    const t = body.translation();
    const q = body.rotation();
    b.mesh.position.set(t.x, t.y, t.z);
    b.mesh.quaternion.set(q.x, q.y, q.z, q.w);

    // Simple pruning
    if (t.y < -50) {
      scene.remove(b.mesh);
      state.bodiesToRemove.add(b.bodyHandle);
    } else {
      keep.push(b);
    }
  }
  state.balls = keep;
}
