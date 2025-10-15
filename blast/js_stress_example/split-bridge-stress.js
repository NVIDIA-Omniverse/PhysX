import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import RAPIER from '@dimforge/rapier3d-compat';
import {
  loadStressSolver,
  ExtForceMode,       // For clarity when adding forces
  ExtDebugMode       // Not used here, but handy to enable quick debug lines later
} from './stress.js';

/**
 * Minimal “split-on-stress” bridge:
 * - Fixed root body with many colliders (the “bridge”)
 * - Click to spawn balls
 * - Contacts → solver forces; solver decides if/where to fracture
 * - Split events are applied safely (two-phase) to migrate colliders to new rigid bodies
 *
 * Aliasing-safe sequence:
 *  - EVENTFUL frame: pre-sweep → world.step(eventQueue) → drain events (apply forces to solver)
 *      → add gravity → solver.update → if overstressed: generate+apply fracture commands
 *      → queue split migrations
 *  - SAFE frame(s): world.step() → apply pending migrations (create bodies, migrate colliders)
 *      → remove disabled colliders/bodies → render
 */

// ---------- Tunables ----------
const BRIDGE_SEGMENTS = 32;        // number of rectangular pieces
const SEGMENT_SIZE = { x: 0.6, y: 0.2, z: 2.0 };
const SEGMENT_SPACING = 0.62;      // spacing along X
const BRIDGE_Y = 0.5;              // bridge height
const BALL_RADIUS = 0.35;
const BALL_DROP_Y = 8;
const GRAVITY = -9.81;
const SAFE_FRAMES_AFTER_SPLIT = 2;

// Stress model (keep limits modest so impacts can cause fractures)
const NODE_MASS_KG = 100;
const NODE_VOLUME_M3 = SEGMENT_SIZE.x * SEGMENT_SIZE.y * SEGMENT_SIZE.z;
const BOND_AREA_M2 = SEGMENT_SIZE.y * SEGMENT_SIZE.z; // contact face area between segments

// ---------- Demo state ----------
const state = {
  // Three/Rapier
  world: null,
  eventQueue: null,
  scene: null,
  camera: null,
  renderer: null,
  controls: null,
  clock: null,

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

  // Contact→node mapping & housekeeping
  colliderToSegment: /** @type {Map<number, number>} */ (new Map()),
  activeContactColliders: /** @type {Set<number>} */ (new Set()),
  disabledCollidersToRemove: /** @type {Set<number>} */ (new Set()),
  bodiesToRemove: /** @type {Set<number>} */ (new Set()),

  // Two-phase split migration queues
  pendingBodiesToCreate: /** @type {Array<{ actorIndex:number, inheritFromBodyHandle:number, nodes:number[] }>} */ ([]),
  pendingColliderMigrations: /** @type {Array<{ nodeIndex:number, targetBodyHandle:number }>} */ ([]),

  // Safe frame countdown after any split
  safeFrames: 0,

  // Raycast helpers
  raycaster: new THREE.Raycaster(),
  ndc: new THREE.Vector2(),
  groundPlane: new THREE.Plane(new THREE.Vector3(0, 1, 0), 0), // y = 0 plane

  // ------------ ExtStressSolver integration ------------
  runtime: null,
  solver: null,
  solverSettings: null,
  // actorIndex -> { bodyHandle }
  actorMap: /** @type {Map<number, { bodyHandle:number }>} */ (new Map()),
};

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

  // Build the stress solver model matching the bridge geometry
  await buildStressModel();

  // Input: click spawns a ball at XZ under cursor (drops from BALL_DROP_Y)
  canvas.addEventListener('pointerdown', onPointerDown);

  // Start loop
  loop();
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

// ---------- Stress solver model ----------
async function buildStressModel() {
  state.runtime = await loadStressSolver();

  // Nodes: one per segment, at the segment’s centroid (bridge-local frame)
  const nodes = state.segments.map((seg) => ({
    centroid: { x: seg.localOffset.x, y: seg.localOffset.y, z: seg.localOffset.z },
    mass: NODE_MASS_KG,
    volume: NODE_VOLUME_M3
  }));

  // Bonds: connect neighbors along X (simple 1D chain)
  const bonds = [];
  for (let i = 0; i < BRIDGE_SEGMENTS - 1; i++) {
    const a = state.segments[i].localOffset;
    const b = state.segments[i + 1].localOffset;
    bonds.push({
      centroid: { x: (a.x + b.x) * 0.5, y: (a.y + b.y) * 0.5, z: (a.z + b.z) * 0.5 },
      normal:   { x: 1, y: 0, z: 0 },  // not critical for the demo
      area: BOND_AREA_M2,
      node0: i,
      node1: i + 1
    });
  }

  // Settings: start from defaults and pick reasonable thresholds
  const settings = state.runtime.defaultExtSettings();
  settings.maxSolverIterationsPerFrame = 32;
  // Keep limits low-ish so impacts can cause failures
  settings.compressionElasticLimit = 1.0e4;
  settings.compressionFatalLimit   = 2.0e4;
  settings.tensionElasticLimit     = 1.0e4;
  settings.tensionFatalLimit       = 2.0e4;
  settings.shearElasticLimit       = 8.0e3;
  settings.shearFatalLimit         = 1.6e4;

  state.solverSettings = settings;
  state.solver = state.runtime.createExtSolver({ nodes, bonds, settings });

  // Actor 0 (the only one initially) maps to the root rigid body
  state.actorMap.clear();
  state.actorMap.set(0, { bodyHandle: state.rootBodyHandle });
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
  const BALL_MASS = 100_000_000.0; // kg

  const body = world.createRigidBody(
    RAPIER.RigidBodyDesc.dynamic()
      .setTranslation(x, BALL_DROP_Y, z)
      .setCanSleep(false)
      .setLinearDamping(0.01)
      .setAngularDamping(0.01)
  );
  const collider = world.createCollider(
    RAPIER.ColliderDesc.ball(BALL_RADIUS)
      .setMass(BALL_MASS)
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
    try { world.step(); } catch (err) { console.warn('Safe step failed', err); }
    state.safeFrames -= 1;

    applyPendingMigrations();
    removeDisabledHandles();

    updateMeshes();
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(loop);
    return;
  }

  // EVENTFUL STEP: pre-sweep → step(eventQueue) → drain and apply forces to solver
  preStepSweep();

  let stepped = false;
  try {
    world.step(eventQueue);
    stepped = true;
  } catch (err) {
    console.warn('Eventful step failed, switching to safe frame', err);
    try { state.eventQueue = new RAPIER.EventQueue(true); } catch {}
    state.safeFrames = Math.max(state.safeFrames, 1);
  }
  if (!stepped) {
    updateMeshes();
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(loop);
    return;
  }

  // Drain contact events and feed the solver with forces (root-local frame)
  drainContactsAndApplyForces();

  // Gravity as acceleration
  state.solver.addGravity({ x: 0, y: GRAVITY, z: 0 });

  // Solve
  state.solver.update();

  // Fracture + split (solver decides if anything should break)
  if (state.solver.overstressedBondCount() > 0) {
    console.log('Overstressed bond count:', state.solver.overstressedBondCount());

    const perActor = state.solver.generateFractureCommandsPerActor();
    console.log('Fracture commands per actor:', perActor);

    if (Array.isArray(perActor) && perActor.length > 0) {
      const splitEvents = state.solver.applyFractureCommands(perActor);
      console.log('Split events:', splitEvents);

      if (Array.isArray(splitEvents) && splitEvents.length > 0) {
        queueSplitResults(splitEvents);
        // Schedule safe frames to perform Rapier mutations
        state.safeFrames = Math.max(state.safeFrames, SAFE_FRAMES_AFTER_SPLIT);
      }
    }
  }

  // Render
  updateMeshes();
  controls.update();
  renderer.render(scene, camera);

  requestAnimationFrame(loop);
}

// ---------- Contact → Solver ----------
function drainContactsAndApplyForces() {
  const { world, eventQueue, colliderToSegment, activeContactColliders, rootBodyHandle } = state;

  // Root transform (to convert world vectors to solver's local bridge frame)
  const rootBody = world.getRigidBody(rootBodyHandle);
  if (!rootBody) return;
  const rq = new THREE.Quaternion(
    rootBody.rotation().x,
    rootBody.rotation().y,
    rootBody.rotation().z,
    rootBody.rotation().w
  );
  const qInv = rq.clone().invert();

  // Simple accumulator per collider
  const acc = new Map(); // handle -> { force:THREE.Vector3, point:THREE.Vector3 }
  const add = (handle, direction, totalForce, worldPoint) => {
    if (handle == null) return;
    if (!colliderToSegment.has(handle)) return;
    if (!activeContactColliders.has(handle)) return;

    const f = new THREE.Vector3(
      (totalForce.x ?? 0) * direction,
      (totalForce.y ?? 0) * direction,
      (totalForce.z ?? 0) * direction
    );
    const p = new THREE.Vector3(worldPoint.x ?? 0, worldPoint.y ?? 0, worldPoint.z ?? 0);

    const prev = acc.get(handle);
    if (prev) {
      prev.force.add(f);
      prev.point.copy(p); // keep last point
    } else {
      acc.set(handle, { force: f, point: p });
    }
  };

  eventQueue.drainContactForceEvents((ev) => {
    const tf = ev.totalForce?.();
    const mag = ev.totalForceMagnitude?.();
    if (!tf || !(mag > 0)) return;

    // Prefer explicit contact points; fall back to collider’s parent body position if absent
    const wp = ev.worldContactPoint ? ev.worldContactPoint() : undefined;
    const wp2 = ev.worldContactPoint2 ? ev.worldContactPoint2() : undefined;

    const h1 = ev.collider1?.();
    const h2 = ev.collider2?.();

    const point1 = wp ?? wp2 ?? fallbackPoint(world, h1);
    const point2 = wp2 ?? wp ?? fallbackPoint(world, h2);

    add(h1, +1, tf, point1);
    add(h2, -1, tf, point2);
  });

  // Apply to solver in root-local coordinates
  acc.forEach((val, handle) => {
    const nodeIndex = colliderToSegment.get(handle);
    if (nodeIndex == null) return;

    const localForce = val.force.clone().applyQuaternion(qInv);
    const localPoint = val.point.clone().applyQuaternion(qInv);

    state.solver.addForce(
      nodeIndex,
      { x: localPoint.x, y: localPoint.y, z: localPoint.z },
      { x: localForce.x, y: localForce.y, z: localForce.z },
      ExtForceMode.Force
    );
  });
}

function fallbackPoint(world, handle) {
  if (handle == null) return { x: 0, y: 0, z: 0 };
  const c = world.getCollider(handle);
  const p = c ? c.parent() : undefined;
  const b = p != null ? world.getRigidBody(p) : undefined;
  const t = b?.translation?.();
  if (t) return { x: t.x ?? 0, y: t.y ?? 0, z: t.z ?? 0 };
  return { x: 0, y: 0, z: 0 };
}

// ---------- Split queueing & migrations ----------
function queueSplitResults(splitEvents) {
  console.log('Queueing split results:', splitEvents);

  // Translate solver split events into body creations + collider migrations
  for (const evt of splitEvents) {
    const parentActor = evt?.parentActorIndex;
    const children = Array.isArray(evt?.children) ? evt.children : [];
    const parentEntry = state.actorMap.get(parentActor);
    const parentBodyHandle = parentEntry?.bodyHandle ?? state.rootBodyHandle;

    // Keep parent if it appears among children; otherwise we don’t remove it here (not needed)
    for (const child of children) {
      if (!child || !Array.isArray(child.nodes) || child.nodes.length === 0) continue;

      // Phase-1: queue body creation for child; initially map child to parent’s body for safety
      state.pendingBodiesToCreate.push({
        actorIndex: child.actorIndex,
        inheritFromBodyHandle: parentBodyHandle,
        nodes: child.nodes.slice()
      });
      state.actorMap.set(child.actorIndex, { bodyHandle: parentBodyHandle });
    }
  }
}

function applyPendingMigrations() {
  const { world, pendingBodiesToCreate, pendingColliderMigrations } = state;
  // const R = world?.RAPIER;
  // if (!world || !R) return;
  const R = RAPIER;
  if (!world) {
    console.warn('World is not initialized');
    return;
  }

  // Create child bodies; queue node→collider migrations
  if (pendingBodiesToCreate.length > 0) {
    const list = pendingBodiesToCreate.splice(0, pendingBodiesToCreate.length);
    for (const pb of list) {
      const inherit = world.getRigidBody(pb.inheritFromBodyHandle);
      const desc = R.RigidBodyDesc.dynamic();
      if (inherit) {
        const pt = inherit.translation(); const pq = inherit.rotation();
        const lv = inherit.linvel?.(); const av = inherit.angvel?.();
        desc.setTranslation(pt.x, pt.y, pt.z).setRotation(pq)
            .setLinvel(lv?.x ?? 0, lv?.y ?? 0, lv?.z ?? 0)
            .setAngvel(av?.x ?? 0, av?.y ?? 0, av?.z ?? 0);
      }
      const body = world.createRigidBody(desc);
      state.actorMap.set(pb.actorIndex, { bodyHandle: body.handle });

      for (const nodeIndex of pb.nodes) {
        state.pendingColliderMigrations.push({ nodeIndex, targetBodyHandle: body.handle });
      }
    }
  }

  // Apply collider migrations for queued nodes
  if (pendingColliderMigrations.length > 0) {
    const jobs = pendingColliderMigrations.splice(0, pendingColliderMigrations.length);
    for (const mig of jobs) {
      const seg = state.segments[mig.nodeIndex];
      if (!seg) continue;

      // Disable old collider on its previous body and queue for removal
      if (seg.colliderHandle != null) {
        const oldC = world.getCollider(seg.colliderHandle);
        if (oldC) oldC.setEnabled(false);
        state.activeContactColliders.delete(seg.colliderHandle);
        state.colliderToSegment.delete(seg.colliderHandle);
        state.disabledCollidersToRemove.add(seg.colliderHandle);
        seg.colliderHandle = null;
      }

      // Create new collider on target body at the segment’s local offset
      const halfX = seg.size.x * 0.5;
      const halfY = seg.size.y * 0.5;
      const halfZ = seg.size.z * 0.5;
      const body = world.getRigidBody(mig.targetBodyHandle);
      if (!body) continue;

      const col = world.createCollider(
        R.ColliderDesc.cuboid(halfX, halfY, halfZ)
          .setTranslation(seg.localOffset.x, seg.localOffset.y, seg.localOffset.z)
          .setActiveEvents(R.ActiveEvents.CONTACT_FORCE_EVENTS)
          .setContactForceEventThreshold(0.0)
          .setFriction(0.9)
          .setRestitution(0.0),
        body
      );

      seg.bodyHandle = body.handle;
      seg.colliderHandle = col.handle;
      seg.detached = true;

      state.colliderToSegment.set(col.handle, seg.index);
      state.activeContactColliders.add(col.handle);
    }
  }
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

// ---------- Safety sweeps ----------
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

// ---------- Rendering sync ----------
function updateMeshes() {
  syncSegmentsMeshes();
  syncBallMeshes();
}

function syncSegmentsMeshes() {
  const { world, segments, rootBodyHandle } = state;

  const root = world.getRigidBody(rootBodyHandle);
  if (!root) return;

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
