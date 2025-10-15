import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import RAPIER from '@dimforge/rapier3d-compat';
import {
  loadStressSolver,
  ExtForceMode,
  ExtDebugMode // (unused here, handy if you want to add a stress overlay)
} from './stress.js';

/**
 * Minimal stress-driven bridge fracture demo.
 * Key invariants:
 *  - NO Rapier world mutation outside the frame loop.
 *  - Mutations happen ONLY on SAFE frames (no event queue bound).
 *  - If a solver split child uses the parent actorIndex, REUSE the parent body (no new body).
 */

// ---------- Tunables ----------
const BRIDGE_SEGMENTS = 32;
const SEGMENT_SIZE = { x: 0.6, y: 0.2, z: 2.0 };
const SEGMENT_SPACING = 0.62;
const BRIDGE_Y = 0.5;

const BALL_RADIUS = 0.35;
const BALL_DROP_Y = 8;

const GRAVITY = -9.81;
const SAFE_FRAMES_AFTER_SPLIT = 2;  // give BVH time to settle after migrations
const SAFE_FRAMES_AFTER_SPAWN = 1;  // spawn-only cycles

// Stress parameters
const NODE_MASS_KG = 10;
const NODE_VOLUME_M3 = SEGMENT_SIZE.x * SEGMENT_SIZE.y * SEGMENT_SIZE.z;
const BOND_AREA_M2 = SEGMENT_SIZE.y * SEGMENT_SIZE.z;

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

  // Bridge
  rootBodyHandle: null,
  segments: /** @type {Array<{
    index:number,
    localOffset: THREE.Vector3,
    size: {x:number,y:number,z:number},
    mesh: THREE.Mesh,
    colliderHandle: number,
    bodyHandle: number,
    detached: boolean
  }>} */ ([]),

  // Balls
  balls: /** @type {Array<{ bodyHandle:number, mesh:THREE.Mesh }>} */ ([]),

  // Handle maps
  colliderToSegment: /** @type {Map<number, number>} */ (new Map()),
  activeContactColliders: /** @type {Set<number>} */ (new Set()),
  disabledCollidersToRemove: /** @type {Set<number>} */ (new Set()),
  bodiesToRemove: /** @type {Set<number>} */ (new Set()),

  // Queues (mutations applied in SAFE frames only)
  pendingBodiesToCreate: /** @type {Array<{ actorIndex:number, inheritFromBodyHandle:number, nodes:number[] }>} */ ([]),
  pendingColliderMigrations: /** @type {Array<{ nodeIndex:number, targetBodyHandle:number }>} */ ([]),
  pendingBallSpawns: /** @type {Array<{ x:number, z:number }>} */ ([]),

  // Safe-step countdown after any split/spawn mutation
  safeFrames: 0,

  // Picking helpers
  raycaster: new THREE.Raycaster(),
  ndc: new THREE.Vector2(),
  groundPlane: new THREE.Plane(new THREE.Vector3(0, 1, 0), 0), // y=0 plane

  // ------------ Stress Solver ------------
  runtime: null,
  solver: null,
  solverSettings: null,
  // actorIndex -> { bodyHandle }
  actorMap: /** @type {Map<number, { bodyHandle:number }>} */ (new Map()),
};

init().catch((e) => console.error('Demo init failed:', e));

async function init() {
  await RAPIER.init();

  // Three
  const canvas = ensureCanvas();
  const { scene, camera, renderer, controls } = initThree(canvas);
  Object.assign(state, { scene, camera, renderer, controls, clock: new THREE.Clock() });

  // Rapier
  const world = new RAPIER.World({ x: 0, y: GRAVITY, z: 0 });
  state.world = world;
  state.eventQueue = new RAPIER.EventQueue(true);

  buildLights(scene);
  buildGround(scene, world);
  buildBridge(scene, world);

  await buildStressModel();

  // IMPORTANT: no direct world mutation in handlers; just queue the spawn
  canvas.addEventListener('pointerdown', (ev) => {
    const { camera, renderer, raycaster, ndc, groundPlane } = state;
    const rect = renderer.domElement.getBoundingClientRect();
    ndc.x = ((ev.clientX - rect.left) / rect.width) * 2 - 1;
    ndc.y = -((ev.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(ndc, camera);
    const point = new THREE.Vector3();
    if (raycaster.ray.intersectPlane(groundPlane, point)) {
      state.pendingBallSpawns.push({ x: point.x, z: point.z });
      state.safeFrames = Math.max(state.safeFrames, SAFE_FRAMES_AFTER_SPAWN);
    }
  });

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

  const camera = new THREE.PerspectiveCamera(
    60,
    (canvas.clientWidth || window.innerWidth) / (canvas.clientHeight || window.innerHeight),
    0.1,
    500
  );
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
  scene.add(new THREE.AmbientLight(0xffffff, 0.35));
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
  world.createCollider(RAPIER.ColliderDesc.cuboid(30, 0.5, 30).setTranslation(0, -2.5, 0), body);
}

function buildBridge(scene, world) {
  const root = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, BRIDGE_Y, 0));
  state.rootBodyHandle = root.handle;

  const mat = new THREE.MeshStandardMaterial({ color: 0x4b6fe8, roughness: 0.4, metalness: 0.45 });

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

    console.info('Setting collider to segment:', collider.handle, i);
    state.colliderToSegment.set(collider.handle, i);
    state.activeContactColliders.add(collider.handle);
  }

  syncSegmentsMeshes();
}

// ---------- Stress model ----------
async function buildStressModel() {
  state.runtime = await loadStressSolver();

  const nodes = state.segments.map((seg) => ({
    centroid: { x: seg.localOffset.x, y: seg.localOffset.y, z: seg.localOffset.z },
    mass: NODE_MASS_KG,
    volume: NODE_VOLUME_M3
  }));

  const bonds = [];
  for (let i = 0; i < BRIDGE_SEGMENTS - 1; i++) {
    const a = state.segments[i].localOffset;
    const b = state.segments[i + 1].localOffset;
    bonds.push({
      centroid: { x: (a.x + b.x) * 0.5, y: (a.y + b.y) * 0.5, z: (a.z + b.z) * 0.5 },
      normal:   { x: 1, y: 0, z: 0 },
      area: BOND_AREA_M2,
      node0: i,
      node1: i + 1
    });
  }

  const settings = state.runtime.defaultExtSettings();
  settings.maxSolverIterationsPerFrame = 32;
  settings.compressionElasticLimit = 1.0e4;
  settings.compressionFatalLimit   = 2.0e4;
  settings.tensionElasticLimit     = 1.0e4;
  settings.tensionFatalLimit       = 2.0e4;
  settings.shearElasticLimit       = 8.0e3;
  settings.shearFatalLimit         = 1.6e4;

  state.solverSettings = settings;
  state.solver = state.runtime.createExtSolver({ nodes, bonds, settings });

  state.actorMap.clear();
  state.actorMap.set(0, { bodyHandle: state.rootBodyHandle });
}

// ---------- Loop ----------
function loop() {
  const { world, eventQueue, renderer, scene, camera, controls, clock } = state;
  const delta = clock.getDelta();

  // SAFE FRAMES: step without events then do all queued world mutations
  if (state.safeFrames > 0) {
    try { world.step(); } catch (err) { console.warn('Safe step failed', err); }
    state.safeFrames -= 1;

    applyPendingSpawns();       // create balls
    applyPendingMigrations();   // create child bodies, migrate colliders
    removeDisabledHandles();    // prune disabled colliders/bodies

    updateMeshes();
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(loop);
    return;
  }

  // EVENTFUL STEP
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

  // Drain contacts → solver forces (root-local). Avoid world.* during drain callback.
  drainContactsAndApplyForces();

  // Gravity
  state.solver.addGravity({ x: 0, y: GRAVITY, z: 0 });

  // Solve
  state.solver.update();

  // Stress-based fracture & split
  if (state.solver.overstressedBondCount() > 0) {
    console.log('Overstressed bond count:', state.solver.overstressedBondCount());

    const perActor = state.solver.generateFractureCommandsPerActor();
    if (Array.isArray(perActor) && perActor.length > 0) {
      const splitEvents = state.solver.applyFractureCommands(perActor);
      if (Array.isArray(splitEvents) && splitEvents.length > 0) {
        queueSplitResults(splitEvents);
        state.safeFrames = Math.max(state.safeFrames, SAFE_FRAMES_AFTER_SPLIT);
      }
    }
  }

  updateMeshes();
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(loop);
}

// ---------- Contacts → Solver ----------
function drainContactsAndApplyForces() {
  const { world, eventQueue, rootBodyHandle } = state;

  const rootBody = world.getRigidBody(rootBodyHandle);
  if (!rootBody) return;

  const rq = new THREE.Quaternion(
    rootBody.rotation().x,
    rootBody.rotation().y,
    rootBody.rotation().z,
    rootBody.rotation().w
  );
  const qInv = rq.clone().invert();

  // Accumulate forces per collider handle without calling world.* inside the drain
  const acc = new Map(); // handle -> { force:THREE.Vector3, point:THREE.Vector3 }
  const add = (handle, direction, totalForce, worldPoint) => {
    console.log('Adding force to collider:', handle, direction, totalForce, worldPoint);
    if (handle == null) return;
    if (!state.colliderToSegment.has(handle)) {
      console.error('No collider to segment found:', handle, state.colliderToSegment);
      return;
    }

    console.log('Adding force to collider:', handle, direction, totalForce, worldPoint);

    const f = new THREE.Vector3(
      (totalForce.x ?? 0) * direction,
      (totalForce.y ?? 0) * direction,
      (totalForce.z ?? 0) * direction
    );
    const p = new THREE.Vector3(worldPoint.x ?? 0, worldPoint.y ?? 0, worldPoint.z ?? 0);

    const prev = acc.get(handle);
    if (prev) {
      prev.force.add(f);
      prev.point.copy(p);
    } else {
      acc.set(handle, { force: f, point: p });
    }
  };

  eventQueue.drainContactForceEvents((ev) => {
    try {
      // console.log('Draining contact force events:', ev);
      const tf = ev.totalForce?.();
      const mag = ev.totalForceMagnitude?.();
      if (!tf || !(mag > 0)) {
        console.log('No total force found:', tf, mag);
        return;
      }
      // console.log('Draining contact force events:', tf, mag);

      // Use the explicit contact points; if unavailable, ignore this event (avoid world.* calls here)
      // const p1 = ev.worldContactPoint?.();
      // const p2 = ev.worldContactPoint2?.();
      // if (!p1 && !p2) return;

      // Prefer explicit contact points; fall back to collider’s parent body position if absent
      const wp = ev.worldContactPoint ? ev.worldContactPoint() : undefined;
      const wp2 = ev.worldContactPoint2 ? ev.worldContactPoint2() : undefined;
      
      const h1 = ev.collider1?.();
      const h2 = ev.collider2?.();

      const p1 = wp ?? wp2 ?? fallbackPoint(world, h1);
      const p2 = wp2 ?? wp ?? fallbackPoint(world, h2);

      if (h1 == null || h2 == null) {
        console.error('No collider found:', h1, h2);
        return;
      }
      console.log('Adding force to collider:', tf, mag, p1, p2);

      // if (h1 != null)
      add(h1, +1, tf, p1 ?? p2);
      // if (h2 != null)
      add(h2, -1, tf, p2 ?? p1);
    } catch (error) {
      console.error(error);
    }
  });

  // Apply to solver (root-local)
  if (acc.size > 0) {
    console.log('Applying forces to solver:', acc);
  }
  acc.forEach((val, handle) => {
    const nodeIndex = state.colliderToSegment.get(handle);
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

  for (const evt of splitEvents) {
    const parentActorIndex = evt?.parentActorIndex;
    const children = Array.isArray(evt?.children) ? evt.children : [];
    const parentEntry = state.actorMap.get(parentActorIndex);
    const parentBodyHandle = parentEntry?.bodyHandle ?? state.rootBodyHandle;

    for (const child of children) {
      if (!child || !Array.isArray(child.nodes) || child.nodes.length === 0) {
        console.warn('No child nodes found:', child);
        continue;
      }

      // **Critical fix**: if the parent actor appears among children, REUSE the parent body.
      if (child.actorIndex === parentActorIndex) {
        // No body creation or migrations needed for these nodes; they remain on the parent body.
        console.warn('Reusing parent body for child:', child.actorIndex, parentBodyHandle);
        state.actorMap.set(child.actorIndex, { bodyHandle: parentBodyHandle });
        continue;
      }

      // Schedule child body creation (inherits parent pose/vel), and migrate its nodes later.
      state.pendingBodiesToCreate.push({
        actorIndex: child.actorIndex,
        inheritFromBodyHandle: parentBodyHandle,
        nodes: child.nodes.slice()
      });

      // Tentative mapping; will be overwritten when body is created on a SAFE frame.
      state.actorMap.set(child.actorIndex, { bodyHandle: parentBodyHandle });
    }
  }
}

function applyPendingSpawns() {
  const { pendingBallSpawns } = state;
  if (pendingBallSpawns.length === 0) return;

  const batch = pendingBallSpawns.splice(0, pendingBallSpawns.length);
  for (const s of batch) spawnBallNow(s.x, s.z);
}

function spawnBallNow(x, z) {
  const { world, scene } = state;

  const body = world.createRigidBody(
    RAPIER.RigidBodyDesc.dynamic()
      .setTranslation(x, BALL_DROP_Y, z)
      .setCanSleep(false)
      .setLinearDamping(0.01)
      .setAngularDamping(0.01)
  );
  world.createCollider(
    RAPIER.ColliderDesc.ball(BALL_RADIUS)
      .setMass(100_000_000.0)
      .setFriction(0.6)
      .setRestitution(0.2)
      .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
      .setContactForceEventThreshold(0.0),
    body
  );

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

function applyPendingMigrations() {
  const { world, pendingBodiesToCreate, pendingColliderMigrations } = state;
  // const R = world?.RAPIER;
  const R = RAPIER;
  if (!world) {
    console.warn('World is not initialized');
    return;
  }
  // if (!world || !R) return;

  // Create child bodies & schedule collider migrations
  if (pendingBodiesToCreate.length > 0) {
    const list = pendingBodiesToCreate.splice(0, pendingBodiesToCreate.length);
    for (const pb of list) {
      const inherit = world.getRigidBody(pb.inheritFromBodyHandle);
      const desc = R.RigidBodyDesc.dynamic();
      if (inherit) {
        const pt = inherit.translation(); const pq = inherit.rotation();
        const lv = inherit.linvel?.();   const av = inherit.angvel?.();
        desc.setTranslation(pt.x, pt.y, pt.z).setRotation(pq)
            .setLinvel(lv?.x ?? 0, lv?.y ?? 0, lv?.z ?? 0)
            .setAngvel(av?.x ?? 0, av?.y ?? 0, av?.z ?? 0);
      }
      const body = world.createRigidBody(desc);
      state.actorMap.set(pb.actorIndex, { bodyHandle: body.handle });

      for (const nodeIndex of pb.nodes) {
        pendingColliderMigrations.push({ nodeIndex, targetBodyHandle: body.handle });
      }
    }
  }

  // Migrate colliders to their new bodies
  if (pendingColliderMigrations.length > 0) {
    const jobs = pendingColliderMigrations.splice(0, pendingColliderMigrations.length);
    for (const mig of jobs) {
      const seg = state.segments[mig.nodeIndex];
      if (!seg) continue;

      // Disable old collider and queue removal
      if (seg.colliderHandle != null) {
        console.warn('Disabling old collider:', seg.colliderHandle);
        const oldC = world.getCollider(seg.colliderHandle);
        if (oldC) oldC.setEnabled(false);
        state.activeContactColliders.delete(seg.colliderHandle);
        state.colliderToSegment.delete(seg.colliderHandle);
        state.disabledCollidersToRemove.add(seg.colliderHandle);
        seg.colliderHandle = null;
      }

      // Create replacement collider on the target body
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

      console.info('Setting collider to segment:', col.handle, seg.index);
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
      console.warn('Deleting active contact collider:', h);
      activeContactColliders.delete(h);
      colliderToSegment.delete(h);
    }
  }

  // Mirror sweep on the map (paranoid redundancy)
  for (const [h] of Array.from(colliderToSegment.entries())) {
    const c = world.getCollider(h);
    const p = c ? c.parent() : undefined;
    const b = p != null ? world.getRigidBody(p) : undefined;
    const enabled = c?.isEnabled ? c.isEnabled() : true;
    if (!c || !enabled || !b) {
      console.warn('Deleting collider to segment:', h);
      colliderToSegment.delete(h);
      activeContactColliders.delete(h);
    }
  }

  // Remove any disabled colliders queued by prior migrations
  for (const h of Array.from(disabledCollidersToRemove)) {
    const c = world.getCollider(h);
    if (c) world.removeCollider(c, false);
    disabledCollidersToRemove.delete(h);
  }
}

// ---------- Rendering ----------
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

    if (t.y < -50) {
      scene.remove(b.mesh);
      state.bodiesToRemove.add(b.bodyHandle);
    } else {
      keep.push(b);
    }
  }
  state.balls = keep;
}
