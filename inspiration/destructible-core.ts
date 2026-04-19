import RAPIER, { type RigidBody as RapierRigidBody } from '@dimforge/rapier3d-compat';
import { loadStressSolver } from 'blast-stress-solver';
import type {
  DestructibleCore,
  ScenarioDesc,
  ChunkData,
  Vec3,
  ProjectileSpawn,
  ProjectileState,
  BondRef,
  CoreProfilerConfig,
  CoreProfilerSample,
  CoreProfilerPass,
  SingleCollisionMode,
  SmallBodyDampingOptions,
  OptimizationMode,
} from './types';
import { DestructibleDamageSystem, type DamageOptions, type DamageStateSnapshot } from './damage';
import { planSplitMigration, type PlannerChild } from './splitMigrator';

type BuildCoreOptions = {
  scenario: ScenarioDesc;
  // Collider sizing strategy for nodes → Box colliders
  nodeSize: (nodeIndex: number, scenario: ScenarioDesc) => Vec3; // full extents
  gravity?: number;
  friction?: number;
  restitution?: number;
  materialScale?: number;
  singleCollisionMode?: SingleCollisionMode;
  damage?: DamageOptions & { autoDetachOnDestroy?: boolean; autoCleanupPhysics?: boolean };
  onNodeDestroyed?: (e: { nodeIndex: number; actorIndex: number; reason: 'impact'|'manual' }) => void;
  // Fracture rollback/resimulation controls
  resimulateOnFracture?: boolean; // default true
  maxResimulationPasses?: number; // default 1
  snapshotMode?: 'perBody' | 'world'; // default 'perBody'
  onWorldReplaced?: (newWorld: RAPIER.World) => void; // only used when snapshotMode==='world'
  resimulateOnDamageDestroy?: boolean;
  skipSingleBodies?: boolean;
  sleepLinearThreshold?: number;
  sleepAngularThreshold?: number;
  sleepMode?: OptimizationMode;
  // Small body damping - apply higher damping to bodies with few colliders
  smallBodyDamping?: SmallBodyDampingOptions;
};

const isDev = true; //process.env.NODE_ENV !== 'production';

const perfNow =
  typeof performance !== 'undefined' && typeof performance.now === 'function'
    ? () => performance.now()
    : () => Date.now();

type MutableCoreProfilerSample = CoreProfilerSample & { finalized?: boolean };
type NumericProfilerField = Exclude<
  {
    [K in keyof MutableCoreProfilerSample]: MutableCoreProfilerSample[K] extends number
      ? K
      : never;
  }[keyof MutableCoreProfilerSample],
  undefined
>;

type WorldWithOptionalTimestep = RAPIER.World & { timestep?: number };
type WorldWithBodyCount = RAPIER.World & { numRigidBodies?: () => number };
type RigidBodyWithColliderCount = RAPIER.RigidBody & { numColliders?: () => number };
type BodyWithUserData = RAPIER.RigidBody & { userData?: { projectile?: boolean } };
type MassReadableBody = RAPIER.RigidBody & { mass?: () => number };
type MaybeCcdBodyDesc = RAPIER.RigidBodyDesc & { setCcdEnabled?: (v: boolean) => unknown };
type InteractionGroupsValue = Parameters<RAPIER.Collider['setCollisionGroups']>[0];
type SolverActorsApi = {
  actors?: () => Array<{ actorIndex: number; nodes: number[] }>;
};
type DebugWindow = Window & {
  debugStressSolver?: { printSolver?: () => unknown };
};


const clonePasses = (passes: CoreProfilerPass[]) =>
  passes.map((pass) => ({
    ...pass,
    reasons: [...pass.reasons],
  }));

export async function buildDestructibleCore({
  scenario,
  nodeSize,
  gravity = -9.81,
  friction = 0.25,
  restitution = 0.0,
  materialScale = 1.0,
  singleCollisionMode = 'all',
  damage,
  onNodeDestroyed,
  resimulateOnFracture = true,
  maxResimulationPasses = 1,
  snapshotMode = 'perBody',
  onWorldReplaced,
  resimulateOnDamageDestroy = !!damage?.enabled,
  skipSingleBodies = false,
  sleepLinearThreshold = 0.1,
  sleepAngularThreshold = 0.1,
  sleepMode = 'off',
  smallBodyDamping,
}: BuildCoreOptions): Promise<DestructibleCore> {
  await RAPIER.init();
  const runtime = await loadStressSolver();
  const profiler = {
    enabled: false,
    onSample: null as CoreProfilerConfig['onSample'] | null,
    frameIndex: 0,
  };

  const createProfilerSample = (dt: number): MutableCoreProfilerSample => ({
    frameIndex: profiler.frameIndex++,
    timestamp: Date.now(),
    dt,
    rapierStepMs: 0,
    contactDrainMs: 0,
    solverUpdateMs: 0,
    damageReplayMs: 0,
    damagePreviewMs: 0,
    damageTickMs: 0,
    fractureMs: 0,
    fractureGenerateMs: 0,
    fractureApplyMs: 0,
    splitQueueMs: 0,
    bodyCreateMs: 0,
    colliderRebuildMs: 0,
    cleanupDisabledMs: 0,
    spawnMs: 0,
    externalForceMs: 0,
    damageSnapshotMs: 0,
    damageRestoreMs: 0,
    damagePreDestroyMs: 0,
    damageFlushMs: 0,
    preStepSweepMs: 0,
    rebuildColliderMapMs: 0,
    projectileCleanupMs: 0,
    initialPassMs: 0,
    resimMs: 0,
    totalMs: 0,
    resimPasses: 0,
    resimReasons: [],
    snapshotBytes: 0,
    snapshotCaptureMs: 0,
    snapshotRestoreMs: 0,
    bufferedExternalContacts: 0,
    bufferedInternalContacts: 0,
    pendingExternalForces: 0,
    projectiles: 0,
    rigidBodies: 0,
    passes: [],
    finalized: false,
  });

  const setProfiler = (config: CoreProfilerConfig | null) => {
    profiler.enabled = !!(config?.enabled && typeof config.onSample === 'function');
    profiler.onSample = profiler.enabled ? config?.onSample ?? null : null;
  };

  let activeProfilerSample: MutableCoreProfilerSample | null = null;
  const startTiming = () => (activeProfilerSample ? perfNow() : null);
  const stopTiming = (start: number | null, field: NumericProfilerField) => {
    if (!activeProfilerSample || start == null) return;
    const sample = activeProfilerSample as MutableCoreProfilerSample &
      Record<NumericProfilerField, number>;
    sample[field] += Math.max(0, perfNow() - start);
  };
  const addDuration = (field: NumericProfilerField, durationMs: number) => {
    if (!activeProfilerSample || !(durationMs > 0)) return;
    const sample = activeProfilerSample as MutableCoreProfilerSample &
      Record<NumericProfilerField, number>;
    sample[field] += durationMs;
  };
  const profiledGenerateFractureCommands = (): ReturnType<typeof solver.generateFractureCommandsPerActor> => {
    const timerStart = startTiming();
    const perActor = solver.generateFractureCommandsPerActor();
    stopTiming(timerStart, 'fractureGenerateMs');
    return perActor;
  };
  type FractureCommands = Parameters<typeof solver.applyFractureCommands>[0];
  const profiledApplyFractureCommands = (
    commands: FractureCommands,
  ): ReturnType<typeof solver.applyFractureCommands> => {
    const timerStart = startTiming();
    const result = solver.applyFractureCommands(commands);
    stopTiming(timerStart, 'fractureApplyMs');
    return result;
  };
  const recordProjectileCleanupDurationInternal = (durationMs: number) => {
    addDuration('projectileCleanupMs', durationMs);
  };

  const settings = runtime.defaultExtSettings();
  const scaledSettings = { ...settings };
  const skipSingleBodiesEnabled = !!skipSingleBodies;
  let singleCollisionModeSetting: SingleCollisionMode = singleCollisionMode;
  // Track bodies that have collided with ground (for conditional optimization modes)
  const bodiesCollidedWithGround = new Set<number>();
  // Track small bodies that may need damping applied when they collide with ground
  const smallBodiesPendingDamping = new Set<number>();

  // Apply small body damping to a specific body (called when ground collision detected if mode is 'afterGroundCollision')
  function applySmallBodyDampingToBody(bodyHandle: number) {
    if (smallBodyDampingSettings.mode === 'off') return;
    if (!smallBodiesPendingDamping.has(bodyHandle)) return;

    const body = world.getRigidBody(bodyHandle);
    if (!body) return;

    try {
      const currentLinDamp = typeof body.linearDamping === 'function' ? body.linearDamping() : 0;
      const currentAngDamp = typeof body.angularDamping === 'function' ? body.angularDamping() : 0;
      const newLinDamp = Math.max(currentLinDamp, smallBodyDampingSettings.minLinearDamping);
      const newAngDamp = Math.max(currentAngDamp, smallBodyDampingSettings.minAngularDamping);
      body.setLinearDamping(newLinDamp);
      body.setAngularDamping(newAngDamp);
    } catch {}

    // Remove from pending set once damping is applied
    smallBodiesPendingDamping.delete(bodyHandle);
  }

  const sleepSettings = {
    mode: (sleepMode as OptimizationMode) ?? 'off',
    linear: Math.max(0, sleepLinearThreshold),
    angular: Math.max(0, sleepAngularThreshold),
  };
  function updateSleepThresholds(linear?: number, angular?: number) {
    if (typeof linear === 'number' && Number.isFinite(linear)) {
      sleepSettings.linear = Math.max(0, linear);
    }
    if (typeof angular === 'number' && Number.isFinite(angular)) {
      sleepSettings.angular = Math.max(0, angular);
    }
  }
  function updateSleepMode(mode: OptimizationMode) {
    sleepSettings.mode = mode;
  }

  // Small body damping - apply higher damping to bodies with few colliders to reduce jitter
  const smallBodyDampingSettings = {
    mode: (smallBodyDamping?.mode as OptimizationMode) ?? 'off',
    colliderCountThreshold: smallBodyDamping?.colliderCountThreshold ?? 3,
    minLinearDamping: smallBodyDamping?.minLinearDamping ?? 2,
    minAngularDamping: smallBodyDamping?.minAngularDamping ?? 2,
  };
  function updateSmallBodyDamping(opts: SmallBodyDampingOptions) {
    if (opts.mode != null) {
      smallBodyDampingSettings.mode = opts.mode;
    }
    if (typeof opts.colliderCountThreshold === 'number' && Number.isFinite(opts.colliderCountThreshold)) {
      smallBodyDampingSettings.colliderCountThreshold = Math.max(0, Math.floor(opts.colliderCountThreshold));
    }
    if (typeof opts.minLinearDamping === 'number' && Number.isFinite(opts.minLinearDamping)) {
      smallBodyDampingSettings.minLinearDamping = Math.max(0, opts.minLinearDamping);
    }
    if (typeof opts.minAngularDamping === 'number' && Number.isFinite(opts.minAngularDamping)) {
      smallBodyDampingSettings.minAngularDamping = Math.max(0, opts.minAngularDamping);
    }
  }

  // Helper to check if optimization should apply to a body
  function shouldApplyOptimization(mode: OptimizationMode, bodyHandle: number): boolean {
    if (mode === 'off') return false;
    if (mode === 'always') return true;
    if (mode === 'afterGroundCollision') return bodiesCollidedWithGround.has(bodyHandle);
    return false;
  }

  // Reasonable defaults; caller can adjust later if needed
  // settings.maxSolverIterationsPerFrame = 64;
  settings.maxSolverIterationsPerFrame = 24;
  settings.graphReductionLevel = 0;

  const baseCompressionElastic = 0.0009;
  const baseCompressionFatal = 0.0027;
  const baseShearElastic = 0.0012;
  const baseShearFatal = 0.0036;
  const baseTensionElastic = 0.0009;
  const baseTensionFatal = 0.0027;

  scaledSettings.compressionElasticLimit = baseCompressionElastic * materialScale;
  scaledSettings.compressionFatalLimit = baseCompressionFatal * materialScale;
  scaledSettings.tensionElasticLimit = baseTensionElastic * materialScale;
  scaledSettings.tensionFatalLimit = baseTensionFatal * materialScale;
  scaledSettings.shearElasticLimit = baseShearElastic * materialScale;
  scaledSettings.shearFatalLimit = baseShearFatal * materialScale;

  // Mark supports via mass=0
  const nodes = scenario.nodes.map((n) => ({ centroid: n.centroid, mass: n.mass, volume: n.volume }));
  const bonds = scenario.bonds.map((b) => ({ node0: b.node0, node1: b.node1, centroid: b.centroid, normal: b.normal, area: b.area }));

  const hasSupports = nodes.some((n) => n.mass === 0);
  if (!hasSupports) {
    console.warn('[Core] no supports (nodes withmass=0)found in scenario', scenario);
  }

  const solver = runtime.createExtSolver({ nodes, bonds, settings: scaledSettings });

  // Persist bond list with indices for cutting and adjacency
  const bondTable: Array<{ index:number; node0:number; node1:number; centroid:Vec3; normal:Vec3; area:number }> = scenario.bonds.map((b, i) => ({ index: i, node0: b.node0, node1: b.node1, centroid: b.centroid, normal: b.normal, area: b.area }));
  const bondsByNode = new Map<number, number[]>();
  for (const b of bondTable) {
    if (!bondsByNode.has(b.node0)) bondsByNode.set(b.node0, []);
    if (!bondsByNode.has(b.node1)) bondsByNode.set(b.node1, []);
    const arr0 = bondsByNode.get(b.node0);
    const arr1 = bondsByNode.get(b.node1);
    if (arr0) arr0.push(b.index);
    if (arr1) arr1.push(b.index);
  }

  let world = new RAPIER.World({ x: 0, y: gravity, z: 0 });
  const eventQueue = new RAPIER.EventQueue(true);

  // Root fixed body
  const rootBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0)
    .setUserData({ root: true })
  );

  // Ground collider (fixed body), wide plane with top aligned to y=0
  const groundBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.fixed()
      .setTranslation(0, 0, 0)
      .setUserData({ ground: true })
  );
  world.createCollider(
    RAPIER.ColliderDesc.cuboid(100, 0.025, 100)
      .setTranslation(0, -0.025, 0)
      .setFriction(0.9)
      .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
      .setContactForceEventThreshold(0.0)
      ,
    groundBody
  );

  // Create chunk colliders attached to root (supports may become separate fixed bodies later on split)
  const chunks: ChunkData[] = [];
  const colliderToNode = new Map<number, number>();
  const activeContactColliders = new Set<number>();
  const actorMap = new Map<number, { bodyHandle: number }>();
  const nodesByBodyHandle = new Map<number, Set<number>>();
  // Track which actor currently owns each node (updated on splits)
  const nodeToActor = new Map<number, number>();
  for (let i = 0; i < scenario.nodes.length; i += 1) nodeToActor.set(i, 0);

  function actorBodyForNode(nodeIndex: number) {
    const actorIndex = nodeToActor.get(nodeIndex) ?? 0;
    const entry = actorMap.get(actorIndex);
    const bodyHandle = entry?.bodyHandle ?? rootBody.handle;
    const body = world.getRigidBody(bodyHandle);
    return { actorIndex, bodyHandle, body };
  }

  function registerNodeBodyLink(
    nodeIndex: number,
    bodyHandle: number | null | undefined,
  ) {
    if (bodyHandle == null) return;
    let set = nodesByBodyHandle.get(bodyHandle);
    if (!set) {
      set = new Set<number>();
      nodesByBodyHandle.set(bodyHandle, set);
    }
    set.add(nodeIndex);
  }

  function unregisterNodeBodyLink(
    nodeIndex: number,
    bodyHandle: number | null | undefined,
  ) {
    if (bodyHandle == null) return;
    const set = nodesByBodyHandle.get(bodyHandle);
    if (!set) return;
    set.delete(nodeIndex);
    if (set.size === 0) nodesByBodyHandle.delete(bodyHandle);
  }

  function percentileFromSorted(sorted: number[], percentile: number): number {
    if (!sorted.length) return 0;
    const clamped = Math.min(1, Math.max(0, percentile));
    const index = Math.min(
      sorted.length - 1,
      Math.round(clamped * (sorted.length - 1)),
    );
    return sorted[index];
  }

  function captureBodyColliderStats():
    | null
    | {
        bodyCount: number;
        min: number;
        max: number;
        avg: number;
        median: number;
        p95: number;
      } {
    const counts: number[] = [];
    for (const set of nodesByBodyHandle.values()) {
      if (!set || set.size === 0) continue;
      counts.push(set.size);
    }
    if (counts.length === 0) return null;
    counts.sort((a, b) => a - b);
    const bodyCount = counts.length;
    const total = counts.reduce((sum, value) => sum + value, 0);
    const avg = total / bodyCount;
    const median = percentileFromSorted(counts, 0.5);
    const p95 = percentileFromSorted(counts, 0.95);
    return {
      bodyCount,
      min: counts[0],
      max: counts[counts.length - 1],
      avg,
      median,
      p95,
    };
  }

  // const spacing = scenario.spacing ?? { x: 0.5, y: 0.5, z: 0.5 };

  function buildColliderDescForNode(args: { nodeIndex: number; halfX: number; halfY: number; halfZ: number; isSupport: boolean }) {
    const { nodeIndex, halfX, halfY, halfZ, isSupport } = args;
    const builder = (scenario.colliderDescForNode && Array.isArray(scenario.colliderDescForNode)) ? (scenario.colliderDescForNode[nodeIndex] ?? null) : null;
    let desc = typeof builder === 'function' ? builder() : null;
    if (!desc) {
      const s = isSupport ? 0.999 : 1.0;
      desc = RAPIER.ColliderDesc.cuboid(halfX * s, halfY * s, halfZ * s);
    }
    return desc;
  }

  scenario.nodes.forEach((node, nodeIndex) => {
    const size = nodeSize(nodeIndex, scenario);
    const halfX = Math.max(0.05, size.x * 0.5);
    const halfY = Math.max(0.05, size.y * 0.5);
    const halfZ = Math.max(0.05, size.z * 0.5);

    const nodeMass = node.mass ?? 1;
    const isSupport = nodeMass === 0;
    const chunk: ChunkData = {
      nodeIndex,
      size: { x: size.x, y: size.y, z: size.z },
      isSupport,
      baseLocalOffset: { x: node.centroid.x, y: node.centroid.y, z: node.centroid.z },
      localOffset: { x: node.centroid.x, y: node.centroid.y, z: node.centroid.z },
      colliderHandle: null,
      bodyHandle: rootBody.handle,
      active: true,
      detached: false,
    };

    const desc = buildColliderDescForNode({ nodeIndex, halfX, halfY, halfZ, isSupport })
      .setMass(nodeMass)
      .setTranslation(chunk.localOffset.x, chunk.localOffset.y, chunk.localOffset.z)
      .setFriction(friction)
      .setRestitution(restitution)
      .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
      .setContactForceEventThreshold(0.0);
    const col = world.createCollider(desc, rootBody);
    chunk.colliderHandle = col.handle;
    colliderToNode.set(col.handle, nodeIndex);
    activeContactColliders.add(col.handle);
    registerNodeBodyLink(nodeIndex, chunk.bodyHandle);
    chunks.push(chunk);
  });

  if (process.env.NODE_ENV !== 'production') {
    try {
      console.debug('[Core] Built chunk colliders', {
        nodeCount: scenario.nodes?.length ?? 0,
        bondCount: scenario.bonds?.length ?? 0,
        mappingSize: colliderToNode.size,
      });
      if (colliderToNode.size === 0) {
        console.warn('[Core] colliderToNode empty after build; contact forces will be dropped unless rebuilt');
      }
    } catch {}
  }

  solver.actors().forEach((actor) => { actorMap.set(actor.actorIndex, { bodyHandle: rootBody.handle }); });

  // Queues and scratch
  const pendingBodiesToCreate: Array<{ actorIndex: number; inheritFromBodyHandle: number; nodes: number[]; isSupport: boolean }> = [];
  const pendingColliderMigrations: Array<{ nodeIndex: number; targetBodyHandle: number }> = [];
  const disabledCollidersToRemove = new Set<number>();
  const bodiesToRemove = new Set<number>();
  const pendingBallSpawns: ProjectileSpawn[] = [];
  const projectiles: ProjectileState[] = [];
  const removedBondIndices = new Set<number>();
  const pendingExternalForces: Array<{ nodeIndex:number; point: Vec3; force: Vec3 }> = [];
  const pendingDamageFractures = new Map<number, Set<number>>();
  const nowSeconds = () =>
    (typeof performance !== 'undefined' ? performance.now() : Date.now()) /
    1000;

  let safeFrames = 0;
  let warnedColliderMapEmptyOnce = false;
  let solverGravityEnabled = true;
  const damageOptions: Required<DamageOptions & { autoDetachOnDestroy?: boolean; autoCleanupPhysics?: boolean }> = {
    enabled: !!damage?.enabled,
    strengthPerVolume: damage?.strengthPerVolume ?? 10000,
    kImpact: damage?.kImpact ?? 0.002,
    enableSupportsDamage: damage?.enableSupportsDamage ?? false,
    autoDetachOnDestroy: damage?.autoDetachOnDestroy ?? true,
    autoCleanupPhysics: damage?.autoCleanupPhysics ?? true,
    contactDamageScale: damage?.contactDamageScale ?? 1.0,
    minImpulseThreshold: damage?.minImpulseThreshold ?? 50,
    contactCooldownMs: damage?.contactCooldownMs ?? 120,
    internalContactScale: damage?.internalContactScale ?? 2.0,
    massExponent: damage?.massExponent ?? 0.5,
    internalMinImpulseThreshold: damage?.internalMinImpulseThreshold ?? 15,
    splashRadius: damage?.splashRadius ?? 1.5,
    splashFalloffExp: damage?.splashFalloffExp ?? 2.0,
    speedMinExternal: damage?.speedMinExternal ?? 0.5,
    speedMinInternal: damage?.speedMinInternal ?? 0.25,
    speedMax: damage?.speedMax ?? 6.0,
    speedExponent: damage?.speedExponent ?? 1.0,
    slowSpeedFactor: damage?.slowSpeedFactor ?? 0.9,
    fastSpeedFactor: damage?.fastSpeedFactor ?? 3.0,
  } as const;

  const damageSystem = new DestructibleDamageSystem({
    chunks,
    scenario,
    materialScale,
    options: damageOptions,
    nodesForBody: (bodyHandle: number) =>
      nodesByBodyHandle.get(bodyHandle)?.values(),
  });
  const MIN_STEP_DT = 1e-4;
  const MAX_STEP_DT = 1 / 30;
  const clampStepDt = (value: number) =>
    Math.min(MAX_STEP_DT, Math.max(MIN_STEP_DT, value));
  const readWorldDt = (): number => {
    try {
      const params = world.integrationParameters;
      const dt = params?.dt;
      if (typeof dt === 'number' && dt > 0) return dt;
    } catch {}
    try {
      const raw = (world as WorldWithOptionalTimestep).timestep;
      if (typeof raw === 'number' && raw > 0) return raw;
    } catch {}
    return 1 / 60;
  };
  const setWorldDtValue = (value: number) => {
    const clamped = clampStepDt(value);
    try {
      (world as WorldWithOptionalTimestep).timestep = clamped;
    } catch {}
    try {
      if (world.integrationParameters) {
        world.integrationParameters.dt = clamped;
      }
    } catch {}
  };
  let lastStepDt = readWorldDt();

  // --- Shared helpers for speed scaling and contact draining ---
  function getDt(): number {
    return lastStepDt;
  }

  function computeSpeedFactor(relSpeed: number, isInternal: boolean): number {
    try {
      const opts = damageSystem.getOptions();
      const vMin = isInternal ? (opts.speedMinInternal ?? 0.25) : (opts.speedMinExternal ?? 0.5);
      const vMax = opts.speedMax ?? 6.0;
      const exp = Math.max(0.01, opts.speedExponent ?? 1.0);
      const slow = Math.max(0, Math.min(1, opts.slowSpeedFactor ?? 0.9));
      const fast = Math.max(1, opts.fastSpeedFactor ?? 3.0);
      const vSpan = Math.max(1e-3, vMax - vMin);
      const t = relSpeed > vMin ? Math.min(1, Math.pow((relSpeed - vMin) / vSpan, exp)) : 0;
      return slow + (fast - slow) * t;
    } catch {
      return 1.0;
    }
  }

  // --- Buffered contact damage (to avoid double-apply across rollback) ---
  type BufferedExternalContact = { node:number; effMag:number; dt:number; local?:{x:number;y:number;z:number} };
  type BufferedInternalContact = { a:number; b:number; effMag:number; dt:number; localA?:{x:number;y:number;z:number}; localB?:{x:number;y:number;z:number} };
  const bufferedExternal: BufferedExternalContact[] = [];
  const bufferedInternal: BufferedInternalContact[] = [];
  function clearBufferedContacts() { bufferedExternal.length = 0; bufferedInternal.length = 0; }
  function replayBufferedContacts() {
    const profilerSample = activeProfilerSample;
    const timerStart = profilerSample ? perfNow() : 0;
    if (!damageSystem.isEnabled()) return;
    for (const e of bufferedExternal) damageSystem.onImpact(e.node, e.effMag, e.dt, e.local ? { localPoint: e.local } : undefined);
    for (const i of bufferedInternal) damageSystem.onInternalImpact(i.a, i.b, i.effMag, i.dt, { localPointA: i.localA, localPointB: i.localB });
    if (profilerSample) {
      profilerSample.damageReplayMs += Math.max(0, perfNow() - timerStart);
    }
  }

  function drainContactForces(params: { injectSolverForces: boolean; applyDamage: boolean; recordForReplay?: boolean }) {
    const applyDamage = !!params.applyDamage;
    const record = !!params.recordForReplay;
    const profilerSample = activeProfilerSample;
    const timerStart = profilerSample ? perfNow() : 0;
    eventQueue.drainContactForceEvents((ev: { totalForce: () => {x:number;y:number;z:number}; totalForceMagnitude: () => number; collider1: () => number; collider2: () => number; worldContactPoint?: () => {x:number; y:number; z:number}; worldContactPoint2?: () => {x:number; y:number; z:number} }) => {
      const tf = ev.totalForce?.();
      const mag = ev.totalForceMagnitude?.();
      if (!tf || !(mag > 0)) {
        return;
      }

      const h1 = ev.collider1?.();
      const h2 = ev.collider2?.();
      const wp = ev.worldContactPoint ? ev.worldContactPoint() : undefined;
      const wp2 = ev.worldContactPoint2 ? ev.worldContactPoint2() : undefined;
      const p1 = wp ?? wp2 ?? fallbackPoint(world, h1);
      const p2 = wp2 ?? wp ?? fallbackPoint(world, h2);

      const node1 = colliderToNode.has(h1) ? colliderToNode.get(h1) : undefined;
      const node2 = colliderToNode.has(h2) ? colliderToNode.get(h2) : undefined;
      const isInternal = (node1 != null && node2 != null);

      const dt = getDt();
      // Use reported contact points if present; otherwise approximate by chunk world center
      const pForNode1 = node1 != null ? (wp ?? wp2 ?? chunkWorldCenter(node1) ?? p1) : undefined;
      const pForNode2 = node2 != null ? (wp2 ?? wp ?? chunkWorldCenter(node2) ?? p2) : undefined;
      const relAnchor = pForNode1 ?? pForNode2 ?? (p1 ?? p2);
      const relSpeed = computeRelativeSpeed(world, h1, h2, relAnchor);
      const speedFactor = computeSpeedFactor(relSpeed, isInternal);
      let effMag = (mag ?? 0) * speedFactor;

      // Projectile momentum boost on initial impact for broader splash
      try {
        const b1 = getBodyForColliderHandle(h1);
        const b2 = getBodyForColliderHandle(h2);

        // Track ground collisions for optimization modes
        if (b1 && b2) {
          const isB1Ground = b1.handle === groundBody.handle;
          const isB2Ground = b2.handle === groundBody.handle;
          if (isB1Ground && !isB2Ground && b2.handle !== rootBody.handle) {
            const wasNew = !bodiesCollidedWithGround.has(b2.handle);
            bodiesCollidedWithGround.add(b2.handle);
            // Apply deferred small body damping if mode is afterGroundCollision
            if (wasNew && smallBodyDampingSettings.mode === 'afterGroundCollision') {
              applySmallBodyDampingToBody(b2.handle);
            }
          } else if (isB2Ground && !isB1Ground && b1.handle !== rootBody.handle) {
            const wasNew = !bodiesCollidedWithGround.has(b1.handle);
            bodiesCollidedWithGround.add(b1.handle);
            // Apply deferred small body damping if mode is afterGroundCollision
            if (wasNew && smallBodyDampingSettings.mode === 'afterGroundCollision') {
              applySmallBodyDampingToBody(b1.handle);
            }
          }
        }

        const ud1 = (b1 as BodyWithUserData | null)?.userData;
        const ud2 = (b2 as BodyWithUserData | null)?.userData;
        const projBody = (ud1?.projectile ? b1 : (ud2?.projectile ? b2 : null));
        if (projBody && (node1 != null || node2 != null)) {
          let m = 1;
          try {
            const reader = projBody as MassReadableBody;
            if (typeof reader.mass === 'function') {
              m = reader.mass();
            }
          } catch {}
          const impulseEstimate = Math.max(0, m) * Math.max(0, relSpeed);
          const forceFromMomentum = impulseEstimate / Math.max(1e-6, dt);
          if (Number.isFinite(forceFromMomentum) && forceFromMomentum > 0) {
            effMag = Math.max(effMag, forceFromMomentum);
          }
        }
      } catch {}

      const local1 = (node1 != null && pForNode1) ? worldPointToActorLocal(node1, pForNode1) : null;
      const local2 = (node2 != null && pForNode2) ? worldPointToActorLocal(node2, pForNode2) : null;

      if (h1 != null) {
        if (node1 != null && node2 == null) {
          if (params.injectSolverForces) addForceForCollider(h1, +1, tf, pForNode1 ?? pForNode2 ?? relAnchor);
          if (damageSystem.isEnabled()) {
            if (applyDamage) {
              try { damageSystem.onImpact(node1, effMag, dt, local1 ? { localPoint: local1 } : undefined); } catch {}
            } else if (record) {
              bufferedExternal.push({ node: node1, effMag, dt, local: local1 ?? undefined });
            }
          }
        }
      }
      if (h2 != null) {
        if (node2 != null && node1 == null) {
          if (params.injectSolverForces) addForceForCollider(h2, -1, tf, pForNode2 ?? pForNode1 ?? relAnchor);
          if (damageSystem.isEnabled()) {
            if (applyDamage) {
              try { damageSystem.onImpact(node2, effMag, dt, local2 ? { localPoint: local2 } : undefined); } catch {}
            } else if (record) {
              bufferedExternal.push({ node: node2, effMag, dt, local: local2 ?? undefined });
            }
          }
        }
        if (node1 != null && node2 != null && damageSystem.isEnabled()) {
          if (applyDamage) {
            try { damageSystem.onInternalImpact(node1, node2, effMag, dt, { localPointA: local1 ?? undefined, localPointB: local2 ?? undefined }); } catch {}
          } else if (record) {
            bufferedInternal.push({ a: node1, b: node2, effMag, dt, localA: local1 ?? undefined, localB: local2 ?? undefined });
          }
        }
      }
    });
    if (profilerSample) {
      profilerSample.contactDrainMs += Math.max(0, perfNow() - timerStart);
      profilerSample.bufferedExternalContacts = Math.max(
        profilerSample.bufferedExternalContacts,
        bufferedExternal.length,
      );
      profilerSample.bufferedInternalContacts = Math.max(
        profilerSample.bufferedInternalContacts,
        bufferedInternal.length,
      );
    }
  }

  function injectPendingExternalForces(): number {
    const count = pendingExternalForces.length;
    if (count === 0) return 0;
    const timerStart = startTiming();
    const profilerSample = activeProfilerSample;
    try {
      const dt = getDt();
      for (const ef of pendingExternalForces) {
        const { body } = actorBodyForNode(ef.nodeIndex);
        const rb = body ?? world.getRigidBody(rootBody.handle);
        if (!rb) {
          if (isDev) console.warn('addForceForCollider', 'body is null', ef.nodeIndex);
          continue;
        }
        const t = rb.translation();
        const r = rb.rotation();
        const qInv = { x: -r.x, y: -r.y, z: -r.z, w: r.w };
        const pRel = { x: ef.point.x - t.x, y: ef.point.y - t.y, z: ef.point.z - t.z };
        const localPoint = applyQuatToVec3(pRel, qInv);
        const scaledForce = { x: ef.force.x * dt, y: ef.force.y * dt, z: ef.force.z * dt };
        const localForce = applyQuatToVec3(scaledForce, qInv);
        solver.addForce(
          ef.nodeIndex,
          { x: localPoint.x, y: localPoint.y, z: localPoint.z },
          { x: localForce.x, y: localForce.y, z: localForce.z },
          runtime.ExtForceMode.Force
        );
        // Map external push impulse to damage
        try {
          if (damageSystem.isEnabled()) {
            const fmag = Math.hypot(ef.force.x, ef.force.y, ef.force.z);
            damageSystem.onImpact(ef.nodeIndex, fmag, dt, { localPoint });
          }
        } catch {}
      }
    } catch (e) {
      if (isDev) console.error('[Core] addForceForCollider failed', e);
    } finally {
      pendingExternalForces.splice(0, pendingExternalForces.length);
    }
    if (profilerSample) {
      profilerSample.pendingExternalForces = Math.max(profilerSample.pendingExternalForces, count);
    }
    stopTiming(timerStart, 'externalForceMs');
    return count;
  }

  function applyDamageTick() {
    if (!damageSystem.isEnabled()) return;
    const profilerSample = activeProfilerSample;
    const timerStart = profilerSample ? perfNow() : 0;
    const dt = getDt();
    damageSystem.tick(dt, (nodeIndex, reason) => {
      try { handleNodeDestroyed(nodeIndex, reason as 'impact'|'manual'); } catch (e) {
        if (isDev) console.error('[Core] handleNodeDestroyed failed', e);
      }
    });
    if (profilerSample) {
      profilerSample.damageTickMs += Math.max(0, perfNow() - timerStart);
    }
  }

  // Helper: determine if a set of nodes contains any supports (mass=0 or chunk flag)
  const actorNodesContainSupport = (nodesList: number[]): boolean => {
    for (const n of nodesList) {
      const ch = chunks[n];
      if (!ch) {
        console.error('chunk not found', n, ch, chunks);
        throw new Error('chunk not found');
      }
      if (ch?.isSupport) return true;
      const mass = scenario.nodes[n]?.mass ?? 0;
      if (!(mass > 0)) return true;
    }
    return false;
  };

  // Rebuild the collider → node mapping from current chunk state
  function rebuildColliderToNodeFromChunks() {
    const timerStart = startTiming();
    let restored = 0;
    for (const seg of chunks) {
      if (seg && seg.colliderHandle != null) {
        colliderToNode.set(seg.colliderHandle, seg.nodeIndex);
        restored += 1;
      }
    }
    if (process.env.NODE_ENV !== 'production') {
      console.warn('[Core] Rebuilt colliderToNode from chunks', { restored, totalChunks: chunks.length });
    }
    stopTiming(timerStart, 'rebuildColliderMapMs');
  }

  function enqueueProjectile(s: ProjectileSpawn) {
    // if (process.env.NODE_ENV !== 'production') console.debug('[Core] enqueueProjectile', s);
    pendingBallSpawns.push(s);
    safeFrames = Math.max(safeFrames, 1);
  }

  // Now that options are ready, apply initial collision groups
  try {
    applyCollisionGroupsForBody(rootBody, {
      mode: singleCollisionModeSetting,
      groundBodyHandle: groundBody.handle,
    });
  } catch {}
  try {
    applyCollisionGroupsForBody(groundBody, {
      mode: singleCollisionModeSetting,
      groundBodyHandle: groundBody.handle,
    });
  } catch {}

  function setGravity(g: number) {
    const newGravity: RAPIER.Vector = { x: 0, y: g, z: 0 };
    try {
      world.gravity = newGravity;
    } catch {
      world.gravity = newGravity;
    }
  }
  function setSolverGravityEnabled(v: boolean) { solverGravityEnabled = !!v; }

  type ActorGravityExtension = {
    addActorGravity(actorIndex: number, localGravity?: Vec3): boolean;
  };

  const readWorldGravityVector = (): Vec3 => {
    const g = world.gravity;
    return { x: g?.x ?? 0, y: g?.y ?? 0, z: g?.z ?? 0 };
  };

  const gravityHasMagnitude = (value: Vec3) => value.x !== 0 || value.y !== 0 || value.z !== 0;

  const toLocalGravityVector = (body: RAPIER.RigidBody, worldGravity: Vec3): Vec3 => {
    const rotation = body.rotation();
    const inverse = { x: -rotation.x, y: -rotation.y, z: -rotation.z, w: rotation.w };
    return applyQuatToVec3(worldGravity, inverse);
  };

  function applyGravityToAllActors(worldGravity: Vec3): boolean {
    const solverWithExtension = solver as typeof solver & ActorGravityExtension;
    if (typeof solverWithExtension.addActorGravity !== 'function') {
      return false;
    }
    for (const [actorIndex, { bodyHandle }] of actorMap.entries()) {
      const body = world.getRigidBody(bodyHandle);
      const localGravity = body ? toLocalGravityVector(body, worldGravity) : worldGravity;
      solverWithExtension.addActorGravity(actorIndex, localGravity);
    }
    return true;
  }

  const applySolverGravityForFrame = () => {
    if (!solverGravityEnabled) return;
    const worldGravity = readWorldGravityVector();
    if (!gravityHasMagnitude(worldGravity)) return;
    const applied = applyGravityToAllActors(worldGravity);
    if (!applied) {
      solver.addGravity(worldGravity);
    }
  };

  function getRigidBodyCount(): number {
    try {
      const getCount = (world as WorldWithBodyCount).numRigidBodies;
      if (typeof getCount === 'function') return getCount.call(world);
    } catch {}
    let count = 0;
    try {
      world.forEachRigidBody(() => {
        count += 1;
      });
    } catch {}
    return count;
  }

  function addForceForCollider(handle: number, direction: number, totalForce: { x:number; y:number; z:number }, worldPoint: { x:number; y:number; z:number }) {
    if (!colliderToNode.has(handle)) return;
    const nodeIndex = colliderToNode.get(handle);
    if (nodeIndex == null) return;

    // Transform world force/point into the owning actor's local space
    const { body } = actorBodyForNode(nodeIndex);
    const rb = body ?? world.getRigidBody(rootBody.handle);
    if (!rb) {
      console.warn('addForceForCollider', 'body is null', handle, nodeIndex);
      return;
    }
    const bt = rb.translation();
    const br = rb.rotation();
    const qInv = { x: -br.x, y: -br.y, z: -br.z, w: br.w };

    const fWorld = { x: (totalForce.x ?? 0) * direction, y: (totalForce.y ?? 0) * direction, z: (totalForce.z ?? 0) * direction };
    const pRel = { x: (worldPoint.x ?? 0) - (bt.x ?? 0), y: (worldPoint.y ?? 0) - (bt.y ?? 0), z: (worldPoint.z ?? 0) - (bt.z ?? 0) };

    const localForce = applyQuatToVec3(fWorld, qInv);
    const localPoint = applyQuatToVec3(pRel, qInv);

    solver.addForce(
      nodeIndex,
      { x: localPoint.x, y: localPoint.y, z: localPoint.z },
      // undefined,
      { x: localForce.x, y: localForce.y, z: localForce.z },
      // undefined,
      runtime.ExtForceMode.Force
    );
  }

  function getBodyForColliderHandle(handle?: number | null): RAPIER.RigidBody | null {
    if (handle == null) return null;
    try {
      const c = world.getCollider(handle);
      // const parentHandle = c ? c.parent() : undefined;
      // return typeof parentHandle === 'number' ? (world.getRigidBody(parentHandle) ?? null) : null;
      const parent: RapierRigidBody | null = c ? c.parent() : null;
      return parent ?? null;
    } catch {
      // console.error('getBodyForColliderHandle', e);
      return null;
    }
  }

  function bodyPointVelocity(body: RAPIER.RigidBody | null, point: { x:number; y:number; z:number }): { x:number; y:number; z:number } {
    if (!body) return { x: 0, y: 0, z: 0 };
    try {
      const lv = body.linvel?.();
      const av = body.angvel?.();
      const t = body.translation();
      const rx = (point.x ?? 0) - (t.x ?? 0);
      const ry = (point.y ?? 0) - (t.y ?? 0);
      const rz = (point.z ?? 0) - (t.z ?? 0);
      const cx = (av?.y ?? 0) * rz - (av?.z ?? 0) * ry;
      const cy = (av?.z ?? 0) * rx - (av?.x ?? 0) * rz;
      const cz = (av?.x ?? 0) * ry - (av?.y ?? 0) * rx;
      return { x: (lv?.x ?? 0) + cx, y: (lv?.y ?? 0) + cy, z: (lv?.z ?? 0) + cz };
    } catch { return { x: 0, y: 0, z: 0 }; }
  }

  function computeRelativeSpeed(_world: RAPIER.World, h1?: number | null, h2?: number | null, atPoint?: { x:number; y:number; z:number }): number {
    const p = atPoint ?? { x: 0, y: 0, z: 0 };
    const b1 = getBodyForColliderHandle(h1 ?? null);
    const b2 = getBodyForColliderHandle(h2 ?? null);
    const v1 = bodyPointVelocity(b1, p);
    const v2 = bodyPointVelocity(b2, p);
    const dx = v1.x - v2.x;
    const dy = v1.y - v2.y;
    const dz = v1.z - v2.z;
    return Math.hypot(dx, dy, dz);
  }

  function worldPointToActorLocal(nodeIndex: number, worldPoint: { x:number; y:number; z:number }): { x:number; y:number; z:number } | null {
    try {
      const { body } = actorBodyForNode(nodeIndex);
      const rb = body ?? world.getRigidBody(rootBody.handle);
      if (!rb) return null;
      const t = rb.translation();
      const r = rb.rotation();
      const qInv = { x: -r.x, y: -r.y, z: -r.z, w: r.w };
      const pRel = { x: (worldPoint.x ?? 0) - (t.x ?? 0), y: (worldPoint.y ?? 0) - (t.y ?? 0), z: (worldPoint.z ?? 0) - (t.z ?? 0) };
      const localPoint = applyQuatToVec3(pRel, qInv);
      return { x: localPoint.x, y: localPoint.y, z: localPoint.z };
    } catch {
      return null;
    }
  }

  function chunkWorldCenter(nodeIndex: number): { x:number; y:number; z:number } | null {
    try {
      const seg = chunks[nodeIndex];
      if (!seg) return null;
      const { body } = actorBodyForNode(nodeIndex);
      const rb = body ?? world.getRigidBody(rootBody.handle);
      if (!rb) return null;
      const t = rb.translation();
      const r = rb.rotation();
      const rotation = { x: r.x, y: r.y, z: r.z, w: r.w };
      const local = applyQuatToVec3(seg.baseLocalOffset, rotation);
      return { x: (t.x ?? 0) + local.x, y: (t.y ?? 0) + local.y, z: (t.z ?? 0) + local.z };
    } catch { return null; }
  }

  function applySleepThresholdsToBodies() {
    // Skip if sleep mode is off
    if (sleepSettings.mode === 'off') return;

    const linearThreshold = Math.max(0, sleepSettings.linear);
    const angularThreshold = Math.max(0, sleepSettings.angular);
    for (const [bodyHandle, nodes] of nodesByBodyHandle.entries()) {
      if (!nodes || nodes.size === 0) continue;
      if (bodyHandle === rootBody.handle || bodyHandle === groundBody.handle) continue;

      // Check if this body should have sleep optimization applied based on mode
      if (!shouldApplyOptimization(sleepSettings.mode, bodyHandle)) continue;

      const body = world.getRigidBody(bodyHandle);
      if (!body) continue;
      if (!body.isDynamic?.()) continue;
      if (body.isSleeping?.()) continue;
      const linvel = body.linvel?.();
      const angvel = body.angvel?.();
      const linSpeed = Math.hypot(linvel?.x ?? 0, linvel?.y ?? 0, linvel?.z ?? 0);
      const angSpeed = Math.hypot(angvel?.x ?? 0, angvel?.y ?? 0, angvel?.z ?? 0);
      const linearWithin = linSpeed <= linearThreshold;
      const angularWithin = angSpeed <= angularThreshold;
      if (!linearWithin || !angularWithin) continue;
      try {
        body.setLinvel({ x: 0, y: 0, z: 0 }, true);
      } catch {}
      try {
        body.setAngvel({ x: 0, y: 0, z: 0 }, true);
      } catch {}
      try {
        body.sleep();
      } catch {}
    }
  }

  // --- Per-body snapshot helpers (Plan A default) ---
  type BodySnapshot = Map<number, {
    t: {x:number;y:number;z:number};
    r: {x:number;y:number;z:number;w:number};
    lv: {x:number;y:number;z:number};
    av: {x:number;y:number;z:number};
    asleep: boolean;
  }>;
  function captureBodySnapshot(): BodySnapshot {
    const snap = new Map<number, { t:{x:number;y:number;z:number}; r:{x:number;y:number;z:number;w:number}; lv:{x:number;y:number;z:number}; av:{x:number;y:number;z:number}; asleep:boolean }>();
    world.forEachRigidBody((b: RAPIER.RigidBody) => {
      const h = b.handle;
      const t = b.translation();
      const r = b.rotation();
      const lv = b.linvel?.() ?? { x: 0, y: 0, z: 0 };
      const av = b.angvel?.() ?? { x: 0, y: 0, z: 0 };
      snap.set(h, { t: { x: t.x, y: t.y, z: t.z }, r: { x: r.x, y: r.y, z: r.z, w: r.w }, lv, av, asleep: b.isSleeping?.() ?? false });
    });
    return snap;
  }
  function restoreBodySnapshot(snap: BodySnapshot) {
    world.forEachRigidBody((b: RAPIER.RigidBody) => {
      const s = snap.get(b.handle);
      if (!s) return;
      try { b.setTranslation(s.t, true); } catch {}
      try { b.setRotation(s.r, true); } catch {}
      try { b.setLinvel(s.lv, true); } catch {}
      try { b.setAngvel(s.av, true); } catch {}
      try { if (s.asleep) b.sleep(); else b.wakeUp(); } catch {}
    });
  }

  function step(dtOverride?: number) {
    const prevDt = readWorldDt();
    const hasOverride = typeof dtOverride === 'number' && Number.isFinite(dtOverride) && dtOverride > 0;
    const targetDt = hasOverride ? clampStepDt(dtOverride) : clampStepDt(prevDt);
    lastStepDt = targetDt;

    const applyOverrideDt = () => {
      if (!hasOverride) return;
      setWorldDtValue(targetDt);
    };
    const restoreDt = () => {
      if (!hasOverride) return;
      setWorldDtValue(prevDt);
    };
    const stepWorld = (onError: (error: unknown) => void): boolean => {
      try {
        applyOverrideDt();
        world.step(eventQueue);
        return true;
      } catch (error) {
        onError(error);
        return false;
      }
    };

    const profilerSample =
      profiler.enabled && typeof profiler.onSample === 'function'
        ? createProfilerSample(targetDt)
        : null;
    if (profilerSample) activeProfilerSample = profilerSample;
    let passStart = profilerSample ? perfNow() : 0;
    let currentPassType: 'initial' | 'resim' = 'initial';
    let profilerFinalized = false;
    const recordPassDuration = (reasons: string[]) => {
      if (!profilerSample) return;
      const duration = Math.max(0, perfNow() - passStart);
      profilerSample.passes.push({
        type: currentPassType,
        durationMs: duration,
        reasons: [...reasons],
      });
      passStart = perfNow();
      currentPassType = 'resim';
    };
    const finalizeProfilerSample = () => {
      if (!profilerSample || profilerFinalized) {
        activeProfilerSample = null;
        return;
      }
      recordPassDuration([]);
      profilerFinalized = true;
      activeProfilerSample = null;
      const totalMs = profilerSample.passes.reduce((sum, pass) => sum + pass.durationMs, 0);
      profilerSample.totalMs = totalMs;
      profilerSample.initialPassMs =
        profilerSample.passes.find((pass) => pass.type === 'initial')?.durationMs ?? totalMs;
      profilerSample.resimMs = totalMs - profilerSample.initialPassMs;
      profilerSample.projectiles = projectiles.length;
      profilerSample.rigidBodies = getRigidBodyCount();
      const colliderStats = captureBodyColliderStats();
      if (colliderStats) {
        profilerSample.bodyCount = colliderStats.bodyCount;
        profilerSample.bodyColliderCountMin = colliderStats.min;
        profilerSample.bodyColliderCountMax = colliderStats.max;
        profilerSample.bodyColliderCountAvg = colliderStats.avg;
        profilerSample.bodyColliderCountMedian = colliderStats.median;
        profilerSample.bodyColliderCountP95 = colliderStats.p95;
      }
      profiler.onSample?.({
        ...profilerSample,
        passes: clonePasses(profilerSample.passes),
      });
    };
    const finalizeAndReturn = () => {
      try {
        applySleepThresholdsToBodies();
      } catch {}
      finalizeProfilerSample();
      return;
    };

    try {
      // Apply queued spawns up front
      applyPendingSpawns();

      const useWorldSnapshot = snapshotMode === 'world';
      const maxPasses = Math.max(0, maxResimulationPasses);
      const fractureResimEnabled = !!resimulateOnFracture && maxPasses > 0;
      const damageResimEnabled =
        damageSystem.isEnabled() &&
        !!resimulateOnDamageDestroy &&
        maxPasses > 0;
      let passesRemaining = maxPasses;

      // Allow multiple rollback attempts in a single frame.
      // Each loop re-simulates the same dt and only breaks once a pass completes without additional rollbacks.
      while (true) {
        let bodySnap: BodySnapshot | null = null;
        let worldSnap: Uint8Array | null = null;
        const snapshotNeeded =
          (fractureResimEnabled || damageResimEnabled) &&
          passesRemaining > 0;
        if (snapshotNeeded) {
          if (useWorldSnapshot) {
            try {
              const snapshotStart = profilerSample ? perfNow() : 0;
              worldSnap = world.takeSnapshot();
              if (profilerSample) {
                profilerSample.snapshotCaptureMs += Math.max(0, perfNow() - snapshotStart);
                if (worldSnap) {
                  profilerSample.snapshotBytes = Math.max(
                    profilerSample.snapshotBytes,
                    worldSnap.byteLength ?? 0,
                  );
                }
              }
            } catch (e) {
              console.warn(
                '[Core] World.takeSnapshot failed; falling back to perBody',
                e,
              );
              const fallbackStart = profilerSample ? perfNow() : 0;
              bodySnap = captureBodySnapshot();
              if (profilerSample) {
                profilerSample.snapshotCaptureMs += Math.max(0, perfNow() - fallbackStart);
              }
            }
          } else {
            const snapshotStart = profilerSample ? perfNow() : 0;
            bodySnap = captureBodySnapshot();
            if (profilerSample) {
              profilerSample.snapshotCaptureMs += Math.max(0, perfNow() - snapshotStart);
            }
          }
        }

        clearBufferedContacts();
        if (colliderToNode.size === 0) {
          try {
            rebuildColliderToNodeFromChunks();
          } catch {}
          if (
            colliderToNode.size === 0 &&
            process.env.NODE_ENV !== 'production' &&
            !warnedColliderMapEmptyOnce
          ) {
            console.warn(
              '[Core] colliderToNode is empty before event drain; contact forces will be dropped',
            );
            warnedColliderMapEmptyOnce = true;
          }
        }
        preStepSweep();
        const rapierStart = profilerSample ? perfNow() : 0;
        const stepped = stepWorld((error) => console.error('world.step', error));
        if (profilerSample) {
          profilerSample.rapierStepMs += Math.max(0, perfNow() - rapierStart);
        }
        if (!stepped) return finalizeAndReturn();
        drainContactForces({
          injectSolverForces: true,
          applyDamage: false,
          recordForReplay: true,
        });

        applySolverGravityForFrame();
        const solverStart = profilerSample ? perfNow() : 0;
        solver.update();
        if (profilerSample) {
          profilerSample.solverUpdateMs += Math.max(0, perfNow() - solverStart);
        }

        const hasFracture = solver.overstressedBondCount() > 0;
        const dt = getDt();
        const damagePreviewEnabled =
          damageSystem.isEnabled() && !!resimulateOnDamageDestroy;
        const shouldSnapshotDamage =
          damageSystem.isEnabled() &&
          (damagePreviewEnabled ||
            ((fractureResimEnabled || damageResimEnabled) && hasFracture));
        let damageSnapshot: DamageStateSnapshot | null = null;
        if (shouldSnapshotDamage) {
          const damageSnapshotTimer = startTiming();
          try {
            damageSnapshot = damageSystem.captureImpactState();
          } catch {}
          stopTiming(damageSnapshotTimer, 'damageSnapshotMs');
        }
        let damagePreviewDestroyed: number[] = [];
        if (damageSystem.isEnabled()) {
          replayBufferedContacts();
          if (damagePreviewEnabled) {
            try {
              const previewStart = profilerSample ? perfNow() : 0;
              damagePreviewDestroyed = damageSystem.previewTick(dt) ?? [];
              if (profilerSample) {
                profilerSample.damagePreviewMs += Math.max(0, perfNow() - previewStart);
              }
            } catch {
              damagePreviewDestroyed = [];
            }
          }
        }
        const shouldResimFracture = fractureResimEnabled && hasFracture;
        const shouldResimDamage =
          damageResimEnabled && damagePreviewDestroyed.length > 0;
        const resimReasons: string[] = [];
        if (shouldResimFracture) resimReasons.push('fracture');
        if (shouldResimDamage) resimReasons.push('damage');

        if (!shouldResimFracture && !shouldResimDamage) {
          const externalForceCount = injectPendingExternalForces();
          if (externalForceCount > 0 && process.env.NODE_ENV !== 'production') {
            try {
              (window as DebugWindow).debugStressSolver?.printSolver?.();
            } catch {}
          }
          applyDamageTick();
          flushPendingDamageFractures();
          removeDisabledHandles();

          if (hasFracture) {
            try {
              const fractureStart = profilerSample ? perfNow() : 0;
              const perActor = profiledGenerateFractureCommands();
              const splitEvents = profiledApplyFractureCommands(
                perActor,
              );
              if (Array.isArray(splitEvents) && splitEvents.length > 0) {
                queueSplitResults(splitEvents);
                applyPendingMigrations();
                removeDisabledHandles();
              }
              if (profilerSample) {
                profilerSample.fractureMs += Math.max(0, perfNow() - fractureStart);
              }
            } catch (e) {
              console.error('[Core] applyFractureCommands', e);
            }
          }
          return finalizeAndReturn();
        }

        if (!snapshotNeeded) {
          /*
          if (process.env.NODE_ENV !== 'production') {
            console.warn(
              '[Core] Resimulation requested but no passes remaining',
              {
                hasFracture,
                damagePreviewCount: damagePreviewDestroyed.length,
              },
            );
          }
          */
          const externalForceCount = injectPendingExternalForces();
          if (externalForceCount > 0 && process.env.NODE_ENV !== 'production') {
            try {
              (window as DebugWindow).debugStressSolver?.printSolver?.();
            } catch {}
          }
          applyDamageTick();
          flushPendingDamageFractures();
          removeDisabledHandles();
          if (hasFracture) {
            try {
              const fractureStart = profilerSample ? perfNow() : 0;
              const perActor = profiledGenerateFractureCommands();
              const splitEvents = profiledApplyFractureCommands(
                perActor,
              ) as Array<{
                parentActorIndex: number;
                children: Array<{ actorIndex: number; nodes: number[] }>;
              }> | undefined;
              if (Array.isArray(splitEvents) && splitEvents.length > 0) {
                queueSplitResults(splitEvents);
                applyPendingMigrations();
                removeDisabledHandles();
              }
              if (profilerSample) {
                profilerSample.fractureMs += Math.max(0, perfNow() - fractureStart);
              }
            } catch (e) {
              console.error('[Core] applyFractureCommands (fallback)', e);
            }
          }
          return finalizeAndReturn();
        }

        if (resimReasons.length > 0) {
          if (profilerSample) {
            profilerSample.resimPasses += 1;
            profilerSample.resimReasons.push(...resimReasons);
          }
          recordPassDuration(resimReasons);
        }

        passesRemaining -= 1;
        const preDestroyedNodes = shouldResimDamage
          ? Array.from(new Set(damagePreviewDestroyed))
          : [];

        try {
          if (useWorldSnapshot && worldSnap) {
            const restoreStart = profilerSample ? perfNow() : 0;
            const newWorld = RAPIER.World.restoreSnapshot(worldSnap);
            if (newWorld) {
              world = newWorld;
              try {
                if (corePublic) {
                  corePublic.world = world;
                }
              } catch {}
              try {
                onWorldReplaced?.(world);
              } catch {}
              if (profilerSample) {
                profilerSample.snapshotRestoreMs += Math.max(0, perfNow() - restoreStart);
              }
            }
          } else if (bodySnap) {
            const restoreStart = profilerSample ? perfNow() : 0;
            restoreBodySnapshot(bodySnap);
            if (profilerSample) {
              profilerSample.snapshotRestoreMs += Math.max(0, perfNow() - restoreStart);
            }
          }
          if (damageSnapshot) {
            const damageRestoreTimer = startTiming();
            try {
              damageSystem.restoreImpactState(damageSnapshot);
            } catch {}
            stopTiming(damageRestoreTimer, 'damageRestoreMs');
          }
        } catch (e) {
          console.error('[Core] rollback failed; proceeding without resim', e);
          const externalForceCount = injectPendingExternalForces();
          if (externalForceCount > 0 && process.env.NODE_ENV !== 'production') {
            try {
              (window as DebugWindow).debugStressSolver?.printSolver?.();
            } catch {}
          }
          applyDamageTick();
          flushPendingDamageFractures();
          removeDisabledHandles();
          if (hasFracture) {
        try {
          const perActor = profiledGenerateFractureCommands();
          const splitEvents = profiledApplyFractureCommands(
            perActor,
          ) as Array<{
            parentActorIndex: number;
            children: Array<{ actorIndex: number; nodes: number[] }>;
          }> | undefined;
              if (Array.isArray(splitEvents) && splitEvents.length > 0) {
                queueSplitResults(splitEvents);
                applyPendingMigrations();
                removeDisabledHandles();
              }
            } catch (err) {
              console.error('[Core] applyFractureCommands (rollback fallback)', err);
            }
          }
          return finalizeAndReturn();
        }

        try {
          const fractureStart = profilerSample ? perfNow() : 0;
          const perActor = profiledGenerateFractureCommands();
          const splitEvents = profiledApplyFractureCommands(
            perActor,
          ) as Array<{
            parentActorIndex: number;
            children: Array<{ actorIndex: number; nodes: number[] }>;
          }> | undefined;
          if (Array.isArray(splitEvents) && splitEvents.length > 0) {
            queueSplitResults(splitEvents);
            applyPendingMigrations();
            removeDisabledHandles();
          }
          if (profilerSample) {
            profilerSample.fractureMs += Math.max(0, perfNow() - fractureStart);
          }
        } catch (e) {
          console.error('[Core] applyFractureCommands (resim)', e);
        }

        if (preDestroyedNodes.length > 0) {
          const preDestroyTimer = startTiming();
          damageSystem.applyPreDestruction(preDestroyedNodes);
          stopTiming(preDestroyTimer, 'damagePreDestroyMs');
          for (const nodeIndex of preDestroyedNodes) {
            const seg = chunks[nodeIndex];
            if (!seg || seg.destroyed) continue;
            if (seg.isSupport) continue;
            try {
              handleNodeDestroyed(nodeIndex, 'impact');
            } catch (err) {
              if (isDev)
                console.warn(
                  '[Core] preDestroy handleNodeDestroyed failed',
                  err,
                );
            }
          }
        }
        flushPendingDamageFractures();
        removeDisabledHandles();
        // Loop to rerun detection with updated solver state.
      }
    } finally {
      finalizeProfilerSample();
      restoreDt();
    }
  }

  // Back-compat aliases
  const stepEventful = step;
  const stepSafe = step;

  function queueSplitResults(splitEvents: Array<{ parentActorIndex:number; children:Array<{ actorIndex:number; nodes:number[] }> }>) {
    const timerStart = startTiming();
    if (isDev) console.debug('[Core] queueSplitResults', splitEvents?.[0]?.children);
    for (const evt of splitEvents) {
      const parentActorIndex = evt?.parentActorIndex;
      const children = Array.isArray(evt?.children) ? evt.children : [];
      const parentEntry = actorMap.get(parentActorIndex);
      const parentBodyHandle = parentEntry?.bodyHandle ?? rootBody.handle;
      const plannerChildren: PlannerChild[] = [];
      for (const child of children) {
        const nodes = Array.isArray(child.nodes) ? child.nodes.slice() : [];
        if (nodes.length === 0) continue;
        if (activeProfilerSample) {
          if (!activeProfilerSample.splitChildCounts) {
            activeProfilerSample.splitChildCounts = [];
          }
          activeProfilerSample.splitChildCounts.push(nodes.length);
        }
        const isActorSupport = actorNodesContainSupport(nodes);
        const shouldCullSingle =
          skipSingleBodiesEnabled &&
          !isActorSupport &&
          nodes.length === 1;
        if (shouldCullSingle) {
          const nodeIndex = nodes[0];
          try {
            handleNodeDestroyed(nodeIndex, 'manual');
          } catch (err) {
            if (isDev) console.warn('[Core] skipSingleBodies handleNodeDestroyed failed', err);
          }
          continue;
        }
        for (const n of nodes) nodeToActor.set(n, child.actorIndex);
        const index = plannerChildren.length;
        plannerChildren.push({
          index,
          actorIndex: child.actorIndex,
          nodes,
          isSupport: isActorSupport,
        });
      }
      if (plannerChildren.length === 0) continue;

      const parentNodes =
        nodesByBodyHandle.get(parentBodyHandle) ?? new Set<number>();
      const parentRigidBody = world.getRigidBody(parentBodyHandle);
      const parentIsFixed = !!parentRigidBody?.isFixed?.();
      let plannerDuration = 0;
      const plan = planSplitMigration(
        [
          {
            handle: parentBodyHandle,
            nodeIndices: parentNodes,
            isFixed: parentIsFixed,
          },
        ],
        plannerChildren,
        {
          onDuration: (ms) => {
            plannerDuration += ms;
          },
        },
      );
      if (activeProfilerSample) {
        activeProfilerSample.splitPlannerMs =
          (activeProfilerSample.splitPlannerMs ?? 0) + plannerDuration;
      }

      const reusedChildren = new Set<number>();
      for (const reuse of plan.reuse) {
        const entry = plannerChildren[reuse.childIndex];
        if (!entry) continue;
        actorMap.set(entry.actorIndex, { bodyHandle: reuse.bodyHandle });
        reusedChildren.add(entry.index);
      }

      for (const create of plan.create) {
        const entry = plannerChildren[create.childIndex];
        if (!entry) continue;
        pendingBodiesToCreate.push({
          actorIndex: entry.actorIndex,
          inheritFromBodyHandle: parentBodyHandle,
          nodes: entry.nodes.slice(),
          isSupport: entry.isSupport,
        });
        actorMap.set(entry.actorIndex, { bodyHandle: parentBodyHandle });
      }
    }
    stopTiming(timerStart, 'splitQueueMs');
  }

  function applyPendingSpawns() {
    if (pendingBallSpawns.length === 0) return;
    const timerStart = startTiming();
    const list = pendingBallSpawns.splice(0, pendingBallSpawns.length);
    // if (process.env.NODE_ENV !== 'production') console.debug('[Core] applyPendingSpawns', { count: list.length });
    for (const s of list) spawnProjectile(s);
    stopTiming(timerStart, 'spawnMs');
  }

  function spawnProjectile(params: ProjectileSpawn) {
    const start = params.start ?? { x: params.x, y: 8, z: params.z };
    const bodyDesc = RAPIER.RigidBodyDesc.dynamic()
      .setTranslation(start.x, start.y, start.z)
      .setCanSleep(false)
      .setLinearDamping(0.0)
      .setAngularDamping(0.0)
      .setUserData({ projectile: true });
    try {
      (bodyDesc as MaybeCcdBodyDesc).setCcdEnabled?.(true);
    } catch {}
    if (params.linvel) {
      bodyDesc.setLinvel(params.linvel.x, params.linvel.y, params.linvel.z);
    } else if (typeof params.linvelY === 'number') {
      bodyDesc.setLinvel(0, params.linvelY, 0);
    }
    const body = world.createRigidBody(bodyDesc);
    try { body.userData = { projectile: true }; } catch {}

    const shape = params.type === 'ball' ? RAPIER.ColliderDesc.ball(params.radius) : RAPIER.ColliderDesc.cuboid(params.radius, params.radius, params.radius);
    const collider = world.createCollider(
      shape
        .setMass(params.mass)
        .setFriction(params.friction)
        .setRestitution(params.restitution)
        .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
        .setContactForceEventThreshold(0.0),
      body
    );
    // Apply collision-group policy for projectiles
    try {
      applyCollisionGroupsForBody(body, {
        mode: singleCollisionModeSetting,
        groundBodyHandle: groundBody.handle,
      });
    } catch {}
    if (process.env.NODE_ENV !== 'production') console.debug('[Core] spawnProjectile', { body: body.handle, collider: collider.handle, start, params });
    projectiles.push({
      bodyHandle: body.handle,
      radius: params.radius,
      type: params.type,
      spawnTime: nowSeconds(),
    });
  }

  function applyPendingMigrations() {
    // console.log('applyPendingMigrations', pendingBodiesToCreate.length, pendingColliderMigrations.length);

    // Create child bodies
    if (pendingBodiesToCreate.length > 0) {
      const bodyCreateTimer = startTiming();
      const list = pendingBodiesToCreate.splice(0, pendingBodiesToCreate.length);
      for (const pb of list) {
        const inherit = world.getRigidBody(pb.inheritFromBodyHandle);
        const desc = pb.isSupport ? RAPIER.RigidBodyDesc.fixed() : RAPIER.RigidBodyDesc.dynamic();
        if (inherit) {
          const pt = inherit.translation();
          const pq = inherit.rotation();
          const lv = inherit.linvel?.();
          const av = inherit.angvel?.();
          const linDamp = typeof inherit.linearDamping === 'function' ? inherit.linearDamping() : undefined;
          const angDamp = typeof inherit.angularDamping === 'function' ? inherit.angularDamping() : undefined;
          const angvelVec = { x: av?.x ?? 0, y: av?.y ?? 0, z: av?.z ?? 0 };
          desc.setTranslation(pt.x, pt.y, pt.z)
            .setRotation(pq)
            .setLinvel(lv?.x ?? 0, lv?.y ?? 0, lv?.z ?? 0)
            .setAngvel(angvelVec);
          if (typeof linDamp === 'number') desc.setLinearDamping(linDamp);
          if (typeof angDamp === 'number') desc.setAngularDamping(angDamp);
          desc.setUserData({
            body: true,
            recreated: true,
          });
        }
        try {
          if (!pb.isSupport) (desc as MaybeCcdBodyDesc).setCcdEnabled?.(true);
        } catch {}
        const body = world.createRigidBody(desc);

        // Apply higher damping to small bodies (few colliders) to reduce jitter
        // Only apply immediately if mode is 'always'; for 'afterGroundCollision' it will be applied later
        const colliderCount = pb.nodes.length;
        const isSmallBody = colliderCount > 0 && colliderCount <= smallBodyDampingSettings.colliderCountThreshold && !pb.isSupport;
        if (isSmallBody && smallBodyDampingSettings.mode === 'always') {
          try {
            const currentLinDamp = typeof body.linearDamping === 'function' ? body.linearDamping() : 0;
            const currentAngDamp = typeof body.angularDamping === 'function' ? body.angularDamping() : 0;
            const newLinDamp = Math.max(currentLinDamp, smallBodyDampingSettings.minLinearDamping);
            const newAngDamp = Math.max(currentAngDamp, smallBodyDampingSettings.minAngularDamping);
            body.setLinearDamping(newLinDamp);
            body.setAngularDamping(newAngDamp);
          } catch {}
        }
        // Track small bodies for potential later damping application (afterGroundCollision mode)
        if (isSmallBody) {
          smallBodiesPendingDamping.add(body.handle);
        }

        actorMap.set(pb.actorIndex, { bodyHandle: body.handle });
        for (const nodeIndex of pb.nodes) pendingColliderMigrations.push({ nodeIndex, targetBodyHandle: body.handle });
      }
      stopTiming(bodyCreateTimer, 'bodyCreateMs');
    }

    // Migrate colliders to new bodies
    if (pendingColliderMigrations.length > 0) {
      const colliderMigrationTimer = startTiming();
      const jobs = pendingColliderMigrations.splice(0, pendingColliderMigrations.length);
      const createdCountByBody = new Map<number, number>();
      for (const mig of jobs) {
        const seg = chunks[mig.nodeIndex];
        if (!seg) continue;
        unregisterNodeBodyLink(seg.nodeIndex, seg.bodyHandle);
        if (seg.colliderHandle != null) {
          const oldC = world.getCollider(seg.colliderHandle);
          if (oldC) oldC.setEnabled(false);
          colliderToNode.delete(seg.colliderHandle);
          disabledCollidersToRemove.add(seg.colliderHandle);
          seg.colliderHandle = null;
        }
        // Skip creating colliders for destroyed segments (standardized path)
        if (seg.destroyed) continue;
        const halfX = seg.size.x * 0.5;
        const halfY = seg.size.y * 0.5;
        const halfZ = seg.size.z * 0.5;
        const body = world.getRigidBody(mig.targetBodyHandle);
        if (!body) continue;

        const tx = seg.isSupport && seg.baseWorldPosition ? seg.baseWorldPosition.x : seg.baseLocalOffset.x;
        const ty = seg.isSupport && seg.baseWorldPosition ? seg.baseWorldPosition.y : seg.baseLocalOffset.y;
        const tz = seg.isSupport && seg.baseWorldPosition ? seg.baseWorldPosition.z : seg.baseLocalOffset.z;

        const node = scenario.nodes[mig.nodeIndex];
        const nodeMass = node.mass ?? 1;
        // const isSupport = nodeMass === 0;

        const desc = buildColliderDescForNode({ nodeIndex: mig.nodeIndex, halfX, halfY, halfZ, isSupport: seg.isSupport })
          .setMass(nodeMass)
          .setTranslation(tx, ty, tz)
          .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
          .setContactForceEventThreshold(0.0)
          .setFriction(0.85)
          .setRestitution(0.8);
          // .setFriction(0.25)
          // .setRestitution(0.0);
        const col = world.createCollider(desc, body);
        seg.bodyHandle = body.handle;
        seg.colliderHandle = col.handle;
        seg.detached = true;
        registerNodeBodyLink(seg.nodeIndex, seg.bodyHandle);
        colliderToNode.set(col.handle, seg.nodeIndex);
        createdCountByBody.set(body.handle, (createdCountByBody.get(body.handle) ?? 0) + 1);
        // Update groups after migration; collider count may be 1
        try {
          applyCollisionGroupsForBody(body, {
            mode: singleCollisionModeSetting,
            groundBodyHandle: groundBody.handle,
          });
        } catch {}
      }
      // Do not proactively remove bodies here; allow existing sweeps to handle
      stopTiming(colliderMigrationTimer, 'colliderRebuildMs');
    }
  }

  function removeDisabledHandles() {
    const timerStart = startTiming();
    for (const h of Array.from(disabledCollidersToRemove)) {
      const c = world.getCollider(h);
      if (c) world.removeCollider(c, false);
      disabledCollidersToRemove.delete(h);
    }
    for (const bh of Array.from(bodiesToRemove)) {
      const b = world.getRigidBody(bh);
      if (b) {
        world.removeRigidBody(b);
        nodesByBodyHandle.delete(bh);
      }
      bodiesToRemove.delete(bh);
    }
    stopTiming(timerStart, 'cleanupDisabledMs');
  }

  function preStepSweep() {
    const timerStart = startTiming();
    // Cull invalid collider → node mappings only when collider truly no longer exists
    for (const [h] of Array.from(colliderToNode.entries())) {
      const c = world.getCollider(h);
      if (!c) {
        colliderToNode.delete(h);
      }
    }
    world.forEachRigidBody((b: RAPIER.RigidBody) => {
      if (!b) return;
      const handle = b.handle;
      if (handle === rootBody.handle || handle === groundBody.handle) return;
      try {
        const rb = b as RigidBodyWithColliderCount;
        const count = typeof rb.numColliders === 'function' ? rb.numColliders() : 0;
        if (count === 0) {
          bodiesToRemove.add(handle);
        }
      } catch {}
    });
    stopTiming(timerStart, 'preStepSweepMs');
  }

  function getSolverDebugLines() {
    const lines = solver.fillDebugRender({ mode: runtime.ExtDebugMode.Max, scale: 1.0 }) || [];
    return lines as Array<{ p0: Vec3; p1: Vec3; color0: number; color1: number }>;
  }

  function applyExternalForce(nodeIndex: number, worldPoint: Vec3, worldForce: Vec3) {
    pendingExternalForces.push({ nodeIndex, point: { x: worldPoint.x, y: worldPoint.y, z: worldPoint.z }, force: { x: worldForce.x, y: worldForce.y, z: worldForce.z } });
    safeFrames = Math.max(safeFrames, 1);
  }

  function getNodeBonds(nodeIndex: number): BondRef[] {
    const indices = bondsByNode.get(nodeIndex) ?? [];
    const out: BondRef[] = [];
    for (const bi of indices) {
      if (removedBondIndices.has(bi)) continue;
      const b = bondTable[bi];
      if (!b) continue;
      out.push({ index: b.index, node0: b.node0, node1: b.node1, area: b.area, centroid: b.centroid, normal: b.normal });
    }
    return out;
  }

  function cutBond(bondIndex: number): boolean {
    if (removedBondIndices.has(bondIndex)) return false;
    const b = bondTable[bondIndex];
    if (!b) return false;
    // Map node -> actorIndex from live solver view
    let actorIndexA: number | undefined;
    let actorIndexB: number | undefined;
    try {
      const solverWithActors = solver as typeof solver & SolverActorsApi;
      const actors = solverWithActors.actors?.() ?? [];
      const nodeToActor = new Map<number, number>();
      for (const a of actors) {
        for (const n of a.nodes ?? []) nodeToActor.set(n, a.actorIndex);
      }
      actorIndexA = nodeToActor.get(b.node0);
      actorIndexB = nodeToActor.get(b.node1);
    } catch {}
    if (actorIndexA == null || actorIndexB == null || actorIndexA !== actorIndexB) {
      // If endpoints aren’t in same actor, either already split or invalid
      removedBondIndices.add(bondIndex);
      return false;
    }
    const fractureSets = [
      { actorIndex: actorIndexA, fractures: [{ userdata: bondIndex, nodeIndex0: b.node0, nodeIndex1: b.node1, health: 1e9 }] },
    ];
    let applied = false;
    try {
      const splitEvents = profiledApplyFractureCommands(fractureSets);
      removedBondIndices.add(bondIndex);
      if (Array.isArray(splitEvents) && splitEvents.length > 0) {
        queueSplitResults(splitEvents as Array<{ parentActorIndex:number; children:Array<{ actorIndex:number; nodes:number[] }> }>);
        safeFrames = Math.max(safeFrames, 2);
      }
      applied = true;
    } catch (e) {
      if (process.env.NODE_ENV !== 'production') console.warn('[Core] cutBond failed', bondIndex, e);
    }
    return applied;
  }

  function cutNodeBonds(nodeIndex: number): boolean {
    const bonds = getNodeBonds(nodeIndex);
    if (bonds.length === 0) return false;
    let any = false;
    for (const br of bonds) any = cutBond(br.index) || any;
    return any;
  }

  function enqueueDamageFracturesForNode(nodeIndex: number) {
    const bonds = getNodeBonds(nodeIndex);
    if (bonds.length === 0) return;
    for (const br of bonds) {
      if (removedBondIndices.has(br.index)) continue;
      const actor0 = nodeToActor.get(br.node0);
      const actor1 = nodeToActor.get(br.node1);
      const actorIndex = actor0 != null ? actor0 : actor1;
      if (actorIndex == null) continue;
      if (actor0 != null && actor1 != null && actor0 !== actor1) continue;
      let set = pendingDamageFractures.get(actorIndex);
      if (!set) {
        set = new Set<number>();
        pendingDamageFractures.set(actorIndex, set);
      }
      set.add(br.index);
    }
  }

  function flushPendingDamageFractures() {
    if (pendingDamageFractures.size === 0) return;
    const timerStart = startTiming();
    const fractureSets: Array<{ actorIndex:number; fractures:Array<{ userdata:number; nodeIndex0:number; nodeIndex1:number; health:number }> }> = [];
    for (const [actorIndex, bondSet] of Array.from(pendingDamageFractures.entries())) {
      const fractures: Array<{ userdata:number; nodeIndex0:number; nodeIndex1:number; health:number }> = [];
      for (const bondIndex of Array.from(bondSet.values())) {
        const bond = bondTable[bondIndex];
        if (!bond || removedBondIndices.has(bondIndex)) continue;
        fractures.push({ userdata: bondIndex, nodeIndex0: bond.node0, nodeIndex1: bond.node1, health: 1e9 });
        removedBondIndices.add(bondIndex);
      }
      if (fractures.length > 0) fractureSets.push({ actorIndex, fractures });
    }
    pendingDamageFractures.clear();
    if (fractureSets.length === 0) {
      stopTiming(timerStart, 'damageFlushMs');
      return;
    }
    try {
      const splitEvents = profiledApplyFractureCommands(fractureSets as Array<{ actorIndex:number; fractures:Array<{ userdata:number; nodeIndex0:number; nodeIndex1:number; health:number }> }>);
      if (Array.isArray(splitEvents) && splitEvents.length > 0) {
        queueSplitResults(splitEvents as Array<{ parentActorIndex:number; children:Array<{ actorIndex:number; nodes:number[] }> }>);
        applyPendingMigrations();
        removeDisabledHandles();
      }
    } catch (e) {
      if (isDev) console.error('[Core] flushPendingDamageFractures failed', e);
    }
    stopTiming(timerStart, 'damageFlushMs');
  }

  function dispose() {
    try { solver.destroy?.(); } catch {}
  }

  function handleNodeDestroyed(nodeIndex: number, reason: 'impact'|'manual') {
    const seg = chunks[nodeIndex];
    if (!seg) return;
    // Ensure flags
    seg.destroyed = true;
    if (seg.health != null) seg.health = 0;

    // Detach bonds in solver
    if (damageOptions.autoDetachOnDestroy) {
      if (reason === 'impact') {
        try { enqueueDamageFracturesForNode(nodeIndex); } catch {}
      } else {
        try { cutNodeBonds(nodeIndex); } catch {}
      }
    }

    // Cleanup physics collider and possibly body
    if (damageOptions.autoCleanupPhysics) {
      try {
        // Disable/remove collider (let existing cleanup pass remove it); DO NOT remove bodies here
        if (seg.colliderHandle != null) {
          const oldC = world.getCollider(seg.colliderHandle);
          if (oldC) oldC.setEnabled(false);
          colliderToNode.delete(seg.colliderHandle);
          disabledCollidersToRemove.add(seg.colliderHandle);
          seg.colliderHandle = null;
        }
      } catch (e) {
        if (isDev) console.warn('[Core] cleanup on destroy failed', e);
      }
    }

    unregisterNodeBodyLink(nodeIndex, seg.bodyHandle);

    // Notify
    try {
      const { actorIndex } = actorBodyForNode(nodeIndex);
      onNodeDestroyed?.({ nodeIndex, actorIndex, reason });
    } catch {}

    // Step safely for a couple frames
    safeFrames = Math.max(safeFrames, 2);
  }

  let corePublic: DestructibleCore | null = null;
  const api: DestructibleCore = {
    world,
    eventQueue,
    solver,
    runtime,
    rootBodyHandle: rootBody.handle,
    groundBodyHandle: groundBody.handle,
    gravity,
    chunks,
    colliderToNode,
    actorMap,
    step,
    projectiles,
    enqueueProjectile,
    stepEventful,
    stepSafe,
    setGravity,
    setSolverGravityEnabled,
    getRigidBodyCount,
    setSingleCollisionMode: (mode: SingleCollisionMode) => {
      if (mode === singleCollisionModeSetting) return;
      singleCollisionModeSetting = mode;
      try {
        world.forEachRigidBody((b) =>
          applyCollisionGroupsForBody(b, {
            mode: singleCollisionModeSetting,
            groundBodyHandle: groundBody.handle,
          }),
        );
      } catch {
        try {
          applyCollisionGroupsForBody(rootBody, {
            mode: singleCollisionModeSetting,
            groundBodyHandle: groundBody.handle,
          });
        } catch {}
        try {
          applyCollisionGroupsForBody(groundBody, {
            mode: singleCollisionModeSetting,
            groundBodyHandle: groundBody.handle,
          });
        } catch {}
        for (const { bodyHandle } of Array.from(actorMap.values())) {
          const b = world.getRigidBody(bodyHandle);
          if (b) {
            applyCollisionGroupsForBody(b, {
              mode: singleCollisionModeSetting,
              groundBodyHandle: groundBody.handle,
            });
          }
        }
      }
    },
    getSolverDebugLines,
    getNodeBonds,
    cutBond,
    cutNodeBonds,
    applyExternalForce,
    setSleepThresholds: (linear: number, angular: number) =>
      updateSleepThresholds(linear, angular),
    setSleepMode: (mode: OptimizationMode) => updateSleepMode(mode),
    getSleepSettings: () => ({ ...sleepSettings }),
    // Small body damping API
    setSmallBodyDamping: (opts: SmallBodyDampingOptions) =>
      updateSmallBodyDamping(opts),
    getSmallBodyDampingSettings: () => ({ ...smallBodyDampingSettings }),
    // Query ground collision status
    hasBodyCollidedWithGround: (bodyHandle: number) => bodiesCollidedWithGround.has(bodyHandle),
    // Damage API
    applyNodeDamage: damageSystem.isEnabled() ? (nodeIndex: number, amount: number) => { try { damageSystem.applyDirect(nodeIndex, amount); } catch {} } : undefined,
    getNodeHealth: damageSystem.isEnabled() ? (nodeIndex: number) => {
      try { return damageSystem.getHealth(nodeIndex); } catch { return null; }
    } : undefined,
    damageEnabled: damageSystem.isEnabled(),
    dispose,
    setProfiler: (config: CoreProfilerConfig | null) => setProfiler(config),
    recordProjectileCleanupDuration: (durationMs: number) => {
      recordProjectileCleanupDurationInternal(durationMs);
    },
  };
  corePublic = api;
  return api;
}

function fallbackPoint(world: RAPIER.World, handle?: number) {
  if (handle == null) return { x: 0, y: 0, z: 0 };
  const c = world.getCollider(handle);
  const parentHandle = c ? c.parent() : undefined;
  const b = typeof parentHandle === 'number' ? world.getRigidBody(parentHandle) : undefined;
  if (b) {
    const t = b.translation();
    return { x: t.x ?? 0, y: t.y ?? 0, z: t.z ?? 0 };
  }
  return { x: 0, y: 0, z: 0 };
}

type Quat = { x:number; y:number; z:number; w:number };
function applyQuatToVec3(v: Vec3, q: Quat): Vec3 {
  // Minimal quaternion-vector rotation without importing Three in core
  const x = v.x, y = v.y, z = v.z;
  const qx = q.x, qy = q.y, qz = q.z, qw = q.w;
  // quat * vec
  const ix = qw * x + qy * z - qz * y;
  const iy = qw * y + qz * x - qx * z;
  const iz = qw * z + qx * y - qy * x;
  const iw = -qx * x - qy * y - qz * z;
  // result * conj(quat)
  return {
    x: ix * qw + iw * -qx + iy * -qz - iz * -qy,
    y: iy * qw + iw * -qy + iz * -qx - ix * -qz,
    z: iz * qw + iw * -qz + ix * -qy - iy * -qx,
  };
}

// --- Collision-group helpers (moved below for clarity) ---
const GROUP_GROUND = 1 << 0;
const GROUP_SINGLE = 1 << 1;
const GROUP_MULTI = 1 << 2;
const mkGroups = (memberships: number, filter: number) => (((memberships & 0xffff) << 16) | (filter & 0xffff));

function applyGroupsForCollider(c: RAPIER.Collider, groups: number) {
  try {
    const interactionGroups = groups as InteractionGroupsValue;
    c.setCollisionGroups(interactionGroups);
    c.setSolverGroups(interactionGroups);
  } catch {}
}

function applyCollisionGroupsForBody(
  body: RAPIER.RigidBody,
  opts: { mode: SingleCollisionMode; groundBodyHandle: number },
) {
  if (!body) return;
  const n = body?.numColliders?.() ?? 0;
  const ud = (body as BodyWithUserData | undefined)?.userData;
  let memberships = 0xffff;
  let filters = 0xffff;

  if (ud?.projectile) {
    memberships = 0xffff;
    filters = 0xffff;
  } else if (opts.mode === 'noSinglePairs' || opts.mode === 'singleGround') {
    if (body.handle === opts.groundBodyHandle) {
      memberships = GROUP_GROUND;
      filters = GROUP_GROUND | GROUP_SINGLE | GROUP_MULTI;
    } else {
      const isDynamicLike = body.isDynamic() || body.isKinematic();
      const isSingle = isDynamicLike && n === 1;
      if (opts.mode === 'noSinglePairs') {
        if (isSingle) {
          memberships = GROUP_SINGLE;
          filters = GROUP_GROUND | GROUP_MULTI;
        } else {
          memberships = GROUP_MULTI;
          filters = GROUP_GROUND | GROUP_MULTI | GROUP_SINGLE;
        }
      } else {
        if (isSingle) {
          memberships = GROUP_SINGLE;
          filters = GROUP_GROUND;
        } else {
          memberships = GROUP_MULTI;
          filters = GROUP_GROUND | GROUP_MULTI;
        }
      }
    }
  } else if (opts.mode === 'singleNone') {
    const isDynamicLike = body.isDynamic() || body.isKinematic();
    const isSingle = isDynamicLike && n === 1;
    if (isSingle) {
      memberships = 0;
      filters = 0;
    } else if (body.handle === opts.groundBodyHandle) {
      memberships = GROUP_GROUND;
      filters = GROUP_GROUND | GROUP_SINGLE | GROUP_MULTI;
    }
  }

  const groups = mkGroups(memberships, filters);
  for (let i = 0; i < n; i += 1) {
    try {
      const col = body.collider(i);
      if (col) applyGroupsForCollider(col, groups);
    } catch {}
  }
}

