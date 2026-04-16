import RAPIER, { type RigidBody as RapierRigidBody } from '@dimforge/rapier3d-compat';
import { loadStressSolver } from '../stress';
import type { ExtStressSolverSettings } from '../types';
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
  DebrisCollisionMode,
  SmallBodyDampingOptions,
  DebrisCleanupOptions,
  OptimizationMode,
  FracturePolicy,
} from './types';
import { DestructibleDamageSystem, type DamageOptions, type DamageStateSnapshot } from './damage';
import { planSplitMigration, type PlannerChild, type ExistingBodyState } from './splitMigrator';
import { createScenarioNodeSizeResolver } from './scenario';
import { applyCollisionGroupsForBody as applyCollisionGroupsForBodyImpl, type CollisionGroupContext } from './collisionGroups';
import { ContactBuffer } from './contactBuffer';
import {
  computeSpeedFactor,
  computeRelativeSpeed,
  getBodyForColliderHandle,
  worldPointToBodyLocal,
  chunkWorldCenter as chunkWorldCenterHelper,
  fallbackContactPoint,
  applyProjectileMomentumBoost,
  type SpeedScalingOptions,
} from './contactHelpers';

export type BuildDestructibleCoreOptions = {
  scenario: ScenarioDesc;
  nodeSize?: (nodeIndex: number, scenario: ScenarioDesc) => Vec3;
  solverSettings?: Partial<ExtStressSolverSettings>;
  gravity?: number;
  friction?: number;
  restitution?: number;
  materialScale?: number;
  /** @deprecated Use debrisCollisionMode instead */
  singleCollisionMode?: SingleCollisionMode;
  debrisCollisionMode?: DebrisCollisionMode;
  damage?: DamageOptions & { autoDetachOnDestroy?: boolean; autoCleanupPhysics?: boolean };
  onNodeDestroyed?: (e: { nodeIndex: number; actorIndex: number; reason: 'impact'|'manual' }) => void;
  resimulateOnFracture?: boolean;
  maxResimulationPasses?: number;
  snapshotMode?: 'perBody' | 'world';
  onWorldReplaced?: (newWorld: RAPIER.World) => void;
  resimulateOnDamageDestroy?: boolean;
  /** Scale factor for contact forces fed into the stress solver (default 30).
   * Higher values make projectile impacts break more bonds. */
  contactForceScale?: number;
  /** Whether newly created split bodies should enable CCD (default true for compatibility). */
  fractureBodyCcdEnabled?: boolean;
  /** Whether spawned projectiles should enable CCD (default true). */
  projectileCcdEnabled?: boolean;
  skipSingleBodies?: boolean;
  sleepLinearThreshold?: number;
  sleepAngularThreshold?: number;
  sleepMode?: OptimizationMode;
  smallBodyDamping?: SmallBodyDampingOptions;
  debrisCleanup?: DebrisCleanupOptions;
  /** Controls fracture rate, body creation budget, and dynamic body limits.
   *  All fields default to -1 (unlimited = original behavior). */
  fracturePolicy?: FracturePolicy;
};

const isDev = typeof process !== 'undefined' ? process.env.NODE_ENV !== 'production' : true;

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
  nodeSize = createScenarioNodeSizeResolver(),
  solverSettings,
  gravity = -9.81,
  friction = 0.25,
  restitution = 0.0,
  materialScale = 1.0,
  singleCollisionMode = 'all',
  debrisCollisionMode,
  damage,
  onNodeDestroyed,
  resimulateOnFracture = true,
  maxResimulationPasses = 1,
  snapshotMode = 'perBody',
  onWorldReplaced,
  resimulateOnDamageDestroy = !!damage?.enabled,
  contactForceScale = 30,
  fractureBodyCcdEnabled = true,
  projectileCcdEnabled = true,
  skipSingleBodies = false,
  sleepLinearThreshold = 0.1,
  sleepAngularThreshold = 0.1,
  sleepMode = 'off',
  smallBodyDamping,
  debrisCleanup,
  fracturePolicy,
}: BuildDestructibleCoreOptions): Promise<DestructibleCore> {
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
    for (const cmd of commands) {
      if (!cmd.fractures) continue;
      for (const frac of cmd.fractures) {
        if (typeof frac.userdata === 'number') {
          removedBondIndices.add(frac.userdata);
        } else if (typeof frac.nodeIndex0 === 'number' && typeof frac.nodeIndex1 === 'number') {
          const bonds0 = bondsByNode.get(frac.nodeIndex0);
          if (bonds0) {
            for (const bi of bonds0) {
              const b = bondTable[bi];
              if (!b) continue;
              if ((b.node0 === frac.nodeIndex0 && b.node1 === frac.nodeIndex1) ||
                  (b.node0 === frac.nodeIndex1 && b.node1 === frac.nodeIndex0)) {
                removedBondIndices.add(bi);
                break;
              }
            }
          }
        }
      }
    }
    return result;
  };
  const recordProjectileCleanupDurationInternal = (durationMs: number) => {
    addDuration('projectileCleanupMs', durationMs);
  };

  const defaultSolverSettings = runtime.defaultExtSettings();
  const scaledSettings = { ...defaultSolverSettings };
  const skipSingleBodiesEnabled = !!skipSingleBodies;
  const bodiesCollidedWithGround = new Set<number>();
  const smallBodiesPendingDamping = new Set<number>();

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
    sleepThresholdsDirty = true;
  }
  function updateSleepMode(mode: OptimizationMode) {
    sleepSettings.mode = mode;
    sleepThresholdsDirty = true;
  }

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

  function shouldApplyOptimization(mode: OptimizationMode, bodyHandle: number): boolean {
    if (mode === 'off') return false;
    if (mode === 'always') return true;
    if (mode === 'afterGroundCollision') return bodiesCollidedWithGround.has(bodyHandle);
    return false;
  }

  const debrisCleanupSettings = {
    mode: (debrisCleanup?.mode as OptimizationMode) ?? 'always',
    debrisTtlMs: debrisCleanup?.debrisTtlMs ?? 10000,
    maxCollidersForDebris: debrisCleanup?.maxCollidersForDebris ?? 2,
  };
  const debrisCreationTimes = new Map<number, number>();

  function updateDebrisCleanup(opts: DebrisCleanupOptions) {
    if (opts.mode != null) {
      debrisCleanupSettings.mode = opts.mode;
    }
    if (typeof opts.debrisTtlMs === 'number' && Number.isFinite(opts.debrisTtlMs)) {
      debrisCleanupSettings.debrisTtlMs = Math.max(0, opts.debrisTtlMs);
    }
    if (typeof opts.maxCollidersForDebris === 'number' && Number.isFinite(opts.maxCollidersForDebris)) {
      debrisCleanupSettings.maxCollidersForDebris = Math.max(1, Math.floor(opts.maxCollidersForDebris));
    }
  }

  // ── Fracture policy settings ──
  const fracturePolicySettings = {
    maxFracturesPerFrame: fracturePolicy?.maxFracturesPerFrame ?? -1,
    maxNewBodiesPerFrame: fracturePolicy?.maxNewBodiesPerFrame ?? -1,
    maxColliderMigrationsPerFrame: fracturePolicy?.maxColliderMigrationsPerFrame ?? -1,
    maxDynamicBodies: fracturePolicy?.maxDynamicBodies ?? -1,
    minChildNodeCount: fracturePolicy?.minChildNodeCount ?? 1,
    idleSkip: fracturePolicy?.idleSkip ?? true,
  };

  function toDebrisCollisionMode(mode: SingleCollisionMode | DebrisCollisionMode): DebrisCollisionMode {
    switch (mode) {
      case 'noSinglePairs': return 'noDebrisPairs';
      case 'singleGround': return 'debrisGroundOnly';
      case 'singleNone': return 'debrisNone';
      default: return mode as DebrisCollisionMode;
    }
  }

  let debrisCollisionModeSetting: DebrisCollisionMode = debrisCollisionMode
    ? toDebrisCollisionMode(debrisCollisionMode)
    : toDebrisCollisionMode(singleCollisionMode);

  scaledSettings.maxSolverIterationsPerFrame = 24;
  scaledSettings.graphReductionLevel = 0;

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

  Object.assign(scaledSettings, solverSettings ?? {});

  const nodes = scenario.nodes.map((n) => ({ centroid: n.centroid, mass: n.mass, volume: n.volume }));
  const bonds = scenario.bonds.map((b) => ({ node0: b.node0, node1: b.node1, centroid: b.centroid, normal: b.normal, area: b.area }));

  const hasSupports = nodes.some((n) => n.mass === 0);
  if (!hasSupports) {
    console.warn('[Core] no supports (nodes with mass=0) found in scenario', scenario);
  }

  const solver = runtime.createExtSolver({ nodes, bonds, settings: scaledSettings });

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

  const rootBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0)
    .setUserData({ root: true })
  );

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

  const chunks: ChunkData[] = [];
  const colliderToNode = new Map<number, number>();
  const activeContactColliders = new Set<number>();
  const actorMap = new Map<number, { bodyHandle: number }>();
  const nodesByBodyHandle = new Map<number, Set<number>>();
  const nodeToActor = new Map<number, number>();
  for (let i = 0; i < scenario.nodes.length; i += 1) nodeToActor.set(i, 0);

  function actorBodyForNode(nodeIndex: number) {
    const actorIndex = nodeToActor.get(nodeIndex) ?? 0;
    const entry = actorMap.get(actorIndex);
    const bodyHandle = entry?.bodyHandle ?? rootBody.handle;
    const body = world.getRigidBody(bodyHandle);
    return { actorIndex, bodyHandle, body };
  }

  function registerNodeBodyLink(nodeIndex: number, bodyHandle: number | null | undefined) {
    if (bodyHandle == null) return;
    let set = nodesByBodyHandle.get(bodyHandle);
    if (!set) {
      set = new Set<number>();
      nodesByBodyHandle.set(bodyHandle, set);
    }
    set.add(nodeIndex);
  }

  function unregisterNodeBodyLink(nodeIndex: number, bodyHandle: number | null | undefined) {
    if (bodyHandle == null) return;
    const set = nodesByBodyHandle.get(bodyHandle);
    if (!set) return;
    set.delete(nodeIndex);
    if (set.size === 0) nodesByBodyHandle.delete(bodyHandle);
  }

  function percentileFromSorted(sorted: number[], percentile: number): number {
    if (!sorted.length) return 0;
    const clamped = Math.min(1, Math.max(0, percentile));
    const index = Math.min(sorted.length - 1, Math.round(clamped * (sorted.length - 1)));
    return sorted[index];
  }

  function captureBodyColliderStats(): null | { bodyCount: number; min: number; max: number; avg: number; median: number; p95: number } {
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
    return { bodyCount, min: counts[0], max: counts[counts.length - 1], avg, median, p95 };
  }

  function countRigidBodies(): number {
    let count = 0;
    try {
      world.forEachRigidBody(() => { count += 1; });
      if (count > 0) return count;
    } catch {}
    const wbc = world as WorldWithBodyCount;
    return typeof wbc.numRigidBodies === 'function' ? wbc.numRigidBodies() : 0;
  }

  function buildColliderDescForNode(args: { nodeIndex: number; halfX: number; halfY: number; halfZ: number; isSupport: boolean }) {
    const { nodeIndex, halfX, halfY, halfZ, isSupport } = args;
    const builder = (scenario.colliderDescForNode && Array.isArray(scenario.colliderDescForNode)) ? (scenario.colliderDescForNode[nodeIndex] ?? null) : null;
    let desc = typeof builder === 'function' ? builder() : null;
    if (!desc) {
      // If fragmentGeometries are available, use convex hull for non-support fragments
      const fragmentGeometries = (scenario.parameters?.fragmentGeometries ?? []) as Array<{ getAttribute?: (name: string) => { array?: unknown; count?: number } | null } | null>;
      const fragGeom = fragmentGeometries[nodeIndex];
      if (!isSupport && fragGeom) {
        const posAttr = fragGeom.getAttribute?.('position');
        const arr = posAttr?.array;
        if (arr instanceof Float32Array && arr.length >= 9) {
          desc = RAPIER.ColliderDesc.convexHull(arr);
        }
      }
      // Fallback to cuboid if convexHull failed or not available
      if (!desc) {
        const s = isSupport ? 0.999 : 1.0;
        desc = RAPIER.ColliderDesc.cuboid(halfX * s, halfY * s, halfZ * s);
      }
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

  if (isDev) {
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

  const pendingBodiesToCreate: Array<{ actorIndex: number; inheritFromBodyHandle: number; nodes: number[]; isSupport: boolean }> = [];
  const pendingColliderMigrations: Array<{ nodeIndex: number; targetBodyHandle: number }> = [];
  const disabledCollidersToRemove = new Set<number>();
  const bodiesToRemove = new Set<number>();
  const pendingBallSpawns: ProjectileSpawn[] = [];
  const projectiles: ProjectileState[] = [];
  const removedBondIndices = new Set<number>();
  const pendingExternalForces: Array<{ nodeIndex:number; point: Vec3; force: Vec3 }> = [];
  const pendingDamageFractures = new Map<number, Set<number>>();
  const nowSeconds = () => (typeof performance !== 'undefined' ? performance.now() : Date.now()) / 1000;

  // ── Spatial grid for fast splash-radius neighbor lookups ──
  // Avoids O(contacts × chunks) full-scan in processOneFracturePass.
  // Grid cell size = splash radius so each lookup only checks 27 cells.
  const SPLASH_RADIUS = 2.0;
  const SPLASH_CELL = SPLASH_RADIUS; // cell size = splash radius
  const SPLASH_INV_CELL = 1 / SPLASH_CELL;
  // Map from "ix,iy,iz" grid key to array of node indices
  const splashGrid = new Map<string, number[]>();
  let splashGridDirty = true; // rebuild on first use and after splits

  function splashGridKey(x: number, y: number, z: number): string {
    const ix = Math.floor(x * SPLASH_INV_CELL);
    const iy = Math.floor(y * SPLASH_INV_CELL);
    const iz = Math.floor(z * SPLASH_INV_CELL);
    return `${ix},${iy},${iz}`;
  }

  function rebuildSplashGrid() {
    splashGrid.clear();
    for (let ci = 0; ci < chunks.length; ci++) {
      const c = chunks[ci];
      if (!c || !c.active) continue;
      const key = splashGridKey(c.baseLocalOffset.x, c.baseLocalOffset.y, c.baseLocalOffset.z);
      let bucket = splashGrid.get(key);
      if (!bucket) { bucket = []; splashGrid.set(key, bucket); }
      bucket.push(ci);
    }
    splashGridDirty = false;
  }

  // Reusable result array for splash neighbor lookups (avoids allocation per call)
  const splashResult: number[] = [];

  function collectSplashNeighbors(px: number, py: number, pz: number, radius: number, bodyHandle: number): number[] {
    splashResult.length = 0;
    const r2 = radius * radius;
    const minIx = Math.floor((px - radius) * SPLASH_INV_CELL);
    const maxIx = Math.floor((px + radius) * SPLASH_INV_CELL);
    const minIy = Math.floor((py - radius) * SPLASH_INV_CELL);
    const maxIy = Math.floor((py + radius) * SPLASH_INV_CELL);
    const minIz = Math.floor((pz - radius) * SPLASH_INV_CELL);
    const maxIz = Math.floor((pz + radius) * SPLASH_INV_CELL);
    for (let ix = minIx; ix <= maxIx; ix++) {
      for (let iy = minIy; iy <= maxIy; iy++) {
        for (let iz = minIz; iz <= maxIz; iz++) {
          const bucket = splashGrid.get(`${ix},${iy},${iz}`);
          if (!bucket) continue;
          for (let bi = 0; bi < bucket.length; bi++) {
            const ci = bucket[bi];
            const c = chunks[ci];
            if (!c || !c.active || c.bodyHandle !== bodyHandle) continue;
            const dx = c.baseLocalOffset.x - px;
            const dy = c.baseLocalOffset.y - py;
            const dz = c.baseLocalOffset.z - pz;
            if (dx * dx + dy * dy + dz * dz <= r2) splashResult.push(ci);
          }
        }
      }
    }
    return splashResult;
  }

  // ── Snapshot pool for reuse across frames ──
  let snapshotPool: BodySnapshot[] = [];
  let snapshotPoolSize = 0;

  // ── Sleep threshold tracking ──
  let sleepThresholdsApplied = false; // Track if we've already set thresholds on all bodies
  let sleepThresholdsDirty = true; // Mark dirty when new bodies are created or settings change

  // ── Solver idle-skip tracking ──
  // When no external forces are applied and no recent topology changes,
  // the solver would recompute the same result. Skip it to save ~15-20ms.
  let solverHadExternalForces = false;
  // Counts down from 2 when fractures happen — solver runs for 2 frames
  // after last fracture (one to recompute on new topology, one to confirm stable)
  let solverFractureCountdown = 2; // start active so initial gravity is computed

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
    chunks, scenario, materialScale,
    options: damageOptions,
    nodesForBody: (bodyHandle: number) => nodesByBodyHandle.get(bodyHandle)?.values(),
  });

  function rebuildColliderToNodeMap() {
    const t0 = startTiming();
    colliderToNode.clear();
    activeContactColliders.clear();
    for (const chunk of chunks) {
      if (chunk.colliderHandle != null && chunk.active) {
        colliderToNode.set(chunk.colliderHandle, chunk.nodeIndex);
        activeContactColliders.add(chunk.colliderHandle);
      }
    }
    stopTiming(t0, 'rebuildColliderMapMs');
  }

  const MIN_STEP_DT = 1e-4;
  const MAX_STEP_DT = 1 / 30;
  function clampStepDt(value: number): number {
    return Math.min(MAX_STEP_DT, Math.max(MIN_STEP_DT, value));
  }

  function readWorldDt(): number {
    try {
      const w = world as WorldWithOptionalTimestep;
      if (typeof w.timestep === 'number') return w.timestep;
    } catch {}
    try {
      const params = (world as any).integrationParameters;
      const dt = params?.dt;
      if (typeof dt === 'number' && dt > 0) return dt;
    } catch {}
    return 1 / 60;
  }

  function setWorldDtValue(dt: number) {
    try {
      const w = world as WorldWithOptionalTimestep;
      if (typeof w.timestep === 'number') w.timestep = dt;
    } catch {}
    try {
      if ((world as any).integrationParameters) {
        (world as any).integrationParameters.dt = dt;
      }
    } catch {}
  }

  // --- Solver force injection (contact forces → stress solver) ---
  const bufferedExternalContacts: Array<{
    nodeIndex: number;
    otherBodyHandle: number;
    totalForceMagnitude: number;
    maxForceMagnitude: number;
    totalForceWorld?: Vec3;
  }> = [];
  const bufferedInternalContacts: Array<{
    nodeA: number;
    nodeB: number;
    bodyHandle: number;
    totalForceMagnitude: number;
    maxForceMagnitude: number;
  }> = [];

  // --- Buffered contacts for damage rollback replay ---
  const contactReplayBuffer = new ContactBuffer();
  const speedScalingOpts: SpeedScalingOptions = {
    speedMinExternal: damageOptions.speedMinExternal,
    speedMinInternal: damageOptions.speedMinInternal,
    speedMax: damageOptions.speedMax,
    speedExponent: damageOptions.speedExponent,
    slowSpeedFactor: damageOptions.slowSpeedFactor,
    fastSpeedFactor: damageOptions.fastSpeedFactor,
  };

  function getChunkWorldCenter(nodeIndex: number): Vec3 | null {
    const chunk = chunks[nodeIndex];
    if (!chunk) return null;
    const { body } = actorBodyForNode(nodeIndex);
    if (!body) return null;
    return chunkWorldCenterHelper(body, chunk.baseLocalOffset);
  }

  function getWorldPointToActorLocal(nodeIndex: number, worldPoint: Vec3): Vec3 | null {
    try {
      const { body } = actorBodyForNode(nodeIndex);
      if (!body) return null;
      return worldPointToBodyLocal(body, worldPoint);
    } catch {
      return null;
    }
  }

  function drainContactForces() {
    const t0 = startTiming();
    bufferedExternalContacts.length = 0;
    bufferedInternalContacts.length = 0;
    contactReplayBuffer.clear();

    const dt = lastStepDt;

    eventQueue.drainContactForceEvents((ev) => {
      const h1 = ev.collider1();
      const h2 = ev.collider2();
      const node1 = colliderToNode.get(h1);
      const node2 = colliderToNode.get(h2);
      const totalForce = ev.totalForceMagnitude();
      const maxForce = ev.maxForceMagnitude();
      // Capture force vector for stress solver injection
      const forceVec: Vec3 | undefined = typeof (ev as any).totalForce === 'function'
        ? (ev as any).totalForce() as Vec3
        : undefined;
      // Extract world contact points (when available from Rapier)
      const wp = typeof (ev as any).worldContactPoint === 'function'
        ? (ev as any).worldContactPoint() as Vec3 | undefined
        : undefined;
      const wp2 = typeof (ev as any).worldContactPoint2 === 'function'
        ? (ev as any).worldContactPoint2() as Vec3 | undefined
        : undefined;
      const p1 = wp ?? wp2 ?? fallbackContactPoint(world, h1);
      const p2 = wp2 ?? wp ?? fallbackContactPoint(world, h2);

      const isInternal = (node1 != null && node2 != null);
      const pForNode1 = node1 != null ? (wp ?? wp2 ?? getChunkWorldCenter(node1) ?? p1) : undefined;
      const pForNode2 = node2 != null ? (wp2 ?? wp ?? getChunkWorldCenter(node2) ?? p2) : undefined;
      const relAnchor = pForNode1 ?? pForNode2 ?? p1 ?? p2;

      // Speed-scaled effective magnitude
      const relSpeed = computeRelativeSpeed(world, h1, h2, relAnchor);
      const speedFactor = computeSpeedFactor(relSpeed, isInternal, speedScalingOpts);
      let effMag = (totalForce ?? 0) * speedFactor;

      // Projectile momentum boost
      try {
        const b1 = getBodyForColliderHandle(world, h1);
        const b2 = getBodyForColliderHandle(world, h2);

        // Track ground collisions for optimization modes
        if (b1 && b2) {
          const isB1Ground = b1.handle === groundBody.handle;
          const isB2Ground = b2.handle === groundBody.handle;
          if (isB1Ground && !isB2Ground && b2.handle !== rootBody.handle) {
            const wasNew = !bodiesCollidedWithGround.has(b2.handle);
            bodiesCollidedWithGround.add(b2.handle);
            if (wasNew && smallBodyDampingSettings.mode === 'afterGroundCollision') {
              applySmallBodyDampingToBody(b2.handle);
            }
          } else if (isB2Ground && !isB1Ground && b1.handle !== rootBody.handle) {
            const wasNew = !bodiesCollidedWithGround.has(b1.handle);
            bodiesCollidedWithGround.add(b1.handle);
            if (wasNew && smallBodyDampingSettings.mode === 'afterGroundCollision') {
              applySmallBodyDampingToBody(b1.handle);
            }
          }
        }

        if (node1 != null || node2 != null) {
          effMag = applyProjectileMomentumBoost(b1, b2, relSpeed, dt, effMag);
        }
      } catch { /* defensive: don't let boost logic crash contact drain */ }

      // Compute body-local contact points for splash AOE damage
      const local1 = (node1 != null && pForNode1) ? getWorldPointToActorLocal(node1, pForNode1) : null;
      const local2 = (node2 != null && pForNode2) ? getWorldPointToActorLocal(node2, pForNode2) : null;

      // --- Buffer contacts for stress solver injection (unchanged logic) ---
      if (node1 != null && node2 != null) {
        const chunk1 = chunks[node1];
        const chunk2 = chunks[node2];
        if (chunk1 && chunk2 && chunk1.bodyHandle === chunk2.bodyHandle) {
          bufferedInternalContacts.push({
            nodeA: node1,
            nodeB: node2,
            bodyHandle: chunk1.bodyHandle!,
            totalForceMagnitude: totalForce,
            maxForceMagnitude: maxForce,
          });
          // Buffer for damage replay with speed-scaled magnitude
          contactReplayBuffer.recordInternal({
            nodeA: node1, nodeB: node2, effMag, dt,
            localPointA: local1 ?? undefined,
            localPointB: local2 ?? undefined,
          });
        } else {
          if (chunk1) {
            bufferedExternalContacts.push({
              nodeIndex: node1,
              otherBodyHandle: chunk2?.bodyHandle ?? -1,
              totalForceMagnitude: totalForce,
              maxForceMagnitude: maxForce,
              totalForceWorld: forceVec,
            });
            contactReplayBuffer.recordExternal({
              nodeIndex: node1, effMag, dt,
              localPoint: local1 ?? undefined,
            });
          }
          if (chunk2) {
            bufferedExternalContacts.push({
              nodeIndex: node2,
              otherBodyHandle: chunk1?.bodyHandle ?? -1,
              totalForceMagnitude: totalForce,
              maxForceMagnitude: maxForce,
              totalForceWorld: forceVec ? { x: -forceVec.x, y: -forceVec.y, z: -forceVec.z } : undefined,
            });
            contactReplayBuffer.recordExternal({
              nodeIndex: node2, effMag, dt,
              localPoint: local2 ?? undefined,
            });
          }
        }
      } else if (node1 != null) {
        const chunk = chunks[node1];
        if (chunk) {
          bufferedExternalContacts.push({
            nodeIndex: node1,
            otherBodyHandle: -1,
            totalForceMagnitude: totalForce,
            maxForceMagnitude: maxForce,
            totalForceWorld: forceVec,
          });
          contactReplayBuffer.recordExternal({
            nodeIndex: node1, effMag, dt,
            localPoint: local1 ?? undefined,
          });
          if (chunk.bodyHandle != null) bodiesCollidedWithGround.add(chunk.bodyHandle);
        }
      } else if (node2 != null) {
        const chunk = chunks[node2];
        if (chunk) {
          bufferedExternalContacts.push({
            nodeIndex: node2,
            otherBodyHandle: -1,
            totalForceMagnitude: totalForce,
            maxForceMagnitude: maxForce,
            totalForceWorld: forceVec ? { x: -forceVec.x, y: -forceVec.y, z: -forceVec.z } : undefined,
          });
          contactReplayBuffer.recordExternal({
            nodeIndex: node2, effMag, dt,
            localPoint: local2 ?? undefined,
          });
          if (chunk.bodyHandle != null) bodiesCollidedWithGround.add(chunk.bodyHandle);
        }
      }
    });
    stopTiming(t0, 'contactDrainMs');
  }

  function applyExternalForcesFromBuffer() {
    const t0 = startTiming();
    for (const pf of pendingExternalForces) {
      const chunk = chunks[pf.nodeIndex];
      if (!chunk || !chunk.active || chunk.bodyHandle == null) continue;
      const body = world.getRigidBody(chunk.bodyHandle);
      if (!body || body.isFixed()) continue;
      body.addForceAtPoint(
        { x: pf.force.x, y: pf.force.y, z: pf.force.z },
        { x: pf.point.x, y: pf.point.y, z: pf.point.z },
        true,
      );
    }
    if (activeProfilerSample) {
      activeProfilerSample.pendingExternalForces = pendingExternalForces.length;
    }
    pendingExternalForces.length = 0;
    stopTiming(t0, 'externalForceMs');
  }

  type BodySnapshot = {
    handle: number;
    translation: { x: number; y: number; z: number };
    rotation: { x: number; y: number; z: number; w: number };
    linvel: { x: number; y: number; z: number };
    angvel: { x: number; y: number; z: number };
  };
  let savedBodySnapshots: BodySnapshot[] | null = null;
  let savedWorldSnapshot: Uint8Array | null = null;

  function captureWorldSnapshot() {
    const t0 = startTiming();
    if (snapshotMode === 'world') {
      savedWorldSnapshot = world.takeSnapshot();
      savedBodySnapshots = null;
      if (activeProfilerSample) {
        activeProfilerSample.snapshotBytes = savedWorldSnapshot?.byteLength ?? 0;
      }
    } else {
      savedWorldSnapshot = null;
      // Reuse snapshot pool to avoid GC pressure — no allocations in steady state
      snapshotPoolSize = 0;
      world.forEachRigidBody((body) => {
        if (body.isFixed()) return;
        const t = body.translation();
        const r = body.rotation();
        const lv = body.linvel();
        const av = body.angvel();
        if (snapshotPoolSize < snapshotPool.length) {
          // Reuse existing object
          const snap = snapshotPool[snapshotPoolSize];
          snap.handle = body.handle;
          snap.translation.x = t.x; snap.translation.y = t.y; snap.translation.z = t.z;
          snap.rotation.x = r.x; snap.rotation.y = r.y; snap.rotation.z = r.z; snap.rotation.w = r.w;
          snap.linvel.x = lv.x; snap.linvel.y = lv.y; snap.linvel.z = lv.z;
          snap.angvel.x = av.x; snap.angvel.y = av.y; snap.angvel.z = av.z;
        } else {
          // Grow pool
          snapshotPool.push({
            handle: body.handle,
            translation: { x: t.x, y: t.y, z: t.z },
            rotation: { x: r.x, y: r.y, z: r.z, w: r.w },
            linvel: { x: lv.x, y: lv.y, z: lv.z },
            angvel: { x: av.x, y: av.y, z: av.z },
          });
        }
        snapshotPoolSize++;
      });
      savedBodySnapshots = snapshotPool;
      if (activeProfilerSample) {
        activeProfilerSample.snapshotBytes = snapshotPoolSize * 13 * 8;
      }
    }
    stopTiming(t0, 'snapshotCaptureMs');
  }

  function restoreWorldSnapshot() {
    const t0 = startTiming();
    if (snapshotMode === 'world' && savedWorldSnapshot) {
      const oldWorld = world;
      const restored = RAPIER.World.restoreSnapshot(savedWorldSnapshot);
      if (restored) {
        world = restored;
        onWorldReplaced?.(world);
      } else {
        console.warn('[Core] world restore failed; fallback to body restore');
      }
    } else if (savedBodySnapshots) {
      const count = snapshotPoolSize;
      for (let si = 0; si < count; si++) {
        const snap = savedBodySnapshots[si];
        const body = world.getRigidBody(snap.handle);
        if (!body) continue;
        body.setTranslation(snap.translation, true);
        body.setRotation(snap.rotation, true);
        body.setLinvel(snap.linvel, true);
        body.setAngvel(snap.angvel, true);
      }
    }
    stopTiming(t0, 'snapshotRestoreMs');
  }

  function processOneFracturePass(passIndex: number, reasons: string[]): boolean {
    const passT0 = startTiming();

    const solverT0 = startTiming();
    if (solverGravityEnabled) {
      const solverActors = (solver as unknown as SolverActorsApi).actors?.() ?? [];
      for (const actor of solverActors) {
        const entry = actorMap.get(actor.actorIndex);
        if (!entry) {
          solver.addActorGravity(actor.actorIndex, { x: 0, y: gravity, z: 0 });
          continue;
        }
        const body = world.getRigidBody(entry.bodyHandle);
        if (!body) {
          solver.addActorGravity(actor.actorIndex, { x: 0, y: gravity, z: 0 });
          continue;
        }
        const rot = body.rotation();
        const qx = rot.x, qy = rot.y, qz = rot.z, qw = rot.w;
        const gx = 0, gy = gravity, gz = 0;
        const ix = qw * qw * gx + 2 * qy * qw * gz - 2 * qz * qw * gy + qx * qx * gx + 2 * qy * qx * gy + 2 * qz * qx * gz - qz * qz * gx - qy * qy * gx;
        const iy = 2 * qx * qy * gx + qy * qy * gy + 2 * qz * qy * gz + 2 * qw * qz * gx - qz * qz * gy + qw * qw * gy - 2 * qx * qw * gz - qx * qx * gy;
        const iz = 2 * qx * qz * gx + 2 * qy * qz * gy + qz * qz * gz - 2 * qw * qy * gx - qy * qy * gz + 2 * qw * qx * gy - qx * qx * gz + qw * qw * gz;
        solver.addActorGravity(actor.actorIndex, { x: ix, y: iy, z: iz });
      }
    }

    // Inject external contact forces (e.g. projectile impacts) into the stress solver.
    // Converts world-space contact forces into body-local space and applies them
    // to the impacted node plus nearby nodes (splash radius) so that bond stress
    // reflects collision impacts, not just gravity.
    for (const contact of bufferedExternalContacts) {
      if (!contact.totalForceWorld) continue;
      const hitChunk = chunks[contact.nodeIndex];
      if (!hitChunk || !hitChunk.active || hitChunk.bodyHandle == null) continue;
      const body = world.getRigidBody(hitChunk.bodyHandle);
      if (!body) continue;
      // Rotate force from world space to body-local space
      const rot = body.rotation();
      const qx = rot.x, qy = rot.y, qz = rot.z, qw = rot.w;
      const fx = contact.totalForceWorld.x, fy = contact.totalForceWorld.y, fz = contact.totalForceWorld.z;
      // Inverse quaternion rotation (conjugate)
      const lx = qw * qw * fx - 2 * qy * qw * fz + 2 * qz * qw * fy + qx * qx * fx - 2 * qy * qx * fy - 2 * qz * qx * fz - qz * qz * fx - qy * qy * fx
        + 2 * qx * qy * fy + 2 * qx * qz * fz;
      const ly = -2 * qw * qz * fx + qw * qw * fy + 2 * qw * qx * fz + 2 * qx * qy * fx + qy * qy * fy - 2 * qz * qy * fz - qx * qx * fy - qz * qz * fy
        + 2 * qy * qz * fz;
      const lz = 2 * qw * qy * fx - 2 * qw * qx * fy + qw * qw * fz + 2 * qx * qz * fx + 2 * qy * qz * fy + qz * qz * fz - qx * qx * fz - qy * qy * fz;
      const scaledForce = { x: lx * contactForceScale, y: ly * contactForceScale, z: lz * contactForceScale };

      // Apply to hit node at full strength
      solver.addForce(contact.nodeIndex, hitChunk.baseLocalOffset, scaledForce);

      // Splash: apply attenuated force to neighboring nodes on the same body
      // Uses spatial grid for O(1) average lookups instead of O(n) full scan
      const hitPos = hitChunk.baseLocalOffset;
      const splashR = SPLASH_RADIUS;
      if (splashGridDirty) rebuildSplashGrid();
      const neighbors = collectSplashNeighbors(hitPos.x, hitPos.y, hitPos.z, splashR, hitChunk.bodyHandle!);
      for (let ni = 0; ni < neighbors.length; ni++) {
        const ci = neighbors[ni];
        if (ci === contact.nodeIndex) continue;
        const c = chunks[ci];
        const dx = c.baseLocalOffset.x - hitPos.x;
        const dy = c.baseLocalOffset.y - hitPos.y;
        const dz = c.baseLocalOffset.z - hitPos.z;
        const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (dist > splashR) continue;
        const falloff = (1 - dist / splashR);
        const f2 = falloff * falloff; // quadratic falloff
        if (f2 <= 0) continue;
        solver.addForce(ci, c.baseLocalOffset, {
          x: scaledForce.x * f2, y: scaledForce.y * f2, z: scaledForce.z * f2,
        });
      }
    }

    // Skip solver when idle: no external contacts, no recent fractures/topology changes,
    // solver has converged (no residual error from prior frames), and not a resimulation pass.
    // Without the convergence check, the solver might skip frames where it hasn't fully
    // resolved stress in large structures (CGNR may need multiple frames to converge).
    const hasExternalForces = bufferedExternalContacts.length > 0 || pendingExternalForces.length > 0;
    const solverConverged = typeof solver.converged === 'function' ? solver.converged() : false;
    const shouldSkipSolver = fracturePolicySettings.idleSkip && !hasExternalForces && solverFractureCountdown <= 0 && solverConverged && passIndex === 0 && safeFrames > 2;
    if (!shouldSkipSolver) {
      solver.update();
    }
    solverHadExternalForces = hasExternalForces;
    if (solverFractureCountdown > 0) solverFractureCountdown--;
    stopTiming(solverT0, 'solverUpdateMs');

    const fractureT0 = startTiming();
    let hadFracture = false;
    if (!shouldSkipSolver && solver.overstressedBondCount() > 0) {
      let perActor = profiledGenerateFractureCommands();

      // ── Fracture policy: progressive fracture budget ──
      // Sort by health (highest damage = most overstressed) and take top N.
      // Remaining bonds stay in the solver graph and are re-evaluated next frame.
      // This models realistic fracture propagation speed.
      const maxFrac = fracturePolicySettings.maxFracturesPerFrame;
      if (maxFrac > 0) {
        for (const cmd of perActor) {
          if (cmd.fractures.length > maxFrac) {
            cmd.fractures.sort((a: any, b: any) => b.health - a.health);
            cmd.fractures.length = maxFrac;
          }
        }
      }

      // ── Fracture policy: dynamic body cap ──
      // Suppress all fractures when at the body limit. Bonds stay intact
      // until bodies are freed (via debris cleanup), then fractures resume.
      const maxBodies = fracturePolicySettings.maxDynamicBodies;
      if (maxBodies > 0 && getRigidBodyCount() >= maxBodies) {
        perActor = [];
      }

      const splitEvents = profiledApplyFractureCommands(perActor);
      processSplitEvents(splitEvents);
      hadFracture = splitEvents.length > 0;
      if (hadFracture) solverFractureCountdown = 2; // run solver 2 more frames to stabilize
    }
    stopTiming(fractureT0, 'fractureMs');

    if (activeProfilerSample) {
      const passMs = passT0 != null ? Math.max(0, perfNow() - passT0) : 0;
      activeProfilerSample.passes.push({
        index: passIndex,
        solverMs: 0,
        fractureMs: 0,
        bodyCreateMs: 0,
        totalMs: passMs,
        reasons: [...reasons],
      });
    }

    return hadFracture;
  }

  function flushPendingBodies() {
    const t0 = startTiming();

    // ── Fracture policy: body creation budgets ──
    // Two caps can limit how many bodies we actually create this
    // frame: `maxNewBodiesPerFrame` (per-frame budget) and
    // `maxDynamicBodies` (global cap on total bodies in the world).
    // When either binds, prioritize the largest children (most
    // visually important) and defer the rest to a later frame —
    // they stay in `pendingBodiesToCreate` and are reconsidered next
    // frame after debris cleanup may have freed room.
    //
    // The global cap has to be enforced here at body-creation time,
    // *not* upstream at fracture-command time: a single fracture
    // step can generate dozens of pending bodies in one batch (e.g.
    // a tower-collapse event), and the upstream "if count >= cap,
    // drop commands" gate only fires *before* those bodies exist,
    // so it can't catch "one frame of fracturing that blows past
    // the cap". Here we break the loop mid-drain as soon as the
    // running count hits the cap.
    const maxPerFrame = fracturePolicySettings.maxNewBodiesPerFrame;
    const maxTotal = fracturePolicySettings.maxDynamicBodies;
    let runningTotal = maxTotal > 0 ? getRigidBodyCount() : 0;

    const perFrameWouldBind =
      maxPerFrame > 0 && pendingBodiesToCreate.length > maxPerFrame;
    const totalWouldBind =
      maxTotal > 0 && runningTotal + pendingBodiesToCreate.length > maxTotal;
    if (perFrameWouldBind || totalWouldBind) {
      pendingBodiesToCreate.sort((a, b) => b.nodes.length - a.nodes.length);
    }
    let bodiesCreated = 0;

    for (const pending of pendingBodiesToCreate) {
      // Enforce per-frame new-body budget — keep remaining entries for next frame
      if (maxPerFrame > 0 && bodiesCreated >= maxPerFrame) break;
      // Enforce global dynamic-body cap — keep remaining entries for next frame
      if (maxTotal > 0 && runningTotal >= maxTotal) break;
      const { actorIndex, inheritFromBodyHandle, nodes: nodeList, isSupport } = pending;

      const parentBody = world.getRigidBody(inheritFromBodyHandle);
      const parentPos = parentBody?.translation() ?? { x: 0, y: 0, z: 0 };
      const parentRot = parentBody?.rotation() ?? { x: 0, y: 0, z: 0, w: 1 };
      const parentLinvel = parentBody?.linvel() ?? { x: 0, y: 0, z: 0 };
      const parentAngvel = parentBody?.angvel() ?? { x: 0, y: 0, z: 0 };
      const parentLinDamp = parentBody && typeof parentBody.linearDamping === 'function' ? parentBody.linearDamping() : undefined;
      const parentAngDamp = parentBody && typeof parentBody.angularDamping === 'function' ? parentBody.angularDamping() : undefined;

      const desc = isSupport
        ? RAPIER.RigidBodyDesc.fixed().setTranslation(parentPos.x, parentPos.y, parentPos.z).setRotation(parentRot)
        : RAPIER.RigidBodyDesc.dynamic().setTranslation(parentPos.x, parentPos.y, parentPos.z).setRotation(parentRot)
            .setLinvel(parentLinvel.x, parentLinvel.y, parentLinvel.z).setAngvel(parentAngvel);
      if (typeof parentLinDamp === 'number') desc.setLinearDamping(parentLinDamp);
      if (typeof parentAngDamp === 'number') desc.setAngularDamping(parentAngDamp);

      if (!isSupport && fractureBodyCcdEnabled) {
        try { (desc as MaybeCcdBodyDesc).setCcdEnabled?.(true); } catch {}
      }

      const newBody = world.createRigidBody(desc);
      const bodyHandle = newBody.handle;

      // Apply small body damping immediately for 'always' mode
      if (!isSupport) {
        const isSmall = nodeList.length <= smallBodyDampingSettings.colliderCountThreshold;
        if (isSmall && smallBodyDampingSettings.mode === 'always') {
          try {
            const curLin = typeof newBody.linearDamping === 'function' ? newBody.linearDamping() : 0;
            const curAng = typeof newBody.angularDamping === 'function' ? newBody.angularDamping() : 0;
            newBody.setLinearDamping(Math.max(curLin, smallBodyDampingSettings.minLinearDamping));
            newBody.setAngularDamping(Math.max(curAng, smallBodyDampingSettings.minAngularDamping));
          } catch {}
        }
        if (isSmall) smallBodiesPendingDamping.add(bodyHandle);

        if (nodeList.length <= debrisCleanupSettings.maxCollidersForDebris) {
          debrisCreationTimes.set(bodyHandle, Date.now());
        }
      }

      // Apply collision groups based on current debris mode
      applyCollisionGroupsForBodyImpl(newBody, getCollisionGroupContext());

      actorMap.set(actorIndex, { bodyHandle });
      sleepThresholdPending.add(bodyHandle);
      splashGridDirty = true; // body topology changed

      for (const ni of nodeList) {
        const oldBody = chunks[ni]?.bodyHandle;
        unregisterNodeBodyLink(ni, oldBody);
        nodeToActor.set(ni, actorIndex);
        chunks[ni].bodyHandle = bodyHandle;
        registerNodeBodyLink(ni, bodyHandle);
        pendingColliderMigrations.push({ nodeIndex: ni, targetBodyHandle: bodyHandle });
      }
      bodiesCreated++;
      runningTotal++;
    }
    // Remove processed entries, keep deferred ones for next frame.
    // Either cap (per-frame or global) can cause an early break, so
    // the check is "did we fully drain the queue?", not "was a
    // specific cap set?".
    if (bodiesCreated < pendingBodiesToCreate.length) {
      pendingBodiesToCreate.splice(0, bodiesCreated);
    } else {
      pendingBodiesToCreate.length = 0;
    }
    stopTiming(t0, 'bodyCreateMs');
  }

  function flushColliderMigrations() {
    const t0 = startTiming();

    // ── Fracture policy: collider migration budget ──
    const maxMigrations = fracturePolicySettings.maxColliderMigrationsPerFrame;
    let migrationsProcessed = 0;
    let writeIdx = 0;
    // Track source bodies that lost colliders — check if they became empty
    const sourceBodiesAffected = new Set<number>();

    for (let mi = 0; mi < pendingColliderMigrations.length; mi++) {
      // Enforce migration budget — keep remaining for next frame
      if (maxMigrations > 0 && migrationsProcessed >= maxMigrations) {
        pendingColliderMigrations[writeIdx++] = pendingColliderMigrations[mi];
        continue;
      }

      const migration = pendingColliderMigrations[mi];
      const chunk = chunks[migration.nodeIndex];
      if (!chunk || chunk.colliderHandle == null) continue;
      if (!chunk.active) continue;

      const oldHandle = chunk.colliderHandle;
      const oldCollider = world.getCollider(oldHandle);
      if (!oldCollider) continue;

      const targetBody = world.getRigidBody(migration.targetBodyHandle);
      if (!targetBody) {
        // Target body doesn't exist yet (deferred) — keep for next frame
        pendingColliderMigrations[writeIdx++] = migration;
        continue;
      }

      // Track the source body that's losing this collider
      const sourceBodyHandle = chunk.bodyHandle;
      if (sourceBodyHandle != null && sourceBodyHandle !== migration.targetBodyHandle) {
        sourceBodiesAffected.add(sourceBodyHandle);
      }

      colliderToNode.delete(oldHandle);
      activeContactColliders.delete(oldHandle);
      world.removeCollider(oldCollider, false);

      // Skip creating colliders for destroyed chunks (matches vibe-city)
      if (chunk.destroyed) { migrationsProcessed++; continue; }

      const size = nodeSize(chunk.nodeIndex, scenario);
      const halfX = Math.max(0.05, size.x * 0.5);
      const halfY = Math.max(0.05, size.y * 0.5);
      const halfZ = Math.max(0.05, size.z * 0.5);
      const isSupport = chunk.isSupport;

      // Support nodes use baseWorldPosition (absolute) when available;
      // dynamic nodes use baseLocalOffset (relative to body).
      const useWorldPos = isSupport && chunk.baseWorldPosition;
      const tx = useWorldPos ? chunk.baseWorldPosition!.x : chunk.baseLocalOffset.x;
      const ty = useWorldPos ? chunk.baseWorldPosition!.y : chunk.baseLocalOffset.y;
      const tz = useWorldPos ? chunk.baseWorldPosition!.z : chunk.baseLocalOffset.z;

      const desc = buildColliderDescForNode({ nodeIndex: chunk.nodeIndex, halfX, halfY, halfZ, isSupport })
        .setMass(scenario.nodes[chunk.nodeIndex]?.mass ?? 1)
        .setTranslation(tx, ty, tz)
        .setFriction(friction)
        .setRestitution(restitution)
        .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
        .setContactForceEventThreshold(0.0);

      const newCol = world.createCollider(desc, targetBody);
      chunk.colliderHandle = newCol.handle;
      chunk.localOffset = { x: tx, y: ty, z: tz };
      chunk.bodyHandle = migration.targetBodyHandle;
      colliderToNode.set(newCol.handle, chunk.nodeIndex);
      activeContactColliders.add(newCol.handle);
      migrationsProcessed++;

      // Reapply collision groups after collider migration
      applyCollisionGroupsForBodyImpl(targetBody, getCollisionGroupContext());
    }
    // Keep deferred migrations, discard processed ones
    pendingColliderMigrations.length = writeIdx;

    // Clean up source bodies that lost all colliders during migration
    for (const bh of sourceBodiesAffected) {
      if (bh === rootBody.handle || bh === groundBody.handle) continue;
      const body = world.getRigidBody(bh);
      if (!body) continue;
      const rb = body as RigidBodyWithColliderCount;
      const count = typeof rb.numColliders === 'function' ? rb.numColliders() : -1;
      if (count === 0) {
        bodiesToRemove.add(bh);
      }
    }

    stopTiming(t0, 'colliderRebuildMs');
  }

  function cleanupDisabledColliders() {
    const t0 = startTiming();
    for (const ch of disabledCollidersToRemove) {
      const col = world.getCollider(ch);
      if (col) {
        colliderToNode.delete(ch);
        activeContactColliders.delete(ch);
        world.removeCollider(col, false);
      }
    }
    disabledCollidersToRemove.clear();
    for (const bh of bodiesToRemove) {
      const body = world.getRigidBody(bh);
      if (body) {
        world.removeRigidBody(body);
      }
      nodesByBodyHandle.delete(bh);
      debrisCreationTimes.delete(bh);
    }
    bodiesToRemove.clear();
    stopTiming(t0, 'cleanupDisabledMs');
  }

  function processDebrisCleanup() {
    if (debrisCleanupSettings.mode === 'off') return;
    const now = Date.now();
    const ttl = debrisCleanupSettings.debrisTtlMs;
    const toRemove: number[] = [];
    for (const [bodyHandle, createdAt] of debrisCreationTimes) {
      if (!shouldApplyOptimization(debrisCleanupSettings.mode, bodyHandle)) continue;
      if (now - createdAt > ttl) {
        toRemove.push(bodyHandle);
      }
    }
    for (const bodyHandle of toRemove) {
      const nodesOnBody = nodesByBodyHandle.get(bodyHandle);
      if (nodesOnBody) {
        // Array.from required because handleNodeDestroyed calls
        // unregisterNodeBodyLink which deletes from the Set
        for (const ni of Array.from(nodesOnBody)) {
          handleNodeDestroyed(ni, 'manual');
        }
      }
      bodiesToRemove.add(bodyHandle);
      debrisCreationTimes.delete(bodyHandle);
    }
  }

  function processSmallBodyDamping() {
    if (smallBodyDampingSettings.mode === 'off') return;
    for (const bodyHandle of smallBodiesPendingDamping) {
      if (shouldApplyOptimization(smallBodyDampingSettings.mode, bodyHandle)) {
        applySmallBodyDampingToBody(bodyHandle);
      }
    }
  }

  // Set of body handles that need sleep threshold applied (newly created bodies)
  const sleepThresholdPending = new Set<number>();

  function processSleepThresholds() {
    if (sleepSettings.mode === 'off') return;

    // If settings changed (dirty), apply to all bodies once
    if (sleepThresholdsDirty) {
      sleepThresholdsDirty = false;
      const threshold = Math.max(sleepSettings.linear, sleepSettings.angular);
      world.forEachRigidBody((body) => {
        if (body.isFixed()) return;
        if (!shouldApplyOptimization(sleepSettings.mode, body.handle)) return;
        try {
          if (typeof (body as any).setSleepThreshold === 'function') {
            (body as any).setSleepThreshold(threshold);
          }
        } catch {}
      });
      sleepThresholdPending.clear();
      return;
    }

    // Otherwise, only apply to newly created bodies
    if (sleepThresholdPending.size === 0) return;
    const threshold = Math.max(sleepSettings.linear, sleepSettings.angular);
    for (const bh of sleepThresholdPending) {
      if (!shouldApplyOptimization(sleepSettings.mode, bh)) continue;
      const body = world.getRigidBody(bh);
      if (!body || body.isFixed()) continue;
      try {
        if (typeof (body as any).setSleepThreshold === 'function') {
          (body as any).setSleepThreshold(threshold);
        }
      } catch {}
    }
    sleepThresholdPending.clear();
  }

  function spawnPendingProjectiles() {
    const t0 = startTiming();
    for (const spawn of pendingBallSpawns) {
      const r = spawn.radius ?? 0.15;
      const bodyDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(spawn.position.x, spawn.position.y, spawn.position.z)
        .setLinvel(spawn.velocity.x, spawn.velocity.y, spawn.velocity.z)
        .setUserData({ projectile: true });
      if (projectileCcdEnabled) {
        const ccdDesc = bodyDesc as MaybeCcdBodyDesc;
        if (typeof ccdDesc.setCcdEnabled === 'function') {
          ccdDesc.setCcdEnabled(true);
        }
      }
      const body = world.createRigidBody(bodyDesc);
      const colDesc = RAPIER.ColliderDesc.ball(r)
        .setMass(spawn.mass ?? 2)
        .setFriction(friction)
        .setRestitution(0.3)
        .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
        .setContactForceEventThreshold(0.0);
      world.createCollider(colDesc, body);
      projectiles.push({
        bodyHandle: body.handle,
        radius: r,
        createdAt: nowSeconds(),
        ttl: spawn.ttl ?? 5,
      });
    }
    pendingBallSpawns.length = 0;
    stopTiming(t0, 'spawnMs');
  }

  function cleanupExpiredProjectiles() {
    const t0 = startTiming();
    const now = nowSeconds();
    let i = 0;
    while (i < projectiles.length) {
      const p = projectiles[i];
      if (now - p.createdAt > p.ttl) {
        const body = world.getRigidBody(p.bodyHandle);
        if (body) world.removeRigidBody(body);
        projectiles.splice(i, 1);
      } else {
        i++;
      }
    }
    stopTiming(t0, 'projectileCleanupMs');
  }

  /**
   * Process a list of split events from the fracture pipeline.
   * Shared between stress-driven and damage-driven fractures.
   */
  function processSplitEvents(
    splitEvents: Array<{ parentActorIndex: number; children: Array<{ actorIndex: number; nodes: number[] }> }>,
  ) {
    for (const split of splitEvents) {
      const parentActorIndex = split.parentActorIndex;
      const parentEntry = actorMap.get(parentActorIndex);
      const parentBodyHandle = parentEntry?.bodyHandle ?? rootBody.handle;
      const plannerChildren: PlannerChild[] = [];

      for (const child of split.children) {
        const childNodes: number[] = child.nodes ?? [];
        if (childNodes.length === 0) continue;
        const isChildSupport = childNodes.some((ni: number) => {
          const ch = chunks[ni];
          if (ch?.isSupport) return true;
          const mass = scenario.nodes[ni]?.mass ?? 0;
          return !(mass > 0);
        });

        if (skipSingleBodiesEnabled && childNodes.length <= 1 && !isChildSupport) {
          const ni = childNodes[0];
          try { handleNodeDestroyed(ni, 'manual'); } catch {}
          continue;
        }

        // Fracture policy: minChildNodeCount — destroy children smaller than threshold
        if (fracturePolicySettings.minChildNodeCount > 1
          && childNodes.length < fracturePolicySettings.minChildNodeCount && !isChildSupport) {
          for (const ni of childNodes) {
            try { handleNodeDestroyed(ni, 'manual'); } catch {}
          }
          continue;
        }

        if (activeProfilerSample) {
          if (!(activeProfilerSample as any).splitChildCounts) {
            (activeProfilerSample as any).splitChildCounts = [];
          }
          (activeProfilerSample as any).splitChildCounts.push(childNodes.length);
        }

        for (const n of childNodes) nodeToActor.set(n, child.actorIndex);
        plannerChildren.push({
          index: plannerChildren.length,
          actorIndex: child.actorIndex,
          nodes: childNodes,
          isSupport: isChildSupport,
        });
      }

      if (plannerChildren.length > 0) {
        const parentNodes = nodesByBodyHandle.get(parentBodyHandle) ?? new Set<number>();
        const parentRigidBody = world.getRigidBody(parentBodyHandle);
        const parentIsFixed = !!parentRigidBody?.isFixed?.();
        let plannerDuration = 0;
        const migration = planSplitMigration(
          [{ handle: parentBodyHandle, nodeIndices: parentNodes, isFixed: parentIsFixed }],
          plannerChildren,
          { onDuration: (ms: number) => { plannerDuration += ms; } },
        );
        if (activeProfilerSample) {
          (activeProfilerSample as any).splitPlannerMs =
            ((activeProfilerSample as any).splitPlannerMs ?? 0) + plannerDuration;
        }

        for (const reuse of migration.reuse) {
          const entry = plannerChildren[reuse.childIndex];
          if (!entry) continue;
          actorMap.set(entry.actorIndex, { bodyHandle: reuse.bodyHandle });
          const reusedBody = world.getRigidBody(reuse.bodyHandle);
          if (reusedBody) {
            if (entry.isSupport) reusedBody.setBodyType(RAPIER.RigidBodyType.Fixed, true);
            else reusedBody.setBodyType(RAPIER.RigidBodyType.Dynamic, true);
            // Reapply collision groups — body type and collider count may have changed
            applyCollisionGroupsForBodyImpl(reusedBody, getCollisionGroupContext());
          }
        }

        for (const create of migration.create) {
          const entry = plannerChildren[create.childIndex];
          if (!entry) continue;
          pendingBodiesToCreate.push({
            actorIndex: entry.actorIndex,
            inheritFromBodyHandle: parentBodyHandle,
            nodes: entry.nodes,
            isSupport: entry.isSupport,
          });
          actorMap.set(entry.actorIndex, { bodyHandle: parentBodyHandle });
        }
      }
    }
  }

  /**
   * Process pending damage-driven fractures through the full split pipeline.
   *
   * Groups damaged bonds by actor, calls applyFractureCommands to generate
   * split events, then processes splits with body creation + collider migration.
   * This ensures damage-caused fractures properly separate physics bodies.
   */
  function flushPendingDamageFractures() {
    if (pendingDamageFractures.size === 0) return;
    const t0 = startTiming();

    // Group fractures by actor index for the fracture command format
    const fracturesByActor = new Map<number, Array<{ userdata: number; nodeIndex0: number; nodeIndex1: number; health: number }>>();
    for (const [nodeA, partners] of pendingDamageFractures) {
      for (const nodeB of partners) {
        const bondList = bondsByNode.get(nodeA);
        if (!bondList) continue;
        for (const bi of bondList) {
          const b = bondTable[bi];
          if (!b) continue;
          if ((b.node0 === nodeA && b.node1 === nodeB) || (b.node0 === nodeB && b.node1 === nodeA)) {
            if (removedBondIndices.has(bi)) break;
            const actorIndex = nodeToActor.get(nodeA) ?? 0;
            let fractures = fracturesByActor.get(actorIndex);
            if (!fractures) { fractures = []; fracturesByActor.set(actorIndex, fractures); }
            fractures.push({ userdata: bi, nodeIndex0: b.node0, nodeIndex1: b.node1, health: 1e9 });
            removedBondIndices.add(bi);
            break;
          }
        }
      }
    }
    pendingDamageFractures.clear();

    const commands: Array<{ actorIndex: number; fractures: Array<{ userdata: number; nodeIndex0: number; nodeIndex1: number; health: number }> }> = [];
    for (const [actorIndex, fractures] of fracturesByActor) {
      if (fractures.length > 0) commands.push({ actorIndex, fractures });
    }
    if (commands.length === 0) {
      stopTiming(t0, 'damageFlushMs');
      return;
    }

    try {
      const splitEvents = profiledApplyFractureCommands(commands as FractureCommands);
      if (splitEvents.length > 0) {
        processSplitEvents(splitEvents);
        flushPendingBodies();
        flushColliderMigrations();
        cleanupDisabledColliders();
      }
    } catch (e) {
      if (isDev) console.error('[Core] flushPendingDamageFractures failed', e);
    }
    stopTiming(t0, 'damageFlushMs');
  }

  function damageDrivePass(dt: number) {
    if (!damageOptions.enabled) return false;

    const dSnapT0 = startTiming();
    const damageSnapshot: DamageStateSnapshot | null = resimulateOnDamageDestroy
      ? damageSystem.captureImpactState()
      : null;
    stopTiming(dSnapT0, 'damageSnapshotMs');

    // Apply buffered contacts (with speed scaling + local points) via replay buffer
    const previewT0 = startTiming();
    contactReplayBuffer.replay(damageSystem);
    const previewDestroyed = damageSystem.previewTick(dt);
    stopTiming(previewT0, 'damagePreviewMs');

    if (previewDestroyed.length > 0 && resimulateOnDamageDestroy && damageSnapshot) {
      const restoreT0 = startTiming();
      damageSystem.restoreImpactState(damageSnapshot);
      stopTiming(restoreT0, 'damageRestoreMs');

      const preDestroyT0 = startTiming();
      for (const ni of previewDestroyed) {
        const chunk = chunks[ni];
        if (!chunk) continue;

        const neighborBondIndices = bondsByNode.get(ni);
        if (neighborBondIndices) {
          for (const bi of neighborBondIndices) {
            if (removedBondIndices.has(bi)) continue;
            const b = bondTable[bi];
            if (!b) continue;
            const otherNode = b.node0 === ni ? b.node1 : b.node0;
            let set = pendingDamageFractures.get(ni);
            if (!set) { set = new Set(); pendingDamageFractures.set(ni, set); }
            set.add(otherNode);
          }
        }

        if (damageOptions.autoDetachOnDestroy) {
          try { handleNodeDestroyed(ni, 'impact'); } catch {}
        }
      }
      stopTiming(preDestroyT0, 'damagePreDestroyMs');

      flushPendingDamageFractures();

      return true;
    }

    // No rollback needed — the first replay() above already accumulated contacts
    // into pendingDamage. previewTick(preview:true) did NOT consume them, so
    // tick() will process them correctly. Do NOT replay again (would double-count).
    const tickT0 = startTiming();
    const tickDestroyed = damageSystem.tick(dt);
    stopTiming(tickT0, 'damageTickMs');

    for (const ni of tickDestroyed) {
      if (!chunks[ni]) continue;

      if (damageOptions.autoDetachOnDestroy) {
        try { handleNodeDestroyed(ni, 'impact'); } catch {}
      }
    }

    return false;
  }

  let lastStepDt = readWorldDt();

  function step(dt: number) {
    if (activeProfilerSample && !activeProfilerSample.finalized) {
      activeProfilerSample.finalized = true;
    }

    const prevDt = readWorldDt();
    const clampedDt = clampStepDt(dt);
    lastStepDt = clampedDt;

    activeProfilerSample = profiler.enabled ? createProfilerSample(clampedDt) : null;

    const totalT0 = startTiming();

    const preStepT0 = startTiming();
    processDebrisCleanup();
    processSmallBodyDamping();
    processSleepThresholds();
    cleanupDisabledColliders();
    stopTiming(preStepT0, 'preStepSweepMs');

    applyExternalForcesFromBuffer();
    spawnPendingProjectiles();

    if (resimulateOnFracture) {
      captureWorldSnapshot();
    }

    const initialT0 = startTiming();
    // Set the clamped dt on the world, restore original afterwards
    setWorldDtValue(clampedDt);
    const rapierT0 = startTiming();
    world.step(eventQueue);
    stopTiming(rapierT0, 'rapierStepMs');

    drainContactForces();

    if (activeProfilerSample) {
      activeProfilerSample.bufferedExternalContacts = bufferedExternalContacts.length;
      activeProfilerSample.bufferedInternalContacts = bufferedInternalContacts.length;
    }

    const needsResim = damageDrivePass(dt);

    const hadFracture = processOneFracturePass(0, ['initial']);
    stopTiming(initialT0, 'initialPassMs');

    if (hadFracture || needsResim) {
      flushPendingBodies();
      flushColliderMigrations();
      rebuildColliderToNodeMap();

      let resimCount = 0;
      const maxResim = Math.max(0, maxResimulationPasses);

      while (resimCount < maxResim) {
        const resimT0 = startTiming();
        if (resimulateOnFracture) {
          restoreWorldSnapshot();
          captureWorldSnapshot();
          const rapierResimT0 = startTiming();
          world.step(eventQueue);
          stopTiming(rapierResimT0, 'rapierStepMs');
          drainContactForces();
        }

        const hadMore = processOneFracturePass(resimCount + 1, ['resim']);
        stopTiming(resimT0, 'resimMs');
        resimCount++;

        if (activeProfilerSample) {
          activeProfilerSample.resimPasses = resimCount;
          activeProfilerSample.resimReasons.push(needsResim ? 'damage' : 'fracture');
        }

        if (!hadMore) break;

        flushPendingBodies();
        flushColliderMigrations();
        rebuildColliderToNodeMap();
      }
    }

    flushPendingBodies();
    flushColliderMigrations();

    cleanupExpiredProjectiles();

    for (let ci = 0; ci < chunks.length; ci++) {
      const chunk = chunks[ci];
      if (!chunk.active || chunk.bodyHandle == null) continue;
      const body = world.getRigidBody(chunk.bodyHandle);
      if (!body) continue;
      const pos = body.translation();
      const rot = body.rotation();
      const lx = chunk.localOffset.x, ly = chunk.localOffset.y, lz = chunk.localOffset.z;
      const qx = rot.x, qy = rot.y, qz = rot.z, qw = rot.w;
      const rx = qw * lx + qy * lz - qz * ly;
      const ry = qw * ly + qz * lx - qx * lz;
      const rz = qw * lz + qx * ly - qy * lx;
      const rw = -(qx * lx + qy * ly + qz * lz);
      chunk.worldPosition = {
        x: pos.x + rw * (-qx) + rx * qw + ry * (-qz) - rz * (-qy),
        y: pos.y + rw * (-qy) + ry * qw + rz * (-qx) - rx * (-qz),
        z: pos.z + rw * (-qz) + rz * qw + rx * (-qy) - ry * (-qx),
      };
      chunk.worldQuaternion = { x: rot.x, y: rot.y, z: rot.z, w: rot.w };
    }

    if (activeProfilerSample) {
      activeProfilerSample.projectiles = projectiles.length;
      activeProfilerSample.rigidBodies = countRigidBodies();
      // Capture body/chunk distribution stats
      const bcs = captureBodyColliderStats();
      if (bcs) {
        activeProfilerSample.bodyCount = bcs.bodyCount;
        activeProfilerSample.bodyColliderCountMin = bcs.min;
        activeProfilerSample.bodyColliderCountMax = bcs.max;
        activeProfilerSample.bodyColliderCountAvg = bcs.avg;
        activeProfilerSample.bodyColliderCountMedian = bcs.median;
        activeProfilerSample.bodyColliderCountP95 = bcs.p95;
      }
    }

    stopTiming(totalT0, 'totalMs');

    if (activeProfilerSample) {
      activeProfilerSample.finalized = true;
      profiler.onSample?.(activeProfilerSample);
    }

    // Restore original world dt if we overrode it
    setWorldDtValue(prevDt);

    safeFrames++;
    if (safeFrames === 1 && colliderToNode.size === 0 && !warnedColliderMapEmptyOnce) {
      warnedColliderMapEmptyOnce = true;
      console.warn('[Core] colliderToNode is empty after first step');
    }
  }

  function dispose() {
    try { world.free(); } catch {}
    try { eventQueue.free(); } catch {}
    try { solver.destroy(); } catch {}
  }

  function getRigidBodyCount(): number {
    return countRigidBodies();
  }

  function getActiveBondsCount(): number {
    return bondTable.length - removedBondIndices.size;
  }

  function getSolverDebugLines(): Array<{ p0: Vec3; p1: Vec3; color0: number; color1: number }> {
    try {
      return (solver as any).fillDebugRender?.({ mode: 0 /* ExtDebugMode.Max */, scale: 1.0 }) ?? [];
    } catch {
      return [];
    }
  }

  function getNodeBonds(nodeIndex: number): BondRef[] {
    const indices = bondsByNode.get(nodeIndex);
    if (!indices) return [];
    return indices
      .filter((bi) => !removedBondIndices.has(bi))
      .map((bi) => {
        const b = bondTable[bi];
        return { index: bi, node0: b.node0, node1: b.node1, centroid: b.centroid, normal: b.normal, area: b.area };
      });
  }

  function cutBond(bondIndex: number): boolean {
    if (removedBondIndices.has(bondIndex)) return false;
    (solver as any).removeBondByUserdata?.(bondIndex);
    removedBondIndices.add(bondIndex);
    return true;
  }

  function cutNodeBonds(nodeIndex: number): boolean {
    const indices = bondsByNode.get(nodeIndex);
    if (!indices) return false;
    let cut = false;
    for (const bi of indices) {
      if (!removedBondIndices.has(bi)) {
        (solver as any).removeBondByUserdata?.(bi);
        removedBondIndices.add(bi);
        cut = true;
      }
    }
    return cut;
  }

  /**
   * Centralized node destruction handler. Marks the chunk as destroyed,
   * cuts its bonds from the WASM solver, cleans up collider/body links,
   * and notifies listeners. Matches vibe-city's handleNodeDestroyed pattern.
   */
  function handleNodeDestroyed(nodeIndex: number, reason: 'impact' | 'manual') {
    const chunk = chunks[nodeIndex];
    if (!chunk) return;

    // Mark flags
    chunk.destroyed = true;
    chunk.active = false;
    if (chunk.health != null) chunk.health = 0;

    // Cut bonds from the WASM solver so it stops computing stress on destroyed nodes
    try { cutNodeBonds(nodeIndex); } catch {}

    // Disable/remove collider
    if (chunk.colliderHandle != null) {
      const oldC = world.getCollider(chunk.colliderHandle);
      if (oldC) oldC.setEnabled(false);
      colliderToNode.delete(chunk.colliderHandle);
      activeContactColliders.delete(chunk.colliderHandle);
      disabledCollidersToRemove.add(chunk.colliderHandle);
      chunk.colliderHandle = null;
    }

    // Unregister body link
    unregisterNodeBodyLink(nodeIndex, chunk.bodyHandle);

    // Notify
    try {
      onNodeDestroyed?.({ nodeIndex, actorIndex: nodeToActor.get(nodeIndex) ?? 0, reason });
    } catch {}
  }

  function applyExternalForce(nodeIndex: number, worldPoint: Vec3, worldForce: Vec3) {
    pendingExternalForces.push({ nodeIndex, point: worldPoint, force: worldForce });
  }

  function enqueueProjectile(spawn: ProjectileSpawn) {
    pendingBallSpawns.push(spawn);
  }

  function setGravityFn(g: number) {
    world.gravity = { x: 0, y: g, z: 0 };
  }

  function setSolverGravityEnabled(v: boolean) {
    solverGravityEnabled = v;
  }

  function getCollisionGroupContext(): CollisionGroupContext {
    return {
      mode: debrisCollisionModeSetting,
      groundBodyHandle: groundBody.handle,
      maxCollidersForDebris: debrisCleanupSettings.maxCollidersForDebris,
    };
  }

  function applyCollisionGroupsToAllBodies() {
    const ctx = getCollisionGroupContext();
    world.forEachRigidBody((b) => {
      if (b.handle === rootBody.handle || b.handle === groundBody.handle) return;
      applyCollisionGroupsForBodyImpl(b, ctx);
    });
    applyCollisionGroupsForBodyImpl(
      world.getRigidBody(rootBody.handle)!,
      ctx,
    );
    applyCollisionGroupsForBodyImpl(
      world.getRigidBody(groundBody.handle)!,
      ctx,
    );
  }

  function setSingleCollisionMode(mode: SingleCollisionMode) {
    debrisCollisionModeSetting = toDebrisCollisionMode(mode);
    applyCollisionGroupsToAllBodies();
  }

  function setDebrisCollisionModeFn(mode: DebrisCollisionMode) {
    debrisCollisionModeSetting = mode;
    applyCollisionGroupsToAllBodies();
  }

  function applyNodeDamage(nodeIndex: number, amount: number) {
    damageSystem.applyDirect(nodeIndex, amount);
  }

  function getNodeHealth(nodeIndex: number) {
    return damageSystem.getHealth(nodeIndex) ?? null;
  }

  // Adapt projectiles to the expected interface shape.
  // Preserve existing entries so that external code (e.g. updateProjectileMeshes)
  // can attach properties like `mesh` that survive across frames.
  const coreProjectiles: DestructibleCore['projectiles'] = [];
  const coreProjectileByHandle = new Map<number, DestructibleCore['projectiles'][number]>();
  function syncProjectilesView() {
    // Build set of live handles for quick lookup
    const liveHandles = new Set<number>();
    for (const p of projectiles) {
      liveHandles.add(p.bodyHandle);
    }

    // Remove entries that no longer exist in the internal list
    for (let i = coreProjectiles.length - 1; i >= 0; i--) {
      if (!liveHandles.has(coreProjectiles[i].bodyHandle)) {
        coreProjectileByHandle.delete(coreProjectiles[i].bodyHandle);
        coreProjectiles.splice(i, 1);
      }
    }

    // Add new entries, preserving existing ones
    for (const p of projectiles) {
      let existing = coreProjectileByHandle.get(p.bodyHandle);
      if (!existing) {
        existing = {
          bodyHandle: p.bodyHandle,
          radius: p.radius,
          type: 'ball',
          spawnTime: p.createdAt,
        };
        coreProjectiles.push(existing);
        coreProjectileByHandle.set(p.bodyHandle, existing);
      } else {
        // Update mutable fields but keep the same object reference
        existing.bodyHandle = p.bodyHandle;
        existing.radius = p.radius;
        existing.spawnTime = p.createdAt;
      }
    }
  }

  // Apply initial collision groups to root and ground bodies
  try {
    const initCtx = getCollisionGroupContext();
    applyCollisionGroupsForBodyImpl(rootBody, initCtx);
    applyCollisionGroupsForBodyImpl(groundBody, initCtx);
  } catch { /* non-fatal */ }

  const originalStep = step;
  function wrappedStep(dtOverride?: number) {
    originalStep(dtOverride ?? (1 / 60));
    syncProjectilesView();
  }

  if (isDev) {
    try {
      const dw = (typeof window !== 'undefined' ? window : undefined) as DebugWindow | undefined;
      if (dw) {
        dw.debugStressSolver = {
          printSolver: () => {
            console.log('[Core] chunks', chunks.length, 'bonds', bondTable.length, 'removed', removedBondIndices.size);
            console.log('[Core] colliderStats', captureBodyColliderStats());
          },
        };
      }
    } catch {}
  }

  const core: DestructibleCore = {
    get world() { return world; },
    eventQueue,
    solver,
    runtime,
    rootBodyHandle: rootBody.handle,
    groundBodyHandle: groundBody.handle,
    gravity,
    chunks,
    colliderToNode,
    actorMap,
    step: wrappedStep,
    projectiles: coreProjectiles,
    enqueueProjectile,
    stepEventful: wrappedStep,
    stepSafe: wrappedStep,
    setGravity: setGravityFn,
    setSolverGravityEnabled,
    setSingleCollisionMode,
    setDebrisCollisionMode: setDebrisCollisionModeFn,
    getRigidBodyCount,
    getActiveBondsCount,
    getSolverDebugLines,
    getNodeBonds,
    cutBond,
    cutNodeBonds,
    applyExternalForce,
    setSleepThresholds: (linear: number, angular: number) => updateSleepThresholds(linear, angular),
    setSleepMode: updateSleepMode,
    getSleepSettings: () => ({ mode: sleepSettings.mode, linear: sleepSettings.linear, angular: sleepSettings.angular }),
    setSmallBodyDamping: updateSmallBodyDamping,
    getSmallBodyDampingSettings: () => ({ ...smallBodyDampingSettings }),
    hasBodyCollidedWithGround: (bodyHandle: number) => bodiesCollidedWithGround.has(bodyHandle),
    setDebrisCleanup: updateDebrisCleanup,
    getDebrisCleanupSettings: () => ({ ...debrisCleanupSettings }),
    setMaxCollidersForDebris: (n: number) => {
      debrisCleanupSettings.maxCollidersForDebris = Math.max(1, Math.floor(n));
      applyCollisionGroupsToAllBodies();
    },
    setFracturePolicy: (policy: FracturePolicy) => {
      if (policy.maxFracturesPerFrame != null) fracturePolicySettings.maxFracturesPerFrame = policy.maxFracturesPerFrame;
      if (policy.maxNewBodiesPerFrame != null) fracturePolicySettings.maxNewBodiesPerFrame = policy.maxNewBodiesPerFrame;
      if (policy.maxColliderMigrationsPerFrame != null) fracturePolicySettings.maxColliderMigrationsPerFrame = policy.maxColliderMigrationsPerFrame;
      if (policy.maxDynamicBodies != null) fracturePolicySettings.maxDynamicBodies = policy.maxDynamicBodies;
      if (policy.minChildNodeCount != null) fracturePolicySettings.minChildNodeCount = Math.max(1, policy.minChildNodeCount);
      if (policy.idleSkip != null) fracturePolicySettings.idleSkip = !!policy.idleSkip;
    },
    getFracturePolicy: () => ({ ...fracturePolicySettings }),
    applyNodeDamage: damageOptions.enabled ? applyNodeDamage : undefined,
    getNodeHealth: damageOptions.enabled ? getNodeHealth : undefined,
    damageEnabled: damageOptions.enabled,
    dispose,
    setProfiler,
    recordProjectileCleanupDuration: recordProjectileCleanupDurationInternal,
  };

  return core;
}

// applyCollisionGroupsForBody is now provided by ./collisionGroups.ts
// and called inline via applyCollisionGroupsForBodyImpl imported above.
