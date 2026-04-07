import type RAPIER from '@dimforge/rapier3d-compat';
import type { ColliderDesc } from '@dimforge/rapier3d-compat';
import type { ExtStressSolver, StressRuntime } from '../types';
export type Vec3 = { x: number; y: number; z: number };

/** @deprecated Use DebrisCollisionMode instead */
export type SingleCollisionMode =
  | 'all'
  | 'noSinglePairs'
  | 'singleGround'
  | 'singleNone';

/**
 * Debris collision mode - controls how debris bodies (bodies with few colliders) collide:
 * - 'all': All collisions allowed
 * - 'noDebrisPairs': Debris bodies don't collide with each other
 * - 'debrisGroundOnly': Debris only collides with the ground
 * - 'debrisNone': Debris has no collisions at all
 */
export type DebrisCollisionMode =
  | 'all'
  | 'noDebrisPairs'
  | 'debrisGroundOnly'
  | 'debrisNone';

/**
 * Mode for when optimization features should be applied:
 * - 'off': Disabled
 * - 'always': Always enabled
 * - 'afterGroundCollision': Only enabled after the body has collided with the ground
 */
export type OptimizationMode = 'off' | 'always' | 'afterGroundCollision';

export type SmallBodyDampingOptions = {
  /** When to apply small body damping (default: 'always') */
  mode?: OptimizationMode;
  /** Maximum collider count to be considered a "small body" (default: 3) */
  colliderCountThreshold?: number;
  /** Minimum linear damping to apply to small bodies (default: 2) */
  minLinearDamping?: number;
  /** Minimum angular damping to apply to small bodies (default: 2) */
  minAngularDamping?: number;
};

export type SleepThresholdOptions = {
  /** When to apply sleep thresholds (default: 'always') */
  mode?: OptimizationMode;
  /** Linear velocity threshold for sleeping (m/s) */
  linear?: number;
  /** Angular velocity threshold for sleeping (rad/s) */
  angular?: number;
};

/**
 * Options for automatic debris cleanup.
 * Debris = rigid bodies with few colliders (small fragments).
 */
export type DebrisCleanupOptions = {
  /** When to apply debris cleanup (default: 'always') */
  mode?: OptimizationMode;
  /** Time-to-live for debris in milliseconds (default: 10000 = 10s) */
  debrisTtlMs?: number;
  /** Maximum collider count for a body to be considered debris (default: 2) */
  maxCollidersForDebris?: number;
};

/**
 * Controls how fractures are processed per frame — a spectrum from full realism
 * to real-time performance. Each knob has a physical interpretation.
 *
 * All limits default to -1 (unlimited = original behavior).
 */
export type FracturePolicy = {
  /** Max bonds to break per frame. Highest-stress bonds are prioritized.
   *  Remaining overstressed bonds stay in graph and re-evaluate next frame —
   *  some may relax as the load path changes (physically accurate).
   *  Models "fracture propagation speed". -1 = unlimited. Default: -1 */
  maxFracturesPerFrame?: number;

  /** Max new rigid bodies to create per frame from splits.
   *  Largest children (most visually important) are created first.
   *  Remaining are deferred to subsequent frames.
   *  Models "fragmentation rate". -1 = unlimited. Default: -1 */
  maxNewBodiesPerFrame?: number;

  /** Max collider migrations per frame. Remaining migrations are deferred.
   *  Chunks stay on parent body briefly — appears as shockwave propagation.
   *  Models "topology change speed". -1 = unlimited. Default: -1 */
  maxColliderMigrationsPerFrame?: number;

  /** Global cap on dynamic rigid bodies in the world. When reached, fracture
   *  generation is suppressed (bonds stay intact until bodies are freed).
   *  Models "simulation complexity budget". -1 = unlimited. Default: -1 */
  maxDynamicBodies?: number;

  /** Minimum node count for a split child to receive its own body.
   *  Smaller children are destroyed (visual debris only, no physics body).
   *  Models "material grain size". Default: 1 (every fragment gets a body) */
  minChildNodeCount?: number;
};

// Builder that returns a Rapier collider descriptor for a node.
// Static Rapier import is required by callers; this type reuses Rapier's own types.
export type ColliderDescBuilder = () => ColliderDesc | null;

export type ScenarioNode = {
  centroid: Vec3;
  mass: number; // 0 => support
  volume: number;
};

export type ScenarioBond = {
  node0: number;
  node1: number;
  centroid: Vec3;
  normal: Vec3;
  area: number;
};

export type ScenarioDesc = {
  nodes: ScenarioNode[];
  bonds: ScenarioBond[];
  gridCoordinates?: Array<{ ix: number; iy: number; iz: number }>;
  spacing?: Vec3;
  parameters?: Record<string, unknown>;
  // Optional per-node collider descriptors (one entry per node index). If omitted or entry returns null,
  // the core falls back to a box collider sized from the node (nodeSize).
  colliderDescForNode?: Array<ColliderDescBuilder | null>;
};

export type ChunkData = {
  nodeIndex: number;
  size: Vec3;
  isSupport: boolean;
  baseLocalOffset: Vec3;
  localOffset: Vec3;
  colliderHandle: number | null;
  bodyHandle: number | null;
  active: boolean;
  detached: boolean;
  // Damage/health tracking for damageable chunks feature
  maxHealth?: number;
  health?: number;
  pendingDamage?: number;
  destroyed?: boolean;
  baseWorldPosition?: Vec3;
  worldPosition?: Vec3;
  worldQuaternion?: { x: number; y: number; z: number; w: number };
};

export type ProjectileSpawn = {
  position: Vec3;
  velocity: Vec3;
  radius?: number;
  mass?: number;
  ttl?: number;
  // Legacy fields
  x?: number;
  z?: number;
  type?: 'ball' | 'box';
  linvelY?: number;
  start?: Vec3;
  linvel?: Vec3;
  friction?: number;
  restitution?: number;
};

export type ProjectileState = {
  bodyHandle: number;
  radius: number;
  type?: 'ball' | 'box';
  spawnTime?: number;
  createdAt: number;
  ttl: number;
  mesh?: unknown;
};

export type BondRef = {
  index: number;
  node0: number;
  node1: number;
  area: number;
  centroid: Vec3;
  normal: Vec3;
};

export type CoreProfilerPass = {
  index: number;
  solverMs: number;
  fractureMs: number;
  bodyCreateMs: number;
  totalMs: number;
  reasons: string[];
};

export type CoreProfilerSample = {
  frameIndex: number;
  timestamp: number;
  dt: number;
  rapierStepMs: number;
  contactDrainMs: number;
  solverUpdateMs: number;
  damageReplayMs: number;
  damagePreviewMs: number;
  damageTickMs: number;
  fractureMs: number;
  fractureGenerateMs: number;
  fractureApplyMs: number;
  splitQueueMs: number;
  bodyCreateMs: number;
  colliderRebuildMs: number;
  cleanupDisabledMs: number;
  spawnMs: number;
  externalForceMs: number;
  damageSnapshotMs: number;
  damageRestoreMs: number;
  damagePreDestroyMs: number;
  damageFlushMs: number;
  preStepSweepMs: number;
  rebuildColliderMapMs: number;
  projectileCleanupMs: number;
  initialPassMs: number;
  resimMs: number;
  totalMs: number;
  resimPasses: number;
  resimReasons: string[];
  snapshotBytes: number;
  snapshotCaptureMs: number;
  snapshotRestoreMs: number;
  bufferedExternalContacts: number;
  bufferedInternalContacts: number;
  pendingExternalForces: number;
  projectiles: number;
  rigidBodies: number;
  passes: CoreProfilerPass[];
  // Extended stats (optional)
  splitChildCounts?: number[];
  splitPlannerMs?: number;
  bodyCount?: number;
  bodyColliderCountMin?: number | null;
  bodyColliderCountMax?: number | null;
  bodyColliderCountAvg?: number | null;
  bodyColliderCountMedian?: number | null;
  bodyColliderCountP95?: number | null;
};

export type SplitChild = {
  actorIndex: number;
  nodes: number[];
  isSupport: boolean;
};

export type CoreProfilerConfig = {
  enabled: boolean;
  onSample?: (sample: CoreProfilerSample) => void;
};

export type DestructibleCore = {
  world: RAPIER.World;
  eventQueue: RAPIER.EventQueue;
  solver: ExtStressSolver;
  runtime: StressRuntime;
  rootBodyHandle: number;
  groundBodyHandle: number;
  gravity: number;
  chunks: ChunkData[];
  colliderToNode: Map<number, number>;
  actorMap: Map<number, { bodyHandle: number }>;
  // Internal step control handled by core
  step: (dtOverride?: number) => void;
  projectiles: Array<{
    bodyHandle: number;
    radius: number;
    type: 'ball' | 'box';
    mesh?: unknown;
    spawnTime: number;
  }>;
  enqueueProjectile: (s: ProjectileSpawn) => void;
  stepEventful: (dtOverride?: number) => void;
  stepSafe: (dtOverride?: number) => void;
  setGravity: (g: number) => void;
  setSolverGravityEnabled: (v: boolean) => void;
  /** @deprecated Use setDebrisCollisionMode instead */
  setSingleCollisionMode: (mode: SingleCollisionMode) => void;
  setDebrisCollisionMode: (mode: DebrisCollisionMode) => void;
  getRigidBodyCount: () => number;
  getActiveBondsCount: () => number;
  getSolverDebugLines: () => Array<{ p0: Vec3; p1: Vec3; color0: number; color1: number }>;
  // Bond interaction helpers
  getNodeBonds: (nodeIndex: number) => BondRef[];
  cutBond: (bondIndex: number) => boolean;
  cutNodeBonds: (nodeIndex: number) => boolean;
  // External force application (non-contact force injection)
  applyExternalForce: (nodeIndex: number, worldPoint: Vec3, worldForce: Vec3) => void;
  setSleepThresholds?: (linear: number, angular: number) => void;
  setSleepMode?: (mode: OptimizationMode) => void;
  getSleepSettings?: () => SleepThresholdOptions;
  // Small body damping API - apply higher damping to bodies with few colliders
  setSmallBodyDamping?: (opts: SmallBodyDampingOptions) => void;
  getSmallBodyDampingSettings?: () => SmallBodyDampingOptions & { mode: OptimizationMode };
  // Query ground collision status for a body
  hasBodyCollidedWithGround?: (bodyHandle: number) => boolean;
  // Debris cleanup API - automatically remove small bodies after TTL
  setDebrisCleanup?: (opts: DebrisCleanupOptions) => void;
  getDebrisCleanupSettings?: () => DebrisCleanupOptions & { mode: OptimizationMode };
  setMaxCollidersForDebris?: (n: number) => void;
  // Damageable chunks API (present when damage is enabled)
  applyNodeDamage?: (nodeIndex: number, amount: number, reason?: string) => void;
  getNodeHealth?: (nodeIndex: number) => { health: number; maxHealth: number; destroyed: boolean } | null;
  damageEnabled?: boolean;
  // Fracture policy API - tune realism ↔ performance spectrum at runtime
  setFracturePolicy?: (policy: FracturePolicy) => void;
  getFracturePolicy?: () => Required<FracturePolicy>;
  dispose: () => void;
  setProfiler: (config: CoreProfilerConfig | null) => void;
  recordProjectileCleanupDuration?: (durationMs: number) => void;
};
