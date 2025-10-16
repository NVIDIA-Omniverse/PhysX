import type RAPIER from '@dimforge/rapier3d-compat';

export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

export interface ChunkData {
  nodeIndex: number;
  size: Vec3;
  isSupport: boolean;
  baseLocalOffset: Vec3;
  localOffset: Vec3;
  mesh: unknown | null; // Three.Mesh in browser, null in headless
  colliderHandle: number | null;
  bodyHandle: number | null;
  active: boolean;
  detached: boolean;
  baseColor: { r: number; g: number; b: number };
  stressSeverity: number;
  baseWorldPosition?: Vec3;
}

export interface ContactForce {
  force: Vec3;
  point: Vec3;
  impulse: Vec3;
}

export interface ActorMapEntry {
  bodyHandle: number;
}

export interface SplitChild {
  actorIndex: number;
  nodes: number[];
}

export interface SplitEvent {
  parentActorIndex: number;
  children: SplitChild[];
}

export interface SolverNode {
  centroid: Vec3;
  mass: number;
  volume: number;
}

export interface SolverBond {
  node0: number;
  node1: number;
  centroid: Vec3;
  normal: Vec3;
  area: number;
}

export interface BridgeCore {
  runtime: any; // stress solver runtime
  world: RAPIER.World & { RAPIER: typeof RAPIER };
  solver: any; // ExtStressSolver instance
  settings: any; // ExtStressSolverSettings
  scenario?: any; // scenario data (optional in browser)
  bodyHandle: number;
  chunks: ChunkData[];
  colliderToNode: Map<number, number>;
  activeContactColliders: Set<number>;
  pendingContactForces: Map<number, ContactForce>;
  contactForceScratch: any[];
  eventQueue: RAPIER.EventQueue;
  actorMap: Map<number, ActorMapEntry>;
  gravity: number;
  targetError: number;
  iterationCount: number;
  lastError: { lin: number; ang: number };
  overstressed: number;
  
  // Two-phase migration state
  pendingSplitResults: SplitEvent[];
  pendingBodiesToCreate: Array<{
    actorIndex: number;
    inheritFromBodyHandle: number;
    nodes: number[];
  }>;
  pendingColliderMigrations: Array<{
    nodeIndex: number;
    targetBodyHandle: number;
  }>;
  disabledCollidersToRemove: Set<number>;
  bodiesToRemove: Set<number>;
  
  // Safe frame control
  safeFrames: number;
  
  // Determinism/logging
  _handleSplitEventsCount: number;
  _splitLog: SplitEvent[][];
}

export interface ProcessFracturesResult {
  splitResults: SplitEvent[];
  fractureSets: any[];
}

