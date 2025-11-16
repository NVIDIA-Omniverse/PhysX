/**
 * TypeScript source-of-truth for the Blast Stress Solver JS/WASM bridge.
 *
 * This module provides a thin wrapper around the low-level stress solver and
 * the higher-level ExtStressSolver, exposing a runtime loader, type-safe
 * helpers, and ergonomic APIs that work both in Node/SSR and browsers with
 * modern bundlers (Vite/Webpack/Next.js). The WASM file is expected to be
 * colocated with the generated JS in the package's dist folder.
 */

import type {
  AuthoringChunkInput,
  BufferGeometryChunkOptions,
  BufferGeometryChunkResolver,
  BondingConfig,
  BondStress,
  ExtDebugModeValue,
  ExtForceModeValue,
  ExtStressBondDesc,
  ExtStressBondFracture,
  ExtStressDebugLine,
  ExtStressFractureSingleResult,
  ExtStressNodeDesc,
  ExtStressSolver as ExtStressSolverType,
  ExtStressSolverDescription,
  ExtStressSolverSettings,
  LoadStressSolverOptions,
  SplitEvent,
  StressImpulse,
  StressLimits as StressLimitsType,
  StressLimitsConfig,
  StressFailureValue,
  StressProcessor as StressProcessorType,
  StressProcessorDataParams,
  StressProcessorDescription,
  StressProcessorSolveOptions,
  StressProcessorSolveResult,
  StressProcessorSolverParams,
  StressRuntime,
  Vec3,
  RuntimeSizes
} from './types';
import type { BufferGeometry, Matrix4 } from 'three';

/** True when running under Node.js (CLI/SSR). */
const isNode = typeof process !== 'undefined' && (process as any).release?.name === 'node';

/**
 * Lazily import the Emscripten factory appropriate to the environment.
 * In Node we import the CommonJS build; otherwise the ESM build is used.
 */
const moduleFactoryPromise: Promise<any> = (async () => {
  if (isNode) {
    const factoryModule: any = await import('./stress_solver.cjs');
    return (factoryModule as any).default ?? factoryModule;
  }
  // Use a browser-safe variant that avoids importing Node's 'module'
  const factoryModule: any = await import('./stress_solver.browser.mjs');
  return (factoryModule as any).default ?? factoryModule;
})();

// Resolve colocated artifacts statically per environment to aid bundlers
// Node/CJS path uses __dirname; browser/ESM uses import.meta.url
// eslint-disable-next-line @typescript-eslint/ban-ts-comment
// @ts-ignore
// const NODE_DIST_DIR_URL = typeof __dirname !== 'undefined' ? new URL('./', 'file://' + __dirname + '/') : undefined as any;
// const BROWSER_DIST_DIR_URL = new URL('./', import.meta.url);

/** Node-only: dynamic reference to `fileURLToPath` for URL → path conversion. */
let fileURLToPathFn: ((url: URL) => string) | undefined;

/** Safely compute a browser base href for URL resolution under bundlers. */
/*
function browserBaseHref(): string | null {
  try {
    // Prefer import.meta.url if it is an http(s) URL (some bundlers rewrite it)
    // eslint-disable-next-line @typescript-eslint/ban-ts-comment
    // @ts-ignore
    const meta = typeof import.meta !== 'undefined' ? (import.meta as any) : undefined;
    if (meta && typeof meta.url === 'string') {
      const u = new URL(meta.url);
      if (u.protocol === 'http:' || u.protocol === 'https:') return u.href;
    }
  } catch {}
  if (typeof window !== 'undefined' && (window as any).location?.href) {
    return (window as any).location.href as string;
  }
  return null;
}
  */

/** Result codes used by fracture command generation helpers. */
export const FractureResult = Object.freeze({
  None: 0 as 0,
  Success: 1 as 1,
  Truncated: 2 as 2
});

/** Failure modes used by `StressLimits` and debug helpers. */
export const StressFailure = Object.freeze({
  Compression: 'compression' as const,
  Tension: 'tension' as const,
  Shear: 'shear' as const
});

/** Modes specifying whether applied values represent forces or accelerations. */
export const ExtForceMode = Object.freeze({
  Force: 0 as 0,
  Acceleration: 1 as 1
});

/** Debug render channels available from the stress solver. */
export const ExtDebugMode = Object.freeze({
  Max: 0 as 0,
  Compression: 1 as 1,
  Tension: 2 as 2,
  Shear: 3 as 3
});

/** Create a Vec3 utility object. */
export function vec3(x: number = 0.0, y: number = 0.0, z: number = 0.0): Vec3 {
  return { x, y, z };
}

/** Format a number for diagnostic output. */
export function formatNumber(value: number, width: number, precision: number): string {
  return value.toFixed(precision).padStart(width);
}

/** Format a vector for diagnostic output. */
export function formatVec3(v: Vec3, width: number = 6, precision: number = 2): string {
  return `(${formatNumber(v.x, width, precision)}, ${formatNumber(v.y, width, precision)}, ${formatNumber(v.z, width, precision)})`;
}

/**
 * Utility for evaluating material stress limits against solver output.
 */
export class StressLimits implements StressLimitsType {
  compressionElasticLimit: number;
  compressionFatalLimit: number;
  tensionElasticLimit: number;
  tensionFatalLimit: number;
  shearElasticLimit: number;
  shearFatalLimit: number;

  /**
   * @param config Optional overrides for elastic/fatal thresholds (Pascals).
   */
  constructor({
    compressionElasticLimit = 1.0,
    compressionFatalLimit = 2.0,
    tensionElasticLimit = -1.0,
    tensionFatalLimit = -1.0,
    shearElasticLimit = -1.0,
    shearFatalLimit = -1.0
  }: StressLimitsConfig = {}) {
    this.compressionElasticLimit = compressionElasticLimit;
    this.compressionFatalLimit = compressionFatalLimit;
    this.tensionElasticLimit = tensionElasticLimit;
    this.tensionFatalLimit = tensionFatalLimit;
    this.shearElasticLimit = shearElasticLimit;
    this.shearFatalLimit = shearFatalLimit;
  }

  /** Map raw stress magnitudes into normalized severities (0..1). */
  severity(stress: BondStress) {
    return {
      compression: mapStressValue(stress.compression, this.#compressionElastic(), this.#compressionFatal()),
      tension: mapStressValue(stress.tension, this.#tensionElastic(), this.#tensionFatal()),
      shear: mapStressValue(stress.shear, this.#shearElastic(), this.#shearFatal())
    } as const;
  }

  /** Determine which failure mode, if any, exceeds its fatal limit. */
  failureMode(stress: BondStress) {
    if (stress.compression > this.#compressionFatal()) return StressFailure.Compression;
    if (stress.tension > this.#tensionFatal()) return StressFailure.Tension;
    if (stress.shear > this.#shearFatal()) return StressFailure.Shear;
    return null;
  }

  /** Return the fatal threshold (Pa) associated with a failure mode. */
  fatalThreshold(mode: StressFailureValue) {
    switch (mode) {
      case 'compression': return this.#compressionFatal();
      case 'tension': return this.#tensionFatal();
      case 'shear': return this.#shearFatal();
      default: return 0.0;
    }
  }

  compressionElasticThreshold() { return this.#compressionElastic(); }
  compressionFatalThreshold() { return this.#compressionFatal(); }
  tensionElasticThreshold() { return this.#tensionElastic(); }
  tensionFatalThreshold() { return this.#tensionFatal(); }
  shearElasticThreshold() { return this.#shearElastic(); }
  shearFatalThreshold() { return this.#shearFatal(); }

  #compressionElastic() { return resolveLimit(this.compressionElasticLimit, 1.0); }
  #compressionFatal() { return resolveLimit(this.compressionFatalLimit, this.#compressionElastic()); }
  #tensionElastic() { return resolveLimit(this.tensionElasticLimit, this.#compressionElastic()); }
  #tensionFatal() { return resolveLimit(this.tensionFatalLimit, this.#compressionFatal()); }
  #shearElastic() { return resolveLimit(this.shearElasticLimit, this.#compressionElastic()); }
  #shearFatal() { return resolveLimit(this.shearFatalLimit, this.#compressionFatal()); }
}

/**
 * Load the WASM stress solver module and expose helper factories.
 *
 * Accepts optional Emscripten module overrides (e.g., custom locateFile).
 */
export async function loadStressSolver({ module: moduleOptions }: LoadStressSolverOptions = {}): Promise<StressRuntime> {
  if (isNode && !fileURLToPathFn) {
    const urlModule = await import('node:url');
    fileURLToPathFn = (urlModule as any).fileURLToPath as (u: URL) => string;
  }

  const factory = await moduleFactoryPromise;
  const options: Record<string, any> = { ...(moduleOptions ?? {}) };
  if (!options.locateFile) {
    options.locateFile = (p: string) => {
      // Node: resolve statically from __dirname
      // if (isNode && fileURLToPathFn) {
      //   try { return fileURLToPathFn(new URL('./' + p, new URL('file://' + __dirname + '/'))); } catch {}
      // }
      // Browser: allow bundlers to statically see the asset
      if (p.endsWith('.wasm')) {
        return new URL('./stress_solver.wasm', import.meta.url).href;
      }
      return p;
    };
  }
  if (isNode && !options.wasmBinary) {
    const fs = await import('node:fs/promises');
    // const wasmUrl = new URL('stress_solver.wasm', NODE_DIST_DIR_URL);
    const wasmUrl = new URL('stress_solver.wasm', import.meta.url);
    options.wasmBinary = await (fs as any).readFile(fileURLToPathFn!(wasmUrl));
  }
  options.print ??= (...args: any[]) => console.log('[blast-wasm]', ...args);
  options.printErr ??= (...args: any[]) => console.error('[blast-wasm]', ...args);

  const module = await factory(options);
  return createRuntime(module);
}

/**
 * Compute bond stress using solver impulse output and node positions.
 *
 * @param bond Bond linking node indices used to fetch centroids.
 * @param impulse Solver impulse acting on the bond (N·s).
 * @param nodes Node descriptors containing centroids (meters).
 * @param bondArea Remaining bond surface area (m²).
 */
export function computeBondStress(
  bond: { node0: number; node1: number },
  impulse: StressImpulse,
  nodes: Array<{ com: Vec3 }> | ReadonlyArray<{ com: Vec3 }> | undefined,
  bondArea: number
): BondStress {
  if (!bond || !impulse || !nodes || bondArea <= 0.0) {
    return { compression: 0.0, tension: 0.0, shear: 0.0 };
  }

  const node0 = nodes[bond.node0];
  const node1 = nodes[bond.node1];
  if (!node0 || !node1) {
    return { compression: 0.0, tension: 0.0, shear: 0.0 };
  }

  const displacement = subtract(node1.com, node0.com);
  const nodeDistance = Math.max(magnitude(displacement), 1.0e-6);
  const bondNormal = normalize(displacement);

  const linear = impulse.lin ?? vec3();
  const angular = impulse.ang ?? vec3();

  const normalLinear = dot(linear, bondNormal);
  const shearLinearSq = Math.max(magnitudeSquared(linear) - normalLinear * normalLinear, 0.0);
  let stressNormal = normalLinear / bondArea;
  let stressShear = Math.sqrt(shearLinearSq) / bondArea;

  const normalAngular = Math.abs(dot(angular, bondNormal));
  const angularMagSq = magnitudeSquared(angular);
  const twist = normalAngular / bondArea;
  const bendSq = Math.max(angularMagSq - normalAngular * normalAngular, 0.0);
  const bend = Math.sqrt(bendSq) / bondArea;

  const twistContribution = (twist * 2.0) / nodeDistance;
  stressShear += twistContribution;

  const bendContribution = (bend * 2.0) / nodeDistance;
  const sign = stressNormal >= 0.0 ? 1.0 : -1.0;
  stressNormal += bendContribution * sign;

  return {
    compression: stressNormal < 0.0 ? -stressNormal : 0.0,
    tension: stressNormal > 0.0 ? stressNormal : 0.0,
    shear: stressShear
  };
}

/** Return a − b. */
export function subtract(a: Vec3, b: Vec3): Vec3 {
  return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

/** Dot product between vectors. */
export function dot(a: Vec3, b: Vec3): number {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

/** Squared magnitude of a vector. */
export function magnitudeSquared(v: Vec3): number {
  return dot(v, v);
}

/** Vector magnitude. */
export function magnitude(v: Vec3): number {
  return Math.sqrt(magnitudeSquared(v));
}

/** Normalized vector (returns zero vector when length is ~0). */
export function normalize(v: Vec3): Vec3 {
  const mag = magnitude(v);
  if (mag <= 1.0e-6) {
    return vec3();
  }
  return vec3(v.x / mag, v.y / mag, v.z / mag);
}

function createRuntime(module: any): StressRuntime {
  const sizes: RuntimeSizes = {
    vec3: module.ccall('stress_sizeof_stress_vec3', 'number', [], []),
    node: module.ccall('stress_sizeof_node_desc', 'number', [], []),
    bond: module.ccall('stress_sizeof_bond_desc', 'number', [], []),
    velocity: module.ccall('stress_sizeof_velocity', 'number', [], []),
    impulse: module.ccall('stress_sizeof_impulse', 'number', [], []),
    dataParams: module.ccall('stress_sizeof_data_params', 'number', [], []),
    solverParams: module.ccall('stress_sizeof_solver_params', 'number', [], []),
    errorSq: module.ccall('stress_sizeof_error_sq', 'number', [], []),
    extNode: module.ccall('ext_stress_sizeof_ext_node_desc', 'number', [], []),
    extBond: module.ccall('ext_stress_sizeof_ext_bond_desc', 'number', [], []),
    extSettings: module.ccall('ext_stress_sizeof_ext_settings', 'number', [], []),
    extDebugLine: module.ccall('ext_stress_sizeof_ext_debug_line', 'number', [], []),
    extBondFracture: module.ccall('ext_stress_sizeof_ext_bond_fracture', 'number', [], []),
    extFractureCommands: module.ccall('ext_stress_sizeof_ext_fracture_commands', 'number', [], []),
    extActor: module.ccall('ext_stress_sizeof_actor_buffer', 'number', [], []),
    extSplitEvent: module.ccall('ext_stress_sizeof_ext_split_event', 'number', [], []),
    authoringBond: module.ccall('authoring_sizeof_ext_bond_desc', 'number', [], [])
  };

  const memory = new ModuleMemory(module);

  return {
    module,
    sizes,
    FractureResult,
    vec3,
    StressLimits,
    StressFailure,
    ExtForceMode,
    ExtDebugMode,
    computeBondStress,
    defaultSolverParams: () => ({ maxIterations: 32, tolerance: 1.0e-6, warmStart: false }),
    defaultExtSettings: () => ({
      maxSolverIterationsPerFrame: 25,
      graphReductionLevel: 0,
      compressionElasticLimit: 1.0,
      compressionFatalLimit: 2.0,
      tensionElasticLimit: -1.0,
      tensionFatalLimit: -1.0,
      shearElasticLimit: -1.0,
      shearFatalLimit: -1.0
    }),
    createProcessor(description: StressProcessorDescription): StressProcessorType {
      return new StressProcessor(module, memory, sizes, description);
    },
    createExtSolver(description: ExtStressSolverDescription): ExtStressSolverType {
      return new ExtStressSolver(module, memory, sizes, description);
    },
    createBondsFromTriangles(chunks: AuthoringChunkInput[], config?: BondingConfig) {
      return generateBondsFromTriangles(module, memory, sizes, chunks, config);
    }
  };
}

class ModuleMemory {
  module: any;
  dataView: DataView;

  constructor(module: any) {
    this.module = module;
    this.dataView = new DataView(module.HEAPU8.buffer);
  }

  view() {
    if (this.dataView.buffer !== this.module.HEAPU8.buffer) {
      this.dataView = new DataView(this.module.HEAPU8.buffer);
    }
    return this.dataView;
  }

  alloc(size: number) {
    const ptr = this.module._malloc(size);
    if (ptr === 0) {
      throw new Error('Stress solver allocation failed');
    }
    this.view();
    return ptr as number;
  }

  free(ptr: number) {
    if (!ptr) return;
    this.module._free(ptr);
  }

  zero(ptr: number, size: number) {
    this.module.HEAPU8.fill(0, ptr, ptr + size);
  }
}

class StressProcessor implements StressProcessorType {
  module: any;
  memory: ModuleMemory;
  sizes: RuntimeSizes;
  nodeCount: number;
  bondCount: number;
  handle: number;
  nodeScratchPtr: number;
  bondScratchPtr: number;
  velocitiesPtr: number;
  impulsesPtr: number;
  errorPtr: number;
  solverParamsPtr: number;
  _nodes: any[];
  _bonds: any[];
  _impulses: StressImpulse[];
  _solverParams: Required<StressProcessorSolverParams>;

  constructor(module: any, memory: ModuleMemory, sizes: RuntimeSizes, description: StressProcessorDescription) {
    if (!description) throw new Error('StressProcessor description is required');
    const nodes = description.nodes ?? [];
    const bonds = description.bonds ?? [];
    if (nodes.length === 0 || bonds.length === 0) {
      throw new Error('StressProcessor requires at least one node and one bond');
    }

    this.module = module;
    this.memory = memory;
    this.sizes = sizes;

    this.nodeCount = nodes.length;
    this.bondCount = bonds.length;

    const nodesPtr = memory.alloc(sizes.node * this.nodeCount);
    const bondsPtr = memory.alloc(sizes.bond * this.bondCount);
    const dataParamsPtr = memory.alloc(sizes.dataParams);
    const view = memory.view();

    nodes.forEach((node, index) => writeNode(view, nodesPtr + index * sizes.node, node));
    bonds.forEach((bond, index) => writeBond(view, bondsPtr + index * sizes.bond, bond));

    const dataParams: StressProcessorDataParams = description.dataParams ?? {};
    view.setUint8(dataParamsPtr, dataParams.equalizeMasses ? 1 : 0);
    view.setUint8(dataParamsPtr + 1, dataParams.centerBonds ? 1 : 0);

    const handle = module.ccall(
      'stress_processor_create',
      'number',
      ['number', 'number', 'number', 'number', 'number'],
      [nodesPtr, this.nodeCount, bondsPtr, this.bondCount, dataParamsPtr]
    );

    memory.free(nodesPtr);
    memory.free(bondsPtr);
    memory.free(dataParamsPtr);

    if (!handle) throw new Error('Failed to create StressProcessor');

    this.handle = handle;

    this.nodeScratchPtr = memory.alloc(sizes.node);
    this.bondScratchPtr = memory.alloc(sizes.bond);
    this.velocitiesPtr = memory.alloc(sizes.velocity * this.nodeCount);
    this.impulsesPtr = memory.alloc(sizes.impulse * this.bondCount);
    this.errorPtr = memory.alloc(sizes.errorSq);
    this.solverParamsPtr = memory.alloc(sizes.solverParams);

    this._nodes = this._fetchAllNodes();
    this._bonds = this._fetchAllBonds();
    this._impulses = Array.from({ length: this.bondCount }, () => createImpulse());
    this._solverParams = { maxIterations: 32, tolerance: 1.0e-6, warmStart: false };
    this._writeSolverParams(this._solverParams);
  }

  getNodes() { return this._nodes.map(cloneNode); }
  getBonds() { this._bonds = this._fetchAllBonds(); return this._bonds.map(cloneBond); }
  internalBondDesc(index: number) { return this._bonds[index]; }
  getImpulses() { return this._impulses.map(cloneImpulse); }
  setImpulses(impulses: StressImpulse[]) {
    if (!Array.isArray(impulses) || impulses.length !== this.bondCount) {
      throw new Error('Impulse array must match bond count');
    }
    this._impulses = impulses.map(cloneImpulse);
  }
  setSolverParams(params: StressProcessorSolverParams) {
    this._solverParams = {
      maxIterations: params?.maxIterations ?? this._solverParams.maxIterations,
      tolerance: params?.tolerance ?? this._solverParams.tolerance,
      warmStart: params?.warmStart ?? this._solverParams.warmStart
    };
    this._writeSolverParams(this._solverParams);
  }

  nodeDesc(index: number) {
    if (index < 0 || index >= this.nodeCount) throw new Error('Node index out of range');
    const ok = this.module.ccall('stress_processor_get_node_desc', 'number', ['number', 'number', 'number'], [this.handle, index, this.nodeScratchPtr]);
    if (!ok) throw new Error(`Failed to read node descriptor ${index}`);
    const view = this.memory.view();
    return readNode(view, this.nodeScratchPtr);
  }

  bondDesc(index: number) {
    if (index < 0 || index >= this.bondCount) throw new Error('Bond index out of range');
    const ok = this.module.ccall('stress_processor_get_bond_desc', 'number', ['number', 'number', 'number'], [this.handle, index, this.bondScratchPtr]);
    if (!ok) throw new Error(`Failed to read bond descriptor ${index}`);
    const view = this.memory.view();
    return readBond(view, this.bondScratchPtr);
  }

  solve({ velocities, solverParams, resume = false }: StressProcessorSolveOptions): StressProcessorSolveResult {
    if (!Array.isArray(velocities) || velocities.length !== this.nodeCount) {
      throw new Error('Velocity array must match node count');
    }
    if (solverParams) this.setSolverParams(solverParams);

    const velocitySize = this.sizes.velocity * this.nodeCount;
    const impulseSize = this.sizes.impulse * this.bondCount;
    this.memory.zero(this.velocitiesPtr, velocitySize);
    this.memory.zero(this.impulsesPtr, impulseSize);
    this.memory.zero(this.errorPtr, this.sizes.errorSq);

    const view = this.memory.view();
    velocities.forEach((velocity, index) => {
      const base = this.velocitiesPtr + index * this.sizes.velocity;
      writeVec3(view, base, velocity.ang ?? vec3());
      writeVec3(view, base + this.sizes.vec3, velocity.lin ?? vec3());
    });

    this._impulses.forEach((impulse, index) => {
      const base = this.impulsesPtr + index * this.sizes.impulse;
      writeVec3(view, base, impulse.ang);
      writeVec3(view, base + this.sizes.vec3, impulse.lin);
    });

    const iterations = this.module.ccall(
      'stress_processor_solve',
      'number',
      ['number', 'number', 'number', 'number', 'number', 'number'],
      [this.handle, this.impulsesPtr, this.velocitiesPtr, this.solverParamsPtr, this.errorPtr, resume ? 1 : 0]
    );

    if (iterations < 0) throw new Error('Stress solver reported an error');

    this._impulses = this._readImpulses();
    const errorView = this.memory.view();
    const angularError = Math.sqrt(errorView.getFloat32(this.errorPtr, true));
    const linearError = Math.sqrt(errorView.getFloat32(this.errorPtr + 4, true));

    return { iterations, error: { ang: angularError, lin: linearError }, impulses: this.getImpulses() };
  }

  removeBond(index: number) {
    if (index < 0 || index >= this.bondCount) return false;
    const removed = this.module.ccall('stress_processor_remove_bond', 'number', ['number', 'number'], [this.handle, index]);
    if (!removed) return false;

    this.bondCount = this.module.ccall('stress_processor_bond_count', 'number', ['number'], [this.handle]);

    this.memory.free(this.impulsesPtr);
    this.impulsesPtr = this.memory.alloc(this.sizes.impulse * this.bondCount);
    this._impulses = Array.from({ length: this.bondCount }, () => createImpulse());
    this._bonds = this._fetchAllBonds();
    return true;
  }

  destroy() {
    if (!this.handle) return;
    this.module.ccall('stress_processor_destroy', 'void', ['number'], [this.handle]);
    this.handle = 0 as any;
    this.memory.free(this.nodeScratchPtr);
    this.memory.free(this.bondScratchPtr);
    this.memory.free(this.velocitiesPtr);
    this.memory.free(this.impulsesPtr);
    this.memory.free(this.errorPtr);
    this.memory.free(this.solverParamsPtr);
  }

  _fetchAllNodes() {
    const nodes = [] as any[];
    for (let i = 0; i < this.nodeCount; ++i) nodes.push(this.nodeDesc(i));
    return nodes;
  }

  _fetchAllBonds() {
    const bonds = [] as any[];
    for (let i = 0; i < this.bondCount; ++i) bonds.push(this.bondDesc(i));
    return bonds;
  }

  _readImpulses(): StressImpulse[] {
    const impulses: StressImpulse[] = [];
    const view = this.memory.view();
    for (let i = 0; i < this.bondCount; ++i) {
      const base = this.impulsesPtr + i * this.sizes.impulse;
      impulses.push({ ang: readVec3(view, base), lin: readVec3(view, base + this.sizes.vec3) });
    }
    return impulses;
  }

  _writeSolverParams(params: Required<StressProcessorSolverParams>) {
    const view = this.memory.view();
    view.setUint32(this.solverParamsPtr, (params.maxIterations! >>> 0), true);
    view.setFloat32(this.solverParamsPtr + 4, params.tolerance!, true);
    view.setUint8(this.solverParamsPtr + 8, params.warmStart ? 1 : 0);
    view.setUint8(this.solverParamsPtr + 9, 0);
    view.setUint8(this.solverParamsPtr + 10, 0);
    view.setUint8(this.solverParamsPtr + 11, 0);
  }
}

class ExtStressSolver implements ExtStressSolverType {
  module: any;
  memory: ModuleMemory;
  sizes: RuntimeSizes;
  nodeCount: number;
  bondCount: number;
  handle: number;
  _debugCapacity: number;
  _fractureCapacity: number;
  _debugPtr: number;
  _fracturePtr: number;
  _fractureCommandsPtr: number;
  _forcePtr: number;
  _torquePtr: number;

  constructor(module: any, memory: ModuleMemory, sizes: RuntimeSizes, description: ExtStressSolverDescription) {
    if (!description) throw new Error('ExtStressSolver description is required');

    const nodes = description.nodes ?? [];
    const bonds = description.bonds ?? [];
    if (nodes.length === 0 || bonds.length === 0) {
      throw new Error('ExtStressSolver requires at least one node and one bond');
    }

    this.module = module;
    this.memory = memory;
    this.sizes = sizes;

    this.nodeCount = nodes.length;
    this.bondCount = bonds.length;

    const nodesPtr = memory.alloc(sizes.extNode * this.nodeCount);
    const bondsPtr = memory.alloc(sizes.extBond * this.bondCount);
    const settingsPtr = description.settings ? memory.alloc(sizes.extSettings) : 0;

    try {
      const view = memory.view();
      nodes.forEach((node, index) => writeExtNode(view, nodesPtr + index * sizes.extNode, node));
      bonds.forEach((bond, index) => writeExtBond(view, bondsPtr + index * sizes.extBond, bond));
      if (settingsPtr) writeExtSettings(view, settingsPtr, description.settings!);

      const handle = module.ccall(
        'ext_stress_solver_create',
        'number',
        ['number', 'number', 'number', 'number', 'number'],
        [nodesPtr, this.nodeCount, bondsPtr, this.bondCount, settingsPtr]
      );

      if (!handle) throw new Error('Failed to create ExtStressSolver');

      this.handle = handle >>> 0;
      this._debugCapacity = Math.max(this.bondCount, 1);
      this._debugPtr = memory.alloc(this._debugCapacity * sizes.extDebugLine);
      this._fractureCapacity = Math.max(this.bondCount, 1);
      this._fracturePtr = memory.alloc(this._fractureCapacity * sizes.extBondFracture);
      this._fractureCommandsPtr = memory.alloc(sizes.extFractureCommands);
      this._forcePtr = memory.alloc(sizes.vec3);
      this._torquePtr = memory.alloc(sizes.vec3);
    } finally {
      memory.free(nodesPtr);
      memory.free(bondsPtr);
      if (settingsPtr) memory.free(settingsPtr);
    }
  }

  destroy(): void {
    if (!this.handle) return;
    this.module.ccall('ext_stress_solver_destroy', null, ['number'], [this.handle]);
    this.handle = 0 as any;
    if (this._debugPtr) { this.memory.free(this._debugPtr); this._debugPtr = 0 as any; }
    if (this._fracturePtr) { this.memory.free(this._fracturePtr); this._fracturePtr = 0 as any; }
    if (this._fractureCommandsPtr) { this.memory.free(this._fractureCommandsPtr); this._fractureCommandsPtr = 0 as any; }
    if (this._forcePtr) { this.memory.free(this._forcePtr); this._forcePtr = 0 as any; }
    if (this._torquePtr) { this.memory.free(this._torquePtr); this._torquePtr = 0 as any; }
  }

  setSettings(settings: ExtStressSolverSettings): void {
    if (!settings || !this.handle) return;
    const ptr = this.memory.alloc(this.sizes.extSettings);
    try {
      writeExtSettings(this.memory.view(), ptr, settings);
      this.module.ccall('ext_stress_solver_set_settings', null, ['number', 'number'], [this.handle, ptr]);
    } finally {
      this.memory.free(ptr);
    }
  }

  graphNodeCount(): number { if (!this.handle) return 0; return this.module.ccall('ext_stress_solver_graph_node_count', 'number', ['number'], [this.handle]) >>> 0; }
  bondCapacity(): number { return this.bondCount; }
  reset(): void { if (!this.handle) return; this.module.ccall('ext_stress_solver_reset', null, ['number'], [this.handle]); }

  addForce(nodeIndex: number, localPosition?: Vec3, localForce?: Vec3, mode: ExtForceModeValue = ExtForceMode.Force): void {
    if (!this.handle) return;
    const positionPtr = this.memory.alloc(this.sizes.vec3);
    const forcePtr = this.memory.alloc(this.sizes.vec3);
    try {
      const view = this.memory.view();
      writeVec3(view, positionPtr, localPosition ?? vec3());
      writeVec3(view, forcePtr, localForce ?? vec3());
      this.module.ccall('ext_stress_solver_add_force', null, ['number', 'number', 'number', 'number', 'number'], [this.handle, nodeIndex >>> 0, positionPtr, forcePtr, mode >>> 0]);
    } finally {
      this.memory.free(positionPtr);
      this.memory.free(forcePtr);
    }
  }

  addGravity(localGravity?: Vec3): void {
    if (!this.handle) return;
    const gravityPtr = this.memory.alloc(this.sizes.vec3);
    try {
      writeVec3(this.memory.view(), gravityPtr, localGravity ?? vec3());
      this.module.ccall('ext_stress_solver_add_gravity', null, ['number', 'number'], [this.handle, gravityPtr]);
    } finally {
      this.memory.free(gravityPtr);
    }
  }

  update(): void { if (!this.handle) return; this.module.ccall('ext_stress_solver_update', null, ['number'], [this.handle]); }
  overstressedBondCount(): number { if (!this.handle) return 0; return this.module.ccall('ext_stress_solver_overstressed_bond_count', 'number', ['number'], [this.handle]) >>> 0; }

  actorCount(): number { if (!this.handle) return 0; return this.module.ccall('ext_stress_solver_actor_count', 'number', ['number'], [this.handle]) >>> 0; }

  actors(): Array<{ actorIndex: number; nodes: number[] }> {
    if (!this.handle) return [];
    const actorCount = this.actorCount();
    if (actorCount === 0) return [];

    const actorStructSize = this.sizes.extActor;
    const actorPtr = this.memory.alloc(actorStructSize * actorCount);
    const nodesPtr = this.memory.alloc(this.nodeCount * 4);
    const actorCountPtr = this.memory.alloc(4);
    const nodeCountPtr = this.memory.alloc(4);

    try {
      this.module.ccall('ext_stress_solver_collect_actors', 'number', ['number', 'number', 'number', 'number', 'number', 'number', 'number'], [this.handle, actorPtr, actorCount, nodesPtr, this.nodeCount, actorCountPtr, nodeCountPtr]);

      const actualActorCount = this.memory.view().getUint32(actorCountPtr, true);
      const view = this.memory.view();
      const heapU32 = this.module.HEAPU32 as Uint32Array;
      const actors: Array<{ actorIndex: number; nodes: number[] }> = [];

      for (let i = 0; i < actualActorCount; ++i) {
        const base = actorPtr + i * actorStructSize;
        const actorIndex = view.getUint32(base, true);
        const nodesAddress = view.getUint32(base + 4, true);
        const nodeCount = view.getUint32(base + 8, true);

        const nodes: number[] = [];
        if (nodesAddress) {
          const offset = nodesAddress >>> 2;
          for (let n = 0; n < nodeCount; ++n) {
            nodes.push((heapU32 as any)[offset + n]);
          }
        }
        actors.push({ actorIndex, nodes });
      }
      return actors;
    } finally {
      this.memory.free(actorPtr);
      this.memory.free(nodesPtr);
      this.memory.free(actorCountPtr);
      this.memory.free(nodeCountPtr);
    }
  }

  generateFractureCommands({ maxBonds = this._fractureCapacity } = {}): ExtStressFractureSingleResult {
    if (!this.handle || !this._fracturePtr || !this._fractureCommandsPtr || maxBonds === 0) {
      return { actorIndex: 0, fractures: [], truncated: false, result: FractureResult.None };
    }
    const limit = Math.min(maxBonds, this._fractureCapacity);
    this.memory.zero(this._fractureCommandsPtr, this.sizes.extFractureCommands);
    const result = this.module.ccall('ext_stress_solver_generate_fracture_commands', 'number', ['number', 'number', 'number', 'number'], [this.handle, this._fractureCommandsPtr, this._fracturePtr, limit]);

    const view = this.memory.view();
    const actorIndex = view.getUint32(this._fractureCommandsPtr + 0, true);
    const count = view.getUint32(this._fractureCommandsPtr + 4, true);
    const fractures: ExtStressBondFracture[] = [];
    for (let i = 0; i < count; ++i) {
      const base = this._fracturePtr + i * this.sizes.extBondFracture;
      fractures.push(readExtBondFracture(view, base));
    }
    return { actorIndex, fractures, truncated: result === FractureResult.Truncated, result };
  }

  generateFractureCommandsPerActor(): Array<{ actorIndex: number; fractures: ExtStressBondFracture[] }> {
    if (!this.handle) return [];
    const actorCount = Math.max(1, this.actorCount());
    const commandPtr = this.memory.alloc(this.sizes.extFractureCommands * actorCount);
    const bondPtr = this.memory.alloc(this.sizes.extBondFracture * this.bondCount);
    const commandCountPtr = this.memory.alloc(4);
    const bondCountPtr = this.memory.alloc(4);

    try {
      this.module.ccall('ext_stress_solver_generate_fracture_commands_per_actor', 'number', ['number', 'number', 'number', 'number', 'number', 'number', 'number'], [this.handle, commandPtr, actorCount, bondPtr, this.bondCount, commandCountPtr, bondCountPtr]);

      const commandCount = this.memory.view().getUint32(commandCountPtr, true);
      const commands: Array<{ actorIndex: number; fractures: ExtStressBondFracture[] }> = [];
      const view = this.memory.view();

      for (let i = 0; i < commandCount; ++i) {
        const base = commandPtr + i * this.sizes.extFractureCommands;
        const actorIndex = view.getUint32(base, true);
        const fracturesPtr = view.getUint32(base + 4, true);
        const fractureCount = view.getUint32(base + 8, true);
        const fractures: ExtStressBondFracture[] = [];
        for (let f = 0; f < fractureCount; ++f) {
          const structBase = fracturesPtr + f * this.sizes.extBondFracture;
          fractures.push({
            userdata: view.getUint32(structBase, true),
            nodeIndex0: view.getUint32(structBase + 4, true),
            nodeIndex1: view.getUint32(structBase + 8, true),
            health: view.getFloat32(structBase + 12, true)
          });
        }
        commands.push({ actorIndex, fractures });
      }
      return commands;
    } finally {
      this.memory.free(commandPtr);
      this.memory.free(bondPtr);
      this.memory.free(commandCountPtr);
      this.memory.free(bondCountPtr);
    }
  }

  applyFractureCommands(fractureSets: Array<{ actorIndex: number; fractures: ExtStressBondFracture[] }>): SplitEvent[] {
    if (!this.handle || !Array.isArray(fractureSets) || fractureSets.length === 0) return [];

    const commandStructSize = this.sizes.extFractureCommands;
    const bondStructSize = this.sizes.extBondFracture;
    const splitEventSize = this.sizes.extSplitEvent;
    const actorStructSize = this.sizes.extActor;

    let totalBonds = 0;
    fractureSets.forEach((set) => { totalBonds += set?.fractures?.length ?? 0; });
    totalBonds = Math.max(totalBonds, 1);
    const baseCapacity = Math.max(this.nodeCount, 1);
    let childCapacity = Math.max(baseCapacity, fractureSets.length + 1);
    let nodeCapacity = baseCapacity;
    let splitEvents: SplitEvent[] = [];
    let status = 0;
    let attempts = 0;
    const maxAttempts = 4;

    do {
      const commandPtr = this.memory.alloc(commandStructSize * fractureSets.length);
      const bondPtr = this.memory.alloc(bondStructSize * totalBonds);
      const splitPtr = this.memory.alloc(splitEventSize * fractureSets.length);
      const childPtr = this.memory.alloc(actorStructSize * childCapacity);
      const nodesPtr = this.memory.alloc(nodeCapacity * 4);
      const eventCountPtr = this.memory.alloc(4);
      const childCountPtr = this.memory.alloc(4);
      const nodeCountPtr = this.memory.alloc(4);

      try {
        const commandView = this.memory.view();
        let bondOffset = 0;
        fractureSets.forEach((set, idx) => {
          const fractures = set?.fractures ?? [];
          const cmdBase = commandPtr + idx * commandStructSize;
          commandView.setUint32(cmdBase, set?.actorIndex >>> 0, true);
          commandView.setUint32(cmdBase + 8, fractures.length >>> 0, true);
          const fractureBase = bondPtr + bondOffset * bondStructSize;
          commandView.setUint32(cmdBase + 4, fractureBase >>> 0, true);
          fractures.forEach((fracture, fIdx) => {
            const base = fractureBase + fIdx * bondStructSize;
            commandView.setUint32(base, fracture.userdata >>> 0, true);
            commandView.setUint32(base + 4, fracture.nodeIndex0 >>> 0, true);
            commandView.setUint32(base + 8, fracture.nodeIndex1 >>> 0, true);
            commandView.setFloat32(base + 12, fracture.health ?? 0, true);
          });
          bondOffset += fractures.length;
        });

        commandView.setUint32(eventCountPtr, 0, true);
        commandView.setUint32(childCountPtr, 0, true);
        commandView.setUint32(nodeCountPtr, 0, true);

        status = this.module.ccall(
          'ext_stress_solver_apply_fracture_commands',
          'number',
          ['number', 'number', 'number', 'number', 'number', 'number', 'number', 'number', 'number', 'number', 'number', 'number'],
          [
            this.handle,
            commandPtr,
            fractureSets.length >>> 0,
            splitPtr,
            fractureSets.length >>> 0,
            childPtr,
            childCapacity >>> 0,
            eventCountPtr,
            childCountPtr,
            nodesPtr,
            nodeCapacity >>> 0,
            nodeCountPtr
          ]
        );

        const eventCount = commandView.getUint32(eventCountPtr, true);
        const parsedEvents: SplitEvent[] = [];
        const heapU32 = this.module.HEAPU32 as Uint32Array;
        for (let i = 0; i < eventCount; ++i) {
          const base = splitPtr + i * splitEventSize;
          const parentActorIndex = commandView.getUint32(base, true);
          const childCountForEvent = commandView.getUint32(base + 8, true);
          const childAddress = commandView.getUint32(base + 4, true);
          const children: Array<{ actorIndex: number; nodes: number[] }> = [];
          if (childAddress) {
            for (let c = 0; c < childCountForEvent; ++c) {
              const actorBase = childAddress + c * actorStructSize;
              const actorIndex = commandView.getUint32(actorBase, true);
              const nodesAddress = commandView.getUint32(actorBase + 4, true);
              const nodeCount = commandView.getUint32(actorBase + 8, true);
              const nodes: number[] = [];
              if (nodesAddress) {
                const nodeOffset = nodesAddress >>> 2;
                for (let n = 0; n < nodeCount; ++n) {
                  nodes.push((heapU32 as any)[nodeOffset + n]);
                }
              }
              children.push({ actorIndex, nodes });
            }
          }
          parsedEvents.push({ parentActorIndex, children });
        }

        if (status !== 2) {
          splitEvents = parsedEvents;
        }
      } finally {
        this.memory.free(commandPtr);
        this.memory.free(bondPtr);
        this.memory.free(splitPtr);
        this.memory.free(childPtr);
        this.memory.free(nodesPtr);
        this.memory.free(eventCountPtr);
        this.memory.free(childCountPtr);
        this.memory.free(nodeCountPtr);
      }

      if (status === 2) {
        attempts += 1;
        childCapacity = Math.min(baseCapacity * 4, childCapacity * 2);
        nodeCapacity = Math.min(baseCapacity * 4, nodeCapacity * 2);
      } else {
        break;
      }
    } while (attempts < maxAttempts);

    if (status === 2) {
      // eslint-disable-next-line no-console
      console.warn('[ExtStressSolver] applyFractureCommands: output truncated; consider enlarging buffers.');
    }
    return splitEvents;
  }

  getExcessForces(actorIndex: number, centerOfMass: Vec3 = vec3()) {
    if (!this.handle || !this._forcePtr || !this._torquePtr) return null;
    const comPtr = this.memory.alloc(this.sizes.vec3);
    try {
      writeVec3(this.memory.view(), comPtr, centerOfMass ?? vec3());
      const ok = this.module.ccall('ext_stress_solver_get_excess_forces', 'number', ['number', 'number', 'number', 'number', 'number'], [this.handle, actorIndex >>> 0, comPtr, this._forcePtr, this._torquePtr]);
      if (!ok) return null;
      const view = this.memory.view();
      const force = readVec3(view, this._forcePtr);
      const torque = readVec3(view, this._torquePtr);
      return { force, torque };
    } finally {
      this.memory.free(comPtr);
    }
  }

  fillDebugRender({ mode = ExtDebugMode.Max as ExtDebugModeValue, scale = 1.0 }: { mode?: ExtDebugModeValue; scale?: number } = {}): ExtStressDebugLine[] {
    if (!this.handle || !this._debugPtr) return [];
    const capacity = this._debugCapacity;
    const count = this.module.ccall('ext_stress_solver_fill_debug_render', 'number', ['number', 'number', 'number', 'number', 'number'], [this.handle, mode >>> 0, scale, this._debugPtr, capacity]);
    const result: ExtStressDebugLine[] = [];
    if (count <= 0) return result;
    const view = this.memory.view();
    for (let i = 0; i < count; ++i) {
      const base = this._debugPtr + i * this.sizes.extDebugLine;
      result.push(readExtDebugLine(view, base));
    }
    return result;
  }

  stressError() {
    if (!this.handle) return { lin: 0.0, ang: 0.0 };
    const lin = this.module.ccall('ext_stress_solver_get_linear_error', 'number', ['number'], [this.handle]);
    const ang = this.module.ccall('ext_stress_solver_get_angular_error', 'number', ['number'], [this.handle]);
    return { lin, ang };
  }

  converged() { if (!this.handle) return false; return this.module.ccall('ext_stress_solver_converged', 'number', ['number'], [this.handle]) !== 0; }
}

function writeNode(view: DataView, base: number, node: { com?: Vec3; mass?: number; inertia?: number }) {
  writeVec3(view, base, node.com ?? vec3());
  view.setFloat32(base + 12, node.mass ?? 0.0, true);
  view.setFloat32(base + 16, node.inertia ?? 0.0, true);
}

function readNode(view: DataView, base: number) {
  return { com: readVec3(view, base), mass: view.getFloat32(base + 12, true), inertia: view.getFloat32(base + 16, true) };
}

function writeBond(view: DataView, base: number, bond: { centroid?: Vec3; node0: number; node1: number }) {
  writeVec3(view, base, bond.centroid ?? vec3());
  view.setUint32(base + 12, bond.node0 >>> 0, true);
  view.setUint32(base + 16, bond.node1 >>> 0, true);
}

function readBond(view: DataView, base: number) {
  return { centroid: readVec3(view, base), node0: view.getUint32(base + 12, true), node1: view.getUint32(base + 16, true) };
}

function writeVec3(view: DataView, base: number, value: Vec3) {
  view.setFloat32(base, value.x ?? 0.0, true);
  view.setFloat32(base + 4, value.y ?? 0.0, true);
  view.setFloat32(base + 8, value.z ?? 0.0, true);
}

function readVec3(view: DataView, base: number): Vec3 {
  return vec3(view.getFloat32(base, true), view.getFloat32(base + 4, true), view.getFloat32(base + 8, true));
}

function writeExtNode(view: DataView, base: number, node: ExtStressNodeDesc) {
  writeVec3(view, base, node.centroid ?? vec3());
  view.setFloat32(base + 12, node.mass ?? 0.0, true);
  view.setFloat32(base + 16, node.volume ?? Math.max(node.mass ?? 0.0, 1.0), true);
}

function writeExtBond(view: DataView, base: number, bond: ExtStressBondDesc) {
  writeVec3(view, base, bond.centroid ?? vec3());
  writeVec3(view, base + 12, bond.normal ?? vec3(0.0, 1.0, 0.0));
  view.setFloat32(base + 24, bond.area ?? 1.0, true);
  view.setUint32(base + 28, bond.node0 >>> 0, true);
  view.setUint32(base + 32, bond.node1 >>> 0, true);
}

function readExtBond(view: DataView, base: number): ExtStressBondDesc {
  return {
    centroid: readVec3(view, base),
    normal: readVec3(view, base + 12),
    area: view.getFloat32(base + 24, true),
    node0: view.getUint32(base + 28, true),
    node1: view.getUint32(base + 32, true)
  };
}

function writeExtSettings(view: DataView, base: number, settings: ExtStressSolverSettings) {
  view.setUint32(base, (settings.maxSolverIterationsPerFrame ?? 25) >>> 0, true);
  view.setUint32(base + 4, (settings.graphReductionLevel ?? 0) >>> 0, true);
  view.setFloat32(base + 8, settings.compressionElasticLimit ?? 1.0, true);
  view.setFloat32(base + 12, settings.compressionFatalLimit ?? 2.0, true);
  view.setFloat32(base + 16, settings.tensionElasticLimit ?? -1.0, true);
  view.setFloat32(base + 20, settings.tensionFatalLimit ?? -1.0, true);
  view.setFloat32(base + 24, settings.shearElasticLimit ?? -1.0, true);
  view.setFloat32(base + 28, settings.shearFatalLimit ?? -1.0, true);
}

function readExtDebugLine(view: DataView, base: number): ExtStressDebugLine {
  return { p0: readVec3(view, base), p1: readVec3(view, base + 12), color0: view.getUint32(base + 24, true), color1: view.getUint32(base + 28, true) };
}

function readExtBondFracture(view: DataView, base: number): ExtStressBondFracture {
  return { userdata: view.getUint32(base, true), nodeIndex0: view.getUint32(base + 4, true), nodeIndex1: view.getUint32(base + 8, true), health: view.getFloat32(base + 12, true) };
}

function createImpulse(): StressImpulse { return { ang: vec3(), lin: vec3() }; }
function cloneImpulse(impulse: StressImpulse): StressImpulse { return { ang: vec3(impulse.ang.x, impulse.ang.y, impulse.ang.z), lin: vec3(impulse.lin.x, impulse.lin.y, impulse.lin.z) }; }
function cloneNode(node: { com: Vec3; mass: number; inertia: number }) { return { com: vec3(node.com.x, node.com.y, node.com.z), mass: node.mass, inertia: node.inertia }; }
function cloneBond(bond: { centroid: Vec3; node0: number; node1: number }) { return { centroid: vec3(bond.centroid.x, bond.centroid.y, bond.centroid.z), node0: bond.node0, node1: bond.node1 }; }

function resolveLimit(limit: number, fallback: number) { if (limit > 0.0) return limit; if (fallback > 0.0) return fallback; return 1.0; }
function mapStressValue(stress: number, elastic: number, fatal: number) {
  if (stress <= 0.0) return 0.0;
  const elasticResolved = elastic > 0.0 ? elastic : fatal;
  const fatalResolved = fatal > 0.0 ? fatal : Math.max(elasticResolved, 1.0);
  if (elasticResolved > 0.0 && stress < elasticResolved) return clamp((stress / elasticResolved) * 0.5, 0.0, 0.5);
  if (fatalResolved > elasticResolved && elasticResolved > 0.0) return clamp(0.5 + 0.5 * (stress - elasticResolved) / (fatalResolved - elasticResolved), 0.5, 1.0);
  return clamp(stress / fatalResolved, 0.0, 1.0);
}
function clamp(value: number, min: number, max: number) { return Math.min(Math.max(value, min), max); }

function generateBondsFromTriangles(
  module: any,
  memory: ModuleMemory,
  sizes: RuntimeSizes,
  chunks: AuthoringChunkInput[],
  config?: BondingConfig
): ExtStressBondDesc[] {
  if (!Array.isArray(chunks) || chunks.length === 0) {
    return [];
  }

  const { offsets, triangles } = flattenTriangleChunks(chunks);
  const supportFlags = new Uint8Array(chunks.length);
  for (let i = 0; i < chunks.length; ++i) {
    const chunk = chunks[i];
    supportFlags[i] = chunk?.isSupport === undefined ? 1 : chunk.isSupport ? 1 : 0;
  }

  const geometryOffsetPtr = memory.alloc(offsets.byteLength);
  const trianglesPtr = memory.alloc(triangles.byteLength);
  const supportPtr = memory.alloc(Math.max(chunks.length, 1));
  const outPtrPtr = memory.alloc(4);

  module.HEAPU32.set(offsets, geometryOffsetPtr >>> 2);
  module.HEAPF32.set(triangles, trianglesPtr >>> 2);
  if (chunks.length > 0) {
    module.HEAPU8.set(supportFlags, supportPtr);
  }
  module.HEAPU32[outPtrPtr >>> 2] = 0;

  const bondMode = config?.mode === 'average' ? 1 : 0;
  const maxSeparation = typeof config?.maxSeparation === 'number' ? config.maxSeparation : 0.0;

  let bondCount = 0;
  try {
    bondCount = module.ccall(
      'authoring_bonds_from_prefractured_triangles',
      'number',
      ['number', 'number', 'number', 'number', 'number', 'number', 'number', 'number'],
      [chunks.length, geometryOffsetPtr, trianglesPtr, triangles.length, supportPtr, bondMode, maxSeparation, outPtrPtr]
    );
  } finally {
    memory.free(geometryOffsetPtr);
    memory.free(trianglesPtr);
    memory.free(supportPtr);
  }

  const outPtr = module.HEAPU32[outPtrPtr >>> 2];
  memory.free(outPtrPtr);

  if (!bondCount || !outPtr) {
    if (outPtr) {
      module.ccall('authoring_free', null, ['number'], [outPtr]);
    }
    return [];
  }

  const structSize =
    sizes.authoringBond && sizes.authoringBond > 0
      ? sizes.authoringBond
      : module.ccall('authoring_sizeof_ext_bond_desc', 'number', [], []);
  const view = memory.view();
  const bonds: ExtStressBondDesc[] = [];
  for (let i = 0; i < bondCount; ++i) {
    bonds.push(readExtBond(view, outPtr + i * structSize));
  }

  module.ccall('authoring_free', null, ['number'], [outPtr]);
  return bonds;
}

function flattenTriangleChunks(chunks: AuthoringChunkInput[]) {
  const chunkCount = chunks.length;
  const offsets = new Uint32Array(chunkCount + 1);
  const normalized: Float32Array[] = new Array(chunkCount);
  let totalFloats = 0;

  for (let i = 0; i < chunkCount; ++i) {
    const chunk = normalizeTriangleChunk(chunks[i]?.triangles);
    if (chunk.length % 9 !== 0) {
      throw new Error('Triangle buffers must contain multiples of 9 floats');
    }
    offsets[i + 1] = offsets[i] + chunk.length / 9;
    normalized[i] = chunk;
    totalFloats += chunk.length;
  }

  const triangles = new Float32Array(totalFloats);
  let cursor = 0;
  for (let i = 0; i < chunkCount; ++i) {
    const chunk = normalized[i];
    triangles.set(chunk, cursor);
    cursor += chunk.length;
  }

  return { offsets, triangles };
}

function normalizeTriangleChunk(chunk: Float32Array | ReadonlyArray<number>): Float32Array {
  if (!chunk) {
    throw new Error('Each chunk must provide triangle data');
  }
  if (chunk instanceof Float32Array) {
    return chunk;
  }
  if (Array.isArray(chunk) || (typeof chunk === 'object' && 'length' in chunk)) {
    return Float32Array.from(chunk as ArrayLike<number>);
  }
  throw new Error('Triangle chunk must be a Float32Array or array of numbers');
}

export function chunkFromBufferGeometry(
  geometry: BufferGeometry,
  options: BufferGeometryChunkOptions = {}
): AuthoringChunkInput {
  if (!geometry) {
    throw new Error('chunkFromBufferGeometry requires a BufferGeometry instance');
  }

  const { isSupport, applyMatrix, nonIndexed = true, cloneGeometry = true } = options;
  let working = cloneGeometry ? geometry.clone() : geometry;

  if (nonIndexed !== false && working.index) {
    working = working.toNonIndexed();
  }

  if (applyMatrix) {
    const apply = (working as any).applyMatrix4;
    if (typeof apply === 'function') {
      apply.call(working, applyMatrix as Matrix4);
    } else {
      throw new Error('BufferGeometry.applyMatrix4 is unavailable in this environment');
    }
  }

  const position = working.getAttribute('position');
  if (!position) {
    throw new Error('BufferGeometry is missing a position attribute');
  }

  const triangles = Float32Array.from(position.array as ArrayLike<number>);
  if (cloneGeometry && working !== geometry) {
    working.dispose?.();
  }

  return { triangles, isSupport };
}

export function chunksFromBufferGeometries(
  geometries: BufferGeometry[],
  options?: BufferGeometryChunkResolver
): AuthoringChunkInput[] {
  if (!Array.isArray(geometries) || geometries.length === 0) {
    return [];
  }
  return geometries.map((geometry, index) => {
    const resolvedOptions =
      typeof options === 'function' ? options(geometry, index) ?? {} : options ?? {};
    return chunkFromBufferGeometry(geometry, resolvedOptions);
  });
}


