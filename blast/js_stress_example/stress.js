const isNode = typeof process !== 'undefined' && process.release?.name === 'node';

const moduleFactoryPromise = (async () => {
  if (isNode) {
    const factoryModule = await import('./dist/stress_solver.cjs');
    return factoryModule.default ?? factoryModule;
  }
  const factoryModule = await import('./dist/stress_solver.mjs');
  return factoryModule.default ?? factoryModule;
})();

const distDirUrl = new URL('./dist/', import.meta.url);

let fileURLToPathFn;

export const StressFailure = Object.freeze({
  Compression: 'compression',
  Tension: 'tension',
  Shear: 'shear'
});

export const ExtForceMode = Object.freeze({
  Force: 0,
  Acceleration: 1
});

export const ExtDebugMode = Object.freeze({
  Max: 0,
  Compression: 1,
  Tension: 2,
  Shear: 3
});

export function vec3(x = 0.0, y = 0.0, z = 0.0) {
  return { x, y, z };
}

export function formatNumber(value, width, precision) {
  return value.toFixed(precision).padStart(width);
}

export function formatVec3(v, width = 6, precision = 2) {
  return `(${formatNumber(v.x, width, precision)}, ${formatNumber(v.y, width, precision)}, ${formatNumber(v.z, width, precision)})`;
}

export class StressLimits {
  constructor({
    compressionElasticLimit = 1.0,
    compressionFatalLimit = 2.0,
    tensionElasticLimit = -1.0,
    tensionFatalLimit = -1.0,
    shearElasticLimit = -1.0,
    shearFatalLimit = -1.0
  } = {}) {
    this.compressionElasticLimit = compressionElasticLimit;
    this.compressionFatalLimit = compressionFatalLimit;
    this.tensionElasticLimit = tensionElasticLimit;
    this.tensionFatalLimit = tensionFatalLimit;
    this.shearElasticLimit = shearElasticLimit;
    this.shearFatalLimit = shearFatalLimit;
  }

  severity(stress) {
    return {
      compression: mapStressValue(
        stress.compression,
        this.#compressionElastic(),
        this.#compressionFatal()
      ),
      tension: mapStressValue(
        stress.tension,
        this.#tensionElastic(),
        this.#tensionFatal()
      ),
      shear: mapStressValue(stress.shear, this.#shearElastic(), this.#shearFatal())
    };
  }

  failureMode(stress) {
    if (stress.compression > this.#compressionFatal()) {
      return StressFailure.Compression;
    }
    if (stress.tension > this.#tensionFatal()) {
      return StressFailure.Tension;
    }
    if (stress.shear > this.#shearFatal()) {
      return StressFailure.Shear;
    }
    return null;
  }

  fatalThreshold(mode) {
    switch (mode) {
      case StressFailure.Compression:
        return this.#compressionFatal();
      case StressFailure.Tension:
        return this.#tensionFatal();
      case StressFailure.Shear:
        return this.#shearFatal();
      default:
        return 0.0;
    }
  }

  compressionElasticThreshold() {
    return this.#compressionElastic();
  }

  compressionFatalThreshold() {
    return this.#compressionFatal();
  }

  tensionElasticThreshold() {
    return this.#tensionElastic();
  }

  tensionFatalThreshold() {
    return this.#tensionFatal();
  }

  shearElasticThreshold() {
    return this.#shearElastic();
  }

  shearFatalThreshold() {
    return this.#shearFatal();
  }

  #compressionElastic() {
    return resolveLimit(this.compressionElasticLimit, 1.0);
  }

  #compressionFatal() {
    return resolveLimit(this.compressionFatalLimit, this.#compressionElastic());
  }

  #tensionElastic() {
    return resolveLimit(this.tensionElasticLimit, this.#compressionElastic());
  }

  #tensionFatal() {
    return resolveLimit(this.tensionFatalLimit, this.#compressionFatal());
  }

  #shearElastic() {
    return resolveLimit(this.shearElasticLimit, this.#compressionElastic());
  }

  #shearFatal() {
    return resolveLimit(this.shearFatalLimit, this.#compressionFatal());
  }
}

export async function loadStressSolver({ module: moduleOptions } = {}) {
  if (isNode && !fileURLToPathFn) {
    const urlModule = await import('node:url');
    fileURLToPathFn = urlModule.fileURLToPath;
  }

  const factory = await moduleFactoryPromise;
  const options = { ...(moduleOptions ?? {}) };
  if (!options.locateFile) {
    options.locateFile = (path) => {
      const url = new URL(path, distDirUrl);
      return isNode ? fileURLToPathFn(url) : url.href;
    };
  }
  if (isNode && !options.wasmBinary) {
    const fs = await import('node:fs/promises');
    const wasmUrl = new URL('stress_solver.wasm', distDirUrl);
    options.wasmBinary = await fs.readFile(fileURLToPathFn(wasmUrl));
  }
  options.print ??= (...args) => console.log('[blast-wasm]', ...args);
  options.printErr ??= (...args) => console.error('[blast-wasm]', ...args);

  const module = await factory(options);
  return createRuntime(module);
}

export function computeBondStress(bond, impulse, nodes, bondArea) {
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

export function subtract(a, b) {
  return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

export function dot(a, b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

export function magnitudeSquared(v) {
  return dot(v, v);
}

export function magnitude(v) {
  return Math.sqrt(magnitudeSquared(v));
}

export function normalize(v) {
  const mag = magnitude(v);
  if (mag <= 1.0e-6) {
    return vec3();
  }
  return vec3(v.x / mag, v.y / mag, v.z / mag);
}

function createRuntime(module) {
  const sizes = {
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
    extDebugLine: module.ccall('ext_stress_sizeof_ext_debug_line', 'number', [], [])
  };

  const memory = new ModuleMemory(module);

  return {
    module,
    sizes,
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
    createProcessor(description) {
      return new StressProcessor(module, memory, sizes, description);
    },
    createExtSolver(description) {
      return new ExtStressSolver(module, memory, sizes, description);
    }
  };
}

class ModuleMemory {
  constructor(module) {
    this.module = module;
    this.dataView = new DataView(module.HEAPU8.buffer);
  }

  view() {
    if (this.dataView.buffer !== this.module.HEAPU8.buffer) {
      this.dataView = new DataView(this.module.HEAPU8.buffer);
    }
    return this.dataView;
  }

  alloc(size) {
    const ptr = this.module._malloc(size);
    if (ptr === 0) {
      throw new Error('Stress solver allocation failed');
    }
    this.view();
    return ptr;
  }

  free(ptr) {
    if (!ptr) {
      return;
    }
    this.module._free(ptr);
  }

  zero(ptr, size) {
    this.module.HEAPU8.fill(0, ptr, ptr + size);
  }
}

class StressProcessor {
  constructor(module, memory, sizes, description) {
    if (!description) {
      throw new Error('StressProcessor description is required');
    }
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

    const dataParams = description.dataParams ?? {};
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

    if (!handle) {
      throw new Error('Failed to create StressProcessor');
    }

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
    this._solverParams = {
      maxIterations: 32,
      tolerance: 1.0e-6,
      warmStart: false
    };
    this._writeSolverParams(this._solverParams);
  }

  getNodes() {
    return this._nodes.map(cloneNode);
  }

  getBonds() {
    this._bonds = this._fetchAllBonds();
    return this._bonds.map(cloneBond);
  }

  internalBondDesc(index) {
    return this._bonds[index];
  }

  getImpulses() {
    return this._impulses.map(cloneImpulse);
  }

  setImpulses(impulses) {
    if (!Array.isArray(impulses) || impulses.length !== this.bondCount) {
      throw new Error('Impulse array must match bond count');
    }
    this._impulses = impulses.map(cloneImpulse);
  }

  setSolverParams(params) {
    this._solverParams = {
      maxIterations: params?.maxIterations ?? this._solverParams.maxIterations,
      tolerance: params?.tolerance ?? this._solverParams.tolerance,
      warmStart: params?.warmStart ?? this._solverParams.warmStart
    };
    this._writeSolverParams(this._solverParams);
  }

  nodeDesc(index) {
    if (index < 0 || index >= this.nodeCount) {
      throw new Error('Node index out of range');
    }
    const ok = this.module.ccall(
      'stress_processor_get_node_desc',
      'number',
      ['number', 'number', 'number'],
      [this.handle, index, this.nodeScratchPtr]
    );
    if (!ok) {
      throw new Error(`Failed to read node descriptor ${index}`);
    }
    const view = this.memory.view();
    return readNode(view, this.nodeScratchPtr);
  }

  bondDesc(index) {
    if (index < 0 || index >= this.bondCount) {
      throw new Error('Bond index out of range');
    }
    const ok = this.module.ccall(
      'stress_processor_get_bond_desc',
      'number',
      ['number', 'number', 'number'],
      [this.handle, index, this.bondScratchPtr]
    );
    if (!ok) {
      throw new Error(`Failed to read bond descriptor ${index}`);
    }
    const view = this.memory.view();
    return readBond(view, this.bondScratchPtr);
  }

  solve({ velocities, solverParams, resume = false } = {}) {
    if (!Array.isArray(velocities) || velocities.length !== this.nodeCount) {
      throw new Error('Velocity array must match node count');
    }
    if (solverParams) {
      this.setSolverParams(solverParams);
    }

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
      [
        this.handle,
        this.impulsesPtr,
        this.velocitiesPtr,
        this.solverParamsPtr,
        this.errorPtr,
        resume ? 1 : 0
      ]
    );

    if (iterations < 0) {
      throw new Error('Stress solver reported an error');
    }

    this._impulses = this._readImpulses();
    const errorView = this.memory.view();
    const angularError = Math.sqrt(errorView.getFloat32(this.errorPtr, true));
    const linearError = Math.sqrt(errorView.getFloat32(this.errorPtr + 4, true));

    return {
      iterations,
      error: { ang: angularError, lin: linearError },
      impulses: this.getImpulses()
    };
  }

  removeBond(index) {
    if (index < 0 || index >= this.bondCount) {
      return false;
    }
    const removed = this.module.ccall(
      'stress_processor_remove_bond',
      'number',
      ['number', 'number'],
      [this.handle, index]
    );
    if (!removed) {
      return false;
    }

    this.bondCount = this.module.ccall(
      'stress_processor_bond_count',
      'number',
      ['number'],
      [this.handle]
    );

    this.memory.free(this.impulsesPtr);
    this.impulsesPtr = this.memory.alloc(this.sizes.impulse * this.bondCount);
    this._impulses = Array.from({ length: this.bondCount }, () => createImpulse());
    this._bonds = this._fetchAllBonds();
    return true;
  }

  destroy() {
    if (!this.handle) {
      return;
    }
    this.module.ccall('stress_processor_destroy', 'void', ['number'], [this.handle]);
    this.handle = 0;

    this.memory.free(this.nodeScratchPtr);
    this.memory.free(this.bondScratchPtr);
    this.memory.free(this.velocitiesPtr);
    this.memory.free(this.impulsesPtr);
    this.memory.free(this.errorPtr);
    this.memory.free(this.solverParamsPtr);
  }

  _fetchAllNodes() {
    const nodes = [];
    for (let i = 0; i < this.nodeCount; ++i) {
      nodes.push(this.nodeDesc(i));
    }
    return nodes;
  }

  _fetchAllBonds() {
    const bonds = [];
    for (let i = 0; i < this.bondCount; ++i) {
      bonds.push(this.bondDesc(i));
    }
    return bonds;
  }

  _readImpulses() {
    const impulses = [];
    const view = this.memory.view();
    for (let i = 0; i < this.bondCount; ++i) {
      const base = this.impulsesPtr + i * this.sizes.impulse;
      impulses.push({
        ang: readVec3(view, base),
        lin: readVec3(view, base + this.sizes.vec3)
      });
    }
    return impulses;
  }

  _writeSolverParams(params) {
    const view = this.memory.view();
    view.setUint32(this.solverParamsPtr, params.maxIterations >>> 0, true);
    view.setFloat32(this.solverParamsPtr + 4, params.tolerance, true);
    view.setUint8(this.solverParamsPtr + 8, params.warmStart ? 1 : 0);
    view.setUint8(this.solverParamsPtr + 9, 0);
    view.setUint8(this.solverParamsPtr + 10, 0);
    view.setUint8(this.solverParamsPtr + 11, 0);
  }
}

class ExtStressSolver {
  constructor(module, memory, sizes, description) {
    if (!description) {
      throw new Error('ExtStressSolver description is required');
    }

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

      if (settingsPtr) {
        writeExtSettings(view, settingsPtr, description.settings);
      }

      const handle = module.ccall(
        'ext_stress_solver_create',
        'number',
        ['number', 'number', 'number', 'number', 'number'],
        [nodesPtr, this.nodeCount, bondsPtr, this.bondCount, settingsPtr]
      );

      if (!handle) {
        throw new Error('Failed to create ExtStressSolver');
      }

      this.handle = handle >>> 0;
      this._debugCapacity = Math.max(this.bondCount, 1);
      this._debugPtr = memory.alloc(this._debugCapacity * sizes.extDebugLine);
    } finally {
      memory.free(nodesPtr);
      memory.free(bondsPtr);
      if (settingsPtr) {
        memory.free(settingsPtr);
      }
    }
  }

  destroy() {
    if (!this.handle) {
      return;
    }
    this.module.ccall('ext_stress_solver_destroy', null, ['number'], [this.handle]);
    this.handle = 0;
    if (this._debugPtr) {
      this.memory.free(this._debugPtr);
      this._debugPtr = 0;
    }
  }

  setSettings(settings) {
    if (!settings || !this.handle) {
      return;
    }
    const ptr = this.memory.alloc(this.sizes.extSettings);
    try {
      writeExtSettings(this.memory.view(), ptr, settings);
      this.module.ccall('ext_stress_solver_set_settings', null, ['number', 'number'], [this.handle, ptr]);
    } finally {
      this.memory.free(ptr);
    }
  }

  graphNodeCount() {
    if (!this.handle) {
      return 0;
    }
    return this.module.ccall('ext_stress_solver_graph_node_count', 'number', ['number'], [this.handle]) >>> 0;
  }

  bondCapacity() {
    return this.bondCount;
  }

  reset() {
    if (!this.handle) {
      return;
    }
    this.module.ccall('ext_stress_solver_reset', null, ['number'], [this.handle]);
  }

  addForce(nodeIndex, localPosition, localForce, mode = ExtForceMode.Force) {
    if (!this.handle) {
      return;
    }
    const positionPtr = this.memory.alloc(this.sizes.vec3);
    const forcePtr = this.memory.alloc(this.sizes.vec3);
    try {
      const view = this.memory.view();
      writeVec3(view, positionPtr, localPosition ?? vec3());
      writeVec3(view, forcePtr, localForce ?? vec3());
      this.module.ccall(
        'ext_stress_solver_add_force',
        null,
        ['number', 'number', 'number', 'number', 'number'],
        [this.handle, nodeIndex >>> 0, positionPtr, forcePtr, mode >>> 0]
      );
    } finally {
      this.memory.free(positionPtr);
      this.memory.free(forcePtr);
    }
  }

  addGravity(localGravity) {
    if (!this.handle) {
      return;
    }
    const gravityPtr = this.memory.alloc(this.sizes.vec3);
    try {
      writeVec3(this.memory.view(), gravityPtr, localGravity ?? vec3());
      this.module.ccall('ext_stress_solver_add_gravity', null, ['number', 'number'], [this.handle, gravityPtr]);
    } finally {
      this.memory.free(gravityPtr);
    }
  }

  update() {
    if (!this.handle) {
      return;
    }
    this.module.ccall('ext_stress_solver_update', null, ['number'], [this.handle]);
  }

  overstressedBondCount() {
    if (!this.handle) {
      return 0;
    }
    return this.module.ccall('ext_stress_solver_overstressed_bond_count', 'number', ['number'], [this.handle]) >>> 0;
  }

  fillDebugRender({ mode = ExtDebugMode.Max, scale = 1.0 } = {}) {
    if (!this.handle || !this._debugPtr) {
      return [];
    }
    const capacity = this._debugCapacity;
    const count = this.module.ccall(
      'ext_stress_solver_fill_debug_render',
      'number',
      ['number', 'number', 'number', 'number', 'number'],
      [this.handle, mode >>> 0, scale, this._debugPtr, capacity]
    );
    const result = [];
    if (count <= 0) {
      return result;
    }
    const view = this.memory.view();
    for (let i = 0; i < count; ++i) {
      const base = this._debugPtr + i * this.sizes.extDebugLine;
      result.push(readExtDebugLine(view, base));
    }
    return result;
  }

  stressError() {
    if (!this.handle) {
      return { lin: 0.0, ang: 0.0 };
    }
    const lin = this.module.ccall('ext_stress_solver_get_linear_error', 'number', ['number'], [this.handle]);
    const ang = this.module.ccall('ext_stress_solver_get_angular_error', 'number', ['number'], [this.handle]);
    return { lin, ang };
  }

  converged() {
    if (!this.handle) {
      return false;
    }
    return this.module.ccall('ext_stress_solver_converged', 'number', ['number'], [this.handle]) !== 0;
  }
}

function writeNode(view, base, node) {
  writeVec3(view, base, node.com ?? vec3());
  view.setFloat32(base + 12, node.mass ?? 0.0, true);
  view.setFloat32(base + 16, node.inertia ?? 0.0, true);
}

function readNode(view, base) {
  return {
    com: readVec3(view, base),
    mass: view.getFloat32(base + 12, true),
    inertia: view.getFloat32(base + 16, true)
  };
}

function writeBond(view, base, bond) {
  writeVec3(view, base, bond.centroid ?? vec3());
  view.setUint32(base + 12, bond.node0 >>> 0, true);
  view.setUint32(base + 16, bond.node1 >>> 0, true);
}

function readBond(view, base) {
  return {
    centroid: readVec3(view, base),
    node0: view.getUint32(base + 12, true),
    node1: view.getUint32(base + 16, true)
  };
}

function writeVec3(view, base, value) {
  view.setFloat32(base, value.x ?? 0.0, true);
  view.setFloat32(base + 4, value.y ?? 0.0, true);
  view.setFloat32(base + 8, value.z ?? 0.0, true);
}

function readVec3(view, base) {
  return vec3(view.getFloat32(base, true), view.getFloat32(base + 4, true), view.getFloat32(base + 8, true));
}

function writeExtNode(view, base, node) {
  writeVec3(view, base, node.centroid ?? vec3());
  view.setFloat32(base + 12, node.mass ?? 0.0, true);
  view.setFloat32(base + 16, node.volume ?? Math.max(node.mass ?? 0.0, 1.0), true);
}

function writeExtBond(view, base, bond) {
  writeVec3(view, base, bond.centroid ?? vec3());
  writeVec3(view, base + 12, bond.normal ?? vec3(0.0, 1.0, 0.0));
  view.setFloat32(base + 24, bond.area ?? 1.0, true);
  view.setUint32(base + 28, bond.node0 >>> 0, true);
  view.setUint32(base + 32, bond.node1 >>> 0, true);
}

function writeExtSettings(view, base, settings) {
  view.setUint32(base, settings.maxSolverIterationsPerFrame >>> 0, true);
  view.setUint32(base + 4, settings.graphReductionLevel >>> 0, true);
  view.setFloat32(base + 8, settings.compressionElasticLimit ?? 1.0, true);
  view.setFloat32(base + 12, settings.compressionFatalLimit ?? 2.0, true);
  view.setFloat32(base + 16, settings.tensionElasticLimit ?? -1.0, true);
  view.setFloat32(base + 20, settings.tensionFatalLimit ?? -1.0, true);
  view.setFloat32(base + 24, settings.shearElasticLimit ?? -1.0, true);
  view.setFloat32(base + 28, settings.shearFatalLimit ?? -1.0, true);
}

function readExtDebugLine(view, base) {
  return {
    p0: readVec3(view, base),
    p1: readVec3(view, base + 12),
    color0: view.getUint32(base + 24, true),
    color1: view.getUint32(base + 28, true)
  };
}

function createImpulse() {
  return { ang: vec3(), lin: vec3() };
}

function cloneImpulse(impulse) {
  return {
    ang: vec3(impulse.ang.x, impulse.ang.y, impulse.ang.z),
    lin: vec3(impulse.lin.x, impulse.lin.y, impulse.lin.z)
  };
}

function cloneNode(node) {
  return {
    com: vec3(node.com.x, node.com.y, node.com.z),
    mass: node.mass,
    inertia: node.inertia
  };
}

function cloneBond(bond) {
  return {
    centroid: vec3(bond.centroid.x, bond.centroid.y, bond.centroid.z),
    node0: bond.node0,
    node1: bond.node1
  };
}

function resolveLimit(limit, fallback) {
  if (limit > 0.0) {
    return limit;
  }
  if (fallback > 0.0) {
    return fallback;
  }
  return 1.0;
}

function mapStressValue(stress, elastic, fatal) {
  if (stress <= 0.0) {
    return 0.0;
  }
  const elasticResolved = elastic > 0.0 ? elastic : fatal;
  const fatalResolved = fatal > 0.0 ? fatal : Math.max(elasticResolved, 1.0);
  if (elasticResolved > 0.0 && stress < elasticResolved) {
    return clamp((stress / elasticResolved) * 0.5, 0.0, 0.5);
  }
  if (fatalResolved > elasticResolved && elasticResolved > 0.0) {
    return clamp(0.5 + 0.5 * (stress - elasticResolved) / (fatalResolved - elasticResolved), 0.5, 1.0);
  }
  return clamp(stress / fatalResolved, 0.0, 1.0);
}

function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}


