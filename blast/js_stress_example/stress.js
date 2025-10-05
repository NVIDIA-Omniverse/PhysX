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

  scaledBy(factor) {
    return new StressLimits({
      compressionElasticLimit: this.compressionElasticLimit * factor,
      compressionFatalLimit: this.compressionFatalLimit * factor,
      tensionElasticLimit: this.tensionElasticLimit * factor,
      tensionFatalLimit: this.tensionFatalLimit * factor,
      shearElasticLimit: this.shearElasticLimit * factor,
      shearFatalLimit: this.shearFatalLimit * factor
    });
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
    errorSq: module.ccall('stress_sizeof_error_sq', 'number', [], [])
  };

  const memory = new ModuleMemory(module);

  return {
    module,
    sizes,
    vec3,
    StressLimits,
    StressFailure,
    computeBondStress,
    defaultSolverParams: () => ({ maxIterations: 32, tolerance: 1.0e-6, warmStart: false }),
    createProcessor(description) {
      return new StressProcessor(module, memory, sizes, description);
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
    this._pendingForces = Array.from({ length: this.nodeCount }, () => ({ force: vec3(), torque: vec3() }));
    this._forceScratchPtr = memory.alloc(sizes.velocity);
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
      const pending = this._pendingForces[index];
      if (pending && (pending.force.x || pending.force.y || pending.force.z || pending.torque.x || pending.torque.y || pending.torque.z)) {
        const applied = this._pendingForces[index];
        const angVec = addVec3(velocity.ang ?? vec3(), applied.torque);
        const linVec = addVec3(velocity.lin ?? vec3(), applied.force);
        if (typeof console !== 'undefined' && console.debug) {
          console.debug('[stress][pendingForce]', { index, applied });
          console.debug('[stress][velocityWithForce]', { index, angVec, linVec });
        }
        writeVec3(view, base, angVec);
        writeVec3(view, base + this.sizes.vec3, linVec);
        applied.force = vec3();
        applied.torque = vec3();
      } else {
        writeVec3(view, base, velocity.ang ?? vec3());
        writeVec3(view, base + this.sizes.vec3, velocity.lin ?? vec3());
      }
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
      const errorView = this.memory.view();
      const angularErrorSq = errorView.getFloat32(this.errorPtr, true);
      const linearErrorSq = errorView.getFloat32(this.errorPtr + 4, true);
      const angularError = Number.isFinite(angularErrorSq) ? Math.sqrt(Math.max(angularErrorSq, 0)) : Number.NaN;
      const linearError = Number.isFinite(linearErrorSq) ? Math.sqrt(Math.max(linearErrorSq, 0)) : Number.NaN;
      const message = `Stress solver failed (code ${iterations}) linear=${linearError.toExponential(3)} angular=${angularError.toExponential(3)}`;
      console.error(message, {
        iterations,
        linearError,
        angularError,
        solverParams: this._solverParams,
        velocitySample: velocities.slice(0, 4)
      });
      throw new Error(message);
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

  accumulateForce(nodeIndex, localPos, localForce) {
    if (nodeIndex < 0 || nodeIndex >= this.nodeCount) {
      return;
    }
    const entry = this._pendingForces[nodeIndex];
    entry.force.x += localForce.x;
    entry.force.y += localForce.y;
    entry.force.z += localForce.z;
    entry.torque.x += localPos.y * localForce.z - localPos.z * localForce.y;
    entry.torque.y += localPos.z * localForce.x - localPos.x * localForce.z;
    entry.torque.z += localPos.x * localForce.y - localPos.y * localForce.x;
  }
}

function writeNode(view, base, node) {
  writeVec3(view, base, node.com ?? vec3());
  view.setFloat32(base + 12, node.mass ?? 0.0, true);
  writeVec3(view, base + 16, node.inertia ?? vec3());
}

function readNode(view, base) {
  return {
    com: readVec3(view, base),
    mass: view.getFloat32(base + 12, true),
    inertia: readVec3(view, base + 16)
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
    inertia: vec3(node.inertia.x, node.inertia.y, node.inertia.z)
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

function addVec3(a, b) {
  return vec3(
    (a?.x ?? 0) + (b?.x ?? 0),
    (a?.y ?? 0) + (b?.y ?? 0),
    (a?.z ?? 0) + (b?.z ?? 0)
  );
}


