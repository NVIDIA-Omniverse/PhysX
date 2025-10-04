import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);
const createStressModule = require('./dist/stress_solver.cjs');

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const wasmPath = resolve(__dirname, 'dist', 'stress_solver.wasm');

const TOP_NODE = 2;
const BOND_NAMES = ['left diagonal', 'right diagonal', 'base tie'];
const BOND_AREAS = [0.6, 0.6, 0.9];

const FRAME_INPUTS = [
  { lin: vec3(0.0, -24.0, 0.0), ang: vec3(0.0, 0.0, 3.0) },
  { lin: vec3(8.0, -42.0, 0.0), ang: vec3(0.0, 0.0, 7.0) },
  { lin: vec3(14.0, -63.0, 0.0), ang: vec3(0.0, 0.0, 11.5) },
  { lin: vec3(20.0, -90.0, 0.0), ang: vec3(0.0, 0.0, 16.0) }
];

class StressLimits {
  constructor() {
    this.compressionElasticLimit = 320.0;
    this.compressionFatalLimit = 520.0;
    this.tensionElasticLimit = 180.0;
    this.tensionFatalLimit = 320.0;
    this.shearElasticLimit = 140.0;
    this.shearFatalLimit = 260.0;
  }

  #resolve(limit, fallback) {
    if (limit > 0.0) return limit;
    if (fallback > 0.0) return fallback;
    return 1.0;
  }

  #compressionElastic() {
    return this.#resolve(this.compressionElasticLimit, 1.0);
  }

  #compressionFatal() {
    return this.#resolve(this.compressionFatalLimit, this.#compressionElastic());
  }

  #tensionElastic() {
    return this.#resolve(this.tensionElasticLimit, this.#compressionElastic());
  }

  #tensionFatal() {
    return this.#resolve(this.tensionFatalLimit, this.#compressionFatal());
  }

  #shearElastic() {
    return this.#resolve(this.shearElasticLimit, this.#compressionElastic());
  }

  #shearFatal() {
    return this.#resolve(this.shearFatalLimit, this.#compressionFatal());
  }

  #map(stress, elastic, fatal) {
    if (stress <= 0.0) {
      return 0.0;
    }
    const resolvedElastic = elastic > 0.0 ? elastic : fatal;
    const resolvedFatal = fatal > 0.0 ? fatal : Math.max(resolvedElastic, 1.0);
    if (resolvedElastic > 0.0 && stress < resolvedElastic) {
      return clamp((stress / resolvedElastic) * 0.5, 0.0, 0.5);
    }
    if (resolvedFatal > resolvedElastic && resolvedElastic > 0.0) {
      return clamp(0.5 + 0.5 * (stress - resolvedElastic) / (resolvedFatal - resolvedElastic), 0.5, 1.0);
    }
    return clamp(stress / resolvedFatal, 0.0, 1.0);
  }

  severity(stress) {
    return {
      compression: this.#map(stress.compression, this.#compressionElastic(), this.#compressionFatal()),
      tension: this.#map(stress.tension, this.#tensionElastic(), this.#tensionFatal()),
      shear: this.#map(stress.shear, this.#shearElastic(), this.#shearFatal())
    };
  }

  failureMode(stress) {
    if (stress.compression > this.#compressionFatal()) {
      return 'compression';
    }
    if (stress.tension > this.#tensionFatal()) {
      return 'tension';
    }
    if (stress.shear > this.#shearFatal()) {
      return 'shear';
    }
    return null;
  }

  fatalThreshold(mode) {
    switch (mode) {
      case 'compression':
        return this.#compressionFatal();
      case 'tension':
        return this.#tensionFatal();
      case 'shear':
        return this.#shearFatal();
      default:
        return 0.0;
    }
  }
}

function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

function vec3(x, y, z) {
  return { x, y, z };
}

function subtract(a, b) {
  return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

function dot(a, b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

function magnitudeSquared(v) {
  return dot(v, v);
}

function magnitude(v) {
  return Math.sqrt(magnitudeSquared(v));
}

function normalize(v) {
  const mag = magnitude(v);
  if (mag <= 1.0e-6) {
    return vec3(0.0, 0.0, 0.0);
  }
  return vec3(v.x / mag, v.y / mag, v.z / mag);
}

function computeBondStress(bond, impulse, nodes, bondArea) {
  if (bondArea <= 0.0) {
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

  const linear = impulse.lin;
  const angular = impulse.ang;

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

function formatNumber(value, width, precision) {
  return value.toFixed(precision).padStart(width);
}

function formatVec3(v) {
  return `(${formatNumber(v.x, 6, 2)}, ${formatNumber(v.y, 6, 2)}, ${formatNumber(v.z, 6, 2)})`;
}

async function main() {
  const module = await createStressModule({
    locateFile: (file) => resolve(__dirname, 'dist', file),
    instantiateWasm(imports, successCallback) {
      try {
        const binary = readFileSync(wasmPath);
        const wasmModule = new WebAssembly.Module(binary);
        const instance = new WebAssembly.Instance(wasmModule, imports);
        successCallback(instance);
        return instance.exports;
      } catch (error) {
        console.error('Failed to instantiate stress solver WASM:', error);
        throw error;
      }
    }
  });

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

  let dataView = new DataView(module.HEAPU8.buffer);
  const refreshDataView = () => {
    if (dataView.buffer !== module.HEAPU8.buffer) {
      dataView = new DataView(module.HEAPU8.buffer);
    }
    return dataView;
  };

  const initialNodes = [
    { com: vec3(-1.0, 0.0, 0.0), mass: 25.0, inertia: 2.5 },
    { com: vec3(1.0, 0.0, 0.0), mass: 25.0, inertia: 2.5 },
    { com: vec3(0.0, 1.5, 0.0), mass: 15.0, inertia: 1.5 }
  ];

  const initialBonds = [
    { centroid: vec3(-0.5, 0.75, 0.0), node0: 0, node1: 2 },
    { centroid: vec3(0.5, 0.75, 0.0), node0: 1, node1: 2 },
    { centroid: vec3(0.0, 0.0, 0.0), node0: 0, node1: 1 }
  ];

  const nodeCount = initialNodes.length;
  const bondCount = initialBonds.length;

  const nodesPtr = module._malloc(sizes.node * nodeCount);
  refreshDataView();
  initialNodes.forEach((node, index) => {
    const base = nodesPtr + index * sizes.node;
    const view = refreshDataView();
    writeVec3(view, base, node.com);
    view.setFloat32(base + 12, node.mass, true);
    view.setFloat32(base + 16, node.inertia, true);
  });

  const bondsPtr = module._malloc(sizes.bond * bondCount);
  refreshDataView();
  initialBonds.forEach((bond, index) => {
    const base = bondsPtr + index * sizes.bond;
    const view = refreshDataView();
    writeVec3(view, base, bond.centroid);
    view.setUint32(base + 12, bond.node0, true);
    view.setUint32(base + 16, bond.node1, true);
  });

  const dataParamsPtr = module._malloc(sizes.dataParams);
  refreshDataView();
  dataView.setUint8(dataParamsPtr, 1); // equalize_masses
  dataView.setUint8(dataParamsPtr + 1, 1); // center_bonds

  const handle = module.ccall(
    'stress_processor_create',
    'number',
    ['number', 'number', 'number', 'number', 'number'],
    [nodesPtr, nodeCount, bondsPtr, bondCount, dataParamsPtr]
  );

  if (!handle) {
    throw new Error('Failed to create StressProcessor');
  }

  const simdEnabled = module.ccall('stress_processor_using_simd', 'number', [], []) !== 0;
  const nodePtr = module._malloc(sizes.node);
  const bondPtr = module._malloc(sizes.bond);

  refreshDataView();
  const nodes = [];
  for (let i = 0; i < nodeCount; ++i) {
    const ok = module.ccall(
      'stress_processor_get_node_desc',
      'number',
      ['number', 'number', 'number'],
      [handle, i, nodePtr]
    );
    if (!ok) {
      throw new Error(`Failed to read node descriptor ${i}`);
    }
    const view = refreshDataView();
    nodes.push({
      com: readVec3(view, nodePtr),
      mass: view.getFloat32(nodePtr + 12, true),
      inertia: view.getFloat32(nodePtr + 16, true)
    });
  }

  const bondEntries = [];
  for (let i = 0; i < bondCount; ++i) {
    const ok = module.ccall(
      'stress_processor_get_bond_desc',
      'number',
      ['number', 'number', 'number'],
      [handle, i, bondPtr]
    );
    if (!ok) {
      throw new Error(`Failed to read bond descriptor ${i}`);
    }
    const view = refreshDataView();
    bondEntries.push({
      name: BOND_NAMES[i],
      area: BOND_AREAS[i],
      desc: {
        centroid: readVec3(view, bondPtr),
        node0: view.getUint32(bondPtr + 12, true),
        node1: view.getUint32(bondPtr + 16, true)
      }
    });
  }

  console.log(
    `StressProcessor created with ${nodeCount} nodes and ${bondCount} bonds (SIMD: ${simdEnabled})`
  );

  console.log('\nNode data:');
  nodes.forEach((node, index) => {
    console.log(
      `  node ${String(index).padEnd(2)} | mass ${formatNumber(node.mass, 7, 2)} | inertia ${formatNumber(node.inertia, 6, 2)} | com ${formatVec3(node.com)}`
    );
  });

  console.log('\nInitial bonds:');
  bondEntries.forEach((entry) => {
    const { desc } = entry;
    console.log(
      `  ${entry.name.padEnd(15)} connects nodes ${desc.node0} and ${desc.node1} (centroid ${formatVec3(desc.centroid)}) area ${entry.area.toFixed(2)}`
    );
  });

  const velocitiesPtr = module._malloc(sizes.velocity * nodeCount);
  const impulsesPtr = module._malloc(sizes.impulse * bondCount);
  const solverParamsPtr = module._malloc(sizes.solverParams);
  const errorPtr = module._malloc(sizes.errorSq);

  const zeroBuffer = (ptr, size) => {
    module.HEAPU8.fill(0, ptr, ptr + size);
    refreshDataView();
  };

  zeroBuffer(velocitiesPtr, sizes.velocity * nodeCount);
  zeroBuffer(impulsesPtr, sizes.impulse * bondCount);
  zeroBuffer(errorPtr, sizes.errorSq);

  refreshDataView();
  dataView.setUint32(solverParamsPtr, 128, true);
  dataView.setFloat32(solverParamsPtr + 4, 1.0e-5, true);
  dataView.setUint8(solverParamsPtr + 8, 1);

  const limits = new StressLimits();

  let activeBonds = bondEntries.slice();
  FRAME_INPUTS.forEach((input, frameIndex) => {
    if (activeBonds.length === 0) {
      return;
    }

    zeroBuffer(velocitiesPtr, sizes.velocity * nodeCount);
    const view = refreshDataView();
    const topBase = velocitiesPtr + TOP_NODE * sizes.velocity;
    writeVec3(view, topBase, input.ang);
    writeVec3(view, topBase + sizes.vec3, input.lin);

    const iterations = module.ccall(
      'stress_processor_solve',
      'number',
      ['number', 'number', 'number', 'number', 'number', 'number'],
      [handle, impulsesPtr, velocitiesPtr, solverParamsPtr, errorPtr, 0]
    );

    if (iterations < 0) {
      throw new Error('Solver returned an error');
    }

    const errorView = refreshDataView();
    const angularError = Math.sqrt(errorView.getFloat32(errorPtr, true));
    const linearError = Math.sqrt(errorView.getFloat32(errorPtr + 4, true));

    console.log(
      `\nFrame ${frameIndex + 1}: iterations ${String(iterations).padStart(3)} , linear error ${formatNumber(linearError, 8, 5)}, angular error ${formatNumber(angularError, 8, 5)}`
    );

    const removals = [];
    activeBonds.forEach((entry, index) => {
      const impulse = readImpulse(refreshDataView(), impulsesPtr, sizes, index);
      const stress = computeBondStress(entry.desc, impulse, nodes, entry.area);
      const severity = limits.severity(stress);
      console.log(
        `  ${entry.name.padEnd(15)} | comp ${formatNumber(stress.compression, 7, 2)} (${formatNumber(severity.compression, 4, 2)}) | tens ${formatNumber(stress.tension, 7, 2)} (${formatNumber(severity.tension, 4, 2)}) | shear ${formatNumber(stress.shear, 7, 2)} (${formatNumber(severity.shear, 4, 2)})`
      );
      const mode = limits.failureMode(stress);
      if (mode) {
        const fatal = limits.fatalThreshold(mode);
        console.log(
          `    -> ${mode} limit exceeded (fatal ${fatal.toFixed(1)}); scheduling removal`
        );
        removals.push({ index, mode, name: entry.name });
      }
    });

    if (removals.length > 0) {
      removals.sort((a, b) => a.index - b.index);
      for (let i = removals.length - 1; i >= 0; --i) {
        const removal = removals[i];
        const removed = module.ccall(
          'stress_processor_remove_bond',
          'number',
          ['number', 'number'],
          [handle, removal.index]
        );
        if (removed) {
          console.log(`  Removing bond '${removal.name}' due to ${removal.mode} failure`);
          if (removal.index !== activeBonds.length - 1) {
            activeBonds[removal.index] = activeBonds[activeBonds.length - 1];
          }
          activeBonds.pop();
        }
      }

      const updatedCount = module.ccall(
        'stress_processor_bond_count',
        'number',
        ['number'],
        [handle]
      );
      console.log(`  Solver now tracks ${updatedCount} remaining bonds`);

      activeBonds = activeBonds.map((entry, index) => {
        const ok = module.ccall(
          'stress_processor_get_bond_desc',
          'number',
          ['number', 'number', 'number'],
          [handle, index, bondPtr]
        );
        if (!ok) {
          throw new Error(`Failed to refresh bond descriptor ${index}`);
        }
        const view = refreshDataView();
        return {
          name: entry.name,
          area: entry.area,
          desc: {
            centroid: readVec3(view, bondPtr),
            node0: view.getUint32(bondPtr + 12, true),
            node1: view.getUint32(bondPtr + 16, true)
          }
        };
      });
    }

    if (activeBonds.length === 0) {
      console.log('\nAll bonds were removed; stress network resolved.');
    }
  });

  module.ccall('stress_processor_destroy', 'void', ['number'], [handle]);
  module._free(nodesPtr);
  module._free(bondsPtr);
  module._free(dataParamsPtr);
  module._free(nodePtr);
  module._free(bondPtr);
  module._free(velocitiesPtr);
  module._free(impulsesPtr);
  module._free(solverParamsPtr);
  module._free(errorPtr);
}

function writeVec3(view, base, value) {
  view.setFloat32(base, value.x, true);
  view.setFloat32(base + 4, value.y, true);
  view.setFloat32(base + 8, value.z, true);
}

function readVec3(view, base) {
  return vec3(
    view.getFloat32(base, true),
    view.getFloat32(base + 4, true),
    view.getFloat32(base + 8, true)
  );
}

function readImpulse(view, impulsesPtr, sizes, index) {
  const base = impulsesPtr + index * sizes.impulse;
  return {
    ang: readVec3(view, base),
    lin: readVec3(view, base + sizes.vec3)
  };
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
