import {
  loadStressSolver,
  StressLimits,
  StressFailure,
  computeBondStress,
  vec3,
  formatNumber,
  formatVec3
} from './stress.js';

const TOP_NODE = 2;
const BOND_NAMES = ['left diagonal', 'right diagonal', 'base tie'];
const BOND_AREAS = [0.6, 0.6, 0.9];

const FRAME_INPUTS = [
  { lin: vec3(0.0, -24.0, 0.0), ang: vec3(0.0, 0.0, 3.0) },
  { lin: vec3(8.0, -42.0, 0.0), ang: vec3(0.0, 0.0, 7.0) },
  { lin: vec3(14.0, -63.0, 0.0), ang: vec3(0.0, 0.0, 11.5) },
  { lin: vec3(20.0, -90.0, 0.0), ang: vec3(0.0, 0.0, 16.0) }
];

async function main() {
  const runtime = await loadStressSolver();
  const processor = runtime.createProcessor({
    nodes: [
      { com: vec3(-1.0, 0.0, 0.0), mass: 25.0, inertia: 2.5 },
      { com: vec3(1.0, 0.0, 0.0), mass: 25.0, inertia: 2.5 },
      { com: vec3(0.0, 1.5, 0.0), mass: 15.0, inertia: 1.5 }
    ],
    bonds: [
      { centroid: vec3(-0.5, 0.75, 0.0), node0: 0, node1: 2 },
      { centroid: vec3(0.5, 0.75, 0.0), node0: 1, node1: 2 },
      { centroid: vec3(0.0, 0.0, 0.0), node0: 0, node1: 1 }
    ],
    dataParams: { equalizeMasses: true, centerBonds: true }
  });

  const simdEnabled = runtime.module.ccall('stress_processor_using_simd', 'number', [], []) !== 0;
  console.log(
    `StressProcessor created with ${processor.nodeCount} nodes and ${processor.bondCount} bonds (SIMD: ${simdEnabled})`
  );

  const nodes = processor.getNodes();
  console.log('\nNode data:');
  nodes.forEach((node, index) => {
    console.log(
      `  node ${String(index).padEnd(2)} | mass ${formatNumber(node.mass, 7, 2)} | inertia ${formatNumber(node.inertia, 6, 2)} | com ${formatVec3(node.com)}`
    );
  });

  let activeBonds = processor.getBonds().map((bond, index) => ({
    name: BOND_NAMES[index],
    area: BOND_AREAS[index],
    desc: bond
  }));

  console.log('\nInitial bonds:');
  activeBonds.forEach((entry) => {
    const { desc } = entry;
    console.log(
      `  ${entry.name.padEnd(15)} connects nodes ${desc.node0} and ${desc.node1} (centroid ${formatVec3(desc.centroid)}) area ${entry.area.toFixed(2)}`
    );
  });

  processor.setSolverParams({ maxIterations: 128, tolerance: 1.0e-5, warmStart: true });
  const limits = new StressLimits();

  FRAME_INPUTS.forEach((input, frameIndex) => {
    if (activeBonds.length === 0) {
      return;
    }

    const velocities = nodes.map(() => ({ lin: vec3(), ang: vec3() }));
    velocities[TOP_NODE] = { lin: input.lin, ang: input.ang };

    const { iterations, error, impulses } = processor.solve({ velocities, resume: frameIndex > 0 });

    console.log(
      `\nFrame ${frameIndex + 1}: iterations ${String(iterations).padStart(3)}, linear error ${formatNumber(error.lin, 8, 5)}, angular error ${formatNumber(error.ang, 8, 5)}`
    );

    const removals = [];
    activeBonds.forEach((entry, index) => {
      const stress = computeBondStress(entry.desc, impulses[index], nodes, entry.area);
      const severity = limits.severity(stress);
      console.log(
        `  ${entry.name.padEnd(15)} | comp ${formatNumber(stress.compression, 7, 2)} (${formatNumber(severity.compression, 4, 2)}) | tens ${formatNumber(stress.tension, 7, 2)} (${formatNumber(severity.tension, 4, 2)}) | shear ${formatNumber(stress.shear, 7, 2)} (${formatNumber(severity.shear, 4, 2)})`
      );
      const mode = limits.failureMode(stress);
      if (mode) {
        console.log(
          `    -> ${mode} limit exceeded (fatal ${limits.fatalThreshold(mode).toFixed(1)}); scheduling removal`
        );
        removals.push({ index, mode, name: entry.name });
      }
    });

    if (removals.length) {
      removals.sort((a, b) => a.index - b.index);
      for (let i = removals.length - 1; i >= 0; --i) {
        const removal = removals[i];
        if (processor.removeBond(removal.index)) {
          console.log(`  Removing bond '${removal.name}' due to ${removal.mode} failure`);
          activeBonds.splice(removal.index, 1);
        }
      }
      activeBonds = processor.getBonds().map((bond, index) => ({
        name: BOND_NAMES[index] ?? `bond_${index}`,
        area: BOND_AREAS[index] ?? BOND_AREAS[BOND_AREAS.length - 1],
        desc: bond
      }));
      console.log(`  Solver now tracks ${processor.bondCount} remaining bonds`);
    }

    if (activeBonds.length === 0) {
      console.log('\nAll bonds were removed; stress network resolved.');
    }
  });

  processor.destroy();
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
