import {
  loadStressSolver,
  vec3,
  formatVec3,
  formatNumber
} from './stress.js';

const NODE_LABELS = ['left', 'right', 'top'];

const NODE_DESCRIPTIONS = [
  { centroid: vec3(-1.0, 0.0, 0.0), mass: 25.0, volume: 2.5 },
  { centroid: vec3(1.0, 0.0, 0.0), mass: 25.0, volume: 2.5 },
  { centroid: vec3(0.0, 1.5, 0.0), mass: 15.0, volume: 1.5 }
];

const BOND_DESCRIPTIONS = [
  {
    name: 'left diagonal',
    desc: { centroid: vec3(-0.5, 0.75, 0.0), normal: vec3(-0.6, 0.8, 0.0), area: 0.6, node0: 0, node1: 2 }
  },
  {
    name: 'right diagonal',
    desc: { centroid: vec3(0.5, 0.75, 0.0), normal: vec3(0.6, 0.8, 0.0), area: 0.6, node0: 1, node1: 2 }
  },
  {
    name: 'base tie',
    desc: { centroid: vec3(0.0, 0.0, 0.0), normal: vec3(0.0, 1.0, 0.0), area: 0.9, node0: 0, node1: 1 }
  }
];

async function main() {
  const runtime = await loadStressSolver();
  const solver = runtime.createExtSolver({
    nodes: NODE_DESCRIPTIONS,
    bonds: BOND_DESCRIPTIONS.map((entry) => entry.desc),
    settings: runtime.defaultExtSettings()
  });

  console.log(`ExtStressSolver created with ${solver.graphNodeCount()} graph nodes`);

  solver.addGravity(vec3(0.0, -9.81, 0.0));
  solver.addForce(2, vec3(), vec3(0.0, -200.0, 0.0));

  let step = 0;
  while (!solver.converged() && step < 10) {
    solver.update();
    const error = solver.stressError();
    console.log(
      `step ${String(step + 1).padStart(2)} | linear error ${formatNumber(error.lin, 8, 5)} | angular error ${formatNumber(
        error.ang,
        8,
        5
      )}`
    );
    step += 1;
  }

  console.log(`\nOverstressed bonds reported: ${solver.overstressedBondCount()}`);

  const debugLines = solver.fillDebugRender({ mode: runtime.ExtDebugMode.Max, scale: 1.0 });
  console.log(`Debug render lines: ${debugLines.length}`);
  debugLines.slice(0, 3).forEach((line, index) => {
    console.log(`  line ${index} | p0 ${formatVec3(line.p0)} | p1 ${formatVec3(line.p1)}`);
  });

  console.log('\nFinal node state:');
  NODE_DESCRIPTIONS.forEach((node, index) => {
    console.log(`  ${NODE_LABELS[index].padEnd(5)} | centroid ${formatVec3(node.centroid)} | mass ${node.mass.toFixed(1)}`);
  });

  solver.destroy();
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
