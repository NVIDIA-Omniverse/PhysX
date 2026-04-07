import {
  loadStressSolver,
  vec3,
  formatVec3,
  formatNumber
} from './stress.js';

function buildCubeScenario({
  cubeSize = 2.0,
  cellsPerAxis = 2,
  totalMass = 160.0
} = {}) {
  const cellSize = cubeSize / cellsPerAxis;
  const start = -cubeSize / 2 + cellSize / 2;
  const coords = Array.from({ length: cellsPerAxis }, (_, i) => start + i * cellSize);
  const nodes = [];
  const indexOf = new Map();
  const volume = cellSize * cellSize * cellSize;
  const massPerNode = totalMass / (cellsPerAxis ** 3);

  const key = (ix, iy, iz) => `${ix},${iy},${iz}`;
  const idx = (ix, iy, iz) => indexOf.get(key(ix, iy, iz));

  coords.forEach((x, ix) => {
    coords.forEach((y, iy) => {
      coords.forEach((z, iz) => {
        const node = { centroid: vec3(x, y, z), mass: massPerNode, volume };
        const index = nodes.length;
        nodes.push(node);
        indexOf.set(key(ix, iy, iz), index);
      });
    });
  });

  const bonds = [];
  const frontFaceNodes = [];
  const area = cellSize * cellSize;

  const addBond = (a, b) => {
    const nodeA = nodes[a];
    const nodeB = nodes[b];
    const centroid = vec3(
      (nodeA.centroid.x + nodeB.centroid.x) / 2,
      (nodeA.centroid.y + nodeB.centroid.y) / 2,
      (nodeA.centroid.z + nodeB.centroid.z) / 2
    );
    const normal = normalize(subtract(nodeB.centroid, nodeA.centroid));
    bonds.push({ centroid, normal, area, node0: a, node1: b });
  };

  for (let ix = 0; ix < cellsPerAxis; ++ix) {
    for (let iy = 0; iy < cellsPerAxis; ++iy) {
      for (let iz = 0; iz < cellsPerAxis; ++iz) {
        const current = idx(ix, iy, iz);
        if (iz === cellsPerAxis - 1) {
          frontFaceNodes.push(current);
        }
        if (ix + 1 < cellsPerAxis) {
          addBond(current, idx(ix + 1, iy, iz));
        }
        if (iy + 1 < cellsPerAxis) {
          addBond(current, idx(ix, iy + 1, iz));
        }
        if (iz + 1 < cellsPerAxis) {
          addBond(current, idx(ix, iy, iz + 1));
        }
      }
    }
  }

  return { nodes, bonds, frontFaceNodes, cellSize };
}

function subtract(a, b) {
  return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

function length(v) {
  return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

function normalize(v) {
  const len = length(v);
  if (len === 0) {
    return vec3();
  }
  return vec3(v.x / len, v.y / len, v.z / len);
}

function logDebugLines(lines, limit = 5) {
  const sorted = [...lines].sort((a, b) => lineLength(b) - lineLength(a));
  sorted.slice(0, limit).forEach((line, index) => {
    console.log(
      `  line ${String(index).padStart(2)} | length ${formatNumber(lineLength(line), 6, 3)} | p0 ${formatVec3(line.p0)} | p1 ${formatVec3(line.p1)} | color0 0x${
        line.color0.toString(16).padStart(8, '0')
      }`
    );
  });
}

function lineLength(line) {
  const dx = line.p1.x - line.p0.x;
  const dy = line.p1.y - line.p0.y;
  const dz = line.p1.z - line.p0.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

async function main() {
  const scenario = buildCubeScenario({ cubeSize: 2.0, cellsPerAxis: 2, totalMass: 160.0 });
  console.log(`Cube scenario created with ${scenario.nodes.length} nodes and ${scenario.bonds.length} bonds`);

  const runtime = await loadStressSolver();
  const settings = runtime.defaultExtSettings();
  settings.compressionElasticLimit = 0.8;
  settings.compressionFatalLimit = 1.3;
  settings.tensionElasticLimit = 0.6;
  settings.tensionFatalLimit = 1.1;
  settings.shearElasticLimit = 0.6;
  settings.shearFatalLimit = 1.0;

  const solver = runtime.createExtSolver({
    nodes: scenario.nodes,
    bonds: scenario.bonds,
    settings
  });

  solver.addGravity(vec3(0.0, -9.81, 0.0));

  const projectileForce = 4200.0;
  scenario.frontFaceNodes.forEach((index) => {
    solver.addForce(index, vec3(), vec3(0.0, 0.0, -projectileForce));
  });

  let step = 0;
  while (!solver.converged() && step < 32) {
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

  const overstressed = solver.overstressedBondCount();
  console.log(`\nProjectile impact produced ${overstressed} overstressed bonds`);

  const actors = solver.actors();
  console.log(`Tracked actors: ${actors.length}`);
  actors.slice(0, 4).forEach((actor, index) => {
    console.log(`  actor ${index} -> id ${actor.actorIndex}, nodes ${actor.nodes.join(', ')}`);
  });

  const fractureSets = solver.generateFractureCommandsPerActor({ maxBonds: solver.bondCapacity() });
  console.log(`Fracture sets generated: ${fractureSets.length}`);
  if (fractureSets.length > 0) {
    const firstSet = fractureSets[0];
    console.log(`  first set: actor ${firstSet.actorIndex}, fractures ${firstSet.fractures.length}`);
  }

  const debugLines = solver.fillDebugRender({ mode: runtime.ExtDebugMode.Max, scale: 1.0 });
  console.log(`Debug render emitted ${debugLines.length} lines (showing top ${Math.min(5, debugLines.length)})`);
  logDebugLines(debugLines);

  console.log('\nFront face nodes:');
  scenario.frontFaceNodes.forEach((nodeIndex) => {
    const node = scenario.nodes[nodeIndex];
    console.log(`  node ${nodeIndex} | centroid ${formatVec3(node.centroid)}`);
  });

  solver.destroy();
}

main().catch((error) => {
  console.error(error);
  process.exit(1);
});
