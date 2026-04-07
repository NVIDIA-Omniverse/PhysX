import { loadStressSolver, vec3, formatNumber } from './stress.js';

function buildBridgeScenario({
  span = 12.0,
  deckWidth = 4.0,
  deckThickness = 0.6,
  spanSegments = 6,
  widthSegments = 2,
  thicknessLayers = 2,
  deckMass = 6000.0,
  pierHeight = 3.0
} = {}) {
  const nodes = [];
  const bonds = [];

  const spacingX = span / (spanSegments - 1);
  const spacingZ = widthSegments > 1 ? deckWidth / (widthSegments - 1) : deckWidth;
  const spacingY = thicknessLayers > 1 ? deckThickness / (thicknessLayers - 1) : deckThickness;

  const originX = -span / 2;
  const originZ = widthSegments > 1 ? -deckWidth / 2 : 0.0;
  const originY = -deckThickness / 2;

  const totalDeckNodes = spanSegments * widthSegments * thicknessLayers;
  const massPerDeckNode = deckMass / totalDeckNodes;
  const volumePerDeckNode = spacingX * spacingZ * spacingY;

  const nodeIndex = Array.from({ length: spanSegments }, () =>
    Array.from({ length: thicknessLayers }, () => Array(widthSegments))
  );
  const topColumnNodes = Array.from({ length: spanSegments }, () => []);
  const deckBottomNodes = [];

  const indexAt = (ix, iy, iz) => nodeIndex[ix][iy][iz];

  for (let ix = 0; ix < spanSegments; ++ix) {
    for (let iy = 0; iy < thicknessLayers; ++iy) {
      for (let iz = 0; iz < widthSegments; ++iz) {
        const centroid = vec3(
          originX + ix * spacingX,
          originY + iy * spacingY,
          originZ + iz * spacingZ
        );
        const node = { centroid, mass: massPerDeckNode, volume: volumePerDeckNode };
        const index = nodes.length;
        nodes.push(node);
        nodeIndex[ix][iy][iz] = index;
        if (iy === thicknessLayers - 1) {
          topColumnNodes[ix].push(index);
        }
        if (iy === 0) {
          deckBottomNodes.push(index);
        }
      }
    }
  }

  const areaScale = 0.05;
  const deckBondAreaX = spacingY * spacingZ * areaScale;
  const deckBondAreaY = spacingX * spacingZ * areaScale;
  const deckBondAreaZ = spacingX * spacingY * areaScale;

  const addBond = (a, b, area) => {
    const na = nodes[a];
    const nb = nodes[b];
    const centroid = vec3(
      (na.centroid.x + nb.centroid.x) / 2,
      (na.centroid.y + nb.centroid.y) / 2,
      (na.centroid.z + nb.centroid.z) / 2
    );
    const normal = normalize(subtract(nb.centroid, na.centroid));
    bonds.push({ centroid, normal, area, node0: a, node1: b });
  };

  for (let ix = 0; ix < spanSegments; ++ix) {
    for (let iy = 0; iy < thicknessLayers; ++iy) {
      for (let iz = 0; iz < widthSegments; ++iz) {
        const current = indexAt(ix, iy, iz);
        if (ix + 1 < spanSegments) {
          addBond(current, indexAt(ix + 1, iy, iz), deckBondAreaX);
        }
        if (iy + 1 < thicknessLayers) {
          addBond(current, indexAt(ix, iy + 1, iz), deckBondAreaY);
        }
        if (iz + 1 < widthSegments) {
          addBond(current, indexAt(ix, iy, iz + 1), deckBondAreaZ);
        }
      }
    }
  }

  const supportIndices = [];
  const supportArea = spacingX * spacingZ * areaScale;
  const supportMass = massPerDeckNode * 6.0;
  const supportVolume = spacingX * spacingZ * pierHeight;

  for (const ix of [0, spanSegments - 1]) {
    for (let iz = 0; iz < widthSegments; ++iz) {
      const baseIndex = indexAt(ix, 0, iz);
      const baseNode = nodes[baseIndex];
      const supportCentroid = vec3(baseNode.centroid.x, baseNode.centroid.y - pierHeight, baseNode.centroid.z);
      const supportIndex = nodes.length;
      nodes.push({ centroid: supportCentroid, mass: supportMass, volume: supportVolume });
      supportIndices.push(supportIndex);
      addBond(baseIndex, supportIndex, supportArea);
    }
  }

  return {
    nodes,
    bonds,
    topColumnNodes,
    supportIndices,
    spanSegments,
    widthSegments
  };
}

function subtract(a, b) {
  return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

function normalize(v) {
  const len = Math.hypot(v.x, v.y, v.z);
  if (len === 0) {
    return vec3();
  }
  return vec3(v.x / len, v.y / len, v.z / len);
}

function summarizeLines(lines, limit = 3) {
  const sorted = [...lines].sort((a, b) => lineMagnitude(b) - lineMagnitude(a));
  return sorted.slice(0, limit).map((line) => ({
    length: lineMagnitude(line),
    centerX: (line.p0.x + line.p1.x) / 2,
    centerY: (line.p0.y + line.p1.y) / 2,
    centerZ: (line.p0.z + line.p1.z) / 2,
    color: `0x${line.color0.toString(16).padStart(8, '0')}`
  }));
}

function lineMagnitude(line) {
  const dx = line.p1.x - line.p0.x;
  const dy = line.p1.y - line.p0.y;
  const dz = line.p1.z - line.p0.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

async function main() {
  const scenario = buildBridgeScenario();
  console.log(`Bridge scenario created with ${scenario.nodes.length} nodes and ${scenario.bonds.length} bonds`);

  const runtime = await loadStressSolver();
  const settings = runtime.defaultExtSettings();
  settings.compressionElasticLimit = 0.01;
  settings.compressionFatalLimit = 0.02;
  settings.tensionElasticLimit = 0.01;
  settings.tensionFatalLimit = 0.02;
  settings.shearElasticLimit = 0.01;
  settings.shearFatalLimit = 0.02;

  const solver = runtime.createExtSolver({
    nodes: scenario.nodes,
    bonds: scenario.bonds,
    settings
  });

  const gravity = vec3(0.0, -6.0, 0.0);
  const carMass = 7200.0;
  const impactMultiplier = 5.0;

  const segments = scenario.spanSegments;
  const contactsPerStep = [];
  for (let column = 0; column < segments - 1; ++column) {
    const contact = [...scenario.topColumnNodes[column], ...scenario.topColumnNodes[column + 1]];
    contactsPerStep.push(contact);
  }

  let failureDetected = false;
  for (let index = 0; index < contactsPerStep.length; ++index) {
    const contact = contactsPerStep[index];
    solver.addGravity(gravity);

    const baseForcePerNode = (carMass * 9.81) / contact.length;
    const severity = 2.0 + index * 2.0;
    contact.forEach((nodeIndex, contactIdx) => {
      const dynamicBoost = contactIdx < contact.length / 2 ? 1.0 : impactMultiplier;
      const downwardForce = baseForcePerNode * severity * dynamicBoost;
      solver.addForce(nodeIndex, vec3(), vec3(0.0, -downwardForce, 0.0));
    });

    let step = 0;
    while (!solver.converged() && step < 30) {
      solver.update();
      const error = solver.stressError();
      console.log(
        `span step ${String(index + 1).padStart(2)}.${String(step + 1).padStart(2)} | lin ${formatNumber(
          error.lin,
          7,
          4
        )} | ang ${formatNumber(error.ang, 7, 4)}`
      );
      step += 1;
    }

    const overstressed = solver.overstressedBondCount();
    console.log(
      `  contact group ${String(index + 1).padStart(2)} on columns ${index}-${index + 1} produced ${overstressed} overstressed bonds`
    );

    const actors = solver.actors();
    console.log(`    actors tracked: ${actors.length}`);
    actors.slice(0, 4).forEach((actor, idx) => {
      console.log(`      actor ${idx} -> id ${actor.actorIndex}, nodes ${actor.nodes.join(', ')}`);
    });

    const debugLines = solver.fillDebugRender({ mode: runtime.ExtDebugMode.Max, scale: 1.0 });
    const highlights = summarizeLines(debugLines, 4);
    highlights.forEach((line, highlightIndex) => {
      console.log(
        `    debug line ${highlightIndex} | length ${formatNumber(line.length, 6, 3)} | center (${formatNumber(
          line.centerX,
          6,
          2
        )}, ${formatNumber(line.centerY, 6, 2)}, ${formatNumber(line.centerZ, 6, 2)}) | color ${line.color}`
      );
    });

    if (overstressed > 0) {
      const fractureSets = solver.generateFractureCommandsPerActor();
      console.log(`    fracture sets generated: ${fractureSets.length}`);

      let splitEvents = [];
      if (fractureSets.length > 0) {
        splitEvents = solver.applyFractureCommands(fractureSets);
        console.log(`    split events returned: ${splitEvents.length}`);
        splitEvents.forEach((evt, evtIndex) => {
          console.log(`      event ${evtIndex} -> parent ${evt.parentActorIndex}, children ${evt.children.length}`);
          evt.children.forEach((child, childIdx) => {
            console.log(`        child ${childIdx}: actor ${child.actorIndex}, nodes ${child.nodes.join(', ')}`);
          });
        });
      } else {
        console.log('    solver reported overstress but generated no fracture commands.');
      }

      if (splitEvents.length > 0) {
        console.log('  -> Bridge deck fractured under this car position. Stopping simulation.');
        failureDetected = true;
        break;
      }

      console.log('    Bonds damaged but no splits yet — continuing sweep for additional accumulation.');
    }
  }

  if (!failureDetected) {
    console.log('Bridge deck survived the scripted car sweep without exceeding the configured stress limits.');
  }

  solver.destroy();
}

main().catch((error) => {
  console.error(error);
  process.exit(1);
});
