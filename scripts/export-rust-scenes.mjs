import { mkdir, writeFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

import * as THREE from 'three';
import { FRACTURED_DEMO_CONFIGS } from '../blast/js_stress_example/fractured-demo-config.js';
import {
  buildFracturedBridgeScenario,
  buildFracturedTowerScenario,
} from '../blast/blast-stress-solver/dist/scenarios.js';
import {
  buildFracturedScenarioAsync,
  setPinataModule,
} from '../blast/blast-stress-solver/dist/three.js';
import * as pinata from '../blast/blast-stress-solver/node_modules/@dgreenheck/three-pinata/build/three-pinata.es.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const outputDir = path.resolve(
  __dirname,
  '../blast/blast-stress-demo-rs/assets/scenes',
);

setPinataModule(pinata);

function toPlainVec3(vec) {
  return { x: Number(vec.x), y: Number(vec.y), z: Number(vec.z) };
}

function getNodeSizes(scenario) {
  const sizes = scenario.parameters?.fragmentSizes;
  if (Array.isArray(sizes) && sizes.length === scenario.nodes.length) {
    return sizes.map(toPlainVec3);
  }

  return scenario.nodes.map((node) => {
    const side = Math.cbrt(Math.max(node.volume ?? 0, 1e-6));
    return { x: side, y: side, z: side };
  });
}

function makeSequentialIndices(count) {
  return Array.from({ length: count }, (_, index) => index);
}

function exportGeometry(geometry) {
  if (!geometry.getAttribute('normal')) {
    geometry.computeVertexNormals();
  }

  const position = geometry.getAttribute('position');
  const normal = geometry.getAttribute('normal');
  if (!position || !normal) {
    throw new Error('Fragment geometry is missing required position/normal attributes');
  }

  const index = geometry.getIndex();
  return {
    positions: Array.from(position.array, Number),
    normals: Array.from(normal.array, Number),
    indices: index ? Array.from(index.array, Number) : makeSequentialIndices(position.count),
  };
}

function exportCollider(node, nodeSize, meshData) {
  if (node.mass === 0) {
    return {
      kind: 'cuboid',
      halfExtents: {
        x: nodeSize.x * 0.5,
        y: nodeSize.y * 0.5,
        z: nodeSize.z * 0.5,
      },
    };
  }

  return {
    kind: 'convex_hull',
    points: meshData.positions,
  };
}

function exportScenarioPack(config, scenario) {
  const fragmentGeometries = scenario.parameters?.fragmentGeometries;
  if (!Array.isArray(fragmentGeometries) || fragmentGeometries.length !== scenario.nodes.length) {
    throw new Error(
      `${config.title}: expected fragment geometries for every node, got ${fragmentGeometries?.length ?? 0}`,
    );
  }

  const nodeSizes = getNodeSizes(scenario);
  const nodeMeshes = fragmentGeometries.map((geometry) => exportGeometry(geometry));
  const nodeColliders = scenario.nodes.map((node, index) =>
    exportCollider(node, nodeSizes[index], nodeMeshes[index]),
  );

  return {
    version: 1,
    key: config.key,
    title: config.title,
    defaults: {
      camera: config.camera,
      projectile: config.projectile,
      solver: config.solver,
      physics: config.physics,
      optimization: config.optimization,
    },
    scenario: {
      nodes: scenario.nodes.map((node) => ({
        centroid: toPlainVec3(node.centroid),
        mass: Number(node.mass),
        volume: Number(node.volume),
      })),
      bonds: scenario.bonds.map((bond) => ({
        node0: Number(bond.node0),
        node1: Number(bond.node1),
        centroid: toPlainVec3(bond.centroid),
        normal: toPlainVec3(bond.normal),
        area: Number(bond.area),
      })),
      nodeSizes,
      nodeColliders,
    },
    nodeMeshes,
  };
}

async function buildWallPack() {
  const config = FRACTURED_DEMO_CONFIGS.wall;
  const geometry = new THREE.BoxGeometry(
    config.wall.span,
    config.wall.height,
    config.wall.thickness,
    2,
    3,
    1,
  );
  const scenario = await buildFracturedScenarioAsync(geometry, {
    fragmentCount: config.wall.fragmentCount,
    pinata,
    voronoiMode: '3D',
    bondMode: 'auto',
    totalMass: config.wall.deckMass,
    areaNormalization: 'perAxis',
    dimensions: {
      x: config.wall.span,
      y: config.wall.height,
      z: config.wall.thickness,
    },
    worldOffset: {
      x: 0,
      y: config.wall.height * 0.5,
      z: 0,
    },
    foundation: {
      enabled: true,
      heightRatio: 0.06,
      maxHeight: 0.08,
    },
  });
  geometry.dispose();
  return {
    filename: 'fractured-wall.json',
    pack: exportScenarioPack(config, scenario),
  };
}

async function buildTowerPack() {
  const config = FRACTURED_DEMO_CONFIGS.tower;
  const scenario = await buildFracturedTowerScenario({
    width: config.tower.width,
    depth: config.tower.width,
    floorCount: config.tower.floorCount,
    floorHeight: config.tower.floorHeight,
    thickness: config.tower.thickness,
    floorThickness: config.tower.floorThickness,
    columnSize: config.tower.columnSize,
    columnsX: config.tower.columnsX,
    columnsZ: config.tower.columnsZ,
    fragmentCountPerWall: config.tower.fragmentCountPerWall,
    fragmentCountPerFloor: config.tower.fragmentCountPerFloor,
    fragmentCountPerColumn: config.tower.fragmentCountPerColumn,
    deckMass: config.tower.deckMass,
    bondMode: 'auto',
    pinata,
  });
  return {
    filename: 'fractured-tower.json',
    pack: exportScenarioPack(config, scenario),
  };
}

async function buildBridgePack() {
  const config = FRACTURED_DEMO_CONFIGS.bridge;
  const scenario = await buildFracturedBridgeScenario({
    ...config.bridge,
    bondMode: 'auto',
    pinata,
  });
  return {
    filename: 'fractured-bridge.json',
    pack: exportScenarioPack(config, scenario),
  };
}

async function main() {
  await mkdir(outputDir, { recursive: true });
  const outputs = await Promise.all([
    buildWallPack(),
    buildTowerPack(),
    buildBridgePack(),
  ]);

  for (const { filename, pack } of outputs) {
    const targetPath = path.join(outputDir, filename);
    await writeFile(targetPath, `${JSON.stringify(pack)}\n`, 'utf8');
    console.log(`wrote ${path.relative(process.cwd(), targetPath)}`);
  }
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
