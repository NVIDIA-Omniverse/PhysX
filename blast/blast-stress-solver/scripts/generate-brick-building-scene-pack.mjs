import { mkdir, writeFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

import * as THREE from 'three';
import { buildScenarioFromFragmentsAsync } from '../dist/three.js';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const BRICK = {
  length: 1.0,
  height: 0.4,
  thickness: 0.5,
};

const BUILDING = {
  key: 'brick_building',
  title: 'Brick Building',
  width: 6.0,
  depth: 4.0,
  stories: 2,
  coursesPerStory: 5,
  parapetCourses: 2,
  totalMass: 25_000,
  projectile: {
    radius: 0.4,
    mass: 1_200,
    speed: 24,
    ttlMs: 8_000,
  },
  solver: {
    gravity: -9.81,
    materialScale: 1e10,
  },
  physics: {
    debrisCollisionMode: 'all',
    friction: 0.25,
    restitution: 0.0,
    contactForceScale: 30,
    skipSingleBodies: false,
  },
  optimization: {
    smallBodyDampingMode: 'always',
    debrisCleanupMode: 'always',
    debrisTtlMs: 10_000,
    maxCollidersForDebris: 3,
  },
  camera: {
    target: { x: 0, y: 2.3, z: 0 },
    distance: 18.0,
  },
};

const OUTPUT_PATH = path.resolve(
  __dirname,
  '../../blast-stress-demo-rs/assets/scenes/brick-building.json',
);

function approxEqual(a, b, eps = 1e-6) {
  return Math.abs(a - b) <= eps;
}

function brickGeometry(length, axis) {
  if (axis === 'x') {
    return new THREE.BoxGeometry(length, BRICK.height, BRICK.thickness);
  }
  return new THREE.BoxGeometry(BRICK.thickness, BRICK.height, length);
}

function halfExtents(length, axis) {
  return axis === 'x'
    ? {
        x: length * 0.5,
        y: BRICK.height * 0.5,
        z: BRICK.thickness * 0.5,
      }
    : {
        x: BRICK.thickness * 0.5,
        y: BRICK.height * 0.5,
        z: length * 0.5,
      };
}

function createBrickFragment({ axis, length, x, y, z, isSupport }) {
  return {
    worldPosition: { x, y, z },
    halfExtents: halfExtents(length, axis),
    geometry: brickGeometry(length, axis),
    isSupport,
  };
}

function buildCoursePattern(outerSpan, extended) {
  if (extended) {
    const brickCount = Math.round(outerSpan / BRICK.length);
    const start = -outerSpan * 0.5;
    return Array.from({ length: brickCount }, (_unused, index) => ({
      center: start + BRICK.length * (index + 0.5),
      length: BRICK.length,
    }));
  }

  const insetSpan = outerSpan - BRICK.thickness * 2;
  const fullCount = Math.round(insetSpan - BRICK.length);
  const parts = [];
  let cursor = -insetSpan * 0.5;

  parts.push({
    center: cursor + BRICK.length * 0.25,
    length: BRICK.length * 0.5,
  });
  cursor += BRICK.length * 0.5;

  for (let index = 0; index < fullCount; index += 1) {
    parts.push({
      center: cursor + BRICK.length * 0.5,
      length: BRICK.length,
    });
    cursor += BRICK.length;
  }

  parts.push({
    center: cursor + BRICK.length * 0.25,
    length: BRICK.length * 0.5,
  });

  return parts;
}

function openingsForWall(wall, courseIndex) {
  const story = Math.floor(courseIndex / BUILDING.coursesPerStory);
  const storyCourse = courseIndex % BUILDING.coursesPerStory;
  const lowerWindowCourse = story === 0 && (storyCourse === 1 || storyCourse === 2);
  const upperWindowCourse = story === 1 && (storyCourse === 1 || storyCourse === 2);
  const isDoorCourse = wall === 'front' && story === 0 && storyCourse <= 3;

  if (wall === 'front') {
    const intervals = [];
    if (isDoorCourse) intervals.push([-1.0, 1.0]);
    if (lowerWindowCourse || upperWindowCourse) {
      intervals.push([-2.0, -1.0], [1.0, 2.0]);
    }
    return intervals;
  }

  if (wall === 'back') {
    return lowerWindowCourse || upperWindowCourse ? [[-2.0, -1.0], [1.0, 2.0]] : [];
  }

  if (wall === 'left' || wall === 'right') {
    return lowerWindowCourse || upperWindowCourse ? [[-1.0, 1.0]] : [];
  }

  return [];
}

function overlapsOpening(center, length, intervals) {
  const min = center - length * 0.5;
  const max = center + length * 0.5;
  return intervals.some(([start, end]) => max > start && min < end);
}

function wallFragmentsForCourse(courseIndex, y, includeOpenings) {
  const frontBackExtended = courseIndex % 2 === 0;
  const frontBackPattern = buildCoursePattern(BUILDING.width, frontBackExtended);
  const sidePattern = buildCoursePattern(BUILDING.depth, !frontBackExtended);
  const frontZ = -BUILDING.depth * 0.5 + BRICK.thickness * 0.5;
  const backZ = BUILDING.depth * 0.5 - BRICK.thickness * 0.5;
  const leftX = -BUILDING.width * 0.5 + BRICK.thickness * 0.5;
  const rightX = BUILDING.width * 0.5 - BRICK.thickness * 0.5;
  const isSupport = courseIndex === 0;
  const fragments = [];

  const frontBackWalls = [
    ['front', frontZ],
    ['back', backZ],
  ];
  for (const [wall, z] of frontBackWalls) {
    const openings = includeOpenings ? openingsForWall(wall, courseIndex) : [];
    for (const brick of frontBackPattern) {
      if (overlapsOpening(brick.center, brick.length, openings)) continue;
      fragments.push(
        createBrickFragment({
          axis: 'x',
          length: brick.length,
          x: brick.center,
          y,
          z,
          isSupport,
        }),
      );
    }
  }

  const sideWalls = [
    ['left', leftX],
    ['right', rightX],
  ];
  for (const [wall, x] of sideWalls) {
    const openings = includeOpenings ? openingsForWall(wall, courseIndex) : [];
    for (const brick of sidePattern) {
      if (overlapsOpening(brick.center, brick.length, openings)) continue;
      fragments.push(
        createBrickFragment({
          axis: 'z',
          length: brick.length,
          x,
          y,
          z: brick.center,
          isSupport,
        }),
      );
    }
  }

  return fragments;
}

function buildWallFragments() {
  const wallCourses = BUILDING.stories * BUILDING.coursesPerStory;
  const fragments = [];

  for (let courseIndex = 0; courseIndex < wallCourses; courseIndex += 1) {
    const y = BRICK.height * (courseIndex + 0.5);
    fragments.push(...wallFragmentsForCourse(courseIndex, y, true));
  }

  return fragments;
}

function buildRoofFragments() {
  const roofY = BRICK.height * (BUILDING.stories * BUILDING.coursesPerStory + 0.5);
  const xStart = -BUILDING.width * 0.5 + BRICK.length * 0.5;
  const xCount = Math.round(BUILDING.width / BRICK.length);
  const zStart = -BUILDING.depth * 0.5 + BRICK.thickness * 0.5;
  const zCount = Math.round(BUILDING.depth / BRICK.thickness);
  const fragments = [];

  for (let xIndex = 0; xIndex < xCount; xIndex += 1) {
    for (let zIndex = 0; zIndex < zCount; zIndex += 1) {
      fragments.push(
        createBrickFragment({
          axis: 'x',
          length: BRICK.length,
          x: xStart + xIndex * BRICK.length,
          y: roofY,
          z: zStart + zIndex * BRICK.thickness,
          isSupport: false,
        }),
      );
    }
  }

  return fragments;
}

function buildParapetFragments() {
  const wallTop = BRICK.height * BUILDING.stories * BUILDING.coursesPerStory;
  const roofTop = wallTop + BRICK.height;
  const baseCourse = BUILDING.stories * BUILDING.coursesPerStory;
  const fragments = [];

  for (let parapetCourse = 0; parapetCourse < BUILDING.parapetCourses; parapetCourse += 1) {
    const courseIndex = baseCourse + parapetCourse;
    const y = roofTop + BRICK.height * (parapetCourse + 0.5);
    fragments.push(...wallFragmentsForCourse(courseIndex, y, false));
  }

  return fragments;
}

function serializeMesh(geometry) {
  const position = geometry.getAttribute('position');
  const normal = geometry.getAttribute('normal');
  const indices = geometry.index
    ? Array.from(geometry.index.array, (value) => Number(value))
    : Array.from({ length: position.count }, (_unused, index) => index);

  return {
    positions: Array.from(position.array, (value) => Number(value)),
    normals: Array.from(normal.array, (value) => Number(value)),
    indices,
  };
}

function serializeScenePack(scenario) {
  const fragmentSizes = scenario.parameters?.fragmentSizes ?? [];
  const fragmentGeometries = scenario.parameters?.fragmentGeometries ?? [];

  return {
    version: 1,
    key: BUILDING.key,
    title: BUILDING.title,
    defaults: {
      camera: BUILDING.camera,
      projectile: BUILDING.projectile,
      solver: BUILDING.solver,
      physics: BUILDING.physics,
      optimization: BUILDING.optimization,
    },
    scenario: {
      nodes: scenario.nodes.map((node) => ({
        centroid: {
          x: Number(node.centroid.x),
          y: Number(node.centroid.y),
          z: Number(node.centroid.z),
        },
        mass: Number(node.mass),
        volume: Number(node.volume),
      })),
      bonds: scenario.bonds.map((bond) => ({
        node0: bond.node0,
        node1: bond.node1,
        centroid: {
          x: Number(bond.centroid.x),
          y: Number(bond.centroid.y),
          z: Number(bond.centroid.z),
        },
        normal: {
          x: Number(bond.normal.x),
          y: Number(bond.normal.y),
          z: Number(bond.normal.z),
        },
        area: Number(bond.area),
      })),
      nodeSizes: fragmentSizes.map((size) => ({
        x: Number(size.x),
        y: Number(size.y),
        z: Number(size.z),
      })),
      nodeColliders: fragmentSizes.map((size) => ({
        kind: 'cuboid',
        halfExtents: {
          x: Number(size.x * 0.5),
          y: Number(size.y * 0.5),
          z: Number(size.z * 0.5),
        },
      })),
    },
    nodeMeshes: fragmentGeometries.map(serializeMesh),
  };
}

function summarizeFragments(fragments) {
  let fullBricks = 0;
  let halfBricks = 0;
  for (const fragment of fragments) {
    const length = approxEqual(fragment.halfExtents.x * 2, BRICK.thickness)
      ? fragment.halfExtents.z * 2
      : fragment.halfExtents.x * 2;
    if (approxEqual(length, BRICK.length)) {
      fullBricks += 1;
    } else if (approxEqual(length, BRICK.length * 0.5)) {
      halfBricks += 1;
    }
  }
  return { fullBricks, halfBricks };
}

async function main() {
  const wallFragments = buildWallFragments();
  const roofFragments = buildRoofFragments();
  const parapetFragments = buildParapetFragments();
  const fragments = [...wallFragments, ...roofFragments, ...parapetFragments];

  const scenario = await buildScenarioFromFragmentsAsync(fragments, {
    totalMass: BUILDING.totalMass,
    bondMode: 'auto',
    autoBondingOptions: {
      mode: 'exact',
    },
    areaNormalization: 'none',
  });

  const pack = serializeScenePack(scenario);
  await mkdir(path.dirname(OUTPUT_PATH), { recursive: true });
  await writeFile(OUTPUT_PATH, `${JSON.stringify(pack, null, 2)}\n`, 'utf8');

  const summary = summarizeFragments(fragments);
  console.log(
    [
      `wrote ${OUTPUT_PATH}`,
      `nodes=${pack.scenario.nodes.length}`,
      `bonds=${pack.scenario.bonds.length}`,
      `full_bricks=${summary.fullBricks}`,
      `half_bricks=${summary.halfBricks}`,
    ].join(' '),
  );

  for (const fragment of fragments) {
    fragment.geometry.dispose();
  }
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
