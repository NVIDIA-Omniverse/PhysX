/**
 * Grid-based wall scenario builder.
 *
 * Ported from vibe-city wallScenario.ts — produces a ScenarioDesc with no
 * Three.js or RAPIER dependencies so it can run headlessly in tests.
 */
import type { ScenarioBond, ScenarioDesc, ScenarioNode, Vec3 } from '../rapier/types';

export type WallScenarioOptions = {
  span?: number;
  height?: number;
  thickness?: number;
  spanSegments?: number;
  heightSegments?: number;
  layers?: number;
  deckMass?: number;
  areaScale?: number;
  addDiagonals?: boolean;
  diagScale?: number;
  normalizeAreas?: boolean;
  bondsX?: boolean;
  bondsY?: boolean;
  bondsZ?: boolean;
};

export const DEFAULT_WALL_OPTIONS: Required<WallScenarioOptions> = {
  span: 6.0,
  height: 3.0,
  thickness: 0.32,
  spanSegments: 12,
  heightSegments: 6,
  layers: 1,
  deckMass: 10_000,
  areaScale: 0.05,
  addDiagonals: false,
  diagScale: 0.75,
  normalizeAreas: true,
  bondsX: true,
  bondsY: true,
  bondsZ: true,
};

function sub(a: Vec3, b: Vec3): Vec3 {
  return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z };
}

function normalize(v: Vec3): Vec3 {
  const len = Math.hypot(v.x, v.y, v.z);
  if (len === 0) return { x: 0, y: 0, z: 0 };
  return { x: v.x / len, y: v.y / len, z: v.z / len };
}

export function buildWallScenario(opts: WallScenarioOptions = {}): ScenarioDesc {
  const {
    span, height, thickness,
    spanSegments, heightSegments, layers,
    deckMass, areaScale,
    addDiagonals, diagScale, normalizeAreas,
    bondsX, bondsY, bondsZ,
  } = { ...DEFAULT_WALL_OPTIONS, ...opts };

  const nodes: ScenarioNode[] = [];
  const bonds: ScenarioBond[] = [];

  const cellX = span / Math.max(spanSegments, 1);
  const cellY = height / Math.max(heightSegments, 1);
  const cellZ = thickness / Math.max(layers, 1);

  const originX = -span * 0.5 + 0.5 * cellX;
  const originY = 0 + 0.5 * cellY;
  const originZ = 0;

  const totalNodes = spanSegments * heightSegments * layers;
  const massPerNode = deckMass / Math.max(totalNodes, 1);
  const volumePerNode = cellX * cellY * cellZ;

  const index3D: number[][][] = Array.from({ length: spanSegments }, () =>
    Array.from({ length: heightSegments }, () => Array(layers)),
  );
  const gridCoordinates: Array<{ ix: number; iy: number; iz: number }> = [];

  for (let ix = 0; ix < spanSegments; ix++) {
    for (let iy = 0; iy < heightSegments; iy++) {
      for (let iz = 0; iz < layers; iz++) {
        const centroid: Vec3 = {
          x: originX + ix * cellX,
          y: originY + iy * cellY,
          z: originZ + (iz - (layers - 1) * 0.5) * cellZ,
        };
        const isSupport = iy === 0;
        const index = nodes.length;
        nodes.push({
          centroid,
          mass: isSupport ? 0 : massPerNode,
          volume: isSupport ? 0 : volumePerNode,
        });
        index3D[ix][iy][iz] = index;
        gridCoordinates[index] = { ix, iy, iz };
      }
    }
  }

  const areaX = cellY * cellZ * areaScale;
  const areaY = cellX * cellZ * areaScale;
  const areaZ = cellX * cellY * areaScale;

  const addBond = (a: number, b: number, area: number) => {
    const na = nodes[a];
    const nb = nodes[b];
    const centroid: Vec3 = {
      x: (na.centroid.x + nb.centroid.x) * 0.5,
      y: (na.centroid.y + nb.centroid.y) * 0.5,
      z: (na.centroid.z + nb.centroid.z) * 0.5,
    };
    const normal = normalize(sub(nb.centroid, na.centroid));
    bonds.push({ node0: a, node1: b, centroid, normal, area: Math.max(area, 1e-8) });
  };

  for (let ix = 0; ix < spanSegments; ix++) {
    for (let iy = 0; iy < heightSegments; iy++) {
      for (let iz = 0; iz < layers; iz++) {
        const current = index3D[ix][iy][iz];
        if (bondsX && ix + 1 < spanSegments) addBond(current, index3D[ix + 1][iy][iz], areaX);
        if (bondsY && iy + 1 < heightSegments) addBond(current, index3D[ix][iy + 1][iz], areaY);
        if (bondsZ && iz + 1 < layers) addBond(current, index3D[ix][iy][iz + 1], areaZ);
        if (addDiagonals) {
          if (ix + 1 < spanSegments && iy + 1 < heightSegments)
            addBond(current, index3D[ix + 1][iy + 1][iz], 0.5 * (areaX + areaY) * diagScale);
          if (ix + 1 < spanSegments && iz + 1 < layers)
            addBond(current, index3D[ix + 1][iy][iz + 1], 0.5 * (areaX + areaZ) * diagScale);
          if (iy + 1 < heightSegments && iz + 1 < layers)
            addBond(current, index3D[ix][iy + 1][iz + 1], 0.5 * (areaY + areaZ) * diagScale);
        }
      }
    }
  }

  if (normalizeAreas && bonds.length) {
    // Use isotropic normalization: apply a single uniform scale factor
    // (geometric mean of per-axis scales) so that bonds in all directions
    // have the same relative strength. Per-axis scaling created anisotropy
    // that caused horizontal layer separation under gravity.
    const target = { x: height * thickness, y: span * thickness, z: span * height };
    const sum = { x: 0, y: 0, z: 0 };
    const pick = (n: Vec3): 'x' | 'y' | 'z' => {
      const ax = Math.abs(n.x), ay = Math.abs(n.y), az = Math.abs(n.z);
      return ax >= ay && ax >= az ? 'x' : (ay >= az ? 'y' : 'z');
    };
    bonds.forEach((b) => { sum[pick(b.normal)] += b.area; });
    const axisScales: number[] = [];
    for (const k of ['x', 'y', 'z'] as const) {
      if (sum[k] > 0) axisScales.push(target[k] / sum[k]);
    }
    const uniformScale = axisScales.length > 0
      ? Math.pow(axisScales.reduce((a, b) => a * b, 1), 1 / axisScales.length)
      : 1;
    bonds.forEach((b) => { b.area *= uniformScale; });
  }

  return {
    nodes,
    bonds,
    gridCoordinates,
    spacing: { x: cellX, y: cellY, z: cellZ },
    parameters: {
      span, height, thickness, spanSegments, heightSegments, layers,
      deckMass, areaScale,
    },
  };
}
