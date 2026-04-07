/**
 * Grid-based tower scenario builder.
 *
 * Produces a square cross-section tower with support layer at the bottom.
 * Includes area normalization (ported from vibe-city patterns).
 */
import type { ScenarioBond, ScenarioDesc, ScenarioNode, Vec3 } from '../rapier/types';

export type TowerScenarioOptions = {
  side?: number;
  stories?: number;
  spacing?: { x: number; y: number; z: number };
  totalMass?: number;
  areaScale?: number;
  addDiagonals?: boolean;
  diagScale?: number;
  normalizeAreas?: boolean;
};

export const DEFAULT_TOWER_OPTIONS: Required<TowerScenarioOptions> = {
  side: 4,
  stories: 8,
  spacing: { x: 0.5, y: 0.5, z: 0.5 },
  totalMass: 5_000,
  areaScale: 0.05,
  addDiagonals: true,
  diagScale: 0.55,
  normalizeAreas: true,
};

function sub(a: Vec3, b: Vec3): Vec3 {
  return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z };
}

function normalize(v: Vec3): Vec3 {
  const len = Math.hypot(v.x, v.y, v.z);
  if (len === 0) return { x: 0, y: 0, z: 0 };
  return { x: v.x / len, y: v.y / len, z: v.z / len };
}

export function buildTowerScenario(opts: TowerScenarioOptions = {}): ScenarioDesc {
  const { side, stories, spacing, totalMass, areaScale, addDiagonals, diagScale, normalizeAreas } =
    { ...DEFAULT_TOWER_OPTIONS, ...opts };

  const nodes: ScenarioNode[] = [];
  const bonds: ScenarioBond[] = [];
  const gridCoordinates: Array<{ ix: number; iy: number; iz: number }> = [];

  const totalRows = stories + 1; // +1 for support row at bottom

  const dynamicNodeCount = side * stories * side;
  const nodeMass = totalMass / Math.max(1, dynamicNodeCount);

  const idx = (ix: number, iy: number, iz: number) =>
    iz * side * totalRows + iy * side + ix;

  for (let iz = 0; iz < side; iz++) {
    for (let iy = 0; iy < totalRows; iy++) {
      for (let ix = 0; ix < side; ix++) {
        const isSupport = iy === 0;
        const centroid: Vec3 = {
          x: (ix - (side - 1) / 2) * spacing.x,
          y: (iy - 1) * spacing.y,
          z: (iz - (side - 1) / 2) * spacing.z,
        };
        const volume = spacing.x * spacing.y * spacing.z;
        nodes.push({
          centroid,
          mass: isSupport ? 0 : nodeMass,
          volume: isSupport ? 0 : volume,
        });
        gridCoordinates.push({
          ix,
          iy: isSupport ? -1 : iy - 1,
          iz,
        });
      }
    }
  }

  const areaXY = spacing.x * spacing.y * areaScale;
  const areaYZ = spacing.y * spacing.z * areaScale;
  const areaXZ = spacing.x * spacing.z * areaScale;

  const offsets: [number, number, number, Vec3, number][] = [
    [1, 0, 0, { x: 1, y: 0, z: 0 }, areaYZ],
    [0, 1, 0, { x: 0, y: 1, z: 0 }, areaXZ],
    [0, 0, 1, { x: 0, y: 0, z: 1 }, areaXY],
  ];

  for (let iz = 0; iz < side; iz++) {
    for (let iy = 0; iy < totalRows; iy++) {
      for (let ix = 0; ix < side; ix++) {
        const i = idx(ix, iy, iz);
        for (const [dx, dy, dz, normal, area] of offsets) {
          const nx = ix + dx;
          const ny = iy + dy;
          const nz = iz + dz;
          if (nx < side && ny < totalRows && nz < side) {
            const j = idx(nx, ny, nz);
            const c0 = nodes[i].centroid;
            const c1 = nodes[j].centroid;
            bonds.push({
              node0: i,
              node1: j,
              centroid: {
                x: (c0.x + c1.x) / 2,
                y: (c0.y + c1.y) / 2,
                z: (c0.z + c1.z) / 2,
              },
              normal,
              area,
            });
          }
        }

        if (addDiagonals) {
          const diagArea = 0.5 * (areaXZ + areaYZ) * diagScale;
          const diagOffsets: [number, number, number][] = [
            [1, 1, 0], [1, -1, 0],
            [0, 1, 1], [0, -1, 1],
          ];
          for (const [ddx, ddy, ddz] of diagOffsets) {
            const nx = ix + ddx;
            const ny = iy + ddy;
            const nz = iz + ddz;
            if (nx >= 0 && nx < side && ny >= 0 && ny < totalRows && nz >= 0 && nz < side) {
              const j = idx(nx, ny, nz);
              const c0 = nodes[i].centroid;
              const c1 = nodes[j].centroid;
              const d = sub(c1, c0);
              const n = normalize(d);
              bonds.push({
                node0: i,
                node1: j,
                centroid: {
                  x: (c0.x + c1.x) / 2,
                  y: (c0.y + c1.y) / 2,
                  z: (c0.z + c1.z) / 2,
                },
                normal: n,
                area: diagArea,
              });
            }
          }
        }
      }
    }
  }

  // Area normalization — use isotropic scaling to avoid directional bias.
  // Per-axis scaling caused horizontal layer separation under gravity.
  if (normalizeAreas && bonds.length) {
    const totalHeight = stories * spacing.y;
    const totalWidth = side * spacing.x;
    const totalDepth = side * spacing.z;
    const target = {
      x: totalHeight * totalDepth,
      y: totalWidth * totalDepth,
      z: totalWidth * totalHeight,
    };
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

  return { nodes, bonds, gridCoordinates, spacing };
}
