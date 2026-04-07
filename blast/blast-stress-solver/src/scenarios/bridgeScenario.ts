/**
 * Beam bridge scenario builder with destructible posts and footing supports.
 *
 * Ported from vibe-city beamBridgeScenario.ts — produces a ScenarioDesc with
 * no Three.js or RAPIER dependencies.
 */
import type { ScenarioBond, ScenarioDesc, ScenarioNode, Vec3 } from '../rapier/types';

const EPS = 1e-8;

export type BeamBridgeOptions = {
  span?: number;
  deckWidth?: number;
  deckThickness?: number;
  spanSegments?: number;
  widthSegments?: number;
  thicknessLayers?: number;
  deckMass?: number;
  pierHeight?: number;
  supportsPerSide?: number;
  supportWidthSegments?: number;
  supportDepthSegments?: number;
  footingThickness?: number;
  areaScale?: number;
  addDiagonals?: boolean;
  diagScale?: number;
  normalizeAreas?: boolean;
  bondsX?: boolean;
  bondsY?: boolean;
  bondsZ?: boolean;
};

export const DEFAULT_BRIDGE_OPTIONS: Required<BeamBridgeOptions> = {
  span: 18.0,
  deckWidth: 5.0,
  deckThickness: 0.6,
  spanSegments: 30,
  widthSegments: 10,
  thicknessLayers: 2,
  deckMass: 60_000,
  pierHeight: 2.8,
  supportsPerSide: 4,
  supportWidthSegments: 2,
  supportDepthSegments: 2,
  footingThickness: 0.12,
  areaScale: 0.05,
  addDiagonals: true,
  diagScale: 0.6,
  normalizeAreas: true,
  bondsX: true,
  bondsY: true,
  bondsZ: true,
};

function v(x: number, y: number, z: number): Vec3 {
  return { x, y, z };
}
function sub(a: Vec3, b: Vec3): Vec3 {
  return v(a.x - b.x, a.y - b.y, a.z - b.z);
}
function nrm(p: Vec3): Vec3 {
  const L = Math.hypot(p.x, p.y, p.z);
  if (L <= EPS) return v(0, 0, 0);
  return v(p.x / L, p.y / L, p.z / L);
}

export function buildBeamBridgeScenario(opts: BeamBridgeOptions = {}): ScenarioDesc {
  const {
    span, deckWidth, deckThickness,
    spanSegments: rawSegX, widthSegments: rawSegZ, thicknessLayers: rawSegY,
    deckMass, pierHeight,
    supportsPerSide, supportWidthSegments, supportDepthSegments, footingThickness,
    areaScale, addDiagonals, diagScale, normalizeAreas,
    bondsX, bondsY, bondsZ,
  } = { ...DEFAULT_BRIDGE_OPTIONS, ...opts };

  const segX = Math.max(1, Math.floor(rawSegX));
  const segY = Math.max(1, Math.floor(rawSegY));
  const segZ = Math.max(1, Math.floor(rawSegZ));

  const cellX = span / segX;
  const cellY = deckThickness / segY;
  const cellZ = deckWidth / segZ;

  const postLayers = Math.max(1, Math.ceil(pierHeight / cellY));
  const deckBottomY = postLayers * cellY;
  const deckOrigin = v(
    -span * 0.5 + 0.5 * cellX,
    deckBottomY + 0.5 * cellY,
    -deckWidth * 0.5 + 0.5 * cellZ,
  );

  const gridDeck: number[][][] = Array.from({ length: segX }, () =>
    Array.from({ length: segY }, () => Array.from({ length: segZ }, () => -1)),
  );

  const nodes: ScenarioNode[] = [];
  const fragmentSizes: Vec3[] = [];
  const gridCoordinates: Array<{ ix: number; iy: number; iz: number }> = [];

  // Build deck nodes
  const deckCellVolume = cellX * cellY * cellZ;
  let deckTotalVolume = 0;
  for (let ix = 0; ix < segX; ix++) {
    for (let iy = 0; iy < segY; iy++) {
      for (let iz = 0; iz < segZ; iz++) {
        const p = v(
          deckOrigin.x + ix * cellX,
          deckOrigin.y + iy * cellY,
          deckOrigin.z + iz * cellZ,
        );
        const idx = nodes.length;
        nodes.push({ centroid: p, mass: deckCellVolume, volume: deckCellVolume });
        fragmentSizes.push({ x: cellX, y: cellY, z: cellZ });
        gridDeck[ix][iy][iz] = idx;
        gridCoordinates[idx] = { ix, iy, iz };
        deckTotalVolume += deckCellVolume;
      }
    }
  }

  // Scale masses so total deck mass matches
  const massScale = deckTotalVolume > 0 ? deckMass / deckTotalVolume : 0;
  if (massScale !== 1) {
    for (const n of nodes) if (n.volume > 0) n.mass = n.volume * massScale;
  }

  // Bond helpers
  const bonds: ScenarioBond[] = [];
  const areaX = cellY * cellZ * areaScale;
  const areaY = cellX * cellZ * areaScale;
  const areaZ = cellX * cellY * areaScale;
  const addBond = (a: number, b: number, area: number) => {
    if (a < 0 || b < 0) return;
    const na = nodes[a];
    const nb = nodes[b];
    const c = v(
      (na.centroid.x + nb.centroid.x) * 0.5,
      (na.centroid.y + nb.centroid.y) * 0.5,
      (na.centroid.z + nb.centroid.z) * 0.5,
    );
    const n = nrm(sub(nb.centroid, na.centroid));
    bonds.push({ node0: a, node1: b, centroid: c, normal: n, area: Math.max(area, EPS) });
  };

  // Deck connectivity
  for (let ix = 0; ix < segX; ix++) {
    for (let iy = 0; iy < segY; iy++) {
      for (let iz = 0; iz < segZ; iz++) {
        const cur = gridDeck[ix][iy][iz];
        if (cur < 0) continue;
        if (bondsX && ix + 1 < segX) addBond(cur, gridDeck[ix + 1][iy][iz], areaX);
        if (bondsY && iy + 1 < segY) addBond(cur, gridDeck[ix][iy + 1][iz], areaY);
        if (bondsZ && iz + 1 < segZ) addBond(cur, gridDeck[ix][iy][iz + 1], areaZ);
        if (addDiagonals) {
          if (bondsX && bondsZ && ix + 1 < segX && iz + 1 < segZ)
            addBond(cur, gridDeck[ix + 1][iy][iz + 1], 0.5 * (areaX + areaZ) * diagScale);
          if (bondsX && bondsY && ix + 1 < segX && iy + 1 < segY)
            addBond(cur, gridDeck[ix + 1][iy + 1][iz], 0.5 * (areaX + areaY) * diagScale);
          if (bondsY && bondsZ && iy + 1 < segY && iz + 1 < segZ)
            addBond(cur, gridDeck[ix][iy + 1][iz + 1], 0.5 * (areaY + areaZ) * diagScale);
        }
      }
    }
  }

  // Build posts under first and last span columns
  const postXCols = [0, segX - 1];
  const postTopYLayer = 0;
  const postTopY = deckOrigin.y - 0.5 * cellY;

  const clamp = (val: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, val));
  const postSpan = Math.max(1, supportsPerSide);
  const slots: number[] = [];
  for (let i = 0; i < postSpan; i++) {
    const t = postSpan === 1 ? 0.5 : i / (postSpan - 1);
    slots.push(clamp(Math.round(t * (segZ - 1)), 0, segZ - 1));
  }

  const uniq = (arr: number[]) => Array.from(new Set(arr));

  for (const ixEdge of postXCols) {
    const ixCover = uniq(
      Array.from({ length: supportDepthSegments }, (_, k) =>
        clamp(ixEdge + (ixEdge === 0 ? k : -k), 0, segX - 1),
      ),
    );
    const ixCoverSet = new Set(ixCover);

    for (const baseZ of slots) {
      const coverZ = uniq(
        Array.from({ length: supportWidthSegments }, (_, k) =>
          clamp(baseZ + k - Math.floor((supportWidthSegments - 1) / 2), 0, segZ - 1),
        ),
      );
      const coverZSet = new Set(coverZ);
      const postMap = new Map<string, number>();
      const key = (ixp: number, py: number, iz: number) => `${ixp}|${py}|${iz}`;

      // Create stacks
      for (const iz of coverZ) {
        for (const ixp of ixCover) {
          for (let py = 0; py < postLayers; py++) {
            const yCenter = postTopY - py * cellY - 0.5 * cellY;
            const nodeIdx = nodes.length;
            const p = v(deckOrigin.x + ixp * cellX, yCenter, deckOrigin.z + iz * cellZ);
            const volume = cellX * cellY * cellZ;
            nodes.push({ centroid: p, mass: volume * massScale, volume });
            fragmentSizes.push({ x: cellX, y: cellY, z: cellZ });
            const gy = -1 - py;
            gridCoordinates[nodeIdx] = { ix: ixp, iy: gy, iz };
            postMap.set(key(ixp, py, iz), nodeIdx);

            if (py > 0) {
              const prevIdx = postMap.get(key(ixp, py - 1, iz));
              if (prevIdx != null) addBond(prevIdx, nodeIdx, areaY);
            } else {
              const deckIndex = gridDeck[ixp][postTopYLayer][iz];
              addBond(nodeIdx, deckIndex, areaY);
            }
          }

          // Footing under this column (mass=0 support)
          const footCenterY = postTopY - postLayers * cellY - 0.5 * footingThickness;
          const fIdx = nodes.length;
          const fPos = v(deckOrigin.x + ixp * cellX, footCenterY, deckOrigin.z + iz * cellZ);
          nodes.push({ centroid: fPos, mass: 0, volume: 0 });
          fragmentSizes.push({ x: cellX, y: Math.max(footingThickness, EPS), z: cellZ });
          gridCoordinates[fIdx] = { ix: ixp, iy: -1 - postLayers, iz };
          const lowestPostIdx = postMap.get(key(ixp, postLayers - 1, iz));
          if (lowestPostIdx != null) addBond(fIdx, lowestPostIdx, areaY);
        }
      }

      // Lateral bonds within the post cluster
      for (const iz of coverZ) {
        for (const ixp of ixCover) {
          for (let py = 0; py < postLayers; py++) {
            const cur = postMap.get(key(ixp, py, iz));
            if (cur == null) continue;
            const nx = ixEdge === 0 ? ixp + 1 : ixp - 1;
            if (ixCoverSet.has(nx)) {
              const nb = postMap.get(key(nx, py, iz));
              if (nb != null) addBond(cur, nb, areaX);
            }
            const nz = iz + 1;
            if (coverZSet.has(nz)) {
              const nbz = postMap.get(key(ixp, py, nz));
              if (nbz != null) addBond(cur, nbz, areaZ);
            }
          }
        }
      }
    }
  }

  // Per-axis area normalization
  if (normalizeAreas && bonds.length) {
    const size = { x: span, y: deckThickness + pierHeight + footingThickness, z: deckWidth };
    const target = { x: size.y * size.z, y: size.x * size.z, z: size.x * size.y };
    const sum = { x: 0, y: 0, z: 0 };
    const pick = (n: Vec3): 'x' | 'y' | 'z' => {
      const ax = Math.abs(n.x), ay = Math.abs(n.y), az = Math.abs(n.z);
      return ax >= ay && ax >= az ? 'x' : (ay >= az ? 'y' : 'z');
    };
    for (const b of bonds) sum[pick(b.normal)] += b.area;
    const scale = {
      x: sum.x > 0 ? target.x / sum.x : 1,
      y: sum.y > 0 ? target.y / sum.y : 1,
      z: sum.z > 0 ? target.z / sum.z : 1,
    };
    for (const b of bonds) b.area *= scale[pick(b.normal)];
  }

  return {
    nodes,
    bonds,
    gridCoordinates,
    spacing: v(cellX, cellY, cellZ),
    parameters: {
      span, deckWidth, deckThickness, deckMass, pierHeight,
      supportsPerSide, supportWidthSegments, supportDepthSegments, footingThickness,
      areaScale, addDiagonals, diagScale,
      fragmentSizes,
    },
  };
}
