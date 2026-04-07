/**
 * Proximity-based bond detection for irregular mesh fragments.
 *
 * Ported from vibe-city fracturedWallScenario.ts / fracturedGlbScenario.ts,
 * consolidated into a single reusable implementation with parameterized options
 * and an optional spatial broadphase for better average-case performance.
 */
import * as THREE from 'three';
import type { ScenarioBond, Vec3 } from '../rapier/types';

// ── Types ──────────────────────────────────────────────────────────────────

export type FragmentInfo = {
  worldPosition: Vec3;
  halfExtents: Vec3;
  geometry: THREE.BufferGeometry;
  isSupport: boolean;
};

export type BondDetectionOptions = {
  /** Factor applied to smallest fragment half-extent to derive global gap tolerance (default: 0.12) */
  toleranceFactor?: number;
  /** Minimum overlap ratio along each tangent axis (0–1). Pairs below this are rejected (default: 0.22) */
  minOverlapRatio?: number;
  /** Absolute minimum gap tolerance in world units (default: 0.006) */
  minGapTolerance?: number;
  /** Pair-relative gap scale: epsGap = max(minGapTolerance, min(globalTol, pairMin * pairGapScale)) (default: 0.15) */
  pairGapScale?: number;
  /** Skip bonds between two support fragments (default: true) */
  skipSupportToSupport?: boolean;
};

const DEFAULT_BOND_OPTIONS: Required<BondDetectionOptions> = {
  toleranceFactor: 0.12,
  minOverlapRatio: 0.22,
  minGapTolerance: 0.006,
  pairGapScale: 0.15,
  skipSupportToSupport: true,
};

// ── Helpers ────────────────────────────────────────────────────────────────

/** Project all vertices of `geometry` (offset by `worldPos`) onto `axis` and return min/max. */
export function projectExtentsOnAxisWorld(
  geometry: THREE.BufferGeometry,
  worldPos: Vec3,
  axis: Vec3,
): { min: number; max: number } {
  const pos = geometry.getAttribute('position') as THREE.BufferAttribute;
  let min = Infinity;
  let max = -Infinity;
  for (let i = 0; i < pos.count; i++) {
    const px = pos.getX(i) + worldPos.x;
    const py = pos.getY(i) + worldPos.y;
    const pz = pos.getZ(i) + worldPos.z;
    const p = px * axis.x + py * axis.y + pz * axis.z;
    if (p < min) min = p;
    if (p > max) max = p;
  }
  return { min, max };
}

function overlap1D(a: { min: number; max: number }, b: { min: number; max: number }): number {
  return Math.max(0, Math.min(a.max, b.max) - Math.max(a.min, b.min));
}

// ── Bond detection ─────────────────────────────────────────────────────────

/**
 * Detect bonds between irregular mesh fragments using proximity-based overlap analysis.
 *
 * For each pair of fragments, the algorithm:
 * 1. Computes the connecting normal (centroid-to-centroid direction)
 * 2. Projects both geometries onto the normal to check separation
 * 3. Projects onto two tangent axes to compute contact overlap area
 * 4. Rejects pairs that are too far apart or have insufficient overlap
 *
 * A spatial broadphase (sort-and-sweep on the X axis) skips obviously distant
 * pairs, giving O(n log n) average-case performance for spatially distributed fragments.
 */
export function computeBondsFromFragments(
  fragments: FragmentInfo[],
  options?: BondDetectionOptions,
): ScenarioBond[] {
  const opts = { ...DEFAULT_BOND_OPTIONS, ...options };
  const bonds: ScenarioBond[] = [];
  if (fragments.length === 0) return bonds;

  // Global gap tolerance derived from the smallest fragment half-extent
  const globalTol =
    opts.toleranceFactor *
    Math.min(
      ...fragments.map((f) => Math.min(f.halfExtents.x, f.halfExtents.z)),
    );

  // Broadphase: sort by X position and use max extent as sweep radius
  const maxExtent = Math.max(
    ...fragments.map((f) => Math.max(f.halfExtents.x, f.halfExtents.y, f.halfExtents.z)),
  );
  const sweepThreshold = maxExtent * 2 + globalTol;

  const sorted = fragments.map((f, i) => ({ index: i, x: f.worldPosition.x }));
  sorted.sort((a, b) => a.x - b.x);

  for (let si = 0; si < sorted.length; si++) {
    for (let sj = si + 1; sj < sorted.length; sj++) {
      // Broadphase early-exit: if X separation exceeds sweep threshold, skip rest
      if (sorted[sj].x - sorted[si].x > sweepThreshold) break;

      const i = sorted[si].index;
      const j = sorted[sj].index;
      const A = fragments[i];
      const B = fragments[j];

      if (opts.skipSupportToSupport && A.isSupport && B.isSupport) continue;

      // Connecting normal: A -> B
      const nx = B.worldPosition.x - A.worldPosition.x;
      const ny = B.worldPosition.y - A.worldPosition.y;
      const nz = B.worldPosition.z - A.worldPosition.z;
      const len = Math.hypot(nx, ny, nz);
      if (len === 0) continue;
      const n = new THREE.Vector3(nx / len, ny / len, nz / len);

      // Separation along the normal
      const aN = projectExtentsOnAxisWorld(A.geometry, A.worldPosition, n);
      const bN = projectExtentsOnAxisWorld(B.geometry, B.worldPosition, n);
      const separation = bN.min - aN.max;

      // Two tangent axes for overlap area
      const up = Math.abs(n.y) < 0.9
        ? new THREE.Vector3(0, 1, 0)
        : new THREE.Vector3(1, 0, 0);
      const t1 = new THREE.Vector3().crossVectors(n, up).normalize();
      const t2 = new THREE.Vector3().crossVectors(n, t1).normalize();

      const a1 = projectExtentsOnAxisWorld(A.geometry, A.worldPosition, t1);
      const b1 = projectExtentsOnAxisWorld(B.geometry, B.worldPosition, t1);
      const a2 = projectExtentsOnAxisWorld(A.geometry, A.worldPosition, t2);
      const b2 = projectExtentsOnAxisWorld(B.geometry, B.worldPosition, t2);

      const o1 = overlap1D(a1, b1);
      const o2 = overlap1D(a2, b2);
      const size1 = Math.min(a1.max - a1.min, b1.max - b1.min);
      const size2 = Math.min(a2.max - a2.min, b2.max - b2.min);

      // Pair-relative gap threshold
      const pairMin = Math.max(1e-6, Math.min(size1, size2));
      const epsGap = Math.max(opts.minGapTolerance, Math.min(globalTol, pairMin * opts.pairGapScale));
      if (separation > epsGap) continue;

      if (o1 < size1 * opts.minOverlapRatio || o2 < size2 * opts.minOverlapRatio) continue;

      const contactArea = o1 * o2;
      if (!(contactArea > 0)) continue;

      // Contact centroid: center of the overlap rectangle in (n, t1, t2) basis
      const cN = 0.5 * (Math.max(aN.min, bN.min) + Math.min(aN.max, bN.max));
      const c1 = 0.5 * (Math.max(a1.min, b1.min) + Math.min(a1.max, b1.max));
      const c2 = 0.5 * (Math.max(a2.min, b2.min) + Math.min(a2.max, b2.max));

      bonds.push({
        node0: i,
        node1: j,
        centroid: {
          x: n.x * cN + t1.x * c1 + t2.x * c2,
          y: n.y * cN + t1.y * c1 + t2.y * c2,
          z: n.z * cN + t1.z * c1 + t2.z * c2,
        },
        normal: { x: n.x, y: n.y, z: n.z },
        area: contactArea,
      });
    }
  }

  return bonds;
}

// ── Area normalization ─────────────────────────────────────────────────────

function pickDominantAxis(n: Vec3): 'x' | 'y' | 'z' {
  const ax = Math.abs(n.x);
  const ay = Math.abs(n.y);
  const az = Math.abs(n.z);
  return ax >= ay && ax >= az ? 'x' : ay >= az ? 'y' : 'z';
}

/**
 * Normalize bond areas per dominant axis so that the sum of areas along each axis
 * matches the expected cross-section of the given dimensions.
 *
 * For a box with dimensions {x, y, z}:
 * - Bonds with X-dominant normal should sum to y*z (the YZ cross-section)
 * - Bonds with Y-dominant normal should sum to x*z
 * - Bonds with Z-dominant normal should sum to x*y
 */
export function normalizeBondAreasByAxis(
  bonds: ScenarioBond[],
  dimensions: { x: number; y: number; z: number },
): ScenarioBond[] {
  const target = {
    x: dimensions.y * dimensions.z,
    y: dimensions.x * dimensions.z,
    z: dimensions.x * dimensions.y,
  };
  const sum = { x: 0, y: 0, z: 0 };
  for (const b of bonds) sum[pickDominantAxis(b.normal)] += b.area;

  const scale = {
    x: sum.x > 0 ? target.x / sum.x : 1,
    y: sum.y > 0 ? target.y / sum.y : 1,
    z: sum.z > 0 ? target.z / sum.z : 1,
  };

  return bonds.map((b) => {
    const axis = pickDominantAxis(b.normal);
    return { ...b, area: Math.max(b.area * scale[axis], 1e-8) };
  });
}

/**
 * Assign uniform bond areas within each dominant-axis group.
 * All bonds along a given axis get the same area = targetCrossSection / bondCount.
 */
export function uniformizeBondAreasByAxis(
  bonds: ScenarioBond[],
  dimensions: { x: number; y: number; z: number },
): ScenarioBond[] {
  const target = {
    x: dimensions.y * dimensions.z,
    y: dimensions.x * dimensions.z,
    z: dimensions.x * dimensions.y,
  };
  const counts = { x: 0, y: 0, z: 0 };
  for (const b of bonds) counts[pickDominantAxis(b.normal)] += 1;

  const areaPerAxis = {
    x: counts.x > 0 ? Math.max(1e-8, target.x / counts.x) : 0,
    y: counts.y > 0 ? Math.max(1e-8, target.y / counts.y) : 0,
    z: counts.z > 0 ? Math.max(1e-8, target.z / counts.z) : 0,
  };

  return bonds.map((b) => ({
    ...b,
    area: areaPerAxis[pickDominantAxis(b.normal)] || Math.max(1e-8, b.area),
  }));
}
