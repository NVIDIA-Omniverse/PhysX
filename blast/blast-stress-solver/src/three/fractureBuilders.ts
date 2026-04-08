/**
 * Reusable fragment builder functions for multi-component fractured structures.
 *
 * Each builder fractures a box geometry (wall panel, floor slab, column) using
 * three-pinata Voronoi tessellation, then transforms fragments to world space.
 * All builders produce FragmentInfo[] arrays compatible with buildScenarioFromFragments().
 *
 * Improvements over vibe-city's fractureUtils.ts:
 * - Uses existing fractureGeometry() with proper pinata module management
 * - Plain Vec3 positions (not THREE.Vector3)
 * - Consistent geometry preparation (ensurePlainAttributes, recenterGeometry)
 */
import * as THREE from 'three';
import type { Vec3, ScenarioBond } from '../rapier/types';
import type { FragmentInfo, FragmentType } from './fracture';
import { fractureGeometry, type FractureGeometryOptions } from './pinataFracture';

// ── Fragment builders ─────────────────────────────────────────────────────

export type WallFragmentOptions = {
  /** Wall span along its local X axis (m) */
  span: number;
  /** Wall height along Y (m) */
  height: number;
  /** Wall thickness along its local Z axis (m) */
  thickness: number;
  /** Number of Voronoi fragments */
  fragmentCount: number;
  /** World X position of wall center */
  centerX?: number;
  /** World Z position of wall center */
  centerZ?: number;
  /** Rotation around Y axis (radians). 0 = wall spans X, PI/2 = wall spans Z */
  rotationY?: number;
  /** Y position of wall bottom edge */
  baseY?: number;
  /** Minimum half-extent for fragments (default: 0.05) */
  minHalfExtent?: number;
  /** Pre-loaded pinata module (optional) */
  pinata?: FractureGeometryOptions['pinata'];
};

/**
 * Fracture a wall panel and position fragments in world space.
 *
 * Creates a box geometry (span x height x thickness), fractures it,
 * then applies rotation and translation to place fragments at the target position.
 */
export function buildWallFragments(options: WallFragmentOptions): FragmentInfo[] {
  const {
    span,
    height,
    thickness,
    fragmentCount,
    centerX = 0,
    centerZ = 0,
    rotationY = 0,
    baseY = 0,
    minHalfExtent = 0.05,
    pinata,
  } = options;

  const geometry = new THREE.BoxGeometry(span, height, thickness, 2, 3, 1);

  const fragments = fractureGeometry(geometry, {
    fragmentCount,
    worldOffset: { x: 0, y: 0, z: 0 },
    minHalfExtent,
    pinata,
  });
  geometry.dispose();

  // Wall center: (centerX, baseY + height/2, centerZ)
  const cy = height * 0.5 + baseY;

  if (Math.abs(rotationY) < 0.001) {
    // No rotation: just translate
    return fragments.map((f) => ({
      ...f,
      worldPosition: {
        x: f.worldPosition.x + centerX,
        y: f.worldPosition.y + cy,
        z: f.worldPosition.z + centerZ,
      },
      fragmentType: 'wall' as const,
    }));
  }

  // Apply Y-axis rotation
  const rotMatrix = new THREE.Matrix4().makeRotationY(rotationY);
  const cos = Math.abs(Math.cos(rotationY));
  const sin = Math.abs(Math.sin(rotationY));

  return fragments.map((f) => {
    // Rotate fragment position offset around Y
    const v = new THREE.Vector3(f.worldPosition.x, f.worldPosition.y, f.worldPosition.z);
    v.applyMatrix4(rotMatrix);

    // Rotate geometry to match
    f.geometry.applyMatrix4(rotMatrix);

    // Recompute bounding box after rotation
    f.geometry.computeBoundingBox();
    const size = new THREE.Vector3();
    (f.geometry.boundingBox as THREE.Box3).getSize(size);

    return {
      worldPosition: {
        x: v.x + centerX,
        y: v.y + cy,
        z: v.z + centerZ,
      },
      halfExtents: {
        x: Math.max(minHalfExtent, size.x * 0.5),
        y: Math.max(minHalfExtent, size.y * 0.5),
        z: Math.max(minHalfExtent, size.z * 0.5),
      },
      geometry: f.geometry,
      isSupport: false,
      fragmentType: 'wall' as const,
    };
  });
}

export type FloorFragmentOptions = {
  /** Floor span along X (m) */
  spanX: number;
  /** Floor span along Z (m) */
  spanZ: number;
  /** Floor thickness along Y (m) */
  thickness: number;
  /** Number of Voronoi fragments */
  fragmentCount: number;
  /** World X position of floor center */
  centerX?: number;
  /** World Y position of floor center */
  centerY?: number;
  /** World Z position of floor center */
  centerZ?: number;
  /** Minimum half-extent for fragments (default: 0.05) */
  minHalfExtent?: number;
  /** Pre-loaded pinata module (optional) */
  pinata?: FractureGeometryOptions['pinata'];
};

/**
 * Fracture a horizontal floor slab and position fragments in world space.
 */
export function buildFloorFragments(options: FloorFragmentOptions): FragmentInfo[] {
  const {
    spanX,
    spanZ,
    thickness,
    fragmentCount,
    centerX = 0,
    centerY = 0,
    centerZ = 0,
    minHalfExtent = 0.05,
    pinata,
  } = options;

  const geometry = new THREE.BoxGeometry(spanX, thickness, spanZ, 3, 1, 3);

  const fragments = fractureGeometry(geometry, {
    fragmentCount,
    worldOffset: { x: centerX, y: centerY, z: centerZ },
    minHalfExtent,
    pinata,
  });
  geometry.dispose();

  return fragments.map((f) => ({
    ...f,
    fragmentType: 'floor' as const,
  }));
}

export type ColumnFragmentOptions = {
  /** Column cross-section X size (m) */
  sizeX: number;
  /** Column cross-section Z size (m) */
  sizeZ: number;
  /** Column height along Y (m) */
  height: number;
  /** Number of Voronoi fragments */
  fragmentCount: number;
  /** World X position of column center */
  centerX?: number;
  /** Y position of column bottom edge */
  baseY?: number;
  /** World Z position of column center */
  centerZ?: number;
  /** Minimum half-extent for fragments (default: 0.05) */
  minHalfExtent?: number;
  /** Pre-loaded pinata module (optional) */
  pinata?: FractureGeometryOptions['pinata'];
};

/**
 * Fracture a vertical column and position fragments in world space.
 */
export function buildColumnFragments(options: ColumnFragmentOptions): FragmentInfo[] {
  const {
    sizeX,
    sizeZ,
    height,
    fragmentCount,
    centerX = 0,
    baseY = 0,
    centerZ = 0,
    minHalfExtent = 0.05,
    pinata,
  } = options;

  const geometry = new THREE.BoxGeometry(sizeX, height, sizeZ, 1, 3, 1);

  // Column center is at baseY + height/2
  const cy = baseY + height * 0.5;
  const fragments = fractureGeometry(geometry, {
    fragmentCount,
    worldOffset: { x: centerX, y: cy, z: centerZ },
    minHalfExtent,
    pinata,
  });
  geometry.dispose();

  return fragments.map((f) => ({
    ...f,
    fragmentType: 'column' as const,
  }));
}

// ── Grid foundation ───────────────────────────────────────────────────────

export type GridFoundationOptions = {
  /** Foundation footprint width (X) in meters */
  width: number;
  /** Foundation footprint depth (Z) in meters */
  depth: number;
  /** Foundation height (Y) in meters */
  height: number;
  /** Gap between foundation bottom and ground plane (default: 0.001) */
  groundClearance?: number;
  /** Target tile size for foundation grid. Auto-calculated if omitted (~6-8 tiles per side). */
  tileSize?: number;
};

/**
 * Create a 2D grid of support tile fragments forming a rectangular foundation.
 *
 * Each tile is a simple box geometry with isSupport=true (mass=0 in the solver).
 * The tile count scales with structure size to maintain reasonable coverage.
 *
 * @returns Array of support FragmentInfo and the Y coordinate of the foundation top surface.
 */
export function buildGridFoundationFragments(
  options: GridFoundationOptions,
): { fragments: FragmentInfo[]; foundationTopY: number } {
  const {
    width,
    depth,
    height,
    groundClearance = 0.001,
  } = options;

  const tileSize = options.tileSize ?? Math.max(1.0, Math.min(width, depth) / 6);
  const segmentsX = Math.max(4, Math.round(width / tileSize));
  const segmentsZ = Math.max(4, Math.round(depth / tileSize));
  const cellW = width / segmentsX;
  const cellD = depth / segmentsZ;

  const halfWidth = width * 0.5;
  const halfDepth = depth * 0.5;
  const foundationTopY = groundClearance + height;

  const fragments: FragmentInfo[] = [];
  for (let ix = 0; ix < segmentsX; ix++) {
    for (let iz = 0; iz < segmentsZ; iz++) {
      const cx = -halfWidth + cellW * (ix + 0.5);
      const cz = -halfDepth + cellD * (iz + 0.5);
      const cy = groundClearance + height * 0.5;

      fragments.push({
        worldPosition: { x: cx, y: cy, z: cz },
        halfExtents: { x: cellW * 0.5, y: height * 0.5, z: cellD * 0.5 },
        geometry: new THREE.BoxGeometry(cellW, height, cellD),
        isSupport: true,
        fragmentType: 'foundation' as const,
      });
    }
  }

  return { fragments, foundationTopY };
}

// ── Bond strength multipliers ─────────────────────────────────────────────

/**
 * Bond strength multiplier based on the structural roles of the two connected fragments.
 *
 * Strength hierarchy (strongest to weakest):
 * 1. column-column: 4.0x  (primary load-bearing)
 * 2. column-anything: 2.5x (column connections)
 * 3. floor-floor: 2.0x     (slab continuity)
 * 4. floor-wall: 1.5x      (wall-to-slab joints)
 * 5. wall-wall: 1.0x       (baseline)
 */
export function getBondStrengthMultiplier(
  type0: FragmentType | undefined,
  type1: FragmentType | undefined,
): number {
  if (type0 === 'column' && type1 === 'column') return 4.0;
  if (type0 === 'column' || type1 === 'column') return 2.5;
  if (type0 === 'floor' && type1 === 'floor') return 2.0;
  if ((type0 === 'floor' && type1 === 'wall') || (type0 === 'wall' && type1 === 'floor')) return 1.5;
  return 1.0;
}

/**
 * Apply type-based bond strength multipliers by scaling bond areas.
 *
 * Larger area → lower stress → harder to break, so multiplying the area
 * of column-column bonds by 4x makes them 4x stronger than wall-wall bonds.
 *
 * @param bonds - The bond array to modify (mutated in place and returned)
 * @param fragmentTypes - Per-node fragment type array (indexed by node index)
 */
export function applyBondStrengthMultipliers(
  bonds: ScenarioBond[],
  fragmentTypes: (FragmentType | undefined)[],
): ScenarioBond[] {
  return bonds.map((bond) => {
    const multiplier = getBondStrengthMultiplier(
      fragmentTypes[bond.node0],
      fragmentTypes[bond.node1],
    );
    if (multiplier === 1.0) return bond;
    return { ...bond, area: bond.area * multiplier };
  });
}
