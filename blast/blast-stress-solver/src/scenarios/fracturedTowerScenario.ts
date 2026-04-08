/**
 * Pre-built fractured tower scenario using Voronoi fracturing.
 *
 * Builds a multi-floor tower from fractured walls, floor plates, interior
 * columns, and a grid foundation. Each component is independently fractured
 * via three-pinata, then assembled into a single ScenarioDesc with bond
 * detection and type-based bond strength multipliers.
 *
 * Realistic skyscraper defaults:
 * - 40m x 40m footprint (typical mid-rise office building)
 * - 4m floor-to-floor height (commercial standard)
 * - 20 floors (~80m total height)
 * - Interior column grid (~8m spacing, auto-calculated)
 * - ~600 kg/m^2 per floor (reinforced concrete)
 *
 * Requires @dgreenheck/three-pinata as an optional peer dependency.
 */
import type { ScenarioDesc } from '../rapier/types';
import type { FragmentInfo, FragmentType } from '../three/fracture';
import type { PinataModule } from '../three/pinataFracture';
import type { AreaNormalizationMode } from '../three/scenarioFromFragments';
import {
  buildWallFragments,
  buildFloorFragments,
  buildColumnFragments,
  buildGridFoundationFragments,
  applyBondStrengthMultipliers,
} from '../three/fractureBuilders';
import {
  buildScenarioFromFragments,
  buildScenarioFromFragmentsAsync,
} from '../three/scenarioFromFragments';

export type FracturedTowerOptions = {
  /** Width of the tower footprint (X dimension) in meters (default: 40) */
  width?: number;
  /** Depth of the tower footprint (Z dimension) in meters (default: 40) */
  depth?: number;
  /** Number of floors / stories (default: 20) */
  floorCount?: number;
  /** Floor-to-floor height in meters (default: 4) */
  floorHeight?: number;
  /** Total height override — if provided, ignores floorCount x floorHeight */
  height?: number;
  /** Exterior wall thickness in meters (default: 0.4) */
  thickness?: number;
  /** Floor slab thickness in meters (default: 0.35) */
  floorThickness?: number;
  /** Interior column cross-section size in meters (default: 1.2) */
  columnSize?: number;
  /** Number of columns in X direction. Auto-calculated from columnSpacing if omitted. */
  columnsX?: number;
  /** Number of columns in Z direction. Auto-calculated from columnSpacing if omitted. */
  columnsZ?: number;
  /** Target spacing between columns in meters (default: 8) */
  columnSpacing?: number;
  /** Inset from walls to first column row (fraction 0–0.5, default: 0.15) */
  columnInset?: number;
  /** Voronoi fragment count per wall section per floor (default: 12) */
  fragmentCountPerWall?: number;
  /** Voronoi fragment count per floor plate (default: 8) */
  fragmentCountPerFloor?: number;
  /** Voronoi fragment count per column section (default: 4) */
  fragmentCountPerColumn?: number;
  /** Total structure mass in kg. Auto-calculated (~600 kg/m^2 per floor) if omitted. */
  deckMass?: number;
  /** Bond detection mode: 'proximity' (default) or 'auto' (WASM triangle-based) */
  bondMode?: 'proximity' | 'auto';
  /** Bond area normalization mode (default: 'perAxis') */
  areaNormalization?: AreaNormalizationMode;
  /**
   * Pre-imported three-pinata module. Required in browser ESM environments where
   * the scenarios bundle cannot dynamically import bare specifiers.
   * @example
   * ```ts
   * import * as pinata from '@dgreenheck/three-pinata';
   * buildFracturedTowerScenario({ pinata });
   * ```
   */
  pinata?: PinataModule;
  /** Rapier module for convex hull colliders. If omitted, colliderDescForNode entries are null. */
  rapier?: { ColliderDesc: { cuboid(hx: number, hy: number, hz: number): unknown; convexHull(points: Float32Array): unknown | null } };
};

/**
 * Build a multi-floor fractured tower scenario.
 *
 * Each floor section consists of 4 exterior walls, interior columns,
 * and a floor plate. The ground floor sits on a grid foundation;
 * upper walls sit on floor plates to ensure proper bonding surfaces.
 */
export async function buildFracturedTowerScenario(
  options?: FracturedTowerOptions,
): Promise<ScenarioDesc> {
  const {
    width = 40,
    depth = 40,
    floorCount = 20,
    floorHeight = 4,
    height,
    thickness = 0.4,
    floorThickness = 0.35,
    columnSize = 1.2,
    columnsX,
    columnsZ,
    columnSpacing = 8,
    columnInset = 0.15,
    fragmentCountPerWall = 12,
    fragmentCountPerFloor = 8,
    fragmentCountPerColumn = 4,
    deckMass,
    bondMode = 'proximity',
    areaNormalization = 'perAxis',
    pinata,
    rapier,
  } = options ?? {};

  const totalHeight = height ?? floorCount * floorHeight;

  // ~600 kg/m^2 per floor (typical reinforced concrete office building)
  const floorArea = width * depth;
  const calculatedMass = deckMass ?? floorArea * floorCount * 600;

  // Foundation
  const foundationHeight = Math.max(0.5, totalHeight * 0.01);
  const groundClearance = Math.max(0.01, foundationHeight * 0.05);
  const baseY = groundClearance + foundationHeight;

  // Floor level Y coordinates
  const floorHeights: number[] = [];
  for (let i = 0; i <= floorCount; i++) {
    floorHeights.push(baseY + i * floorHeight);
  }

  const halfWidth = width * 0.5;
  const halfDepth = depth * 0.5;

  // Interior column grid
  const interiorWidth = width - thickness * 2;
  const interiorDepth = depth - thickness * 2;
  const numColumnsX = columnsX ?? Math.max(2, Math.round(interiorWidth / columnSpacing));
  const numColumnsZ = columnsZ ?? Math.max(2, Math.round(interiorDepth / columnSpacing));

  const columnRangeX = interiorWidth * (1 - 2 * columnInset);
  const columnRangeZ = interiorDepth * (1 - 2 * columnInset);
  const columnStartX = -columnRangeX * 0.5;
  const columnStartZ = -columnRangeZ * 0.5;

  const columnPositions: Array<{ x: number; z: number }> = [];
  for (let ix = 0; ix < numColumnsX; ix++) {
    for (let iz = 0; iz < numColumnsZ; iz++) {
      const tx = numColumnsX > 1 ? ix / (numColumnsX - 1) : 0.5;
      const tz = numColumnsZ > 1 ? iz / (numColumnsZ - 1) : 0.5;
      columnPositions.push({
        x: columnStartX + tx * columnRangeX,
        z: columnStartZ + tz * columnRangeZ,
      });
    }
  }

  // ── Collect all fragments ──────────────────────────────────
  const allFragments: FragmentInfo[] = [];

  // Walls + columns for each floor section
  for (let floorIdx = 0; floorIdx < floorHeights.length - 1; floorIdx++) {
    const wallBottomY = floorIdx === 0
      ? floorHeights[0]
      : floorHeights[floorIdx] + floorThickness * 0.5;
    const wallTopY = floorHeights[floorIdx + 1] - floorThickness * 0.5;
    const wallHeight = wallTopY - wallBottomY;
    if (wallHeight <= 0.1) continue;

    const sideWallSpan = depth - thickness * 2;

    // Front wall
    allFragments.push(...buildWallFragments({
      span: width, height: wallHeight, thickness,
      fragmentCount: fragmentCountPerWall,
      centerX: 0, centerZ: -halfDepth + thickness * 0.5,
      rotationY: 0, baseY: wallBottomY, pinata,
    }));

    // Back wall
    allFragments.push(...buildWallFragments({
      span: width, height: wallHeight, thickness,
      fragmentCount: fragmentCountPerWall,
      centerX: 0, centerZ: halfDepth - thickness * 0.5,
      rotationY: 0, baseY: wallBottomY, pinata,
    }));

    // Left wall (rotated 90 degrees)
    allFragments.push(...buildWallFragments({
      span: sideWallSpan, height: wallHeight, thickness,
      fragmentCount: fragmentCountPerWall,
      centerX: -halfWidth + thickness * 0.5, centerZ: 0,
      rotationY: Math.PI * 0.5, baseY: wallBottomY, pinata,
    }));

    // Right wall (rotated 90 degrees)
    allFragments.push(...buildWallFragments({
      span: sideWallSpan, height: wallHeight, thickness,
      fragmentCount: fragmentCountPerWall,
      centerX: halfWidth - thickness * 0.5, centerZ: 0,
      rotationY: Math.PI * 0.5, baseY: wallBottomY, pinata,
    }));

    // Interior columns for this floor section
    for (const colPos of columnPositions) {
      allFragments.push(...buildColumnFragments({
        sizeX: columnSize, sizeZ: columnSize, height: wallHeight,
        fragmentCount: fragmentCountPerColumn,
        centerX: colPos.x, baseY: wallBottomY, centerZ: colPos.z, pinata,
      }));
    }
  }

  // Floor plates (skip ground floor — covered by foundation)
  for (let floorIdx = 1; floorIdx < floorHeights.length - 1; floorIdx++) {
    allFragments.push(...buildFloorFragments({
      spanX: width, spanZ: depth, thickness: floorThickness,
      fragmentCount: fragmentCountPerFloor,
      centerX: 0, centerY: floorHeights[floorIdx], centerZ: 0, pinata,
    }));
  }

  // Roof plate at the top
  const lastWallTopY = floorHeights[floorCount] - floorThickness * 0.5;
  const roofY = lastWallTopY + floorThickness * 0.5;
  allFragments.push(...buildFloorFragments({
    spanX: width, spanZ: depth, thickness: floorThickness,
    fragmentCount: fragmentCountPerFloor,
    centerX: 0, centerY: roofY, centerZ: 0, pinata,
  }));

  // Grid foundation
  const { fragments: foundationFragments } = buildGridFoundationFragments({
    width, depth,
    height: foundationHeight,
    groundClearance,
  });
  allFragments.push(...foundationFragments);

  // ── Build scenario ─────────────────────────────────────────
  const dims = { x: width, y: totalHeight, z: depth };

  const scenarioOptions = {
    totalMass: calculatedMass,
    bondMode: bondMode as 'proximity' | 'auto',
    areaNormalization,
    dimensions: dims,
    rapier,
  };

  const scenario = bondMode === 'auto'
    ? await buildScenarioFromFragmentsAsync(allFragments, scenarioOptions)
    : buildScenarioFromFragments(allFragments, scenarioOptions);

  // Apply type-based bond strength multipliers
  const fragmentTypes: (FragmentType | undefined)[] = allFragments.map((f) => f.fragmentType);
  scenario.bonds = applyBondStrengthMultipliers(scenario.bonds, fragmentTypes);

  // Store metadata
  scenario.parameters = {
    ...scenario.parameters,
    width, depth, height: totalHeight,
    thickness, floorThickness, columnSize,
    columnsX: numColumnsX, columnsZ: numColumnsZ,
    columnSpacing, columnInset,
    totalColumns: columnPositions.length,
    floorCount, floorHeight,
    fragmentCountPerWall, fragmentCountPerFloor, fragmentCountPerColumn,
  };

  return scenario;
}
