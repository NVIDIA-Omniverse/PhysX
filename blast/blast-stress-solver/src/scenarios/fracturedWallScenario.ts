/**
 * Pre-built fractured wall scenario using Voronoi fracturing.
 *
 * This scenario fractures a box geometry into irregular fragments using
 * three-pinata, generates a foundation slab, and produces a complete
 * ScenarioDesc ready for the destructible-core runtime.
 *
 * Requires @dgreenheck/three-pinata as an optional peer dependency.
 */
import * as THREE from 'three';
import type { ScenarioDesc } from '../rapier/types';
import {
  buildFracturedScenario,
  buildFracturedScenarioAsync,
  type FracturedScenarioOptions,
} from '../three/pinataFracture';
import type { AreaNormalizationMode } from '../three/scenarioFromFragments';
import type { AutoBondingRequest } from '../three/autoBonding';

export type FracturedWallOptions = {
  /** Wall width along X axis (default: 6.0) */
  span?: number;
  /** Wall height along Y axis (default: 3.0) */
  height?: number;
  /** Wall depth along Z axis (default: 0.32) */
  thickness?: number;
  /** Number of Voronoi fragments (default: 120) */
  fragmentCount?: number;
  /** Total mass for all non-support nodes (default: 10000) */
  deckMass?: number;
  /** Bond detection mode: 'proximity' (default) or 'auto' (WASM triangle-based) */
  bondMode?: 'proximity' | 'auto';
  /** Bond area normalization mode (default: 'perAxis') */
  areaNormalization?: AreaNormalizationMode;
  /** Options forwarded to WASM auto-bonding (used only when `bondMode: 'auto'`) */
  autoBondingOptions?: Omit<AutoBondingRequest, 'enabled'>;
  /** Rapier module reference for convex hull colliders */
  rapier?: { ColliderDesc: { cuboid(hx: number, hy: number, hz: number): unknown; convexHull(points: Float32Array): unknown | null } };
};

export const DEFAULT_FRACTURED_WALL_OPTIONS: Required<
  Omit<FracturedWallOptions, 'rapier' | 'bondMode' | 'autoBondingOptions'>
> = {
  span: 6.0,
  height: 3.0,
  thickness: 0.32,
  fragmentCount: 120,
  deckMass: 10_000,
  areaNormalization: 'perAxis',
};

/**
 * Build a fractured wall scenario.
 *
 * Creates a box geometry, fractures it via Voronoi tessellation,
 * adds a segmented foundation slab, detects bonds between fragments,
 * normalizes bond areas, and returns a complete ScenarioDesc.
 */
export function buildFracturedWallScenario(
  options?: FracturedWallOptions,
): ScenarioDesc {
  const opts = { ...DEFAULT_FRACTURED_WALL_OPTIONS, ...options };
  const { span, height, thickness, fragmentCount, deckMass, areaNormalization } = opts;

  const geometry = new THREE.BoxGeometry(span, height, thickness, 2, 3, 1);

  const fractureOptions: FracturedScenarioOptions = {
    fragmentCount,
    voronoiMode: '3D',
    totalMass: deckMass,
    areaNormalization,
    dimensions: { x: span, y: height, z: thickness },
    rapier: opts.rapier,
    worldOffset: { x: 0, y: height * 0.5, z: 0 },
    foundation: {
      enabled: true,
      heightRatio: 0.06,
      maxHeight: 0.08,
    },
  };

  const scenario = buildFracturedScenario(geometry, fractureOptions);
  geometry.dispose();

  return scenario;
}

/**
 * Async variant that supports public WASM auto-bonding via `bondMode: 'auto'`.
 */
export async function buildFracturedWallScenarioAsync(
  options?: FracturedWallOptions,
): Promise<ScenarioDesc> {
  const opts = { ...DEFAULT_FRACTURED_WALL_OPTIONS, ...options };
  const {
    span,
    height,
    thickness,
    fragmentCount,
    deckMass,
    bondMode,
    areaNormalization,
    autoBondingOptions,
  } = opts;

  const geometry = new THREE.BoxGeometry(span, height, thickness, 2, 3, 1);

  const fractureOptions: FracturedScenarioOptions = {
    fragmentCount,
    voronoiMode: '3D',
    totalMass: deckMass,
    bondMode,
    areaNormalization,
    autoBondingOptions,
    dimensions: { x: span, y: height, z: thickness },
    rapier: opts.rapier,
    worldOffset: { x: 0, y: height * 0.5, z: 0 },
    foundation: {
      enabled: true,
      heightRatio: 0.06,
      maxHeight: 0.08,
    },
  };

  const scenario = await buildFracturedScenarioAsync(geometry, fractureOptions);
  geometry.dispose();

  return scenario;
}
