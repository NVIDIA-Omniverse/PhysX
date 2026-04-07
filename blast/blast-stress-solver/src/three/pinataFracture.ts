/**
 * three-pinata integration for Voronoi mesh fracturing.
 *
 * This module wraps @dgreenheck/three-pinata's DestructibleMesh API to produce
 * FragmentInfo[] arrays that feed into the blast-stress-solver scenario pipeline.
 *
 * three-pinata is an **optional** peer dependency. If not installed, calling
 * these functions throws a descriptive error.
 */
import * as THREE from 'three';
import type { Vec3, ScenarioDesc } from '../rapier/types';
import type { FragmentInfo, BondDetectionOptions } from './fracture';
import { recenterGeometry } from './geometryUtils';
import { ensurePlainAttributes, prepareGeometryForFracture } from './geometryUtils';
import { buildScenarioFromFragments, buildScenarioFromFragmentsAsync, type AreaNormalizationMode, type FragmentScenarioOptions } from './scenarioFromFragments';
import { buildFoundationFragments, type FoundationOptions } from './foundation';

// ── Dynamic three-pinata import ────────────────────────────────────────────

type PinataModule = {
  DestructibleMesh: new (geometry: THREE.BufferGeometry, ...materials: THREE.Material[]) => {
    fracture(options: unknown): THREE.Mesh[];
  };
  FractureOptions: new (opts: Record<string, unknown>) => unknown;
};

let pinataCache: PinataModule | null = null;

async function loadPinata(): Promise<PinataModule> {
  if (pinataCache) return pinataCache;
  try {
    const mod = await import('@dgreenheck/three-pinata') as PinataModule;
    pinataCache = mod;
    return mod;
  } catch {
    throw new Error(
      '[blast-stress-solver] @dgreenheck/three-pinata is not installed. ' +
      'Install it with: npm install @dgreenheck/three-pinata',
    );
  }
}

function loadPinataSync(): PinataModule {
  if (pinataCache) return pinataCache;
  // Try synchronous require for CJS environments
  try {
    // eslint-disable-next-line @typescript-eslint/no-require-imports
    const mod = require('@dgreenheck/three-pinata') as PinataModule;
    pinataCache = mod;
    return mod;
  } catch {
    throw new Error(
      '[blast-stress-solver] @dgreenheck/three-pinata is not installed or not pre-loaded. ' +
      'Either install it (npm install @dgreenheck/three-pinata) or call ensurePinataLoaded() first.',
    );
  }
}

/**
 * Pre-load the three-pinata module. Call this once at startup to ensure
 * synchronous fracture functions work in ESM environments.
 */
export async function ensurePinataLoaded(): Promise<void> {
  await loadPinata();
}

// ── Fracturing ─────────────────────────────────────────────────────────────

export type FractureGeometryOptions = {
  /** Number of Voronoi fragments (default: 120) */
  fragmentCount?: number;
  /** Voronoi tessellation mode (default: '3D') */
  voronoiMode?: '3D' | '2.5D';
  /** Offset applied to all fragment world positions (default: {x:0, y:0, z:0}) */
  worldOffset?: Vec3;
  /** Minimum half-extent for fragments to avoid degenerate physics (default: 0.05) */
  minHalfExtent?: number;
};

/**
 * Fracture a Three.js geometry into fragments using Voronoi tessellation.
 *
 * Requires @dgreenheck/three-pinata. Call `ensurePinataLoaded()` once before
 * using this in ESM environments, or install three-pinata as a dependency.
 *
 * @returns Array of FragmentInfo ready for `buildScenarioFromFragments()`
 */
export function fractureGeometry(
  geometry: THREE.BufferGeometry,
  options?: FractureGeometryOptions,
): FragmentInfo[] {
  const pinata = loadPinataSync();
  const {
    fragmentCount = 120,
    voronoiMode = '3D',
    worldOffset = { x: 0, y: 0, z: 0 },
    minHalfExtent = 0.05,
  } = options ?? {};

  // Prepare geometry for fracturing
  const prepared = prepareGeometryForFracture(geometry);

  const fractureOptions = new pinata.FractureOptions({
    fractureMethod: 'voronoi',
    fragmentCount,
    voronoiOptions: { mode: voronoiMode },
  });

  const destructibleMesh = new pinata.DestructibleMesh(prepared);
  const pieceMeshes = destructibleMesh.fracture(fractureOptions);

  const fragments: FragmentInfo[] = pieceMeshes.map((mesh) => {
    const g = mesh.geometry;
    ensurePlainAttributes(g);
    const { offset } = recenterGeometry(g);

    g.computeBoundingBox();
    const size = new THREE.Vector3();
    (g.boundingBox as THREE.Box3).getSize(size);

    return {
      worldPosition: {
        x: worldOffset.x + mesh.position.x + offset.x,
        y: worldOffset.y + mesh.position.y + offset.y,
        z: worldOffset.z + mesh.position.z + offset.z,
      },
      halfExtents: {
        x: Math.max(minHalfExtent, size.x * 0.5),
        y: Math.max(minHalfExtent, size.y * 0.5),
        z: Math.max(minHalfExtent, size.z * 0.5),
      },
      geometry: g,
      isSupport: false,
    };
  });

  return fragments;
}

/**
 * Async version of fractureGeometry that handles dynamic import of three-pinata.
 */
export async function fractureGeometryAsync(
  geometry: THREE.BufferGeometry,
  options?: FractureGeometryOptions,
): Promise<FragmentInfo[]> {
  await loadPinata();
  return fractureGeometry(geometry, options);
}

// ── Convenience: fracture + scenario ───────────────────────────────────────

export type FracturedScenarioOptions = FragmentScenarioOptions & FractureGeometryOptions & {
  /** Foundation configuration. If provided, support fragments are generated below the structure. */
  foundation?: Omit<FoundationOptions, 'span'> & { enabled?: boolean };
};

/**
 * Fracture a geometry and produce a complete ScenarioDesc in one call.
 * Optionally generates a foundation slab underneath.
 */
export function buildFracturedScenario(
  geometry: THREE.BufferGeometry,
  options?: FracturedScenarioOptions,
): ScenarioDesc {
  const fragments = fractureGeometry(geometry, options);
  return assembleScenario(geometry, fragments, options);
}

/**
 * Async version that handles three-pinata dynamic import and optional WASM auto-bonding.
 */
export async function buildFracturedScenarioAsync(
  geometry: THREE.BufferGeometry,
  options?: FracturedScenarioOptions,
): Promise<ScenarioDesc> {
  await loadPinata();
  const fragments = fractureGeometry(geometry, options);

  if (options?.bondMode === 'auto') {
    return buildScenarioFromFragmentsAsync(addFoundation(geometry, fragments, options), options);
  }
  return assembleScenario(geometry, fragments, options);
}

// ── Internals ──────────────────────────────────────────────────────────────

function assembleScenario(
  geometry: THREE.BufferGeometry,
  fragments: FragmentInfo[],
  options?: FracturedScenarioOptions,
): ScenarioDesc {
  const allFragments = addFoundation(geometry, fragments, options);
  return buildScenarioFromFragments(allFragments, options);
}

function addFoundation(
  geometry: THREE.BufferGeometry,
  fragments: FragmentInfo[],
  options?: FracturedScenarioOptions,
): FragmentInfo[] {
  if (!options?.foundation?.enabled) return fragments;

  geometry.computeBoundingBox();
  const bbox = geometry.boundingBox as THREE.Box3;
  const size = new THREE.Vector3();
  bbox.getSize(size);

  const { fragments: foundationFragments, foundationTopY } = buildFoundationFragments({
    span: { x: size.x, y: size.y, z: size.z },
    ...options.foundation,
  });

  // Shift wall fragments up so they sit on the foundation
  const minY = Math.min(...fragments.map((f) => f.worldPosition.y - f.halfExtents.y));
  const liftY = foundationTopY - minY + 0.0005; // tiny gap to avoid z-fighting

  const liftedFragments = fragments.map((f) => ({
    ...f,
    worldPosition: { ...f.worldPosition, y: f.worldPosition.y + liftY },
  }));

  return [...liftedFragments, ...foundationFragments];
}
