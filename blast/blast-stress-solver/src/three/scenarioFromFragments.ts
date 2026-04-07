/**
 * Build a complete ScenarioDesc from an array of FragmentInfo.
 *
 * This is the main "glue" function that converts fracture output into a form
 * the destructible-core runtime can consume. It handles node/mass assembly,
 * bond detection, collider descriptor generation, and area normalization.
 */
import * as THREE from 'three';
import type { ColliderDescBuilder, ScenarioBond, ScenarioDesc, ScenarioNode, Vec3 } from '../rapier/types';
import { computeBondsFromFragments, normalizeBondAreasByAxis, uniformizeBondAreasByAxis } from './fracture';
import type { FragmentInfo, BondDetectionOptions } from './fracture';
import type { AutoBondingRequest } from './autoBonding';

export type AreaNormalizationMode = 'perAxis' | 'uniform' | 'isotropic' | 'none';

export type FragmentScenarioOptions = {
  /** Total mass for all non-support fragments (default: 10000) */
  totalMass?: number;
  /** Bond detection approach: 'proximity' (fast JS), 'auto' (WASM triangle-based), 'both' (union) */
  bondMode?: 'proximity' | 'auto';
  /** Area normalization mode (default: 'perAxis') */
  areaNormalization?: AreaNormalizationMode;
  /** Bounding dimensions for area normalization. If omitted, computed from fragment extents. */
  dimensions?: { x: number; y: number; z: number };
  /** Options forwarded to proximity bond detection */
  bondDetectionOptions?: BondDetectionOptions;
  /** Options forwarded to WASM auto-bonding (only used when bondMode includes 'auto') */
  autoBondingOptions?: Omit<AutoBondingRequest, 'enabled'>;
  /**
   * Rapier module reference. Required for convex hull colliders.
   * If omitted, only cuboid colliders are generated (for supports),
   * and non-support nodes get null colliderDescForNode entries.
   */
  rapier?: RapierModule;
};

/** Minimal Rapier module interface for collider descriptor creation */
type RapierModule = {
  ColliderDesc: {
    cuboid(hx: number, hy: number, hz: number): unknown;
    convexHull(points: Float32Array): unknown | null;
  };
};

const DEFAULT_TOTAL_MASS = 10_000;

/**
 * Build a ScenarioDesc from fragments using synchronous proximity-based bond detection.
 */
export function buildScenarioFromFragments(
  fragments: FragmentInfo[],
  options?: FragmentScenarioOptions,
): ScenarioDesc {
  const opts = options ?? {};
  const totalMass = opts.totalMass ?? DEFAULT_TOTAL_MASS;
  const normMode = opts.areaNormalization ?? 'perAxis';

  // Build nodes
  const nodes: ScenarioNode[] = [];
  const fragmentSizes: Vec3[] = [];
  const fragmentGeometries: THREE.BufferGeometry[] = [];
  const colliderDescForNode: (ColliderDescBuilder | null)[] = [];
  let totalVolume = 0;

  for (let i = 0; i < fragments.length; i++) {
    const f = fragments[i];
    const size: Vec3 = {
      x: f.halfExtents.x * 2,
      y: f.halfExtents.y * 2,
      z: f.halfExtents.z * 2,
    };
    const volume = size.x * size.y * size.z;
    const mass = f.isSupport ? 0 : volume; // scaled below
    if (!f.isSupport) totalVolume += volume;

    nodes.push({ centroid: { ...f.worldPosition }, mass, volume });
    fragmentSizes.push(size);
    fragmentGeometries.push(f.geometry);

    // Collider descriptors
    if (f.isSupport) {
      const hx = f.halfExtents.x, hy = f.halfExtents.y, hz = f.halfExtents.z;
      if (opts.rapier) {
        const rapier = opts.rapier;
        colliderDescForNode.push(() => rapier.ColliderDesc.cuboid(hx, hy, hz) as ReturnType<ColliderDescBuilder>);
      } else {
        colliderDescForNode.push(null);
      }
    } else {
      if (opts.rapier) {
        const rapier = opts.rapier;
        const pos = f.geometry.getAttribute('position') as THREE.BufferAttribute;
        const points = pos?.array instanceof Float32Array
          ? pos.array
          : new Float32Array((pos?.array ?? []) as ArrayLike<number>);
        colliderDescForNode.push(() => rapier.ColliderDesc.convexHull(points) as ReturnType<ColliderDescBuilder>);
      } else {
        colliderDescForNode.push(null);
      }
    }
  }

  // Scale masses
  const scale = totalVolume > 0 ? totalMass / totalVolume : 0;
  for (const n of nodes) {
    if (n.mass === 0) continue;
    n.mass = n.volume > 0 ? n.volume * scale : 0;
  }

  // Bond detection (proximity)
  let bonds = computeBondsFromFragments(fragments, opts.bondDetectionOptions);

  // Compute dimensions for normalization if not provided
  const dims = opts.dimensions ?? computeFragmentBounds(fragments);

  // Normalize bond areas
  bonds = applyAreaNormalization(bonds, dims, normMode);

  return {
    nodes,
    bonds,
    parameters: { fragmentSizes, fragmentGeometries },
    colliderDescForNode,
  };
}

/**
 * Async variant that supports WASM auto-bonding.
 * When bondMode is 'auto', uses the WASM triangle-based bond generator
 * from autoBonding.ts instead of proximity-based detection.
 */
export async function buildScenarioFromFragmentsAsync(
  fragments: FragmentInfo[],
  options?: FragmentScenarioOptions,
): Promise<ScenarioDesc> {
  const bondMode = options?.bondMode ?? 'proximity';

  // Start with sync build (proximity bonds)
  const scenario = buildScenarioFromFragments(fragments, {
    ...options,
    // If mode is 'auto', we'll replace bonds below; use proximity as fallback
    areaNormalization: bondMode === 'auto' ? 'none' : options?.areaNormalization,
  });

  // If auto-bonding requested, replace proximity bonds with WASM bonds
  if (bondMode === 'auto') {
    const { applyAutoBondingToScenario } = await import('./autoBonding');
    const autoBonded = await applyAutoBondingToScenario(scenario, options?.autoBondingOptions);
    const dims = options?.dimensions ?? computeFragmentBounds(fragments);
    autoBonded.bonds = applyAreaNormalization(
      autoBonded.bonds,
      dims,
      options?.areaNormalization ?? 'perAxis',
    );
    return autoBonded;
  }

  return scenario;
}

// ── Internals ──────────────────────────────────────────────────────────────

function computeFragmentBounds(fragments: FragmentInfo[]): { x: number; y: number; z: number } {
  let minX = Infinity, maxX = -Infinity;
  let minY = Infinity, maxY = -Infinity;
  let minZ = Infinity, maxZ = -Infinity;
  for (const f of fragments) {
    minX = Math.min(minX, f.worldPosition.x - f.halfExtents.x);
    maxX = Math.max(maxX, f.worldPosition.x + f.halfExtents.x);
    minY = Math.min(minY, f.worldPosition.y - f.halfExtents.y);
    maxY = Math.max(maxY, f.worldPosition.y + f.halfExtents.y);
    minZ = Math.min(minZ, f.worldPosition.z - f.halfExtents.z);
    maxZ = Math.max(maxZ, f.worldPosition.z + f.halfExtents.z);
  }
  return {
    x: Math.max(1e-6, maxX - minX),
    y: Math.max(1e-6, maxY - minY),
    z: Math.max(1e-6, maxZ - minZ),
  };
}

function applyAreaNormalization(
  bonds: ScenarioBond[],
  dims: { x: number; y: number; z: number },
  mode: AreaNormalizationMode,
): ScenarioBond[] {
  switch (mode) {
    case 'perAxis':
      return normalizeBondAreasByAxis(bonds, dims);
    case 'uniform':
      return uniformizeBondAreasByAxis(bonds, dims);
    case 'isotropic': {
      // Single uniform scale factor (geometric mean of per-axis scales)
      const totalArea = bonds.reduce((s, b) => s + b.area, 0);
      const targetArea = (dims.y * dims.z + dims.x * dims.z + dims.x * dims.y) / 3;
      const scale = totalArea > 0 ? targetArea * bonds.length / totalArea : 1;
      return bonds.map((b) => ({ ...b, area: Math.max(b.area * scale, 1e-8) }));
    }
    case 'none':
      return bonds;
  }
}
