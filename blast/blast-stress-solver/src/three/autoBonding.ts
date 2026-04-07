import type { BondingMode } from '../types';
import { chunksFromBufferGeometries, loadStressSolver } from '../stress';
import * as THREE from "three";
import type { ScenarioBond, ScenarioDesc } from '../rapier/types';

export type AutoBondingRequest = {
  enabled?: boolean;
  mode?: BondingMode;
  maxSeparation?: number;
  label?: string;
};

export type AutoBondChunkInput = {
  geometry: THREE.BufferGeometry;
  isSupport?: boolean;
  matrix?: THREE.Matrix4;
};

const MIN_AREA = 1e-8;
const DEFAULT_MODE: BondingMode = "exact";

export async function generateAutoBondsFromChunks(
  chunks: AutoBondChunkInput[],
  options?: AutoBondingRequest,
): Promise<ScenarioBond[] | null> {
  if (!chunks.length) return [];
  try {
    const chunkInputs = chunksFromBufferGeometries(
      chunks.map((chunk) => chunk.geometry),
      (_geometry, index) => {
        const source = chunks[index];
        return {
          // isSupport: !!source.isSupport,
          isSupport: true, // isSupport actually means should have bonds connected to it
          applyMatrix: source.matrix,
          nonIndexed: true,
          cloneGeometry: true,
        };
      },
    );
    const runtime = await loadStressSolver();
    const bondDescs = runtime.createBondsFromTriangles(chunkInputs, {
      mode: options?.mode ?? DEFAULT_MODE,
      maxSeparation: options?.maxSeparation,
    });
    const bonds: ScenarioBond[] = [];
    for (const bond of bondDescs) {
      if (!bond.centroid || !bond.normal) continue;
      bonds.push({
        node0: bond.node0,
        node1: bond.node1,
        centroid: bond.centroid,
        normal: bond.normal,
        area: Math.max(bond.area ?? MIN_AREA, MIN_AREA),
      });
    }
    return bonds;
  } catch (error) {
    const label = options?.label ?? "AutoBonding";
    console.error(`[${label}] Failed to generate bonds`, error);
    return null;
  }
}

/**
 * Applies auto-bonding to a scenario using fragmentGeometries.
 * If fragmentGeometries are missing, logs a warning and returns the scenario unchanged.
 * Only call this function when auto bonding is enabled.
 */
export async function applyAutoBondingToScenario(
  scenario: ScenarioDesc,
  options?: Omit<AutoBondingRequest, "enabled">,
): Promise<ScenarioDesc> {
  const fragmentGeometries = scenario.parameters?.fragmentGeometries as
    | THREE.BufferGeometry[]
    | undefined;
  if (!fragmentGeometries?.length) {
    console.warn(
      "[AutoBonding] Enabled but scenario missing fragmentGeometries, using predefined bonds",
    );
    return scenario;
  }

  const chunks: AutoBondChunkInput[] = scenario.nodes.map((node, i) => ({
    geometry: fragmentGeometries[i],
    isSupport: node.mass === 0,
    matrix: new THREE.Matrix4().makeTranslation(
      node.centroid.x,
      node.centroid.y,
      node.centroid.z,
    ),
  }));

  const autoBonds = await generateAutoBondsFromChunks(chunks, options);
  if (!autoBonds?.length) return scenario;

  return { ...scenario, bonds: autoBonds };
}
