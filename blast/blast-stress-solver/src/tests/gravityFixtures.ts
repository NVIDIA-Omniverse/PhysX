import type { ExtStressBondDesc, ExtStressNodeDesc, ExtStressSolverSettings } from '../types';

export type ColumnOrientation = 'vertical' | 'horizontal';

const verticalNodes: ExtStressNodeDesc[] = [
  { centroid: { x: 0, y: 0, z: 0 }, mass: 0, volume: 1 },
  { centroid: { x: 0, y: 1, z: 0 }, mass: 1, volume: 1 }
];

const horizontalNodes: ExtStressNodeDesc[] = [
  { centroid: { x: 0, y: 0, z: 0 }, mass: 0, volume: 1 },
  { centroid: { x: 1, y: 0, z: 0 }, mass: 1, volume: 1 }
];

const verticalBond: ExtStressBondDesc = {
  centroid: { x: 0, y: 0.5, z: 0 },
  normal: { x: 0, y: 1, z: 0 },
  area: 1,
  node0: 0,
  node1: 1
};

const horizontalBond: ExtStressBondDesc = {
  centroid: { x: 0.5, y: 0, z: 0 },
  normal: { x: 1, y: 0, z: 0 },
  area: 1,
  node0: 0,
  node1: 1
};

function cloneNodes(base: ExtStressNodeDesc[]): ExtStressNodeDesc[] {
  return base.map((node) => ({
    centroid: node.centroid ? { ...node.centroid } : undefined,
    mass: node.mass,
    volume: node.volume
  }));
}

function cloneBond(base: ExtStressBondDesc): ExtStressBondDesc {
  return {
    centroid: base.centroid ? { ...base.centroid } : undefined,
    normal: base.normal ? { ...base.normal } : undefined,
    area: base.area,
    node0: base.node0,
    node1: base.node1
  };
}

export function createColumnNodes(orientation: ColumnOrientation): ExtStressNodeDesc[] {
  return orientation === 'vertical' ? cloneNodes(verticalNodes) : cloneNodes(horizontalNodes);
}

export function createColumnBond(orientation: ColumnOrientation): ExtStressBondDesc {
  return orientation === 'vertical' ? cloneBond(verticalBond) : cloneBond(horizontalBond);
}

/**
 * Tight compression/tension limits with extremely high shear thresholds so tests can focus
 * purely on the axial component introduced by different gravity directions.
 */
export const columnStressSettings: ExtStressSolverSettings = {
  maxSolverIterationsPerFrame: 16,
  compressionElasticLimit: 0.5,
  compressionFatalLimit: 1.0,
  tensionElasticLimit: 0.5,
  tensionFatalLimit: 1.0,
  shearElasticLimit: 1e6,
  shearFatalLimit: 1e6
};

export const gravityMagnitude = 5.0;

