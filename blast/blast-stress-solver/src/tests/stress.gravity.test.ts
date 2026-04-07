import { describe, it, expect } from 'vitest';
import type * as Runtime from '..';
import {
  createColumnNodes,
  createColumnBond,
  columnStressSettings,
  gravityMagnitude
} from './gravityFixtures';

async function importRuntime(): Promise<typeof Runtime> {
  return (await import('../../dist/index.js')) as typeof Runtime;
}

describe('ExtStressSolver addActorGravity', () => {
  const createSolver = async (orientation: 'vertical' | 'horizontal') => {
    const { loadStressSolver } = await importRuntime();
    const rt = await loadStressSolver();
    return rt.createExtSolver({
      nodes: createColumnNodes(orientation),
      bonds: [createColumnBond(orientation)],
      settings: columnStressSettings
    });
  };

  it('distinguishes gravity directions based on actor rotation', async () => {
    const worldGravity = { x: 0, y: -gravityMagnitude, z: 0 };

    // Upright column: global gravity fractures the bond
    const verticalSolver = await createSolver('vertical');
    verticalSolver.addGravity(worldGravity);
    verticalSolver.update();
    const verticalFracture = verticalSolver.generateFractureCommands();
    expect(verticalFracture.fractures.length).toBeGreaterThan(0);
    verticalSolver.destroy();

    // Horizontal column without any per-actor gravity: remains intact
    const horizontalSolver = await createSolver('horizontal');
    horizontalSolver.update();
    const horizontalFracture = horizontalSolver.generateFractureCommands();
    expect(horizontalFracture.fractures.length).toBe(0);
    horizontalSolver.destroy();

    // Horizontal column with gravity transformed into local space using addActorGravity
    const rotatedSolver = await createSolver('horizontal');
    const [actor] = rotatedSolver.actors();
    expect(actor).toBeDefined();
    rotatedSolver.addActorGravity(actor!.actorIndex, { x: -gravityMagnitude, y: 0, z: 0 });
    rotatedSolver.update();
    const rotatedFracture = rotatedSolver.generateFractureCommands();
    expect(rotatedFracture.fractures.length).toBeGreaterThan(0);
    rotatedSolver.destroy();
  });
});

