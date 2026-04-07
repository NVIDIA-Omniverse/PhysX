import { describe, expect, it } from 'vitest';
import { planSplitMigration } from '../rapier/splitMigrator';

describe('rapier/splitMigrator', () => {
  it('reuses exact body matches before planning overlaps', () => {
    const plan = planSplitMigration(
      [
        { handle: 10, nodeIndices: new Set([0, 1]), isFixed: false },
        { handle: 20, nodeIndices: new Set([2, 3]), isFixed: true },
      ],
      [
        { index: 0, actorIndex: 100, nodes: [0, 1], isSupport: false },
        { index: 1, actorIndex: 200, nodes: [2, 3], isSupport: true },
      ],
    );

    expect(plan.reuse).toEqual([
      { childIndex: 0, bodyHandle: 10 },
      { childIndex: 1, bodyHandle: 20 },
    ]);
    expect(plan.create).toEqual([]);
  });

  it('creates unmatched children when no overlap exists', () => {
    const plan = planSplitMigration(
      [{ handle: 10, nodeIndices: new Set([0, 1]), isFixed: false }],
      [
        { index: 0, actorIndex: 100, nodes: [4, 5], isSupport: false },
        { index: 1, actorIndex: 200, nodes: [6], isSupport: false },
      ],
    );

    expect(plan.reuse).toEqual([]);
    expect(plan.create).toEqual([{ childIndex: 0 }, { childIndex: 1 }]);
  });
});
