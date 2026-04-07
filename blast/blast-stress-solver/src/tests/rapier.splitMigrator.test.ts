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

  // ── Regression: batched children must not double-assign ──

  it('does not assign the same body to multiple children in a single call', () => {
    // Simulates a parent body with 4 nodes splitting into 4 single-node children.
    // The parent body overlaps every child, but only one child should reuse it.
    const plan = planSplitMigration(
      [{ handle: 10, nodeIndices: new Set([0, 1, 2, 3]), isFixed: false }],
      [
        { index: 0, actorIndex: 100, nodes: [0], isSupport: false },
        { index: 1, actorIndex: 101, nodes: [1], isSupport: false },
        { index: 2, actorIndex: 102, nodes: [2], isSupport: false },
        { index: 3, actorIndex: 103, nodes: [3], isSupport: false },
      ],
    );

    // At most ONE child should reuse body 10; the rest must create new bodies
    expect(plan.reuse.length).toBeLessThanOrEqual(1);
    expect(plan.reuse.length + plan.create.length).toBe(4);

    // Verify no body handle appears in reuse more than once
    const reusedHandles = plan.reuse.map(r => r.bodyHandle);
    expect(new Set(reusedHandles).size).toBe(reusedHandles.length);
  });

  it('prevents non-support children from reusing a fixed body', () => {
    const plan = planSplitMigration(
      [{ handle: 10, nodeIndices: new Set([0, 1, 2]), isFixed: true }],
      [
        { index: 0, actorIndex: 100, nodes: [0], isSupport: false },
        { index: 1, actorIndex: 101, nodes: [1, 2], isSupport: false },
      ],
    );

    // Non-support children should never reuse a fixed body
    for (const r of plan.reuse) {
      expect(r.bodyHandle).not.toBe(10);
    }
    // Both should be created fresh
    expect(plan.create.length).toBe(2);
  });

  it('allows a support child to reuse a fixed body', () => {
    const plan = planSplitMigration(
      [{ handle: 10, nodeIndices: new Set([0, 1, 2]), isFixed: true }],
      [
        { index: 0, actorIndex: 100, nodes: [0], isSupport: true },
        { index: 1, actorIndex: 101, nodes: [1, 2], isSupport: false },
      ],
    );

    // The support child should reuse the fixed body
    const supportReuse = plan.reuse.find(r => r.childIndex === 0);
    expect(supportReuse).toBeDefined();
    expect(supportReuse!.bodyHandle).toBe(10);

    // The non-support child should be created fresh (can't reuse fixed body)
    expect(plan.create).toContainEqual({ childIndex: 1 });
  });
});
