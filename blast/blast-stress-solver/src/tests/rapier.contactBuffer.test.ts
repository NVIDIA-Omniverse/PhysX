import { describe, it, expect, vi } from 'vitest';
import { ContactBuffer } from '../rapier/contactBuffer';
import type { DestructibleDamageSystem } from '../rapier/damage';

function mockDamageSystem() {
  return {
    isEnabled: vi.fn(() => true),
    onImpact: vi.fn(),
    onInternalImpact: vi.fn(),
  } as unknown as DestructibleDamageSystem & {
    onImpact: ReturnType<typeof vi.fn>;
    onInternalImpact: ReturnType<typeof vi.fn>;
    isEnabled: ReturnType<typeof vi.fn>;
  };
}

describe('ContactBuffer', () => {
  it('starts empty', () => {
    const buf = new ContactBuffer();
    expect(buf.externalCount).toBe(0);
    expect(buf.internalCount).toBe(0);
  });

  it('records and counts external contacts', () => {
    const buf = new ContactBuffer();
    buf.recordExternal({ nodeIndex: 0, effMag: 100, dt: 1 / 60 });
    buf.recordExternal({ nodeIndex: 1, effMag: 200, dt: 1 / 60 });
    expect(buf.externalCount).toBe(2);
  });

  it('records and counts internal contacts', () => {
    const buf = new ContactBuffer();
    buf.recordInternal({ nodeA: 0, nodeB: 1, effMag: 50, dt: 1 / 60 });
    expect(buf.internalCount).toBe(1);
  });

  it('clear() resets all contacts', () => {
    const buf = new ContactBuffer();
    buf.recordExternal({ nodeIndex: 0, effMag: 100, dt: 1 / 60 });
    buf.recordInternal({ nodeA: 0, nodeB: 1, effMag: 50, dt: 1 / 60 });
    buf.clear();
    expect(buf.externalCount).toBe(0);
    expect(buf.internalCount).toBe(0);
  });

  it('replay() calls damageSystem.onImpact for external contacts with local point', () => {
    const buf = new ContactBuffer();
    const ds = mockDamageSystem();

    buf.recordExternal({
      nodeIndex: 3,
      effMag: 150,
      dt: 1 / 60,
      localPoint: { x: 1, y: 2, z: 3 },
    });

    buf.replay(ds);

    expect(ds.onImpact).toHaveBeenCalledTimes(1);
    expect(ds.onImpact).toHaveBeenCalledWith(
      3, 150, 1 / 60,
      { localPoint: { x: 1, y: 2, z: 3 } },
    );
  });

  it('replay() calls damageSystem.onImpact without local point when omitted', () => {
    const buf = new ContactBuffer();
    const ds = mockDamageSystem();

    buf.recordExternal({ nodeIndex: 5, effMag: 99, dt: 0.01 });
    buf.replay(ds);

    expect(ds.onImpact).toHaveBeenCalledWith(5, 99, 0.01, undefined);
  });

  it('replay() calls damageSystem.onInternalImpact for internal contacts', () => {
    const buf = new ContactBuffer();
    const ds = mockDamageSystem();

    buf.recordInternal({
      nodeA: 2,
      nodeB: 7,
      effMag: 300,
      dt: 1 / 60,
      localPointA: { x: 0.5, y: 0, z: 0 },
      localPointB: { x: -0.5, y: 0, z: 0 },
    });

    buf.replay(ds);

    expect(ds.onInternalImpact).toHaveBeenCalledTimes(1);
    expect(ds.onInternalImpact).toHaveBeenCalledWith(
      2, 7, 300, 1 / 60,
      {
        localPointA: { x: 0.5, y: 0, z: 0 },
        localPointB: { x: -0.5, y: 0, z: 0 },
      },
    );
  });

  it('replay() skips when damage system is disabled', () => {
    const buf = new ContactBuffer();
    const ds = mockDamageSystem();
    ds.isEnabled.mockReturnValue(false);

    buf.recordExternal({ nodeIndex: 0, effMag: 100, dt: 1 / 60 });
    buf.recordInternal({ nodeA: 0, nodeB: 1, effMag: 50, dt: 1 / 60 });

    buf.replay(ds);

    expect(ds.onImpact).not.toHaveBeenCalled();
    expect(ds.onInternalImpact).not.toHaveBeenCalled();
  });

  it('can replay multiple times (for rollback scenarios)', () => {
    const buf = new ContactBuffer();
    const ds = mockDamageSystem();

    buf.recordExternal({ nodeIndex: 0, effMag: 100, dt: 1 / 60 });

    buf.replay(ds);
    expect(ds.onImpact).toHaveBeenCalledTimes(1);

    buf.replay(ds);
    expect(ds.onImpact).toHaveBeenCalledTimes(2);
  });

  it('clear + record + replay cycle works correctly across frames', () => {
    const buf = new ContactBuffer();
    const ds = mockDamageSystem();

    // Frame 1
    buf.recordExternal({ nodeIndex: 0, effMag: 100, dt: 1 / 60 });
    buf.replay(ds);
    expect(ds.onImpact).toHaveBeenCalledTimes(1);

    // Frame 2
    buf.clear();
    buf.recordExternal({ nodeIndex: 1, effMag: 200, dt: 1 / 60 });
    buf.recordExternal({ nodeIndex: 2, effMag: 300, dt: 1 / 60 });
    buf.replay(ds);
    expect(ds.onImpact).toHaveBeenCalledTimes(3); // 1 from frame 1 + 2 from frame 2

    // Verify last calls had correct node indices
    const calls = ds.onImpact.mock.calls;
    expect(calls[1][0]).toBe(1);
    expect(calls[2][0]).toBe(2);
  });
});
