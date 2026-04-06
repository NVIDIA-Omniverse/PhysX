import { describe, it, expect } from 'vitest';
import { DestructibleDamageSystem } from '../rapier/damage';
import type { ChunkData, ScenarioDesc } from '../rapier/types';

/** Build a minimal scenario with N chunks for damage testing. */
function makeScenario(count: number, opts?: { mass?: number; volume?: number }) {
  const mass = opts?.mass ?? 1;
  const volume = opts?.volume ?? 1;
  const nodes = Array.from({ length: count }, (_, i) => ({
    centroid: { x: i, y: 0, z: 0 },
    mass,
    volume,
  }));
  const bonds: ScenarioDesc['bonds'] = [];
  for (let i = 0; i < count - 1; i++) {
    bonds.push({
      node0: i,
      node1: i + 1,
      centroid: { x: i + 0.5, y: 0, z: 0 },
      normal: { x: 1, y: 0, z: 0 },
      area: 1,
    });
  }
  return { nodes, bonds } as ScenarioDesc;
}

function makeChunks(count: number): ChunkData[] {
  return Array.from({ length: count }, (_, i) => ({
    nodeIndex: i,
    size: { x: 1, y: 1, z: 1 },
    isSupport: false,
    baseLocalOffset: { x: i, y: 0, z: 0 },
    localOffset: { x: i, y: 0, z: 0 },
    colliderHandle: i,
    bodyHandle: 0,
    active: true,
    detached: false,
  }));
}

describe('DestructibleDamageSystem', () => {
  it('initializes health from node volume and strengthPerVolume', () => {
    const scenario = makeScenario(3, { volume: 2 });
    const chunks = makeChunks(3);
    const sys = new DestructibleDamageSystem({
      chunks,
      scenario,
      materialScale: 1,
      options: { enabled: true, strengthPerVolume: 500 },
    });

    // maxHealth = strengthPerVolume * volume = 500 * 2 = 1000
    const h = sys.getHealth(0);
    expect(h).not.toBeNull();
    expect(h!.maxHealth).toBe(1000);
    expect(h!.health).toBe(1000);
    expect(h!.destroyed).toBe(false);
  });

  it('does nothing when disabled', () => {
    const scenario = makeScenario(2);
    const chunks = makeChunks(2);
    const sys = new DestructibleDamageSystem({
      chunks,
      scenario,
      materialScale: 1,
      options: { enabled: false },
    });

    sys.onImpact(0, 99999, 1);
    const destroyed = sys.tick(1 / 60);
    expect(destroyed).toEqual([]);
  });

  it('accumulates damage from onImpact and destroys nodes', () => {
    const scenario = makeScenario(2, { volume: 1 });
    const chunks = makeChunks(2);
    const sys = new DestructibleDamageSystem({
      chunks,
      scenario,
      materialScale: 1,
      options: {
        enabled: true,
        strengthPerVolume: 100,
        kImpact: 1.0,
        minImpulseThreshold: 0,
        contactDamageScale: 1.0,
        massExponent: 0,
        contactCooldownMs: 0,
      },
    });

    // maxHealth = 100 * 1 = 100
    // impulse = forceMagnitude * dt = 500 * 1 = 500
    // dmg = kImpact * (impulse / mass^massExponent) = 1.0 * (500 / 1) = 500
    sys.onImpact(0, 500, 1);
    const destroyed = sys.tick(1 / 60);
    expect(destroyed).toContain(0);
    expect(sys.getHealth(0)?.destroyed).toBe(true);
    expect(sys.getHealth(0)?.health).toBe(0);
  });

  it('respects minImpulseThreshold', () => {
    const scenario = makeScenario(1, { volume: 1 });
    const chunks = makeChunks(1);
    const sys = new DestructibleDamageSystem({
      chunks,
      scenario,
      materialScale: 1,
      options: {
        enabled: true,
        strengthPerVolume: 100,
        kImpact: 1.0,
        minImpulseThreshold: 100,
        contactDamageScale: 1.0,
        massExponent: 0,
        contactCooldownMs: 0,
      },
    });

    // impulse = 50 * 1 = 50 < threshold 100 → no damage
    sys.onImpact(0, 50, 1);
    const destroyed = sys.tick(1 / 60);
    expect(destroyed).toEqual([]);
    expect(sys.getHealth(0)?.health).toBe(100);
  });

  it('respects contact cooldown', () => {
    const scenario = makeScenario(1, { volume: 1 });
    const chunks = makeChunks(1);
    const sys = new DestructibleDamageSystem({
      chunks,
      scenario,
      materialScale: 1,
      options: {
        enabled: true,
        strengthPerVolume: 1000,
        kImpact: 0.1,
        minImpulseThreshold: 0,
        contactDamageScale: 1.0,
        massExponent: 0,
        contactCooldownMs: 500,
      },
    });

    // First impact applies
    sys.onImpact(0, 100, 1);
    sys.tick(0.01); // advance time by 10ms

    // Second impact within cooldown window → ignored
    sys.onImpact(0, 100, 1);
    sys.tick(0.01);

    const h = sys.getHealth(0)!;
    // Only one hit of damage should have applied
    // impulse = 100, dmg = 0.1 * 100 = 10
    expect(h.health).toBe(1000 - 10);
  });

  it('previewTick does not mutate state', () => {
    const scenario = makeScenario(1, { volume: 1 });
    const chunks = makeChunks(1);
    const sys = new DestructibleDamageSystem({
      chunks,
      scenario,
      materialScale: 1,
      options: {
        enabled: true,
        strengthPerVolume: 10,
        kImpact: 1.0,
        minImpulseThreshold: 0,
        massExponent: 0,
        contactCooldownMs: 0,
      },
    });

    sys.onImpact(0, 100, 1);
    const preview = sys.previewTick(1 / 60);
    expect(preview).toContain(0);

    // State should be unchanged
    expect(sys.getHealth(0)?.destroyed).toBe(false);
    expect(sys.getHealth(0)?.health).toBe(10);
  });

  it('captureImpactState and restoreImpactState round-trip', () => {
    const scenario = makeScenario(2, { volume: 1 });
    const chunks = makeChunks(2);
    const sys = new DestructibleDamageSystem({
      chunks,
      scenario,
      materialScale: 1,
      options: {
        enabled: true,
        strengthPerVolume: 100,
        kImpact: 0.1,
        minImpulseThreshold: 0,
        massExponent: 0,
        contactCooldownMs: 0,
      },
    });

    sys.onImpact(0, 200, 1);
    sys.tick(0.016);
    const healthAfterHit = sys.getHealth(0)!.health;

    const snapshot = sys.captureImpactState();

    // Apply more damage
    sys.onImpact(0, 500, 1);
    sys.tick(0.016);
    expect(sys.getHealth(0)!.health).toBeLessThan(healthAfterHit);

    // Restore snapshot
    sys.restoreImpactState(snapshot);
    expect(sys.getHealth(0)!.health).toBe(healthAfterHit);
  });

  it('applyDirect bypasses cooldown and threshold', () => {
    const scenario = makeScenario(1, { volume: 1 });
    const chunks = makeChunks(1);
    const sys = new DestructibleDamageSystem({
      chunks,
      scenario,
      materialScale: 1,
      options: {
        enabled: true,
        strengthPerVolume: 100,
        minImpulseThreshold: 9999,
        contactCooldownMs: 99999,
      },
    });

    sys.applyDirect(0, 50);
    sys.tick(1 / 60);
    expect(sys.getHealth(0)?.health).toBe(50);
  });

  it('onInternalImpact applies scaled damage to both nodes', () => {
    const scenario = makeScenario(2, { volume: 1 });
    const chunks = makeChunks(2);
    const sys = new DestructibleDamageSystem({
      chunks,
      scenario,
      materialScale: 1,
      options: {
        enabled: true,
        strengthPerVolume: 1000,
        kImpact: 1.0,
        minImpulseThreshold: 0,
        contactDamageScale: 1.0,
        internalContactScale: 0.5,
        massExponent: 0,
        contactCooldownMs: 0,
      },
    });

    sys.onInternalImpact(0, 1, 100, 1);
    sys.tick(1 / 60);

    // Both nodes should have taken damage (scaled by internalContactScale 0.5)
    // forceMagnitude * scale = 100 * 0.5 = 50, impulse = 50 * 1 = 50, dmg = 1.0 * 50 = 50
    expect(sys.getHealth(0)!.health).toBeLessThan(1000);
    expect(sys.getHealth(1)!.health).toBeLessThan(1000);
  });

  it('supports (mass=0) are immune when enableSupportsDamage is false', () => {
    const scenario = {
      nodes: [
        { centroid: { x: 0, y: 0, z: 0 }, mass: 0, volume: 1 },
        { centroid: { x: 1, y: 0, z: 0 }, mass: 1, volume: 1 },
      ],
      bonds: [],
    } as unknown as ScenarioDesc;
    const chunks: ChunkData[] = [
      { ...makeChunks(1)[0], isSupport: true },
      makeChunks(2)[1],
    ];
    const sys = new DestructibleDamageSystem({
      chunks,
      scenario,
      materialScale: 1,
      options: {
        enabled: true,
        strengthPerVolume: 100,
        kImpact: 1.0,
        minImpulseThreshold: 0,
        enableSupportsDamage: false,
        massExponent: 0,
        contactCooldownMs: 0,
      },
    });

    // Support node (mass=0) - the impulse goes through but mass=0 is treated as infinite mass
    // Mass exponent 0 means denom = 1, but the code checks rawMass === 0 for support
    // Even if damage accrues, supports don't get destroyed in tick()
    sys.onImpact(0, 9999, 1);
    const destroyed = sys.tick(1 / 60);
    // Supports shouldn't appear in destroyed list (tick skips isSupport)
    expect(destroyed).not.toContain(0);
  });
});
