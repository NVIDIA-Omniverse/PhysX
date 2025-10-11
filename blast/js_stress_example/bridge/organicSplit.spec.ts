import { beforeAll, describe, expect, it } from 'vitest';
import { buildBridgeShared } from '../bridge/buildBridge.headless.js';
import { applyForcesAndSolve, processSolverFractures, drainContactForces, normalizeSplitResults } from '../bridge/coreLogic.js';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import { spawnLoadVehicle, updateLoadVehicle, spawnProjectile, updateProjectiles } from '../bridge/dynamics.js';

describe('Organic split flow (no manual handleSplitEvents)', () => {
  beforeAll(async () => {
    // RAPIER is in bridge-sim builder
  });

  it('produces deterministic split events that match expected patterns', async () => {
    const bridge = await buildBridgeShared({ gravity: -9.81, strengthScale: 0.05 });
    bridge.world.RAPIER = (await import('@dimforge/rapier3d-compat')).default;
    bridge.projectiles = [];
    bridge.car = spawnLoadVehicle(bridge.world, {});

    // Fixed timestep and seeded randomness for determinism
    const dt = 1/60;
    const seed = 1337;
    let rand = seed;
    const rand01 = () => { rand = (rand * 1664525 + 1013904223) >>> 0; return (rand & 0xffff) / 0x10000; };

    let handleCount = 0;
    for (let i = 0; i < 480; i++) {
      // Match browser loop ordering: update dynamics → step world → drain → apply forces → process
      if (rand01() < 0.02) bridge.projectiles.push(spawnProjectile(bridge.world, { kind: 'box' }));
      bridge.projectiles = updateProjectiles(bridge.world, bridge.projectiles, dt);
      updateLoadVehicle(bridge.world, bridge.car, dt);

      bridge.world.step(bridge.eventQueue);
      drainContactForces(bridge);

      applyForcesAndSolve(bridge);
      if (bridge.overstressed > 0) processSolverFractures(bridge, bridge.world.RAPIER, 0.0);

      handleCount = bridge._handleSplitEventsCount ?? 0;
      if (handleCount >= 2) { break; }
    }

    expect(handleCount).toBeGreaterThanOrEqual(2);

    // Assert normalized split log against a stable expectation schema (shape-only)
    expect(Array.isArray(bridge._splitLog)).toBe(true);
    const flat = (bridge._splitLog ?? []).flat();
    expect(flat.length).toBeGreaterThan(0);
    flat.forEach((evt: any) => {
      expect(typeof evt.parentActorIndex).toBe('number');
      expect(Array.isArray(evt.children)).toBe(true);
      evt.children.forEach((c: any) => {
        expect(typeof c.actorIndex).toBe('number');
        expect(Array.isArray(c.nodes)).toBe(true);
        // nodes are sorted
        for (let k = 1; k < c.nodes.length; k++) {
          expect(c.nodes[k]).toBeGreaterThanOrEqual(c.nodes[k-1]);
        }
      });
    });

    // Optional: compare to a predefined snapshot shape by counts
    const summary = flat.map((evt: any) => ({ parent: evt.parentActorIndex, childCount: evt.children.length }));
    expect(summary.length).toBeGreaterThan(0);

    // Strict equality vs browser-captured first split result
    const __filename = fileURLToPath(import.meta.url);
    const __dirname = dirname(__filename);
    const fixturePath = resolve(__dirname, '../tests/fixtures/bridge_splitResults_1.json');
    const fixtureRaw = JSON.parse(readFileSync(fixturePath, 'utf-8'));
    const expectedFirst = normalizeSplitResults(fixtureRaw);
    const actualFirst = (bridge._splitLog ?? [])[0];
    console.log('    actualFirst:')
    console.log(JSON.stringify(actualFirst));
    console.log('    expectedFirst:')
    console.log(JSON.stringify(expectedFirst));
    expect(actualFirst).toEqual(expectedFirst);
  });
});


