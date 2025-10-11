import { beforeAll, describe, expect, it } from 'vitest';
import { buildBridgeShared } from '../bridge/buildBridge.headless.js';
import { applyForcesAndSolve, processSolverFractures, drainContactForces } from '../bridge/coreLogic.js';
import { spawnLoadVehicle, updateLoadVehicle, spawnProjectile, updateProjectiles } from '../bridge/dynamics.js';

describe('Organic split flow (no manual handleSplitEvents)', () => {
  beforeAll(async () => {
    // RAPIER is in bridge-sim builder
  });

  it('produces split events and updates actorMap using the same core logic', async () => {
    const bridge = await buildBridgeShared({ gravity: -9.81, strengthScale: 0.05 });
    bridge.world.RAPIER = (await import('@dimforge/rapier3d-compat')).default;
    bridge.projectiles = [];
    bridge.car = spawnLoadVehicle(bridge.world, {});

    let broke = false;
    let handleCount = 0;
    for (let i = 0; i < 240; i++) {
      updateLoadVehicle(bridge.world, bridge.car, 1/60);
      if (Math.random() < 0.02) bridge.projectiles.push(spawnProjectile(bridge.world, { kind: 'box' }));
      bridge.projectiles = updateProjectiles(bridge.world, bridge.projectiles, 1/60);

      applyForcesAndSolve(bridge);
      if (bridge.overstressed > 0) processSolverFractures(bridge, bridge.world.RAPIER, 0.0);
      bridge.world.step(bridge.eventQueue);
      drainContactForces(bridge);

      handleCount = bridge._handleSplitEventsCount ?? 0;
      if (handleCount > 0) { broke = true; break; }
    }

    expect(handleCount).toBeGreaterThan(0);
    expect(broke).toBe(true);
  });
});


