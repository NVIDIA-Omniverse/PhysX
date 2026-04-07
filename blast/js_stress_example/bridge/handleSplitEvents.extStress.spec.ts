import { beforeAll, describe, expect, it } from 'vitest';
import RAPIER from '@dimforge/rapier3d-compat';
import { buildBridgeShared } from '../bridge/buildBridge.headless.js';
import { applyForcesAndSolve, processSolverFractures, drainContactForces } from '../bridge/coreLogic.js';
import { spawnLoadVehicle, updateLoadVehicle, spawnProjectile, updateProjectiles } from '../bridge/dynamics.js';

function makeBridgeChunk(nodeIndex: number, centroid: { x:number;y:number;z:number }, size: { x:number;y:number;z:number }, isSupport: boolean) {
  return {
    nodeIndex,
    size,
    isSupport,
    baseLocalOffset: { x: centroid.x, y: centroid.y, z: centroid.z },
    localOffset: { x: centroid.x, y: centroid.y, z: centroid.z, copy(v: any) { this.x=v.x; this.y=v.y; this.z=v.z; } },
    mesh: null,
    colliderHandle: null as number | null,
    bodyHandle: null as number | null,
    active: true,
    detached: false
  };
}

describe('handleSplitEvents with ExtStressSolver-generated splits', () => {
  beforeAll(async () => {
    await RAPIER.init();
  });

  it('generates organic splits via the shared bridge simulation', async () => {
    const bridge = await buildBridgeShared({ gravity: -9.81, strengthScale: 0.03 });
    bridge.world.RAPIER = (await import('@dimforge/rapier3d-compat')).default;
    bridge.projectiles = [];
    bridge.car = spawnLoadVehicle(bridge.world, {});

    let broke = false;
    let handleCount = 0;
    for (let i = 0; i < 480; i++) {
      // Match browser loop ordering: update dynamics → step world → drain → apply forces → process
      if (Math.random() < 0.03) bridge.projectiles.push(spawnProjectile(bridge.world, { kind: 'box' }));
      bridge.projectiles = updateProjectiles(bridge.world, bridge.projectiles, 1/60, bridge);
      // keep car static during this test

      bridge.world.step(bridge.eventQueue);
      drainContactForces(bridge);

      applyForcesAndSolve(bridge);
      if (bridge.overstressed > 0) processSolverFractures(bridge, bridge.world.RAPIER, 0.0);

      handleCount = bridge._handleSplitEventsCount ?? 0;
      if (handleCount > 0) { broke = true; break; }
    }

    expect(handleCount).toBeGreaterThan(0);
    expect(broke).toBe(true);
  });
});


