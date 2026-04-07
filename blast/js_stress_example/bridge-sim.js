// Headless builder to construct the same world/bridge/solver as the web demo (minus Three.js).
import RAPIER from '@dimforge/rapier3d-compat';
import { buildBridgeShared } from './bridge/buildBridge.js';
import { applyForcesAndSolve, processSolverFractures, drainContactForces } from './bridge/coreLogic.js';
import { spawnLoadVehicle, updateLoadVehicle, spawnProjectile, updateProjectiles } from './bridge/dynamics.js';

export async function buildHeadlessBridge({ gravity = -9.81, strengthScale = 0.05 } = {}) {
  const bridge = await buildBridgeShared({ gravity, strengthScale });
  bridge.world.RAPIER = RAPIER;
  bridge.projectiles = [];
  bridge.car = spawnLoadVehicle(bridge.world, {});
  return bridge;
 

export function stepHeadlessBridge(bridge) {
  updateLoadVehicle(bridge.world, bridge.car, 1/60);
  if (Math.random() < 0.02) {
    bridge.projectiles.push(spawnProjectile(bridge.world, { kind: 'box' }));
  }
  bridge.projectiles = updateProjectiles(bridge.world, bridge.projectiles, 1/60);

  // Advance solver, apply splits
  applyForcesAndSolve(bridge);
  if (bridge.overstressed > 0) {
    processSolverFractures(bridge, RAPIER, 0.0);
  }

  // 2) Step Rapier and feed back contacts to solver for next frame
  bridge.world.step(bridge.eventQueue);
  drainContactForces(bridge);

  return {
    handleSplitEventsCount: bridge._handleSplitEventsCount ?? 0,
    overstressed: bridge.overstressed
  };
}


