import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';

import { loadStressSolver } from '../stress.js';
import { initThree } from './scene.js';
import { buildBridge, spawnLoadVehicle } from './buildBridge.js';
import { setupControls } from './controls.js';
import { updateBridge } from './simulation.js';
import { pushEvent } from './ui.js';
import { GRAVITY_DEFAULT } from './constants.js';
import { toleranceFromExponent } from './utils.js';

async function init() {
  await RAPIER.init();
  const stressRuntime = await loadStressSolver();

  const world = new RAPIER.World(new RAPIER.Vector3(0, GRAVITY_DEFAULT, 0));
  const { scene, renderer, camera, controls } = initThree();
  const bridge = buildBridge(scene, world, stressRuntime);

  spawnLoadVehicle(world, bridge);
  setupControls(world, bridge);

  bridge.stressProcessor.setSolverParams({
    maxIterations: bridge.solverSettings.maxIterations,
    tolerance: toleranceFromExponent(bridge.solverSettings.toleranceExponent)
  });

  const clock = new THREE.Clock();

  function loop() {
    const delta = clock.getDelta();
    updateBridge(world, bridge, delta);
    world.step();
    if (bridge.debugRenderer?.enabled) {
      bridge.debugRenderer.update();
    }
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(loop);
  }

  loop();
}

init().catch((err) => {
  console.error('Failed to initialize bridge demo', err);
  pushEvent(`Initialization failed: ${err.message ?? err}`);
});

