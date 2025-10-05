import RAPIER from '@dimforge/rapier3d-compat';

import { GRAVITY_DEFAULT, STRENGTH_DEFAULT, SOLVER_DEFAULTS } from './constants.js';
import { HUD, pushEvent, controlsUI, updateStrengthUI, updateIterationsUI, updateToleranceUI } from './ui.js';
import { spawnProjectile } from './spawning.js';
import { scaleStressLimits, clamp, toleranceFromExponent } from './utils.js';

export function setupControls(world, bridge) {
  if (!bridge.solverSettings) {
    bridge.solverSettings = {
      maxIterations: SOLVER_DEFAULTS.maxIterations,
      toleranceExponent: SOLVER_DEFAULTS.toleranceExponent
    };
  }

  if (controlsUI.gravitySlider) {
    controlsUI.gravitySlider.value = GRAVITY_DEFAULT.toString();
    controlsUI.gravitySlider.addEventListener('input', (event) => {
      const value = parseFloat(event.target.value);
      world.gravity = new RAPIER.Vector3(0, value, 0);
      bridge.activeGravity = value;
      if (HUD.gravityValue) {
        HUD.gravityValue.textContent = value.toFixed(2);
      }
      pushEvent(`Gravity set to ${value.toFixed(2)} m/s²`);
    });
  }
  if (controlsUI.fireButton) {
    controlsUI.fireButton.addEventListener('click', () => {
      spawnProjectile(world, bridge);
    });
  }
  if (controlsUI.resetButton) {
    controlsUI.resetButton.addEventListener('click', () => {
      window.location.reload();
    });
  }
  if (controlsUI.strengthSlider) {
    controlsUI.strengthSlider.value = bridge.strengthScale.toString();
    updateStrengthUI(bridge.strengthScale);
    controlsUI.strengthSlider.addEventListener('input', (event) => {
      const value = parseFloat(event.target.value);
      bridge.strengthScale = clamp(Number.isFinite(value) ? value : STRENGTH_DEFAULT, 0, 5);
      bridge.limits = new bridge.limitsCtor(scaleStressLimits(bridge.baseLimits, bridge.strengthScale));
      const display = updateStrengthUI(bridge.strengthScale);
      pushEvent(`Material strength scaled to ${display}`);
    });
  }
  if (controlsUI.iterSlider) {
    controlsUI.iterSlider.value = bridge.solverSettings.maxIterations.toString();
    updateIterationsUI(bridge.solverSettings.maxIterations);
    controlsUI.iterSlider.addEventListener('input', (event) => {
      const value = parseInt(event.target.value, 10);
      bridge.solverSettings.maxIterations = clamp(value, 1, 256);
      bridge.stressProcessor.setSolverParams({
        maxIterations: bridge.solverSettings.maxIterations,
        tolerance: toleranceFromExponent(bridge.solverSettings.toleranceExponent)
      });
      updateIterationsUI(bridge.solverSettings.maxIterations);
      pushEvent(`Solver iterations set to ${bridge.solverSettings.maxIterations}`);
    });
  }
  if (controlsUI.toleranceSlider) {
    controlsUI.toleranceSlider.value = bridge.solverSettings.toleranceExponent.toString();
    updateToleranceUI(bridge.solverSettings.toleranceExponent);
    controlsUI.toleranceSlider.addEventListener('input', (event) => {
      const exponent = clamp(parseFloat(event.target.value), -12, -2);
      bridge.solverSettings.toleranceExponent = exponent;
      bridge.stressProcessor.setSolverParams({
        tolerance: toleranceFromExponent(exponent),
        maxIterations: bridge.solverSettings.maxIterations
      });
      updateToleranceUI(exponent);
      pushEvent(`Solver tolerance set to 1.0e${exponent.toFixed(2)}`);
    });
  }
  if (controlsUI.debugToggle && bridge.debugRenderer) {
    controlsUI.debugToggle.addEventListener('click', () => {
      const enabled = bridge.debugRenderer.toggle();
      controlsUI.debugToggle.textContent = enabled ? 'Hide Debug' : 'Show Debug';
    });
  }
}

