import { MAX_EVENTS } from './constants.js';

export const HUD = {
  gravityValue: document.getElementById('gravity-value'),
  bondTable: document.getElementById('bond-table'),
  eventLog: document.getElementById('event-log'),
  overlay: document.getElementById('overlay')
};

export const controlsUI = {
  gravitySlider: document.getElementById('gravity-slider'),
  fireButton: document.getElementById('fire-projectile'),
  resetButton: document.getElementById('reset-bridge'),
  debugToggle: document.getElementById('toggle-debug'),
  strengthSlider: document.getElementById('strength-slider'),
  strengthValue: document.getElementById('strength-value'),
  iterSlider: document.getElementById('iter-slider'),
  iterValue: document.getElementById('iter-value'),
  toleranceSlider: document.getElementById('tolerance-slider'),
  toleranceValue: document.getElementById('tolerance-value')
};

export function updateBondTable(bonds) {
  if (!HUD.bondTable) {
    return;
  }
  HUD.bondTable.innerHTML = '';
  bonds
    .filter((bond) => bond.active)
    .forEach((bond) => {
      const row = document.createElement('div');
      row.className = 'bond-row';
      const title = document.createElement('div');
      title.textContent = bond.name;
      row.appendChild(title);
      const value = document.createElement('div');
      value.textContent = `${(bond.severity.max * 100).toFixed(0)}%`;
      row.appendChild(value);
      const bar = document.createElement('div');
      bar.className = 'bars';
      const fill = document.createElement('span');
      fill.style.width = `${Math.min(1, bond.severity.max) * 100}%`;
      bar.appendChild(fill);
      row.appendChild(bar);
      HUD.bondTable.appendChild(row);
    });
}

export function pushEvent(message) {
  if (!HUD.eventLog) {
    return;
  }
  const item = document.createElement('li');
  item.textContent = message;
  HUD.eventLog.prepend(item);
  while (HUD.eventLog.children.length > MAX_EVENTS) {
    HUD.eventLog.removeChild(HUD.eventLog.lastChild);
  }
}

export function updateStrengthUI(strengthScale) {
  const value = `${(strengthScale * 100).toFixed(1)}%`;
  if (controlsUI.strengthValue) {
    controlsUI.strengthValue.textContent = value;
  }
  return value;
}

export function updateIterationsUI(iterations) {
  if (controlsUI.iterValue) {
    controlsUI.iterValue.textContent = `${iterations}`;
  }
}

export function updateToleranceUI(exponent) {
  if (controlsUI.toleranceValue) {
    controlsUI.toleranceValue.textContent = `1.0e${exponent.toFixed(2)}`;
  }
}

