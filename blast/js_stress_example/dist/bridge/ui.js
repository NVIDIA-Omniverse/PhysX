import { MAX_EVENTS } from './constants.js';
import { stressColors } from './simulation.js';
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
    return;
    if (!HUD.bondTable) {
        return;
    }
    HUD.bondTable.innerHTML = '';
    const sorted = [...bonds].sort((a, b) => {
        if (a.active === b.active) {
            return b.severity - a.severity;
        }
        return a.active ? -1 : 1;
    });
    sorted.forEach((bond) => {
        const row = document.createElement('div');
        row.className = `bond-row${bond.active ? '' : ' bond-row--inactive'}`;
        const name = document.createElement('div');
        name.className = 'bond-name';
        name.textContent = bond.name ?? bond.key ?? `Bond ${bond.index}`;
        row.appendChild(name);
        const status = document.createElement('div');
        status.className = 'bond-status';
        status.textContent = bond.active ? `${formatPercent(bond.severity)} stress` : 'broken';
        row.appendChild(status);
        const meter = document.createElement('div');
        meter.className = 'bond-meter';
        const fill = document.createElement('span');
        const fillValue = Math.min(1, Math.max(0, bond.severity ?? 0));
        fill.style.setProperty('--fill', `${fillValue}`);
        fill.style.width = `${fillValue * 100}%`;
        meter.appendChild(fill);
        const badge = document.createElement('i');
        badge.className = 'bond-badge';
        badge.style.backgroundColor = bondColorForSeverity(bond);
        meter.appendChild(badge);
        row.appendChild(meter);
        HUD.bondTable.appendChild(row);
    });
}
function formatPercent(value = 0) {
    return `${Math.round(value * 100)}%`;
}
function bondColorForSeverity(bond) {
    if (!bond.active) {
        return 'rgba(255, 94, 105, 0.8)';
    }
    const v = Math.min(1, Math.max(0, bond.severity ?? 0));
    const base = stressColors.low.clone().lerp(stressColors.high, v);
    return `rgb(${Math.round(base.r * 255)}, ${Math.round(base.g * 255)}, ${Math.round(base.b * 255)})`;
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
export function setDebugToggleLabel(enabled) {
    if (controlsUI.debugToggle) {
        controlsUI.debugToggle.textContent = enabled ? 'Hide Debug' : 'Show Debug';
    }
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
