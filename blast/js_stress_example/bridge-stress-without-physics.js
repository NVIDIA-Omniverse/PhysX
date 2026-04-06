import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

import { loadStressSolver, vec3, formatNumber, ExtDebugMode } from './stress.js';
import { buildBridgeScenario } from './extBridgeScenario.js';

const GRAVITY_DEFAULT = -9.81;
const MAX_EVENTS = 8;
const IMPACT_MULTIPLIER = 3.5;

const CAR_PROPS = {
  mass: 5200,
  length: 3.6,
  width: 1.8,
  height: 1.2,
  speed: 4.5,
  idleDuration: 0.75
};

const BASE_LIMITS = {
  compressionElasticLimit: 0.05,
  compressionFatalLimit: 0.1,
  tensionElasticLimit: 0.05,
  tensionFatalLimit: 0.1,
  shearElasticLimit: 0.05,
  shearFatalLimit: 0.1
};

const HUD = {
  gravityValue: document.getElementById('gravity-value'),
  strengthValue: document.getElementById('strength-value'),
  iterValue: document.getElementById('iter-value'),
  toleranceValue: document.getElementById('tolerance-value'),
  bondTable: document.getElementById('bond-table'),
  eventLog: document.getElementById('event-log'),
  overlay: document.getElementById('overlay')
};

const controlsUI = {
  gravitySlider: document.getElementById('gravity-slider'),
  strengthSlider: document.getElementById('strength-slider'),
  fireButton: document.getElementById('fire-projectile'),
  resetButton: document.getElementById('reset-bridge'),
  debugToggle: document.getElementById('toggle-debug'),
  iterSlider: document.getElementById('iter-slider'),
  toleranceSlider: document.getElementById('tolerance-slider')
};

async function init() {
  try {
    const runtime = await loadStressSolver();
    const { scene, renderer, camera, controls } = initThree();
    const bridge = buildBridge(scene, runtime);

    setupControls(bridge);

    const clock = new THREE.Clock();
    function loop() {
      const delta = clock.getDelta();
      updateBridge(bridge, delta);
      controls.update();
      renderer.render(scene, camera);
      requestAnimationFrame(loop);
    }

    loop();
  } catch (err) {
    console.error('Failed to initialize bridge demo', err);
    pushEvent(`Initialization failed: ${err?.message ?? err}`);
  }
}

function initThree() {
  const canvas = document.getElementById('bridge-canvas');
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.shadowMap.enabled = true;

  const initialWidth = canvas.clientWidth || window.innerWidth;
  const initialHeight = canvas.clientHeight || window.innerHeight;
  renderer.setSize(initialWidth, initialHeight, false);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x05070a);

  const camera = new THREE.PerspectiveCamera(60, initialWidth / initialHeight, 0.1, 500);
  camera.position.set(0, 8, 18);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.target.set(0, 0, 0);

  const ambient = new THREE.AmbientLight(0xffffff, 0.35);
  scene.add(ambient);
  const dir = new THREE.DirectionalLight(0xffffff, 0.85);
  dir.position.set(12, 18, 10);
  dir.castShadow = true;
  dir.shadow.mapSize.set(2048, 2048);
  scene.add(dir);

  window.addEventListener('resize', () => {
    const width = canvas.clientWidth || window.innerWidth;
    const height = window.innerHeight - 1;
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  });

  return { scene, renderer, camera, controls };
}

function buildBridge(scene, runtime) {
  const scenario = buildBridgeScenario();
  const settings = runtime.defaultExtSettings();
  settings.maxSolverIterationsPerFrame = 32;
  settings.graphReductionLevel = 0;
  applyStrengthScale(settings, BASE_LIMITS, 1.0);

  const solver = runtime.createExtSolver({
    nodes: scenario.nodes,
    bonds: scenario.bonds,
    settings
  });

  const nodeMeshes = createNodeMeshesForScenario(scenario);
  scene.add(nodeMeshes.group);

  const bondLines = createBondLines(scenario);
  scene.add(bondLines);

  const debugHelper = createDebugLineHelper();
  scene.add(debugHelper.object);

  const ground = new THREE.Mesh(
    new THREE.BoxGeometry(40, 1, 40),
    new THREE.MeshStandardMaterial({ color: 0x1a1e2f, roughness: 0.8, metalness: 0.1 })
  );
  ground.position.set(0, -4, 0);
  ground.receiveShadow = true;
  scene.add(ground);

  const car = createCar(scenario);
  scene.add(car.mesh);

  const bridge = {
    runtime,
    solver,
    scenario,
    settings,
    baseLimits: { ...BASE_LIMITS },
    nodeMeshes,
    bondLines,
    debugHelper,
    car,
    gravity: GRAVITY_DEFAULT,
    strengthScale: 1.0,
    targetError: 1.0e-6,
    contactNodes: [],
    iterationCount: 0,
    lastError: { lin: 0, ang: 0 },
    overstressed: 0,
    failureDetected: false,
    debugEnabled: true,
    highlightSummary: [],
    forceBoost: 1.0
  };

  debugHelper.currentLines = [];
  updateDebugLineObject(debugHelper, [], bridge.debugEnabled);

  pushEvent(
    `Bridge scenario loaded with ${scenario.nodes.length} nodes, ${scenario.bonds.length} bonds`
  );
  updateOverlay(bridge, { column: -1, nodes: [] });
  updateBondTable([]);
  updateStrengthDisplay(bridge);
  updateIterationDisplay(bridge);
  updateToleranceDisplay(bridge);

  return bridge;
}

function createNodeMeshesForScenario(scenario) {
  const group = new THREE.Group();
  const deckMaterial = new THREE.MeshStandardMaterial({
    color: 0x486fe3,
    roughness: 0.35,
    metalness: 0.45
  });
  const topMaterial = new THREE.MeshStandardMaterial({
    color: 0x5b86ff,
    roughness: 0.28,
    metalness: 0.55
  });
  const deckMeshes = [];
  const meshByNode = new Map();

  const spacing = scenario.spacing;
  scenario.nodes.forEach((node, index) => {
    const coord = scenario.gridCoordinates[index];
    if (!coord || coord.iy < 0) {
      return;
    }
    const sizeX = Math.max(spacing.x * 0.9, 0.25);
    const sizeY = Math.max(spacing.y * 0.9, 0.2);
    const sizeZ =
      scenario.widthSegments > 1
        ? Math.max(spacing.z * 0.9, 0.25)
        : Math.max(scenario.parameters.deckWidth * 0.9, 0.8);
    const geometry = new THREE.BoxGeometry(sizeX, sizeY, sizeZ);
    const material = coord.iy === scenario.thicknessLayers - 1 ? topMaterial.clone() : deckMaterial.clone();
    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.position.set(node.centroid.x, node.centroid.y, node.centroid.z);
    mesh.userData.nodeIndex = index;
    group.add(mesh);
    deckMeshes.push({ nodeIndex: index, mesh });
    meshByNode.set(index, mesh);
  });

  const supportMaterial = new THREE.MeshStandardMaterial({
    color: 0x2f3e56,
    roughness: 0.6,
    metalness: 0.25
  });

  scenario.supportLinks.forEach((link) => {
    const deckNode = scenario.nodes[link.deckIndex];
    if (!deckNode) {
      return;
    }
    const columnHeight = scenario.parameters.pierHeight;
    const columnWidth = Math.max(spacing.x * 0.6, 0.4);
    const columnDepth =
      scenario.widthSegments > 1
        ? Math.max(spacing.z * 0.6, 0.4)
        : Math.max(scenario.parameters.deckWidth * 0.4, 0.4);
    const geometry = new THREE.BoxGeometry(columnWidth, columnHeight, columnDepth);
    const mesh = new THREE.Mesh(geometry, supportMaterial.clone());
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    const topY = deckNode.centroid.y - spacing.y * 0.5;
    mesh.position.set(deckNode.centroid.x, topY - columnHeight * 0.5, deckNode.centroid.z);
    group.add(mesh);
  });

  return { group, deckMeshes, meshByNode };
}

function createBondLines(scenario) {
  const geometry = new THREE.BufferGeometry();
  const count = scenario.bonds.length;
  const positions = new Float32Array(count * 2 * 3);
  const colors = new Float32Array(count * 2 * 3);
  const color = new THREE.Color(0x30353f);

  scenario.bonds.forEach((bond, index) => {
    const nodeA = scenario.nodes[bond.node0];
    const nodeB = scenario.nodes[bond.node1];
    const base = index * 6;
    positions[base] = nodeA.centroid.x;
    positions[base + 1] = nodeA.centroid.y;
    positions[base + 2] = nodeA.centroid.z;
    positions[base + 3] = nodeB.centroid.x;
    positions[base + 4] = nodeB.centroid.y;
    positions[base + 5] = nodeB.centroid.z;

    colors[base] = color.r;
    colors[base + 1] = color.g;
    colors[base + 2] = color.b;
    colors[base + 3] = color.r;
    colors[base + 4] = color.g;
    colors[base + 5] = color.b;
  });

  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));

  const material = new THREE.LineBasicMaterial({
    vertexColors: true,
    transparent: true,
    opacity: 0.35
  });

  return new THREE.LineSegments(geometry, material);
}

function createDebugLineHelper() {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
  geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(0), 3));

  const material = new THREE.LineBasicMaterial({
    vertexColors: true,
    transparent: true,
    opacity: 0.95,
    depthTest: false
  });

  const object = new THREE.LineSegments(geometry, material);
  object.visible = false;

  return { object, geometry, material, currentLines: [] };
}

function createCar(scenario) {
  const geometry = new THREE.BoxGeometry(CAR_PROPS.length, CAR_PROPS.height, CAR_PROPS.width);
  const material = new THREE.MeshStandardMaterial({ color: 0xffd166, roughness: 0.4, metalness: 0.6 });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;

  const minX = scenario.origins.x;
  const maxX = scenario.origins.x + scenario.spacing.x * (scenario.spanSegments - 1);
  const topColumn = scenario.topColumnNodes[0]?.[0];
  const topNode = topColumn != null ? scenario.nodes[topColumn] : { centroid: vec3(0, 0, 0) };
  const deckTopY = topNode.centroid.y + scenario.spacing.y * 0.5;
  const margin = CAR_PROPS.length * 0.5;

  mesh.position.set(minX - margin, deckTopY + CAR_PROPS.height * 0.5 + 0.1, 0);

  return {
    mesh,
    mass: CAR_PROPS.mass,
    speed: CAR_PROPS.speed,
    baseSpeed: CAR_PROPS.speed,
    direction: 1,
    waitTimer: CAR_PROPS.idleDuration,
    active: true,
    minX,
    maxX,
    margin
  };
}

function updateBridge(bridge, delta) {
  advanceCar(bridge, delta);
  const contact = computeContactNodes(bridge);
  bridge.contactNodes = contact.nodes;

  runSolver(bridge, contact);
  updateNodeColors(bridge);
  updateBondTable(bridge.highlightSummary);
  updateOverlay(bridge, contact);

  if (bridge.forceBoost > 1.0) {
    bridge.forceBoost = THREE.MathUtils.lerp(bridge.forceBoost, 1.0, Math.min(delta * 0.75, 1.0));
  }
}

function advanceCar(bridge, delta) {
  const car = bridge.car;
  if (!car.active) {
    return;
  }
  if (bridge.failureDetected) {
    return;
  }
  if (car.waitTimer > 0) {
    car.waitTimer = Math.max(0, car.waitTimer - delta);
    return;
  }
  car.mesh.position.x += car.speed * delta * car.direction;

  if (car.mesh.position.x > car.maxX + car.margin) {
    car.mesh.position.x = car.maxX + car.margin;
    car.direction = -1;
    car.waitTimer = CAR_PROPS.idleDuration;
  } else if (car.mesh.position.x < car.minX - car.margin) {
    car.mesh.position.x = car.minX - car.margin;
    car.direction = 1;
    car.waitTimer = CAR_PROPS.idleDuration;
  }
}

function computeContactNodes(bridge) {
  const { car, scenario } = bridge;
  const deckLength = car.maxX - car.minX;
  if (deckLength <= 0) {
    return { column: -1, nodes: [] };
  }
  const progress = (car.mesh.position.x - car.minX) / deckLength;
  if (progress <= 0 || progress >= 1) {
    return { column: -1, nodes: [] };
  }
  const segments = scenario.spanSegments;
  const clamped = THREE.MathUtils.clamp(progress, 0, 0.999);
  const column = Math.min(Math.floor(clamped * (segments - 1)), segments - 2);
  const nodes = [
    ...scenario.topColumnNodes[column],
    ...scenario.topColumnNodes[column + 1]
  ];
  return { column, nodes };
}

function runSolver(bridge, contact) {
  const { solver } = bridge;
  solver.reset();
  solver.addGravity(vec3(0.0, bridge.gravity, 0.0));

  const contactNodes = contact.nodes ?? [];
  if (contactNodes.length > 0) {
    const baseForcePerNode = (bridge.car.mass * 9.81) / contactNodes.length;
    const severity = contact.column >= 0 ? 1.0 + contact.column * 1.4 : 1.0;
    const half = Math.floor(contactNodes.length / 2);
    contactNodes.forEach((nodeIndex, idx) => {
      const dynamicBoost = idx < half ? 1.0 : IMPACT_MULTIPLIER;
      const downwardForce = baseForcePerNode * severity * dynamicBoost * bridge.forceBoost;
      solver.addForce(nodeIndex, vec3(), vec3(0.0, -downwardForce, 0.0));
    });
  }

  let iterations = 0;
  let error = { lin: 0, ang: 0 };
  const maxIterations = bridge.settings.maxSolverIterationsPerFrame;
  for (; iterations < maxIterations; ++iterations) {
    solver.update();
    error = solver.stressError();
    if (solver.converged()) {
      break;
    }
    if (error.lin <= bridge.targetError && error.ang <= bridge.targetError) {
      break;
    }
  }

  const overstressed = solver.overstressedBondCount();
  const debugLines = solver.fillDebugRender({ mode: ExtDebugMode.Max, scale: 1.0 });

  bridge.iterationCount = iterations + 1;
  bridge.lastError = error;
  bridge.overstressed = overstressed;
  bridge.highlightSummary = summarizeLines(debugLines, 6);
  bridge.debugHelper.currentLines = debugLines;

  updateDebugLineObject(bridge.debugHelper, debugLines, bridge.debugEnabled);

  if (!bridge.failureDetected && overstressed > 0) {
    bridge.failureDetected = true;
    bridge.car.active = false;
    pushEvent('Bridge deck predicted to fail under current car load. Simulation paused.');
  }
}

function updateNodeColors(bridge) {
  const baseColor = new THREE.Color(0x486fe3);
  const failureColor = new THREE.Color(0xff4d4d);
  const highlightColor = new THREE.Color(0xffd166);

  const contactSet = new Set(bridge.contactNodes);

  bridge.nodeMeshes.deckMeshes.forEach(({ nodeIndex, mesh }) => {
    if (bridge.failureDetected) {
      mesh.material.color.copy(failureColor);
      return;
    }
    if (contactSet.has(nodeIndex)) {
      mesh.material.color.copy(highlightColor);
    } else {
      mesh.material.color.copy(baseColor);
    }
  });
}

function updateDebugLineObject(debugHelper, debugLines, enabled) {
  if (!enabled || !debugLines || debugLines.length === 0) {
    debugHelper.geometry.setAttribute(
      'position',
      new THREE.BufferAttribute(new Float32Array(0), 3)
    );
    debugHelper.geometry.setAttribute(
      'color',
      new THREE.BufferAttribute(new Float32Array(0), 3)
    );
    debugHelper.object.visible = false;
    return;
  }

  const positions = new Float32Array(debugLines.length * 2 * 3);
  const colors = new Float32Array(debugLines.length * 2 * 3);

  debugLines.forEach((line, index) => {
    const base = index * 6;
    positions[base] = line.p0.x;
    positions[base + 1] = line.p0.y;
    positions[base + 2] = line.p0.z;
    positions[base + 3] = line.p1.x;
    positions[base + 4] = line.p1.y;
    positions[base + 5] = line.p1.z;

    const color0 = unpackColor(line.color0);
    const color1 = unpackColor(line.color1 ?? line.color0);

    colors[base] = color0.r;
    colors[base + 1] = color0.g;
    colors[base + 2] = color0.b;
    colors[base + 3] = color1.r;
    colors[base + 4] = color1.g;
    colors[base + 5] = color1.b;
  });

  debugHelper.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  debugHelper.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  debugHelper.geometry.computeBoundingSphere();
  debugHelper.object.visible = true;
}

function summarizeLines(lines, limit = 3) {
  const sorted = [...lines].sort((a, b) => lineMagnitude(b) - lineMagnitude(a));
  return sorted.slice(0, limit).map((line) => {
    const length = lineMagnitude(line);
    return {
      length,
      centerX: (line.p0.x + line.p1.x) / 2,
      centerY: (line.p0.y + line.p1.y) / 2,
      centerZ: (line.p0.z + line.p1.z) / 2,
      color: toColorString(line.color0)
    };
  });
}

function lineMagnitude(line) {
  const dx = line.p1.x - line.p0.x;
  const dy = line.p1.y - line.p0.y;
  const dz = line.p1.z - line.p0.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

function unpackColor(value) {
  const r = ((value >> 16) & 0xff) / 255;
  const g = ((value >> 8) & 0xff) / 255;
  const b = (value & 0xff) / 255;
  return { r, g, b };
}

function toColorString(value) {
  const hex = (value & 0xffffff).toString(16).padStart(6, '0');
  return `#${hex}`;
}

function updateBondTable(highlights) {
  if (!HUD.bondTable) {
    return;
  }
  HUD.bondTable.innerHTML = '';

  if (!highlights || highlights.length === 0) {
    const empty = document.createElement('div');
    empty.className = 'bond-row';
    empty.textContent = 'No stressed bonds reported';
    HUD.bondTable.appendChild(empty);
    return;
  }

  highlights.forEach((highlight, index) => {
    const row = document.createElement('div');
    row.className = 'bond-row';

    const title = document.createElement('div');
    title.textContent = `Line ${index + 1}`;
    row.appendChild(title);

    const value = document.createElement('div');
    value.textContent = `${formatNumber(highlight.length, 6, 3)} m`;
    row.appendChild(value);

    const info = document.createElement('div');
    info.className = 'bond-info';
    info.textContent = `center (${formatNumber(highlight.centerX, 6, 2)}, ${formatNumber(
      highlight.centerY,
      6,
      2
    )}, ${formatNumber(highlight.centerZ, 6, 2)})`;
    row.appendChild(info);

    const colorLabel = document.createElement('div');
    colorLabel.className = 'bond-color';
    colorLabel.style.backgroundColor = highlight.color;
    row.appendChild(colorLabel);

    HUD.bondTable.appendChild(row);
  });
}

function updateOverlay(bridge, contact) {
  if (!HUD.overlay) {
    return;
  }
  const contactLabel =
    contact.column >= 0 ? `${contact.column} – ${contact.column + 1}` : 'off-deck';
  HUD.overlay.innerHTML = `
    <div>Car span: ${contactLabel}</div>
    <div>Iterations: ${bridge.iterationCount}/${bridge.settings.maxSolverIterationsPerFrame}</div>
    <div>Error lin ${formatNumber(bridge.lastError.lin, 7, 4)} | ang ${formatNumber(
      bridge.lastError.ang,
      7,
      4
    )}</div>
    <div>Overstressed bonds: ${bridge.overstressed}</div>
    <div>Force multiplier: ${bridge.forceBoost.toFixed(2)}x</div>
    <div>Status: ${bridge.failureDetected ? 'FAILED' : 'Stable'}</div>
  `;
}

function pushEvent(message) {
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

function resetBridge(bridge) {
  bridge.failureDetected = false;
  bridge.overstressed = 0;
  bridge.iterationCount = 0;
  bridge.lastError = { lin: 0, ang: 0 };
  bridge.forceBoost = 1.0;
  bridge.contactNodes = [];
  bridge.car.mesh.position.x = bridge.car.minX - bridge.car.margin;
  bridge.car.direction = 1;
  bridge.car.waitTimer = CAR_PROPS.idleDuration;
  bridge.car.speed = bridge.car.baseSpeed;
  bridge.car.active = true;
  bridge.solver.reset();
  bridge.debugHelper.currentLines = [];
  updateDebugLineObject(bridge.debugHelper, [], bridge.debugEnabled);
  updateNodeColors(bridge);
  updateBondTable([]);
  updateOverlay(bridge, { column: -1, nodes: [] });
  if (HUD.eventLog) {
    HUD.eventLog.innerHTML = '';
  }
  pushEvent('Bridge reset to initial state');
}

function applyStrengthScale(settings, baseLimits, scale) {
  settings.compressionElasticLimit = baseLimits.compressionElasticLimit * scale;
  settings.compressionFatalLimit = baseLimits.compressionFatalLimit * scale;
  settings.tensionElasticLimit = baseLimits.tensionElasticLimit * scale;
  settings.tensionFatalLimit = baseLimits.tensionFatalLimit * scale;
  settings.shearElasticLimit = baseLimits.shearElasticLimit * scale;
  settings.shearFatalLimit = baseLimits.shearFatalLimit * scale;
}

function setupControls(bridge) {
  if (controlsUI.gravitySlider) {
    controlsUI.gravitySlider.value = GRAVITY_DEFAULT.toString();
    controlsUI.gravitySlider.addEventListener('input', (event) => {
      const value = parseFloat(event.target.value);
      bridge.gravity = Number.isFinite(value) ? value : GRAVITY_DEFAULT;
      if (HUD.gravityValue) {
        HUD.gravityValue.textContent = bridge.gravity.toFixed(2);
      }
      pushEvent(`Gravity set to ${bridge.gravity.toFixed(2)} m/s²`);
    });
    if (HUD.gravityValue) {
      HUD.gravityValue.textContent = GRAVITY_DEFAULT.toFixed(2);
    }
  }

  if (controlsUI.strengthSlider) {
    controlsUI.strengthSlider.value = bridge.strengthScale.toString();
    controlsUI.strengthSlider.addEventListener('input', (event) => {
      const value = parseFloat(event.target.value);
      bridge.strengthScale = Number.isFinite(value) ? value : 1.0;
      applyStrengthScale(bridge.settings, bridge.baseLimits, bridge.strengthScale);
      bridge.solver.setSettings(bridge.settings);
      updateStrengthDisplay(bridge);
      pushEvent(`Material strength scaled to ${(bridge.strengthScale * 100).toFixed(1)}%`);
    });
  }

  if (controlsUI.iterSlider) {
    controlsUI.iterSlider.value = bridge.settings.maxSolverIterationsPerFrame.toString();
    controlsUI.iterSlider.addEventListener('input', (event) => {
      const value = parseInt(event.target.value, 10);
      bridge.settings.maxSolverIterationsPerFrame = Number.isFinite(value) ? value : 32;
      bridge.solver.setSettings(bridge.settings);
      updateIterationDisplay(bridge);
    });
  }

  if (controlsUI.toleranceSlider) {
    const initial = Math.log10(bridge.targetError);
    controlsUI.toleranceSlider.value = initial.toString();
    controlsUI.toleranceSlider.addEventListener('input', (event) => {
      const exponent = parseFloat(event.target.value);
      bridge.targetError = Math.pow(10, exponent);
      updateToleranceDisplay(bridge);
    });
  }

  if (controlsUI.fireButton) {
    controlsUI.fireButton.addEventListener('click', () => {
      bridge.forceBoost = Math.min(bridge.forceBoost + 0.75, 6.0);
      pushEvent(`Applied transient shock (force multiplier ${bridge.forceBoost.toFixed(2)}x)`);
    });
  }

  if (controlsUI.resetButton) {
    controlsUI.resetButton.addEventListener('click', () => resetBridge(bridge));
  }

  if (controlsUI.debugToggle) {
    controlsUI.debugToggle.addEventListener('click', () => {
      bridge.debugEnabled = !bridge.debugEnabled;
      controlsUI.debugToggle.textContent = bridge.debugEnabled ? 'Hide Debug' : 'Show Debug';
      updateDebugLineObject(
        bridge.debugHelper,
        bridge.debugHelper.currentLines,
        bridge.debugEnabled
      );
    });
    controlsUI.debugToggle.textContent = 'Hide Debug';
  }
}

function updateStrengthDisplay(bridge) {
  if (HUD.strengthValue) {
    HUD.strengthValue.textContent = `${(bridge.strengthScale * 100).toFixed(1)}%`;
  }
}

function updateIterationDisplay(bridge) {
  if (HUD.iterValue) {
    HUD.iterValue.textContent = bridge.settings.maxSolverIterationsPerFrame.toString();
  }
}

function updateToleranceDisplay(bridge) {
  if (HUD.toleranceValue) {
    HUD.toleranceValue.textContent = formatTolerance(bridge.targetError);
  }
}

function formatTolerance(value) {
  return Number(value).toExponential(2);
}

init().catch((err) => {
  console.error(err);
});

