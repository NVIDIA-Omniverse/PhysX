// Testing
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import Stats from 'three/addons/libs/stats.module.js';
import RAPIER from '@dimforge/rapier3d-compat';
import { RapierDebugRenderer } from './rapier-debug-renderer.js';
import {
  loadStressSolver,
  ExtForceMode,
  ExtDebugMode // (unused here, handy if you want to add a stress overlay)
} from './stress.js';
import { createBridgeCore } from './bridge/buildBridge.headless.js';
import { buildBridgeScenario } from './extBridgeScenario.js';
import { 
  ConfigManager, 
  DEFAULT_CONFIG, 
  CONFIG_DESCRIPTORS, 
  type DemoConfig,
  saveConfigToStorage,
  loadConfigFromStorage,
  clearConfigFromStorage
} from './config.js';

/**
 * Minimal stress-driven bridge fracture demo with stable identity tracking.
 * 
 * Key invariants:
 *  - NO Rapier world mutation outside the frame loop.
 *  - Mutations happen ONLY on SAFE frames (no event queue bound).
 *  - If a solver split child uses the parent actorIndex, REUSE the parent body (no new body).
 * 
 * Stable Identity System:
 *  - Each rigid body and collider gets human-readable metadata (name, type, etc.)
 *  - Segment colliders maintain stable names across handle changes (e.g., "Segment-5")
 *  - Snapshots capture world state before migrations to compute deltas
 *  - Delta tracking shows: NEW bodies/colliders, MOVED colliders, handle changes (recreated)
 *  - After migrations, printWorldHierarchy() logs complete state with deltas
 */

// ---------- Tunables ----------
const BALL_RADIUS = 0.35;
const BALL_DROP_Y = 8;

const GRAVITY = -9.81;
const SAFE_FRAMES_AFTER_SPLIT = 2;  // give BVH time to settle after migrations
const SAFE_FRAMES_AFTER_SPAWN = 1;  // spawn-only cycles

// ---------- Demo state ----------
const state = {
  // Three/Rapier
  world: null,
  eventQueue: null,
  scene: null,
  camera: null,
  renderer: null,
  controls: null,
  clock: null,
  debugRenderer: null,
  debugHelper: null,
  stats: null,

  // Bridge
  rootBodyHandle: null,
  chunks: /** @type {Array<{
    nodeIndex:number,
    localOffset: THREE.Vector3,
    size: {x:number,y:number,z:number},
    mesh: THREE.Mesh,
    colliderHandle: number,
    bodyHandle: number,
    detached: boolean,
    isSupport: boolean
  }>} */ ([]),

  // Balls
  balls: /** @type {Array<{ bodyHandle:number, mesh:THREE.Mesh }>} */ ([]),

  // Handle maps
  colliderToNode: /** @type {Map<number, number>} */ (new Map()),
  activeContactColliders: /** @type {Set<number>} */ (new Set()),
  disabledCollidersToRemove: /** @type {Set<number>} */ (new Set()),
  bodiesToRemove: /** @type {Set<number>} */ (new Set()),

  // Stable identity tracking
  bodyMetadata: /** @type {Map<number, {type:string, name:string, actorIndex?:number, createdAt:number}>} */ (new Map()),
  colliderMetadata: /** @type {Map<number, {type:string, name:string, segmentIndex?:number, nodeIndex?:number, createdAt:number}>} */ (new Map()),
  
  // Previous state snapshot for delta tracking
  previousWorldState: /** @type {{bodies:Map<number,any>, colliders:Map<number,any>, collidersByStableId?:Map<string,any>}} */ ({ bodies: new Map(), colliders: new Map(), collidersByStableId: new Map() }),
  
  // Counters for unique naming
  bodyIdCounter: 0,
  ballIdCounter: 0,

  // Queues (mutations applied in SAFE frames only)
  pendingBodiesToCreate: /** @type {Array<{ actorIndex:number, inheritFromBodyHandle:number, nodes:number[], isSupport:boolean }>} */ ([]),
  pendingColliderMigrations: /** @type {Array<{ nodeIndex:number, targetBodyHandle:number }>} */ ([]),
  pendingBallSpawns: /** @type {Array<{ x:number, z:number }>} */ ([]),

  // Safe-step countdown after any split/spawn mutation
  safeFrames: 0,

  // Picking helpers
  raycaster: new THREE.Raycaster(),
  ndc: new THREE.Vector2(),
  groundPlane: new THREE.Plane(new THREE.Vector3(0, 1, 0), 0), // y=0 plane

  // ------------ Stress Solver ------------
  runtime: null,
  solver: null,
  solverSettings: null,
  // actorIndex -> { bodyHandle }
  actorMap: /** @type {Map<number, { bodyHandle:number }>} */ (new Map()),
  
  // Additional bridge state
  scenario: null,
  pendingContactForces: /** @type {Map<number, {force:any, point:any, impulse:any}>} */ (new Map()),

  // ------------ Configuration Manager (NEW) ------------
  configManager: null as ConfigManager | null,
};

init().catch((e) => console.error('Demo init failed:', e));

async function init() {
  await RAPIER.init();
  
  // Initialize configuration manager with saved config or defaults
  const savedConfig = loadConfigFromStorage();
  if (savedConfig) {
    console.log('🔄 Restoring configuration from previous reset...');
    state.configManager = new ConfigManager(savedConfig);
  } else {
    console.log('🆕 Initializing with default configuration');
    state.configManager = new ConfigManager(DEFAULT_CONFIG);
  }
  
  // Load stress solver runtime FIRST
  state.runtime = await loadStressSolver();

  // Three
  const canvas = ensureCanvas();
  const { scene, camera, renderer, controls } = initThree(canvas);
  const stats = new Stats();
  try { document.body.appendChild(stats.dom); } catch {}
  Object.assign(state, { scene, camera, renderer, controls, stats, clock: new THREE.Clock() });

  // Rapier - use current config for gravity
  const currentConfig = state.configManager.getConfig();
  const world = new RAPIER.World({ x: 0, y: currentConfig.environment.gravity, z: 0 });
  state.world = world;
  state.eventQueue = new RAPIER.EventQueue(true);

  buildLights(scene);
  buildGround(scene, world);
  buildBridge(scene, world);

  // Debug renderer (enabled by default)
  const debugRenderer = new RapierDebugRenderer(scene, world, { enabled: true });
  state.debugRenderer = debugRenderer;
  // Stress solver debug line helper (rendered when physics debugger is enabled)
  const debugHelper = createDebugLineHelper();
  try { scene.add(debugHelper.object); } catch {}
  state.debugHelper = debugHelper;
  
  // Capture initial state as baseline
  captureWorldSnapshot();
  
  // Expose debug helpers globally
  window.debugBridge = {
    printHierarchy: () => printWorldHierarchy(),
    captureSnapshot: () => captureWorldSnapshot(),
    printSolver: () => debugPrintSolver(),
    state
  };
  
  console.log('%c🌉 Bridge Stress Demo Ready!', 'font-size: 16px; font-weight: bold; color: #4b6fe8');
  console.log('%cDebug Tools Available:', 'font-weight: bold');
  console.log('  • window.debugBridge.printHierarchy() - Print current world hierarchy');
  console.log('  • window.debugBridge.captureSnapshot() - Capture current state as baseline');
  console.log('  • window.debugBridge.state - Access demo state');
  console.log('\n💡 Click anywhere to drop a heavy ball and watch the bridge fracture!');

  // IMPORTANT: no direct world mutation in handlers; just queue the spawn
  canvas.addEventListener('pointerdown', (ev) => {
    const { camera, renderer, raycaster, ndc, groundPlane } = state;
    const rect = renderer.domElement.getBoundingClientRect();
    ndc.x = ((ev.clientX - rect.left) / rect.width) * 2 - 1;
    ndc.y = -((ev.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(ndc, camera);
    const point = new THREE.Vector3();
    if (raycaster.ray.intersectPlane(groundPlane, point)) {
      state.pendingBallSpawns.push({ x: point.x, z: point.z });
      state.safeFrames = Math.max(state.safeFrames, SAFE_FRAMES_AFTER_SPAWN);
    }
  });

  setupControls();
  loop();
}

// ---------- UI Controls ----------
function setupControls() {
  // Legacy gravity slider - now handled through config system
  const gravitySlider = document.getElementById('config-gravity');
  const gravityValue = document.getElementById('config-gravity-value');
  if (gravitySlider && gravityValue) {
    gravitySlider.addEventListener('input', (e) => {
      const newGravity = parseFloat((e.target as HTMLInputElement).value);
      gravityValue.textContent = newGravity.toFixed(2);
      if (state.world) {
        state.world.gravity = { x: 0, y: newGravity, z: 0 };
      }
      if (state.configManager) {
        state.configManager.set('environment', 'gravity', newGravity);
      }
    });
  }

  // Reset button
  const resetButton = document.getElementById('reset-bridge');
  if (resetButton) {
    resetButton.addEventListener('click', () => {
      // Save current configuration before reset
      if (state.configManager) {
        const currentConfig = state.configManager.getConfig();
        saveConfigToStorage(currentConfig);
        // Mark that config has been persisted for this reset
        state.configManager.confirmBridgeReset();
      }
      // Reload page - will restore config from localStorage on init
      location.reload();
    });
  }

  // Debug toggle
  const debugToggle = document.getElementById('toggle-debug');
  if (debugToggle) {
    debugToggle.textContent = 'Hide Debug Wireframe';
    debugToggle.addEventListener('click', () => {
      if (state.debugRenderer) {
        const enabled = state.debugRenderer.toggle();
        debugToggle.textContent = enabled ? 'Hide Debug Wireframe' : 'Show Debug Wireframe';
        try {
          if (state.debugHelper) {
            if (!enabled) {
              updateDebugLineObject(state.debugHelper, [], false);
            } else {
              state.debugHelper.object.visible = !!(state.debugHelper.currentLines && state.debugHelper.currentLines.length > 0);
            }
          }
        } catch {}
      }
    });
  }

  // Setup comprehensive configuration UI
  setupConfigurationUI();
}

/**
 * Setup all configuration UI controls with immediate and deferred updates
 */
function setupConfigurationUI() {
  if (!state.configManager) return;

  // Helper to update UI display value
  const updateDisplayValue = (elementId: string, value: any) => {
    const el = document.getElementById(elementId);
    if (el) {
      el.textContent = typeof value === 'number' 
        ? (value > 1000 ? value.toLocaleString() : value.toFixed(2))
        : String(value);
    }
  };

  // Helper to toggle between slider and number input
  const toggleInputMode = (sliderId: string) => {
    try {
      // Try to find the slider element (might not exist if we're in input mode)
      let slider = document.getElementById(sliderId) as HTMLInputElement | null;
      let inputGroup: HTMLElement | null = null;
      
      if (slider) {
        // Normal case: slider exists, get its parent
        inputGroup = slider.parentElement;
        if (!inputGroup) {
          console.warn('Slider exists but has no parent element');
          return;
        }
      } else {
        // Input mode case: slider was replaced, find the input group via toggle button
        const toggleBtn = document.querySelector(`[data-target="${sliderId}"]`);
        if (toggleBtn && toggleBtn.parentElement) {
          inputGroup = toggleBtn.parentElement as HTMLElement;
        } else {
          console.warn('Could not find toggle button or its parent for:', sliderId);
          return;
        }
      }
      
      const valueSpan = inputGroup.querySelector('.config-value') as HTMLElement | null;
      const toggleBtn = inputGroup.querySelector('.config-toggle') as HTMLButtonElement | null;
      
      if (!toggleBtn) {
        console.warn('Could not find toggle button in input group');
        return;
      }
      
      const isCurrentlySlider = slider && slider.type === 'range';
      
      if (isCurrentlySlider && slider) {
        // Switch to number input mode
        const currentValue = parseFloat(slider.value);
        
        // Create number input
        const numberInput = document.createElement('input');
        numberInput.type = 'number';
        numberInput.className = 'config-number-input';
        numberInput.value = String(currentValue);
        numberInput.step = '0.01';
        
        // Store slider attributes for restoration - distinguish between original and current
        numberInput.setAttribute('data-original-min', slider.min);
        numberInput.setAttribute('data-original-max', slider.max);
        numberInput.setAttribute('data-min', slider.min);
        numberInput.setAttribute('data-max', slider.max);
        numberInput.setAttribute('data-step', slider.step);
        
        // Replace slider with number input (keep same position in DOM)
        slider.replaceWith(numberInput);
        
        // Hide value display
        if (valueSpan) valueSpan.style.display = 'none';
        
        // Mark button as active
        toggleBtn.classList.add('active');
        
        // Focus the input
        numberInput.focus();
        numberInput.select();
        
        // Handle input changes
        numberInput.addEventListener('input', (e) => {
          const value = parseFloat((e.target as HTMLInputElement).value);
          if (!isNaN(value)) {
            // Find the config category and key from the slider ID
            const matches = sliderId.match(/config-(\w+)-(\w+)/);
            if (matches) {
              const category = matches[1] as keyof DemoConfig;
              const key = matches[2];
              state.configManager!.set(category, key, value);
              updatePendingIndicator();
            }
          }
        });
        
        // Handle blur to save and switch back
        numberInput.addEventListener('blur', () => {
          setTimeout(() => toggleInputMode(sliderId), 100);
        });
        
        // Handle Enter key to confirm and switch back
        numberInput.addEventListener('keydown', (e) => {
          if (e.key === 'Enter') {
            e.preventDefault();
            // Just save the value, don't toggle mode
            const finalValue = parseFloat(numberInput.value);
            if (!isNaN(finalValue)) {
              const matches = sliderId.match(/config-(\w+)-(\w+)/);
              if (matches) {
                const category = matches[1] as keyof DemoConfig;
                const key = matches[2];
                // Force save to config
                state.configManager!.set(category, key, finalValue);
                updatePendingIndicator();
              }
            }
            // Blur to lose focus and trigger any validation
            numberInput.blur();
          }
        });
      } else {
        // Switch back to slider mode
        const numberInput = inputGroup.querySelector('input[type="number"]') as HTMLInputElement | null;
        if (!numberInput) {
          console.warn('Could not find number input to switch back');
          return;
        }
        
        // Get the value that was entered
        const enteredValue = parseFloat(numberInput.value);
        if (isNaN(enteredValue)) {
          console.warn('Invalid number in input:', numberInput.value);
          return;
        }
        
        // Get the ORIGINAL default min/max from data attributes (these never change)
        const originalMin = parseFloat(numberInput.getAttribute('data-original-min') || '0');
        const originalMax = parseFloat(numberInput.getAttribute('data-original-max') || '100');
        const step = numberInput.getAttribute('data-step') || '1';
        
        // Start with the original defaults
        let min = originalMin;
        let max = originalMax;
        
        // Only expand the range if the entered value is outside the original bounds
        if (enteredValue < min) {
          min = enteredValue;
        }
        if (enteredValue > max) {
          max = enteredValue;
        }
        
        // Add some padding to the expanded range for usability
        const range = max - min;
        if (range > 0) {
          const lowerPadding = min - range * 0.1;
          const upperPadding = max + range * 0.1;
          // Only apply padding if it doesn't shrink below the original defaults
          min = Math.max(lowerPadding, originalMin);
          max = Math.max(upperPadding, originalMax); // max can only increase
        }
        
        // Create slider
        const newSlider = document.createElement('input');
        newSlider.type = 'range';
        newSlider.id = sliderId;
        newSlider.className = 'config-slider';
        
        // Set slider properties BEFORE setting the value
        newSlider.min = String(min);
        newSlider.max = String(max);
        newSlider.step = step;
        newSlider.value = String(enteredValue); // Now set the actual value
        
        // Replace number input with slider
        numberInput.replaceWith(newSlider);
        
        // Show value display
        if (valueSpan) valueSpan.style.display = 'inline';
        
        // Mark button as inactive
        toggleBtn.classList.remove('active');
        
        // Update display immediately
        const displayId = sliderId + '-value';
        updateDisplayValue(displayId, enteredValue);
        
        // Re-attach slider event listener
        newSlider.addEventListener('input', (e) => {
          const value = parseFloat((e.target as HTMLInputElement).value);
          
          // Find the config category and key
          const matches = sliderId.match(/config-(\w+)-(\w+)/);
          if (matches) {
            const category = matches[1] as keyof DemoConfig;
            const key = matches[2];
            state.configManager!.set(category, key, value);
            updateDisplayValue(displayId, value);
            updatePendingIndicator();
          }
        });
      }
    } catch (error) {
      console.error('Error in toggleInputMode:', error, 'sliderId:', sliderId);
    }
  };

  // Create input handler
  const createInputHandler = (category: keyof DemoConfig, key: string, displayId: string, isImmediate: boolean) => {
    return (e: Event) => {
      const input = e.target as HTMLInputElement;
      let value: any = input.value;
      
      // Parse numeric values
      if (input.type === 'range') {
        value = parseFloat(value);
      }

      // Update config
      state.configManager!.set(category, key, value);
      updateDisplayValue(displayId, value);

      // Apply immediate changes
      if (isImmediate) {
        if (category === 'environment' && key === 'gravity' && state.world) {
          state.world.gravity = { x: 0, y: value, z: 0 };
        }
      }

      // Update pending indicator if bridge/solver changed
      updatePendingIndicator();
    };
  };

  // Scan all config elements and attach listeners
  for (const [category, keys] of Object.entries(CONFIG_DESCRIPTORS)) {
    for (const [key, descriptor] of Object.entries(keys as any)) {
      const inputId = `config-${category}-${key}`;
      const displayId = `config-${category}-${key}-value`;
      const input = document.getElementById(inputId) as HTMLInputElement | HTMLSelectElement;

      if (!input) continue;

      const isImmediate = (descriptor as any).immediate ?? false;
      
      // Attach event listener (use 'change' for select, 'input' for others)
      const eventType = input.tagName === 'SELECT' ? 'change' : 'input';
      input.addEventListener(eventType, createInputHandler(category as keyof DemoConfig, key, displayId, isImmediate));
      
      // Initialize current value
      const currentConfig = state.configManager.getConfig();
      const currentValue = (currentConfig[category as keyof DemoConfig] as any)[key];
      
      // For select elements, set the selected value; for others, update display
      if (input.tagName === 'SELECT') {
        (input as HTMLSelectElement).value = String(currentValue);
      } else {
        // Set input value too (important for sliders)
        (input as HTMLInputElement).value = String(currentValue);
        updateDisplayValue(displayId, currentValue);
        
        // Store min/max/step on the input for later restoration
        if (input.type === 'range') {
          input.setAttribute('data-min', input.min);
          input.setAttribute('data-max', input.max);
          input.setAttribute('data-step', input.step);
        }
      }
    }
  }
  
  // Setup toggle button listeners
  document.querySelectorAll('.config-toggle').forEach((btn) => {
    btn.addEventListener('click', (e) => {
      e.preventDefault();
      const target = (btn as HTMLElement).getAttribute('data-target');
      if (target) {
        toggleInputMode(target);
      }
    });
  });
}

/**
 * Update pending changes indicator
 */
function updatePendingIndicator() {
  const indicator = document.getElementById('pending-changes-indicator');
  if (!indicator || !state.configManager) return;

  const hasPending = state.configManager.hasPendingBridgeChanges();
  if (hasPending) {
    indicator.classList.remove('hidden');
  } else {
    indicator.classList.add('hidden');
  }
}

// ---------- Scene builders ----------
function ensureCanvas() {
  let canvas = document.getElementById('bridge-canvas');
  if (!canvas) {
    canvas = document.createElement('canvas');
    canvas.id = 'bridge-canvas';
    canvas.style.width = '100vw';
    canvas.style.height = '100vh';
    document.body.style.margin = '0';
    document.body.appendChild(canvas);
  }
  return /** @type {HTMLCanvasElement} */ (canvas);
}

function initThree(canvas) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(devicePixelRatio);
  renderer.setSize(canvas.clientWidth || window.innerWidth, canvas.clientHeight || window.innerHeight, false);
  renderer.shadowMap.enabled = true;

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0a0d13);

  const camera = new THREE.PerspectiveCamera(
    60,
    (canvas.clientWidth || window.innerWidth) / (canvas.clientHeight || window.innerHeight),
    0.1,
    500
  );
  camera.position.set(0, 8, 18);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0);
  controls.enableDamping = true;

  window.addEventListener('resize', () => {
    const w = canvas.clientWidth || window.innerWidth;
    const h = canvas.clientHeight || window.innerHeight;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  });

  return { scene, camera, renderer, controls };
}

function buildLights(scene) {
  scene.add(new THREE.AmbientLight(0xffffff, 0.35));
  const dir = new THREE.DirectionalLight(0xffffff, 0.9);
  dir.position.set(10, 16, 10);
  dir.castShadow = true;
  dir.shadow.mapSize.set(2048, 2048);
  scene.add(dir);
}

function buildGround(scene, world) {
  // Define geometry and position options once
  const groundGeometry = new THREE.BoxGeometry(60, 1, 60);
  const groundMaterialOptions = { color: 0x1a1e2f, roughness: 0.85, metalness: 0.1 };
  const groundPosition = { x: 0, y: -5, z: 0 };

  // Create mesh with reusable options
  const mesh = new THREE.Mesh(
    groundGeometry,
    new THREE.MeshStandardMaterial(groundMaterialOptions)
  );
  mesh.position.set(groundPosition.x, groundPosition.y, groundPosition.z);
  mesh.receiveShadow = true;
  scene.add(mesh);

  // Create collider using reusable position
  const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  world.createCollider(
    RAPIER.ColliderDesc.cuboid(30, 0.5, 30).setTranslation(
      groundPosition.x, groundPosition.y, groundPosition.z
    ),
    body
  );

  state.colliderMetadata.set(body.handle, {
    type: 'ground',
    name: 'Ground',
    createdAt: Date.now()
  });
}

function buildBridge(scene, world) {
  // Build bridge with current configuration
  const currentConfig = state.configManager?.getConfig();
  const bridgeConfig = currentConfig?.bridge || DEFAULT_CONFIG.bridge;
  
  const scenario = buildBridgeScenario(bridgeConfig);
  state.scenario = scenario;
  
  // Create physics/solver core using proven buildBridge.headless approach
  const solverConfig = currentConfig?.solver || DEFAULT_CONFIG.solver;
  const core = createBridgeCore({ 
    runtime: state.runtime,
    world: world,
    scenario,
    gravity: currentConfig?.environment?.gravity ?? DEFAULT_CONFIG.environment.gravity,
    strengthScale: 10_000_000.0,
    solverSettings: solverConfig
  });
  
  // Materials
  const deckMat = new THREE.MeshStandardMaterial({ color: 0x4b6fe8, roughness: 0.4, metalness: 0.45 });
  const supportMat = new THREE.MeshStandardMaterial({ color: 0x2f3e56, roughness: 0.6, metalness: 0.25 });
  
  // Create Three.js meshes for each chunk
  console.log('buildBridge chunks', core.chunks);
  core.chunks.forEach((chunk) => {
    const mat = chunk.isSupport ? supportMat.clone() : deckMat.clone();
    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(chunk.size.x, chunk.size.y, chunk.size.z),
      mat
    );
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    chunk.mesh = mesh;
    chunk.baseColor = mat.color.clone();
    
    // Store baseLocalOffset as THREE.Vector3 for existing mesh sync logic
    chunk.baseLocalOffset = new THREE.Vector3(
      chunk.baseLocalOffset.x,
      chunk.baseLocalOffset.y,
      chunk.baseLocalOffset.z
    );
    chunk.localOffset = new THREE.Vector3(
      chunk.localOffset.x,
      chunk.localOffset.y,
      chunk.localOffset.z
    );
    
    // For support piers, also convert baseWorldPosition to THREE.Vector3
    if (chunk.isSupport && chunk.baseWorldPosition) {
      chunk.baseWorldPosition = new THREE.Vector3(
        chunk.baseWorldPosition.x,
        chunk.baseWorldPosition.y,
        chunk.baseWorldPosition.z
      );
    }
    
    scene.add(mesh);
  });
  
  // Register body metadata (preserve existing metadata tracking)
  state.bodyMetadata.set(core.bodyHandle, {
    type: 'bridge-root',
    name: 'Bridge-Deck-Fixed',
    actorIndex: 0,
    createdAt: Date.now()
  });
  
  core.chunks.forEach((chunk) => {
    // Support piers have separate fixed bodies
    if (chunk.isSupport && chunk.bodyHandle !== core.bodyHandle) {
      state.bodyMetadata.set(chunk.bodyHandle, {
        type: 'bridge-support',
        name: `Support-Pier-${chunk.nodeIndex}`,
        createdAt: Date.now()
      });
    }
    
    // Register collider metadata
    if (chunk.colliderHandle !== null) {
      state.colliderMetadata.set(chunk.colliderHandle, {
        type: chunk.isSupport ? 'bridge-support' : 'bridge-segment',
        name: chunk.isSupport ? `Support-${chunk.nodeIndex}` : `Segment-${chunk.nodeIndex}`,
        segmentIndex: chunk.nodeIndex,
        nodeIndex: chunk.nodeIndex,
        createdAt: Date.now()
      });
    }
  });
  
  // Copy core state to global state
  state.chunks = core.chunks;
  state.colliderToNode = core.colliderToNode;
  state.activeContactColliders = core.activeContactColliders;
  state.pendingContactForces = core.pendingContactForces;
  state.actorMap = core.actorMap;
  state.solver = core.solver;
  state.solverSettings = core.settings;
  state.rootBodyHandle = core.bodyHandle;
  
  syncChunkMeshes();
}

// ---------- Loop ----------
const physicsEnabled = true;
function loop() {
  const { world, eventQueue, renderer, scene, camera, controls, clock } = state;
  const delta = clock.getDelta();

  // SAFE FRAMES: step without events then do all queued world mutations
  if (state.safeFrames > 0) {
    try {
      if (physicsEnabled) {
        world.step();
      }
    } catch (err) {
      console.warn('Safe step failed', err);
    }
    state.safeFrames -= 1;

    applyPendingSpawns();       // create balls
    applyPendingMigrations();   // create child bodies, migrate colliders
    removeDisabledHandles();    // prune disabled colliders/bodies

    updateMeshes();
    updateStatus();
    if (state.debugRenderer) state.debugRenderer.update();
    try { if (state.debugRenderer?.enabled) renderDebugLines(); else if (state.debugHelper) updateDebugLineObject(state.debugHelper, [], false); } catch {}
    controls.update();
    renderer.render(scene, camera);
    if (state.stats) state.stats.update();
    requestAnimationFrame(loop);
    return;
  }

  // EVENTFUL STEP
  preStepSweep();

  let stepped = false;
  try {
    if (physicsEnabled) {
      world.step(eventQueue);
    }
    stepped = true;
  } catch (err) {
    console.warn('Eventful step failed, switching to safe frame', err);
    try { state.eventQueue = new RAPIER.EventQueue(true); } catch {}
    state.safeFrames = Math.max(state.safeFrames, 1);
  }

  if (!stepped) {
    updateMeshes();
    updateStatus();
    if (state.debugRenderer) state.debugRenderer.update();
    try { if (state.debugRenderer?.enabled) renderDebugLines(); else if (state.debugHelper) updateDebugLineObject(state.debugHelper, [], false); } catch {}
    controls.update();
    renderer.render(scene, camera);
    if (state.stats) state.stats.update();
    requestAnimationFrame(loop);
    return;
  }

  // Drain contacts → solver forces (root-local). Avoid world.* during drain callback.
  drainContactsAndApplyForces();

  // Gravity
  state.solver.addGravity({ x: 0, y: GRAVITY, z: 0 });

  // Solve
  state.solver.update();
  // console.log('converged?', state.solver.converged());
  // state.solver.update();
  // state.solver.update();

  /*
  // Iterate solver to convergence
  let iterations = 0;
  const maxIterations = state.solverSettings?.maxSolverIterationsPerFrame ?? 5;
  let error = { lin: 0, ang: 0 };
  
  for (; iterations < maxIterations; ++iterations) {
    state.solver.update();
    error = state.solver.stressError();
    if (state.solver.converged()) break;
    if (error.lin <= state.solverSettings?.targetError && error.ang <= state.solverSettings?.targetError) break;
  }
  state.solver.iterationCount = iterations + 1;
  state.solver.lastError = error;
  */

  // Optional: quick debug log if errors spike
  // const err = state.solver.stressError();
  // if (err.lin > 1e-3 || err.ang > 1e-3) console.debug('Stress error', err);

  // Stress-based fracture & split
  if (state.solver.overstressedBondCount() > 0) {
    // console.log('Overstressed bond count:', state.solver.overstressedBondCount());

    const perActor = state.solver.generateFractureCommandsPerActor();
    if (Array.isArray(perActor) && perActor.length > 0) {
      const splitEvents = state.solver.applyFractureCommands(perActor);
      if (Array.isArray(splitEvents) && splitEvents.length > 0) {
        queueSplitResults(splitEvents);
        state.safeFrames = Math.max(state.safeFrames, SAFE_FRAMES_AFTER_SPLIT);
      }
    }
  }

  updateMeshes();
  updateStatus();
  if (state.debugRenderer) state.debugRenderer.update();
  try { if (state.debugRenderer?.enabled) renderDebugLines(); else if (state.debugHelper) updateDebugLineObject(state.debugHelper, [], false); } catch {}
  controls.update();
  renderer.render(scene, camera);
  if (state.stats) state.stats.update();
  requestAnimationFrame(loop);
}

// ---------- Contacts → Solver ----------
function drainContactsAndApplyForces() {
  const { world, eventQueue, rootBodyHandle } = state;

  const rootBody = world.getRigidBody(rootBodyHandle);
  if (!rootBody) {
    console.error('No root body found:', rootBodyHandle);
    return;
  }

  const rq = new THREE.Quaternion(
    rootBody.rotation().x,
    rootBody.rotation().y,
    rootBody.rotation().z,
    rootBody.rotation().w
  );
  const qInv = rq.clone().invert();

  // Accumulate forces per collider handle without calling world.* inside the drain
  const acc = new Map(); // handle -> { force:THREE.Vector3, point:THREE.Vector3 }
  const add = (handle, direction, totalForce, worldPoint) => {
    // console.log('Adding force to collider:', handle, direction, totalForce, worldPoint);
    if (handle == null) return;
    
    const colliderMetadata = state.colliderMetadata.get(handle);
    if (!['bridge-segment', 'bridge-support'].includes(colliderMetadata.type)) {
      // console.warn('Skipping bridge collider:', handle, colliderMetadata.type, colliderMetadata);
      return;
    }
    // Is Bridge collider
    if (!state.colliderToNode.has(handle)) {
      console.error('No collider to node found:', handle, state.colliderToNode);
      return;
    }

    // console.log('Adding force to collider:', handle, direction, totalForce, worldPoint);

    const f = new THREE.Vector3(
      (totalForce.x ?? 0) * direction,
      (totalForce.y ?? 0) * direction,
      (totalForce.z ?? 0) * direction
    );
    const p = new THREE.Vector3(worldPoint.x ?? 0, worldPoint.y ?? 0, worldPoint.z ?? 0);

    const prev = acc.get(handle);
    if (prev) {
      prev.force.add(f);
      prev.point.copy(p);
    } else {
      acc.set(handle, { force: f, point: p });
    }
  };

  eventQueue.drainContactForceEvents((ev) => {
    try {
      // console.log('Draining contact force events:', ev);
      const tf = ev.totalForce?.();
      const mag = ev.totalForceMagnitude?.();
      if (!tf || !(mag > 0)) {
        console.log('No total force found:', tf, mag);
        return;
      }
      // console.log('Draining contact force events:', tf, mag);

      // Use the explicit contact points; if unavailable, ignore this event (avoid world.* calls here)
      // const p1 = ev.worldContactPoint?.();
      // const p2 = ev.worldContactPoint2?.();
      // if (!p1 && !p2) return;

      // Prefer explicit contact points; fall back to collider's parent body position if absent
      const wp = ev.worldContactPoint ? ev.worldContactPoint() : undefined;
      const wp2 = ev.worldContactPoint2 ? ev.worldContactPoint2() : undefined;
      
      const h1 = ev.collider1?.();
      const h2 = ev.collider2?.();

      const p1 = wp ?? wp2 ?? fallbackPoint(world, h1);
      const p2 = wp2 ?? wp ?? fallbackPoint(world, h2);

      if (h1 == null || h2 == null) {
        console.error('No collider found:', h1, h2);
        return;
      }
      // console.log('Adding force to collider:', tf, mag, p1, p2);

      // if (h1 != null)
      add(h1, +1, tf, p1 ?? p2);
      // if (h2 != null)
      add(h2, -1, tf, p2 ?? p1);
    } catch (error) {
      console.error(error);
    }
  });

  // Apply to solver (root-local)
  // if (acc.size > 0) {
  //   console.log('Applying forces to solver:', acc);
  // }
  acc.forEach((val, handle) => {
    const nodeIndex = state.colliderToNode.get(handle);
    if (nodeIndex == null) return;

    const localForce = val.force.clone().applyQuaternion(qInv);
    const localPoint = val.point.clone().applyQuaternion(qInv);

    state.solver.addForce(
      nodeIndex,
      { x: localPoint.x, y: localPoint.y, z: localPoint.z },
      { x: localForce.x, y: localForce.y, z: localForce.z },
      ExtForceMode.Force
    );
  });
}


function fallbackPoint(world, handle) {
  if (handle == null) return { x: 0, y: 0, z: 0 };
  const c = world.getCollider(handle);
  const p = c ? c.parent() : undefined;
  const b = p != null ? world.getRigidBody(p) : undefined;
  const t = b?.translation?.();
  if (t) return { x: t.x ?? 0, y: t.y ?? 0, z: t.z ?? 0 };
  return { x: 0, y: 0, z: 0 };
}

// ---------- Split queueing & migrations ----------
function queueSplitResults(splitEvents) {
  console.log('Queueing split results:', splitEvents, state);

  for (const evt of splitEvents) {
    const parentActorIndex = evt?.parentActorIndex;
    const children = Array.isArray(evt?.children) ? evt.children : [];
    const parentEntry = state.actorMap.get(parentActorIndex);
    const parentBodyHandle = parentEntry?.bodyHandle ?? state.rootBodyHandle;

    for (const child of children) {
      if (!child || !Array.isArray(child.nodes) || child.nodes.length === 0) {
        console.warn('No child nodes found:', child);
        continue;
      }

      // **Critical fix**: if the parent actor appears among children, REUSE the parent body.
      if (child.actorIndex === parentActorIndex) {
        // No body creation or migrations needed for these nodes; they remain on the parent body.
        console.warn('Reusing parent body for child:', child.actorIndex, parentBodyHandle);
        state.actorMap.set(child.actorIndex, { bodyHandle: parentBodyHandle });
        continue;
      }

      const isActorSupport = child.nodes.some(
        nodeIndex => {
          const chunk = state.chunks[nodeIndex];
          return chunk?.isSupport === true;
        }
      );

      // Schedule child body creation (inherits parent pose/vel), and migrate its nodes later.
      state.pendingBodiesToCreate.push({
        actorIndex: child.actorIndex,
        isSupport: isActorSupport,
        inheritFromBodyHandle: parentBodyHandle,
        nodes: child.nodes.slice()
      });

      // Tentative mapping; will be overwritten when body is created on a SAFE frame.
      state.actorMap.set(child.actorIndex, { bodyHandle: parentBodyHandle });
    }
  }
}

function applyPendingSpawns() {
  const { pendingBallSpawns } = state;
  if (pendingBallSpawns.length === 0) return;

  const batch = pendingBallSpawns.splice(0, pendingBallSpawns.length);
  for (const s of batch) spawnBallNow(s.x, s.z);
}

function spawnBallNow(x: number, z: number, type: 'box' | 'ball' = 'box') {
  const { world, scene } = state;
  const ballId = ++state.ballIdCounter;

  // Get current projectile configuration
  const config = state.configManager?.getConfig();
  const projConfig = config?.projectile || DEFAULT_CONFIG.projectile;
  
  const radius = projConfig.radius;
  const mass = projConfig.mass;
  const dropY = projConfig.dropY;
  const linvelY = projConfig.linvelY;
  const friction = projConfig.friction;
  const restitution = projConfig.restitution;
  const projType = projConfig.type;

  const body = world.createRigidBody(
    RAPIER.RigidBodyDesc.dynamic()
      .setTranslation(x, dropY, z)
      .setCanSleep(false)
      .setLinearDamping(0.01)
      .setAngularDamping(0.01)
      .setLinvel(0, linvelY, 0)
  );

  // Pick collider desc based on config type, then chain common settings
  let colliderDesc = projType === 'ball'
    ? RAPIER.ColliderDesc.ball(radius)
    : RAPIER.ColliderDesc.cuboid(radius, radius, radius);

  colliderDesc = colliderDesc
    .setMass(mass)
    .setFriction(friction)
    .setRestitution(restitution)
    .setActiveEvents(RAPIER.ActiveEvents.CONTACT_FORCE_EVENTS)
    .setContactForceEventThreshold(0.0);

  const collider = world.createCollider(colliderDesc, body);

  // Choose geometry based on config type
  const geometry = projType === 'ball'
    ? new THREE.SphereGeometry(radius, 20, 20)
    : new THREE.BoxGeometry(radius * 2, radius * 2, radius * 2);

  const mesh = new THREE.Mesh(
    geometry,
    new THREE.MeshStandardMaterial({ color: 0xff9147, emissive: 0x331100 })
  );

  // Register metadata, track type
  state.bodyMetadata.set(body.handle, {
    type: 'projectile',
    projectileType: projType,
    name: `Ball-${ballId}`,
    createdAt: Date.now()
  });

  state.colliderMetadata.set(collider.handle, {
    type: 'projectile',
    projectileType: projType,
    name: `Ball-${ballId}-Collider`,
    createdAt: Date.now()
  });

  mesh.castShadow = true;
  mesh.receiveShadow = true;
  mesh.position.set(x, dropY, z);
  scene.add(mesh);

  state.balls.push({ bodyHandle: body.handle, mesh });
}

function applyPendingMigrations() {
  const { world, pendingBodiesToCreate, pendingColliderMigrations } = state;
  const R = RAPIER;
  if (!world) {
    console.warn('World is not initialized');
    return;
  }

  // Capture state BEFORE any mutations for delta tracking
  captureWorldSnapshot();

  // Create child bodies & schedule collider migrations
  if (pendingBodiesToCreate.length > 0) {
    const list = pendingBodiesToCreate.splice(0, pendingBodiesToCreate.length);
    for (const pb of list) {
      const inherit = world.getRigidBody(pb.inheritFromBodyHandle);
      const desc = pb.isSupport ? R.RigidBodyDesc.fixed() : R.RigidBodyDesc.dynamic();
      // console.log('applyPendingMigrations pb', pb, inherit);
      if (inherit) {
        const pt = inherit.translation();
        const pq = inherit.rotation();
        const lv = inherit.linvel?.();
        const av = inherit.angvel?.();
        desc.setTranslation(pt.x, pt.y, pt.z)
            .setRotation(pq)
            .setLinvel(lv?.x ?? 0, lv?.y ?? 0, lv?.z ?? 0)
            // .setAngvel(av?.x ?? 0, av?.y ?? 0, av?.z ?? 0)
            ;
        // console.log('applyPendingMigrations translation', desc);
      }
      const body = world.createRigidBody(desc);
      state.actorMap.set(pb.actorIndex, { bodyHandle: body.handle });
      
      // Register new body metadata
      const bodyId = ++state.bodyIdCounter;
      const nodeNames = pb.nodes.map(n => `Seg${n}`).join(',');
      state.bodyMetadata.set(body.handle, {
        type: 'bridge-fragment',
        name: `Bridge-Actor${pb.actorIndex}-Body${bodyId} [${nodeNames}]`,
        actorIndex: pb.actorIndex,
        createdAt: Date.now()
      });

      for (const nodeIndex of pb.nodes) {
        pendingColliderMigrations.push({ nodeIndex, targetBodyHandle: body.handle });
      }
    }
  }

  // Migrate colliders to their new bodies
  if (pendingColliderMigrations.length > 0) {
    const jobs = pendingColliderMigrations.splice(0, pendingColliderMigrations.length);
    for (const mig of jobs) {
      const seg = state.chunks.find(c => c.nodeIndex === mig.nodeIndex);
      if (!seg) continue;

      // Preserve old collider metadata before removal
      const oldMetadata = seg.colliderHandle != null ? state.colliderMetadata.get(seg.colliderHandle) : null;

      // Disable old collider and queue removal
      if (seg.colliderHandle != null) {
        // console.warn('Disabling old collider:', seg.colliderHandle);
        const oldC = world.getCollider(seg.colliderHandle);
        if (oldC) oldC.setEnabled(false);
        state.activeContactColliders.delete(seg.colliderHandle);
        state.colliderToNode.delete(seg.colliderHandle);
        state.disabledCollidersToRemove.add(seg.colliderHandle);
        // Metadata will be removed in cleanup
        state.colliderMetadata.delete(seg.colliderHandle);
        seg.colliderHandle = null;
      }

      // Create replacement collider on the target body
      const halfX = seg.size.x * 0.5;
      const halfY = seg.size.y * 0.5;
      const halfZ = seg.size.z * 0.5;
      const body = world.getRigidBody(mig.targetBodyHandle);
      if (!body) continue;

      const { x: tx, y: ty, z: tz } = seg.isSupport
        ? seg.baseWorldPosition
        : seg.baseLocalOffset;

      // console.log('applyPendingMigrations collider seg', seg.isSupport, seg);
      const col = world.createCollider(
        R.ColliderDesc.cuboid(halfX, halfY, halfZ)
          // .setTranslation(seg.localOffset.x, seg.localOffset.y, seg.localOffset.z)
          // .setTranslation(seg.baseLocalOffset.x, seg.baseLocalOffset.y, seg.baseLocalOffset.z)
          .setTranslation(tx, ty, tz)
          .setActiveEvents(R.ActiveEvents.CONTACT_FORCE_EVENTS)
          .setContactForceEventThreshold(0.0)
          .setFriction(0.9)
          .setRestitution(0.0),
        body
      );

      seg.bodyHandle = body.handle;
      seg.colliderHandle = col.handle;
      seg.detached = true;

      // console.info('Setting collider to node:', col.handle, seg.nodeIndex);
      state.colliderToNode.set(col.handle, seg.nodeIndex);
      state.activeContactColliders.add(col.handle);
      
      // Restore/create metadata with stable name (preserve the segment identity)
      state.colliderMetadata.set(col.handle, {
        type: 'bridge-segment',
        name: oldMetadata?.name ?? `Segment-${seg.nodeIndex}`,
        segmentIndex: seg.nodeIndex,
        nodeIndex: seg.nodeIndex,
        createdAt: oldMetadata?.createdAt ?? Date.now()
      });
    }
  }

  // Print final state after migrations
  // printWorldHierarchy();
}

/**
 * Captures current world state for delta tracking.
 * Called before any migrations to establish a baseline for comparison.
 */
function captureWorldSnapshot() {
  const { world, bodyMetadata, colliderMetadata } = state;
  
  const bodies = new Map();
  const colliders = new Map();
  const collidersByStableId = new Map(); // Track by segment index for stable identity
  
  // Capture current state of all bodies
  world.forEachRigidBody((body) => {
    const handle = body.handle;
    const meta = bodyMetadata.get(handle);
    const bodyColliders = [];
    
    // Find all colliders attached to this body
    for (let i = 0; i < body.numColliders(); i++) {
      const cHandle = body.collider(i)?.handle;
      if (cHandle != null) bodyColliders.push(cHandle);
    }
    
    bodies.set(handle, {
      handle,
      name: meta?.name,
      actorIndex: meta?.actorIndex,
      colliders: bodyColliders
    });
  });
  
  // Capture current state of all colliders
  world.forEachCollider((col) => {
    const handle = col.handle;
    const meta = colliderMetadata.get(handle);
    const parentHandle = col.parent();
    
    const info = {
      handle,
      name: meta?.name,
      parentBodyHandle: parentHandle,
      segmentIndex: meta?.segmentIndex,
      nodeIndex: meta?.nodeIndex
    };
    
    colliders.set(handle, info);
    
    // Also index by stable ID (segment index) for tracking across handle changes
    if (meta?.segmentIndex != null) {
      collidersByStableId.set(`segment-${meta.segmentIndex}`, info);
    }
  });
  
  state.previousWorldState = { bodies, colliders, collidersByStableId };
}

/**
 * Prints complete world hierarchy with delta tracking.
 * 
 * Shows:
 * - All rigid bodies with their metadata (name, type, actor index, position, velocity)
 * - All colliders attached to each body with stable names
 * - Delta indicators: NEW (just created), MOVED (changed parent), recreated (new handle)
 * - Summary statistics
 * 
 * Example output after a bridge split:
 * 
 * 🌍 WORLD HIERARCHY AFTER MIGRATIONS
 * 
 * 📖 LEGEND:
 *   Bodies:    🏛️ = Root (fixed)  |  📦 = Dynamic  |  🔒 = Fixed
 *   Status:    ✨ NEW = Just created  |  ✅ = Existed before  |  🔄 MOVED = Changed parent
 *   Handles:   "(recreated, old handle: N)" = Collider was destroyed and recreated
 * 
 * 📦 RIGID BODIES:
 * 
 *   🏛️ ✅ Bridge-Root-Fixed (Handle: 0)
 *   ├─ Type: Fixed
 *   ├─ Actor Index: 0
 *   ├─ Translation: (0.00, 0.50, 0.00)
 *   └─ Colliders: 20 total (0 added, 20 retained)
 *      ├─ ✅ Segment-0 (Handle: 1)
 *      │  ├─ Segment Index: 0
 *      │  ├─ Node Index: 0
 *      │  └─ Parent Body: Bridge-Root-Fixed
 *      ...
 * 
 *   📦 ✨ NEW Bridge-Actor1-Body1 [Seg25,Seg26,...,Seg31] (Handle: 5)
 *   ├─ Type: Dynamic
 *   ├─ Actor Index: 1
 *   ├─ Translation: (8.53, 0.48, 0.00)
 *   ├─ Linear Velocity: (0.12, -0.03, 0.00)
 *   ├─ Angular Velocity: (0.00, 0.00, 0.01)
 *   └─ Colliders: 7 total (7 added, 0 retained)
 *      ├─ 🔄 MOVED from Bridge-Root-Fixed Segment-25 (Handle: 39, recreated, old handle: 26)
 *      │  ├─ Segment Index: 25
 *      │  ├─ Node Index: 25
 *      │  └─ Parent Body: Bridge-Actor1-Body1 [Seg25,Seg26,...,Seg31]
 *      ...
 * 
 * 📊 SUMMARY:
 * ├─ Total Actors: 2
 * ├─ Rigid Bodies: 2 (1 new)
 * ├─ Colliders: 32
 * ├─   New Colliders: 0
 * ├─   Moved Colliders: 7
 * └─ Removed Bodies: 0
 */
function printWorldHierarchy() {
  const { world, actorMap, bodyMetadata, colliderMetadata, previousWorldState } = state;
  
  console.group('🌍 WORLD HIERARCHY AFTER MIGRATIONS');
  
  // Print legend
  console.log('\n📖 LEGEND:');
  console.log('  Bodies:    🏛️ = Root (fixed)  |  📦 = Dynamic  |  🔒 = Fixed');
  console.log('  Status:    ✨ NEW = Just created  |  ✅ = Existed before  |  🔄 MOVED = Changed parent');
  console.log('  Handles:   "(recreated, old handle: N)" = Collider was destroyed and recreated');
  console.log('');
  
  // Build current state maps
  const currentBodies = new Map();
  const currentColliders = new Map();
  const bodyToColliders = new Map();
  
  // Scan current world state
  world.forEachRigidBody((body) => {
    const handle = body.handle;
    const meta = bodyMetadata.get(handle);
    const wasPresent = previousWorldState.bodies.has(handle);
    
    currentBodies.set(handle, {
      handle,
      name: meta?.name ?? `Body-${handle}`,
      actorIndex: meta?.actorIndex,
      isNew: !wasPresent,
      body
    });
    
    bodyToColliders.set(handle, []);
  });
  
  world.forEachCollider((col) => {
    const handle = col.handle;
    const meta = colliderMetadata.get(handle);
    const parentHandle = col.parent();
    
    // Check by handle first (for balls and other non-segment colliders)
    let prevState = previousWorldState.colliders.get(handle);
    
    // For segment colliders, also check by stable ID to track across handle changes
    if (!prevState && meta?.segmentIndex != null) {
      prevState = previousWorldState.collidersByStableId?.get(`segment-${meta.segmentIndex}`);
    }
    
    const wasPresent = prevState != null;
    const wasMoved = wasPresent && prevState.parentBodyHandle !== parentHandle;
    const isNew = !wasPresent;
    
    const colliderInfo = {
      handle,
      name: meta?.name ?? `Collider-${handle}`,
      parentBodyHandle: parentHandle,
      segmentIndex: meta?.segmentIndex,
      nodeIndex: meta?.nodeIndex,
      isNew,
      wasMoved,
      prevParentHandle: wasMoved ? prevState.parentBodyHandle : null,
      prevParentName: null, // will fill below
      oldHandle: prevState?.handle !== handle ? prevState?.handle : null
    };
    
    currentColliders.set(handle, colliderInfo);
    
    if (parentHandle != null) {
      if (!bodyToColliders.has(parentHandle)) {
        bodyToColliders.set(parentHandle, []);
      }
      bodyToColliders.get(parentHandle).push(colliderInfo);
    }
  });
  
  // Fill in previous parent names for moved colliders
  currentColliders.forEach((cInfo) => {
    if (cInfo.wasMoved && cInfo.prevParentHandle != null) {
      const prevBody = previousWorldState.bodies.get(cInfo.prevParentHandle);
      cInfo.prevParentName = prevBody?.name ?? `Body-${cInfo.prevParentHandle}`;
    }
  });
  
  // Print each body with delta indicators
  console.log('\n📦 RIGID BODIES:\n');
  
  currentBodies.forEach((bInfo) => {
    const { handle, name, actorIndex, isNew, body } = bInfo;
    const isRoot = handle === state.rootBodyHandle;
    const isDynamic = body.isDynamic();
    const isFixed = body.isFixed();
    const t = body.translation();
    
    const statusIcon = isNew ? '✨ NEW' : '✅';
    const bodyIcon = isRoot ? '🏛️' : (isDynamic ? '📦' : '🔒');
    
    console.group(`${bodyIcon} ${statusIcon} ${name} (Handle: ${handle})`);
    console.log('├─ Type:', isDynamic ? 'Dynamic' : (isFixed ? 'Fixed' : 'Other'));
    console.log('├─ Actor Index:', actorIndex ?? 'N/A');
    console.log('├─ Translation:', `(${t.x.toFixed(2)}, ${t.y.toFixed(2)}, ${t.z.toFixed(2)})`);
    
    if (isDynamic) {
      const lv = body.linvel();
      const av = body.angvel();
      console.log('├─ Linear Velocity:', `(${lv.x.toFixed(2)}, ${lv.y.toFixed(2)}, ${lv.z.toFixed(2)})`);
      console.log('├─ Angular Velocity:', `(${av.x.toFixed(2)}, ${av.y.toFixed(2)}, ${av.z.toFixed(2)})`);
    }
    
    // Show colliders attached to this body
    const colliders = bodyToColliders.get(handle) || [];
    
    // Determine what changed with colliders
    const prevBodyState = previousWorldState.bodies.get(handle);
    const prevColliders = new Set(prevBodyState?.colliders ?? []);
    const currColliders = new Set(colliders.map(c => c.handle));
    
    const addedColliders = colliders.filter(c => c.isNew || c.wasMoved);
    const retainedColliders = colliders.filter(c => !c.isNew && !c.wasMoved);
    const removedCount = prevBodyState ? prevColliders.size - retainedColliders.length : 0;
    
    console.log(`└─ Colliders: ${colliders.length} total (${addedColliders.length} added, ${retainedColliders.length} retained${removedCount > 0 ? `, ${removedCount} removed` : ''})`);
    
    if (colliders.length > 0) {
      colliders.forEach((cInfo, idx) => {
        const isLast = idx === colliders.length - 1;
        const prefix = isLast ? '   └─' : '   ├─';
        
        let status = '';
        if (cInfo.isNew) status = '✨ NEW';
        else if (cInfo.wasMoved) status = `🔄 MOVED from ${cInfo.prevParentName}`;
        else status = '✅';
        
        const handleInfo = cInfo.oldHandle ? ` (recreated, old handle: ${cInfo.oldHandle})` : '';
        
        console.group(`${prefix} ${status} ${cInfo.name} (Handle: ${cInfo.handle}${handleInfo})`);
        if (cInfo.segmentIndex != null) console.log('├─ Segment Index:', cInfo.segmentIndex);
        if (cInfo.nodeIndex != null) console.log('├─ Node Index:', cInfo.nodeIndex);
        console.log('└─ Parent Body:', name);
        console.groupEnd();
      });
    }
    
    console.groupEnd();
  });
  
  // Show removed bodies
  const removedBodies = [];
  previousWorldState.bodies.forEach((prevBody, handle) => {
    if (!currentBodies.has(handle)) {
      removedBodies.push(prevBody);
    }
  });
  
  if (removedBodies.length > 0) {
    console.log('\n🗑️  REMOVED BODIES:\n');
    removedBodies.forEach((bInfo) => {
      console.log(`   ❌ ${bInfo.name} (Handle: ${bInfo.handle}) with ${bInfo.colliders.length} colliders`);
    });
  }
  
  // Summary
  console.log('\n📊 SUMMARY:');
  console.log('├─ Total Actors:', actorMap.size);
  console.log('├─ Rigid Bodies:', currentBodies.size, `(${Array.from(currentBodies.values()).filter(b => b.isNew).length} new)`);
  console.log('├─ Colliders:', currentColliders.size);
  
  const newColliders = Array.from(currentColliders.values()).filter(c => c.isNew).length;
  const movedColliders = Array.from(currentColliders.values()).filter(c => c.wasMoved).length;
  console.log('├─   New Colliders:', newColliders);
  console.log('├─   Moved Colliders:', movedColliders);
  console.log('└─ Removed Bodies:', removedBodies.length);
  
  console.groupEnd();
}

function removeDisabledHandles() {
  const { world, disabledCollidersToRemove, bodiesToRemove, colliderMetadata, bodyMetadata } = state;
  // console.log('Removing disabled handles', { disabledCollidersToRemove, bodiesToRemove });
  // return

  for (const h of Array.from(disabledCollidersToRemove)) {
    const c = world.getCollider(h);
    if (c) world.removeCollider(c, false);
    colliderMetadata.delete(h); // Clean up metadata
    disabledCollidersToRemove.delete(h);
  }

  for (const bh of Array.from(bodiesToRemove)) {
    const b = world.getRigidBody(bh);
    if (b) world.removeRigidBody(b);
    bodyMetadata.delete(bh); // Clean up metadata
    bodiesToRemove.delete(bh);
  }
}

// ---------- Safety sweeps ----------
function preStepSweep() {
  const { world, activeContactColliders, colliderToNode, disabledCollidersToRemove } = state;

  // Drop any colliders that no longer exist, are disabled, or lost their parent.
  for (const h of Array.from(activeContactColliders)) {
    const c = world.getCollider(h);
    const p = c ? c.parent() : undefined;
    const b = p != null ? world.getRigidBody(p) : undefined;
    const enabled = c?.isEnabled ? c.isEnabled() : true;
    if (!c || !enabled || !b) {
      console.warn('Deleting active contact collider:', h);
      activeContactColliders.delete(h);
      colliderToNode.delete(h);
    }
  }

  // Mirror sweep on the map (paranoid redundancy)
  for (const [h] of Array.from(colliderToNode.entries())) {
    const c = world.getCollider(h);
    const p = c ? c.parent() : undefined;
    const b = p != null ? world.getRigidBody(p) : undefined;
    const enabled = c?.isEnabled ? c.isEnabled() : true;
    if (!c || !enabled || !b) {
      console.warn('Deleting collider to node:', h);
      colliderToNode.delete(h);
      activeContactColliders.delete(h);
    }
  }

  // Remove any disabled colliders queued by prior migrations
  for (const h of Array.from(disabledCollidersToRemove)) {
    const c = world.getCollider(h);
    if (c) world.removeCollider(c, false);
    disabledCollidersToRemove.delete(h);
  }
}

// ---------- Rendering ----------
function updateMeshes() {
  syncChunkMeshes();
  syncBallMeshes();
}

function updateStatus() {
  const attachedCount = state.chunks.filter(c => !c.detached).length;
  const detachedCount = state.chunks.filter(c => c.detached).length;
  const ballCount = state.balls.length;
  const rigidbodyCount = state.world.bodies.len();
  const solverActorCount = state.solver ? state.solver.actorCount() : 0;
  const solverNodeCount = state.solver ? state.solver.graphNodeCount() : 0;
  const overstressed = state.solver ? state.solver.overstressedBondCount() : 0;
  const stressErr = state.solver ? state.solver.stressError() : { lin: 0, ang: 0 };
  const converged = state.solver ? state.solver.converged() : false;

  const segmentCountEl = document.getElementById('segment-count');
  const detachedCountEl = document.getElementById('detached-count');
  const ballCountEl = document.getElementById('ball-count');
  const rigidbodyCountEl = document.getElementById('rigidbody-count');
  const solverActorsEl = document.getElementById('solver-actors');
  const solverNodesEl = document.getElementById('solver-nodes');
  const solverOverstressedEl = document.getElementById('solver-overstressed');
  const solverBondsEl = document.getElementById('solver-bonds');
  const solverErrLinEl = document.getElementById('solver-error-lin');
  const solverErrAngEl = document.getElementById('solver-error-ang');
  const solverConvergedEl = document.getElementById('solver-converged');

  if (segmentCountEl) segmentCountEl.textContent = String(attachedCount);
  if (detachedCountEl) detachedCountEl.textContent = String(detachedCount);
  if (ballCountEl) ballCountEl.textContent = String(ballCount);
  if (rigidbodyCountEl) rigidbodyCountEl.textContent = String(rigidbodyCount);
  if (solverActorsEl) solverActorsEl.textContent = String(solverActorCount);
  if (solverNodesEl) solverNodesEl.textContent = String(solverNodeCount);
  if (solverOverstressedEl) solverOverstressedEl.textContent = String(overstressed);
  if (solverBondsEl) solverBondsEl.textContent = String(state.solver ? state.solver.bondCapacity() : 0);
  if (solverErrLinEl) solverErrLinEl.textContent = stressErr.lin.toFixed(5);
  if (solverErrAngEl) solverErrAngEl.textContent = stressErr.ang.toFixed(5);
  if (solverConvergedEl) solverConvergedEl.textContent = converged ? '(converged)' : '';
}

function syncChunkMeshes() {
  const { world, chunks } = state;
  const tmp = new THREE.Vector3();
  const quat = new THREE.Quaternion();

  for (const chunk of chunks) {
    if (!chunk.mesh) continue;
    
    const body = world.getRigidBody(chunk.bodyHandle);
    if (!body) continue;

    const t = body.translation();
    const r = body.rotation();

    // Set mesh to body position
    chunk.mesh.position.set(t.x, t.y, t.z);
    quat.set(r.x, r.y, r.z, r.w);
    chunk.mesh.quaternion.copy(quat);

    // Apply local offset (bridge-local -> world)
    // For supports, localOffset is (0,0,0) so this has no effect
    // For deck chunks, localOffset positions them relative to root body
    // tmp.copy(chunk.localOffset).applyQuaternion(chunk.mesh.quaternion);
    tmp.copy(chunk.baseLocalOffset).applyQuaternion(chunk.mesh.quaternion);
    chunk.mesh.position.add(tmp);
  }
}

function syncBallMeshes() {
  const { world, balls, scene, bodyMetadata, colliderMetadata } = state;
  const keep = [];

  for (const b of balls) {
    const body = world.getRigidBody(b.bodyHandle);
    if (!body) continue;

    const t = body.translation();
    const q = body.rotation();
    b.mesh.position.set(t.x, t.y, t.z);
    b.mesh.quaternion.set(q.x, q.y, q.z, q.w);

    if (t.y < -50) {
      scene.remove(b.mesh);
      state.bodiesToRemove.add(b.bodyHandle);
      
      // Clean up metadata for the ball body and its colliders
      bodyMetadata.delete(b.bodyHandle);
      for (let i = 0; i < body.numColliders(); i++) {
        const cHandle = body.collider(i)?.handle;
        if (cHandle != null) colliderMetadata.delete(cHandle);
      }
    } else {
      keep.push(b);
    }
  }
  state.balls = keep;
}

// ---------- Solver Debug Helpers ----------
function debugPrintSolver() {
  const s = state.solver;
  if (!s) {
    console.log('Solver not ready');
    return;
  }

  const actors = s.actors();
  const nodes = s.graphNodeCount();
  const bonds = s.bondCapacity();
  const overstressed = s.overstressedBondCount();
  const err = s.stressError();
  const converged = s.converged();
  const settings = state.solverSettings;
  const debugLines = s.fillDebugRender({ mode: ExtDebugMode.Max, scale: 1.0 });

  console.group('🧮 ExtStressSolver Snapshot');
  console.log('Actors:', actors.length);
  actors.forEach((a) => console.log(`  - Actor ${a.actorIndex}: nodes [${(a.nodes || []).join(', ')}]`));
  console.log('Graph Nodes:', nodes);
  console.log('Bond Capacity:', bonds);
  console.log('Overstressed Bonds:', overstressed);
  console.log('Stress Error:', `lin=${err.lin.toFixed(6)} ang=${err.ang.toFixed(6)}`, converged ? '(converged)' : '');
  if (settings) console.log('Settings:', settings);
  console.log('Debug Lines (for overlay):', debugLines?.length ?? 0);

  // Planned fractures (without applying) — per-actor preview
  try {
    const planned = s.generateFractureCommandsPerActor();
    if (planned && planned.length) {
      console.log('Planned fractures (by actor):');
      planned.forEach((p) => {
        const total = p.fractures?.length ?? 0;
        const sample = (p.fractures || []).slice(0, 5).map((f) => ({ n0: f.nodeIndex0, n1: f.nodeIndex1, health: f.health?.toFixed?.(4) ?? f.health }));
        console.log(`  • Actor ${p.actorIndex}: ${total} bond(s)`, sample.length ? { sample } : '');
      });
    }
  } catch {}

  // Top stressed bonds (derived from debug colors)
  try {
    const top = computeTopStressedBonds(10);
    if (top.length) {
      console.log('Top stressed bonds (percent of limits):');
      console.log('  idx |   max%    C%     T%     S%   | C=Compression  T=Tension  S=Shear');
      const fmt = (p:number) => (p*100).toFixed(1).padStart(6);
      top.forEach((t, i) => {
        const idx = String(i + 1).padStart(2, '0');
        console.log(`  ${idx}  | ${fmt(t.maxPct)}  ${fmt(t.cPct)}  ${fmt(t.tPct)}  ${fmt(t.sPct)}  |`);
      });
    }
  } catch {}
  console.groupEnd();
}

// ---------- Stress Debug Line Rendering ----------
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

function updateDebugLineObject(debugHelper, debugLines, enabled) {
  if (!enabled || !debugLines || debugLines.length === 0) {
    debugHelper.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
    debugHelper.geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(0), 3));
    debugHelper.object.visible = false;
    return;
  }

  const positions = new Float32Array(debugLines.length * 2 * 3);
  const colors = new Float32Array(debugLines.length * 2 * 3);

  const colorFromUint24 = (u24) => ({
    r: ((u24 >> 16) & 0xff) / 255,
    g: ((u24 >> 8) & 0xff) / 255,
    b: (u24 & 0xff) / 255
  });

  debugLines.forEach((line, index) => {
    const base = index * 6;
    positions[base] = line.p0.x;
    positions[base + 1] = line.p0.y;
    positions[base + 2] = line.p0.z;
    positions[base + 3] = line.p1.x;
    positions[base + 4] = line.p1.y;
    positions[base + 5] = line.p1.z;

    const c0 = colorFromUint24(line.color0);
    const c1 = colorFromUint24(line.color1 ?? line.color0);
    colors[base] = c0.r; colors[base + 1] = c0.g; colors[base + 2] = c0.b;
    colors[base + 3] = c1.r; colors[base + 4] = c1.g; colors[base + 5] = c1.b;
  });

  debugHelper.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  debugHelper.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  debugHelper.geometry.computeBoundingSphere();
  debugHelper.object.visible = true;
}

function renderDebugLines() {
  const solver = state.solver;
  const world = state.world;
  const bodyHandle = state.rootBodyHandle;
  const helper = state.debugHelper;
  if (!solver || !world || !helper) return;

  // Lines are in bridge-local space; fetch them
  const debugLines = solver.fillDebugRender({ mode: ExtDebugMode.Max, scale: 1.0 }) || [];
  helper.currentLines = debugLines;

  if (!state.debugRenderer?.enabled) {
    updateDebugLineObject(helper, [], false);
    return;
  }

  const body = world.getRigidBody(bodyHandle);
  if (!body) {
    updateDebugLineObject(helper, [], false);
    return;
  }

  const tr = body.translation();
  const rot = body.rotation();
  const q = new THREE.Quaternion(rot.x, rot.y, rot.z, rot.w);
  const t = new THREE.Vector3(tr.x, tr.y, tr.z);

  // Transform to world-space for rendering
  const worldLines = debugLines.map((line) => {
    const p0 = new THREE.Vector3(line.p0.x, line.p0.y, line.p0.z).applyQuaternion(q).add(t);
    const p1 = new THREE.Vector3(line.p1.x, line.p1.y, line.p1.z).applyQuaternion(q).add(t);
    return { p0: { x: p0.x, y: p0.y, z: p0.z }, p1: { x: p1.x, y: p1.y, z: p1.z }, color0: line.color0, color1: line.color1 };
  });

  updateDebugLineObject(helper, worldLines, true);
}

// Compute stress percentages for compression/tension/shear by decoding debug colors.
// Note: This mirrors the C++ hue mapping used in NvBlastExtStressSolver (bondHealthColor).
function computeTopStressedBonds(limit = 10) {
  const s = state.solver;
  if (!s) return [];

  // Per-mode debug lines; arrays are expected to align by solver bond order
  const linesMax = s.fillDebugRender({ mode: ExtDebugMode.Max, scale: 1.0 }) || [];
  const linesC = s.fillDebugRender({ mode: ExtDebugMode.Compression, scale: 1.0 }) || [];
  const linesT = s.fillDebugRender({ mode: ExtDebugMode.Tension, scale: 1.0 }) || [];
  const linesS = s.fillDebugRender({ mode: ExtDebugMode.Shear, scale: 1.0 }) || [];

  const N = Math.min(linesMax.length, linesC.length, linesT.length, linesS.length);
  const entries = [];
  for (let i = 0; i < N; i++) {
    const cPct = stressPctFromColor(linesC[i].color0);
    const tPct = stressPctFromColor(linesT[i].color0);
    const sPct = stressPctFromColor(linesS[i].color0);
    const mPct = Math.max(cPct, tPct, sPct);
    if (mPct >= 0) {
      entries.push({ idx: i, maxPct: mPct, cPct, tPct, sPct });
    }
  }

  entries.sort((a, b) => b.maxPct - a.maxPct);
  return entries.slice(0, limit);
}

function stressPctFromColor(color: number): number {
  // Detect unbreakable color (approx 0xFF00AEFF) — treat as 0
  if ((color & 0x00FFFFFF) === 0x00AEFF) return 0.0;

  const r = (color >> 16) & 0xFF;
  const g = (color >> 8) & 0xFF;
  const b = color & 0xFF;
  const { h, s, v } = rgbToHsv(r / 255, g / 255, b / 255);
  if (s < 0.5 || v < 0.5) return -1; // unexpected

  const GREEN = 1.0 / 3.0;
  const RED = 0.0;
  const BLUE = 2.0 / 3.0;
  const MAGENTA = 5.0 / 6.0;

  // Map hue back to stressPct per solver mapping
  // 0..0.5: GREEN -> RED, 0.5..1.0: BLUE -> MAGENTA
  if (h <= GREEN + 1e-3 && h >= RED - 1e-3) {
    const t = (GREEN - h) / (GREEN - RED);
    return 0.5 * clamp01(t);
  }
  if (h >= BLUE - 1e-3 && h <= MAGENTA + 1e-3) {
    const t = (h - BLUE) / (MAGENTA - BLUE);
    return 0.5 + 0.5 * clamp01(t);
  }
  return -1;
}

function rgbToHsv(r: number, g: number, b: number) {
  const max = Math.max(r, g, b), min = Math.min(r, g, b);
  const d = max - min;
  let h = 0;
  if (d !== 0) {
    switch (max) {
      case r: h = ((g - b) / d + (g < b ? 6 : 0)); break;
      case g: h = ((b - r) / d + 2); break;
      case b: h = ((r - g) / d + 4); break;
    }
    h /= 6;
  }
  const s = max === 0 ? 0 : d / max;
  const v = max;
  return { h, s, v };
}

function clamp01(x: number) { return x < 0 ? 0 : (x > 1 ? 1 : x); }
