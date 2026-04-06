
/**
 * Configuration system for bridge stress demo.
 * Tracks immediate changes (projectile, environment) and deferred changes (bridge, solver).
 */

export interface ProjectileConfig {
  radius: number;
  mass: number;
  dropY: number;
  linvelY: number;
  type: 'ball' | 'box';
  friction: number;
  restitution: number;
}

export interface BridgeConfig {
  span: number;
  deckWidth: number;
  deckThickness: number;
  spanSegments: number;
  widthSegments: number;
  thicknessLayers: number;
  deckMass: number;
  pierHeight: number;
  areaScale: number;
  supportAreaFactor: number;
  addDiagonals: number; // 0/1
  diagScale: number;
  bondJitter: number;
  normalizeAreas: number; // 0/1
}

export interface SolverConfig {
  materialScale: number;
  maxSolverIterationsPerFrame: number;
  compressionElasticLimit: number;
  compressionFatalLimit: number;
  tensionElasticLimit: number;
  tensionFatalLimit: number;
  shearElasticLimit: number;
  shearFatalLimit: number;
  graphReductionLevel: number;
}

export interface EnvironmentConfig {
  gravity: number;
}

export interface DemoConfig {
  projectile: ProjectileConfig;
  bridge: BridgeConfig;
  solver: SolverConfig;
  environment: EnvironmentConfig;
}

// Default configuration
export const DEFAULT_CONFIG: DemoConfig = {
  projectile: {
    radius: 0.35,
    mass: 15_000.0,
    dropY: 8,
    linvelY: -10,
    type: 'ball',
    friction: 0.6,
    restitution: 0.2
  },
  bridge: {
    span: 20.0,
    deckWidth: 8.0,
    deckThickness: 0.6,
    spanSegments: 15,
    widthSegments: 5,
    thicknessLayers: 2,
    deckMass: 60_000.0,
    pierHeight: 3.0,
    areaScale: 0.05,  // 0.01–0.03 is a nice interactive range
    supportAreaFactor: 2.5,
    addDiagonals: 1,
    diagScale: 0.7,
    bondJitter: 0.12,
    normalizeAreas: 1
  },
  solver: {
    materialScale: 1.0,
    maxSolverIterationsPerFrame: 64,
    // Base “concrete-ish” shape (unitless). Ratios: C≈10×T, S≈3–4×T, fatal≈3×elastic
    compressionElasticLimit: 0.009,
    compressionFatalLimit:  0.027,
    tensionElasticLimit:    0.0009,
    tensionFatalLimit:      0.0027,
    shearElasticLimit:      0.0012,
    shearFatalLimit:        0.0036,
    graphReductionLevel: 0
  },
  environment: {
    gravity: -9.81
  }
};

// UI descriptors for each parameter
export const CONFIG_DESCRIPTORS: Record<string, Record<string, any>> = {
  projectile: {
    radius: {
      label: 'Ball Radius',
      min: 0.1,
      max: 1.0,
      step: 0.05,
      unit: 'm',
      immediate: true
    },
    mass: {
      label: 'Ball Mass',
      min: 1_000,
      max: 50_000,
      step: 1_000,
      unit: 'kg',
      immediate: true
    },
    dropY: {
      label: 'Drop Height',
      min: 2,
      max: 15,
      step: 0.5,
      unit: 'm',
      immediate: true
    },
    linvelY: {
      label: 'Initial Velocity (Y)',
      min: -30,
      max: -1,
      step: 1,
      unit: 'm/s',
      immediate: true
    },
    friction: {
      label: 'Friction',
      min: 0,
      max: 1,
      step: 0.1,
      unit: '',
      immediate: true
    },
    restitution: {
      label: 'Restitution',
      min: 0,
      max: 1,
      step: 0.05,
      unit: '',
      immediate: true
    },
    type: {
      label: 'Projectile Type',
      options: ['ball', 'box'],
      immediate: true
    }
  },
  bridge: {
    span: {
      label: 'Deck Span',
      min: 10,
      max: 40,
      step: 2,
      unit: 'm',
      immediate: false
    },
    deckWidth: {
      label: 'Deck Width',
      min: 4,
      max: 16,
      step: 0.5,
      unit: 'm',
      immediate: false
    },
    deckThickness: {
      label: 'Deck Thickness',
      min: 0.2,
      max: 2,
      step: 0.1,
      unit: 'm',
      immediate: false
    },
    spanSegments: {
      label: 'Span Segments',
      min: 5,
      max: 30,
      step: 1,
      unit: '',
      immediate: false
    },
    widthSegments: {
      label: 'Width Segments',
      min: 2,
      max: 10,
      step: 1,
      unit: '',
      immediate: false
    },
    thicknessLayers: {
      label: 'Thickness Layers',
      min: 1,
      max: 5,
      step: 1,
      unit: '',
      immediate: false
    },
    deckMass: {
      label: 'Deck Mass',
      min: 10_000,
      max: 200_000,
      step: 5_000,
      unit: 'kg',
      immediate: false
    },
    pierHeight: {
      label: 'Pier Height',
      min: 1,
      max: 8,
      step: 0.5,
      unit: 'm',
      immediate: false
    },
    areaScale: {
      label: 'Bond Area Scale',
      min: 0.01,
      max: 0.5,
      step: 0.02,
      unit: '',
      immediate: false
    },
    supportAreaFactor: {
      label: 'Support Area Factor',
      min: 1.0,
      max: 4.0,
      step: 0.1,
      unit: '×',
      immediate: false
    },
    addDiagonals: {
      label: 'Add Diagonals',
      min: 0,
      max: 1,
      step: 1,
      unit: '',
      immediate: false
    },
    diagScale: {
      label: 'Diagonal Strength',
      min: 0.4,
      max: 1.0,
      step: 0.05,
      unit: '',
      immediate: false
    },
    bondJitter: {
      label: 'Bond Jitter',
      min: 0.0,
      max: 0.25,
      step: 0.01,
      unit: '',
      immediate: false
    },
    normalizeAreas: {
      label: 'Normalize Areas',
      min: 0,
      max: 1,
      step: 1,
      unit: '',
      immediate: false
    }
  },
  solver: {
    materialScale: {
      label: 'Material Scale',
      min: 0.05,
      max: 5,
      step: 0.05,
      unit: '×',
      immediate: false
    },
    maxSolverIterationsPerFrame: {
      label: 'Max Solver Iterations',
      min: 1,
      max: 256,
      step: 1,
      unit: '',
      immediate: false
    },
    compressionElasticLimit: {
      label: 'Compression Elastic Limit',
      min: 0.001,
      max: 0.1,
      step: 0.001,
      unit: '',
      immediate: false,
      exponent: 1
    },
    compressionFatalLimit: {
      label: 'Compression Fatal Limit',
      min: 0.005,
      max: 0.5,
      step: 0.005,
      unit: '',
      immediate: false,
      exponent: 1
    },
    tensionElasticLimit: {
      label: 'Tension Elastic Limit',
      min: 0.0001,
      max: 0.01,
      step: 0.0001,
      unit: '',
      immediate: false,
      exponent: 1
    },
    tensionFatalLimit: {
      label: 'Tension Fatal Limit',
      min: 0.0005,
      max: 0.05,
      step: 0.0005,
      unit: '',
      immediate: false,
      exponent: 1
    },
    shearElasticLimit: {
      label: 'Shear Elastic Limit',
      min: 0.0001,
      max: 0.01,
      step: 0.0001,
      unit: '',
      immediate: false,
      exponent: 1
    },
    shearFatalLimit: {
      label: 'Shear Fatal Limit',
      min: 0.0005,
      max: 0.05,
      step: 0.0005,
      unit: '',
      immediate: false,
      exponent: 1
    },
    graphReductionLevel: {
      label: 'Graph Reduction Level',
      min: 0,
      max: 3,
      step: 1,
      unit: '',
      immediate: false
    }
  },
  environment: {
    gravity: {
      label: 'Gravity',
      min: -30,
      max: -0.5,
      step: 0.5,
      unit: 'm/s²',
      immediate: true
    }
  }
};

/**
 * Configuration manager: track current config and changes
 */
export class ConfigManager {
  private current: DemoConfig;
  private baselineForReset: DemoConfig;
  private changeTracker: Set<string> = new Set();

  constructor(initial: DemoConfig = DEFAULT_CONFIG) {
    this.current = JSON.parse(JSON.stringify(initial));
    this.baselineForReset = JSON.parse(JSON.stringify(initial));
  }

  /**
   * Get current configuration (deep copy)
   */
  getConfig(): DemoConfig {
    return JSON.parse(JSON.stringify(this.current));
  }

  /**
   * Update a single value; track if it requires reset
   */
  set(category: keyof DemoConfig, key: string, value: any): boolean {
    const oldValue = (this.current[category] as any)[key];
    if (oldValue === value) return false;

    (this.current[category] as any)[key] = value;

    // Track changes for non-immediate settings
    const descriptor = CONFIG_DESCRIPTORS[category]?.[key];
    if (descriptor && !descriptor.immediate) {
      this.changeTracker.add(`${category}.${key}`);
    }

    return true;
  }

  /**
   * Check if bridge config has pending changes requiring reset
   */
  hasPendingBridgeChanges(): boolean {
    for (const change of this.changeTracker) {
      if (change.startsWith('bridge.') || change.startsWith('solver.')) {
        return true;
      }
    }
    return false;
  }

  /**
   * Reset bridge to current config and clear tracking
   */
  confirmBridgeReset(): void {
    this.changeTracker.clear();
  }

  /**
   * Get list of pending changes (for debugging)
   */
  getPendingChanges(): string[] {
    return Array.from(this.changeTracker);
  }

  /**
   * Revert all changes
   */
  revert(): void {
    this.current = JSON.parse(JSON.stringify(this.baselineForReset));
    this.changeTracker.clear();
  }

  /**
   * Save current config as new baseline (for after reset)
   */
  updateBaseline(): void {
    this.baselineForReset = JSON.parse(JSON.stringify(this.current));
    this.changeTracker.clear();
  }

  /**
   * Export config for persistence
   */
  exportForStorage(): DemoConfig {
    return JSON.parse(JSON.stringify(this.current));
  }
}

/**
 * Save configuration to localStorage for persistence across page reloads
 */
export function saveConfigToStorage(config: DemoConfig): void {
  try {
    localStorage.setItem('bridgeStressConfig', JSON.stringify(config));
  } catch (e) {
    console.warn('Failed to save config to localStorage:', e);
  }
}

/**
 * Load configuration from localStorage if available
 */
export function loadConfigFromStorage(): DemoConfig | null {
  try {
    const saved = localStorage.getItem('bridgeStressConfig');
    if (saved) {
      return JSON.parse(saved) as DemoConfig;
    }
  } catch (e) {
    console.warn('Failed to load config from localStorage:', e);
  }
  return null;
}

/**
 * Clear saved configuration from localStorage
 */
export function clearConfigFromStorage(): void {
  try {
    localStorage.removeItem('bridgeStressConfig');
  } catch (e) {
    console.warn('Failed to clear config from localStorage:', e);
  }
}
