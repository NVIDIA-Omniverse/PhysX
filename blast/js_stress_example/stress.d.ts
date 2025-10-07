/**
 * 3D vector structure used for centroids, forces, and torques (units depend on caller).
 */
export interface Vec3 {
  /** X component of the vector. */
  x: number;
  /** Y component of the vector. */
  y: number;
  /** Z component of the vector. */
  z: number;
}

/**
 * Stress magnitudes resolved into compression, tension, and shear components (Pascals).
 */
export interface BondStress {
  /** Compression pressure acting along the bond normal (positive = Pa). */
  compression: number;
  /** Tension pressure acting opposite the bond normal (positive = Pa). */
  tension: number;
  /** Shear pressure acting tangentially across the bond (positive = Pa). */
  shear: number;
}

/** Alias representing mapped stress severity percentages. */
export interface StressSeverity extends BondStress {}

/**
 * Material stress limits in Pascals used by `StressLimits` to classify failure modes.
 */
export interface StressLimitsConfig {
  /** Compression pressure below which bonds are considered elastic (no damage). */
  compressionElasticLimit?: number;
  /** Compression pressure guaranteeing failure when exceeded. */
  compressionFatalLimit?: number;
  /** Tension pressure below which bonds remain elastic (falls back to compression value if omitted). */
  tensionElasticLimit?: number;
  /** Tension pressure guaranteeing failure when exceeded (defaults to compression fatal limit). */
  tensionFatalLimit?: number;
  /** Shear pressure below which bonds are elastic (defaults to compression elastic limit). */
  shearElasticLimit?: number;
  /** Shear pressure guaranteeing failure when exceeded (defaults to compression fatal limit). */
  shearFatalLimit?: number;
}

/** Result codes returned by the fracture bridge wrapper. */
export const FractureResult: {
  /** No fractures were generated. */
  readonly None: 0;
  /** All requested fractures were copied successfully. */
  readonly Success: 1;
  /** Fractures were returned but truncated by the provided buffer size. */
  readonly Truncated: 2;
};
export type FractureResultValue =
  (typeof FractureResult)[keyof typeof FractureResult];

/**
 * Failure modes returned by the stress solver when a bond exceeds a specific limit.
 */
export const StressFailure: {
  /** Compression failure along the bond normal. */
  readonly Compression: 'compression';
  /** Tension failure opposite the bond normal. */
  readonly Tension: 'tension';
  /** Shear failure tangential to the bond. */
  readonly Shear: 'shear';
};
export type StressFailureValue =
  (typeof StressFailure)[keyof typeof StressFailure];

/** Modes specifying whether applied values represent forces or accelerations. */
export const ExtForceMode: {
  /** Treat the supplied vector as a force (N). */
  readonly Force: 0;
  /** Treat the supplied vector as an acceleration (m/s²). */
  readonly Acceleration: 1;
};
export type ExtForceModeValue =
  (typeof ExtForceMode)[keyof typeof ExtForceMode];

/** Debug render channels available from the stress solver. */
export const ExtDebugMode: {
  /** Maximum of compression, tension, or shear stress percentages. */
  readonly Max: 0;
  /** Compression stress percentage. */
  readonly Compression: 1;
  /** Tension stress percentage. */
  readonly Tension: 2;
  /** Shear stress percentage. */
  readonly Shear: 3;
};
export type ExtDebugModeValue =
  (typeof ExtDebugMode)[keyof typeof ExtDebugMode];

/**
 * Utility for evaluating material stress limits against solver output.
 */
export class StressLimits {
  /**
   * @param config Optional overrides for elastic/fatal thresholds (Pascals).
   */
  constructor(config?: StressLimitsConfig);

  /** Map raw stress magnitudes into normalized severities (0-1). */
  severity(stress: BondStress): StressSeverity;
  /** Determine which failure mode, if any, exceeds its fatal limit. */
  failureMode(stress: BondStress): StressFailureValue | null;
  /** Return the fatal threshold (Pa) associated with a failure mode. */
  fatalThreshold(mode: StressFailureValue): number;
  /** Compression elastic threshold (Pa) after defaults are resolved. */
  compressionElasticThreshold(): number;
  /** Compression fatal threshold (Pa) after defaults are resolved. */
  compressionFatalThreshold(): number;
  /** Tension elastic threshold (Pa). */
  tensionElasticThreshold(): number;
  /** Tension fatal threshold (Pa). */
  tensionFatalThreshold(): number;
  /** Shear elastic threshold (Pa). */
  shearElasticThreshold(): number;
  /** Shear fatal threshold (Pa). */
  shearFatalThreshold(): number;
}

/** Create a Vec3 utility object. */
export function vec3(x?: number, y?: number, z?: number): Vec3;
/** Format a number for diagnostic output. */
export function formatNumber(value: number, width: number, precision: number): string;
/** Format a vector for diagnostic output. */
export function formatVec3(v: Vec3, width?: number, precision?: number): string;

/** Solver node descriptor consumed by the low-level stress processor. */
export interface StressNodeDesc {
  /** Node center of mass in local coordinates (meters). */
  com?: Vec3;
  /** Node mass (kg). Zero represents an infinitely massive/static node. */
  mass?: number;
  /** Rotational inertia used for impulse solving (kg·m²). */
  inertia?: number;
}

/** Bond descriptor linking two solver nodes. */
export interface StressBondDesc {
  /** Bond centroid in local coordinates (meters). */
  centroid?: Vec3;
  /** Index of the first node connected by the bond. */
  node0: number;
  /** Index of the second node connected by the bond. */
  node1: number;
}

/** Optional preprocessing parameters controlling solver data preparation. */
export interface StressProcessorDataParams {
  /** Equalize masses across nodes to improve numerical stability. */
  equalizeMasses?: boolean;
  /** Re-center bond centroids during preprocessing. */
  centerBonds?: boolean;
}

/** Description required to construct a low-level StressProcessor instance. */
export interface StressProcessorDescription {
  /** Node descriptors (one per graph node). */
  nodes: StressNodeDesc[];
  /** Bond descriptors linking node indices. */
  bonds: StressBondDesc[];
  /** Optional preprocessing flags influencing solver setup. */
  dataParams?: StressProcessorDataParams;
}

/** Linear/angular velocities applied to solver nodes (m/s, rad/s). */
export interface StressVelocity {
  /** Angular velocity around the node centroid (rad/s). */
  ang?: Vec3;
  /** Linear velocity applied to the node (m/s). */
  lin?: Vec3;
}

/** Impulses returned per bond from the solver (N·s). */
export interface StressImpulse {
  /** Angular impulse around the bond centroid. */
  ang: Vec3;
  /** Linear impulse through the bond centroid. */
  lin: Vec3;
}

/** Iteration control parameters for the conjugate gradient solver. */
export interface StressProcessorSolverParams {
  /** Maximum solver iterations allowed per call. */
  maxIterations?: number;
  /** Residual tolerance used to declare convergence. */
  tolerance?: number;
  /** Resume from previous solution state instead of cold-starting. */
  warmStart?: boolean;
}

/** Arguments for StressProcessor.solve. */
export interface StressProcessorSolveOptions {
  /** Per-node velocities that drive bond impulses for this frame. */
  velocities: StressVelocity[];
  /** Optional override for solver iteration/tolerance settings. */
  solverParams?: StressProcessorSolverParams;
  /** Continue a previous solve instead of resetting internal state. */
  resume?: boolean;
}

/** Result returned from StressProcessor.solve. */
export interface StressProcessorSolveResult {
  /** Iteration count actually consumed by the solver. */
  iterations: number;
  /** Residual errors after the solve (angular/linear). */
  error: { ang: number; lin: number };
  /** Final bond impulses computed for the provided velocities. */
  impulses: StressImpulse[];
}

/**
 * Thin wrapper over the low-level stress solver used for pre-integration testing.
 */
export class StressProcessor {
  private constructor();

  /** Fetch the original node descriptors. */
  getNodes(): StressNodeDesc[];
  /** Fetch the original bond descriptors. */
  getBonds(): StressBondDesc[];
  /** Access the internal descriptor for a specific bond index. */
  internalBondDesc(index: number): StressBondDesc;
  /** Retrieve the most recent bond impulses. */
  getImpulses(): StressImpulse[];
  /** Override the starting impulses before solving. */
  setImpulses(impulses: StressImpulse[]): void;
  /** Update solver iteration/tolerance parameters. */
  setSolverParams(params: StressProcessorSolverParams): void;
  /** Lookup an individual node descriptor by index. */
  nodeDesc(index: number): StressNodeDesc;
  /** Lookup an individual bond descriptor by index. */
  bondDesc(index: number): StressBondDesc;
  /** Run one solver pass using supplied velocities. */
  solve(options: StressProcessorSolveOptions): StressProcessorSolveResult;
  /** Remove a bond; returns true if the bond existed. */
  removeBond(index: number): boolean;
  /** Release solver resources. */
  destroy(): void;
}

/** Extended solver node description mirroring NvBlast support graph nodes. */
export interface ExtStressNodeDesc {
  /** Chunk centroid in asset local coordinates (meters). */
  centroid?: Vec3;
  /** Physical mass assigned to the support chunk (kg). */
  mass?: number;
  /** Volume of the support chunk (m³) used for derived densities. */
  volume?: number;
}

/** Extended solver bond description mapped from NvBlastBond. */
export interface ExtStressBondDesc {
  /** Bond centroid in asset local coordinates (meters). */
  centroid?: Vec3;
  /** Bond normal pointing from node0 to node1. */
  normal?: Vec3;
  /** Bond surface area (m²). */
  area?: number;
  /** Index of the first support node connected by the bond. */
  node0: number;
  /** Index of the second support node connected by the bond. */
  node1: number;
}

/**
 * Settings forwarded to the Blast ExtStressSolver extension.
 */
export interface ExtStressSolverSettings {
  /** Maximum solver iterations performed each update tick. */
  maxSolverIterationsPerFrame?: number;
  /** Graph reduction level applied before solving. */
  graphReductionLevel?: number;
  /** Compression elastic limit (Pa). */
  compressionElasticLimit?: number;
  /** Compression fatal limit (Pa). */
  compressionFatalLimit?: number;
  /** Tension elastic limit (Pa). */
  tensionElasticLimit?: number;
  /** Tension fatal limit (Pa). */
  tensionFatalLimit?: number;
  /** Shear elastic limit (Pa). */
  shearElasticLimit?: number;
  /** Shear fatal limit (Pa). */
  shearFatalLimit?: number;
}

/** Factory description for creating an ExtStressSolver instance. */
export interface ExtStressSolverDescription {
  /** Solver nodes derived from support chunks. */
  nodes: ExtStressNodeDesc[];
  /** Solver bonds derived from asset bonds. */
  bonds: ExtStressBondDesc[];
  /** Optional solver settings overriding defaults. */
  settings?: ExtStressSolverSettings;
}

/** Debug line rendered by ExtStressSolver.fillDebugRender. */
export interface ExtStressDebugLine {
  /** Line start in asset local coordinates (meters). */
  p0: Vec3;
  /** Line end in asset local coordinates (meters). */
  p1: Vec3;
  /** Packed ARGB color at line start. */
  color0: number;
  /** Packed ARGB color at line end. */
  color1: number;
}

/** Fracture command describing a single bond’s damage output. */
export interface ExtStressBondFracture {
  /** User data forwarded from the NvBlastBond. */
  userdata: number;
  /** Graph node index of the first bond endpoint. */
  nodeIndex0: number;
  /** Graph node index of the second bond endpoint. */
  nodeIndex1: number;
  /** Damage value to subtract from the bond’s remaining health. */
  health: number;
}

/** Result returned by ExtStressSolver.generateFractureCommands. */
export interface ExtStressFractureResult {
  /** Collection of bond fractures generated by the solver. */
  fractures: ExtStressBondFracture[];
  /** True when the output was truncated by the provided buffer. */
  truncated: boolean;
  /** Bridge result code indicating success or truncation. */
  result: FractureResultValue;
}

/**
 * High-level wrapper over the Blast ExtStressSolver extension exposed to JS.
 */
export class ExtStressSolver {
  private constructor();

  /** Release solver resources and cached buffers. */
  destroy(): void;
  /** Apply new solver settings (graph reduction may trigger a rebuild). */
  setSettings(settings: ExtStressSolverSettings): void;
  /** Number of graph nodes currently tracked by the solver. */
  graphNodeCount(): number;
  /** Maximum number of internal bonds the solver can process. */
  bondCapacity(): number;
  /** Flush solver state and reset warm-start caches. */
  reset(): void;
  /**
   * Apply a force or acceleration at the nearest node to the supplied position.
   */
  addForce(
    nodeIndex: number,
    localPosition?: Vec3,
    localForce?: Vec3,
    mode?: ExtForceModeValue
  ): void;
  /** Apply gravity to each node in the actor (useful for static structures). */
  addGravity(localGravity?: Vec3): void;
  /** Run one solver update using previously applied forces/accelerations. */
  update(): void;
  /**
   * Count of bonds currently overstressed beyond elastic limits.
   *
   * Call {@link ExtStressSolver.generateFractureCommands} when this is greater than zero
   * and feed the resulting commands to NvBlast (or equivalent) before advancing the
   * destructible simulation.
   */
  overstressedBondCount(): number;
  /**
   * Query fracture commands produced by the last update. Optionally limit copies.
   */
  generateFractureCommands(options?: { maxBonds?: number }): ExtStressFractureResult;
  /**
   * Return the force/torque that should be applied to an actor after bond failure.
   *
   * Invoke immediately after applying generated fracture commands (before the next
   * {@link ExtStressSolver.update}) so that rigid-body impulses stay in sync with the
   * solver’s notion of bond breakage.
   */
  getExcessForces(actorIndex: number, centerOfMass?: Vec3): { force: Vec3; torque: Vec3 } | null;
  /** Retrieve debug lines for visualization of stress magnitudes. */
  fillDebugRender(options?: { mode?: ExtDebugModeValue; scale?: number }): ExtStressDebugLine[];
  /** Solver linear and angular residual errors. */
  stressError(): { lin: number; ang: number };
  /** True if the solver converged within the requested tolerance. */
  converged(): boolean;
}

/** Sizes (in bytes) of C++ structures exported by the WASM bridge. */
export interface RuntimeSizes {
  /** sizeof(StressVec3). */
  vec3: number;
  /** sizeof(StressNodeDesc). */
  node: number;
  /** sizeof(StressBondDesc). */
  bond: number;
  /** sizeof(StressVelocity). */
  velocity: number;
  /** sizeof(StressImpulse). */
  impulse: number;
  /** sizeof(StressDataParams). */
  dataParams: number;
  /** sizeof(StressSolverParams). */
  solverParams: number;
  /** sizeof(StressErrorSq). */
  errorSq: number;
  /** sizeof(ExtStressNodeDesc). */
  extNode: number;
  /** sizeof(ExtStressBondDesc). */
  extBond: number;
  /** sizeof(ExtStressSolverSettingsDesc). */
  extSettings: number;
  /** sizeof(ExtStressDebugLine). */
  extDebugLine: number;
  /** sizeof(ExtStressBondFracture). */
  extBondFracture: number;
  /** sizeof(ExtStressFractureCommands). */
  extFractureCommands: number;
}

/** Runtime interface returned by `loadStressSolver`. */
export interface StressRuntime {
  /** Underlying Emscripten module powering the runtime. */
  module: unknown;
  /** Structure sizes resolved from the bridge for buffer allocation. */
  sizes: RuntimeSizes;
  /** Exported fracture result codes for convenience. */
  FractureResult: typeof FractureResult;
  /** Vector factory shared with stress.js. */
  vec3: typeof vec3;
  /** Stress limit helper class. */
  StressLimits: typeof StressLimits;
  /** Failure mode enumerations. */
  StressFailure: typeof StressFailure;
  /** Force mode enumerations. */
  ExtForceMode: typeof ExtForceMode;
  /** Debug visualization mode enumerations. */
  ExtDebugMode: typeof ExtDebugMode;
  /** Utility for computing bond stress from impulses. */
  computeBondStress: typeof computeBondStress;
  /** Default conjugate gradient solver parameters. */
  defaultSolverParams(): Required<StressProcessorSolverParams>;
  /** Default ExtStressSolver settings. */
  defaultExtSettings(): Required<ExtStressSolverSettings>;
  /** Factory for the low-level StressProcessor. */
  createProcessor(description: StressProcessorDescription): StressProcessor;
  /** Factory for the high-level ExtStressSolver. */
  createExtSolver(description: ExtStressSolverDescription): ExtStressSolver;
}

/** Options to customize module instantiation when loading the solver. */
export interface LoadStressSolverOptions {
  /** Custom Emscripten module overrides (passed to the factory). */
  module?: Record<string, unknown>;
}

/** Load the WASM stress solver module and expose helper factories. */
export function loadStressSolver(options?: LoadStressSolverOptions): Promise<StressRuntime>;

/**
 * Compute bond stress using solver impulse output and node positions.
 *
 * @param bond Bond linking node indices used to fetch centroids.
 * @param impulse Solver impulse acting on the bond (N·s).
 * @param nodes Node descriptors containing centroids (meters).
 * @param bondArea Remaining bond surface area (m²).
 */
export function computeBondStress(
  bond: { node0: number; node1: number },
  impulse: StressImpulse,
  nodes: Array<{ com: Vec3 }> | ReadonlyArray<{ com: Vec3 }> | undefined,
  bondArea: number
): BondStress;

/** Return a − b. */
export function subtract(a: Vec3, b: Vec3): Vec3;
/** Dot product between vectors. */
export function dot(a: Vec3, b: Vec3): number;
/** Squared magnitude of a vector. */
export function magnitudeSquared(v: Vec3): number;
/** Vector magnitude. */
export function magnitude(v: Vec3): number;
/** Normalized vector (returns zero vector when length is ~0). */
export function normalize(v: Vec3): Vec3;
export interface Vec3 {
  x: number;
  y: number;
  z: number;
}
