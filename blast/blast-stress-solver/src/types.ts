import type { BufferGeometry, Matrix4 } from 'three';

/**
 * Public type definitions for the Blast Stress Solver JS/WASM bridge.
 *
 * These mirror the API made available from the WASM module and the higher-level
 * ExtStressSolver wrapper. Most units follow SI conventions:
 * - Positions and lengths: meters
 * - Mass: kilograms
 * - Time: seconds
 * - Forces: Newtons
 * - Pressures/Stresses: Pascals
 */

/**
 * 3D vector structure used for centroids, forces, torques, and impulses.
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

/** Alias representing mapped stress severity percentages (0..1). */
export type StressSeverity = BondStress;

/**
 * Bond generation modes used by {@link StressRuntime.createBondsFromTriangles}.
 *
 * - `'exact'` — Computes precise shared surfaces from triangle geometry. Requires chunks
 *   to have coincident coplanar faces (shared vertices/edges). Best for Voronoi fracture
 *   output or other pre-fractured meshes where chunks share exact geometry.
 *
 * - `'average'` — Approximates bond interfaces using convex hulls and a distance tolerance.
 *   Requires {@link BondingConfig.maxSeparation} to be set. Use for manually placed bricks/blocks,
 *   imported meshes that don't share exact vertices, or geometry with floating-point noise.
 */
export type BondingMode = 'exact' | 'average';

/**
 * Configuration for bond generation via {@link StressRuntime.createBondsFromTriangles}.
 *
 * @example
 * ```ts
 * // Exact mode (default) for pre-fractured geometry with shared faces
 * rt.createBondsFromTriangles(chunks, { mode: 'exact' });
 *
 * // Average mode for geometry with small gaps (e.g., 1cm tolerance)
 * rt.createBondsFromTriangles(chunks, { mode: 'average', maxSeparation: 0.01 });
 * ```
 */
export interface BondingConfig {
  /**
   * Algorithm used to build bond interfaces.
   *
   * - `'exact'` (default): Precise triangle intersection. Fails silently if faces don't align exactly.
   * - `'average'`: Hull-based approximation with distance tolerance. Requires `maxSeparation`.
   *
   * **Tip:** If you get 0 bonds with `'exact'`, try `'average'` with a small `maxSeparation`
   * to handle floating-point gaps.
   */
  mode?: BondingMode;
  /**
   * Maximum face-to-face distance (meters) that will still form a bond in `'average'` mode.
   *
   * **Required** when `mode: 'average'`. Must be > 0. A 5cm gap needs `maxSeparation >= 0.05`.
   *
   * @throws Error if `mode: 'average'` and `maxSeparation` is missing or <= 0.
   */
  maxSeparation?: number;
}

/**
 * Triangle soup input for a single chunk in bond generation.
 *
 * Each chunk represents one piece of a fractured object. Bonds are computed between
 * chunks that share faces (exact mode) or are within `maxSeparation` distance (average mode).
 *
 * @example
 * ```ts
 * const chunk: AuthoringChunkInput = {
 *   triangles: new Float32Array([
 *     // Triangle 1: v0, v1, v2 (9 floats)
 *     0, 0, 0,  1, 0, 0,  1, 1, 0,
 *     // Triangle 2: ...
 *   ]),
 *   isSupport: true
 * };
 * ```
 */
export interface AuthoringChunkInput {
  /**
   * Flattened vertex positions in asset-local space.
   *
   * Format: `[x0, y0, z0, x1, y1, z1, x2, y2, z2, ...]` where each group of 9 floats
   * defines one triangle (3 vertices × 3 components). Must be divisible by 9.
   */
  triangles: Float32Array | ReadonlyArray<number>;
  /**
   * Whether this chunk participates in bond generation.
   *
   * When `true`, the bond generator will look for adjacent chunks to connect via bonds.
   * When `false`, no bonds will be generated to or from this chunk.
   *
   * **This is NOT the same as "anchored to ground."** To make a chunk act as a fixed
   * anchor point (infinite mass, like the ground), set `mass: 0` on the corresponding
   * {@link ExtStressNodeDesc} when creating the solver.
   *
   * @default true
   */
  isSupport?: boolean;
}

/**
 * Options for converting a THREE.BufferGeometry to an {@link AuthoringChunkInput}.
 *
 * @example
 * ```ts
 * const chunk = chunkFromBufferGeometry(geometry, {
 *   isSupport: true,
 *   applyMatrix: mesh.matrixWorld, // bake world transform
 *   nonIndexed: true,
 *   cloneGeometry: true
 * });
 * ```
 */
export interface BufferGeometryChunkOptions {
  /**
   * Whether this chunk participates in bond generation.
   *
   * **Note:** This controls bond generation only, not physics behavior.
   * To make a chunk act as a fixed anchor (like the ground), set `mass: 0`
   * on the corresponding {@link ExtStressNodeDesc}.
   *
   * @default true
   */
  isSupport?: boolean;
  /**
   * World or local transform to apply before extracting triangle data.
   *
   * Pass `mesh.matrixWorld` to convert geometry from local to world space.
   * The geometry is cloned first (unless `cloneGeometry: false`).
   */
  applyMatrix?: Matrix4;
  /**
   * Convert indexed geometries to non-indexed triangle lists.
   *
   * The bond generator requires non-indexed triangles. Set to `false` only if
   * your geometry is already in triangle-list form.
   *
   * @default true
   */
  nonIndexed?: boolean;
  /**
   * Clone the geometry before applying transforms or conversion.
   *
   * Set to `false` to mutate the original geometry in-place (faster but destructive).
   *
   * @default true
   */
  cloneGeometry?: boolean;
}

/** Resolver used by helper utilities to determine per-geometry options. */
export type BufferGeometryChunkResolver =
  | BufferGeometryChunkOptions
  | ((geometry: BufferGeometry, index: number) => BufferGeometryChunkOptions | void);

/**
 * Material stress limits in Pascals used by StressLimits to classify failure modes.
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

/**
 * Result code values returned by fracture generation helpers.
 *
 * - None (0): No commands were produced (e.g., no overstressed bonds or capacity was zero).
 * - Success (1): Commands successfully generated and fully written to the provided buffers.
 * - Truncated (2): Output was truncated due to insufficient buffer capacity; increase buffer sizes and retry.
 */
export enum FractureResultValue {
  /** No commands were produced. */
  None = 0,
  /** Commands generated successfully without truncation. */
  Success = 1,
  /** Output was truncated by buffer limits. */
  Truncated = 2
}

/** Failure mode keys returned by the solver when limits are exceeded. */
export type StressFailureValue = 'compression' | 'tension' | 'shear';

/**
 * Force mode used by ExtStressSolver.addForce.
 *
 * - Force (0): The vector is interpreted as a force in Newtons (mass-dependent effect).
 * - Acceleration (1): The vector is interpreted as an acceleration in m/s² (mass-independent effect).
 */
export enum ExtForceModeValue {
  /** Apply a force in Newtons (N). */
  Force = 0,
  /** Apply an acceleration in meters per second squared (m/s²). */
  Acceleration = 1
}

/**
 * Debug render channels available from the solver.
 *
 * - Max (0): Maximum of compression, tension, and shear stress percentages.
 * - Compression (1): Compression stress percentage only.
 * - Tension (2): Tension stress percentage only.
 * - Shear (3): Shear stress percentage only.
 */
export enum ExtDebugModeValue {
  /** Maximum of compression, tension, and shear stress percentages. */
  Max = 0,
  /** Compression stress percentage only. */
  Compression = 1,
  /** Tension stress percentage only. */
  Tension = 2,
  /** Shear stress percentage only. */
  Shear = 3
}

/**
 * Solver node descriptor consumed by the low-level stress processor.
 */
export interface StressNodeDesc {
  /** Node center of mass in local coordinates (meters). */
  com?: Vec3;
  /** Node mass (kg). Zero represents an infinitely massive/static node. */
  mass?: number;
  /** Rotational inertia used for impulse solving (kg·m²). */
  inertia?: number;
}

/**
 * Bond descriptor linking two solver nodes.
 */
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

/** Interface of the thin low-level stress solver wrapper. */
export interface StressProcessor {
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

/**
 * Node descriptor for the ExtStressSolver, representing one chunk in the stress graph.
 *
 * @example
 * ```ts
 * // Dynamic chunk (participates in stress calculations)
 * const dynamicNode: ExtStressNodeDesc = {
 *   centroid: { x: 0, y: 1, z: 0 },
 *   mass: 10,    // 10 kg
 *   volume: 0.01 // 0.01 m³
 * };
 *
 * // Static/anchored chunk (fixed to ground, infinite mass)
 * const anchorNode: ExtStressNodeDesc = {
 *   centroid: { x: 0, y: 0, z: 0 },
 *   mass: 0,     // 0 = infinite mass = fixed anchor
 *   volume: 0.01
 * };
 * ```
 */
export interface ExtStressNodeDesc {
  /** Chunk centroid in asset local coordinates (meters). */
  centroid?: Vec3;
  /**
   * Physical mass of the chunk (kg).
   *
   * - **Positive value**: Dynamic chunk that responds to forces and gravity.
   * - **Zero (0)**: Static/anchored chunk with infinite mass. Acts as a fixed point
   *   (like the ground or a wall). Gravity and forces pull other chunks against
   *   these anchors, creating internal stresses in the bonds.
   *
   * @example
   * ```ts
   * // Ground-anchored chunks should have mass = 0
   * const nodes = chunks.map((chunk, i) => ({
   *   centroid: getCentroid(chunk),
   *   mass: isGroundChunk(i) ? 0 : computeMass(chunk),
   *   volume: computeVolume(chunk)
   * }));
   * ```
   */
  mass?: number;
  /** Volume of the chunk (m³), used for density-based calculations. */
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

/** Settings forwarded to the Blast ExtStressSolver extension. */
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

/** Result returned by ExtStressSolver.generateFractureCommands (single-actor). */
export interface ExtStressFractureSingleResult extends ExtStressFractureResult {
  /** Family index of the actor these fractures belong to. */
  actorIndex: number;
}

/**
 * Split event reported by the WASM bridge after applying fracture commands.
 * Contains the parent actor and the set of child actors created, including
 * the node lists owned by each child.
 */
export interface SplitEvent {
  /** Family index of the parent actor that split. */
  parentActorIndex: number;
  /** Child actors created by the split (indices and owned nodes). */
  children: Array<{ actorIndex: number; nodes: number[] }>;
}

/** High-level wrapper over the Blast ExtStressSolver extension exposed to JS. */
export interface ExtStressSolver {
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
  /** Apply a force or acceleration at the nearest node to the supplied position. */
  addForce(nodeIndex: number, localPosition?: Vec3, localForce?: Vec3, mode?: ExtForceModeValue): void;
  /**
   * Apply the same gravity vector to every actor currently tracked by the solver (useful when all actors share a static frame).
   * For actor-specific gravity aligned to each body’s rotation, prefer {@link ExtStressSolver.addActorGravity}.
   */
  addGravity(localGravity?: Vec3): void;
  /**
   * Apply gravity to a specific actor using that actor’s local coordinate frame.
   *
   * Typical workflow:
   * 1. Fetch actors via {@link ExtStressSolver.actors} (or split-event payloads) and cache their `actorIndex`.
   * 2. Convert your world gravity vector into the actor’s local space. Example (Three.js):
   *    `const local = world.clone().applyQuaternion(actorWorldQuaternion.clone().invert());`
   * 3. Call `addActorGravity(actorIndex, local)`. The method returns `false` if the actor no longer exists (e.g., split/destroyed).
   *
   * Passing `{0,0,0}` is a no-op, so skip the call entirely when you do not need to apply per-actor gravity for that frame.
   */
  addActorGravity(actorIndex: number, localGravity?: Vec3): boolean;
  /** Run one solver update using previously applied forces/accelerations. */
  update(): void;
  /** Count of bonds currently overstressed beyond elastic limits. */
  overstressedBondCount(): number;
  /** Snapshot the current actor table (actor index + owned nodes). */
  actors(): Array<{ actorIndex: number; nodes: number[] }>;
  /** Generate fracture commands for the last update. */
  generateFractureCommands(options?: { maxBonds?: number }): ExtStressFractureSingleResult;
  /** Generate fracture commands for each actor individually. */
  generateFractureCommandsPerActor(): Array<{ actorIndex: number; fractures: ExtStressBondFracture[] }>;
  /** Apply per-actor fracture commands and return split information. */
  applyFractureCommands(fractureSets: Array<{ actorIndex: number; fractures: ExtStressBondFracture[] }>): SplitEvent[];
  /** Force/torque that should be applied to an actor after bond failure. */
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
  /** sizeof(ExtActorBuffer). */
  extActor: number;
  /** sizeof(ExtSplitEvent). */
  extSplitEvent: number;
  /** sizeof(ExtStressBondDesc) returned by authoring bridge. */
  authoringBond?: number;
}

/**
 * Runtime interface returned by loadStressSolver. Provides factories for the
 * low-level StressProcessor and the high-level ExtStressSolver, along with
 * constants and helpers mirrored from the underlying module.
 */
export interface StressRuntime {
  /** Underlying Emscripten module powering the runtime. */
  module: unknown;
  /** Structure sizes resolved from the bridge for buffer allocation. */
  sizes: RuntimeSizes;
  /** Exported fracture result codes for convenience. */
  FractureResult: { readonly None: 0; readonly Success: 1; readonly Truncated: 2 };
  /** Vector factory shared with stress.ts. */
  vec3: (x?: number, y?: number, z?: number) => Vec3;
  /** Stress limit helper class. */
  StressLimits: new (config?: StressLimitsConfig) => StressLimits;
  /** Failure mode enumerations. */
  StressFailure: { readonly Compression: 'compression'; readonly Tension: 'tension'; readonly Shear: 'shear' };
  /** Force mode enumerations. */
  ExtForceMode: { readonly Force: 0; readonly Acceleration: 1 };
  /** Debug visualization mode enumerations. */
  ExtDebugMode: { readonly Max: 0; readonly Compression: 1; readonly Tension: 2; readonly Shear: 3 };
  /** Utility for computing bond stress from impulses. */
  computeBondStress: (bond: { node0: number; node1: number }, impulse: StressImpulse, nodes: Array<{ com: Vec3 }> | ReadonlyArray<{ com: Vec3 }> | undefined, bondArea: number) => BondStress;
  /** Default conjugate gradient solver parameters. */
  defaultSolverParams(): Required<StressProcessorSolverParams>;
  /**
   * Default `ExtStressSolver` settings (Pascals), suitable as a starting point.
   *
   * @example
   * ```ts
   * const rt = await loadStressSolver();
   * const settings = rt.defaultExtSettings();
   * // Customize material behavior (e.g., stronger tension, more iterations)
   * settings.maxSolverIterationsPerFrame = 64;
   * settings.tensionElasticLimit = 0.05;
   * settings.tensionFatalLimit = 0.2;
   * const solver = rt.createExtSolver({ nodes, bonds, settings });
   * ```
   * @see StressRuntime.createExtSolver
   * @see ExtStressSolver.setSettings
   */
  defaultExtSettings(): Required<ExtStressSolverSettings>;
  /** Factory for the low-level StressProcessor. */
  createProcessor(description: StressProcessorDescription): StressProcessor;
  /**
   * Factory for the high-level `ExtStressSolver`.
   *
   * Typical lifecycle: add forces/accelerations → `update()` → if overstressed,
   * generate and apply fracture commands.
   *
   * @example
   * ```ts
   * import { loadStressSolver } from 'blast-stress-solver';
   * const rt = await loadStressSolver();
   * const settings = rt.defaultExtSettings();
   * const solver = rt.createExtSolver({ nodes, bonds, settings });
   * 
   * // Per-frame
   * solver.addGravity({ x: 0, y: -9.81, z: 0 });
   * solver.update();
   * if (solver.overstressedBondCount() > 0) {
   *   const perActor = solver.generateFractureCommandsPerActor();
   *   const splitEvents = solver.applyFractureCommands(perActor);
   *   // Rebuild physics bodies for split children here
   * }
   * ```
   * @see ExtStressSolver.addForce
   * @see ExtStressSolver.addGravity
   * @see ExtStressSolver.update
   * @see ExtStressSolver.overstressedBondCount
   * @see ExtStressSolver.generateFractureCommandsPerActor
   * @see ExtStressSolver.applyFractureCommands
   */
  createExtSolver(description: ExtStressSolverDescription): ExtStressSolver;
  /**
   * Generate NvBlast-style bonds from prefractured triangle meshes.
   *
   * Analyzes triangle geometry across all chunks to find where they share faces
   * (exact mode) or are close enough to be considered adjacent (average mode).
   *
   * **Common reasons for 0 bonds returned:**
   * - Chunks don't actually touch (verify positions match up)
   * - Using `'exact'` mode with floating-point gaps—try `'average'` with `maxSeparation`
   * - All chunks have `isSupport: false`—bonds only connect support chunks
   * - World transforms not applied—use `applyMatrix` in `chunksFromBufferGeometries`
   *
   * @param chunks Per-chunk triangle soups in local coordinates (meters). Each chunk
   *   needs `triangles` (9 floats per triangle) and optionally `isSupport` (default true).
   * @param config Bonding configuration. Use `mode: 'exact'` (default) for pre-fractured
   *   geometry with shared faces, or `mode: 'average'` with `maxSeparation` for geometry
   *   with gaps.
   * @returns Array of bond descriptors with `centroid`, `normal`, `area`, `node0`, `node1`.
   *   Returns empty array if no bonds found or fewer than 2 support chunks provided.
   * @throws Error if `mode: 'average'` and `maxSeparation` is missing or <= 0.
   * @throws Error if any chunk's triangle count is not divisible by 9.
   *
   * @example
   * ```ts
   * // Exact mode for Voronoi-fractured geometry
   * const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });
   *
   * // Average mode for manually placed bricks with small gaps
   * const bonds = rt.createBondsFromTriangles(chunks, {
   *   mode: 'average',
   *   maxSeparation: 0.01 // 1cm tolerance
   * });
   * ```
   *
   * @see AuthoringChunkInput
   * @see BondingConfig
   * @see chunksFromBufferGeometries - Helper to convert THREE.BufferGeometry arrays
   */
  createBondsFromTriangles(chunks: AuthoringChunkInput[], config?: BondingConfig): ExtStressBondDesc[];
}

/** Options to customize module instantiation when loading the solver. */
export interface LoadStressSolverOptions {
  /** Custom Emscripten module overrides (passed to the factory). */
  module?: Record<string, unknown>;
}

/** Public methods exposed by StressLimits helper. */
export interface StressLimits {
  /** Map raw stress magnitudes into normalized severities (0..1). */
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


