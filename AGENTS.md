# AGENTS.md

## Cursor Cloud specific instructions

### Project overview

This is the NVIDIA PhysX monorepo containing PhysX, Blast, and Flow SDKs. The active development focus is on the **Blast Stress Solver JavaScript/WASM library** (`blast/blast-stress-solver`) and Three.js + Rapier browser demos (`blast/js_stress_example`).

### Prerequisites

- **Node.js** (v22+) is available via nvm.
- **Emscripten SDK** is required for WASM compilation. It should be installed at `/opt/emsdk`. If not present, install it:
  ```bash
  git clone https://github.com/emscripten-core/emsdk.git /opt/emsdk
  cd /opt/emsdk
  ./emsdk install 3.1.51
  ./emsdk activate 3.1.51
  source /opt/emsdk/emsdk_env.sh
  ```
  The `~/.bashrc` should source it automatically. Verify with `emcc --version`.

### First-time setup

```bash
# 1. Install root dependencies (three.js, etc.)
cd /home/user/PhysX
npm install

# 2. Install blast-stress-solver dependencies
cd blast/blast-stress-solver
npm install --ignore-scripts   # avoid triggering full WASM rebuild on install

# 3. Install js_stress_example dependencies
cd ../js_stress_example
npm install
```

### Full build (from scratch)

```bash
# Ensure emsdk is sourced
source /opt/emsdk/emsdk_env.sh

# Build the blast-stress-solver package (WASM + TypeScript)
cd /home/user/PhysX/blast/blast-stress-solver
npm run build

# Build bridge demo TypeScript (for bridge-split-demo.html, bridge-ext.html)
cd ../js_stress_example
npx tsc

# Build wall/tower demo JS (esbuild, not tsc — they use import-map bare specifiers)
npx esbuild wall-demolition.ts --outfile=dist/wall-demolition.js --format=esm
npx esbuild tower-collapse.ts --outfile=dist/tower-collapse.js --format=esm

# Run tests
cd ../blast-stress-solver
npm test

# Serve demos
cd /home/user/PhysX
npm start   # http://localhost:8000
```

### Build chain details

1. `cd blast/js_stress_example && npm run build` — compiles C++ stress solver to WASM via `emcc`, outputs `dist/stress_solver.{cjs,mjs,wasm}`. Takes ~20 seconds (two targets: node-cjs + browser-esm).
2. `cd blast/blast-stress-solver && npm run build` — runs step 1 as `prebuild`, then bundles TypeScript with `tsup`, copies WASM to `dist/`
3. `cd blast/js_stress_example && npx tsc` — compiles bridge demo TS to `dist/`. Pre-existing type errors are expected; `noEmitOnError: false` ensures output is still generated.
4. Wall/tower demo JS: `npx esbuild wall-demolition.ts --outfile=dist/wall-demolition.js --format=esm` (repeat for `tower-collapse.ts`). These use import-map bare specifiers so **esbuild** is used instead of tsc.
5. `npm start` (root) — starts static file server on port 8000, serves demos and vendor aliases for three.js/rapier

### Running tests

- `cd blast/blast-stress-solver && npm test` — runs all vitest tests (authoring, gravity, damage, headless scenarios, integration, Three.js adapter)
- Key test files in `blast/blast-stress-solver/src/tests/`:
  - `rapier.headless-scenarios.test.ts` — gravity stability, projectile collisions, material strength, catastrophic vs partial damage, damage toggle, parameter sweeps, bond inspection, structure-specific behavior, determinism, API surface, scenario builder correctness (including bond isotropy verification)
  - `rapier.integration.test.ts` — full WASM+Rapier pipeline (fracture, projectiles, damage, profiler)
  - `rapier.damage.test.ts` — damage system unit tests
  - `authoring.*.test.ts` — bond authoring / triangle bonding tests
- `cd blast/js_stress_example && npm run test:split` — split/fracture integration tests (vitest)
- `cd blast/js_stress_example && npm run test:bridge-ui` — Playwright browser tests (requires `npx playwright install chromium` first)

### Browser demos

After running `npm start` at the root:

**Primary demos (high-level API):**
- **`/blast/js_stress_example/wall-demolition.html`** — Destructible brick wall. Click to shoot projectiles. Config panel for wall geometry, projectile params, material scale (log slider), and **Auto Bonds (experimental)** toggle.
- **`/blast/js_stress_example/tower-collapse.html`** — Destructible tower. Same config panel pattern. Includes diagonal bonds and small-body damping.

**Bridge demos:**
- **`bridge-split-demo.html`** — Fully featured destructible bridge with real-time config, projectile spawning, and full fracture into independent physics bodies. Requires `npx tsc` build step.
- `bridge-ext.html` — Older two-phase fracture demo (also requires `npx tsc`)

**Legacy:** `bridge-demo.html` — Stress coloring only; fracture disabled at `simulation.js:117`.

### Architecture notes

**Stress solver paths for bond breaking:**
1. **Gravity stress** — `solver.addActorGravity()` + `solver.update()` computes bond stress from gravity. Overstressed bonds are fractured via `generateFractureCommandsPerActor()` + `applyFractureCommands()`.
2. **Contact force injection** — Rapier contact forces from projectile impacts are rotated to body-local space and injected via `solver.addForce()` with splash radius, so the stress solver sees collision impacts too. Controlled by `contactForceScale` option in `destructible-core.ts`.
3. **Damage system** — Contact forces -> `damageSystem.onImpact()` -> node health decrement -> bond removal. Separate from the stress solver. Disabled by default in wall/tower demos.

**Bond normalization:** Area normalization in scenario builders (`wallScenario.ts`, `towerScenario.ts`, `bridgeScenario.ts`) uses **isotropic scaling** — a single uniform scale factor (geometric mean of per-axis scales) to avoid directional bias. Per-axis normalization causes horizontal layer separation under gravity because it makes vertical bonds stronger than horizontal bonds.

**Auto-bonding (experimental):** `applyAutoBondingToScenario()` from `blast-stress-solver/three` replaces manual grid bonds with geometry-derived bonds computed by the WASM solver's `createBondsFromTriangles` API. Uses actual triangle-mesh shared surfaces for bond area/normal calculation. Requires `scenario.parameters.fragmentGeometries` (array of `THREE.BufferGeometry` per node). Toggle available in wall/tower demo UIs.

**Key source locations:**
- Scenario builders: `blast/blast-stress-solver/src/scenarios/` (wallScenario.ts, towerScenario.ts, bridgeScenario.ts)
- Core fracture logic: `blast/blast-stress-solver/src/rapier/destructible-core.ts` (`processOneFracturePass`, contact force injection, bond tracking)
- WASM bridge: `blast/blast-stress-solver/src/stress.ts` (addForce, addActorGravity, generateFractureCommandsPerActor, applyFractureCommands, createBondsFromTriangles)
- Auto-bonding: `blast/blast-stress-solver/src/three/autoBonding.ts`
- Damage system: `blast/blast-stress-solver/src/rapier/damage.ts`

### Gotchas

- The `blast/blast-stress-solver` build has benign `import.meta` CJS warnings from tsup — these do not affect functionality.
- TypeScript strict checking (`tsc --noEmit`) in `blast/js_stress_example` shows pre-existing type errors; `noEmitOnError: false` in tsconfig ensures files are still emitted.
- The WASM build takes ~20 seconds per run (two targets: node-cjs + browser-esm).
- `npm install --ignore-scripts` is used for `blast/blast-stress-solver` during dependency refresh to avoid triggering a full rebuild on install.
- Wall/tower demo dist JS files are built with **esbuild** (not tsc). They use bare import specifiers resolved by the HTML import map at runtime.
- Projectile TTL is in **wall-clock seconds** (via `performance.now()`), not simulation time. In headless tests, use very short TTL values (e.g., 0.001) since physics steps execute much faster than real time.
- `getActiveBondsCount()` uses JS-side tracking (`bondTable.length - removedBondIndices.size`) which may lag behind the WASM solver's internal bond state after `applyFractureCommands`. Multiple resimulation passes (controlled by `maxResimulationPasses`) help propagate fractures.
- Bond areas directly affect stress: `stress = force / area`. Larger area = lower stress = harder to break. This is why isotropic normalization matters — asymmetric areas create directional weakness.
- **Missing `stress_solver.cjs` / `stress_solver.wasm`:** Tests that load the WASM stress solver (authoring, gravity, damage, integration) will fail with `Cannot find module './stress_solver.cjs'` if the C++ → WASM build has not been run. This happens when the Emscripten SDK (`emcc`) is not installed or not on `PATH`. To resolve:
  1. Install and activate emsdk (see **Prerequisites** above).
  2. Run `cd blast/blast-stress-solver && npm run build` (the `prebuild` step compiles the WASM).
  3. Alternatively, set `BLAST_STRESS_SOLVER_SKIP_WASM_BUILD=1` to skip the WASM compilation and iterate on TypeScript only — but WASM-dependent tests will still fail.
  
  The 7 test files that do **not** require the WASM binary (scenario builders, Three.js adapter, bundle exports, split migrator, headless scenarios) will pass regardless.

### CI

- GitHub Actions workflow at `.github/workflows/ci.yml`
- **Push** trigger: only runs on `feat/rapier-destruction` branch (production)
- **Pull request** trigger: runs on all PRs
- This prevents double CI runs when PRs are opened from feature branches
