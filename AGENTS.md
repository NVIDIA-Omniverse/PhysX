# AGENTS.md

## Cursor Cloud specific instructions

### Project overview

This is the NVIDIA PhysX monorepo containing PhysX, Blast, and Flow SDKs. The active development focus is on the **Blast Stress Solver JavaScript/WASM library** (`blast/blast-stress-solver`) and Three.js + Rapier browser demos (`blast/js_stress_example`).

### Prerequisites

- **Emscripten SDK** must be installed at `/opt/emsdk` and sourced (`source /opt/emsdk/emsdk_env.sh`) before building the WASM artifacts. The `~/.bashrc` sources it automatically.
- **Node.js** (v22+) is available via nvm.

### Build chain (dependency order)

1. `cd blast/js_stress_example && npm run build` — compiles C++ stress solver to WASM via `emcc`, outputs `dist/stress_solver.{cjs,mjs,wasm}`
2. `cd blast/js_stress_example && npx tsc` — compiles TypeScript demo sources (`bridge-stress-ext.ts`, `split-bridge-stress.ts`, etc.) to `dist/`. Required for `bridge-ext.html` and `bridge-split-demo.html`. Pre-existing type errors are expected; `noEmitOnError: false` ensures output is still generated.
3. `cd blast/blast-stress-solver && npm run build` — runs step 1 as `prebuild`, then bundles TypeScript with `tsup`, copies WASM to `dist/`
4. `cd blast/js_stress_example && npx esbuild wall-demolition.ts --outfile=dist/wall-demolition.js --format=esm` — builds wall demo JS (repeat for `tower-collapse.ts`). These use import-map bare specifiers so esbuild is used instead of tsc.
5. `npm start` (root) — starts static file server on port 8000, serves demos and vendor aliases for three.js/rapier

### Running tests

- `cd blast/blast-stress-solver && npm test` — **110 tests across 11 files, all passing** (authoring, gravity, damage, headless scenarios, integration, Three.js adapter)
- Key test files:
  - `src/tests/rapier.headless-scenarios.test.ts` — **40 tests**: gravity stability, projectile collisions, material strength, catastrophic vs partial damage, damage toggle, parameter sweeps, bond inspection, structure-specific behavior, determinism, API surface, and scenario builder correctness (including bond isotropy verification)
  - `src/tests/rapier.integration.test.ts` — 9 tests: full WASM+Rapier pipeline (fracture, projectiles, damage, profiler)
  - `src/tests/rapier.damage.test.ts` — damage system unit tests
  - `src/tests/authoring.*.test.ts` — bond authoring / triangle bonding tests
- `cd blast/js_stress_example && npm run test:split` — split/fracture integration tests (vitest); 9/10 pass, `organicSplit.spec.ts` has a stale expectation
- `cd blast/js_stress_example && npm run test:bridge-ui` — Playwright browser tests (requires `npx playwright install chromium` first)

### Browser demos

After running `npm start` at the root:

**Primary demos (high-level API):**
- **`http://localhost:8000/blast/js_stress_example/wall-demolition.html`** — Destructible brick wall. Click to shoot projectiles. Config panel for wall geometry, projectile params, material scale (log slider), and **Auto Bonds (experimental)** toggle.
- **`http://localhost:8000/blast/js_stress_example/tower-collapse.html`** — Destructible tower. Same config panel pattern. Includes diagonal bonds and small-body damping.

**Bridge demos:**
- **`bridge-split-demo.html`** — Fully featured destructible bridge with real-time config, projectile spawning, and full fracture into independent physics bodies. Requires `npx tsc` build step.
- `bridge-ext.html` — Older two-phase fracture demo (also requires `npx tsc`)

**Legacy:** `bridge-demo.html` — Stress coloring only; fracture disabled at `simulation.js:117`.

### Architecture notes

**Stress solver paths for bond breaking:**
1. **Gravity stress** — `solver.addActorGravity()` + `solver.update()` computes bond stress from gravity. Overstressed bonds are fractured via `generateFractureCommandsPerActor()` + `applyFractureCommands()`.
2. **Contact force injection** — Rapier contact forces from projectile impacts are rotated to body-local space and injected via `solver.addForce()` with splash radius, so the stress solver sees collision impacts too.
3. **Damage system** — Contact forces → `damageSystem.onImpact()` → node health decrement → bond removal. Separate from the stress solver. Disabled by default in wall/tower demos.

**Bond normalization:** Area normalization in scenario builders (`wallScenario.ts`, `towerScenario.ts`, `bridgeScenario.ts`) uses **isotropic scaling** — a single uniform scale factor (geometric mean of per-axis scales) to avoid directional bias. Previous per-axis normalization caused horizontal layer separation under gravity.

**Auto-bonding (experimental):** `applyAutoBondingToScenario()` from `blast-stress-solver/three` replaces manual grid bonds with geometry-derived bonds computed by the WASM solver's `createBondsFromTriangles` API. Uses actual triangle-mesh shared surfaces for bond area/normal calculation. Toggle available in wall/tower demo UIs.

### Gotchas

- The `blast/blast-stress-solver` build has benign `import.meta` CJS warnings from tsup — these do not affect functionality.
- TypeScript strict checking (`tsc --noEmit`) in `blast/js_stress_example` shows pre-existing type errors; `noEmitOnError: false` in tsconfig ensures files are still emitted.
- The WASM build takes ~20 seconds per run (two targets: node-cjs + browser-esm).
- `npm install --ignore-scripts` is used for `blast/blast-stress-solver` during dependency refresh to avoid triggering a full rebuild on install.
- Wall/tower demo dist JS files (`dist/wall-demolition.js`, `dist/tower-collapse.js`) are built with esbuild, NOT tsc. They use bare import specifiers resolved by the HTML import map at runtime.
- Projectile TTL is in **wall-clock seconds** (via `performance.now()`), not simulation time. In headless tests, use very short TTL values (e.g., 0.001) since 60 physics steps execute in ~25ms real time.
- The `getActiveBondsCount()` JS-side tracking (`bondTable.length - removedBondIndices.size`) may lag behind the WASM solver's internal state after `applyFractureCommands`. Multiple resimulation passes help propagate fractures.

### CI

- GitHub Actions workflow at `.github/workflows/ci.yml`
- **Push** trigger: only runs on `feat/rapier-destruction` branch (production)
- **Pull request** trigger: runs on all PRs
- This prevents double CI runs when PRs are opened from feature branches
