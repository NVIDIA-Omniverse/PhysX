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
4. `npm start` (root) — starts static file server on port 8000, serves demos and vendor aliases for three.js/rapier

### Running tests

- `cd blast/blast-stress-solver && npm test` — **30 tests, all passing** (authoring + gravity tests via vitest)
- `cd blast/js_stress_example && npm run test:split` — split/fracture integration tests (vitest); 9/10 pass, `organicSplit.spec.ts` has a stale expectation
- `cd blast/js_stress_example && npm run test:bridge-ui` — Playwright browser tests (requires `npx playwright install chromium` first)

### Browser demos

After running `npm start` at the root, the **primary demo** is:

- **`http://localhost:8000/blast/js_stress_example/bridge-split-demo.html`** — The most up-to-date and fully featured demo. Destructible bridge with real-time config panel, working reset, projectile spawning, and full fracture into independent physics bodies. Requires the `npx tsc` build step (step 2 above). **Always use this demo for testing and development.**

Other demos (secondary/legacy):
- `bridge-ext.html` — Older two-phase fracture demo (also requires `npx tsc`); less polished than `bridge-split-demo.html`
- `bridge-demo.html` — Legacy demo with stress coloring only; fracture is disabled (`simulation.js:117` early return)
- `/demos/three-rapier.html` — Basic Three.js + Rapier physics demo (no Blast)

### Gotchas

- The `blast/blast-stress-solver` build has benign `import.meta` CJS warnings from tsup — these do not affect functionality.
- TypeScript strict checking (`tsc --noEmit`) in `blast/js_stress_example` shows pre-existing type errors; `noEmitOnError: false` in tsconfig ensures files are still emitted.
- The WASM build takes ~20 seconds per run (two targets: node-cjs + browser-esm).
- `npm install --ignore-scripts` is used for `blast/blast-stress-solver` during dependency refresh to avoid triggering a full rebuild on install.
- `bridge-demo.html` uses an older approach where `splitChunk` has a FIXME early return at `simulation.js:117` — fracture is intentionally disabled there. Use `bridge-split-demo.html` or `bridge-ext.html` for working fracture demos.
