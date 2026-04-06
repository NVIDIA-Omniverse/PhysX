# AGENTS.md

## Cursor Cloud specific instructions

### Project overview

This is the NVIDIA PhysX monorepo containing PhysX, Blast, and Flow SDKs. The active development focus is on the **Blast Stress Solver JavaScript/WASM library** (`blast/blast-stress-solver`) and Three.js + Rapier browser demos (`blast/js_stress_example`).

### Prerequisites

- **Emscripten SDK** must be installed at `/opt/emsdk` and sourced (`source /opt/emsdk/emsdk_env.sh`) before building the WASM artifacts. The `~/.bashrc` sources it automatically.
- **Node.js** (v22+) is available via nvm.

### Build chain (dependency order)

1. `cd blast/js_stress_example && npm run build` — compiles C++ stress solver to WASM via `emcc`, outputs `dist/stress_solver.{cjs,mjs,wasm}`
2. `cd blast/blast-stress-solver && npm run build` — runs step 1 as `prebuild`, then bundles TypeScript with `tsup`, copies WASM to `dist/`
3. `npm start` (root) — starts static file server on port 8000, serves demos and vendor aliases for three.js/rapier

### Running tests

- `cd blast/blast-stress-solver && npm test` — **30 tests, all passing** (authoring + gravity tests via vitest)
- `cd blast/js_stress_example && npm run test:split` — split/fracture integration tests (vitest); 9/10 pass, `organicSplit.spec.ts` has a stale expectation
- `cd blast/js_stress_example && npm run test:bridge-ui` — Playwright browser tests (requires `npx playwright install chromium` first)

### Browser demos

After running `npm start` at the root, navigate to:
- `http://localhost:8000/blast/js_stress_example/bridge-demo.html` — Destructible bridge demo (Three.js + Rapier + Blast stress solver)
- `http://localhost:8000/blast/js_stress_example/bridge-ext.html` — Extended bridge demo
- `http://localhost:8000/demos/three-rapier.html` — Basic Three.js + Rapier physics demo

### Gotchas

- The `blast/blast-stress-solver` build has benign `import.meta` CJS warnings from tsup — these do not affect functionality.
- TypeScript strict checking (`tsc --noEmit`) in `blast/js_stress_example` shows pre-existing type errors in browser-side code; these don't block WASM builds or tests.
- The WASM build takes ~20 seconds per run (two targets: node-cjs + browser-esm).
- `npm install --ignore-scripts` is used for `blast/blast-stress-solver` during dependency refresh to avoid triggering a full rebuild on install.
