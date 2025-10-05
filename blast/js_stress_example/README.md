# Blast Stress Solver JavaScript Demo

This example shows how to compile the Blast stress solver extension to WebAssembly
with Emscripten and drive it from Node.js. The scenario matches the Rust demo:
three chunks connected by three bonds, simulated across several frames while we
monitor the resulting bond stresses and remove overstressed connections.

## Prerequisites

- Emscripten (`emcc`) available on the `PATH`.
- Node.js 18+ (installed automatically when the Emscripten Debian package is
  used).

## Build and run

```bash
cd blast/js_stress_example
npm run demo
```

The build step compiles `stress_bridge.cpp` together with the Blast stress solver
implementation (`stress.cpp`) to `dist/stress_solver.{cjs,wasm}`. The demo script
then loads the generated module, feeds the same frame inputs as the Rust example,
computes bond stresses, and logs which bonds fail under the configured limits.

To exercise the high-level `ExtStressSolver` wrapper from Node.js, run the
dedicated example:

```bash
cd blast/js_stress_example
npm run demo:ext
```

This script applies gravity and a downward impulse on the top node, steps the
solver until it converges, and prints debug-render line samples generated via
`fillDebugRender`.

Additional `ExtStressSolver` demos are available:

```bash
cd blast/js_stress_example
npm run demo:ext-cube      # Projectile vs. fractured cube
npm run demo:ext-bridge    # Heavy vehicles collapsing a bridge deck
```

- `ext-cube-projectile.js` procedurally fractures a cube into eight pieces,
  applies gravity plus a high-speed projectile impulse, and reports the bonds
  predicted to fail along the impacted face. Debug-render lines are sorted by
  magnitude so that the most stressed bonds are easy to inspect.
- `ext-bridge-cars.js` builds a beam bridge with layered deck fragments and
  suspension piers. It sweeps a heavy "car" load across successive deck
  segments, printing per-span solver errors, overstressed bond counts, and the
  hottest debug lines. Once the car load creates an overstressed bond the demo
  flags the deck as failed, mirroring how a game would generate fracture
  commands from the solver output.

## ExtStressSolver bindings

The build also bundles the Blast `ExtStressSolver` extension.  You can create and
drive the high-level solver from JavaScript by calling
`runtime.createExtSolver({ nodes, bonds, settings })`.  Each node entry should
include a centroid, mass, and volume, while each bond entry describes the
centroid, surface normal, area, and the connected node indices.  Once created,
the solver exposes helpers for applying forces or gravity, advancing the solver,
querying overstressed bonds, and fetching `fillDebugRender` output for visual
debugging.

```js
const runtime = await loadStressSolver();
const solver = runtime.createExtSolver({
  nodes: [
    { centroid: vec3(-1, 0, 0), mass: 25, volume: 2.5 },
    { centroid: vec3(1, 0, 0), mass: 25, volume: 2.5 },
    { centroid: vec3(0, 1.5, 0), mass: 15, volume: 1.5 }
  ],
  bonds: [
    { centroid: vec3(-0.5, 0.75, 0), normal: vec3(-0.6, 0.8, 0), area: 0.6, node0: 0, node1: 2 },
    { centroid: vec3(0.5, 0.75, 0), normal: vec3(0.6, 0.8, 0), area: 0.6, node0: 1, node1: 2 }
  ],
  settings: runtime.defaultExtSettings()
});

solver.addForce(2, vec3(), vec3(0, -50, 0));
solver.update();
const debugLines = solver.fillDebugRender({ mode: runtime.ExtDebugMode.Max });
solver.destroy();
```

All helper enums (`ExtForceMode`, `ExtDebugMode`) and default settings are
exposed on the runtime object returned by `loadStressSolver`.
