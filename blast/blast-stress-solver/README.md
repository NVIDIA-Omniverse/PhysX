# blast-stress-solver

TypeScript + WASM destruction toolkit built on the NVIDIA Blast stress solver.

It now ships in three layers:

- **`blast-stress-solver`** — low-level WASM bridge, solver types, bond authoring helpers.
- **`blast-stress-solver/rapier`** — high-level Rapier destruction runtime for chunks, splitting, damage, debris handling, projectiles, and profiling.
- **`blast-stress-solver/three`** — Three.js helpers for auto-bonding, chunk visuals, batched rendering, debug lines, and bundled scene integration.

The goal is simple: move the hard-won destruction code out of app code and into the package, so projects like Vibe City can get to "spawn destructible structure and start simulating" with much less custom glue.

## Install

Base package:

```bash
npm install blast-stress-solver
```

For Rapier + Three integrations:

```bash
npm install blast-stress-solver @dimforge/rapier3d-compat three
```

## Package layout

```ts
import { loadStressSolver } from 'blast-stress-solver';
import { buildDestructibleCore } from 'blast-stress-solver/rapier';
import { createDestructibleThreeBundle } from 'blast-stress-solver/three';
```

## Low-level solver usage

```ts
import { loadStressSolver } from 'blast-stress-solver';

const runtime = await loadStressSolver();
const solver = runtime.createExtSolver({
  nodes,
  bonds,
  settings: runtime.defaultExtSettings(),
});

solver.addGravity({ x: 0, y: -9.81, z: 0 });
solver.update();

if (solver.overstressedBondCount() > 0) {
  const fractureSets = solver.generateFractureCommandsPerActor();
  const splitEvents = solver.applyFractureCommands(fractureSets);
  console.log(splitEvents);
}
```

If an actor is rotated relative to world space, convert world gravity into that actor's local frame before calling `addActorGravity`:

```ts
const localGravity = worldGravity
  .clone()
  .applyQuaternion(actorWorldQuaternion.clone().invert());

solver.addActorGravity(actorIndex, localGravity);
```

## Author bonds from prefractured meshes

```ts
import { chunksFromBufferGeometries, loadStressSolver } from 'blast-stress-solver';

const chunkInputs = chunksFromBufferGeometries(geometries, (geometry, index) => ({
  isSupport: true,
  applyMatrix: meshes[index].matrixWorld,
  cloneGeometry: true,
  nonIndexed: true,
}));

const runtime = await loadStressSolver();
const bonds = runtime.createBondsFromTriangles(chunkInputs, {
  mode: 'average',
  maxSeparation: 0.01,
});
```

## Rapier quick start

`buildDestructibleCore` ports the Vibe City runtime into the package.

Features included:

- stress solver integration and fracture application
- body reuse and split migration planning
- contact-driven damage accumulation
- optional direct node damage
- debris collision modes, sleep tuning, damping, cleanup TTL
- projectile spawning and cleanup
- solver debug-line extraction
- optional profiling callbacks

```ts
import { buildDestructibleCore } from 'blast-stress-solver/rapier';

const core = await buildDestructibleCore({
  scenario,
  gravity: -9.81,
  materialScale: 1.25,
  debrisCollisionMode: 'noDebrisPairs',
  damage: {
    enabled: true,
    autoDetachOnDestroy: true,
    autoCleanupPhysics: true,
    contactDamageScale: 1.0,
    minImpulseThreshold: 50,
  },
});

core.step(1 / 60);
```

### Automatic node sizing

You no longer need to pass a custom `nodeSize` callback for common Vibe City scenarios.

The Rapier runtime now resolves chunk size in this order:

1. `scenario.parameters.fragmentSizes[nodeIndex]`
2. `scenario.spacing`
3. package fallback size

Override it only when you need custom collider sizing:

```ts
const core = await buildDestructibleCore({
  scenario,
  nodeSize: (nodeIndex, scenario) => ({ x: 0.4, y: 0.2, z: 0.4 }),
});
```

Or build a reusable resolver:

```ts
import { createScenarioNodeSizeResolver } from 'blast-stress-solver/rapier';

const nodeSize = createScenarioNodeSizeResolver({
  fallbackSize: { x: 0.5, y: 0.5, z: 0.32 },
  minimumComponent: 0.05,
});
```

### Solver settings overrides

The runtime now accepts direct overrides for the underlying Blast solver settings:

```ts
const core = await buildDestructibleCore({
  scenario,
  solverSettings: {
    maxSolverIterationsPerFrame: 48,
    graphReductionLevel: 0,
  },
});
```

## Three.js quick start

### Auto-bonding from fragment geometry

```ts
import { applyAutoBondingToScenario } from 'blast-stress-solver/three';

const bondedScenario = await applyAutoBondingToScenario(scenario, {
  mode: 'average',
  maxSeparation: 0.01,
});
```

### Build chunk visuals from a scenario

The Three helpers can now read `scenario.parameters.fragmentGeometries` directly.

```ts
import {
  buildBatchedChunkMeshFromScenario,
  buildChunkMeshesFromScenario,
} from 'blast-stress-solver/three';

const batched = buildBatchedChunkMeshFromScenario(core, scenario, {
  enableBVH: false,
  bvhMargin: 5,
});

scene.add(batched.batchedMesh);
```

### Bundle the whole visual layer

`createDestructibleThreeBundle` is the fastest way to get a Three scene wired up.

```ts
import { createDestructibleThreeBundle } from 'blast-stress-solver/three';

const visuals = createDestructibleThreeBundle({
  core,
  scenario,
  root: group,
  useBatchedMesh: true,
  batchedMeshOptions: {
    enableBVH: false,
    bvhMargin: 5,
  },
  includeDebugLines: true,
});

function tick(dt: number, showDebugLines: boolean) {
  core.step(dt);
  visuals.update({
    debug: showDebugLines,
    updateBVH: false,
    updateProjectiles: true,
  });
}
```

On cleanup:

```ts
visuals.dispose();
core.dispose();
```

## Vibe City migration

See:

- [`docs/vibe-city-upgrade.md`](./docs/vibe-city-upgrade.md)
- [`examples/vibe-city-upgrade/destructible-scene-snippet.tsx`](./examples/vibe-city-upgrade/destructible-scene-snippet.tsx)

The main migration path is:

- move `src/lib/stress/core/destructible-core.ts` usage to `blast-stress-solver/rapier`
- move `src/lib/stress/core/autoBonding.ts` usage to `blast-stress-solver/three`
- move `src/lib/stress/three/destructible-adapter.ts` usage to `blast-stress-solver/three`
- replace manual geometry branching with `build*FromScenario(...)`
- replace manual scene glue with `createDestructibleThreeBundle(...)`

## Next.js notes

- Server imports use the Node export automatically.
- Client components should create the runtime inside an effect or other client-only path.
- The WASM is referenced with `new URL(..., import.meta.url)` so modern bundlers copy it automatically.

Custom WASM hosting:

```ts
import { loadStressSolver } from 'blast-stress-solver';

await loadStressSolver({
  module: {
    locateFile: (path) =>
      path.endsWith('.wasm') ? '/wasm/blast/stress_solver.wasm' : path,
  },
});
```

## Local development

### Full package build

```bash
npm run build
```

### TypeScript-only wrapper iteration

If you are iterating on the TypeScript package surface and already have runtime artifacts available elsewhere, you can skip the C++/Emscripten prebuild step:

```bash
BLAST_STRESS_SOLVER_SKIP_WASM_BUILD=1 npm run build:ts
```

The final published package still requires:

- `js_stress_example/dist/stress_solver.wasm`
- `js_stress_example/dist/stress_solver.cjs`
- `js_stress_example/dist/stress_solver.mjs`

## Tests

Run the full suite with:

```bash
npm test
```

Additional pure TypeScript tests were added for:

- scenario node-size resolution
- split migration planning

## License

MIT
