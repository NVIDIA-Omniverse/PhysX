# Upgrading Vibe City to the packaged blast-stress-solver runtime

This package now absorbs the main destruction helpers that previously lived in Vibe City.

## What moved

| Old Vibe City file | New package API |
| --- | --- |
| `src/lib/stress/core/destructible-core.ts` | `blast-stress-solver/rapier` |
| `src/lib/stress/core/autoBonding.ts` | `blast-stress-solver/three` |
| `src/lib/stress/three/destructible-adapter.ts` | `blast-stress-solver/three` |
| `src/lib/stress/three/solver-debug-lines.ts` | `blast-stress-solver/three` |

## Biggest simplifications

### 1. No more mandatory custom `nodeSize` callback

`buildDestructibleCore(...)` now reads:

- `scenario.parameters.fragmentSizes`
- `scenario.spacing`

So the common Vibe City callback can be deleted unless you truly need custom collider sizing.

### 2. No more manual geometry branching

Instead of checking `scenario.parameters.fragmentGeometries` yourself, use:

- `buildChunkMeshesFromScenario(...)`
- `buildBatchedChunkMeshFromScenario(...)`

### 3. No more hand-wiring the common visual loop

Use `createDestructibleThreeBundle(...)` to manage:

- chunk mesh creation
- batched mesh creation
- projectile mesh syncing
- solver debug lines
- cleanup

## Before

```ts
const core = await buildDestructibleCore({
  scenario,
  nodeSize: (index, scen) => {
    const sizes = scen.parameters?.fragmentSizes as Array<{ x: number; y: number; z: number }> | undefined;
    return sizes?.[index] ?? scen.spacing ?? { x: 0.5, y: 0.5, z: 0.32 };
  },
  gravity,
  materialScale,
  damage: {
    enabled: damageEnabled,
    autoDetachOnDestroy: true,
    autoCleanupPhysics: true,
  },
});

const params = scenario.parameters as { fragmentGeometries?: THREE.BufferGeometry[] } | undefined;

const visuals = useBatchedMesh
  ? params?.fragmentGeometries?.length
    ? buildBatchedChunkMeshFromGeometries(core, params.fragmentGeometries, { enableBVH: false })
    : buildBatchedChunkMesh(core, { enableBVH: false })
  : params?.fragmentGeometries?.length
    ? buildChunkMeshesFromGeometries(core, params.fragmentGeometries)
    : buildChunkMeshes(core);

const debugHelper = new SolverDebugLinesHelper();
scene.add(debugHelper.object);
```

## After

```ts
import { buildDestructibleCore } from 'blast-stress-solver/rapier';
import {
  applyAutoBondingToScenario,
  createDestructibleThreeBundle,
} from 'blast-stress-solver/three';

let scenario = await builder(params);
if (autoBondingEnabled) {
  scenario = await applyAutoBondingToScenario(scenario);
}

const core = await buildDestructibleCore({
  scenario,
  gravity,
  materialScale,
  damage: {
    enabled: damageEnabled,
    autoDetachOnDestroy: true,
    autoCleanupPhysics: true,
  },
});

const visuals = createDestructibleThreeBundle({
  core,
  scenario,
  root: group,
  useBatchedMesh,
  batchedMeshOptions: {
    enableBVH: false,
    bvhMargin: 5,
  },
  includeDebugLines: true,
});
```

Frame loop:

```ts
core.step(dt);
visuals.update({
  debug: showStressDebug,
  updateBVH: false,
  updateProjectiles: true,
});
```

Cleanup:

```ts
visuals.dispose();
core.dispose();
```

## Recommended Vibe City import replacements

```ts
// Before
import { buildDestructibleCore } from '@/lib/stress/core/destructible-core';
import { applyAutoBondingToScenario } from '@/lib/stress/core/autoBonding';
import { buildBatchedChunkMeshFromGeometries } from '@/lib/stress/three/destructible-adapter';

// After
import { buildDestructibleCore } from 'blast-stress-solver/rapier';
import {
  applyAutoBondingToScenario,
  buildBatchedChunkMeshFromScenario,
  createDestructibleThreeBundle,
} from 'blast-stress-solver/three';
```

## Suggested next cleanup inside Vibe City

1. Delete local stress core and Three adapter forks after migration.
2. Keep only Vibe-specific scenario builders and UI.
3. Push future destruction features into the package first, then consume them from Vibe City.
