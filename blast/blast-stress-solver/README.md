# blast-stress-solver

Dual-format (ESM + CJS) JavaScript/WASM package exposing the NVIDIA Blast Stress Solver bridge with first-class TypeScript types.

## Install

```bash
npm install blast-stress-solver
```

## Usage (browser)

```ts
import { loadStressSolver } from 'blast-stress-solver';

const rt = await loadStressSolver();
const solver = rt.createExtSolver({ nodes, bonds, settings: rt.defaultExtSettings() });
solver.addGravity({ x: 0, y: -9.81, z: 0 });
const [{ actorIndex }] = solver.actors();
solver.addActorGravity(actorIndex, { x: 0, y: -9.81, z: 0 }); // per-actor helper
solver.update();
const cmds = solver.generateFractureCommands();
```

If an actor is rotated relative to world space (e.g., a column rotated 90° so its local “down” axis points sideways), convert the world gravity vector using the actor’s rotation before calling `addActorGravity`:

```ts
const localGravity = worldGravity.clone().applyQuaternion(actorWorldQuaternion.clone().invert());
solver.addActorGravity(actorIndex, localGravity);
```

This mirrors what the physics body experiences and lets the stress solver distinguish between compression along the column versus shear when it lies on its side.

## Usage (Node / SSR)

```ts
import { loadStressSolver } from 'blast-stress-solver';
const rt = await loadStressSolver();
```

Works in Vite/Webpack/Next.js without custom config. The WASM is referenced via `new URL(..., import.meta.url)` so bundlers copy it automatically. To host the WASM from a custom path, override locateFile:

```ts
await loadStressSolver({
  module: {
    locateFile: (p) => (p.endsWith('.wasm') ? '/wasm/blast/stress_solver.wasm' : p)
  }
});
```

## Next.js notes

- Server (SSR) imports use the Node export automatically.
- Client: import inside an effect or use dynamic import to avoid SSR hydration issues.

Client-side example:

```ts
import { useEffect, useState } from 'react';
import { loadStressSolver } from 'blast-stress-solver';

export default function Component() {
  const [ready, setReady] = useState(false);
  useEffect(() => {
    (async () => {
      const rt = await loadStressSolver();
      // ... create solver, etc.
      setReady(true);
    })();
  }, []);
  return ready ? <div>Ready</div> : null;
}
```

Or dynamically import on the client only:

```ts
import dynamic from 'next/dynamic';
const ClientOnly = dynamic(() => import('./ClientComponent'), { ssr: false });
```

## Types

This package ships TypeScript types generated from source; see `StressRuntime`, `ExtStressSolver`, and helpers in the API.

## Author bonds from prefractured meshes

Generate Blast-style bonds from prefractured triangle meshes:

```ts
import { loadStressSolver } from 'blast-stress-solver';

const rt = await loadStressSolver();

// Exact mode (default) for geometry with shared faces
const bonds = rt.createBondsFromTriangles(chunks, { mode: 'exact' });

// Average mode for geometry with small gaps
const bonds = rt.createBondsFromTriangles(chunks, { mode: 'average', maxSeparation: 0.01 });
```

### Three.js helpers

```ts
import { chunksFromBufferGeometries, loadStressSolver } from 'blast-stress-solver';

const chunks = chunksFromBufferGeometries(geometries, (geometry, index) => ({
  isSupport: true,
  applyMatrix: mesh.matrixWorld // bake world transforms
}));

const rt = await loadStressSolver();
const bonds = rt.createBondsFromTriangles(chunks);
```

> **Tip:** If you get 0 bonds, try `{ mode: 'average', maxSeparation: 0.01 }` to handle floating-point gaps.
> See JSDoc on `BondingConfig` and `createBondsFromTriangles` for detailed troubleshooting.

## License
## Tests

Run the full build + test suite (includes the WASM authoring helpers) with:

```bash
npm install
npm test
```

## Implementation notes

- The WASM build of Blast used by this package is compiled with SIMD intrinsics disabled (`COMPILE_VECTOR_INTRINSICS=0`), using the scalar math path for maximum portability across browsers.
- Native C++ builds may use SSE/NEON for higher performance; future releases may enable WebAssembly SIMD behind the same JS API.
- Average-mode bonding relies on a lightweight convex hull builder compiled into the WASM module. When using `'average'`, make sure to provide a reasonable `maxSeparation` so the helper knows how much gap/noise to tolerate.


MIT


