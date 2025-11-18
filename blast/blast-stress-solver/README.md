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

If you already have a fractured mesh (triangle soup per chunk), you can let the WASM bridge generate Blast-style bonds for you:

```ts
import { loadStressSolver } from 'blast-stress-solver';

const rt = await loadStressSolver();
const chunks = [
  {
    triangles: Float32Array.from([
      // chunk 0 triangles (xyz per vertex, 9 floats per triangle)
      -0.5, -0.5, -0.5, 0.5, -0.5, -0.5, 0.5, 0.5, -0.5,
      // ...
    ])
  },
  {
    triangles: Float32Array.from([/* chunk 1 triangles */]),
    isSupport: true
  }
];

const exactBonds = rt.createBondsFromTriangles(chunks); // defaults to mode: 'exact'
const tolerantBonds = rt.createBondsFromTriangles(chunks, { mode: 'average', maxSeparation: 0.02 });

const solver = rt.createExtSolver({ nodes, bonds: tolerantBonds, settings: rt.defaultExtSettings() });
```

- Every chunk entry is a flat array of local-space vertices (`[x0, y0, z0, x1, y1, z1, ...]`); each triangle consumes 9 floats.
- Set `isSupport` on a chunk to opt-in/out of the support graph (default: `true`).
- `mode: 'exact'` (default) computes the precise shared surface from triangle geometry.
- `mode: 'average'` approximates the bond interface using convex hulls and a `maxSeparation` tolerance so that slightly separated chunks (due to noise/gaps) can still be connected. Always supply a positive `maxSeparation` when using this mode.
- The generated bonds already contain centroid, normal, area, and chunk indices, so they can be used directly when creating your solver or Blast asset.

### Three.js example

If your chunks are `THREE.BufferGeometry` instances (often pre-fractured offline), convert them to triangle soups like so:

```ts
import * as THREE from 'three';
import {
  loadStressSolver,
  chunksFromBufferGeometries,
  chunkFromBufferGeometry
} from 'blast-stress-solver';

const chunkGeometries: THREE.BufferGeometry[] = /* your pre-fractured chunks */;

// Option A: convert the full collection in one call
const chunks = chunksFromBufferGeometries(chunkGeometries, (geometry, index) => ({
  isSupport: index > 0, // simple predicate
  applyMatrix: worldTransforms[index] // optional Matrix4 per chunk
}));

// Option B: convert ad hoc
const extraChunk = chunkFromBufferGeometry(chunkGeometries[0], {
  isSupport: true,
  nonIndexed: true
});

const rt = await loadStressSolver();
const bonds = rt.createBondsFromTriangles([...chunks, extraChunk], { mode: 'exact' });
```

- By default, `chunkFromBufferGeometry` converts indexed geometry to non-indexed triangle lists (`nonIndexed: true`), which is what the bond generator expects. Set `nonIndexed: false` if your data is already in triangle-list form.
- `chunkFromBufferGeometry` clones the geometry by default (`cloneGeometry: true`) so your originals remain untouched. Pass `cloneGeometry: false` if you prefer to mutate in-place.
- Supply any `Matrix4` (e.g., `mesh.matrixWorld`) via `applyMatrix` to bake transforms before sampling the triangle data.
- You can precompute chunk centroids/masses separately and feed them into the solver nodes.

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


