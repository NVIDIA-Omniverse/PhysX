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
solver.update();
const cmds = solver.generateFractureCommands();
```

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

const bonds = rt.createBondsFromTriangles(chunks, { mode: 'average', maxSeparation: 0.02 });

const solver = rt.createExtSolver({ nodes, bonds, settings: rt.defaultExtSettings() });
```

- Every chunk entry is a flat array of local-space vertices (`[x0, y0, z0, x1, y1, z1, ...]`); each triangle consumes 9 floats.
- Set `isSupport` on a chunk to opt-in/out of the support graph (default: `true`).
- `mode: 'exact'` (default) computes the precise shared surface from triangle geometry. `'average'` projects against a mid-plane and uses `maxSeparation` to allow small gaps/noise.
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

- This example assumes every `BufferGeometry` already uses Triangle lists (most fracture tools export that). If yours uses indexed geometry, call `geometry.toNonIndexed()` first.
- `chunkFromBufferGeometry` clones the geometry by default so your originals remain untouched. Pass `cloneGeometry: false` if you prefer to mutate in-place.
- Supply any `Matrix4` (e.g., `mesh.matrixWorld`) via `applyMatrix` to bake transforms before sampling the triangle data.
- You can precompute chunk centroids/masses separately and feed them into the solver nodes.

## License
## Tests

Run the full build + test suite (includes the WASM authoring helpers) with:

```bash
npm install
npm test
```


MIT


