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

## License

MIT


