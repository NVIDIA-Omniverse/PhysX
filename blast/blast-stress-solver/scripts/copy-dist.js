import { cp, mkdir } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const src = resolve(here, '../../js_stress_example/dist');
const dst = resolve(here, '../dist');

await mkdir(dst, { recursive: true });
await cp(resolve(src, 'stress_solver.wasm'), resolve(dst, 'stress_solver.wasm'));
await cp(resolve(src, 'stress_solver.mjs'), resolve(dst, 'stress_solver.mjs'));
await cp(resolve(src, 'stress_solver.cjs'), resolve(dst, 'stress_solver.cjs'));


