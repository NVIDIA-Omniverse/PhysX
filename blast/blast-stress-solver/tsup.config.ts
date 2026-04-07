import { defineConfig } from 'tsup';

export default defineConfig({
  entry: {
    index: 'src/index.ts',
    rapier: 'src/rapier.ts',
    three: 'src/three.ts',
  },
  format: ['esm', 'cjs'],
  dts: true,
  splitting: false,
  sourcemap: true,
  minify: true,
  clean: true,
  target: 'es2020',
  external: [
    './stress_solver.cjs',
    './stress_solver.mjs',
    './stress_solver.browser.mjs',
    '@dimforge/rapier3d-compat',
    'three',
  ],
});
