import { defineConfig } from 'tsup';

export default defineConfig({
  entry: { index: 'src/index.ts' },
  format: ['esm', 'cjs'],
  dts: true,
  splitting: false,
  sourcemap: true,
  minify: true,
  clean: true,
  target: 'es2020',
  external: ['./stress_solver.cjs', './stress_solver.mjs', './stress_solver.browser.mjs']
});


