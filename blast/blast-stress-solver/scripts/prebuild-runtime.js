import { access } from 'node:fs/promises';
import { constants } from 'node:fs';
import { spawn } from 'node:child_process';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const runtimeDist = resolve(here, '../../js_stress_example/dist');
const requiredFiles = [
  'stress_solver.wasm',
  'stress_solver.cjs',
  'stress_solver.mjs'
];

async function hasRuntimeArtifacts() {
  try {
    await Promise.all(
      requiredFiles.map((name) =>
        access(resolve(runtimeDist, name), constants.R_OK)
      )
    );
    return true;
  } catch {
    return false;
  }
}

function runExampleBuild() {
  return new Promise((resolvePromise, rejectPromise) => {
    const child = spawn('npm', ['--prefix', '../js_stress_example', 'run', 'build'], {
      cwd: resolve(here, '..'),
      stdio: 'inherit',
      shell: process.platform === 'win32'
    });

    child.on('error', rejectPromise);
    child.on('exit', (code) => {
      if (code === 0) {
        resolvePromise(undefined);
      } else {
        rejectPromise(new Error(`js_stress_example build failed with exit code ${code ?? 'unknown'}`));
      }
    });
  });
}

if (!(await hasRuntimeArtifacts())) {
  if (process.env.BLAST_STRESS_SOLVER_SKIP_WASM_BUILD === '1') {
    console.warn(
      '[blast-stress-solver] Skipping WASM runtime build because BLAST_STRESS_SOLVER_SKIP_WASM_BUILD=1. ' +
        'The final package build will still require stress_solver.{wasm,cjs,mjs} under js_stress_example/dist.'
    );
  } else {
    try {
      await runExampleBuild();
    } catch (error) {
      console.error(
        '[blast-stress-solver] Failed to build the Blast runtime. ' +
          'Install the Emscripten toolchain used by js_stress_example, or set BLAST_STRESS_SOLVER_SKIP_WASM_BUILD=1 ' +
          'when iterating on TypeScript wrappers only.'
      );
      throw error;
    }
  }
}
