import { existsSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import { createRequire } from 'node:module';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const root = resolve(here, '..');

const runtimeArtifacts = [
  resolve(root, 'blast/js_stress_example/dist/stress_solver.wasm'),
  resolve(root, 'blast/js_stress_example/dist/stress_solver.cjs'),
  resolve(root, 'blast/js_stress_example/dist/stress_solver.mjs')
];

const failures = [];

function requireFrom(packageDir, specifier) {
  const require = createRequire(resolve(packageDir, 'package.json'));
  return require(specifier);
}

function checkRollup() {
  const packageDir = resolve(root, 'blast/blast-stress-solver');
  try {
    requireFrom(packageDir, 'rollup');
  } catch (error) {
    failures.push({
      message: 'blast-stress-solver has an invalid or stale native Rollup install for this platform.',
      fixCommands: [
        '  rm -rf blast/blast-stress-solver/node_modules',
        '  npm --prefix blast/blast-stress-solver install --ignore-scripts'
      ]
    });
  }
}

async function checkEsbuild() {
  const packageDir = resolve(root, 'blast/js_stress_example');
  try {
    const esbuild = requireFrom(packageDir, 'esbuild');
    await esbuild.build({
      stdin: {
        contents: 'console.log(1)',
        resolveDir: packageDir,
        sourcefile: 'preflight.js'
      },
      write: false,
      format: 'esm'
    });
  } catch (error) {
    failures.push({
      message: 'blast/js_stress_example has an invalid or stale native esbuild install for this platform.',
      fixCommands: [
        '  rm -rf blast/js_stress_example/node_modules',
        '  npm --prefix blast/js_stress_example install'
      ]
    });
  }
}

function checkEmscripten() {
  const hasRuntime = runtimeArtifacts.every((file) => existsSync(file));
  if (hasRuntime) {
    return;
  }

  const result = spawnSync('emcc', ['--version'], { stdio: 'ignore' });
  if (result.status !== 0) {
    failures.push({
      message: 'Emscripten is not available on PATH and the WASM runtime artifacts are missing.',
      fixCommands: [
        '  source /opt/emsdk/emsdk_env.sh'
      ]
    });
  }
}

checkRollup();
await checkEsbuild();
checkEmscripten();

if (failures.length > 0) {
  console.error('');
  for (const failure of failures) {
    console.error('[demo-preflight] ' + failure.message);
    if (failure.fixCommands.length > 0) {
      console.error('Suggested fix:');
      for (const command of failure.fixCommands) {
        console.error(command);
      }
    }
    console.error('');
  }

  console.error('Then rerun:');
  console.error('  npm start');
  process.exit(1);
}
