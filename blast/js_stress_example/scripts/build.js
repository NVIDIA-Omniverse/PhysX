import { mkdirSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const projectRoot = resolve(__dirname, '..');
const blastRoot = resolve(projectRoot, '..');
const distDir = resolve(projectRoot, 'dist');

mkdirSync(distDir, { recursive: true });

const ffiDir = resolve(blastRoot, 'rust_stress_example/ffi');
const solverDir = resolve(blastRoot, 'source/shared/stress_solver');
const sharedDir = resolve(blastRoot, 'source/shared');
const includeDir = resolve(blastRoot, 'include');
const includeGlobalsDir = resolve(includeDir, 'globals');
const includeSharedDir = resolve(includeDir, 'shared');
const foundationDir = resolve(includeSharedDir, 'NvFoundation');

const exportedFunctions = [
  '_stress_processor_create',
  '_stress_processor_destroy',
  '_stress_processor_node_count',
  '_stress_processor_bond_count',
  '_stress_processor_solve',
  '_stress_processor_remove_bond',
  '_stress_processor_get_node_desc',
  '_stress_processor_get_bond_desc',
  '_stress_processor_using_simd',
  '_stress_sizeof_stress_vec3',
  '_stress_sizeof_node_desc',
  '_stress_sizeof_bond_desc',
  '_stress_sizeof_velocity',
  '_stress_sizeof_impulse',
  '_stress_sizeof_data_params',
  '_stress_sizeof_solver_params',
  '_stress_sizeof_error_sq',
  '_malloc',
  '_free'
];

const exportedRuntimeMethods = [
  'cwrap',
  'ccall',
  'getValue',
  'setValue',
  'UTF8ToString',
  'stringToUTF8',
  'lengthBytesUTF8',
  'HEAP8',
  'HEAPU8',
  'HEAP16',
  'HEAPU16',
  'HEAP32',
  'HEAPU32',
  'HEAPF32',
  'HEAPF64',
  'stackSave',
  'stackRestore',
  'stackAlloc'
];

const args = [
  resolve(ffiDir, 'stress_bridge.cpp'),
  resolve(solverDir, 'stress.cpp'),
  '-I' + ffiDir,
  '-I' + solverDir,
  '-I' + sharedDir,
  '-I' + includeDir,
  '-I' + includeGlobalsDir,
  '-I' + includeSharedDir,
  '-I' + foundationDir,
  '-DSTRESS_SOLVER_FORCE_SCALAR=1',
  '-DSTRESS_SOLVER_NO_SIMD=1',
  '-D__linux__=1',
  '-D__x86_64__=1',
  '-DNDEBUG=1',
  '-std=c++17',
  '-O3',
  '-sWASM=1',
  '-sMODULARIZE=1',
  '-sEXPORT_NAME=createStressModule',
  '-sALLOW_MEMORY_GROWTH=1',
  '-sENVIRONMENT=node',
  `-sEXPORTED_FUNCTIONS=[${exportedFunctions.map((fn) => `"${fn}"`).join(',')}]`,
  `-sEXPORTED_RUNTIME_METHODS=[${exportedRuntimeMethods.map((name) => `"${name}"`).join(',')}]`,
  '-o',
  resolve(distDir, 'stress_solver.cjs')
];

const result = spawnSync('emcc', args, { stdio: 'inherit' });

if (result.error) {
  console.error(result.error);
  process.exit(1);
}

if (result.status !== 0) {
  process.exit(result.status ?? 1);
}
