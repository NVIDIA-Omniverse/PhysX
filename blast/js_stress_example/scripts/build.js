import { mkdirSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const projectRoot = resolve(__dirname, '..');
const blastRoot = resolve(projectRoot, '..');
const distDir = resolve(projectRoot, 'dist');
const jsFfiDir = resolve(projectRoot, 'ffi');

mkdirSync(distDir, { recursive: true });

const ffiDir = resolve(blastRoot, 'rust_stress_example/ffi');
const solverDir = resolve(blastRoot, 'source/shared/stress_solver');
const sharedDir = resolve(blastRoot, 'source/shared');
const sharedNsFoundationIncludeDir = resolve(sharedDir, 'NsFoundation', 'include');
const includeDir = resolve(blastRoot, 'include');
const includeGlobalsDir = resolve(includeDir, 'globals');
const includeSharedDir = resolve(includeDir, 'shared');
const includeLowLevelDir = resolve(includeDir, 'lowlevel');
const includeExtensionsDir = resolve(includeDir, 'extensions');
const includeStressExtDir = resolve(includeExtensionsDir, 'stress');
const includeAuthoringDir = resolve(includeExtensionsDir, 'authoring');
const includeAuthoringCommonDir = resolve(includeExtensionsDir, 'authoringCommon');
const includeAssetUtilsDir = resolve(includeExtensionsDir, 'assetutils');
const foundationDir = resolve(includeSharedDir, 'NvFoundation');
const sdkCommonDir = resolve(blastRoot, 'source/sdk/common');
const sdkGlobalsDir = resolve(blastRoot, 'source/sdk/globals');
const sdkLowLevelDir = resolve(blastRoot, 'source/sdk/lowlevel');
const sdkStressDir = resolve(blastRoot, 'source/sdk/extensions/stress');
const sdkAuthoringDir = resolve(blastRoot, 'source/sdk/extensions/authoring');
const sdkAuthoringCommonDir = resolve(blastRoot, 'source/sdk/extensions/authoringCommon');

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
  '_ext_stress_solver_create',
  '_ext_stress_solver_destroy',
  '_ext_stress_solver_set_settings',
  '_ext_stress_solver_graph_node_count',
  '_ext_stress_solver_bond_count',
  '_ext_stress_solver_reset',
  '_ext_stress_solver_add_force',
  '_ext_stress_solver_add_gravity',
  '_ext_stress_solver_update',
  '_ext_stress_solver_overstressed_bond_count',
  '_ext_stress_solver_fill_debug_render',
  '_ext_stress_solver_generate_fracture_commands',
  '_ext_stress_solver_actor_count',
  '_ext_stress_solver_collect_actors',
  '_ext_stress_solver_generate_fracture_commands_per_actor',
  '_ext_stress_solver_apply_fracture_commands',
  '_ext_stress_solver_get_excess_forces',
  '_ext_stress_solver_get_linear_error',
  '_ext_stress_solver_get_angular_error',
  '_ext_stress_solver_converged',
  '_ext_stress_sizeof_ext_node_desc',
  '_ext_stress_sizeof_ext_bond_desc',
  '_ext_stress_sizeof_ext_settings',
  '_ext_stress_sizeof_ext_debug_line',
  '_ext_stress_sizeof_ext_bond_fracture',
  '_ext_stress_sizeof_ext_fracture_commands',
  '_ext_stress_sizeof_actor',
  '_ext_stress_sizeof_actor_buffer',
  '_ext_stress_sizeof_ext_split_event',
  '_authoring_sizeof_ext_bond_desc',
  '_authoring_bonds_from_prefractured_triangles',
  '_authoring_free',
  '_malloc',
  '_free'
];

const enableAssertions = process.env.EMCC_ASSERTIONS !== '0';
const enableProfiling = process.env.EMCC_PROFILING === '1';

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
  'HEAPF64'
];

const commonArgs = [
  resolve(ffiDir, 'stress_bridge.cpp'),
  resolve(ffiDir, 'ext_stress_bridge.cpp'),
  resolve(jsFfiDir, 'authoring_bridge.cpp'),
  resolve(solverDir, 'stress.cpp'),
  resolve(sdkStressDir, 'NvBlastExtStressSolver.cpp'),
  resolve(sdkAuthoringDir, 'NvBlastExtAuthoring.cpp'),
  resolve(sdkAuthoringDir, 'NvBlastExtAuthoringBondGeneratorImpl.cpp'),
  resolve(sdkAuthoringDir, 'NvBlastExtTriangleProcessor.cpp'),
  resolve(sdkAuthoringDir, 'NvBlastExtApexSharedParts.cpp'),
  resolve(sdkCommonDir, 'NvBlastAssert.cpp'),
  resolve(sdkCommonDir, 'NvBlastAtomic.cpp'),
  resolve(sdkCommonDir, 'NvBlastTime.cpp'),
  resolve(sdkCommonDir, 'NvBlastTimers.cpp'),
  resolve(sdkGlobalsDir, 'NvBlastGlobals.cpp'),
  resolve(sdkGlobalsDir, 'NvBlastInternalProfiler.cpp'),
  resolve(sdkLowLevelDir, 'NvBlastActor.cpp'),
  resolve(sdkLowLevelDir, 'NvBlastActorSerializationBlock.cpp'),
  resolve(sdkLowLevelDir, 'NvBlastAsset.cpp'),
  resolve(sdkLowLevelDir, 'NvBlastAssetHelper.cpp'),
  resolve(sdkLowLevelDir, 'NvBlastFamily.cpp'),
  resolve(sdkLowLevelDir, 'NvBlastFamilyGraph.cpp'),
  '-I' + ffiDir,
  '-I' + solverDir,
  '-I' + sharedDir,
  '-I' + sharedNsFoundationIncludeDir,
  '-I' + includeDir,
  '-I' + includeGlobalsDir,
  '-I' + includeSharedDir,
  '-I' + includeLowLevelDir,
  '-I' + includeExtensionsDir,
  '-I' + includeStressExtDir,
  '-I' + includeAuthoringDir,
  '-I' + includeAuthoringCommonDir,
  '-I' + includeAssetUtilsDir,
  '-I' + foundationDir,
  '-I' + sdkCommonDir,
  '-I' + sdkGlobalsDir,
  '-I' + sdkLowLevelDir,
  '-I' + sdkStressDir,
  '-I' + sdkAuthoringDir,
  '-I' + sdkAuthoringCommonDir,
  '-DSTRESS_SOLVER_FORCE_SCALAR=1',
  '-DSTRESS_SOLVER_NO_SIMD=1',
  '-D__linux__=1',
  '-D__arm__=1',
  '-DCOMPILE_VECTOR_INTRINSICS=0',
  '-DNDEBUG=1',
  '-std=c++17',
  '-O3',
  '-sWASM=1',
  '-sMODULARIZE=1',
  '-sALLOW_MEMORY_GROWTH=1',
  `-sEXPORTED_FUNCTIONS=[${exportedFunctions.map((fn) => `"${fn}"`).join(',')}]`,
  `-sEXPORTED_RUNTIME_METHODS=[${exportedRuntimeMethods.map((name) => `"${name}"`).join(',')}]`
];

if (enableAssertions) {
  console.log('Building with assertions');
  commonArgs.push('-sASSERTIONS=1');
}

if (enableProfiling) {
  console.log('Building with profiling symbols');
  commonArgs.push('--profiling');
}

const builds = [
  {
    name: 'node-cjs',
    args: [
      ...commonArgs,
      '-sEXPORT_NAME=createStressModule',
      '-sENVIRONMENT=node',
      '-o',
      resolve(distDir, 'stress_solver.cjs')
    ]
  },
  {
    name: 'browser-esm',
    args: [
      ...commonArgs,
      '-sEXPORT_ES6=1',
      '-sENVIRONMENT=web,worker,node',
      '-sEXPORT_NAME=createStressModule',
      '-o',
      resolve(distDir, 'stress_solver.mjs')
    ]
  }
];

for (const build of builds) {
  const result = spawnSync('emcc', build.args, { stdio: 'inherit' });

  if (result.error) {
    console.error(`Failed ${build.name} build:`, result.error);
    process.exit(1);
  }

  if (result.status !== 0) {
    console.error(`Emscripten exited with code ${result.status} for ${build.name}`);
    process.exit(result.status ?? 1);
  }
}
