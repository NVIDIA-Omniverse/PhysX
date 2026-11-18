#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Link official physics population and optionally OvPhysX into native OVStage WASM.

Reuses a completed native OVStage WASM Makefiles build without changing it.
The runtime option links the genuine CPU simulation implementation.
"""
import argparse
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
PHYSX = HERE.parents[3]
RUNTIME_GROUPS = ('common', 'parse', 'ovstage', 'foundation', 'cuda_shim', 'cooking', 'convex',
                  'runtime', 'api', 'sidecar', 'bootstrap', 'closure', 'platform')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError(f'Expected exactly one bridge anchor: {old[:70]}')
    return text.replace(old, new)


def exception_mode_from_flags(flags, label):
    if not isinstance(flags, list) or any(not isinstance(flag, str) for flag in flags):
        raise ValueError(f'{label} must provide a list of compiler flags')
    javascript_eh, native_eh = '-fexceptions' in flags, '-fwasm-exceptions' in flags
    if javascript_eh == native_eh or '-fno-exceptions' in flags:
        raise ValueError(f'{label} must select exactly one enabled exception ABI')
    settings = {flag.split('=', 1)[1] for flag in flags if flag.startswith('-sWASM_LEGACY_EXCEPTIONS=')}
    if len(settings) > 1 or settings - {'0', '1'}:
        raise ValueError(f'{label} has conflicting or invalid WASM_LEGACY_EXCEPTIONS settings')
    if javascript_eh:
        if settings:
            raise ValueError(f'{label} selects a native exception setting with JavaScript exceptions')
        return 'javascript'
    # Pinned Emscripten 5.0.7 defaults native exceptions to its legacy encoding.
    return 'wasm' if settings == {'0'} else 'wasm-legacy'


def validate_runtime_exception_abi(stage_mode, receipt):
    if not isinstance(receipt, dict):
        raise ValueError('OvPhysX runtime build receipt must be an object')
    runtime_mode = exception_mode_from_flags(receipt.get('flags'), 'OvPhysX runtime build receipt')
    if runtime_mode != stage_mode:
        raise ValueError(f'OvPhysX runtime exception ABI {runtime_mode} does not match OVStage {stage_mode}; '
                         'rebuild every runtime component in a separate directory with the matching stage build')
    return runtime_mode


def validate_runtime_pool_abi(stage_bits, receipt):
    if not isinstance(receipt, dict):
        raise ValueError('Runtime pool ABI receipt must be an object')
    runtime_bits = receipt.get('sdfPathPoolRegionBits', 8)
    if runtime_bits not in (8, 16) or runtime_bits != stage_bits:
        raise ValueError(f'OvPhysX runtime Sdf path pool ABI {runtime_bits} does not match OVStage {stage_bits}; '
                         'rebuild every USD-header consumer')


def validate_runtime_object_abis(directory, exception_mode, pool_bits, sdk_math):
    if sdk_math not in ('scalar', 'sse2'):
        raise ValueError(f'Unknown runtime SDK math ABI: {sdk_math}')
    inspected = 0
    for path in sorted((Path(directory) / 'runtime-objects').glob('*.json')):
        value = json.loads(path.read_text())
        if not isinstance(value, dict) or 'command' not in value or 'object' not in value:
            continue
        if value.get('exitCode') != 0 or not isinstance(value['object'], dict):
            raise ValueError(f'Failed runtime object receipt: {path}')
        flags = value['command']
        validate_runtime_exception_abi(exception_mode, {'flags': flags})
        validate_runtime_pool_abi(pool_bits, value)
        scalar = any(flag.startswith('-DPX_SIMD_DISABLED') for flag in flags)
        vector = '-msse2' in flags
        if (sdk_math == 'scalar' and (not scalar or vector)) or (sdk_math == 'sse2' and (scalar or not vector)):
            raise ValueError(f'Runtime object SDK math ABI differs from {sdk_math}: {path}')
        identity = value['object']
        obj = path.parent / Path(identity['path']).name
        if not obj.is_file() or obj.stat().st_size != identity['bytes'] or sha(obj) != identity['sha256']:
            raise ValueError(f'Runtime object no longer matches its compile receipt: {path}')
        inspected += 1
    if not inspected:
        raise ValueError('No completed runtime object receipts to validate')
    return inspected


def validate_runtime_archives(directory, archiver, groups=RUNTIME_GROUPS):
    """Reject stale archives left behind by a partial object rebuild."""
    directory = Path(directory) / 'runtime-objects'
    for group in groups:
        archive = directory / f'libOvPhysx_{group}.a'
        members = subprocess.check_output([str(archiver), 't', str(archive)], text=True).splitlines()
        expected = {path.stem + '.o' for path in directory.glob(group + '_*.json')
                    if isinstance(value := json.loads(path.read_text()), dict)
                    and value.get('group') == group and 'object' in value}
        if not members or len(members) != len(set(members)) or set(members) != expected:
            raise ValueError(f'Runtime archive members differ from compiled object receipts: {archive}')
        expected_hash = hashlib.sha256()
        for name in members:
            expected_hash.update((directory / name).read_bytes())
        archived_hash = hashlib.sha256()
        with subprocess.Popen([str(archiver), 'p', str(archive)], stdout=subprocess.PIPE) as process:
            for chunk in iter(lambda: process.stdout.read(1024 * 1024), b''):
                archived_hash.update(chunk)
            if process.wait():
                raise ValueError(f'Cannot inspect runtime archive: {archive}')
        if expected_hash.digest() != archived_hash.digest():
            raise ValueError(f'Runtime archive contains stale or changed object bytes: {archive}')


def prepare_artifact_abi(out, helper, run, receipt):
    """Validate a fresh stamped closure before replacing the ordinary linker outputs."""
    raw_receipt = out / 'raw-build-info.json'
    shutil.copyfile(out / 'build-info.json', raw_receipt)
    with tempfile.TemporaryDirectory(prefix='ovphysx-validated-artifacts-', dir=out.parent) as temporary:
        prepared = Path(temporary) / 'prepared'
        run([sys.executable, helper, '--source', out, '--output', prepared], 'prepare-artifact-abi')
        proof = json.loads((prepared / 'abi-build-info.json').read_text())
        if proof.get('kind') != 'experimental-validated-compiled-artifact-pair' or proof.get('protocol') != 1:
            raise ValueError('Unexpected artifact ABI preparation receipt')
        if proof.get('sourceReceiptSha256') != sha(raw_receipt) or proof.get('sourceArtifacts') != receipt['artifacts']:
            raise ValueError('Artifact ABI preparation used a different raw build set')
        expected = {'ovstage.mjs', 'ovstage.wasm', 'ovphysx.wasm', 'compiled-artifacts.mjs'}
        if set(proof.get('artifacts', {})) != expected:
            raise ValueError('Unexpected validated artifact closure')
        helper = Path(helper).resolve()
        loader_helper = helper.parent.parent / 'population/js/compiled-artifacts.mjs'
        expected_helper = receipt['inputs'].get(str(helper))
        expected_loader = receipt['inputs'].get(str(loader_helper))
        if not expected_helper or proof.get('helperSha256') != expected_helper or sha(helper) != expected_helper:
            raise ValueError('Artifact ABI preparer changed after source provenance was recorded')
        if not expected_loader or proof['artifacts']['compiled-artifacts.mjs']['sha256'] != expected_loader or sha(loader_helper) != expected_loader:
            raise ValueError('Compiled artifact loader changed after source provenance was recorded')
        for name, identity in proof['artifacts'].items():
            path = prepared / name
            if path.is_symlink() or not path.is_file() or path.stat().st_size != identity['bytes'] or sha(path) != identity['sha256']:
                raise ValueError(f'Validated artifact changed before installation: {name}')
        for name in sorted(expected):
            shutil.copyfile(prepared / name, out / name)
        receipt = {**receipt, 'artifactAbi': proof, 'artifacts': proof['artifacts']}
        (out / 'build-info.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return receipt


def configure_pthread_pool(command, *, maximum=None, stage_only=None, solver_workers=0, shared=False):
    """Choose the startup pool from declared libraries; never resize a live owner."""
    link = list(command)
    options = [i for i, value in enumerate(link) if value.startswith('-sPTHREAD_POOL_SIZE=')]
    if len(options) != 1:
        raise ValueError('Expected one main module pthread pool configuration')
    index = options[0]
    original = link[index].split('=', 1)[1]
    if not original.isdigit():
        raise ValueError('Input stage must provide a fixed numeric pthread pool')
    pool_size = int(original) if maximum is None else maximum
    if (not isinstance(pool_size, int) or isinstance(pool_size, bool) or pool_size < 0
            or ((maximum is not None or solver_workers) and pool_size < solver_workers + 4)):
        raise ValueError('Threaded physics requires a larger pthread pool with population headroom')
    policy = None
    if stage_only is not None:
        if not shared:
            raise ValueError('--stage-only-pthread-pool-size requires --shared')
        if not isinstance(stage_only, int) or isinstance(stage_only, bool) or stage_only < 8:
            raise ValueError('Stage-only pthread pool must retain at least eight qualified workers')
        if stage_only > pool_size:
            raise ValueError('Stage-only pthread pool cannot exceed the maximum pool')
        expression = f'((Module["dynamicLibraries"]||[]).length?{pool_size}:{stage_only})'
        link[index] = '-sPTHREAD_POOL_SIZE=' + expression
        policy = {'stageOnly': stage_only, 'requestedLibraries': pool_size, 'maximum': pool_size,
                  'selection': 'declared dynamicLibraries at initialization', 'expression': expression}
    elif maximum is not None:
        link[index] = f'-sPTHREAD_POOL_SIZE={pool_size}'
    return link, pool_size, policy



def solver_configuration(runtime_linked, default_workers):
    """Population metadata alone does not provide simulation entrypoints."""
    if not runtime_linked:
        return None
    return {'supportedWorkers': [0, 2, 4], 'nativeDefaultWorkers': default_workers,
            'setter': 'ovp_wasm_configure_solver_threads', 'capacityHeadroom': 4}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ovstage-build', type=Path, default=Path('/tmp/ovstage-population-build'))
    parser.add_argument('--out', type=Path, default=Path('/tmp/ovphysx-stage-integration'))
    parser.add_argument('--runtime-dir', type=Path, help='Link completed genuine OvPhysX runtime archives and prepared host')
    parser.add_argument('--shared', action='store_true', help='Link OVStage main and OvPhysX side WASM modules instead of one image')
    parser.add_argument('--solver-threads', type=int, choices=[0, 2, 4], default=0,
                        help='Additional PhysX SDK CPU dispatcher workers (requires owner-side pool preflight)')
    parser.add_argument('--pthread-pool-size', type=int,
                        help='Override the main module pthread pool; includes population and solver workers')
    parser.add_argument('--stage-only-pthread-pool-size', type=int,
                        help='Use a smaller startup pool without requested side libraries (shared mode, minimum eight)')
    parser.add_argument('--allocator', choices=['dlmalloc', 'mimalloc'], default='dlmalloc',
                        help='Allocator owned by the main module and shared by its physics side module')
    parser.add_argument('--side-private-functions', action='store_true',
                        help='Bind audited side-only functions locally in the qualified one-main/one-side graph')
    parser.add_argument('--prune-side-stubs', action='store_true',
                        help='Remove audited obsolete side-function JavaScript stubs without relinking native images')
    parser.add_argument('--validate-artifact-abi', action='store_true',
                        help='Require matching stamped main/side artifacts before creating native instances')
    parser.add_argument('--diagnostic-names', action='store_true', help='Retain WASM function names for runtime debugging')
    parser.add_argument('--debug-link', action='store_true', help='Disable link optimization and retain available DWARF for runtime diagnosis')
    args = parser.parse_args()
    if args.shared and not args.runtime_dir:
        parser.error('--shared requires --runtime-dir with a completed --pic build')
    if args.side_private_functions and not args.shared:
        parser.error('--side-private-functions requires --shared with a completed runtime')
    if args.stage_only_pthread_pool_size is not None and not args.shared:
        parser.error('--stage-only-pthread-pool-size requires --shared')
    if args.stage_only_pthread_pool_size is not None and args.stage_only_pthread_pool_size < 8:
        parser.error('--stage-only-pthread-pool-size must retain at least eight qualified workers')
    if args.prune_side_stubs and not args.side_private_functions:
        parser.error('--prune-side-stubs requires --side-private-functions')
    if args.validate_artifact_abi and not args.shared:
        parser.error('--validate-artifact-abi requires --shared with a completed runtime')
    if args.solver_threads and not args.runtime_dir:
        parser.error('--solver-threads requires --runtime-dir')
    if args.pthread_pool_size is not None and args.pthread_pool_size < args.solver_threads + 4:
        parser.error('--pthread-pool-size must leave at least four workers beyond the solver count')
    build = args.ovstage_build.resolve()
    out = args.out.resolve()
    if args.side_private_functions and (out / 'private-functions').exists():
        raise ValueError('Private function output must be fresh; preserve the prior proof and choose a new --out directory')
    if args.prune_side_stubs and (out / 'loader-pruning').exists():
        raise ValueError('Loader pruning output must be fresh; preserve the prior proof and choose a new --out directory')
    out.mkdir(parents=True, exist_ok=True)
    metadata = build / 'CMakeFiles/population_wasm.dir'
    flags_text = (metadata / 'flags.make').read_text()
    flags = []
    for name in ['CXX_DEFINES', 'CXX_INCLUDES', 'CXX_FLAGS']:
        match = re.search(r'^' + name + r' = (.*)$', flags_text, re.M)
        if not match:
            raise ValueError(f'Missing CMake {name}')
        flags += shlex.split(match[1])
    original_link = shlex.split((metadata / 'link.txt').read_text())
    compiler = Path(original_link[0])
    if '-sMEMORY64=1' not in flags or '-pthread' not in flags:
        raise ValueError('The input must be the memory64/pthreads native OVStage build')
    exception_mode = exception_mode_from_flags(flags, 'OVStage compile flags')
    javascript_eh, legacy_eh = exception_mode == 'javascript', exception_mode == 'wasm-legacy'
    exception_flags = ['-fexceptions'] if javascript_eh else [
        '-fwasm-exceptions', '-sWASM_LEGACY_EXCEPTIONS=' + ('1' if legacy_eh else '0')]
    for flag in exception_flags:
        if flag not in original_link:
            raise ValueError(f'Stage compile/link exception flags disagree: {flag}')
    runtime_build_receipt = None
    runtime_port_receipt = None
    runtime_port = {}
    if args.runtime_dir:
        runtime_build_receipt = args.runtime_dir / 'stage-build-inputs.json'
        validate_runtime_exception_abi(exception_mode, json.loads(runtime_build_receipt.read_text()))
        runtime_port_receipt = args.runtime_dir / 'port-receipt.json'
        runtime_port = json.loads(runtime_port_receipt.read_text())
    cache_file = build / 'CMakeCache.txt'
    cache = dict(re.findall(r'^([A-Za-z_][A-Za-z_0-9]*):[^=\n]+=(.*)$', cache_file.read_text(), re.M))
    abi_path = Path(cache['CMAKE_HOME_DIRECTORY']).parent / 'tools/exception_model.py'
    abi_spec = importlib.util.spec_from_file_location('ovstage_dependency_abi', abi_path)
    abi = importlib.util.module_from_spec(abi_spec)
    abi_spec.loader.exec_module(abi)
    pool_region_bits = abi.validate_usd_pool_abi(cache['FABRIC_USD_ROOT'], cache['OPENUSD_SOURCE_ROOT'])
    verified_runtime_objects = 0
    if args.runtime_dir:
        validate_runtime_pool_abi(pool_region_bits, json.loads(runtime_build_receipt.read_text()))
        verified_runtime_objects = validate_runtime_object_abis(
            args.runtime_dir, exception_mode, pool_region_bits, runtime_port.get('sdkMath', 'scalar'))
        validate_runtime_archives(args.runtime_dir, compiler.parent.parent / 'bin/llvm-ar')
    configured_link, pool_size, pool_policy = configure_pthread_pool(
        original_link, maximum=args.pthread_pool_size, stage_only=args.stage_only_pthread_pool_size,
        solver_workers=args.solver_threads, shared=args.shared)
    env = dict(os.environ)
    env['PATH'] = str(compiler.parent) + os.pathsep + env.get('PATH', '')
    commands = []
    runtime_inputs = []
    archive_overlay = None

    def run(command, name):
        command = [str(value) for value in command]
        commands.append({'name': name, 'cwd': str(build), 'argv': command})
        (out / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        with (out / f'{name}.log').open('w') as log:
            result = subprocess.run(command, cwd=build, env=env, stdout=log, stderr=subprocess.STDOUT)
        if result.returncode:
            raise RuntimeError(f'{name} failed ({result.returncode}); see {out / (name + ".log")}')

    # These are the actual codeless schema resources, with their original layout.
    schema_root = out / 'schemas'
    for family in ['physxSchema', 'omniUsdPhysicsDeformableSchema']:
        destination = schema_root / family / 'resources'
        destination.mkdir(parents=True, exist_ok=True)
        for name in ['plugInfo.json', 'generatedSchema.usda']:
            shutil.copy2(PHYSX / 'schemas/physx/source' / family / name, destination / name)

    original_bridge = build / 'bindings/bridge.cpp'
    text = original_bridge.read_text()
    text = replace_once(text, '#include <ovstage/ovstage_population.h>',
                        '#include <ovstage/ovstage_population.h>\n#include <ovphysx/ovphysx_population.h>\n#include <carb/Framework.h>\n#include <stdexcept>')
    startup_anchor = ('    concurrency = std::make_unique<tbb::global_control>(tbb::global_control::max_allowed_parallelism, 1);\n'
                      '    try\n    {\n')
    text = replace_once(text, startup_anchor, startup_anchor + '''
    constexpr char physxSchema[] = "/physics-schemas/physxSchema/resources/plugInfo.json";
    constexpr char deformableSchema[] = "/physics-schemas/omniUsdPhysicsDeformableSchema/resources/plugInfo.json";
    const ovx_string_t schemas[] = {
        { physxSchema, sizeof(physxSchema) - 1 },
        { deformableSchema, sizeof(deformableSchema) - 1 },
    };
    const ovstage_api_status_t registration = ovstage_population_register_usd_schemas(schemas, 2);
    if (registration != OVSTAGE_OK)
    {
        const ovx_string_t detail = ovstage_population_get_last_error();
        throw std::runtime_error(std::string(detail.ptr, detail.length));
    }''')
    text = replace_once(text, '        const auto submitted =\n', '''        const ovstage_population_desc_t descriptions[] = {
            *ovphysx_get_physics_population_desc(), description,
        };
        const auto submitted =
''')
    if text.count('ordinal, time, &description, 1)') != 2:
        raise ValueError('Unexpected population call layout')
    text = text.replace('ordinal, time, &description, 1)', 'ordinal, time, descriptions, 2)')
    text += '''
// Internal runtime borrowing only. Ownership remains with the validated stage handle.
extern "C" ovstage_instance_t* ovphysx_wasm_borrow_stage(uint32_t handle)
{
    return stageFor(handle);
}
extern "C" carb::Framework* ovphysx_wasm_borrow_framework()
{
    return carb::getFramework();
}
'''
    bridge = out / 'bridge.cpp'
    bridge.write_text(text)
    includes = [PHYSX / 'ovphysx/include', PHYSX / 'ovphysx/src/include', PHYSX / 'ovphysx/ovruntime/include']
    compile_flags = flags + [f'-I{value}' for value in includes]
    if args.runtime_dir:
        args.runtime_dir = args.runtime_dir.resolve()
        compile_flags = [f'-I{args.runtime_dir}/host/carb/include'] + compile_flags
    population = PHYSX / 'ovphysx/src/ovphysx/ovphysxPopulation.cpp'
    sources = [('bridge', bridge)]
    if args.shared:
        sources += [('physics-population', population)]
    if args.runtime_dir:
        sources += [('physics-bindings', HERE / 'bindings.cpp'), ('host', args.runtime_dir / 'host/host.cpp')]
        runtime_inputs.append(HERE / 'controls.inl')
    else:
        sources += [('physics-population', population)]
    for name, source in sources:
        relocatable = ['-fPIC'] if args.shared else []
        source_flags = [f'-DOVP_WASM_SOLVER_THREADS={args.solver_threads}'] if name == 'physics-bindings' else []
        run([compiler, *compile_flags, *relocatable, *source_flags, '-c', source, '-o', out / f'{name}.o'], name)
    link = configured_link
    old_object = 'CMakeFiles/population_wasm.dir/bindings/bridge.cpp.o'
    if link.count(old_object) != 1:
        raise ValueError('Unexpected native bridge object layout')
    index = link.index(old_object)
    link[index:index + 1] = [str(out / 'bridge.o')]
    if args.runtime_dir:
        host_objects = [arg for arg in link if arg.endswith('/fabric/host/host.cpp.o')]
        if len(host_objects) != 1:
            raise ValueError('Expected the original stage host object exactly once')
        link[link.index(host_objects[0])] = str(out / 'host.o')
        original_usdrt_name = 'usdrt/libusdrt_population_cpu.a'
        if link.count(original_usdrt_name) != 1:
            raise ValueError('Expected the original USDRT archive exactly once')
        original_usdrt = build / original_usdrt_name
        ar = compiler.parent.parent / 'bin/llvm-ar'
        members = subprocess.check_output([str(ar), 't', str(original_usdrt)], text=True).splitlines()
        removed = 'UnavailableServices.cpp.o'
        if members.count(removed) != 1:
            raise ValueError('Expected exactly one stage-only unavailable service provider')
        copied_usdrt = out / 'libusdrt_population_shared_host.a'
        shutil.copy2(original_usdrt, copied_usdrt)
        run([ar, 'd', copied_usdrt, removed], 'shared-host-archive')
        link[link.index(original_usdrt_name)] = str(copied_usdrt)
        archive_overlay = {'original': str(original_usdrt), 'originalSha256': sha(original_usdrt),
                           'removedMember': removed, 'replacement': 'bootstrap/StaticServices.cpp',
                           'derivedSha256': sha(copied_usdrt)}
        groups = RUNTIME_GROUPS
        archives = [args.runtime_dir / 'runtime-objects' / f'libOvPhysx_{group}.a' for group in groups]
        sdk = sorted((args.runtime_dir / 'build/physx/bin/UNKNOWN/release').glob('*.a'))
        if len(sdk) != 8 or any(not path.is_file() for path in archives):
            raise ValueError('Full genuine runtime/SDK archives are not ready')
        runtime_inputs = archives + sdk + [runtime_build_receipt, runtime_port_receipt]
        if args.shared:
            pic_receipt = json.loads((args.runtime_dir / 'stage-build-inputs.json').read_text())
            if '-fPIC' not in pic_receipt['flags']:
                raise ValueError('Shared physics requires a runtime built with --pic')
            # One host owns generic services; physics imports them alongside OVStage.
            shared_services = ['bootstrap_Log.o', 'bootstrap_StaticServices.o', 'bootstrap_Events.o']
            bootstrap = args.runtime_dir / 'runtime-objects/libOvPhysx_bootstrap.a'
            side_bootstrap = out / 'libOvPhysx_bootstrap_side.a'
            shutil.copyfile(bootstrap, side_bootstrap)
            run([ar, 'd', side_bootstrap, *shared_services], 'side-bootstrap-archive')
            side_archives = [side_bootstrap if path == bootstrap else path for path in archives]
            side_output = out / 'ovphysx.wasm'
            exports = ['_ovp_wasm_' + name for name in
                       ['create', 'step', 'read_vectors', 'read_links', 'read_joints', 'values', 'value_count', 'error', 'publish', 'destroy', 'solver_threads', 'configure_solver_threads',
                        'set_joint_velocity_targets', 'read_joint_states', 'set_kinematic_targets',
                        'read_body_states', 'apply_controls', 'control_error_is_partial']]
            side_link = [compiler, '-O2', '-sMEMORY64=1', '-pthread', *exception_flags, '-sSIDE_MODULE=2',
                         '-Wl,--error-limit=0', '-sEXPORTED_FUNCTIONS=' + json.dumps(exports),
                         str(out / 'physics-bindings.o'), '-Wl,--start-group',
                         *map(str, side_archives + sdk), '-Wl,--end-group', '-o', side_output]
            if args.diagnostic_names:
                side_link.append('--profiling-funcs')
            if args.debug_link:
                side_link += ['-O0', '-g3']
            run(side_link, 'link-physics')
            export_args = [i for i, value in enumerate(link) if value.startswith('-sEXPORTED_FUNCTIONS=')]
            if len(export_args) != 1:
                raise ValueError('Expected one stage JavaScript export list')
            export_index = export_args[0]
            main_exports = ast.literal_eval(link[export_index].split('=', 1)[1])
            link[export_index] = '-sEXPORTED_FUNCTIONS=' + json.dumps(main_exports + exports)
            link += ['-sDEFAULT_LIBRARY_FUNCS_TO_INCLUDE=' + json.dumps([name[1:] for name in exports])]
            host_objects = [args.runtime_dir / 'runtime-objects' / name for name in shared_services]
            runtime_inputs += host_objects
            # The official population description is metadata, independent of the solver.
            # Keeping it here lets the main module populate USD without loading physics.
            # Keep the shared C++ runtime in the main even when the optional side exports
            # weak inline definitions. Section GC still removes unused library code.
            cxx_suffix = '' if javascript_eh else '-legacyexcept' if legacy_eh else '-wasmexcept'
            cxx_runtime = compiler.parent / ('cache/sysroot/lib/wasm64-emscripten/pic/libc++-mt' + cxx_suffix + '.a')
            if not cxx_runtime.is_file():
                raise ValueError(f'Missing matching PIC C++ runtime: {cxx_runtime}')
            runtime_inputs.append(cxx_runtime)
            link += ['-Wl,--whole-archive', str(cxx_runtime), '-Wl,--no-whole-archive',
                     str(out / 'physics-population.o'), *map(str, host_objects), str(side_output),
                     '-sMAIN_MODULE=2', '-sAUTOLOAD_DYLIBS=0']
        else:
            link += [str(out / 'physics-bindings.o'), '-Wl,--start-group', *map(str, archives + sdk), '-Wl,--end-group']
    else:
        link += [str(out / 'physics-population.o')]
    output = out / ('ovstage.mjs' if args.shared else 'ovstage-ovphysx.mjs' if args.runtime_dir else 'ovstage-physics-population.mjs')
    link[link.index('-o') + 1] = str(output)
    if any(value.startswith('-sMALLOC=') for value in link):
        raise ValueError('Input stage link already selects an allocator; qualify that configuration explicitly')
    link += ['--embed-file', f'{schema_root}@/physics-schemas', '-sMALLOC=' + args.allocator]
    if args.diagnostic_names:
        link.append('--profiling-funcs')
    if args.debug_link:
        link += ['-O0', '-g3', '-sASSERTIONS=1']
    run(link, 'link')
    private_functions = None
    if args.side_private_functions:
        helper_path = HERE / 'side_function_visibility.py'
        helper_spec = importlib.util.spec_from_file_location('ovphysx_side_function_visibility', helper_path)
        helper = importlib.util.module_from_spec(helper_spec)
        helper_spec.loader.exec_module(helper)
        node = helper.resolve_emscripten_node(compiler, env=env)
        private_functions = helper.optimize_side_functions(
            main_wasm=output.with_suffix('.wasm'), main_loader=output, side_wasm=side_output,
            main_link=link, side_link=side_link, cwd=build, work_dir=out / 'private-functions',
            node=node, llvm_nm=compiler.parent.parent / 'bin/llvm-nm', run=run)
        runtime_inputs.append(helper_path)
    loader_pruning = None
    if args.prune_side_stubs:
        pruning_path = HERE / 'loader_stub_pruning.py'
        pruning_spec = importlib.util.spec_from_file_location('ovphysx_loader_stub_pruning', pruning_path)
        pruning = importlib.util.module_from_spec(pruning_spec)
        pruning_spec.loader.exec_module(pruning)
        loader_pruning = pruning.prune_side_stubs(
            main_loader=output, main_wasm=output.with_suffix('.wasm'), side_wasm=side_output,
            visibility=private_functions, main_link=link, cwd=build, node=node,
            work_dir=out / 'loader-pruning')
        runtime_inputs.append(pruning_path)
    outputs = [output, output.with_suffix('.wasm')]
    if args.shared:
        outputs.append(out / 'ovphysx.wasm')
    stage_inputs = [(Path(value) if Path(value).is_absolute() else build / value)
                    for value in original_link if value.endswith(('.a', '.o'))]
    kit = Path(cache['CMAKE_HOME_DIRECTORY']).parents[4]
    provenance_files = {
        'openUsd': Path(cache['FABRIC_USD_ROOT']) / 'share/ovstage-wasm/build-info.json',
        'abseil': Path(cache['USDRT_ABSEIL_ROOT']) / 'share/ovstage-wasm/abseil-build-info.json',
        'carbonite': build / 'carb-port/manifest.json',
    }
    experiment_root = kit / 'rendering/ovstage/experimental/wasm'
    helpers = sorted({str(path.relative_to(kit))
                      for component in ['fabric', 'population', 'cmake', 'tools']
                      for path in (experiment_root / component).rglob('*')
                      if path.is_file() and '__pycache__' not in path.parts and
                      (path.suffix in {'.py', '.h', '.cpp', '.inl', '.cmake', '.json'}
                       or path.name in {'CMakeLists.txt', 'compiled-artifacts.mjs'})})
    stage_inputs += [Path(__file__).resolve(), cache_file, *provenance_files.values(), *[kit / name for name in helpers]]
    receipt = {
        'kind': 'official-ovphysx-dylink-runtime' if args.shared else 'official-ovphysx-runtime' if args.runtime_dir else 'official-physics-population-only',
        'linkage': 'main-and-side-modules' if args.shared else 'static',
        'simulationRuntimeLinked': bool(args.runtime_dir),
        'exceptionModel': exception_mode,
        'allocator': args.allocator,
        'sidePrivateFunctions': private_functions,
        'loaderPruning': loader_pruning,
        'sdfPathPoolRegionBits': pool_region_bits,
        'verifiedRuntimeObjects': verified_runtime_objects,
        'sdkMath': runtime_port.get('sdkMath', 'scalar') if args.runtime_dir else None,
        'solverThreads': args.solver_threads,
        'solverConfiguration': solver_configuration(bool(args.runtime_dir), args.solver_threads),
        'pthreadPoolSize': pool_size,
        'pthreadPoolPolicy': pool_policy,
        'stageDependencies': {
            **{name: json.loads(path.read_text()) for name, path in provenance_files.items()},
            'positionIndependentCode': cache.get('CMAKE_POSITION_INDEPENDENT_CODE') == 'ON',
            'kitRevision': subprocess.check_output(['git', '-C', str(kit), 'rev-parse', 'HEAD'], text=True).strip(),
            'kitBuildHelpers': {name: sha(kit / name) for name in helpers},
        },
        'physxRevision': subprocess.check_output(['git', '-C', str(PHYSX), 'rev-parse', 'HEAD'], text=True).strip(),
        'inputs': {str(path): sha(path) for path in [original_bridge, metadata / 'flags.make', metadata / 'link.txt', population, *[source for _, source in sources], *runtime_inputs, *stage_inputs]},
        'sharedHostArchiveOverlay': archive_overlay,
        'schemaResources': {str(path.relative_to(schema_root)): sha(path) for path in schema_root.rglob('*') if path.is_file()},
        'artifacts': {path.name: {'bytes': path.stat().st_size, 'sha256': sha(path)} for path in outputs},
    }
    (out / 'build-info.json').write_text(json.dumps(receipt, indent=2) + '\n')
    if args.validate_artifact_abi:
        receipt = prepare_artifact_abi(out, experiment_root / 'tools/prepare_abi_pair.py', run, receipt)
    print(json.dumps(receipt['artifacts'], indent=2))


if __name__ == '__main__':
    main()
