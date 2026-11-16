#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Reproduce genuine OvPhysX WASM64 prerequisite and source compilation probes.

This does not produce a usable @omniverse/ovphysx runtime yet. It never substitutes
an SDK descriptor bridge or a JavaScript physics implementation for OvPhysX.
"""
from __future__ import annotations
import argparse
import concurrent.futures
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import shlex
import subprocess

REVISION = "7d6f6ac8b83e1610bdf715f1f2f1d6b718aee586"
EMSDK_REVISION = "708dcbcc9ee23bc7d2dad44fc90db377e38b771c"

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def run(args, env, log):
    with Path(log).open('w') as handle:
        result = subprocess.run([str(x) for x in args], env=env, stdout=handle, stderr=subprocess.STDOUT)
    return result.returncode

def verify_source_baseline(source, baseline):
    revision = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    result = subprocess.run(['git', '-C', str(source), 'merge-base', '--is-ancestor', baseline, revision],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if result.returncode != 0:
        raise ValueError(f'Expected baseline {baseline} in the history at {source}, found {revision}')
    return revision


def sdk(args, env):
    configure = ['cmake', '-S', Path(__file__).parent, '-B', args.out / 'build', '-G', 'Unix Makefiles',
                 f'-DPHYSX_SOURCE_ROOT={args.source}',
                 f'-DCMAKE_TOOLCHAIN_FILE={args.emsdk}/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake',
                 '-DCMAKE_C_FLAGS=-sMEMORY64=1', '-DCMAKE_CXX_FLAGS=-sMEMORY64=1', '-DCMAKE_BUILD_TYPE=release']
    if args.pic:
        configure.append('-DCMAKE_POSITION_INDEPENDENT_CODE=ON')
    code = run(configure, env, args.out / 'configure.log')
    if code:
        raise RuntimeError(f'SDK configure failed ({code}); see configure.log')
    build = ['cmake', '--build', args.out / 'build', '--target', 'PhysX', 'PhysXExtensions',
             'PhysXCharacterKinematic', 'PhysXVehicle', 'PVDRuntime', '--parallel', args.jobs]
    code = run(build, env, args.out / 'sdk-build.log')
    archives = {str(p.relative_to(args.out)): {'bytes': p.stat().st_size, 'sha256': digest(p)}
                for p in sorted((args.out / 'build').rglob('*.a'))}
    return {'exitCode': code, 'archives': archives, 'configure': [str(x) for x in configure], 'build': [str(x) for x in build]}

def named_sources(root, component, variable):
    directory = root / 'ovphysx/ovruntime/source' / component
    text = (directory / 'CMakeLists.txt').read_text()
    match = re.search(r'set\(' + re.escape(variable) + r'\s+(.*?)\n\)', text, re.S)
    if not match:
        raise ValueError(variable)
    appended = re.findall(r'list\(APPEND ' + re.escape(variable) + r'\s+(.*?)\n\)', text, re.S)
    names = re.findall(r'(?:\$\{CMAKE_CURRENT_SOURCE_DIR\}/)?([A-Za-z0-9_./-]+\.cpp)', '\n'.join([match[1]] + appended))
    return [directory / x for x in names]

def api_sources(root):
    text = (root / 'CMakeLists.txt').read_text()
    match = re.search(r'add_library\(ovphysx SHARED\s+(.*?)\n\)', text, re.S)
    excluded = {'src/CarboniteLoader/CarboniteLoader.cpp', 'src/ovphysx/ovphysxSidecarLoader.cpp'}
    return [root / name for name in re.findall(r'((?:src|companion)/[A-Za-z0-9_./-]+\.cpp)', match[1]) if name not in excluded]

def cache_identity(args, flags, generated, env):
    roots = [args.source / 'ovphysx/ovruntime' / x for x in ('include', 'source', 'cmake', 'deps')]
    roots += [args.source / 'physx' / x for x in ('include', 'source')]
    roots += [args.source / 'ovphysx/cmake', Path(__file__).parent]
    files = sorted({p for root in roots for p in root.rglob('*') if p.is_file() and
                    'tests' not in p.relative_to(root).parts and '__pycache__' not in p.parts and p.suffix != '.pyc'})
    files += [args.source / 'ovphysx/ovruntime/CMakeLists.txt', args.out / 'stage-build-inputs.json']
    identity = generated / 'CompiledCacheBuildIdentity.txt'
    manifest = generated / 'CompiledCacheBuildFiles.txt'
    identity.write_text(json.dumps({'platform': 'Emscripten-wasm64', 'emsdkRevision': EMSDK_REVISION,
                                    'sourceRevision': args.source_revision, 'sourceBaselineRevision': REVISION, 'flags': flags}, sort_keys=True) + '\n')
    manifest.write_text('\n'.join(str(x) for x in files) + '\n')
    command = ['cmake', f'-DIDENTITY={identity}', f'-DFILES={manifest}', '-DCONFIG=release',
               f'-DOUTPUT={generated}/CompiledCacheBuild.h', '-P',
               args.source / 'ovphysx/ovruntime/cmake/CompiledCacheIdentityHash.cmake']
    if run(command, env, generated / 'cache-identity.log'):
        raise RuntimeError('Official compiled-cache identity generator failed')


def stage_flags(args):
    path = args.stage_build / 'native/CMakeFiles/ovstage_native_cpu.dir/flags.make'
    cache_path = args.stage_build / 'CMakeCache.txt'
    text = path.read_text()
    flags = []
    for name in ('CXX_DEFINES', 'CXX_INCLUDES', 'CXX_FLAGS'):
        match = re.search(r'^' + name + r' = (.*)$', text, re.M)
        if not match:
            raise ValueError(f'{name} is missing from generated OvStage flags: {path}')
        flags += shlex.split(match[1])
    cache = dict((key, value) for key, value in re.findall(r'^([A-Za-z_][A-Za-z_0-9]*):[^=\n]+=(.*)$', cache_path.read_text(), re.M))
    includes = [Path(x[2:]) for x in flags if x.startswith('-I')]
    host_source = next(x for x in includes if (x / 'host.cpp').is_file() and x.name == 'host')
    module_spec = importlib.util.spec_from_file_location('ovphysx_wasm_host', Path(__file__).parent / 'host/prepare.py')
    host = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(host)
    paths = host.prepare(cache['FABRIC_CARB_PORT_INCLUDE'], host_source, args.out / 'host')
    excluded = ('-DOVSTAGE_BUILDING_DLL', '-DFABRIC_IS_BUILDING', '-DFABRIC_SHARED')
    flags = [x for x in flags if x not in excluded and not x.startswith(('-std=', '-O'))]
    flags = [str(args.emsdk / 'upstream/emscripten/em++'), '-std=c++17', '-O1', '-c', '-I' + str(paths['include'])] + flags
    if args.pic and '-fPIC' not in flags:
        flags.append('-fPIC')
    receipt = {'positionIndependentCode': args.pic, 'flagsMake': str(path), 'flagsSha256': digest(path), 'cache': str(cache_path),
               'hostOverlay': str(paths['output'] / 'overlay-receipt.json'), 'flags': flags}
    (args.out / 'stage-build-inputs.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return flags

def prepare_sdf_cooking(source, generated):
    # The browser pool cannot grow while PxThreadImpl::start waits synchronously.
    # The SDK's one-thread SDF path executes inline and preserves the algorithm.
    text = source.read_text()
    original = 'sdfDesc.numThreadsForSdfConstruction = 16;'
    if text.count(original) != 1:
        raise ValueError('SDF cooking thread-count anchor changed')
    output = generated / 'cooking' / source.name
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text.replace(original, 'sdfDesc.numThreadsForSdfConstruction = 1;'))
    return output


def compile_sources(args, env):
    flags = stage_flags(args)
    root = args.source / 'ovphysx'
    generated = args.out / 'generated'
    version = generated / 'omni/physx/Version.h'
    version.parent.mkdir(parents=True, exist_ok=True)
    build_version = (root / 'ovruntime/VERSION').read_text().strip()
    build_date = subprocess.check_output(['git', '-C', str(args.source), 'show', '-s', '--format=%ad', '--date=format:%b-%d-%Y'], text=True).strip()
    version.write_text('#pragma once\n' + '\n'.join('#define ' + k + ' ' + json.dumps(v) for k, v in {
        'PHYSICS_BUILD_SHA': args.source_revision, 'PHYSICS_BUILD_DATE': build_date,
        'PHYSICS_BUILD_BRANCH': 'ovphysx-0.8.0-wasm64-port', 'PHYSICS_BUILD_VERSION': build_version}.items()) + '\n')
    version_template = (root / 'include/ovphysx/version.h.in').read_text()
    public_version = generated / 'ovphysx/version.h'
    public_version.parent.mkdir(parents=True, exist_ok=True)
    for key, value in {'MAJOR': '0', 'MINOR': '8', 'PATCH': '0', 'PRERELEASE': '', 'FULL': '0.8.0-wasm64-experimental'}.items():
        version_template = version_template.replace('@OVPHYSX_VERSION_' + key + '@', value)
    public_version.write_text(version_template)
    companion_inputs = sorted(p for p in (args.source / 'physx/include').rglob('*')
                              if p.is_file() and p.suffix in ('.h', '.hpp', '.inl'))
    companion_inputs += sorted((args.out / 'build').rglob('*.a'))
    companion_manifest = 'Emscripten/wasm64\n' + ''.join(
        str(p.relative_to(args.source) if p.is_relative_to(args.source) else p.relative_to(args.out)) + ':' + digest(p) + '\n'
        for p in companion_inputs)
    companion_version = generated / 'ovphysx_companion/native_build.h'
    companion_version.parent.mkdir(parents=True, exist_ok=True)
    companion_template = (root / 'companion/cmake/native_build.h.in').read_text()
    companion_template = companion_template.replace('@COMPANION_PHYSX_BUILD@', hashlib.sha256(companion_manifest.encode()).hexdigest())
    companion_template = companion_template.replace('@OVPHYSX_COMPANION_DEBUG_BUILD@', '0')
    companion_version.write_text(companion_template)
    (generated / 'CompanionSdkManifest.txt').write_text(companion_manifest)
    include = [generated] + [args.source / 'physx/include', args.source, args.source / 'physx/pvdruntime/include', root / 'include', root / 'companion/include',
               root / 'src', root / 'src/include', root / 'src/ovphysx', root / 'src/UsdSchemaPaths', root / 'ovruntime/include',
               root / 'ovruntime/include/private', root / 'ovruntime/source/common/include',
               root / 'ovruntime/source/common/include/common/foundation', root / 'ovruntime/source/common/include/common/utilities',
               root / 'ovruntime/source/omni.physx/plugins', root / 'ovruntime/source/omni.physx.foundation/plugins',
               root / 'ovruntime/source/omni.physx.cooking/plugins', root / 'ovruntime/source/omni.physx.gpu/plugins',
               root / 'ovruntime/source/omni.physics.ovstage']
    flags[1:1] = ['-msimd128', '-DPX_SUPPORT_GPU_PHYSX=1', '-DPX_SIMD_DISABLED=1', '-DPX_PHYSX_STATIC_LIB', '-DOMNI_PHYSX_OVX_STATIC',
                  '-DOVPHYSX_CARB_NVTX_BACKEND=0', '-DOVRUNTIME_NO_UJITSO=1', '-DOMNI_LOG_DEFAULT_CHANNEL=kDefaultChannel',
                  '-Dcarb_eventdispatcher_IEventDispatcher=CARB_HEXVERSION(1,4)'] + [f'-I{x}' for x in include]
    cache_identity(args, flags, generated, env)
    platform_spec = importlib.util.spec_from_file_location('ovphysx_wasm_cuda', Path(__file__).parent / 'platform/generate.py')
    platform = importlib.util.module_from_spec(platform_spec)
    platform_spec.loader.exec_module(platform)
    platform.generate(args.source, generated)
    groups = {
        'common': named_sources(args.source, 'common', 'COMMON_SOURCE'),
        'parse': named_sources(args.source, 'omni.physics.parse', 'PARSE_SOURCE'),
        'ovstage': named_sources(args.source, 'omni.physics.ovstage', 'PARSE_OVSTAGE_SOURCE'),
        'foundation': named_sources(args.source, 'omni.physx.foundation', 'PHYSXFOUNDATION_PLUGIN_SOURCE'),
        'cuda_shim': named_sources(args.source, 'omni.physx.foundation', 'PHYSX_CUDA_SHIM_SOURCE'),
        'cooking': named_sources(args.source, 'omni.physx.cooking', 'COOKING_PLUGIN_SOURCE'),
        'convex': named_sources(args.source, 'omni.convexdecomposition', 'CONVEXDECOMP_PLUGIN_SOURCE'),
        'runtime': named_sources(args.source, 'omni.physx', 'PHYSX_PLUGIN_SOURCE'),
        'api': api_sources(root),
        'closure': [root / 'ovruntime/source/omni.physx/plugins/cache/RuntimeArchive.cpp',
                    root / 'ovruntime/source/omni.physx.gpu/plugins/PhysXGpuLoader.cpp'],
        'platform': [Path(__file__).parent / 'platform/CudaUnavailable.cpp', generated / 'CudaKernelsUnavailable.cpp'],
        'sidecar': [root / 'src/ovphysxInternal' / name for name in ('ovphysxInternal.cpp', 'ovphysxInternalInterop.cpp', 'ovphysxInternalObjectChange.cpp')],
        'bootstrap': [Path(__file__).parent / 'host' / name for name in ('StaticBootstrap.cpp', 'StaticSidecar.cpp', 'Events.cpp', 'OvStageOutputCudaUnavailable.cpp', 'Log.cpp', 'StaticServices.cpp')],
    }
    requested = args.components.split(',')
    unknown = set(requested) - groups.keys()
    if unknown:
        raise ValueError(f'Unknown source components: {sorted(unknown)}')
    groups = {name: groups[name] for name in requested}
    for name, sources in groups.items():
        stems = [s.stem for s in sources]
        if len(stems) != len(set(stems)):
            raise ValueError(f'Duplicate object names in {name}')
    destination = args.out / 'runtime-objects'
    destination.mkdir(exist_ok=True)
    def compile_one(item):
        group, source = item
        stem = group + '_' + source.stem
        output = destination / (stem + '.o')
        compiled_source = source
        extra_includes = []
        if group == 'cooking' and source.name == 'TriangleMeshCookingTask.cpp':
            compiled_source = prepare_sdf_cooking(source, generated)
            extra_includes = ['-I' + str(source.parent)]
        command = flags + extra_includes + [str(compiled_source), '-o', str(output)]
        code = run(command, env, destination / (stem + '.log'))
        result = {'group': group, 'source': str(source.relative_to(args.source)) if source.is_relative_to(args.source) else str(source), 'sourceSha256': digest(source),
                'exitCode': code, 'command': command,
                'object': {'path': str(output), 'bytes': output.stat().st_size, 'sha256': digest(output)} if code == 0 else None}
        if compiled_source != source:
            result['compiledSource'] = {'path': str(compiled_source), 'sha256': digest(compiled_source),
                                        'adaptation': 'serial SDF construction in the bounded WASM worker pool'}
        (destination / (stem + '.json')).write_text(json.dumps(result, indent=2) + '\n')
        return result
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        results = list(pool.map(compile_one, [(group, source) for group, sources in groups.items() for source in sources]))
    (destination / ('results-' + args.components.replace(',', '_') + '.json')).write_text(json.dumps(results, indent=2) + '\n')
    for group in groups:
        members = [r for r in results if r['group'] == group]
        archive = destination / ('libOvPhysx_' + group + '.a')
        archive.unlink(missing_ok=True)
        if all(r['exitCode'] == 0 for r in members):
            command = [args.emsdk / 'upstream/emscripten/emar', 'rcs', archive]
            command += [r['object']['path'] for r in members]
            code = run(command, env, destination / (group + '-archive.log'))
            if code:
                raise RuntimeError(f'Archiving {group} failed: {code}')
    return results

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument('--emsdk', type=Path, default=Path('/tmp/ovstage-wasm-emsdk'))
    parser.add_argument('--out', type=Path, default=Path('/tmp/ovphysx-wasm-canonical'))
    parser.add_argument('--stage-build', type=Path, default=Path('/tmp/ovstage-population-build'), help='Configured and built native OvStage WASM CMake tree')
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--pic', action='store_true', help='Compile position-independent objects for a WASM side module')
    parser.add_argument('--sdk', action='store_true')
    parser.add_argument('--runtime-objects', action='store_true')
    parser.add_argument('--components', default='common,parse,ovstage')
    args = parser.parse_args()
    if not 1 <= args.jobs <= 4:
        parser.error('jobs must be between 1 and 4')
    args.source_revision = verify_source_baseline(args.source, REVISION)
    emsdk_revision = subprocess.check_output(['git', '-C', str(args.emsdk), 'rev-parse', 'HEAD'], text=True).strip()
    if emsdk_revision != EMSDK_REVISION:
        raise ValueError(f'Expected pinned revision {EMSDK_REVISION} at {args.emsdk}, found {emsdk_revision}')
    args.out.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, EM_CONFIG=str(args.emsdk / '.emscripten'))
    receipt_path = args.out / 'port-receipt.json'
    receipt = json.loads(receipt_path.read_text()) if receipt_path.exists() else {}
    receipt.update({'source': {'repository': 'https://github.com/NVIDIA-Omniverse/PhysX', 'revision': args.source_revision, 'baselineRevision': REVISION, 'release': 'ovphysx-0.8.0'},
                    'toolchain': {'emscripten': '5.0.7', 'emsdkRevision': EMSDK_REVISION},
                    'status': 'prerequisites-only; no linked OvPhysX browser runtime', 'customPhysicsBridge': False,
                    'experimentalSourceSha256': {str(p.relative_to(args.source)): digest(p) for p in Path(__file__).parent.rglob('*') if p.is_file() and '__pycache__' not in p.parts},
                    'positionIndependentCode': args.pic, 'recipeSha256': digest(__file__), 'sourceDiffSha256': hashlib.sha256(subprocess.check_output(['git', '-C', str(args.source), 'diff'])).hexdigest(), 'sourceDiff': subprocess.check_output(['git', '-C', str(args.source), 'diff', '--stat'], text=True),
                    'sourceModified': subprocess.check_output(['git', '-C', str(args.source), 'status', '--porcelain'], text=True).splitlines()})
    if args.sdk:
        receipt['sdk'] = sdk(args, env)
        receipt_path.write_text(json.dumps(receipt, indent=2) + '\n')
        if receipt['sdk']['exitCode']:
            print(json.dumps(receipt, indent=2))
            raise SystemExit(receipt['sdk']['exitCode'])
    if args.runtime_objects:
        results = compile_sources(args, env)
        receipt['runtimeObjects_' + args.components.replace(',', '_')] = {'passed': sum(r['exitCode'] == 0 for r in results), 'failed': sum(r['exitCode'] != 0 for r in results),
                                     'results': str(args.out / ('runtime-objects/results-' + args.components.replace(',', '_') + '.json'))}
    receipt_path.write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))
    if (args.sdk and receipt['sdk']['exitCode']) or (args.runtime_objects and any(r['exitCode'] for r in results)):
        raise SystemExit(1)

if __name__ == '__main__':
    main()
