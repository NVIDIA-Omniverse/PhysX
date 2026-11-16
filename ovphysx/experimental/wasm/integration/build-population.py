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
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
PHYSX = HERE.parents[3]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError(f'Expected exactly one bridge anchor: {old[:70]}')
    return text.replace(old, new)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ovstage-build', type=Path, default=Path('/tmp/ovstage-population-build'))
    parser.add_argument('--out', type=Path, default=Path('/tmp/ovphysx-stage-integration'))
    parser.add_argument('--runtime-dir', type=Path, help='Link completed genuine OvPhysX runtime archives and prepared host')
    parser.add_argument('--shared', action='store_true', help='Link OVStage main and OvPhysX side WASM modules instead of one image')
    parser.add_argument('--diagnostic-names', action='store_true', help='Retain WASM function names for runtime debugging')
    parser.add_argument('--debug-link', action='store_true', help='Disable link optimization and retain available DWARF for runtime diagnosis')
    args = parser.parse_args()
    if args.shared and not args.runtime_dir:
        parser.error('--shared requires --runtime-dir with a completed --pic build')
    build = args.ovstage_build.resolve()
    out = args.out.resolve()
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
    else:
        sources += [('physics-population', population)]
    for name, source in sources:
        relocatable = ['-fPIC'] if args.shared else []
        run([compiler, *compile_flags, *relocatable, '-c', source, '-o', out / f'{name}.o'], name)
    link = list(original_link)
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
        groups = ['common','parse','ovstage','foundation','cuda_shim','cooking','convex','runtime','api','sidecar','bootstrap','closure','platform']
        archives = [args.runtime_dir / 'runtime-objects' / f'libOvPhysx_{group}.a' for group in groups]
        sdk = sorted((args.runtime_dir / 'build/physx/bin/UNKNOWN/release').glob('*.a'))
        if len(sdk) != 8 or any(not path.is_file() for path in archives):
            raise ValueError('Full genuine runtime/SDK archives are not ready')
        runtime_inputs = archives + sdk
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
                       ['create', 'step', 'read_vectors', 'read_links', 'read_joints', 'values', 'value_count', 'error', 'publish', 'destroy']]
            side_link = [compiler, '-O2', '-sMEMORY64=1', '-pthread', '-fexceptions', '-sSIDE_MODULE=2',
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
            cxx_runtime = compiler.parent / 'cache/sysroot/lib/wasm64-emscripten/pic/libc++-mt.a'
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
    link += ['--embed-file', f'{schema_root}@/physics-schemas']
    if args.diagnostic_names:
        link.append('--profiling-funcs')
    if args.debug_link:
        link += ['-O0', '-g3', '-sASSERTIONS=1']
    run(link, 'link')
    outputs = [output, output.with_suffix('.wasm')]
    if args.shared:
        outputs.append(out / 'ovphysx.wasm')
    stage_inputs = [(Path(value) if Path(value).is_absolute() else build / value)
                    for value in original_link if value.endswith(('.a', '.o'))]
    cache_file = build / 'CMakeCache.txt'
    cache = dict(re.findall(r'^([A-Za-z_][A-Za-z_0-9]*):[^=\\n]+=(.*)$', cache_file.read_text(), re.M))
    kit = Path(cache['CMAKE_HOME_DIRECTORY']).parents[4]
    provenance_files = {
        'openUsd': Path(cache['FABRIC_USD_ROOT']) / 'share/ovstage-wasm/build-info.json',
        'abseil': Path(cache['USDRT_ABSEIL_ROOT']) / 'share/ovstage-wasm/abseil-build-info.json',
        'carbonite': build / 'carb-port/manifest.json',
    }
    helpers = [
        'rendering/ovstage/experimental/wasm/population/build.py',
        'rendering/ovstage/experimental/wasm/population/usdrt/build-abseil.py',
        'rendering/ovstage/experimental/wasm/fabric/port/prepare.py',
        'rendering/ovstage/experimental/wasm/fabric/port/manifest.json',
    ]
    stage_inputs += [cache_file, *provenance_files.values(), *[kit / name for name in helpers]]
    receipt = {
        'kind': 'official-ovphysx-dylink-runtime' if args.shared else 'official-ovphysx-runtime' if args.runtime_dir else 'official-physics-population-only',
        'linkage': 'main-and-side-modules' if args.shared else 'static',
        'simulationRuntimeLinked': bool(args.runtime_dir),
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
    print(json.dumps(receipt['artifacts'], indent=2))


if __name__ == '__main__':
    main()
