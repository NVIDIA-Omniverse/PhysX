# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Remove obsolete Emscripten side-function stubs without changing native images.

This is deliberately restricted to the audited main plus optional physics side
module. The pinned generated-JavaScript grammar fails closed; this is not a
minifier or support for arbitrary later dlopen graphs.
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import tempfile
from collections.abc import Sequence

_SPEC = importlib.util.spec_from_file_location(
    'ovphysx_loader_visibility', Path(__file__).with_name('side_function_visibility.py'))
_visibility = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_visibility)

_IDENTIFIER = r'[A-Za-z_$][A-Za-z0-9_$]*'
_STUB = re.compile(r'function (' + _IDENTIFIER + r')\(\.\.\.args\)\{return wasmImports\["([^"\\]+)"\]\(\.\.\.args\)\}\1\.stub=true;')
_IMPORT_PREFIX = 'function assignWasmImports(){wasmImports={'
_IMPORT_SUFFIX = '}}function applySignatureConversions('


def _identity(path: Path) -> dict:
    return {'path': str(path), 'bytes': path.stat().st_size,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def _write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2) + '\n')


def prune_source(source: str, names: set[str]) -> tuple[str, dict]:
    """Delete exact declaration/initializer spans and preserve every other byte."""
    if (source.count('export default createNativePopulationModule;') != 1
            or source.count('var sharedModules={};') != 1
            or 'const artifactDescription=' in source):
        raise ValueError('Pruning requires the original unwrapped Emscripten loader')
    if source.count(_IMPORT_PREFIX) != 1 or source.count(_IMPORT_SUFFIX) != 1:
        raise ValueError('Unsupported generated import-table boundary')
    begin = source.index(_IMPORT_PREFIX) + len(_IMPORT_PREFIX)
    end = source.index(_IMPORT_SUFFIX, begin)
    entries = source[begin:end].split(',')
    if any(not re.fullmatch(_IDENTIFIER + ':' + _IDENTIFIER, entry) for entry in entries):
        raise ValueError('Unsupported generated import-table expression')
    imports = {}
    for entry in entries:
        key, value = entry.split(':')
        if key in imports:
            raise ValueError('Duplicate generated import entry: ' + key)
        imports[key] = value
    stubs = {}
    for match in _STUB.finditer(source):
        if match[2] in stubs:
            raise ValueError('Duplicate generated stub: ' + match[2])
        stubs[match[2]] = match
    if not names or names - stubs.keys():
        raise ValueError('No gain or audited obsolete stubs are missing')
    spans = []
    for name in sorted(names):
        match = stubs[name]
        if imports.get(name) != match[1] or match.end() > begin:
            raise ValueError('Stub declaration and import entry do not match: ' + name)
        spans.append((match.start(), match.end()))
    filtered = ','.join(entry for entry in entries if entry.split(':', 1)[0] not in names)
    result = source[:begin] + filtered + source[end:]
    for start, stop in sorted(spans, reverse=True):
        result = result[:start] + result[stop:]
    for name in names:
        if name in result or stubs[name][1] in result:
            raise ValueError('Removed function retains a generated-source reference: ' + name)
    return result, {'removedFunctions': sorted(names), 'stubsBefore': len(stubs),
                    'stubsAfter': len(stubs) - len(names),
                    'removedSourceBytes': len(source.encode()) - len(result.encode()),
                    'onlyDeclarationsAndImportEntriesRemoved': True}


def _selected_functions(proof: dict, policy: dict, modules: list[dict]) -> set[str]:
    if (proof.get('protocol') != 1 or proof.get('applied') is not True
            or proof.get('mainArtifactsUnchanged') is not True
            or proof.get('originalInputsUnchanged') is not True):
        raise ValueError('An applied, successful side-visibility proof is required')
    for key in ['functions', 'protected', 'publicFunctions']:
        values = policy.get(key)
        if (not isinstance(values, list) or any(not isinstance(value, str) or (not value and key != 'protected') for value in values)
                or len(values) != len(set(values))):
            raise ValueError('Invalid visibility policy: ' + key)
    removed = proof.get('audit', {}).get('removedImports')
    if not isinstance(removed, list) or any(
        not isinstance(row, dict) or (row.get('module'), row.get('kind'))
        not in {('env', 'function'), ('GOT.func', 'global')} or not isinstance(row.get('name'), str)
        for row in removed
    ):
        raise ValueError('Only audited function imports and their function-address entries are eligible')
    functions = [row['name'] for row in removed if row['module'] == 'env']
    selected = set(functions)
    if (not selected or len(selected) != len(functions) or selected != set(policy['functions'])
            or proof.get('selectedFunctions') != len(selected)
            or any(row['name'] not in selected for row in removed)):
        raise ValueError('Removed imports differ from the audited private-function set or provide no gain')
    if selected & (set(policy['protected']) | set(policy['publicFunctions'])):
        raise ValueError('Cannot prune protected shared or public functions')
    visible = {row['name'] for module in modules for kind in ['imports', 'exports'] for row in module[kind]}
    if selected & visible:
        raise ValueError('Selected functions remain in the actual main/side native interface')
    exports = {row['name'] for row in modules[1]['exports'] if row['kind'] == 'function'}
    if not set(policy['publicFunctions']) <= exports:
        raise ValueError('Actual side module is missing declared public functions')
    return selected


def prune_side_stubs(*, main_loader: Path, main_wasm: Path, side_wasm: Path,
                     visibility: dict, main_link: Sequence[str], cwd: Path,
                     node: Sequence[str], work_dir: Path) -> dict:
    """Install one audited loader-only transform after native side optimization."""
    for path in [main_loader, main_wasm, side_wasm]:
        if Path(path).is_symlink():
            raise ValueError('Pruning requires ordinary linked artifact files')
    main_loader, main_wasm, side_wasm = [Path(path).resolve() for path in [main_loader, main_wasm, side_wasm]]
    cwd, work_dir = Path(cwd).resolve(), Path(work_dir).resolve()
    if len({main_loader, main_wasm, side_wasm}) != 3 or work_dir.exists():
        raise ValueError('Use distinct artifacts and a fresh loader-pruning work directory')
    if '-sMAIN_MODULE=2' not in main_link or '-sAUTOLOAD_DYLIBS=0' not in main_link:
        raise ValueError('Only the optional main/side module graph is supported')
    dsos = [Path(arg) if Path(arg).is_absolute() else cwd / arg
            for arg in main_link if not arg.startswith('-') and arg.endswith('.wasm')]
    if [path.resolve() for path in dsos] != [side_wasm]:
        raise ValueError('Extra or unexpected side modules require separate qualification')
    if _visibility._output_path(main_link, cwd) != main_loader:
        raise ValueError('Main link output differs from the supplied loader')
    actual_inputs = _visibility._link_inputs(main_link, cwd)
    input_rows = visibility.get('mainInputDefinitions', [])
    recorded_inputs = {Path(row['path']).resolve(): row for row in input_rows}
    if len(recorded_inputs) != len(input_rows) or set(actual_inputs) != set(recorded_inputs):
        raise ValueError('Actual main inputs differ from the visibility proof')
    artifact_rows = visibility.get('mainArtifacts', [])
    recorded_artifacts = {Path(row['path']).resolve(): row for row in artifact_rows}
    installed = visibility.get('installedSide')
    if len(artifact_rows) != 2 or set(recorded_artifacts) != {main_loader, main_wasm} or not isinstance(installed, dict):
        raise ValueError('Missing original main or optimized-side artifact identities')
    policy_identity = visibility.get('policy')
    if not isinstance(policy_identity, dict) or not isinstance(policy_identity.get('path'), str):
        raise ValueError('Missing hashed visibility policy')
    policy_path = Path(policy_identity['path']).resolve()
    expected = {**recorded_inputs, **recorded_artifacts, side_wasm: installed, policy_path: policy_identity}
    for path, identity in expected.items():
        actual = _identity(path)
        if any(actual.get(key) != identity.get(key) for key in ['path', 'bytes', 'sha256']):
            raise ValueError('Visibility input or artifact changed before pruning: ' + str(path))
    policy = json.loads(policy_path.read_text())
    helpers = [Path(__file__).resolve(), Path(_visibility.__file__).resolve()]
    frozen = {path: _identity(path) for path in [*expected, *helpers]}
    modules = _visibility._inspect_modules(node, [main_wasm, side_wasm])
    selected = _selected_functions(visibility, policy, modules)
    source = main_loader.read_bytes().decode('utf-8')
    result, audit = prune_source(source, selected)
    work_dir.mkdir(parents=True)
    original = work_dir / 'unpruned-ovstage.mjs'
    original.write_bytes(main_loader.read_bytes())
    candidate = work_dir / 'pruned-ovstage.mjs'
    candidate_bytes = result.encode()
    candidate.write_bytes(candidate_bytes)
    receipt = {'protocol': 1, 'scope': 'one main and one optional physics side; not an arbitrary dlopen ABI',
               'applied': False, 'helper': _identity(helpers[0]), 'visibilityHelper': _identity(helpers[1]),
               'policy': policy_identity, 'originalLoader': frozen[main_loader],
               'preservedLoader': _identity(original), 'candidateLoader': _identity(candidate),
               'nativeArtifacts': [frozen[main_wasm], frozen[side_wasm]],
               'mainInputDefinitions': list(recorded_inputs.values()), 'audit': audit,
               'inspectionNode': list(map(str, node)),
               'nativeInterfaceSha256': hashlib.sha256(json.dumps(modules, sort_keys=True).encode()).hexdigest()}
    _write_json(work_dir / 'audit.json', receipt)
    for path, identity in frozen.items():
        if _identity(path) != identity:
            raise ValueError('Frozen input changed during loader pruning: ' + str(path))
    if _identity(candidate) != receipt['candidateLoader']:
        raise ValueError('Derived loader changed before installation')
    # Atomic replacement happens only after every graph, source and identity guard.
    with tempfile.NamedTemporaryFile(prefix='.ovstage-pruned-', dir=main_loader.parent, delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(candidate_bytes)
    try:
        os.replace(temporary, main_loader)
    finally:
        temporary.unlink(missing_ok=True)
    for path, identity in frozen.items():
        if path != main_loader and _identity(path) != identity:
            raise ValueError('Frozen input changed during loader installation: ' + str(path))
    if (main_loader.read_bytes() != candidate_bytes
            or _identity(candidate) != receipt['candidateLoader']):
        raise ValueError('Derived loader changed during installation')
    receipt.update(applied=True, nativeArtifactsUnchanged=True, originalInputsUnchanged=True,
                   installedLoader=_identity(main_loader))
    _write_json(work_dir / 'audit.json', receipt)
    return receipt
