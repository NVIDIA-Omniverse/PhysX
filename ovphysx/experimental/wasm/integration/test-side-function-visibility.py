#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Protect shared ABI identities and immutable inputs during private-function relinking."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import side_function_visibility as helper


def uleb(value):
    result = bytearray()
    while value >= 128:
        result.append((value & 127) | 128)
        value >>= 7
    return bytes(result + bytes([value]))


def string(value):
    encoded = value.encode()
    return uleb(len(encoded)) + encoded


def object_bytes():
    symbols = (uleb(3) + b'\0\x01\0' + string('private_function')
               + b'\0\x20\x01' + string('ovp_wasm_create')
               + b'\x01\0' + string('shared_data') + b'\0\0\x04')
    linking = string('linking') + b'\x02\x08' + uleb(len(symbols)) + symbols
    payload = string('unchanged_code_and_data_fixture') + bytes(range(256))
    return b'\0asm\x01\0\0\0' + b'\0' + uleb(len(linking)) + linking + b'\0' + uleb(len(payload)) + payload


def archive_header(name, size):
    return name.encode().ljust(16) + b'0'.ljust(12) + b'0'.ljust(6) + b'0'.ljust(6) + b'644'.ljust(8) + str(size).encode().ljust(10) + b'`\n'


def imported(name, kind='function', module='env'):
    return {'name': name, 'kind': kind, 'module': module}


def exported(name, kind='function'):
    return {'name': name, 'kind': kind}


MAIN = {'imports': [], 'exports': [exported('shared_function'), exported('shared_data', 'global')]}
SIDE = {'imports': [imported('private_function'), imported('shared_function'), imported('shared_data', 'global', 'GOT.mem')],
        'exports': [exported('private_function'), exported('ovp_wasm_create'), exported('shared_data', 'global')]}
LOCAL = {'imports': SIDE['imports'][1:], 'exports': SIDE['exports'][1:]}
POLICY = {'functions': ['private_function'], 'protected': ['shared_function', 'shared_data', 'ovp_wasm_create'],
          'publicFunctions': ['ovp_wasm_create']}


class MetadataTests(unittest.TestCase):
    def test_only_defined_function_visibility_changes_and_repeated_pass_is_noop(self):
        source = object_bytes()
        result, changes = helper.transform_relocatable(source, {'private_function'}, {'shared_data', 'ovp_wasm_create'})
        self.assertEqual(len(changes), 1)
        self.assertEqual(changes[0]['before'], 1)
        self.assertEqual(changes[0]['after'], 5)
        self.assertEqual(source[:changes[0]['offset']], result[:changes[0]['offset']])
        self.assertEqual(source[changes[0]['offset'] + 1:], result[changes[0]['offset'] + 1:])
        self.assertEqual(helper.transform_relocatable(result, {'private_function'}, set()), (result, []))

    def test_data_explicit_exports_and_protected_symbols_cannot_be_hidden(self):
        for selected, protected in [({'shared_data'}, set()), ({'ovp_wasm_create'}, set()), ({'private_function'}, {'private_function'})]:
            with self.subTest(selected=selected), self.assertRaises(ValueError):
                helper.transform_relocatable(object_bytes(), selected, protected)

    def test_archive_indexes_and_offsets_stay_identical(self):
        obj, symbol_index = object_bytes(), b'index-preserved0'
        archive = b'!<arch>\n' + archive_header('/', len(symbol_index)) + symbol_index + archive_header('member.o/', len(obj)) + obj + (b'\n' if len(obj) & 1 else b'')
        result, changes = helper.transform_relocatable(archive, {'private_function'}, set())
        self.assertEqual(len(changes), 1)
        self.assertEqual(len(result), len(archive))
        self.assertEqual(sum(a != b for a, b in zip(archive, result)), 1)
        self.assertEqual(result[:8 + 60 + len(symbol_index)], archive[:8 + 60 + len(symbol_index)])

    def test_malformed_and_unsupported_objects_archives_reject(self):
        cases = [object_bytes()[:-1], b'!<arch>\n' + archive_header('/', -60),
                 b'!<arch>\n' + archive_header('x.o/', 900),
                 b'!<arch>\n' + archive_header('#1/99', 8) + b'12345678',
                 b'!<thin>\n', b'BC\xc0\xde']
        for value in cases:
            with self.subTest(value=value[:70]), self.assertRaises(ValueError):
                helper.transform_relocatable(value, {'private_function'}, set())

    def test_policy_protects_nonexported_main_definitions_and_linker_public_functions(self):
        side = {'imports': [imported(name) for name in ['private_function', 'main_internal', 'explicit_api']],
                'exports': [exported(name) for name in ['private_function', 'main_internal', 'explicit_api']]}
        policy = helper._derive_policy(MAIN, side, {'explicit_api'}, {'main_internal'})
        self.assertEqual(policy['functions'], ['private_function'])
        self.assertTrue({'main_internal', 'explicit_api', 'shared_data'} <= set(policy['protected']))

    def test_relink_must_preserve_public_global_and_shared_symbols(self):
        self.assertEqual(helper.audit_modules(SIDE, LOCAL, POLICY)['functionImportsAfter'], 1)
        for after in [
            {'imports': LOCAL['imports'], 'exports': [exported('different_api')]},
            {'imports': LOCAL['imports'], 'exports': [exported('ovp_wasm_create')]},
            {'imports': [], 'exports': LOCAL['exports']},
            {'imports': [*LOCAL['imports'], imported('new_dependency')], 'exports': LOCAL['exports']},
            SIDE,
        ]:
            with self.subTest(after=after), self.assertRaises(ValueError):
                helper.audit_modules(SIDE, after, POLICY)
        with self.assertRaisesRegex(ValueError, 'did not reduce'):
            helper.audit_modules(LOCAL, LOCAL, {**POLICY, 'functions': []})


class ConfiguredNodeTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.compiler = self.root / 'em++'
        self.compiler.write_text('compiler placeholder')
        (self.root / 'em-config.py').write_text('configuration placeholder')

    def test_uses_configured_executable_arguments_and_memory64_probe(self):
        engine = str(Path(sys.executable).resolve())
        calls = []
        def run(command, **options):
            calls.append((command, options))
            return subprocess.CompletedProcess(command, 0,
                repr([engine, '--configured-flag']) if len(calls) == 1 else 'ovstage-memory64-probe-ok\n', '')
        with patch.object(helper.subprocess, 'run', side_effect=run):
            result = helper.resolve_emscripten_node(self.compiler, env={'EM_CONFIG': 'pinned-config'})
        self.assertEqual(result, [engine, '--configured-flag'])
        self.assertEqual(calls[0][0][-1], 'NODE_JS')
        self.assertEqual(calls[1][0][:3], [engine, '--configured-flag', '-e'])
        self.assertIn('new WebAssembly.Module', calls[1][0][3])
        self.assertEqual(calls[1][1]['env'], {'EM_CONFIG': 'pinned-config'})

    def test_relative_or_invalid_configuration_never_falls_back_to_system_node(self):
        for encoded in ["['node']", "[]", "['/missing/node']", "{'NODE_JS': 'node'}", 'bad config']:
            with self.subTest(encoded=encoded), patch.object(helper.subprocess, 'run',
                    return_value=subprocess.CompletedProcess([], 0, encoded, '')) as run:
                with self.assertRaises(ValueError):
                    helper.resolve_emscripten_node(self.compiler)
                self.assertEqual(run.call_count, 1)

    def test_engine_without_actual_memory64_support_rejects(self):
        engine = str(Path(sys.executable).resolve())
        with patch.object(helper.subprocess, 'run', side_effect=[
            subprocess.CompletedProcess([], 0, repr([engine]), ''),
            subprocess.CompletedProcess([], 1, '', 'CompileError: invalid memory limits flags'),
        ]):
            with self.assertRaisesRegex(ValueError, 'cannot compile shared memory64/table64'):
                helper.resolve_emscripten_node(self.compiler)


class InstallationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.main = self.root / 'main.wasm'
        self.loader = self.root / 'main.mjs'
        self.side = self.root / 'physics.wasm'
        self.main_object = self.root / 'main.o'
        self.side_object = self.root / 'side.o'
        for path, data in [(self.main, b'main image'), (self.loader, b'main loader'), (self.side, b'original side'),
                           (self.main_object, object_bytes()), (self.side_object, object_bytes())]:
            path.write_bytes(data)
        self.main_link = ['em++', '-sMAIN_MODULE=2', '-sAUTOLOAD_DYLIBS=0', str(self.main_object), str(self.side), '-o', str(self.loader)]
        self.side_link = ['em++', '-sSIDE_MODULE=2', '-sEXPORTED_FUNCTIONS=["_ovp_wasm_create"]', str(self.side_object), '-o', str(self.side)]
        self.kwargs = dict(main_wasm=self.main, main_loader=self.loader, side_wasm=self.side,
                           main_link=self.main_link, side_link=self.side_link, cwd=self.root, work_dir=self.root/'visibility',
                           node=['node'], llvm_nm=Path('llvm-nm'))
        self.before = {path: path.read_bytes() for path in [self.main, self.loader, self.main_object, self.side_object]}
        self.commands = []

    def run_link(self, command, name):
        self.commands.append((command, name))
        self.assertNotIn(str(self.side_object), command)
        derived = next(Path(value) for value in command if value.endswith('.o'))
        self.assertNotEqual(derived.read_bytes(), self.side_object.read_bytes())
        Path(command[command.index('-o') + 1]).write_bytes(b'qualified side')

    def inspect(self, node, files):
        return [MAIN, SIDE] if len(files) == 2 else [LOCAL]

    def assert_original_inputs(self):
        for path, data in self.before.items():
            self.assertEqual(path.read_bytes(), data, str(path))

    def call(self, **changes):
        with patch.object(helper, '_inspect_modules', side_effect=self.inspect), patch.object(
            helper.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, 'shared_function\nshared_data\n', '')
        ):
            return helper.optimize_side_functions(**(self.kwargs | {'run': self.run_link} | changes))

    def test_audited_side_install_does_not_modify_source_inputs_or_main(self):
        receipt = self.call()
        self.assertTrue(receipt['applied'])
        self.assertEqual(self.side.read_bytes(), b'qualified side')
        self.assert_original_inputs()
        self.assertEqual((self.kwargs['work_dir']/'control/physics.wasm').read_bytes(), b'original side')
        self.assertEqual(receipt['installedSide']['sha256'], hashlib.sha256(b'qualified side').hexdigest())
        self.assertTrue((self.kwargs['work_dir']/'receipt.json').is_file())

    def test_link_failure_retains_original_side_and_inputs(self):
        def fail(command, name):
            self.run_link(command, name)
            raise RuntimeError('link failed')
        with self.assertRaisesRegex(RuntimeError, 'link failed'):
            self.call(run=fail)
        self.assertEqual(self.side.read_bytes(), b'original side')
        self.assert_original_inputs()
        self.assertFalse((self.kwargs['work_dir']/'receipt.json').exists())

    def test_rejected_abi_never_installs_candidate(self):
        with patch.object(helper, 'audit_modules', side_effect=ValueError('public API changed')):
            with self.assertRaisesRegex(ValueError, 'public API changed'):
                self.call()
        self.assertEqual(self.side.read_bytes(), b'original side')
        self.assert_original_inputs()

    def test_input_mutation_during_link_prevents_installation(self):
        def changed(command, name):
            self.run_link(command, name)
            self.main_object.write_bytes(b'external concurrent modification')
        with self.assertRaisesRegex(ValueError, 'changed during visibility relink'):
            self.call(run=changed)
        self.assertEqual(self.side.read_bytes(), b'original side')
        self.assertEqual(self.main_object.read_bytes(), b'external concurrent modification')

    def test_no_eligible_functions_records_noop_without_link(self):
        with patch.object(helper, '_derive_policy', return_value={**POLICY, 'functions': []}):
            receipt = self.call()
        self.assertFalse(receipt['applied'])
        self.assertFalse(self.commands)
        self.assertEqual(self.side.read_bytes(), b'original side')
        self.assert_original_inputs()

    def test_existing_workspace_and_unsupported_graph_do_not_replace_outputs(self):
        self.kwargs['work_dir'].mkdir()
        with self.assertRaisesRegex(ValueError, 'fresh'):
            self.call()
        with self.assertRaisesRegex(ValueError, 'Only one'):
            self.call(main_link=[*self.main_link, str(self.root/'other.wasm')])
        self.assertEqual(self.side.read_bytes(), b'original side')
        self.assert_original_inputs()

    def test_hardlinked_main_side_artifacts_reject(self):
        self.side.unlink()
        self.side.hardlink_to(self.main)
        with self.assertRaisesRegex(ValueError, 'alias'):
            self.call()
        self.assert_original_inputs()


if __name__ == '__main__':
    unittest.main()
