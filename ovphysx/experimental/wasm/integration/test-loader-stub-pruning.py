#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Preserve loader/native identity while deleting only obsolete private stubs."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import loader_stub_pruning as helper


def stub(name):
    return f'function _{name}(...args){{return wasmImports["{name}"](...args)}}_{name}.stub=true;'


PREFIX = '// Preserve CRLF and unrelated literal metadata.\r\nvar sharedModules={};var ASM_CONSTS={17:()=>123n};'
TAIL = '}}function applySignatureConversions(wasmExports){return wasmExports}export default createNativePopulationModule;'
SOURCE = PREFIX + stub('private_a') + stub('shared_fn') + 'var wasmImports;function assignWasmImports(){wasmImports={private_a:_private_a,shared_fn:_shared_fn' + TAIL
EXPECTED = PREFIX + stub('shared_fn') + 'var wasmImports;function assignWasmImports(){wasmImports={shared_fn:_shared_fn' + TAIL
MODULES = [
    {'imports': [], 'exports': [{'name': 'shared_fn', 'kind': 'function'}, {'name': 'shared_data', 'kind': 'global'}]},
    {'imports': [{'name': 'shared_fn', 'kind': 'function', 'module': 'env'}],
     'exports': [{'name': 'ovp_wasm_create', 'kind': 'function'}, {'name': 'shared_data', 'kind': 'global'}]},
]
POLICY = {'functions': ['private_a'], 'protected': ['shared_fn', 'shared_data', 'ovp_wasm_create'],
          'publicFunctions': ['ovp_wasm_create']}


class SourceTest(unittest.TestCase):
    def test_removal_preserves_other_functions_metadata_and_line_endings(self):
        result, audit = helper.prune_source(SOURCE, {'private_a'})
        self.assertEqual(result, EXPECTED)
        self.assertEqual(audit['removedFunctions'], ['private_a'])
        self.assertEqual(audit['removedSourceBytes'], len(SOURCE.encode()) - len(EXPECTED.encode()))

    def test_unknown_duplicate_or_mismatched_generated_syntax_fails_closed(self):
        candidates = [SOURCE + stub('private_a'),
                      SOURCE.replace('shared_fn:_shared_fn', 'shared_fn:_shared_fn,shared_fn:_shared_fn'),
                      SOURCE.replace('private_a:_private_a', 'private_a:_wrong'),
                      SOURCE.replace('private_a:_private_a', 'private_a:()=>42'),
                      SOURCE.replace('function _private_a(...args)', 'function _private_a( ...args )'),
                      SOURCE.replace('}}function applySignatureConversions(', '};function applySignatureConversions('),
                      SOURCE.replace('export default createNativePopulationModule;', ''),
                      SOURCE + 'const artifactDescription={};']
        for candidate in candidates:
            with self.subTest(candidate=candidate), self.assertRaises(ValueError):
                helper.prune_source(candidate, {'private_a'})

    def test_remaining_source_references_and_no_gain_reject(self):
        for suffix in [';_private_a();', ';const callback=_private_a;', ';const name="private_a";']:
            with self.subTest(suffix=suffix), self.assertRaisesRegex(ValueError, 'retains'):
                helper.prune_source(SOURCE + suffix, {'private_a'})
        for selected in [set(), {'unknown'}]:
            with self.assertRaisesRegex(ValueError, 'No gain'):
                helper.prune_source(SOURCE, selected)


class IntegrationTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='ovphysx-prune-test-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.loader, self.main, self.side, self.obj, self.policy = [self.root / name for name in
            ['ovstage.mjs', 'ovstage.wasm', 'ovphysx.wasm', 'main.o', 'policy.json']]
        self.loader.write_bytes(SOURCE.encode())
        self.main.write_bytes(b'unchanged main native bytes')
        self.side.write_bytes(b'unchanged optimized side native bytes')
        self.obj.write_bytes(b'unchanged main input')
        self.policy.write_text(json.dumps(POLICY))
        self.proof = {'protocol': 1, 'applied': True, 'mainArtifactsUnchanged': True,
                      'originalInputsUnchanged': True, 'selectedFunctions': 1,
                      'mainArtifacts': [helper._identity(self.loader), helper._identity(self.main)],
                      'installedSide': helper._identity(self.side),
                      'mainInputDefinitions': [helper._identity(self.obj)],
                      'policy': helper._identity(self.policy),
                      'audit': {'removedImports': [{'name': 'private_a', 'kind': 'function', 'module': 'env'},
                                                  {'name': 'private_a', 'kind': 'global', 'module': 'GOT.func'}]}}
        self.command = ['em++', str(self.obj), str(self.side), '-sMAIN_MODULE=2', '-sAUTOLOAD_DYLIBS=0', '-o', str(self.loader)]

    def run_helper(self, modules=None, inspect=None):
        with patch.object(helper._visibility, '_inspect_modules', side_effect=inspect, return_value=modules or MODULES):
            return helper.prune_side_stubs(main_loader=self.loader, main_wasm=self.main, side_wasm=self.side,
                visibility=self.proof, main_link=self.command, cwd=self.root, node=['configured-node'],
                work_dir=self.root / 'audit')

    def assert_original_loader(self):
        self.assertEqual(self.loader.read_bytes(), SOURCE.encode())

    def test_success_keeps_native_bytes_and_records_both_phases(self):
        before = (self.main.read_bytes(), self.side.read_bytes(), self.obj.read_bytes())
        receipt = self.run_helper()
        self.assertTrue(receipt['applied'])
        self.assertTrue(receipt['nativeArtifactsUnchanged'])
        self.assertEqual(self.loader.read_bytes(), EXPECTED.encode())
        self.assertEqual((self.main.read_bytes(), self.side.read_bytes(), self.obj.read_bytes()), before)
        self.assertEqual((self.root / 'audit/unpruned-ovstage.mjs').read_bytes(), SOURCE.encode())
        self.assertEqual(receipt['installedLoader'], helper._identity(self.loader))
        self.assertTrue(self.proof['mainArtifactsUnchanged'], 'Earlier visibility phase remains independently recorded')
        with self.assertRaisesRegex(ValueError, 'fresh'):
            self.run_helper()

    def test_unnamed_object_definitions_remain_protected_but_are_not_selected(self):
        policy = copy.deepcopy(POLICY)
        policy['protected'].append('')
        self.policy.write_text(json.dumps(policy))
        self.proof['policy'] = helper._identity(self.policy)
        self.assertTrue(self.run_helper()['applied'])
        for key in ['functions', 'publicFunctions']:
            invalid = copy.deepcopy(policy)
            invalid[key].append('')
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'Invalid visibility policy'):
                helper._selected_functions(self.proof, invalid, MODULES)

    def test_current_native_interfaces_prevent_pruning_any_kind_of_shared_identity(self):
        for module_index in [0, 1]:
            for kind, category in [('function', 'imports'), ('global', 'imports'), ('function', 'exports'), ('global', 'exports')]:
                modules = copy.deepcopy(MODULES)
                modules[module_index][category].append({'name': 'private_a', 'kind': kind, 'module': 'GOT.func' if kind == 'global' else 'env'})
                with self.subTest(module=module_index, kind=kind, category=category), self.assertRaisesRegex(ValueError, 'actual main/side'):
                    self.run_helper(modules)
                self.assert_original_loader()

    def test_missing_public_export_and_protected_policy_reject_before_install(self):
        modules = copy.deepcopy(MODULES)
        modules[1]['exports'] = [row for row in modules[1]['exports'] if row['name'] != 'ovp_wasm_create']
        with self.assertRaisesRegex(ValueError, 'missing declared public'):
            self.run_helper(modules)
        policy = copy.deepcopy(POLICY); policy['protected'].append('private_a')
        self.policy.write_text(json.dumps(policy)); self.proof['policy'] = helper._identity(self.policy)
        with self.assertRaisesRegex(ValueError, 'protected'):
            self.run_helper()
        self.assert_original_loader()

    def test_nonfunction_got_and_unmatched_function_address_removals_reject(self):
        for name, module in [('private_a', 'GOT.mem'), ('another', 'GOT.func')]:
            self.proof['audit']['removedImports'][1] = {'name': name, 'kind': 'global', 'module': module}
            with self.subTest(module=module), self.assertRaises(ValueError):
                self.run_helper()
            self.assert_original_loader()

    def test_changed_artifacts_policy_or_main_inputs_reject_before_native_inspection(self):
        for path in [self.main, self.side, self.policy, self.obj]:
            original = path.read_bytes(); path.write_bytes(original + b'changed')
            with self.subTest(path=path), patch.object(helper._visibility, '_inspect_modules') as inspect:
                with self.assertRaisesRegex(ValueError, 'changed before pruning'):
                    helper.prune_side_stubs(main_loader=self.loader, main_wasm=self.main, side_wasm=self.side,
                        visibility=self.proof, main_link=self.command, cwd=self.root, node=['configured-node'], work_dir=self.root/'audit')
                inspect.assert_not_called()
            path.write_bytes(original)
            self.assert_original_loader()

    def test_extra_dso_and_changed_main_input_set_reject(self):
        for extra in [str(self.root/'other.wasm'), str(self.root/'extra.o'), '@unexpanded-response']:
            if extra.endswith('.o'): Path(extra).write_bytes(b'new input')
            self.command.insert(1, extra)
            with self.subTest(extra=extra), self.assertRaises(ValueError):
                self.run_helper()
            self.command.remove(extra)
            self.assert_original_loader()

    def test_mutation_during_native_inspection_is_detected_before_replacing_loader(self):
        def mutate(*args):
            self.obj.write_bytes(b'concurrent source change')
            return MODULES
        with self.assertRaisesRegex(ValueError, 'changed during loader pruning'):
            self.run_helper(inspect=mutate)
        self.assert_original_loader()
        self.assertFalse(json.loads((self.root/'audit/audit.json').read_text())['applied'])


if __name__ == '__main__':
    unittest.main()
