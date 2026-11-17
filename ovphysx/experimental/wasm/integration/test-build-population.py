#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Prevent silently linking incompatible stage and physics exception ABIs."""
import contextlib
import hashlib
import io
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

SPEC = importlib.util.spec_from_file_location('population_link', Path(__file__).with_name('build-population.py'))
BUILD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILD)
FLAGS = {
    'javascript': ['-fexceptions'],
    'wasm-legacy': ['-fwasm-exceptions', '-sWASM_LEGACY_EXCEPTIONS=1'],
    'wasm': ['-fwasm-exceptions', '-sWASM_LEGACY_EXCEPTIONS=0'],
}


class PhysicsCapabilityReceiptTest(unittest.TestCase):
    def test_population_only_cannot_advertise_a_solver_setter(self):
        self.assertIsNone(BUILD.solver_configuration(False, 0))
        for default in [0, 2, 4]:
            with self.subTest(default=default):
                capability = BUILD.solver_configuration(True, default)
                self.assertEqual(capability['nativeDefaultWorkers'], default)
                self.assertEqual(capability['supportedWorkers'], [0, 2, 4])
                self.assertEqual(capability['setter'], 'ovp_wasm_configure_solver_threads')
                self.assertEqual(capability['capacityHeadroom'], 4)


class ExceptionAbiTest(unittest.TestCase):
    def test_matching_modes_preserve_unrelated_compile_options(self):
        for mode, flags in FLAGS.items():
            with self.subTest(mode=mode):
                self.assertEqual(BUILD.validate_runtime_exception_abi(mode, {
                    'flags': ['em++', '-O1', '-pthread', '-fPIC', *flags],
                }), mode)
        self.assertEqual(BUILD.exception_mode_from_flags(['-fwasm-exceptions'], 'pinned default'), 'wasm-legacy')

    def test_every_mixed_exception_pair_is_rejected(self):
        for stage_mode in FLAGS:
            for runtime_mode, flags in FLAGS.items():
                if runtime_mode == stage_mode:
                    continue
                with self.subTest(stage=stage_mode, runtime=runtime_mode):
                    with self.assertRaisesRegex(ValueError, 'does not match OVStage'):
                        BUILD.validate_runtime_exception_abi(stage_mode, {'flags': flags})

    def test_missing_disabled_or_ambiguous_receipts_fail_closed(self):
        invalid = [None, [], {}, {'flags': '-fexceptions'}, {'flags': []},
                   {'flags': ['-fexceptions', '-fwasm-exceptions']},
                   {'flags': ['-fexceptions', '-fno-exceptions']},
                   {'flags': ['-fwasm-exceptions', '-sWASM_LEGACY_EXCEPTIONS=2']},
                   {'flags': ['-fwasm-exceptions', '-sWASM_LEGACY_EXCEPTIONS=0', '-sWASM_LEGACY_EXCEPTIONS=1']},
                   {'flags': ['-fexceptions', '-sWASM_LEGACY_EXCEPTIONS=1']}]
        for receipt in invalid:
            with self.subTest(receipt=receipt), self.assertRaises(ValueError):
                BUILD.validate_runtime_exception_abi('javascript', receipt)

    def test_mismatch_rejects_before_any_compiler_or_schema_work(self):
        with tempfile.TemporaryDirectory(prefix='ovphysx-eh-guard-') as directory:
            root = Path(directory)
            build, runtime, output = root / 'stage', root / 'runtime', root / 'output'
            metadata = build / 'CMakeFiles/population_wasm.dir'
            metadata.mkdir(parents=True)
            runtime.mkdir()
            flags = ['-sMEMORY64=1', '-pthread', *FLAGS['wasm-legacy']]
            (metadata / 'flags.make').write_text('CXX_DEFINES = \nCXX_INCLUDES = \nCXX_FLAGS = ' + ' '.join(flags) + '\n')
            (metadata / 'link.txt').write_text('em++ ' + ' '.join(flags) + '\n')
            (runtime / 'stage-build-inputs.json').write_text(json.dumps({'flags': FLAGS['javascript']}))
            arguments = ['build-population.py', '--ovstage-build', str(build), '--runtime-dir', str(runtime), '--out', str(output), '--shared']
            with mock.patch.object(sys, 'argv', arguments), mock.patch.object(BUILD.subprocess, 'run') as compiler:
                with self.assertRaisesRegex(ValueError, 'javascript does not match OVStage wasm-legacy'):
                    BUILD.main()
                compiler.assert_not_called()
            self.assertFalse((output / 'schemas').exists())
            self.assertFalse((output / 'commands.json').exists())


class RuntimeObjectAbiTest(unittest.TestCase):
    def write_object(self, root, name='api_first', bits=16, mode='wasm-legacy', math='scalar', payload=b'first object'):
        directory = root / 'runtime-objects'
        directory.mkdir(exist_ok=True)
        obj = directory / (name + '.o')
        obj.write_bytes(payload)
        receipt = {'group': name.split('_', 1)[0], 'exitCode': 0,
                   'sdfPathPoolRegionBits': bits,
                   'command': ['em++', *FLAGS[mode], '-DPX_SIMD_DISABLED=1' if math == 'scalar' else '-msse2'],
                   'object': {'path': str(obj), 'bytes': len(payload), 'sha256': hashlib.sha256(payload).hexdigest()}}
        (directory / (name + '.json')).write_text(json.dumps(receipt))
        return obj

    def test_legacy_pool_receipt_only_matches_original_abi(self):
        BUILD.validate_runtime_pool_abi(8, {})
        for stage, receipt in [(16, {}), (8, {'sdfPathPoolRegionBits': 16}),
                               (16, {'sdfPathPoolRegionBits': 12}), (16, None)]:
            with self.subTest(stage=stage, receipt=receipt), self.assertRaises(ValueError):
                BUILD.validate_runtime_pool_abi(stage, receipt)

    def test_partial_rebuild_rejects_one_stale_object_even_with_matching_top_level_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_object(root)
            BUILD.validate_runtime_pool_abi(16, {'sdfPathPoolRegionBits': 16})
            self.assertEqual(BUILD.validate_runtime_object_abis(root, 'wasm-legacy', 16, 'scalar'), 1)
            for field, value in [('bits', 8), ('mode', 'javascript'), ('math', 'sse2')]:
                with self.subTest(field=field):
                    self.write_object(root, name='api_second', **{field: value})
                    with self.assertRaises(ValueError):
                        BUILD.validate_runtime_object_abis(root, 'wasm-legacy', 16, 'scalar')

    def test_object_identity_and_missing_receipts_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaisesRegex(ValueError, 'No completed'):
                BUILD.validate_runtime_object_abis(root, 'wasm-legacy', 16, 'scalar')
            obj = self.write_object(root)
            obj.write_bytes(b'other object')
            with self.assertRaisesRegex(ValueError, 'no longer matches'):
                BUILD.validate_runtime_object_abis(root, 'wasm-legacy', 16, 'scalar')
            with self.assertRaisesRegex(ValueError, 'Unknown runtime SDK math'):
                BUILD.validate_runtime_object_abis(root, 'wasm-legacy', 16, 'unknown')

    @unittest.skipUnless(shutil.which('ar'), 'Archive validation requires a system ar')
    def test_stale_archive_and_member_set_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            obj = self.write_object(root)
            archive = obj.parent / 'libOvPhysx_api.a'
            subprocess.run(['ar', 'rcs', str(archive), str(obj)], check=True)
            BUILD.validate_runtime_archives(root, 'ar', groups=['api'])
            self.write_object(root, payload=b'new compiled object')
            with self.assertRaisesRegex(ValueError, 'stale or changed'):
                BUILD.validate_runtime_archives(root, 'ar', groups=['api'])
            self.write_object(root, name='api_second')
            with self.assertRaisesRegex(ValueError, 'members differ'):
                BUILD.validate_runtime_archives(root, 'ar', groups=['api'])


class ArtifactAbiPackagingTest(unittest.TestCase):
    def fixtures(self, root):
        out = root / 'runtime'
        out.mkdir()
        raw = {}
        for name in ('ovstage.mjs', 'ovstage.wasm', 'ovphysx.wasm'):
            data = ('raw fixture ' + name).encode()
            (out / name).write_bytes(data)
            raw[name] = {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
        helper = root / 'source/tools/prepare_abi_pair.py'
        loader = root / 'source/population/js/compiled-artifacts.mjs'
        helper.parent.mkdir(parents=True)
        loader.parent.mkdir(parents=True)
        helper.write_bytes(b'fixture preparer')
        loader.write_bytes(b'validated fixture compiled-artifacts.mjs')
        receipt = {'artifacts': raw, 'kind': 'official-ovphysx-dylink-runtime',
                   'inputs': {str(path): BUILD.sha(path) for path in (helper, loader)}}
        (out / 'build-info.json').write_text(json.dumps(receipt))
        return out, receipt

    def prepare_fixture(self, out, receipt, invalid=None):
        def run(command, label):
            target = Path(command[command.index('--output') + 1])
            target.mkdir()
            artifacts = {}
            for name in ('ovstage.mjs', 'ovstage.wasm', 'ovphysx.wasm', 'compiled-artifacts.mjs'):
                data = ('validated fixture ' + name).encode()
                (target / name).write_bytes(data)
                artifacts[name] = {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
            proof = {'kind': 'experimental-validated-compiled-artifact-pair', 'protocol': 1,
                     'sourceReceiptSha256': BUILD.sha(out / 'build-info.json'),
                     'sourceArtifacts': receipt['artifacts'], 'artifacts': artifacts,
                     'helperSha256': BUILD.sha(Path(command[1]))}
            if invalid == 'source':
                proof['sourceReceiptSha256'] = '0' * 64
            elif invalid == 'closure':
                del proof['artifacts']['compiled-artifacts.mjs']
            elif invalid == 'helper':
                proof['helperSha256'] = '0' * 64
            elif invalid == 'loader':
                source = Path(command[1]).parent.parent / 'population/js/compiled-artifacts.mjs'
                source.write_bytes(b'concurrently edited loader')
            elif invalid == 'tamper':
                (target / 'ovphysx.wasm').write_bytes(b'changed after preparation')
            (target / 'abi-build-info.json').write_text(json.dumps(proof))
        return run

    def test_validated_closure_replaces_raw_files_and_preserves_source_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            out, receipt = self.fixtures(Path(directory))
            original_receipt = (out / 'build-info.json').read_bytes()
            result = BUILD.prepare_artifact_abi(out, out.parent / 'source/tools/prepare_abi_pair.py',
                                              self.prepare_fixture(out, receipt), receipt)
            self.assertEqual((out / 'raw-build-info.json').read_bytes(), original_receipt)
            self.assertEqual(json.loads((out / 'build-info.json').read_text()), result)
            self.assertEqual(result['artifactAbi']['sourceArtifacts'], receipt['artifacts'])
            self.assertEqual(set(result['artifacts']), set(receipt['artifacts']) | {'compiled-artifacts.mjs'})
            for name, identity in result['artifacts'].items():
                self.assertEqual(BUILD.sha(out / name), identity['sha256'])

    def test_invalid_prepared_set_leaves_original_runtime_and_receipt_intact(self):
        for invalid in ('source', 'closure', 'tamper', 'helper', 'loader'):
            with self.subTest(invalid=invalid), tempfile.TemporaryDirectory() as directory:
                out, receipt = self.fixtures(Path(directory))
                before = {path.name: path.read_bytes() for path in out.iterdir()}
                with self.assertRaises(ValueError):
                    BUILD.prepare_artifact_abi(out, out.parent / 'source/tools/prepare_abi_pair.py',
                                               self.prepare_fixture(out, receipt, invalid), receipt)
                self.assertEqual({name: (out / name).read_bytes() for name in before}, before)
                self.assertFalse((out / 'compiled-artifacts.mjs').exists())

    def test_validation_requires_shared_runtime_before_creating_build_outputs(self):
        for option in ('--validate-artifact-abi', '--side-private-functions'):
            with self.subTest(option=option), tempfile.TemporaryDirectory() as directory:
                out = Path(directory) / 'output'
                argv = ['build-population.py', option, '--out', str(out)]
                with mock.patch.object(sys, 'argv', argv), contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit):
                        BUILD.main()
                self.assertFalse(out.exists())

    def test_existing_visibility_proof_is_rejected_before_previous_outputs_are_replaced(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory) / 'output'
            (out / 'private-functions').mkdir(parents=True)
            previous = out / 'ovstage.wasm'
            previous.write_bytes(b'previous qualified artifact')
            argv = ['build-population.py', '--side-private-functions', '--shared',
                    '--runtime-dir', str(Path(directory) / 'runtime'), '--out', str(out)]
            with mock.patch.object(sys, 'argv', argv), mock.patch.object(BUILD.subprocess, 'run') as compiler:
                with self.assertRaisesRegex(ValueError, 'output must be fresh'):
                    BUILD.main()
                compiler.assert_not_called()
            self.assertEqual(previous.read_bytes(), b'previous qualified artifact')


class LoaderPruningOptionTest(unittest.TestCase):
    def test_pruning_requires_native_visibility_before_any_build_work(self):
        with mock.patch.object(sys, 'argv', ['build-population.py', '--prune-side-stubs']), \
                mock.patch.object(BUILD.subprocess, 'run') as compiler, \
                contextlib.redirect_stderr(io.StringIO()) as errors:
            with self.assertRaises(SystemExit) as rejected:
                BUILD.main()
            self.assertEqual(rejected.exception.code, 2)
            self.assertIn('--prune-side-stubs requires --side-private-functions', errors.getvalue())
            compiler.assert_not_called()


class StartupPoolPolicyTest(unittest.TestCase):
    def test_fixed_pool_default_preserves_original_command(self):
        command = ['em++', '-pthread', '-sPTHREAD_POOL_SIZE=8', '-o', 'ovstage.mjs']
        result, maximum, policy = BUILD.configure_pthread_pool(command)
        self.assertEqual(result, command)
        self.assertIsNot(result, command)
        self.assertEqual(maximum, 8)
        self.assertIsNone(policy)
        fixed, maximum, policy = BUILD.configure_pthread_pool(command, maximum=16)
        self.assertEqual(fixed, ['em++', '-pthread', '-sPTHREAD_POOL_SIZE=16', '-o', 'ovstage.mjs'])
        self.assertIsNone(policy)
        self.assertEqual(command[2], '-sPTHREAD_POOL_SIZE=8')

    def test_shared_policy_records_initial_stage_and_declared_library_counts(self):
        command = ['em++', '-pthread', '-sPTHREAD_POOL_SIZE=8', '-o', 'ovstage.mjs']
        result, maximum, policy = BUILD.configure_pthread_pool(command, maximum=16, stage_only=8, shared=True)
        self.assertEqual(maximum, 16)
        self.assertEqual((policy['stageOnly'], policy['requestedLibraries'], policy['maximum']), (8, 16, 16))
        self.assertEqual(result[2], '-sPTHREAD_POOL_SIZE=' + policy['expression'])
        self.assertEqual(result[:2] + result[3:], command[:2] + command[3:])
        self.assertEqual(command[2], '-sPTHREAD_POOL_SIZE=8')

    def test_pool_constraints_reject_unsafe_or_unqualified_startup_capacity(self):
        command = ['em++', '-sPTHREAD_POOL_SIZE=8']
        cases = [{'stage_only': 8}, {'stage_only': 0, 'shared': True},
                 {'stage_only': -1, 'shared': True}, {'stage_only': 7, 'shared': True},
                 {'stage_only': 8.5, 'shared': True}, {'stage_only': 16, 'shared': True},
                 {'maximum': 7, 'solver_workers': 4}, {'maximum': 3}, {'maximum': True}]
        for options in cases:
            with self.subTest(options=options), self.assertRaises(ValueError):
                BUILD.configure_pthread_pool(command, **options)
        for invalid in [[], command + ['-sPTHREAD_POOL_SIZE=16'], ['em++', '-sPTHREAD_POOL_SIZE=dynamic()']]:
            with self.subTest(command=invalid), self.assertRaises(ValueError):
                BUILD.configure_pthread_pool(invalid, stage_only=8, shared=True)

    def test_cli_policy_requires_shared_mode_before_build_work(self):
        with mock.patch.object(sys, 'argv', ['build-population.py', '--stage-only-pthread-pool-size', '8']), \
                mock.patch.object(BUILD.subprocess, 'run') as compiler, \
                contextlib.redirect_stderr(io.StringIO()) as errors:
            with self.assertRaises(SystemExit):
                BUILD.main()
            self.assertIn('--stage-only-pthread-pool-size requires --shared', errors.getvalue())
            compiler.assert_not_called()


if __name__ == '__main__':
    unittest.main()
