#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""The port accepts committed experiment descendants but rejects unrelated sources."""
import importlib.util
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('wasm_build', Path(__file__).with_name('build.py'))
build = importlib.util.module_from_spec(spec)
spec.loader.exec_module(build)


class SourceBaselineTest(unittest.TestCase):
    def test_baseline_descendant_and_unrelated_history(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)
            env = dict(os.environ, GIT_AUTHOR_NAME='Test', GIT_AUTHOR_EMAIL='test@example.invalid',
                       GIT_COMMITTER_NAME='Test', GIT_COMMITTER_EMAIL='test@example.invalid')
            def git(*arguments):
                return subprocess.check_output(['git', '-C', directory, *arguments], env=env,
                                               stderr=subprocess.DEVNULL, text=True).strip()
            git('init', '-q')
            git('commit', '--allow-empty', '-qm', 'Release baseline')
            baseline = git('rev-parse', 'HEAD')
            self.assertEqual(build.verify_source_baseline(source, baseline), baseline)
            git('commit', '--allow-empty', '-qm', 'Experiment')
            descendant = git('rev-parse', 'HEAD')
            self.assertNotEqual(descendant, baseline)
            self.assertEqual(build.verify_source_baseline(source, baseline), descendant)
            git('checkout', '--orphan', 'unrelated')
            git('commit', '--allow-empty', '-qm', 'Unrelated source')
            with self.assertRaisesRegex(ValueError, 'Expected baseline'):
                build.verify_source_baseline(source, baseline)


class SdkMathModeTest(unittest.TestCase):
    def test_matching_and_mixed_static_abis(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = root / 'build/sdk/CMakeFiles/PhysX.dir/flags.make'
            second = root / 'build/sdk/CMakeFiles/PhysXCommon.dir/flags.make'
            for path in (first, second):
                path.parent.mkdir(parents=True)
                path.write_text('CXX_DEFINES = -DPX_SIMD_DISABLED=1\nCXX_FLAGS = -msimd128 -fno-exceptions\n')
            build.validate_sdk_math(root, 'scalar')
            with self.assertRaisesRegex(ValueError, 'math mode mismatch'):
                build.validate_sdk_math(root, 'sse2')
            first.write_text('CXX_DEFINES =\nCXX_FLAGS = -msimd128 -msse2 -fno-exceptions\n')
            for mode in ('scalar', 'sse2'):
                with self.subTest(mode=mode), self.assertRaisesRegex(ValueError, 'math mode mismatch'):
                    build.validate_sdk_math(root, mode)
            second.write_text(first.read_text())
            build.validate_sdk_math(root, 'sse2')

    def test_vector_build_requires_sdk_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            build.validate_sdk_math(directory, 'scalar')
            with self.assertRaisesRegex(ValueError, 'matching SIMD SDK'):
                build.validate_sdk_math(directory, 'sse2')


if __name__ == '__main__':
    unittest.main()
