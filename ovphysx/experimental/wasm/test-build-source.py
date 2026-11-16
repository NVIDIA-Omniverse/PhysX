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


if __name__ == '__main__':
    unittest.main()
