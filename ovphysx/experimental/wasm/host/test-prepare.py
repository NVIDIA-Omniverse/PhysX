#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Host preparation preserves both dependency inputs, including aliased destinations."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('wasm_host_prepare', HERE / 'prepare.py')
prepare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare)

HOST_SOURCE = '''#include <shared_mutex>
carb::settings::ISettings& settings()
{
        return result;
}
carb::dictionary::IDictionary& dictionary()
{
        initializeUnsupportedDictionary(result);
        return result;
}
carb::tasking::ITasking& tasking()
{
        initializeUnsupportedTasking(result);
        return result;
}
'''


class HostInputIsolationTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='ovphysx-host-prepare-')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.carb = self.root / 'inputs' / 'carb'
        self.host = self.root / 'inputs' / 'host'
        self.recipe = self.root / 'recipe'
        for directory in (self.carb / 'carb', self.host, self.recipe):
            directory.mkdir(parents=True)
        self.header = self.carb / 'carb' / 'fixture.h'
        self.header.write_text('original header\n')
        (self.host / 'host.cpp').write_text(HOST_SOURCE)
        patches = [{'path': 'carb/fixture.h',
                    'source_sha256': hashlib.sha256(self.header.read_bytes()).hexdigest(),
                    'edits': [{'old': 'original header', 'new': 'prepared header'}]}]
        (self.recipe / 'platform-patches.json').write_text(json.dumps(patches))
        for name in ('DictionaryPayload.inl', 'SettingsArrays.inl', 'TaskStorage.inl'):
            (self.recipe / name).write_text('// ' + name + '\n')
        for patch in (mock.patch.object(prepare, 'HERE', self.recipe),
                      mock.patch.object(prepare, 'HOST_SHA256',
                                        hashlib.sha256(HOST_SOURCE.encode()).hexdigest())):
            patch.start()
            self.addCleanup(patch.stop)

    def snapshot(self):
        """Capture every existing path, file byte and link without following output aliases."""
        result = {}
        for path in self.root.rglob('*'):
            key = str(path.relative_to(self.root))
            if path.is_symlink():
                result[key] = ('link', str(path.readlink()))
            elif path.is_file():
                result[key] = ('file', path.read_bytes())
            else:
                result[key] = ('directory',)
        return result

    def assert_rejected_without_writes(self, output):
        before = self.snapshot()
        try:
            with self.assertRaisesRegex(ValueError, 'input|dependency|source|file destination'):
                prepare.prepare(self.carb, self.host, output)
        finally:
            self.assertEqual(self.snapshot(), before, 'Rejection must precede every output mutation')

    def test_direct_and_symlinked_output_inside_either_input_is_rejected(self):
        for source in (self.carb, self.host):
            for nested in (False, True):
                for alias in (False, True):
                    with self.subTest(source=source.name, nested=nested, alias=alias):
                        output = source / 'generated' if nested else source
                        if alias:
                            link = self.root / 'output-link'
                            link.symlink_to(source, target_is_directory=True)
                            output = link / 'generated' if nested else link
                        self.assert_rejected_without_writes(output)
                        if alias:
                            link.unlink()

    def test_copied_include_tree_cannot_alias_either_input(self):
        for source in (self.carb, self.host):
            with self.subTest(source=source.name):
                output = self.root / ('out-' + source.name)
                (output / 'carb').mkdir(parents=True)
                (output / 'carb' / 'include').symlink_to(source, target_is_directory=True)
                self.assert_rejected_without_writes(output)

    def test_existing_nested_copy_destinations_cannot_redirect_into_inputs(self):
        for alias_kind in ('directory', 'file', 'hardlink'):
            with self.subTest(alias_kind=alias_kind):
                output = self.root / ('nested-' + alias_kind)
                include = output / 'carb' / 'include'
                include.mkdir(parents=True)
                if alias_kind == 'directory':
                    (include / 'carb').symlink_to(self.host, target_is_directory=True)
                else:
                    (include / 'carb').mkdir()
                    destination = include / 'carb' / 'fixture.h'
                    if alias_kind == 'hardlink':
                        destination.hardlink_to(self.header)
                    else:
                        destination.symlink_to(self.header)
                self.assert_rejected_without_writes(output)

    def test_copied_file_destination_directory_cannot_redirect_copy2(self):
        output = self.root / 'file-container-alias'
        container = output / 'carb' / 'include' / 'carb' / 'fixture.h'
        container.mkdir(parents=True)
        (container / 'fixture.h').symlink_to(self.host / 'host.cpp')
        self.assert_rejected_without_writes(output)

    def test_generated_file_destinations_cannot_be_directories(self):
        for name in ('host.cpp', 'DictionaryPayload.inl', 'SettingsArrays.inl',
                     'TaskStorage.inl', 'overlay-receipt.json'):
            with self.subTest(name=name):
                output = self.root / ('directory-' + name)
                (output / name).mkdir(parents=True)
                self.assert_rejected_without_writes(output)

    def test_symlink_chain_under_copied_output_tree_is_checked(self):
        output = self.root / 'chain-output'
        redirected = self.root / 'redirected'
        redirected.mkdir()
        (redirected / 'fixture.h').symlink_to(self.header)
        include = output / 'carb' / 'include'
        include.mkdir(parents=True)
        (include / 'carb').symlink_to(redirected, target_is_directory=True)
        self.assert_rejected_without_writes(output)

    def test_generated_include_tree_cannot_contain_dependency_input(self):
        original = self.carb
        output = self.root / 'ancestor-output'
        self.carb = output / 'carb' / 'include' / 'source'
        self.carb.parent.mkdir(parents=True)
        original.rename(self.carb)
        self.assert_rejected_without_writes(output)

    def test_empty_linked_dependency_directory_is_protected(self):
        linked = self.root / 'linked-input'
        linked.mkdir()
        (self.carb / 'linked').symlink_to(linked, target_is_directory=True)
        for output in (linked, linked / 'generated'):
            with self.subTest(output=output):
                self.assert_rejected_without_writes(output)

    def test_recursive_input_links_reject_before_creating_output(self):
        (self.carb / 'recursive').symlink_to(self.carb, target_is_directory=True)
        self.assert_rejected_without_writes(self.root / 'recursive-output')

    def test_individual_generated_files_cannot_alias_inputs(self):
        for name in ('host.cpp', 'DictionaryPayload.inl', 'SettingsArrays.inl',
                     'TaskStorage.inl', 'overlay-receipt.json'):
            with self.subTest(name=name):
                output = self.root / ('file-' + name)
                output.mkdir()
                (output / name).symlink_to(self.host / 'host.cpp')
                self.assert_rejected_without_writes(output)

    def test_generated_include_cannot_overwrite_another_recipe_input(self):
        output = self.root / 'recipe-include-alias'
        output.mkdir()
        (output / 'DictionaryPayload.inl').symlink_to(self.recipe / 'TaskStorage.inl')
        self.assert_rejected_without_writes(output)

    def test_receipt_cannot_overwrite_recipe_patch_configuration(self):
        output = self.root / 'recipe-receipt-alias'
        output.mkdir()
        (output / 'overlay-receipt.json').symlink_to(self.recipe / 'platform-patches.json')
        self.assert_rejected_without_writes(output)

    def test_generated_file_cannot_hardlink_a_recipe_input(self):
        output = self.root / 'recipe-hardlink-alias'
        output.mkdir()
        (output / 'host.cpp').hardlink_to(self.recipe / 'TaskStorage.inl')
        self.assert_rejected_without_writes(output)

    def test_output_under_recipe_directory_is_allowed_when_files_are_separate(self):
        sources = {path: path.read_bytes() for path in self.recipe.iterdir()}
        result = prepare.prepare(self.carb, self.host, self.recipe / 'generated')
        self.assertTrue(result['host'].is_file())
        self.assertEqual({path: path.read_bytes() for path in sources}, sources)

    def test_unwritten_output_entries_are_not_arbitrarily_restricted(self):
        output = self.root / 'safe-reused-output'
        include = output / 'carb' / 'include'
        include.mkdir(parents=True)
        unused = include / 'unwritten-source-alias'
        unused.symlink_to(self.host, target_is_directory=True)
        source = (self.host / 'host.cpp').read_bytes()
        prepare.prepare(self.carb, self.host, output)
        self.assertTrue(unused.is_symlink())
        self.assertEqual((self.host / 'host.cpp').read_bytes(), source)

    def test_separate_output_and_common_ancestor_output_keep_sources_and_bytes(self):
        for output in (self.root / 'separate', self.root):
            with self.subTest(output=output.name):
                sources = {path: path.read_bytes() for root in (self.carb, self.host)
                           for path in root.rglob('*') if path.is_file()}
                result = prepare.prepare(self.carb, self.host, output)
                generated = {path.relative_to(output): path.read_bytes()
                             for path in [result['host'], output / 'overlay-receipt.json',
                                          result['include'] / 'carb' / 'fixture.h']}
                self.assertEqual((result['include'] / 'carb' / 'fixture.h').read_text(),
                                 'prepared header\n')
                self.assertIn('installTaskStorage(result)', result['host'].read_text())
                prepare.prepare(self.carb, self.host, output)
                self.assertEqual({path: (output / path).read_bytes() for path in generated}, generated)
                self.assertEqual({path: path.read_bytes() for path in sources}, sources)


if __name__ == '__main__':
    unittest.main()
