# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OVD-NAMES-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-1 AC-2
"""Dependency-free public configuration and native declaration migration checks."""
import dataclasses
import importlib
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'python'))
from ovphysx.config import PhysXConfig, OmniPvdDestination
from ovphysx.types import ConfigBool, ConfigInt32, ConfigString, GlobalConfigKey

REMOVED = ('omnipvd_output_enabled', 'omnipvd_ovd_recording_directory',
           'omnipvd_transport', 'omnipvd_tcp_address', 'omnipvd_tcp_port',
           'omnipvd_tcp_timeout_ms', 'omnipvd_recording_capable')

class PublicOvdApiTests(unittest.TestCase):
    def test_global_recording_capability_key(self):
        self.assertEqual(GlobalConfigKey.OVD_RECORDING_CAPABLE, 0x0000000C)
        self.assertEqual(GlobalConfigKey.DEFAULT_LOG_OUTPUT, 0x0000000B)

    def test_normal_config_rejects_startup_controls(self):
        fields = {field.name for field in dataclasses.fields(PhysXConfig)}
        for name in REMOVED:
            with self.subTest(name=name):
                self.assertNotIn(name, fields)
                with self.assertRaises(TypeError):
                    PhysXConfig(**{name: True})

    def test_unrelated_numbers_remain_stable(self):
        self.assertEqual(ConfigInt32.OVSTAGE_READ_POOL_MAX_MB, 4)
        self.assertEqual(ConfigInt32.PROFILER_BACKEND, 5)
        self.assertEqual(ConfigString.COOKED_COLLIDER_CACHE_DIRECTORY, 1)
        self.assertEqual(ConfigString.PROFILER_OUTPUT_PATH, 4)

    def test_current_enums_exclude_legacy_values(self):
        self.assertTrue({3, 4}.isdisjoint(int(value) for value in ConfigBool))
        self.assertTrue({2, 3}.isdisjoint(int(value) for value in ConfigInt32))
        self.assertTrue({0, 2, 3}.isdisjoint(int(value) for value in ConfigString))

    def test_retired_startup_module_is_not_importable(self):
        with self.assertRaises(ModuleNotFoundError) as error:
            importlib.import_module('ovphysx.legacy_ovd_config')
        self.assertEqual(error.exception.name, 'ovphysx.legacy_ovd_config')

    def test_destination_remains_explicit(self):
        self.assertEqual(OmniPvdDestination.file('capture.ovd').file_path, 'capture.ovd')
        self.assertEqual(OmniPvdDestination.tcp('localhost', 5425).tcp_port, 5425)
        with self.assertRaises(ValueError):
            OmniPvdDestination.tcp('localhost', 0)

    def test_canonical_native_declarations(self):
        types = (ROOT / 'include/ovphysx/ovphysx_types.h').read_text()
        header = (ROOT / 'include/ovphysx/ovphysx.h').read_text()
        self.assertTrue('typedef uint64_t ovphysx_instance_handle_t;' in types)
        for verb in ('start', 'stop', 'is'):
            self.assertIn('ovphysx_ovd_' + verb + '_recording(', header)

if __name__ == '__main__':
    unittest.main()
