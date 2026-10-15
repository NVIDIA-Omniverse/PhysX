# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import importlib.util
import io
import os
import tempfile
import unittest
from contextlib import redirect_stderr
from pathlib import Path
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "build_wheel.py"
SPEC = importlib.util.spec_from_file_location("build_wheel", SCRIPT)
builder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builder)


class WheelVersionTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)

    def read_version(self, base="25.11.2", **environment):
        version_file = self.root / "VERSION"
        version_file.write_text(base + "\n", encoding="utf-8")
        env = os.environ.copy()
        for name in ("PHYSX_SCHEMA_BUILD_NUMBER", "CI_PIPELINE_ID"):
            env.pop(name, None)
        env.update(environment)
        with patch.object(builder, "SCHEMA_ROOT", self.root), patch.dict(os.environ, env, clear=True):
            result = builder.read_version()
        self.assertEqual(version_file.read_text(encoding="utf-8"), base + "\n")
        return result

    def test_local_build_keeps_base_version(self):
        self.assertEqual(self.read_version(), "25.11.2")

    def test_pipeline_id_is_appended(self):
        self.assertEqual(self.read_version(CI_PIPELINE_ID="1234567"), "25.11.2.1234567")

    def test_parent_pipeline_takes_precedence_over_child(self):
        self.assertEqual(
            self.read_version(PHYSX_SCHEMA_BUILD_NUMBER="1234567", CI_PIPELINE_ID="7654321"),
            "25.11.2.1234567",
        )

    def test_explicit_local_build_number(self):
        self.assertEqual(self.read_version(PHYSX_SCHEMA_BUILD_NUMBER="42"), "25.11.2.42")

    def test_empty_override_uses_pipeline_id(self):
        self.assertEqual(
            self.read_version(PHYSX_SCHEMA_BUILD_NUMBER="", CI_PIPELINE_ID="42"), "25.11.2.42"
        )

    def test_whitespace_matches_ovphysx(self):
        for value, expected in [(" 42\t", "25.11.2.42"), (" \t", "25.11.2")]:
            with self.subTest(value=value):
                self.assertEqual(
                    self.read_version(PHYSX_SCHEMA_BUILD_NUMBER=value, CI_PIPELINE_ID="99"), expected
                )

    def test_invalid_build_number_warns_and_keeps_base(self):
        for value in ("abc", "12a", "1.2", "-1", "١٢"):
            with self.subTest(value=value), redirect_stderr(io.StringIO()) as stderr:
                self.assertEqual(
                    self.read_version(PHYSX_SCHEMA_BUILD_NUMBER=value, CI_PIPELINE_ID="99"), "25.11.2"
                )
                self.assertTrue(stderr.getvalue())

    def test_non_release_versions_are_unchanged(self):
        for base in ("25.11", "25.11.2-rc1", "25.11.2.rc1", "25.11.2.42", "25.11.2+local"):
            with self.subTest(base=base):
                self.assertEqual(self.read_version(base, CI_PIPELINE_ID="99"), base)


if __name__ == "__main__":
    unittest.main()
