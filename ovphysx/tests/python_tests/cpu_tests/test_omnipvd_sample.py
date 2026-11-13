# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-OMNIPVD-SAMPLE-001
# @covers AC-1
# @maps_to TEST-PYTHON-OMNIPVD-SAMPLE-001

"""Dependency-free checks for the documented OmniPVD recording sample."""

import contextlib
import io
import runpy
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock


SAMPLE = Path(__file__).resolve().parents[2] / "python_samples" / "omnipvd_recording.py"


class OmniPvdSampleTests(unittest.TestCase):
    def test_recording_summary_handles_literal_brackets(self):
        for directory_name in ("recording_plain", "recording_[1]"):
            with self.subTest(directory_name=directory_name):
                with tempfile.TemporaryDirectory() as temp_root:
                    output_dir = Path(temp_root) / directory_name
                    output_dir.mkdir()

                    class FakeStage:
                        def __init__(self, name):
                            self.name = name

                        def advance_write_floor(self, *, ordinal):
                            return self

                        def wait(self):
                            pass

                        def destroy(self):
                            pass

                    class FakePhysX:
                        def attach_ovstage(self, stage, *, read_ordinal):
                            return None

                        def wait_task(self, task):
                            pass

                        def wait_all(self):
                            pass

                        def start_recording(self, destination):
                            Path(destination).write_bytes(b"x" * 2048)

                        def step_sync(self, dt):
                            pass

                        def stop_recording(self):
                            pass

                        def detach_ovstage(self):
                            return None

                        def destroy(self):
                            pass

                    ovphysx = types.ModuleType("ovphysx")
                    ovphysx.__file__ = str(Path(temp_root) / "ovphysx_stub.py")
                    ovphysx.__path__ = []
                    ovphysx.GlobalConfigKey = types.SimpleNamespace(OVD_RECORDING_CAPABLE=object())
                    ovphysx.OmniPvdDestination = types.SimpleNamespace(file=lambda path: path)
                    ovphysx.PhysX = FakePhysX
                    ovphysx.set_global_config = lambda key, value: None
                    ovphysx.newton_schema_root = lambda: Path(temp_root)

                    # The sample imports the installed recipe, so the stub must be a package.
                    samples = types.ModuleType("ovphysx.samples")
                    samples.__path__ = []
                    python_samples = types.ModuleType("ovphysx.samples.python_samples")
                    python_samples.__path__ = []
                    schema_support = types.ModuleType("ovphysx.samples.python_samples.schema_support")
                    schema_support.physx_schema_paths = lambda: []
                    ovphysx.samples = samples
                    samples.python_samples = python_samples
                    python_samples.schema_support = schema_support

                    population = types.ModuleType("ovphysx.population")
                    population.physics_population_descs = lambda rendering=False: []
                    ovphysx.population = population

                    ovstage = types.ModuleType("ovstage")
                    ovstage.Stage = FakeStage
                    ovstage.population = types.SimpleNamespace(
                        available=lambda: True,
                        register_usd_schemas=lambda paths: None,
                        open_usd_with_desc=lambda stage, path, *, ordinal, time, descs: None,
                        DEFAULT_TIME=float("nan"),
                    )

                    output = io.StringIO()
                    with (
                        mock.patch.dict(
                            sys.modules,
                            {
                                "ovphysx": ovphysx,
                                "ovphysx.population": population,
                                "ovphysx.samples": samples,
                                "ovphysx.samples.python_samples": python_samples,
                                "ovphysx.samples.python_samples.schema_support": schema_support,
                                "ovstage": ovstage,
                            },
                        ),
                        mock.patch.object(tempfile, "mkdtemp", return_value=str(output_dir)),
                        contextlib.redirect_stdout(output),
                    ):
                        runpy.run_path(str(SAMPLE), run_name="__main__")

                    self.assertTrue((output_dir / "capture.ovd").is_file())
                    self.assertEqual((output_dir / "capture.ovd").stat().st_size, 2048)
                    self.assertIn("Recorded 1 OVD file(s)", output.getvalue())
                    self.assertIn("capture.ovd  (2.0 KB)", output.getvalue())
                    self.assertNotIn("WARNING: No .ovd files found", output.getvalue())


if __name__ == "__main__":
    unittest.main()
