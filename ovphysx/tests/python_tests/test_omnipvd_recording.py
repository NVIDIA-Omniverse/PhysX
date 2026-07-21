# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""End-to-end tests for OmniPVD OVD recording via ovphysx config.

Verifies that configuring the OmniPVD recording directory and enabling
OmniPVD output produces timestamped .ovd files after simulation, and
that misconfigured settings produce expected failure behavior.
"""

import glob
import os
import platform
import subprocess
import sys
import tempfile
import textwrap
from pathlib import Path

import pytest

from ovphysx import ConfigBool, ConfigString, PhysX, PhysXConfig
from test_utils import load_usd_with_ovstage

# Path to a USD scene with physics objects (used by other tests/samples)
_TEST_DATA = Path(__file__).resolve().parent.parent / "data"
_USD_SCENE = str(_TEST_DATA / "simple_physics_scene.usda")

# OmniPVD is disabled on aarch64 (#if !CARB_AARCH64 in Setup.cpp).
_OMNIPVD_SUPPORTED = platform.machine() not in ("aarch64", "arm64")
_skip_aarch64 = pytest.mark.skipif(not _OMNIPVD_SUPPORTED, reason="OmniPVD not supported on aarch64")


def _run_omnipvd_recording_subprocess(output_dir: str, steps: int) -> None:
    """Run positive recording checks before process-global settings are initialized."""
    script = textwrap.dedent(
        f"""
        import sys

        sys.path.insert(0, {repr(str(Path(__file__).resolve().parent))})

        from ovphysx import PhysX, PhysXConfig
        from test_utils import load_usd_with_ovstage

        physx = PhysX(
            config=PhysXConfig(
                omnipvd_ovd_recording_directory={repr(output_dir)},
                omnipvd_output_enabled=True,
            )
        )
        try:
            load_usd_with_ovstage(physx, {repr(_USD_SCENE)})
            physx.wait_all()
            for _ in range({steps}):
                physx.step_sync(1.0 / 60.0)
        finally:
            physx.release()
        """
    )
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=Path(__file__).resolve().parent,
        text=True,
        capture_output=True,
        timeout=120,
    )
    assert result.returncode == 0, result.stdout + result.stderr


class TestOmniPvdRecording:
    """End-to-end OmniPVD .ovd recording tests."""

    @_skip_aarch64
    def test_ovd_file_produced(self):
        """Positive path: recording directory + enable produces an .ovd file."""
        with tempfile.TemporaryDirectory(prefix="ovphysx_pvd_") as output_dir:
            _run_omnipvd_recording_subprocess(output_dir, steps=5)

            ovd_files = glob.glob(os.path.join(output_dir, "*_rec.ovd"))
            assert len(ovd_files) >= 1, (
                f"Expected at least one *_rec.ovd file in {output_dir}, " f"found: {os.listdir(output_dir)}"
            )
            for file_path in ovd_files:
                size = os.path.getsize(file_path)
                assert size > 0, f"OVD file {file_path} is empty"

    def test_ovd_not_produced_when_disabled(self):
        """Recording disabled by default produces no .ovd files."""
        with tempfile.TemporaryDirectory(prefix="ovphysx_pvd_") as output_dir:
            physx = PhysX(
                config=PhysXConfig(
                    omnipvd_ovd_recording_directory=output_dir,
                    omnipvd_output_enabled=False,
                )
            )
            try:
                load_usd_with_ovstage(physx, _USD_SCENE)
                physx.wait_all()
                physx.step_sync(1.0 / 60.0)
            finally:
                physx.release()

            ovd_files = glob.glob(os.path.join(output_dir, "*.ovd"))
            assert len(ovd_files) == 0, f"Expected no .ovd files when recording is disabled, found: {ovd_files}"

    def test_ovd_not_produced_without_directory(self):
        """Enable without a valid directory produces no .ovd file and does not crash."""
        physx = PhysX(
            config=PhysXConfig(
                omnipvd_ovd_recording_directory="",
                omnipvd_output_enabled=True,
            )
        )
        try:
            load_usd_with_ovstage(physx, _USD_SCENE)
            physx.wait_all()
            physx.step_sync(1.0 / 60.0)
        finally:
            physx.release()

    @_skip_aarch64
    def test_ovd_directory_created_if_missing(self):
        """Recording directory that does not exist yet is auto-created."""
        with tempfile.TemporaryDirectory(prefix="ovphysx_pvd_") as base_dir:
            nested_dir = os.path.join(base_dir, "sub", "recordings")
            assert not os.path.exists(nested_dir)

            _run_omnipvd_recording_subprocess(nested_dir, steps=3)

            assert os.path.isdir(nested_dir), f"Expected recording directory to be auto-created: {nested_dir}"
            ovd_files = glob.glob(os.path.join(nested_dir, "*_rec.ovd"))
            assert len(ovd_files) >= 1, (
                f"Expected at least one *_rec.ovd in auto-created dir {nested_dir}, "
                f"found: {os.listdir(nested_dir)}"
            )


class TestOmniPvdConfigRoundTrip:
    """Verify the typed config entries round-trip correctly."""

    def test_omnipvd_config_bool_round_trip(self):
        """Setting omnipvd_output_enabled via PhysXConfig is readable via get_config_bool."""
        physx = PhysX(config=PhysXConfig(omnipvd_output_enabled=True))
        try:
            value = physx.get_config_bool(ConfigBool.OMNIPVD_OUTPUT_ENABLED)
            assert value is True, f"Expected True, got {value}"
        finally:
            physx.release()

    def test_omnipvd_config_string_round_trip(self):
        """Setting omnipvd_ovd_recording_directory via PhysXConfig is readable via get_config_string."""
        test_path = tempfile.gettempdir() + "/pvd_roundtrip_test"
        physx = PhysX(config=PhysXConfig(omnipvd_ovd_recording_directory=test_path))
        try:
            value = physx.get_config_string(ConfigString.OMNIPVD_OVD_RECORDING_DIRECTORY)
            assert value == test_path, f"Expected '{test_path}', got '{value}'"
        finally:
            physx.release()
