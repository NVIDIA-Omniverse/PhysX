# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-3
# @implements REQ-PYTHON-OMNIPVD-LATE-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8

"""End-to-end tests for OmniPVD OVD recording via ovphysx config.

Verifies that configuring the OmniPVD recording directory and enabling
OmniPVD output produces timestamped .ovd files after simulation, and
that misconfigured settings produce expected failure behavior.
"""

import ctypes
import gc
import glob
import os
import socket
import subprocess
import sys
import tempfile
import textwrap
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

# Path to a USD scene with physics objects (used by other tests/samples)
_TEST_DATA = Path(__file__).resolve().parent.parent / "data"
_USD_SCENE = str(_TEST_DATA / "simple_physics_scene.usda")


def _run_ovphysx_subprocess(script_body: str) -> None:
    """Run startup-only configuration in a fresh process."""
    script = f"import sys\nsys.path.insert(0, {repr(str(Path(__file__).resolve().parent))})\n" + textwrap.dedent(
        script_body
    )
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=Path(__file__).resolve().parent,
        text=True,
        capture_output=True,
        timeout=120,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def _run_omnipvd_recording_subprocess(output_dir: str, steps: int, *, enabled: bool = True) -> None:
    """Run recording checks before process-global settings are initialized."""
    _run_ovphysx_subprocess(f"""
        from ovphysx import PhysX, PhysXConfig
        from test_utils import load_usd_with_ovstage

        physx = PhysX(
            config=PhysXConfig(
                omnipvd_ovd_recording_directory={repr(output_dir)},
                omnipvd_output_enabled={enabled!r},
            )
        )
        try:
            load_usd_with_ovstage(physx, {repr(_USD_SCENE)})
            physx.wait_all()
            for _ in range({steps}):
                physx.step_sync(1.0 / 60.0)
        finally:
            physx.destroy()
        """)


def test_late_destination_factories_validate_exact_shapes():
    """Python destinations expose only exact FILE and TCP tuples."""
    from ovphysx import OmniPvdDestination

    assert OmniPvdDestination.file("capture.ovd").file_path == "capture.ovd"
    tcp = OmniPvdDestination.tcp("127.0.0.1", 5425, timeout_ms=3000)
    assert (tcp.tcp_address, tcp.tcp_port, tcp.tcp_timeout_ms) == ("127.0.0.1", 5425, 3000)
    with pytest.raises(ValueError):
        OmniPvdDestination.file("")
    with pytest.raises(ValueError):
        OmniPvdDestination.tcp("127.0.0.1", 0)


def test_start_recording_keeps_destination_strings_alive_through_native_call():
    """Nested ctypes fields borrow their source string buffers during the call."""
    from ovphysx import OmniPvdDestination, PhysX
    from ovphysx import api
    from ovphysx.types import ApiStatus

    class FakeLib:
        def __init__(self):
            self.destinations = []

        def ovphysx_start_recording(self, handle, destination):
            assert handle == 17
            gc.collect()
            native_destination = ctypes.cast(
                destination, ctypes.POINTER(api.ovphysx_omnipvd_destination_t)
            ).contents
            self.destinations.append(
                (
                    native_destination.transport,
                    ctypes.string_at(
                        native_destination.file_path.ptr,
                        native_destination.file_path.length,
                    ).decode("utf-8"),
                    ctypes.string_at(
                        native_destination.tcp_address.ptr,
                        native_destination.tcp_address.length,
                    ).decode("utf-8"),
                )
            )
            return SimpleNamespace(status=ApiStatus.SUCCESS)

    fake_lib = FakeLib()
    physx = PhysX.__new__(PhysX)
    physx._lib = fake_lib
    physx._omni_physx_sdk_handle = ctypes.c_uint64(17)
    physx._attached_ovstage = None
    physx._lifecycle_acquired = False
    physx._released = True
    physx.start_recording(OmniPvdDestination.file("capture.ovd"))
    physx.start_recording(OmniPvdDestination.tcp("127.0.0.1", 5425))

    assert fake_lib.destinations == [
        (0, "capture.ovd", ""),
        (1, "", "127.0.0.1"),
    ]


def test_late_start_requires_creation_capability():
    _run_ovphysx_subprocess(f"""
        from ovphysx import OmniPvdDestination, PhysX
        from test_utils import load_usd_with_ovstage
        physx = PhysX()
        try:
            load_usd_with_ovstage(physx, {repr(_USD_SCENE)})
            physx.wait_all()
            physx.step_sync(1.0 / 60.0)
            try:
                physx.start_recording(OmniPvdDestination.file("unavailable.ovd"))
                raise AssertionError("default instance unexpectedly became recording-capable")
            except RuntimeError as exc:
                assert "INVALID_STATE" in str(exc)
                assert "omnipvd_recording_capable" in str(exc)
        finally:
            physx.destroy()
    """)


def test_late_file_recording_retries_rejects_takeover_and_restarts():
    """Python supports retry and sequential sessions without active takeover."""
    with tempfile.TemporaryDirectory(prefix="ovphysx_pvd_late_") as output_dir:
        first_path = str(Path(output_dir) / "first.ovd")
        second_path = str(Path(output_dir) / "second.ovd")
        _run_ovphysx_subprocess(f"""
            from ovphysx import OmniPvdDestination, PhysX, PhysXConfig
            from test_utils import load_usd_with_ovstage

            physx = PhysX(config=PhysXConfig(omnipvd_recording_capable=True))
            try:
                load_usd_with_ovstage(physx, {repr(_USD_SCENE)})
                physx.wait_all()
                physx.step_sync(1.0 / 60.0)
                assert not physx.is_recording()

                try:
                    physx.start_recording(OmniPvdDestination.file({output_dir!r}))
                    raise AssertionError("opening a directory as a file unexpectedly succeeded")
                except RuntimeError:
                    pass
                assert not physx.is_recording()

                first = OmniPvdDestination.file({first_path!r})
                second = OmniPvdDestination.file({second_path!r})
                physx.start_recording(first)
                assert physx.is_recording()
                physx.step_sync(1.0 / 60.0)
                try:
                    physx.start_recording(second)
                    raise AssertionError("an active recording was replaced")
                except RuntimeError as exc:
                    assert "INVALID_STATE" in str(exc)
                assert physx.is_recording()
                physx.stop_recording()
                assert not physx.is_recording()
                physx.step_sync(1.0 / 60.0)
                physx.start_recording(second)
                physx.step_sync(1.0 / 60.0)
                physx.stop_recording()
            finally:
                physx.destroy()
            """)
        assert Path(first_path).stat().st_size > 12
        assert Path(second_path).stat().st_size > 12


def test_cold_startup_owner_can_stop_and_restart_late():
    """Startup output restarts on reattach before another late FILE session."""
    with tempfile.TemporaryDirectory(prefix="ovphysx_pvd_startup_owner_") as output_dir:
        late_path = str(Path(output_dir) / "late.ovd")
        reattach_late_path = str(Path(output_dir) / "reattach_late.ovd")
        _run_ovphysx_subprocess(f"""
            from ovphysx import OmniPvdDestination, PhysX, PhysXConfig
            from test_utils import attach_usd_with_ovstage

            physx = PhysX(config=PhysXConfig(
                omnipvd_output_enabled=True,
                omnipvd_ovd_recording_directory={output_dir!r},
            ))
            peer = None
            try:
                peer = PhysX()
                stage = attach_usd_with_ovstage(peer, {repr(_USD_SCENE)})
                peer.wait_all()
                peer.step_sync(1.0 / 60.0)
                assert physx.is_recording()
                assert not peer.is_recording()
                try:
                    peer.stop_recording()
                    raise AssertionError("peer stopped the creator's startup session")
                except RuntimeError as exc:
                    assert "INVALID_STATE" in str(exc)

                physx.stop_recording()
                assert not physx.is_recording()
                physx.start_recording(OmniPvdDestination.file({late_path!r}))
                peer.step_sync(1.0 / 60.0)
                physx.stop_recording()

                peer.detach_ovstage()
                peer.attach_ovstage(stage, read_ordinal=1)
                assert peer.is_recording()
                try:
                    physx.start_recording(OmniPvdDestination.file({reattach_late_path!r}))
                    raise AssertionError("late start replaced the restarted startup session")
                except RuntimeError as exc:
                    assert "INVALID_STATE" in str(exc)
                peer.stop_recording()
                physx.start_recording(OmniPvdDestination.file({reattach_late_path!r}))
                peer.step_sync(1.0 / 60.0)
                physx.stop_recording()
            finally:
                if peer is not None:
                    peer.destroy()
                physx.destroy()
        """)
        assert Path(late_path).stat().st_size > 12
        assert Path(reattach_late_path).stat().st_size > 12
        assert glob.glob(os.path.join(output_dir, "*_rec.ovd"))


@pytest.mark.parametrize("late_start", [False, True], ids=["startup", "late"])
def test_tcp_streams_to_ready_listener(late_start, tmp_path):
    """TCP startup and late start emit payload through their public Python paths."""
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.bind(("127.0.0.1", 0))
    port = listener.getsockname()[1]
    ready = threading.Event()
    received = bytearray()
    reader_error = []

    def read_stream():
        peer = None
        try:
            listener.listen(1)
            listener.settimeout(10)
            ready.set()
            peer, _ = listener.accept()
            peer.settimeout(10)
            while chunk := peer.recv(64 * 1024):
                received.extend(chunk)
        except Exception as exc:
            reader_error.append(exc)
        finally:
            if peer is not None:
                peer.close()
            listener.close()
            ready.set()

    reader = threading.Thread(target=read_stream, daemon=True)
    reader.start()
    assert ready.wait(timeout=5), "TCP listener did not become ready"

    if late_start:
        late_file_path = tmp_path / "after_late_tcp.ovd"
        physx_constructor = "PhysX(config=PhysXConfig(omnipvd_recording_capable=True))"
        before_recording = "physx.step_sync(1.0 / 60.0)"
        start_recording = f'physx.start_recording(OmniPvdDestination.tcp("127.0.0.1", {port}, timeout_ms=3000))'
        stop_recording = "physx.stop_recording()"
        after_recording = "\n            ".join((
            "physx.step_sync(1.0 / 60.0)",
            f"physx.start_recording(OmniPvdDestination.file({str(late_file_path)!r}))",
            "physx.step_sync(1.0 / 60.0)",
            "physx.stop_recording()",
        ))
    else:
        physx_constructor = (
            "PhysX(config=PhysXConfig("
            "omnipvd_output_enabled=True, "
            'omnipvd_transport="tcp", '
            'omnipvd_tcp_address="127.0.0.1", '
            f"omnipvd_tcp_port={port}, omnipvd_tcp_timeout_ms=3000))"
        )
        before_recording = start_recording = stop_recording = after_recording = ""

    script = textwrap.dedent(f"""
        import sys

        sys.path.insert(0, {repr(str(Path(__file__).resolve().parent))})

        from ovphysx import OmniPvdDestination, PhysX, PhysXConfig
        from test_utils import load_usd_with_ovstage

        physx = {physx_constructor}
        try:
            load_usd_with_ovstage(physx, {repr(_USD_SCENE)})
            physx.wait_all()
            {before_recording}
            {start_recording}
            physx.step_sync(1.0 / 60.0)
            {stop_recording}
            {after_recording}
        finally:
            physx.destroy()
        """)
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=Path(__file__).resolve().parent,
        text=True,
        capture_output=True,
        timeout=120,
    )
    reader.join(timeout=15)
    if reader.is_alive():
        listener.close()
        reader.join(timeout=2)

    assert result.returncode == 0, result.stdout + result.stderr
    assert not reader.is_alive(), "TCP reader did not finish within its bounded timeout"
    assert not reader_error, f"TCP reader failed: {reader_error[0]!r}"
    assert len(received) > 20, "Expected OmniPVD commands after the socket handshake and version header"
    if late_start:
        assert late_file_path.stat().st_size > 12


class TestOmniPvdRecording:
    """End-to-end OmniPVD .ovd recording tests."""

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
            _run_omnipvd_recording_subprocess(output_dir, steps=1, enabled=False)

            ovd_files = glob.glob(os.path.join(output_dir, "*.ovd"))
            assert len(ovd_files) == 0, f"Expected no .ovd files when recording is disabled, found: {ovd_files}"

    def test_ovd_not_produced_without_directory(self):
        """Enable without a valid directory produces no .ovd file and does not crash."""
        _run_omnipvd_recording_subprocess("", steps=1)

    def test_ovd_directory_created_if_missing(self):
        """Recording directory that does not exist yet is auto-created."""
        with tempfile.TemporaryDirectory(prefix="ovphysx_pvd_") as base_dir:
            nested_dir = os.path.join(base_dir, "sub", "recordings")
            assert not os.path.exists(nested_dir)

            _run_omnipvd_recording_subprocess(nested_dir, steps=3)

            assert os.path.isdir(nested_dir), f"Expected recording directory to be auto-created: {nested_dir}"
            ovd_files = glob.glob(os.path.join(nested_dir, "*_rec.ovd"))
            assert len(ovd_files) >= 1, (
                f"Expected at least one *_rec.ovd in auto-created dir {nested_dir}, " f"found: {os.listdir(nested_dir)}"
            )


class TestOmniPvdConfigRoundTrip:
    """Verify the typed config entries round-trip correctly."""

    def test_omnipvd_config_bool_round_trip(self):
        """Setting omnipvd_output_enabled via PhysXConfig is readable via get_config_bool."""
        _run_ovphysx_subprocess("""
            from ovphysx import ConfigBool, PhysX, PhysXConfig

            physx = PhysX(config=PhysXConfig(omnipvd_output_enabled=True))
            try:
                value = physx.get_config_bool(ConfigBool.OMNIPVD_OUTPUT_ENABLED)
                assert value is True, f"Expected True, got {value}"
            finally:
                physx.destroy()
            """)

    def test_omnipvd_recording_capable_round_trip(self):
        _run_ovphysx_subprocess("""
            from ovphysx import ConfigBool, PhysX, PhysXConfig
            physx = PhysX(config=PhysXConfig(omnipvd_recording_capable=True))
            try:
                assert physx.get_config_bool(ConfigBool.OMNIPVD_RECORDING_CAPABLE) is True
            finally:
                physx.destroy()
        """)

    def test_omnipvd_config_string_round_trip(self):
        """Setting omnipvd_ovd_recording_directory via PhysXConfig is readable via get_config_string."""
        test_path = tempfile.gettempdir() + "/pvd_roundtrip_test"
        _run_ovphysx_subprocess(f"""
            from ovphysx import ConfigString, PhysX, PhysXConfig

            physx = PhysX(config=PhysXConfig(omnipvd_ovd_recording_directory={test_path!r}))
            try:
                value = physx.get_config_string(ConfigString.OMNIPVD_OVD_RECORDING_DIRECTORY)
                assert value == {test_path!r}, f"Expected {test_path!r}, got {{value}}"
            finally:
                physx.destroy()
            """)
