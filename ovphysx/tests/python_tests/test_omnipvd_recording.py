# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-OMNIPVD-001
# @covers AC-3
# @implements REQ-PYTHON-OMNIPVD-LATE-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8

"""End-to-end OVD recording through modern capability and explicit sessions."""

import ctypes
import gc
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


def _run_ovd_recording_subprocess(output_path: str, steps: int, *, start: bool = True) -> None:
    """Create and attach dormant, then optionally start before the first step."""
    _run_ovphysx_subprocess(f"""
        from pathlib import Path
        from ovphysx import GlobalConfigKey, OmniPvdDestination, PhysX, set_global_config
        from test_utils import load_usd_with_ovstage

        set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)
        physx = PhysX()
        try:
            assert not physx.is_recording()
            load_usd_with_ovstage(physx, {repr(_USD_SCENE)})
            physx.wait_all()
            assert not physx.is_recording()
            assert not Path({output_path!r}).exists()
            if {start!r}:
                physx.start_recording(OmniPvdDestination.file({output_path!r}))
                assert physx.is_recording()
            for _ in range({steps}):
                physx.step_sync(1.0 / 60.0)
            if {start!r}:
                physx.stop_recording()
            assert not physx.is_recording()
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

        def ovphysx_ovd_start_recording(self, handle, destination):
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
                assert "OVD_RECORDING_CAPABLE" in str(exc)
        finally:
            physx.destroy()
    """)


def test_late_file_recording_retries_rejects_takeover_and_restarts():
    """Python supports retry and sequential sessions without active takeover."""
    with tempfile.TemporaryDirectory(prefix="ovphysx_pvd_late_") as output_dir:
        first_path = str(Path(output_dir) / "first.ovd")
        second_path = str(Path(output_dir) / "second.ovd")
        _run_ovphysx_subprocess(f"""
            from ovphysx import GlobalConfigKey, OmniPvdDestination, PhysX, set_global_config
            from test_utils import load_usd_with_ovstage

            set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)
            physx = PhysX()
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


def test_explicit_recording_owner_and_reattach_stay_dormant():
    """Only explicit starts record; peer attachment never changes ownership."""
    with tempfile.TemporaryDirectory(prefix="ovphysx_ovd_owner_") as output_dir:
        first_path = str(Path(output_dir) / "owner.ovd")
        second_path = str(Path(output_dir) / "peer.ovd")
        _run_ovphysx_subprocess(f"""
            from ovphysx import GlobalConfigKey, OmniPvdDestination, PhysX, set_global_config
            from test_utils import attach_usd_with_ovstage

            set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)
            physx = PhysX()
            peer = None
            try:
                assert not physx.is_recording()
                owner_stage = attach_usd_with_ovstage(physx, {repr(_USD_SCENE)})
                physx.wait_all()
                peer = PhysX()
                stage = attach_usd_with_ovstage(peer, {repr(_USD_SCENE)})
                peer.wait_all()
                assert not physx.is_recording()
                assert not peer.is_recording()
                physx.start_recording(OmniPvdDestination.file({first_path!r}))
                assert physx.is_recording()
                assert not peer.is_recording()
                try:
                    peer.stop_recording()
                    raise AssertionError("peer stopped another owner's session")
                except RuntimeError as exc:
                    assert "INVALID_STATE" in str(exc)
                try:
                    peer.start_recording(OmniPvdDestination.file({second_path!r}))
                    raise AssertionError("peer replaced another owner's session")
                except RuntimeError as exc:
                    assert "INVALID_STATE" in str(exc)
                peer.wait_task(peer.detach_ovstage())
                peer.wait_task(peer.attach_ovstage(stage, read_ordinal=1))
                assert physx.is_recording()
                assert not peer.is_recording()
                physx.step_sync(1.0 / 60.0)
                physx.wait_task(physx.detach_ovstage())
                assert not physx.is_recording()
                physx.wait_task(physx.attach_ovstage(owner_stage, read_ordinal=1))
                assert not physx.is_recording()
                assert not peer.is_recording()
                peer.step_sync(1.0 / 60.0)
                assert not peer.is_recording()
                peer.start_recording(OmniPvdDestination.file({second_path!r}))
                peer.step_sync(1.0 / 60.0)
                peer.stop_recording()
            finally:
                if peer is not None:
                    peer.destroy()
                physx.destroy()
        """)
        assert Path(first_path).stat().st_size > 12
        assert Path(second_path).stat().st_size > 12
        assert {path.name for path in Path(output_dir).iterdir()} == {"owner.ovd", "peer.ovd"}


@pytest.mark.parametrize("step_before_start", [False, True], ids=["before-first-step", "after-first-step"])
def test_tcp_streams_to_ready_listener(step_before_start, tmp_path):
    """Explicit TCP start emits payload before or after the first simulation step."""
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.bind(("127.0.0.1", 0))
    port = listener.getsockname()[1]
    ready = threading.Event()
    stop_reader = threading.Event()
    clean_eof = threading.Event()
    received = bytearray()
    reader_error = []

    def read_stream():
        peer = None
        try:
            listener.listen(1)
            # The child may spend its full subprocess budget importing and attaching.
            # Poll cancellation instead of expiring before it reaches TCP start.
            listener.settimeout(0.25)
            ready.set()
            while not stop_reader.is_set():
                try:
                    peer, _ = listener.accept()
                    break
                except socket.timeout:
                    continue
            if peer is None:
                return
            peer.settimeout(0.25)
            while not stop_reader.is_set():
                try:
                    chunk = peer.recv(64 * 1024)
                except socket.timeout:
                    continue
                if not chunk:
                    clean_eof.set()
                    break
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

    file_path = tmp_path / "after_tcp.ovd"
    script = textwrap.dedent(f"""
        import sys

        sys.path.insert(0, {repr(str(Path(__file__).resolve().parent))})

        from ovphysx import GlobalConfigKey, OmniPvdDestination, PhysX, set_global_config
        from test_utils import load_usd_with_ovstage

        set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)
        physx = PhysX()
        try:
            load_usd_with_ovstage(physx, {repr(_USD_SCENE)})
            physx.wait_all()
            assert not physx.is_recording()
            if {step_before_start!r}:
                physx.step_sync(1.0 / 60.0)
            assert not physx.is_recording()
            physx.start_recording(OmniPvdDestination.tcp("127.0.0.1", {port}, timeout_ms=3000))
            assert physx.is_recording()
            physx.step_sync(1.0 / 60.0)
            physx.stop_recording()
            assert not physx.is_recording()
            physx.step_sync(1.0 / 60.0)
            physx.start_recording(OmniPvdDestination.file({str(file_path)!r}))
            physx.step_sync(1.0 / 60.0)
            physx.stop_recording()
        finally:
            physx.destroy()
        """)
    try:
        assert ready.wait(timeout=5), "TCP listener did not become ready"
        result = subprocess.run(
            [sys.executable, "-c", script],
            cwd=Path(__file__).resolve().parent,
            text=True,
            capture_output=True,
            timeout=120,
        )
        if result.returncode == 0:
            reader.join(timeout=15)  # Drain through EOF before cancelling.
    finally:
        stop_reader.set()
        reader.join(timeout=2)

    assert result.returncode == 0, result.stdout + result.stderr
    assert not reader.is_alive(), "TCP reader did not finish within its bounded timeout"
    assert not reader_error, f"TCP reader failed: {reader_error[0]!r}"
    assert clean_eof.is_set(), "TCP recording did not reach clean EOF before listener cancellation"
    assert len(received) > 20, "Expected OVD commands after the socket handshake and version header"
    assert file_path.stat().st_size > 12


class TestOvdRecording:
    """Exact-path output requires an explicit recording start."""

    def test_ovd_file_produced_before_first_step(self, tmp_path):
        output = tmp_path / "capture.ovd"
        _run_ovd_recording_subprocess(str(output), steps=5)
        assert output.stat().st_size > 12
        assert list(tmp_path.iterdir()) == [output]

    def test_ovd_not_produced_until_explicit_start(self, tmp_path):
        output = tmp_path / "capture.ovd"
        _run_ovd_recording_subprocess(str(output), steps=1, start=False)
        assert not list(tmp_path.iterdir())

    def test_ovd_missing_parent_fails_without_starting(self, tmp_path):
        output = tmp_path / "missing" / "capture.ovd"
        _run_ovphysx_subprocess(f"""
            from ovphysx import GlobalConfigKey, OmniPvdDestination, PhysX, set_global_config
            from test_utils import load_usd_with_ovstage
            set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)
            physx = PhysX()
            try:
                load_usd_with_ovstage(physx, {_USD_SCENE!r})
                physx.wait_all()
                try:
                    physx.start_recording(OmniPvdDestination.file({str(output)!r}))
                except RuntimeError:
                    pass
                else:
                    raise AssertionError("missing parent directory unexpectedly accepted")
                assert not physx.is_recording()
                physx.step_sync(1.0 / 60.0)
            finally:
                physx.destroy()
        """)
        assert not output.exists()


def test_retired_ovd_carbonite_overrides_rejected_without_freezing_capability():
    """The raw escape hatch cannot silently reintroduce startup OVD config."""
    _run_ovphysx_subprocess("""
        from ovphysx import GlobalConfigKey, PhysX, PhysXConfig, get_global_config, set_global_config
        retired = {
            '/physics/omniPvdOutputEnabled': True,
            '/physics/omniPvdRecordingCapable': True,
            '/persistent/physics/omniPvdOvdRecordingDirectory': 'captures',
            '/physics/omniPvdTransport': 'file',
            '/physics/omniPvdTcpAddress': '127.0.0.1',
            '/physics/omniPvdTcpPort': 5425,
            '/physics/omniPvdTcpTimeoutMs': 100,
        }
        for path, value in retired.items():
            try:
                unexpected = PhysX(config=PhysXConfig(carbonite_overrides={path: value}))
            except RuntimeError as error:
                assert 'OVD' in str(error), str(error)
            else:
                unexpected.destroy()
                raise AssertionError(f'retired OVD path accepted: {path}')
            assert get_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE) is False
            set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)
            set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, False)
        set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)
        physx = PhysX()
        try:
            assert not physx.is_recording()
        finally:
            physx.destroy()
    """)


def test_global_ovd_capability_rejects_instance_scope_without_mutation():
    """An instance setter cannot write the global capability or its local configuration."""
    _run_ovphysx_subprocess("""
        import pytest
        from ovphysx import ConfigKey, GlobalConfigKey, PhysX, get_global_config, set_global_config

        key = GlobalConfigKey.OVD_RECORDING_CAPABLE
        set_global_config(key, True)
        PhysX.set_cpu_mode(True)
        physx = PhysX()
        try:
            local_key = ConfigKey.ACTIVE_CUDA_GPUS
            before = physx.get_config(local_key)
            for value in (False, True):
                with pytest.raises(ValueError):
                    physx.set_config(key, value)
                assert get_global_config(key) is True
                assert physx.get_config(local_key) == before
                assert not physx.is_recording()
        finally:
            physx.destroy()
        set_global_config(key, False)
        assert get_global_config(key) is False
    """)
