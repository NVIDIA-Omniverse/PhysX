# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for Python-side logging configuration.

Verifies that the ovphysx Python library uses the standard logging module
correctly, that the ovphysx source log level API works, and that enable_python_logging
routes native messages into Python's logging system.
"""

# @implements REQ-CAPI-LOG-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
# @implements REQ-PYTHON-LIFECYCLE-001
# @covers AC-4 AC-7

import logging
from pathlib import Path
import subprocess
import sys
import textwrap
import threading
from types import SimpleNamespace

import pytest


class _InterruptAfterLoggingTransitionClaim:
    """Inject one interruption as a logging-transition lock is released."""

    def __init__(self, condition, api, message):
        self._condition = condition
        self._api = api
        self._message = message
        self._raised = False

    def __enter__(self):
        return self._condition.__enter__()

    def __exit__(self, exc_type, exc_value, traceback):
        suppressed = self._condition.__exit__(exc_type, exc_value, traceback)
        if not self._raised and self._api._python_log_callback_transition:
            self._raised = True
            raise KeyboardInterrupt(self._message)
        return suppressed

    def __getattr__(self, name):
        return getattr(self._condition, name)


# ============================================================================
# Logger setup: ovphysx should follow library best practices (NullHandler)
# ============================================================================


def test_ovphysx_logger_exists():
    """The ovphysx package should register a logger named 'ovphysx'."""
    logger = logging.getLogger("ovphysx")
    assert logger is not None
    assert logger.name == "ovphysx"


def test_ovphysx_logger_has_null_handler():
    """The ovphysx logger should have a NullHandler by default (library best practice).

    This ensures the library does not emit warnings about 'No handlers could be
    found for logger' when the application has not configured logging.
    """
    logger = logging.getLogger("ovphysx")
    handler_types = [type(h) for h in logger.handlers]
    assert logging.NullHandler in handler_types


# ============================================================================
# Log level constants: verify Python constants match C-side semantics
# ============================================================================


def test_default_log_level_matches_warning_constant():
    """The C-side default log level should match Python's LogLevel.WARNING.

    This validates the semantic meaning of at least one constant: if the C
    default is WARNING (3) and get_log_level() returns the same value as
    Python's LogLevel.WARNING, the two sides agree on what WARNING means.
    """
    from ovphysx.types import LogLevel

    from ovphysx import get_log_level

    assert get_log_level() == LogLevel.WARNING, (
        f"Expected default level to be LogLevel.WARNING ({LogLevel.WARNING}), " f"got {get_log_level()}"
    )


def test_log_level_constants_semantic_filtering(physx_sdk_cpu, native_log_callback_factory):
    """Python log level constants should produce correct C-side filtering.

    For each level, set the threshold, emit test messages from the C side,
    and verify a custom callback only receives messages at or above that level.
    This proves the Python constants control C-side filtering correctly,
    not just that they are in valid range.
    """
    from ovphysx._bindings import _lib
    from ovphysx.types import LogLevel

    from ovphysx import flush_log, set_log_level

    # Collect messages received by a custom C-level callback
    received = []

    @native_log_callback_factory
    def collector(level, message, channel, timestamp, user_data):
        try:
            import ctypes
            text = ctypes.string_at(message.ptr, message.length).decode("utf-8", errors="replace")
        except Exception:
            text = str(message)
        # Only collect the known test messages.
        if "[LogTest]" in text:
            received.append((level, text))

    import ctypes

    result = _lib.ovphysx_set_log_callback(
        LogLevel.VERBOSE, None, ctypes.cast(collector, ctypes.c_void_p), None
    )
    assert result.status == 0, "Failed to register test callback"

    try:
        # ovphysx-source messages emitted by ovphysx_log_emit_test_messages():
        #   FATAL:   NONE-only suppression probe (reported as ERROR)
        #   ERROR:   "[LogTest] ERROR test message"
        #   WARNING: "[LogTest] WARNING test message"
        #   INFO:    "[LogTest] INFO test message"
        #   VERBOSE: "[LogTest] VERBOSE test message"
        expectations = {
            LogLevel.NONE: set(),
            LogLevel.ERROR: {LogLevel.ERROR},
            LogLevel.WARNING: {LogLevel.ERROR, LogLevel.WARNING},
            LogLevel.INFO: {LogLevel.ERROR, LogLevel.WARNING, LogLevel.INFO},
            LogLevel.VERBOSE: {LogLevel.ERROR, LogLevel.WARNING, LogLevel.INFO, LogLevel.VERBOSE},
        }

        for threshold, expected_levels in expectations.items():
            received.clear()
            set_log_level(threshold)
            _lib.ovphysx_log_emit_test_messages()
            flush_log()

            actual_levels = {level for level, _ in received}
            assert actual_levels == expected_levels, (
                f"At threshold {threshold}: expected levels {expected_levels}, "
                f"got {actual_levels} from messages {received}"
            )
    finally:
        _lib.ovphysx_set_log_callback(LogLevel.DEFAULT, None, None, None)
        set_log_level(LogLevel.WARNING)


# ============================================================================
# Public API surface: logging functions should be importable from ovphysx
# ============================================================================


def test_logging_api_exported():
    """set_log_level, get_log_level, enable/disable_python_logging should be importable."""
    from ovphysx import (
        disable_python_logging,
        enable_python_logging,
        flush_log,
        get_log_level,
        set_log_level,
    )

    assert callable(set_log_level)
    assert callable(get_log_level)
    assert callable(enable_python_logging)
    assert callable(disable_python_logging)
    assert callable(flush_log)


# ============================================================================
# ovphysx source log level: set/get round-trip and default value
# ============================================================================


def test_set_get_log_level_roundtrip():
    """set_log_level / get_log_level should round-trip correctly."""
    from ovphysx.types import LogLevel

    from ovphysx import get_log_level, set_log_level

    original = get_log_level()
    try:
        set_log_level(LogLevel.VERBOSE)
        assert get_log_level() == LogLevel.VERBOSE

        set_log_level(LogLevel.ERROR)
        assert get_log_level() == LogLevel.ERROR
    finally:
        set_log_level(original)


def test_error_level_does_not_emit_expected_missing_plugin_warnings():
    """Fresh startup must not report expected-absent static plugins as warnings.

    Carbonite is process-global, so this must run in a fresh subprocess. The
    behavior belongs at the ovphysx API level because ovphysx coordinates the
    static-plugin bootstrap.
    """
    script = textwrap.dedent(
        """
        import logging
        import sys
        import ovphysx

        probe_warnings = []

        class ProbeWarningHandler(logging.Handler):
            def emit(self, record):
                message = record.getMessage()
                print(message, file=sys.stderr)
                if "getPluginDesc: Failed to find a plugin with a name" in message:
                    probe_warnings.append(message)

        logger = logging.getLogger("ovphysx.startup_probe_test")
        logger.handlers.clear()
        logger.addHandler(ProbeWarningHandler())
        logger.setLevel(logging.DEBUG)
        logger.propagate = False

        ovphysx.PhysX.set_cpu_mode(True)
        # ERROR is intentional: these static-Carbonite warnings bypassed the
        # documented ovphysx threshold in ovphysx 0.5.9.
        ovphysx.set_log_level(ovphysx.LogLevel.ERROR)
        ovphysx.enable_python_logging(logger.name)

        physx = None
        try:
            physx = ovphysx.PhysX(ignore_version_mismatch=True)
        finally:
            if physx is not None:
                physx.destroy()
            ovphysx.disable_python_logging()

        if probe_warnings:
            raise RuntimeError(f"startup emitted {len(probe_warnings)} missing-plugin warnings")
        """
    )

    result = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        timeout=120,
    )
    combined_output = result.stdout + result.stderr
    assert result.returncode == 0, (
        f"Fresh ovphysx startup failed with exit code {result.returncode}.\n"
        f"stdout:\n{result.stdout}\n"
        f"stderr:\n{result.stderr}"
    )
    assert "getPluginDesc: Failed to find a plugin with a name" not in combined_output


def test_none_suppresses_owned_runtime_warnings():
    """NONE suppresses the owned runtime warnings from NVBug 6605401."""
    usd_path = Path(__file__).parents[2] / "data" / "minimal_scene.usda"
    script = textwrap.dedent(
        """
            import logging
            import sys

            import ovstage

            gpu_warning = "GPU broadphase requires a CUDA context manager; falling back to ePABP."
            deformable_cuda_warning = (
                "InternalScene::updateDeformableTransforms: CUDA context unavailable, skipping."
            )
            records = []

            class CaptureHandler(logging.Handler):
                def emit(self, record):
                    records.append((record.getMessage(), record.ovphysx_channel))

            logger = logging.getLogger("ovphysx.runtime_source_test")
            logger.handlers.clear()
            logger.addHandler(CaptureHandler())
            logger.setLevel(logging.DEBUG)
            logger.propagate = False

            # ovstage owns libovstage.so; construct the stage before any ovphysx API
            # that loads libovphysx.so.
            stage = ovstage.Stage("ovphysx-runtime-source-test")

            import ovphysx

            ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])
            level = getattr(ovphysx.LogLevel, sys.argv[2])
            expect_gpu_warning = level == ovphysx.LogLevel.WARNING
            ovphysx.PhysX.set_cpu_mode(True)
            ovphysx.set_log_level(level)
            ovphysx.enable_python_logging(logger.name)
            physx = ovphysx.PhysX(ignore_version_mismatch=True)
            attached = False
            try:
                ovstage.population.open_usd(
                    stage,
                    sys.argv[1],
                    ordinal=1,
                    domains=ovstage.PopulationDomain.PHYSICS,
                )
                stage.advance_write_floor(ordinal=1).wait()
                physx.attach_ovstage(stage, read_ordinal=1)
                attached = True
                physx.step(1.0 / 60.0)
                physx.wait_all()
            finally:
                if attached:
                    physx.detach_ovstage()
                stage.destroy()
                physx.destroy()
                ovphysx.disable_python_logging()

            gpu_records = [record for record in records if gpu_warning in record[0]]
            deformable_cuda_records = [
                record for record in records if deformable_cuda_warning in record[0]
            ]
            if deformable_cuda_records:
                raise RuntimeError(
                    f"rigid-only scene emitted deformable CUDA warnings: {deformable_cuda_records}"
                )
            if expect_gpu_warning:
                if [record[1] for record in gpu_records] != ["omni.physx"]:
                    raise RuntimeError(f"unexpected GPU warning records: {gpu_records}")
            elif gpu_records:
                raise RuntimeError(f"NONE forwarded GPU warnings: {gpu_records}")
            """
    )

    for level_name in ("NONE", "WARNING"):
        result = subprocess.run(
            [sys.executable, "-c", script, str(usd_path), level_name],
            capture_output=True,
            text=True,
            timeout=120,
        )
        assert result.returncode == 0, (
            f"Runtime-source logging probe failed at {level_name}.\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )


def test_none_does_not_consume_gpu_fallback_once_latch():
    """A muted GPU-fallback emit must not eat the process-wide WARN_ONCE latch."""
    usd_path = Path(__file__).parents[2] / "data" / "minimal_scene.usda"
    script = textwrap.dedent(
        """
        import logging
        import sys

        import ovphysx
        import ovstage

        gpu_warning = "GPU broadphase requires a CUDA context manager; falling back to ePABP."
        records = []

        class CaptureHandler(logging.Handler):
            def emit(self, record):
                records.append((record.getMessage(), record.ovphysx_channel))

        logger = logging.getLogger("ovphysx.runtime_once_latch_test")
        logger.handlers.clear()
        logger.addHandler(CaptureHandler())
        logger.setLevel(logging.DEBUG)
        logger.propagate = False

        def run_attach(physx, stage):
            from ovphysx.types import ObjectScope, SimObjectType

            attached = False
            try:
                physx.attach_ovstage(stage, read_ordinal=1)
                attached = True
                physx.wait_all()
                # Exercise the runtime read path. The GPU-fallback warning fires during the scene
                # setup that wait_all drains, so capturing it does not depend on the read itself.
                with physx.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL):
                    pass
            finally:
                if attached:
                    physx.detach_ovstage()

        # ovstage owns libovstage.so; construct it before PhysX so attach can
        # reuse the loaded USD closure.
        stage = ovstage.Stage("ovphysx-once-latch-test")
        ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])
        ovphysx.PhysX.set_cpu_mode(True)
        ovphysx.set_log_level(ovphysx.LogLevel.NONE)
        ovphysx.enable_python_logging(logger.name)
        physx = ovphysx.PhysX(ignore_version_mismatch=True)
        try:
            ovstage.population.open_usd(
                stage,
                sys.argv[1],
                ordinal=1,
                domains=ovstage.PopulationDomain.PHYSICS,
            )
            stage.advance_write_floor(ordinal=1).wait()
            run_attach(physx, stage)
            gpu_at_none = [record for record in records if gpu_warning in record[0]]
            if gpu_at_none:
                raise RuntimeError(f"NONE forwarded GPU warnings: {gpu_at_none}")
            records.clear()
            ovphysx.set_log_level(ovphysx.LogLevel.WARNING)
            run_attach(physx, stage)
            gpu_at_warning = [record for record in records if gpu_warning in record[0]]
            if [record[1] for record in gpu_at_warning] != ["omni.physx"]:
                raise RuntimeError(
                    f"WARNING after NONE lost the GPU fallback warning: {gpu_at_warning}"
                )
        finally:
            stage.destroy()
            physx.destroy()
            ovphysx.disable_python_logging()
        """
    )
    result = subprocess.run(
        [sys.executable, "-c", script, str(usd_path)],
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert result.returncode == 0, (
        "NONE-then-WARNING once-latch probe failed.\n"
        f"stdout:\n{result.stdout}\n"
        f"stderr:\n{result.stderr}"
    )


def test_warning_does_not_override_host_global_disable():
    """A source-level update must preserve Carbonite's host-global disable."""
    usd_path = Path(__file__).parents[2] / "data" / "minimal_scene.usda"
    script = textwrap.dedent(
        """
        import logging
        import sys

        import ovstage

        gpu_warning = "GPU broadphase requires a CUDA context manager; falling back to ePABP."
        records = []

        class CaptureHandler(logging.Handler):
            def emit(self, record):
                records.append((record.getMessage(), record.ovphysx_channel))

        logger = logging.getLogger("ovphysx.host_global_disable_test")
        logger.handlers.clear()
        logger.addHandler(CaptureHandler())
        logger.setLevel(logging.DEBUG)
        logger.propagate = False

        stage = ovstage.Stage("ovphysx-host-global-disable-test")
        import ovphysx
        from ovphysx._bindings import _lib

        ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])
        ovphysx.PhysX.set_cpu_mode(True)
        ovphysx.enable_python_logging(logger.name)
        physx = ovphysx.PhysX(ignore_version_mismatch=True)
        attached = False
        try:
            _lib.ovphysx_log_set_global_enabled_for_test(False)
            if _lib.ovphysx_log_get_global_enabled_for_test():
                raise RuntimeError("failed to disable Carbonite process-global logging")

            ovphysx.set_log_level(ovphysx.LogLevel.WARNING)
            if _lib.ovphysx_log_get_global_enabled_for_test():
                raise RuntimeError("set_log_level(WARNING) re-enabled process-global logging")

            ovstage.population.open_usd(
                stage,
                sys.argv[1],
                ordinal=1,
                domains=ovstage.PopulationDomain.PHYSICS,
            )
            stage.advance_write_floor(ordinal=1).wait()
            physx.attach_ovstage(stage, read_ordinal=1)
            attached = True
            physx.wait_all()
            leaked = [record for record in records if gpu_warning in record[0]]
            if leaked:
                raise RuntimeError(f"host-disabled GPU warning reached the logger: {leaked}")
        finally:
            # Restore host-owned process state before teardown can log.
            _lib.ovphysx_log_set_global_enabled_for_test(True)
            if attached:
                physx.detach_ovstage()
            stage.destroy()
            physx.destroy()
            ovphysx.disable_python_logging()
        """
    )
    result = subprocess.run(
        [sys.executable, "-c", script, str(usd_path)],
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert result.returncode == 0, (
        "Host-global logging disable probe failed.\n"
        f"stdout:\n{result.stdout}\n"
        f"stderr:\n{result.stderr}"
    )


def test_default_log_level_is_warning():
    """Default ovphysx source log level should be LogLevel.WARNING."""
    from ovphysx.types import LogLevel

    from ovphysx import get_log_level

    assert get_log_level() == LogLevel.WARNING


# ============================================================================
# PhysX class: no per-instance log_level parameter, the global API controls it
# ============================================================================


def test_physx_init_no_log_level_param():
    """PhysX.__init__ should NOT have a log_level parameter.

    Log level is controlled globally via ovphysx.set_log_level(), not
    per-instance via the constructor.
    """
    import inspect

    from ovphysx import PhysX

    sig = inspect.signature(PhysX.__init__)
    assert (
        "log_level" not in sig.parameters
    ), "log_level was removed from PhysX.__init__ in favour of ovphysx.set_log_level()"


# ============================================================================
# Internal _bindings logger: verify it routes through the ovphysx hierarchy
# ============================================================================


def test_bindings_logger_wired():
    """The _bindings module should use a logger routed through the ovphysx hierarchy."""
    from ovphysx import _bindings

    assert hasattr(_bindings, "_logger"), "_bindings module should have a _logger attribute"
    assert isinstance(_bindings._logger, logging.Logger)
    assert _bindings._logger.name == "ovphysx" or _bindings._logger.name.startswith("ovphysx.")


def test_bindings_logger_captures_messages(caplog):
    """Messages emitted through _bindings._logger should be capturable via caplog."""
    from ovphysx import _bindings

    with caplog.at_level(logging.DEBUG, logger="ovphysx"):
        _bindings._logger.info("test_log_capture_verification")

    info_records = [r for r in caplog.records if "test_log_capture_verification" in r.message]
    assert len(info_records) == 1, (
        "Expected the test message to be captured by caplog; " f"got {len(info_records)} matching records"
    )


# ============================================================================
# enable_python_logging / disable_python_logging
#
# These bridge native CARB_LOG_* messages into Python's logging system via a
# C-level callback.  Most tests here can run without a PhysX instance (the
# callback is registered pre-init and wired once Carbonite initializes).
# ============================================================================


def test_enable_disable_python_logging_roundtrip():
    """enable_python_logging and disable_python_logging should toggle bridge state."""
    import ovphysx.api as _api

    from ovphysx import disable_python_logging, enable_python_logging

    # Precondition: bridge should be inactive
    assert _api._python_log_callback is None, "Bridge should start inactive"
    assert _api._python_log_logger_name is None
    assert not _api._python_log_retained_callbacks

    # Enable: bridge should become active with the default logger name
    enable_python_logging()
    assert _api._python_log_callback is not None, "Bridge should be active after enable"
    assert _api._python_log_logger_name == "ovphysx"

    # Disable: bridge should become inactive again
    disable_python_logging()
    assert _api._python_log_callback is None, "Bridge should be inactive after disable"
    assert _api._python_log_logger_name is None
    assert not _api._python_log_retained_callbacks

    # A second disable is a safe no-op: no exception, state unchanged.
    disable_python_logging()
    assert _api._python_log_callback is None


def test_successful_process_shutdown_clears_python_logging_bridge(monkeypatch):
    """Final lifecycle shutdown must drop Python's native callback owner."""
    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    callback_owner = object()
    retained_owner = object()
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 1)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(_api, "_python_log_callback", callback_owner)
    monkeypatch.setattr(_api, "_python_log_logger_name", "test-shutdown")
    monkeypatch.setattr(_api, "_python_log_retained_callbacks", (retained_owner,))
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_shutdown",
        lambda: SimpleNamespace(status=ApiStatus.SUCCESS),
    )

    _api._release_process_lifecycle()

    assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0
    assert _api._python_log_callback is None
    assert _api._python_log_logger_name is None
    assert not _api._python_log_retained_callbacks
    assert not _api._python_log_callback_transition
    assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN


def test_process_shutdown_waits_for_python_logging_enable_transition(monkeypatch):
    """Final shutdown must retain a concurrently installed CFUNCTYPE owner."""
    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    from ovphysx import disable_python_logging, enable_python_logging

    enable_entered = threading.Event()
    release_enable = threading.Event()
    shutdown_waiting = threading.Event()
    shutdown_entered = threading.Event()
    release_shutdown = threading.Event()
    enable_errors = []
    shutdown_errors = []
    shutdown_observation = {}
    original_condition_wait = _api._python_log_callback_condition.wait

    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 1)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(_api, "_python_log_callback", None)
    monkeypatch.setattr(_api, "_python_log_logger_name", None)
    monkeypatch.setattr(_api, "_python_log_retained_callbacks", ())
    monkeypatch.setattr(_api, "_python_log_callback_transition", False)

    def blocking_setter(*args):
        enable_entered.set()
        if not release_enable.wait(timeout=5):
            return SimpleNamespace(status=ApiStatus.ERROR)
        return SimpleNamespace(status=ApiStatus.SUCCESS)

    def observing_shutdown():
        shutdown_observation["callback"] = _api._python_log_callback
        shutdown_observation["transition"] = _api._python_log_callback_transition
        shutdown_observation["process_shutdown"] = _api._PROCESS_LIFECYCLE_SHUTTING_DOWN
        shutdown_entered.set()
        if not release_shutdown.wait(timeout=5):
            return SimpleNamespace(status=ApiStatus.ERROR)
        return SimpleNamespace(status=ApiStatus.SUCCESS)

    def observe_condition_wait(timeout=None):
        shutdown_waiting.set()
        return original_condition_wait(timeout)

    def enable_bridge():
        try:
            enable_python_logging("ovphysx.concurrent-shutdown")
        except BaseException as error:
            enable_errors.append(error)

    def release_lifecycle():
        try:
            _api._release_process_lifecycle()
        except BaseException as error:
            shutdown_errors.append(error)

    monkeypatch.setattr(_api._lib, "ovphysx_set_log_callback", blocking_setter)
    monkeypatch.setattr(_api._lib, "ovphysx_shutdown", observing_shutdown)
    monkeypatch.setattr(_api._python_log_callback_condition, "wait", observe_condition_wait)

    enable_worker = threading.Thread(target=enable_bridge)
    shutdown_worker = threading.Thread(target=release_lifecycle)
    shutdown_worker_started = False
    enable_worker.start()
    try:
        assert enable_entered.wait(timeout=5), "Enable did not reach the native setter"
        shutdown_worker.start()
        shutdown_worker_started = True
        assert shutdown_waiting.wait(timeout=5), "Shutdown did not wait for the logging transition"
        assert not shutdown_entered.is_set(), (
            "Native shutdown entered before bridge ownership was available"
        )
        release_enable.set()
        assert shutdown_entered.wait(timeout=5), "Shutdown did not claim the callback transition"
        with pytest.raises(RuntimeError, match="transition is in progress"):
            enable_python_logging("ovphysx.during-shutdown")
        with pytest.raises(RuntimeError, match="transition is in progress"):
            disable_python_logging()
    finally:
        release_enable.set()
        release_shutdown.set()
        enable_worker.join(timeout=5)
        if shutdown_worker_started:
            shutdown_worker.join(timeout=5)

    assert not enable_worker.is_alive(), "Enable transition did not finish"
    assert not shutdown_worker.is_alive(), "Process shutdown did not finish"
    assert not enable_errors
    assert not shutdown_errors
    assert shutdown_entered.is_set()
    assert shutdown_observation["callback"] is not None
    assert shutdown_observation["transition"] is True
    assert shutdown_observation["process_shutdown"] is True
    assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0
    assert _api._python_log_callback is None
    assert _api._python_log_logger_name is None
    assert not _api._python_log_retained_callbacks
    assert not _api._python_log_callback_transition
    assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN


def test_failed_process_shutdown_retains_python_logging_callback(monkeypatch):
    """A failed native shutdown must not drop a potentially live callback."""
    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    callback_owner = object()
    retained_owner = object()
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 1)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(_api, "_python_log_callback", callback_owner)
    monkeypatch.setattr(_api, "_python_log_logger_name", "test-failed-shutdown")
    monkeypatch.setattr(_api, "_python_log_retained_callbacks", (retained_owner,))
    monkeypatch.setattr(_api, "_python_log_callback_transition", False)
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_shutdown",
        lambda: SimpleNamespace(status=ApiStatus.ERROR),
    )
    monkeypatch.setattr(_api, "_get_last_error_from_lib", lambda: "injected shutdown failure")

    with pytest.raises(RuntimeError, match="injected shutdown failure"):
        _api._release_process_lifecycle()

    assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0
    assert _api._python_log_callback is callback_owner
    assert _api._python_log_logger_name == "test-failed-shutdown"
    assert _api._python_log_retained_callbacks == (retained_owner,)
    assert not _api._python_log_callback_transition
    assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN


def test_callback_side_shutdown_does_not_wait_for_logging_transition(monkeypatch):
    """Callback-side destroy must reject intact and succeed when retried later."""
    import ctypes

    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    from ovphysx import PhysX

    transition_entered = threading.Event()
    release_transition = threading.Event()
    callback_started = threading.Event()
    callback_errors = []
    destroy_calls = []
    shutdown_calls = []
    callback_owner = object()

    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 1)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(_api, "_python_log_callback", callback_owner)
    monkeypatch.setattr(_api, "_python_log_logger_name", "test-callback-shutdown")
    monkeypatch.setattr(_api, "_python_log_retained_callbacks", ())
    monkeypatch.setattr(_api, "_python_log_callback_transition", False)
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_destroy_instance",
        lambda handle: destroy_calls.append(handle) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )

    def successful_shutdown():
        shutdown_calls.append(True)
        return SimpleNamespace(status=ApiStatus.SUCCESS)

    monkeypatch.setattr(_api._lib, "ovphysx_shutdown", successful_shutdown)

    physx = PhysX.__new__(PhysX)
    physx._lib = _api._lib
    physx._omni_physx_sdk_handle = ctypes.c_uint64(17)
    physx._attached_ovstage = None
    physx._lifecycle_acquired = True
    physx._released = False

    def hold_transition():
        with _api._python_log_callback_condition:
            _api._python_log_callback_transition = True
        transition_entered.set()
        release_transition.wait(timeout=5)
        with _api._python_log_callback_condition:
            _api._python_log_callback_transition = False
            _api._python_log_callback_condition.notify_all()

    def release_from_callback():
        _api._python_log_callback_context.active = True
        callback_started.set()
        try:
            physx.destroy()
        except BaseException as error:
            callback_errors.append(error)
        finally:
            _api._python_log_callback_context.active = False

    transition_worker = threading.Thread(target=hold_transition)
    callback_worker = threading.Thread(target=release_from_callback)
    callback_worker_started = False
    transition_worker.start()
    try:
        assert transition_entered.wait(timeout=5), "Logging transition did not start"
        callback_worker.start()
        callback_worker_started = True
        assert callback_started.wait(timeout=5), "Callback-side destroy did not start"
        callback_worker.join(timeout=5)
        assert not callback_worker.is_alive(), "Callback-side shutdown waited on its own transition"
    finally:
        release_transition.set()
        transition_worker.join(timeout=5)
        if callback_worker_started:
            callback_worker.join(timeout=5)

    try:
        assert not transition_worker.is_alive(), "Logging transition did not finish"
        assert len(callback_errors) == 1
        assert "ovphysx_shutdown cannot be called from within a log callback" in str(callback_errors[0])
        assert destroy_calls == []
        assert not shutdown_calls
        assert physx._omni_physx_sdk_handle.value == 17
        assert physx._lifecycle_acquired
        assert not physx._released
        assert _api._PROCESS_LIFECYCLE_REFCOUNT == 1
        assert _api._python_log_callback is callback_owner
        assert _api._python_log_logger_name == "test-callback-shutdown"
        assert not _api._python_log_callback_transition

        physx.destroy()

        assert destroy_calls == [17]
        assert shutdown_calls == [True]
        assert physx._omni_physx_sdk_handle is None
        assert not physx._lifecycle_acquired
        assert physx._released
        assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0
        assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN
        assert _api._python_log_callback is None
        assert _api._python_log_logger_name is None
        assert not _api._python_log_retained_callbacks

        _api._python_log_callback_context.active = True
        try:
            physx.destroy()
        finally:
            _api._python_log_callback_context.active = False
        assert destroy_calls == [17]
        assert shutdown_calls == [True]
    finally:
        _api._python_log_callback_context.active = False
        if getattr(physx, "_lifecycle_acquired", False):
            try:
                physx.destroy()
            except BaseException:
                pass


def test_concurrent_physx_construction_waits_for_initialization(monkeypatch):
    """Concurrent public construction waits and shares one native initialization."""
    import ctypes

    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    from ovphysx import PhysX

    initialize_entered = threading.Event()
    release_initialize = threading.Event()
    second_started = threading.Event()
    second_done = threading.Event()
    errors = []
    instances = []
    initialize_calls = []
    create_calls = []
    create_lock = threading.Lock()
    destroy_calls = []
    shutdown_calls = []

    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 0)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_INITIALIZING", False)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(_api, "_python_log_callback", None)
    monkeypatch.setattr(_api, "_python_log_logger_name", None)
    monkeypatch.setattr(_api, "_python_log_retained_callbacks", ())
    monkeypatch.setattr(_api, "_python_log_callback_transition", False)

    def initialize():
        initialize_calls.append(True)
        initialize_entered.set()
        if not release_initialize.wait(timeout=5):
            return SimpleNamespace(status=ApiStatus.ERROR)
        return SimpleNamespace(status=ApiStatus.SUCCESS)

    def construct(done=None):
        if done is not None:
            second_started.set()
        try:
            instances.append(PhysX(ignore_version_mismatch=True))
        except BaseException as error:
            errors.append(error)
        finally:
            if done is not None:
                done.set()

    def create_instance(args, out_handle):
        with create_lock:
            handle = 101 + len(create_calls)
            create_calls.append(handle)
        ctypes.cast(out_handle, ctypes.POINTER(ctypes.c_uint64))[0] = handle
        return SimpleNamespace(status=ApiStatus.SUCCESS)

    monkeypatch.setattr(_api._lib, "ovphysx_initialize", initialize)
    monkeypatch.setattr(_api._lib, "ovphysx_create_instance", create_instance)
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_destroy_instance",
        lambda handle: destroy_calls.append(handle) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_shutdown",
        lambda: shutdown_calls.append(True) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )

    first = threading.Thread(target=construct)
    second = threading.Thread(target=construct, args=(second_done,))
    first.start()
    try:
        assert initialize_entered.wait(timeout=5)
        second.start()
        assert second_started.wait(timeout=5)
        assert not second_done.wait(timeout=0.1)
    finally:
        release_initialize.set()
        first.join(timeout=5)
        second.join(timeout=5)

    try:
        assert not first.is_alive()
        assert not second.is_alive()
        assert not errors
        assert len(instances) == 2
        assert initialize_calls == [True]
        assert sorted(create_calls) == [101, 102]
        assert _api._PROCESS_LIFECYCLE_REFCOUNT == 2
    finally:
        for instance in instances:
            instance.destroy()

    assert sorted(destroy_calls) == [101, 102]
    assert shutdown_calls == [True]
    assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0


def test_constructor_failure_before_ownership_emits_no_resource_warning(monkeypatch):
    """An early constructor failure owns no resource and emits no cleanup warning."""
    import gc
    import warnings

    import ovphysx.api as _api

    from ovphysx import PhysX

    initialize_calls = []

    def fail_version_check():
        raise RuntimeError("injected version failure")

    monkeypatch.setattr(_api, "_check_version_match", fail_version_check)
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_initialize",
        lambda: initialize_calls.append(True),
    )

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always", ResourceWarning)
        with pytest.raises(RuntimeError, match="injected version failure"):
            PhysX()
        gc.collect()

    assert initialize_calls == []
    assert not any(issubclass(item.category, ResourceWarning) for item in caught)


def test_shared_lifecycle_token_exit_interruption_releases_public_constructor(monkeypatch):
    """A public constructor interrupted after sharing must release exactly its token."""
    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    from ovphysx import PhysX

    real_condition = _api._PROCESS_LIFECYCLE_CONDITION
    initialize_calls = []
    create_calls = []
    shutdown_calls = []

    class InterruptAfterSharedToken:
        def __init__(self):
            self._raised = False

        def __enter__(self):
            real_condition.__enter__()
            return self

        def __exit__(self, exc_type, exc_value, traceback):
            interrupt = not self._raised and _api._PROCESS_LIFECYCLE_REFCOUNT == 2
            if interrupt:
                self._raised = True
            suppressed = real_condition.__exit__(exc_type, exc_value, traceback)
            if interrupt:
                # The existing peer may release after this constructor published
                # its token but before its interrupted cleanup reacquires the lock.
                _api._release_process_lifecycle()
                raise KeyboardInterrupt("injected shared-token exit interruption")
            return suppressed

        def __getattr__(self, name):
            return getattr(real_condition, name)

    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_CONDITION", InterruptAfterSharedToken())
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 1)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_INITIALIZING", False)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(_api, "_python_log_callback", None)
    monkeypatch.setattr(_api, "_python_log_logger_name", None)
    monkeypatch.setattr(_api, "_python_log_retained_callbacks", ())
    monkeypatch.setattr(_api, "_python_log_callback_transition", False)
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_initialize",
        lambda: initialize_calls.append(True) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_create_instance",
        lambda *args: create_calls.append(True) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_shutdown",
        lambda: shutdown_calls.append(True) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )

    with pytest.raises(KeyboardInterrupt, match="shared-token exit interruption"):
        PhysX(ignore_version_mismatch=True)

    assert not initialize_calls
    assert not create_calls
    assert shutdown_calls == [True]
    assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0
    assert not _api._PROCESS_LIFECYCLE_INITIALIZING
    assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN


def test_callback_dependency_construction_does_not_deadlock_final_shutdown(monkeypatch):
    """Final shutdown rejects callback dependencies before they can wait."""
    import ctypes

    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    from ovphysx import PhysX

    callback_started = threading.Event()
    callback_done = threading.Event()
    dependency_started = threading.Event()
    callback_errors = []
    callback_instances = []
    callback_workers = []
    dependency_errors = []
    dependency_instances = []
    dependency_workers = []
    release_errors = []
    initialize_calls = []
    create_calls = []
    destroy_calls = []
    shutdown_calls = []

    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 1)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(_api, "_python_log_callback", object())
    monkeypatch.setattr(_api, "_python_log_logger_name", "test-callback-construction")
    monkeypatch.setattr(_api, "_python_log_retained_callbacks", ())
    monkeypatch.setattr(_api, "_python_log_callback_transition", False)
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_initialize",
        lambda: initialize_calls.append(True) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )

    def create_instance(args, out_handle):
        create_calls.append(True)
        ctypes.cast(out_handle, ctypes.POINTER(ctypes.c_uint64))[0] = 73
        return SimpleNamespace(status=ApiStatus.SUCCESS)

    monkeypatch.setattr(_api._lib, "ovphysx_create_instance", create_instance)
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_destroy_instance",
        lambda handle: destroy_calls.append(handle) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )

    def construct_dependency():
        dependency_started.set()
        try:
            dependency_instances.append(PhysX(ignore_version_mismatch=True))
        except BaseException as error:
            dependency_errors.append(error)

    def invoke_callback():
        _api._python_log_callback_context.active = True
        callback_started.set()
        try:
            try:
                callback_instances.append(PhysX(ignore_version_mismatch=True))
            except BaseException as error:
                callback_errors.append(error)

            # A callback may wait on unrelated application work. During final
            # shutdown that worker must reject lifecycle acquisition instead
            # of waiting on shutdown, which is draining this callback.
            dependency = threading.Thread(target=construct_dependency)
            dependency_workers.append(dependency)
            dependency.start()
            dependency.join(timeout=5)
        finally:
            _api._python_log_callback_context.active = False
            callback_done.set()

    def draining_shutdown():
        shutdown_calls.append(True)
        if len(shutdown_calls) > 1:
            return SimpleNamespace(status=ApiStatus.SUCCESS)
        worker = threading.Thread(target=invoke_callback)
        callback_workers.append(worker)
        worker.start()
        if not callback_started.wait(timeout=5) or not dependency_started.wait(timeout=5):
            return SimpleNamespace(status=ApiStatus.ERROR)
        status = ApiStatus.SUCCESS if callback_done.wait(timeout=5) else ApiStatus.ERROR
        return SimpleNamespace(status=status)

    monkeypatch.setattr(_api._lib, "ovphysx_shutdown", draining_shutdown)

    try:
        try:
            _api._release_process_lifecycle()
        except BaseException as error:
            release_errors.append(error)
    finally:
        for worker in callback_workers:
            worker.join(timeout=5)
        for worker in dependency_workers:
            worker.join(timeout=5)
        for instance in callback_instances:
            instance.destroy()
        for instance in dependency_instances:
            instance.destroy()

    assert all(not worker.is_alive() for worker in callback_workers)
    assert all(not worker.is_alive() for worker in dependency_workers)
    assert callback_started.is_set()
    assert dependency_started.is_set()
    assert not release_errors
    assert len(callback_errors) == 1
    assert "PhysX() cannot be called from within a native log callback" in str(callback_errors[0])
    assert len(dependency_errors) == 1
    assert "ovphysx process shutdown is in progress" in str(dependency_errors[0])
    assert not callback_instances
    assert not dependency_instances
    assert not initialize_calls
    assert not create_calls
    assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0
    assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN

    retry = PhysX(ignore_version_mismatch=True)
    try:
        assert retry.handle == 73
    finally:
        retry.destroy()

    assert initialize_calls == [True]
    assert create_calls == [True]
    assert destroy_calls == [73]
    assert shutdown_calls == [True, True]
    assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0
    assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN


def test_callback_dependency_construction_does_not_deadlock_initialization(monkeypatch):
    """Direct callback reentry rejects while an ordinary dependency waits."""
    import ctypes

    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    from ovphysx import PhysX

    callback_started = threading.Event()
    callback_done = threading.Event()
    dependency_started = threading.Event()
    callback_errors = []
    callback_instances = []
    callback_workers = []
    dependency_errors = []
    dependency_instances = []
    dependency_workers = []
    construction_errors = []
    initialize_calls = []
    create_calls = []
    destroy_calls = []
    shutdown_calls = []

    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 0)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_INITIALIZING", False)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(_api, "_python_log_callback", object())
    monkeypatch.setattr(_api, "_python_log_logger_name", "test-callback-initialize")
    monkeypatch.setattr(_api, "_python_log_retained_callbacks", ())
    monkeypatch.setattr(_api, "_python_log_callback_transition", False)

    def construct_dependency():
        dependency_started.set()
        try:
            dependency_instances.append(PhysX(ignore_version_mismatch=True))
        except BaseException as error:
            dependency_errors.append(error)

    def invoke_callback():
        _api._python_log_callback_context.active = True
        callback_started.set()
        try:
            try:
                callback_instances.append(PhysX(ignore_version_mismatch=True))
            except BaseException as error:
                callback_errors.append(error)

            dependency = threading.Thread(target=construct_dependency)
            dependency_workers.append(dependency)
            dependency.start()
        finally:
            _api._python_log_callback_context.active = False
            callback_done.set()

    def initializing_with_callback():
        initialize_calls.append(True)
        if len(initialize_calls) > 1:
            return SimpleNamespace(status=ApiStatus.ERROR)
        worker = threading.Thread(target=invoke_callback)
        callback_workers.append(worker)
        worker.start()
        if not callback_started.wait(timeout=5) or not dependency_started.wait(timeout=5):
            return SimpleNamespace(status=ApiStatus.ERROR)
        status = ApiStatus.SUCCESS if callback_done.wait(timeout=5) else ApiStatus.ERROR
        return SimpleNamespace(status=status)

    def create_instance(args, out_handle):
        create_calls.append(True)
        ctypes.cast(out_handle, ctypes.POINTER(ctypes.c_uint64))[0] = 89
        return SimpleNamespace(status=ApiStatus.SUCCESS)

    monkeypatch.setattr(_api._lib, "ovphysx_initialize", initializing_with_callback)
    monkeypatch.setattr(_api._lib, "ovphysx_create_instance", create_instance)
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_destroy_instance",
        lambda handle: destroy_calls.append(handle) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_shutdown",
        lambda: shutdown_calls.append(True) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )

    physx = None
    try:
        try:
            physx = PhysX(ignore_version_mismatch=True)
        except BaseException as error:
            construction_errors.append(error)
    finally:
        for worker in callback_workers:
            worker.join(timeout=5)
        for worker in dependency_workers:
            worker.join(timeout=5)
        for instance in callback_instances:
            instance.destroy()
        for instance in dependency_instances:
            instance.destroy()

    try:
        assert all(not worker.is_alive() for worker in callback_workers)
        assert all(not worker.is_alive() for worker in dependency_workers)
        assert callback_started.is_set()
        assert dependency_started.is_set()
        assert not construction_errors
        assert len(callback_errors) == 1
        assert "PhysX() cannot be called from within a native log callback" in str(callback_errors[0])
        assert not dependency_errors
        assert not callback_instances
        assert len(dependency_instances) == 1
        assert initialize_calls == [True]
        assert create_calls == [True, True]
        assert physx is not None
        assert physx.handle == 89
        assert _api._PROCESS_LIFECYCLE_REFCOUNT == 1
        assert not _api._PROCESS_LIFECYCLE_INITIALIZING

        physx.destroy()

        assert destroy_calls == [89, 89]
        assert shutdown_calls == [True]
        assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0
        assert not _api._PROCESS_LIFECYCLE_INITIALIZING
        assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN
    finally:
        if physx is not None and getattr(physx, "_lifecycle_acquired", False):
            try:
                physx.destroy()
            except BaseException:
                pass


def test_post_commit_initialization_interruption_rolls_back_lifecycle(monkeypatch):
    """An ambiguous initialization interruption must release any native token."""
    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    shutdown_observations = []
    waiter_errors = []

    class InterruptingResult:
        @property
        def status(self):
            raise KeyboardInterrupt("injected post-initialize interruption")

    def observing_shutdown():
        shutdown_observations.append(
            (
                _api._PROCESS_LIFECYCLE_REFCOUNT,
                _api._PROCESS_LIFECYCLE_INITIALIZING,
                _api._PROCESS_LIFECYCLE_SHUTTING_DOWN,
            )
        )
        try:
            _api._acquire_process_lifecycle()
        except BaseException as error:
            waiter_errors.append(error)
        return SimpleNamespace(status=ApiStatus.SUCCESS)

    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 0)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_INITIALIZING", False)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(_api, "_python_log_callback", object())
    monkeypatch.setattr(_api, "_python_log_logger_name", "test-interrupted-initialize")
    monkeypatch.setattr(_api, "_python_log_retained_callbacks", (object(),))
    monkeypatch.setattr(_api, "_python_log_callback_transition", False)
    monkeypatch.setattr(_api._lib, "ovphysx_initialize", lambda: InterruptingResult())
    monkeypatch.setattr(_api._lib, "ovphysx_shutdown", observing_shutdown)

    with pytest.raises(KeyboardInterrupt, match="post-initialize interruption"):
        _api._acquire_process_lifecycle()

    assert shutdown_observations == [(0, False, True)]
    assert len(waiter_errors) == 1
    assert "ovphysx process shutdown is in progress" in str(waiter_errors[0])
    assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0
    assert not _api._PROCESS_LIFECYCLE_INITIALIZING
    assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN
    assert _api._python_log_callback is None
    assert _api._python_log_logger_name is None
    assert not _api._python_log_retained_callbacks
    assert not _api._python_log_callback_transition


def test_ambiguous_initialization_handoff_interruption_finishes_shutdown(monkeypatch):
    """Cleanup interruption cannot strand the ambiguous shutdown handoff."""
    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    real_condition = _api._PROCESS_LIFECYCLE_CONDITION
    initialize_calls = []
    shutdown_calls = []

    class InterruptingResult:
        @property
        def status(self):
            raise KeyboardInterrupt("injected post-initialize interruption")

    class InterruptAfterShutdownHandoff:
        def __init__(self):
            self._raised = False

        def __enter__(self):
            real_condition.__enter__()
            return self

        def wait_for(self, predicate, timeout=None):
            return real_condition.wait_for(predicate, timeout)

        def notify_all(self):
            real_condition.notify_all()

        def __exit__(self, exc_type, exc_value, traceback):
            interrupt = (
                not self._raised
                and _api._PROCESS_LIFECYCLE_REFCOUNT == 0
                and not _api._PROCESS_LIFECYCLE_INITIALIZING
                and _api._PROCESS_LIFECYCLE_SHUTTING_DOWN
            )
            if interrupt:
                self._raised = True
            real_condition.__exit__(exc_type, exc_value, traceback)
            if interrupt:
                raise KeyboardInterrupt("injected shutdown-handoff interruption")
            return False

    def initialize():
        initialize_calls.append(True)
        if len(initialize_calls) == 1:
            return InterruptingResult()
        return SimpleNamespace(status=ApiStatus.SUCCESS)

    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_CONDITION", InterruptAfterShutdownHandoff())
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 0)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_INITIALIZING", False)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(_api, "_python_log_callback", None)
    monkeypatch.setattr(_api, "_python_log_logger_name", None)
    monkeypatch.setattr(_api, "_python_log_retained_callbacks", ())
    monkeypatch.setattr(_api, "_python_log_callback_transition", False)
    monkeypatch.setattr(_api._lib, "ovphysx_initialize", initialize)
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_shutdown",
        lambda: shutdown_calls.append(True) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )

    with pytest.raises(KeyboardInterrupt, match="post-initialize interruption"):
        _api._acquire_process_lifecycle()

    assert shutdown_calls == [True]
    assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0
    assert not _api._PROCESS_LIFECYCLE_INITIALIZING
    assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN

    _api._acquire_process_lifecycle()
    _api._release_process_lifecycle()

    assert initialize_calls == [True, True]
    assert shutdown_calls == [True, True]


def test_initialization_claim_interruption_releases_waiters(monkeypatch):
    """Interruption after claiming initialization must not strand the transition."""
    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    real_condition = _api._PROCESS_LIFECYCLE_CONDITION
    initialize_calls = []
    shutdown_calls = []

    class InterruptAfterClaim:
        def __init__(self):
            self._raised = False

        def __enter__(self):
            real_condition.__enter__()
            return self

        def wait_for(self, predicate, timeout=None):
            return real_condition.wait_for(predicate, timeout)

        def notify_all(self):
            real_condition.notify_all()

        def __exit__(self, exc_type, exc_value, traceback):
            interrupt = (
                not self._raised
                and _api._PROCESS_LIFECYCLE_INITIALIZING
                and _api._PROCESS_LIFECYCLE_REFCOUNT == 0
            )
            if interrupt:
                self._raised = True
            real_condition.__exit__(exc_type, exc_value, traceback)
            if interrupt:
                raise KeyboardInterrupt("injected initialization-claim interruption")
            return False

    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_CONDITION", InterruptAfterClaim())
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 0)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_INITIALIZING", False)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_initialize",
        lambda: initialize_calls.append(True) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_shutdown",
        lambda: shutdown_calls.append(True) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )

    with pytest.raises(KeyboardInterrupt, match="initialization-claim interruption"):
        _api._acquire_process_lifecycle()

    assert not initialize_calls
    assert not shutdown_calls
    assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0
    assert not _api._PROCESS_LIFECYCLE_INITIALIZING
    assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN

    _api._acquire_process_lifecycle()
    _api._release_process_lifecycle()

    assert initialize_calls == [True]
    assert shutdown_calls == [True]


def test_post_publication_initialization_interruption_preserves_concurrent_token(monkeypatch):
    """An interrupted initializer must not overwrite a concurrent lifecycle token."""
    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    real_condition = _api._PROCESS_LIFECYCLE_CONDITION
    publication_released = threading.Event()
    concurrent_acquired = threading.Event()
    concurrent_errors = []
    shutdown_calls = []

    class InterruptAfterPublication:
        def __init__(self):
            self._raised = False

        def __enter__(self):
            real_condition.__enter__()
            return self

        def wait_for(self, predicate, timeout=None):
            return real_condition.wait_for(predicate, timeout)

        def notify_all(self):
            real_condition.notify_all()

        def __exit__(self, exc_type, exc_value, traceback):
            interrupt = (
                not self._raised
                and _api._PROCESS_LIFECYCLE_REFCOUNT == 1
                and not _api._PROCESS_LIFECYCLE_INITIALIZING
            )
            if interrupt:
                self._raised = True
            real_condition.__exit__(exc_type, exc_value, traceback)
            if interrupt:
                publication_released.set()
                if not concurrent_acquired.wait(timeout=5):
                    raise AssertionError("Concurrent lifecycle acquisition did not finish")
                raise KeyboardInterrupt("injected post-publication interruption")
            return False

    def acquire_concurrently():
        if not publication_released.wait(timeout=5):
            concurrent_errors.append(AssertionError("Lifecycle publication was not released"))
            concurrent_acquired.set()
            return
        try:
            _api._acquire_process_lifecycle()
        except BaseException as error:
            concurrent_errors.append(error)
        finally:
            concurrent_acquired.set()

    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_CONDITION", InterruptAfterPublication())
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 0)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_INITIALIZING", False)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(_api, "_python_log_callback", None)
    monkeypatch.setattr(_api, "_python_log_logger_name", None)
    monkeypatch.setattr(_api, "_python_log_retained_callbacks", ())
    monkeypatch.setattr(_api, "_python_log_callback_transition", False)
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_initialize",
        lambda: SimpleNamespace(status=ApiStatus.SUCCESS),
    )
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_shutdown",
        lambda: shutdown_calls.append(True) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )

    worker = threading.Thread(target=acquire_concurrently)
    worker.start()
    try:
        with pytest.raises(KeyboardInterrupt, match="post-publication interruption"):
            _api._acquire_process_lifecycle()
    finally:
        publication_released.set()
        worker.join(timeout=5)

    try:
        assert not worker.is_alive()
        assert not concurrent_errors
        assert concurrent_acquired.is_set()
        assert _api._PROCESS_LIFECYCLE_REFCOUNT == 1
        assert not _api._PROCESS_LIFECYCLE_INITIALIZING
        assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN
        assert not shutdown_calls
    finally:
        if _api._PROCESS_LIFECYCLE_REFCOUNT:
            _api._release_process_lifecycle()

    assert shutdown_calls == [True]
    assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0


def test_known_initialization_failure_releases_transition(monkeypatch):
    """A native initialize error must not leave the process transition wedged."""
    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 0)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_INITIALIZING", False)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_initialize",
        lambda: SimpleNamespace(status=ApiStatus.ERROR),
    )
    monkeypatch.setattr(_api, "_get_last_error_from_lib", lambda: "injected initialize failure")

    with pytest.raises(RuntimeError, match="injected initialize failure"):
        _api._acquire_process_lifecycle()

    assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0
    assert not _api._PROCESS_LIFECYCLE_INITIALIZING
    assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN


def test_known_initialization_failure_remains_exclusive_through_error_retrieval(monkeypatch):
    """A failing initializer must retain transition ownership while its error is read."""
    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    error_retrieval_started = threading.Event()
    release_error_retrieval = threading.Event()
    concurrent_started = threading.Event()
    concurrent_done = threading.Event()
    failing_errors = []
    concurrent_errors = []
    initialize_calls = []
    shutdown_calls = []

    def initialize():
        initialize_calls.append(threading.get_ident())
        status = ApiStatus.ERROR if len(initialize_calls) == 1 else ApiStatus.SUCCESS
        return SimpleNamespace(status=status)

    def get_error():
        error_retrieval_started.set()
        if not release_error_retrieval.wait(timeout=5):
            raise AssertionError("Error retrieval was not released")
        return "injected initialize failure"

    def acquire_concurrently():
        if not error_retrieval_started.wait(timeout=5):
            concurrent_errors.append(AssertionError("Error retrieval did not start"))
            concurrent_done.set()
            return
        concurrent_started.set()
        try:
            _api._acquire_process_lifecycle()
        except BaseException as error:
            concurrent_errors.append(error)
        finally:
            concurrent_done.set()

    def acquire_failing():
        try:
            _api._acquire_process_lifecycle()
        except BaseException as error:
            failing_errors.append(error)

    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 0)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_INITIALIZING", False)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(_api._lib, "ovphysx_initialize", initialize)
    monkeypatch.setattr(_api, "_get_last_error_from_lib", get_error)
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_shutdown",
        lambda: shutdown_calls.append(True) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )

    failing_worker = threading.Thread(target=acquire_failing)
    concurrent_worker = threading.Thread(target=acquire_concurrently)
    failing_worker.start()
    try:
        assert error_retrieval_started.wait(timeout=5)
        concurrent_worker.start()
        assert concurrent_started.wait(timeout=5)
        assert not concurrent_done.wait(timeout=0.1)
    finally:
        release_error_retrieval.set()
        failing_worker.join(timeout=5)
        concurrent_worker.join(timeout=5)

    try:
        assert not failing_worker.is_alive()
        assert not concurrent_worker.is_alive()
        assert concurrent_done.is_set()
        assert len(failing_errors) == 1
        assert isinstance(failing_errors[0], RuntimeError)
        assert "injected initialize failure" in str(failing_errors[0])
        assert not concurrent_errors
        assert len(initialize_calls) == 2
        assert _api._PROCESS_LIFECYCLE_REFCOUNT == 1
        assert not _api._PROCESS_LIFECYCLE_INITIALIZING
        assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN
        assert not shutdown_calls
    finally:
        if _api._PROCESS_LIFECYCLE_REFCOUNT:
            _api._release_process_lifecycle()


def test_enable_python_logging_base_exception_releases_transition(monkeypatch):
    """An ambiguous setter interruption retains its callback and releases transition ownership."""
    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    from ovphysx import disable_python_logging, enable_python_logging

    def interrupted_setter(*args):
        raise KeyboardInterrupt("injected setter interruption")

    monkeypatch.setattr(_api._lib, "ovphysx_set_log_callback", interrupted_setter)

    with pytest.raises(KeyboardInterrupt, match="injected setter interruption"):
        enable_python_logging("ovphysx.interrupted-enable")

    assert not _api._python_log_callback_transition
    assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN
    assert _api._python_log_callback is None
    assert _api._python_log_logger_name is None
    assert len(_api._python_log_retained_callbacks) == 1

    # A later known-successful disable proves the native slot is drained and
    # releases every callback retained from the ambiguous call.
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_set_log_callback",
        lambda *args: SimpleNamespace(status=ApiStatus.SUCCESS),
    )
    disable_python_logging()
    assert not _api._python_log_retained_callbacks


def test_enable_python_logging_post_commit_interruption_retains_callback(monkeypatch):
    """Interruption after native publication must not drop the CFUNCTYPE owner."""
    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    from ovphysx import disable_python_logging, enable_python_logging

    published_callbacks = []

    class InterruptingResult:
        @property
        def status(self):
            raise KeyboardInterrupt("injected post-commit interruption")

    def publish_then_interrupt(min_severity, channel_filter, callback, user_data):
        published_callbacks.append(callback)
        return InterruptingResult()

    monkeypatch.setattr(_api, "_python_log_callback", None)
    monkeypatch.setattr(_api, "_python_log_logger_name", None)
    monkeypatch.setattr(_api, "_python_log_retained_callbacks", ())
    monkeypatch.setattr(_api, "_python_log_callback_transition", False)
    monkeypatch.setattr(_api._lib, "ovphysx_set_log_callback", publish_then_interrupt)

    with pytest.raises(KeyboardInterrupt, match="post-commit interruption"):
        enable_python_logging("ovphysx.interrupted-after-publish")

    assert len(published_callbacks) == 1
    assert _api._python_log_callback is None
    assert len(_api._python_log_retained_callbacks) == 1
    assert not _api._python_log_callback_transition

    monkeypatch.setattr(
        _api._lib,
        "ovphysx_set_log_callback",
        lambda *args: SimpleNamespace(status=ApiStatus.SUCCESS),
    )
    disable_python_logging()
    assert not _api._python_log_retained_callbacks


def test_enable_python_logging_post_claim_interruption_releases_transition(monkeypatch):
    """Interruption immediately after claiming transition ownership must not wedge it."""
    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    from ovphysx import disable_python_logging, enable_python_logging

    real_condition = _api._python_log_callback_condition
    setter_calls = []

    monkeypatch.setattr(_api, "_python_log_callback", None)
    monkeypatch.setattr(_api, "_python_log_logger_name", None)
    monkeypatch.setattr(_api, "_python_log_retained_callbacks", ())
    monkeypatch.setattr(_api, "_python_log_callback_transition", False)
    monkeypatch.setattr(
        _api,
        "_python_log_callback_condition",
        _InterruptAfterLoggingTransitionClaim(
            real_condition,
            _api,
            "injected post-claim interruption",
        ),
    )
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_set_log_callback",
        lambda *args: setter_calls.append(args) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )

    with pytest.raises(KeyboardInterrupt, match="post-claim interruption"):
        enable_python_logging("ovphysx.interrupted-after-claim")

    assert not setter_calls
    assert not _api._python_log_callback_transition
    assert len(_api._python_log_retained_callbacks) == 1

    monkeypatch.setattr(_api, "_python_log_callback_condition", real_condition)
    disable_python_logging()
    assert len(setter_calls) == 1
    assert not _api._python_log_retained_callbacks


def test_process_shutdown_defers_condition_interruption_until_released(monkeypatch):
    """An interrupted bridge wait must still release both lifecycle tokens."""
    import ctypes

    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    from ovphysx import PhysX

    wait_interrupted = threading.Event()
    shutdown_entered = threading.Event()
    destroy_errors = []
    destroy_calls = []
    shutdown_calls = []
    original_wait = _api._python_log_callback_condition.wait
    interrupt_count = 0

    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 1)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(_api, "_python_log_callback", object())
    monkeypatch.setattr(_api, "_python_log_logger_name", "test-interrupted-shutdown")
    monkeypatch.setattr(_api, "_python_log_retained_callbacks", ())
    monkeypatch.setattr(_api, "_python_log_callback_transition", True)
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_destroy_instance",
        lambda handle: destroy_calls.append(handle) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )

    def interrupt_wait_once(timeout=None):
        nonlocal interrupt_count
        interrupt_count += 1
        if interrupt_count == 1:
            wait_interrupted.set()
            raise KeyboardInterrupt("injected condition interruption")
        return original_wait(timeout)

    def observing_shutdown():
        shutdown_calls.append(True)
        shutdown_entered.set()
        return SimpleNamespace(status=ApiStatus.SUCCESS)

    monkeypatch.setattr(_api._python_log_callback_condition, "wait", interrupt_wait_once)
    monkeypatch.setattr(_api._lib, "ovphysx_shutdown", observing_shutdown)

    physx = PhysX.__new__(PhysX)
    physx._lib = _api._lib
    physx._omni_physx_sdk_handle = ctypes.c_uint64(23)
    physx._attached_ovstage = None
    physx._lifecycle_acquired = True
    physx._released = False

    def destroy_instance():
        try:
            physx.destroy()
        except BaseException as error:
            destroy_errors.append(error)

    worker = threading.Thread(target=destroy_instance)
    worker.start()
    try:
        assert wait_interrupted.wait(timeout=5), "Shutdown did not enter the interrupted condition wait"
        assert not shutdown_entered.is_set(), "Native shutdown entered before bridge ownership was available"
    finally:
        with _api._python_log_callback_condition:
            _api._python_log_callback_transition = False
            _api._python_log_callback_condition.notify_all()
        worker.join(timeout=5)

    assert not worker.is_alive(), "Interrupted process shutdown did not finish"
    assert len(destroy_errors) == 1
    assert isinstance(destroy_errors[0], KeyboardInterrupt)
    assert "injected condition interruption" in str(destroy_errors[0])
    assert shutdown_entered.is_set()
    assert destroy_calls == [23]
    assert shutdown_calls == [True]
    assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0
    assert physx._omni_physx_sdk_handle is None
    assert not physx._lifecycle_acquired
    assert physx._released
    assert not _api._python_log_retained_callbacks
    assert not _api._python_log_callback_transition
    assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN

    # The deferred interruption is reported only after native shutdown commits.
    # A retry is an idempotent no-op and cannot strand a phantom lifecycle token.
    physx.destroy()
    assert destroy_calls == [23]
    assert shutdown_calls == [True]
    assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0


def test_process_shutdown_defers_post_claim_interruption(monkeypatch):
    """A post-claim interruption must not strand the transition or lifecycle token."""
    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    real_condition = _api._python_log_callback_condition
    shutdown_calls = []

    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 1)
    monkeypatch.setattr(_api, "_python_log_callback", object())
    monkeypatch.setattr(_api, "_python_log_logger_name", "test-post-claim-shutdown")
    monkeypatch.setattr(_api, "_python_log_retained_callbacks", (object(),))
    monkeypatch.setattr(_api, "_python_log_callback_transition", False)
    monkeypatch.setattr(
        _api,
        "_python_log_callback_condition",
        _InterruptAfterLoggingTransitionClaim(
            real_condition,
            _api,
            "injected shutdown post-claim interruption",
        ),
    )
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_shutdown",
        lambda: shutdown_calls.append(True) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )

    with pytest.raises(KeyboardInterrupt, match="shutdown post-claim interruption"):
        _api._release_process_lifecycle()

    assert shutdown_calls == [True]
    assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0
    assert _api._python_log_callback is None
    assert _api._python_log_logger_name is None
    assert not _api._python_log_retained_callbacks
    assert not _api._python_log_callback_transition


def test_process_shutdown_handoff_exit_interruption_finishes_shutdown(monkeypatch):
    """A process-condition exit interruption cannot skip final native shutdown."""
    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    real_condition = _api._PROCESS_LIFECYCLE_CONDITION
    initialize_calls = []
    shutdown_calls = []

    class InterruptAfterFinalHandoff:
        def __init__(self):
            self._raised = False

        def __enter__(self):
            real_condition.__enter__()
            return self

        def __exit__(self, exc_type, exc_value, traceback):
            interrupt = (
                not self._raised
                and _api._PROCESS_LIFECYCLE_REFCOUNT == 0
                and _api._PROCESS_LIFECYCLE_SHUTTING_DOWN
            )
            if interrupt:
                self._raised = True
            suppressed = real_condition.__exit__(exc_type, exc_value, traceback)
            if interrupt:
                raise KeyboardInterrupt("injected final-shutdown handoff interruption")
            return suppressed

        def __getattr__(self, name):
            return getattr(real_condition, name)

    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_CONDITION", InterruptAfterFinalHandoff())
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 1)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_INITIALIZING", False)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(_api, "_python_log_callback", None)
    monkeypatch.setattr(_api, "_python_log_logger_name", None)
    monkeypatch.setattr(_api, "_python_log_retained_callbacks", ())
    monkeypatch.setattr(_api, "_python_log_callback_transition", False)
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_initialize",
        lambda: initialize_calls.append(True) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_shutdown",
        lambda: shutdown_calls.append(True) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )

    with pytest.raises(KeyboardInterrupt, match="final-shutdown handoff interruption"):
        _api._release_process_lifecycle()

    assert shutdown_calls == [True]
    assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0
    assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN

    _api._acquire_process_lifecycle()
    _api._release_process_lifecycle()

    assert initialize_calls == [True]
    assert shutdown_calls == [True, True]


def test_process_shutdown_clear_interruption_releases_transition(monkeypatch):
    """An interruption before the final state clear cannot strand SHUTTING_DOWN."""
    import ovphysx.api as _api
    from ovphysx.types import ApiStatus

    real_condition = _api._PROCESS_LIFECYCLE_CONDITION
    initialize_calls = []
    shutdown_calls = []
    shutdown_returned = False

    class InterruptBeforeFinalClear:
        def __init__(self):
            self._raised = False

        def __enter__(self):
            if shutdown_returned and not self._raised:
                self._raised = True
                raise KeyboardInterrupt("injected final-shutdown clear interruption")
            real_condition.__enter__()
            return self

        def __exit__(self, exc_type, exc_value, traceback):
            return real_condition.__exit__(exc_type, exc_value, traceback)

        def __getattr__(self, name):
            return getattr(real_condition, name)

    def shutdown():
        nonlocal shutdown_returned
        shutdown_calls.append(True)
        shutdown_returned = True
        return SimpleNamespace(status=ApiStatus.SUCCESS)

    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_CONDITION", InterruptBeforeFinalClear())
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_REFCOUNT", 1)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_INITIALIZING", False)
    monkeypatch.setattr(_api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(_api, "_python_log_callback", None)
    monkeypatch.setattr(_api, "_python_log_logger_name", None)
    monkeypatch.setattr(_api, "_python_log_retained_callbacks", ())
    monkeypatch.setattr(_api, "_python_log_callback_transition", False)
    monkeypatch.setattr(
        _api._lib,
        "ovphysx_initialize",
        lambda: initialize_calls.append(True) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )
    monkeypatch.setattr(_api._lib, "ovphysx_shutdown", shutdown)

    with pytest.raises(KeyboardInterrupt, match="final-shutdown clear interruption"):
        _api._release_process_lifecycle()

    assert shutdown_calls == [True]
    assert _api._PROCESS_LIFECYCLE_REFCOUNT == 0
    assert not _api._PROCESS_LIFECYCLE_SHUTTING_DOWN

    _api._acquire_process_lifecycle()
    _api._release_process_lifecycle()

    assert initialize_calls == [True]
    assert shutdown_calls == [True, True]


def test_enable_python_logging_default_preserves_source_threshold(monkeypatch):
    """The Python bridge defaults to VERBOSE and adds no second severity gate."""
    import ovphysx.api as _api

    from ovphysx import disable_python_logging, enable_python_logging
    from ovphysx.types import LogLevel

    captured = {}
    original_setter = _api._lib.ovphysx_set_log_callback

    def capture_setter(min_severity, channel_filter, callback, user_data):
        captured["min_severity"] = min_severity
        return original_setter(min_severity, channel_filter, callback, user_data)

    monkeypatch.setattr(_api._lib, "ovphysx_set_log_callback", capture_setter)
    try:
        enable_python_logging()
        assert captured["min_severity"] == LogLevel.VERBOSE
    finally:
        monkeypatch.undo()
        disable_python_logging()


def test_enable_python_logging_replaces_previous():
    """Calling enable_python_logging twice should replace the previous bridge.

    The Python bridge is intentionally single-instance: Python's own logging
    hierarchy handles fan-out (multiple handlers on the same or parent loggers).
    Calling enable again with a different logger name should unregister the old
    callback and register a new one targeting the new logger.
    """
    import ovphysx.api as _api

    from ovphysx import disable_python_logging, enable_python_logging

    enable_python_logging("ovphysx.test1")
    assert _api._python_log_logger_name == "ovphysx.test1", "Bridge should target test1"
    assert _api._python_log_callback is not None

    # The second call replaces the bridge rather than adding a second one.
    enable_python_logging("ovphysx.test2")
    assert _api._python_log_logger_name == "ovphysx.test2", "Bridge should now target test2 (replaced test1)"
    assert _api._python_log_callback is not None

    disable_python_logging()


def test_python_logging_rejects_overlapping_transition(monkeypatch):
    """Only one Python bridge transition may own native callback state."""
    import ovphysx.api as _api

    from ovphysx import disable_python_logging, enable_python_logging
    from ovphysx.types import ApiStatus

    entered = threading.Event()
    release = threading.Event()
    worker_errors = []

    def blocking_setter(*args):
        entered.set()
        if not release.wait(timeout=5):
            return SimpleNamespace(status=ApiStatus.ERROR)
        return SimpleNamespace(status=ApiStatus.SUCCESS)

    def enable_first():
        try:
            enable_python_logging("ovphysx.first")
        except BaseException as error:
            worker_errors.append(error)

    monkeypatch.setattr(_api._lib, "ovphysx_set_log_callback", blocking_setter)
    worker = threading.Thread(target=enable_first)
    worker.start()
    try:
        assert entered.wait(timeout=5), "First logging transition did not reach the native setter"
        with pytest.raises(RuntimeError, match="transition is in progress"):
            disable_python_logging()
        with pytest.raises(RuntimeError, match="transition is in progress"):
            enable_python_logging("ovphysx.second")
    finally:
        release.set()
        worker.join(timeout=5)

    assert not worker.is_alive(), "First logging transition did not finish"
    assert not worker_errors
    assert _api._python_log_logger_name == "ovphysx.first"
    assert _api._python_log_callback is not None
    assert not _api._python_log_callback_transition

    monkeypatch.undo()
    disable_python_logging()


def test_python_logging_failed_enable_releases_transition(monkeypatch):
    """An ambiguous native error retains its candidate until a known drain."""
    import ovphysx.api as _api

    from ovphysx import disable_python_logging, enable_python_logging
    from ovphysx.types import ApiStatus

    monkeypatch.setattr(
        _api._lib,
        "ovphysx_set_log_callback",
        lambda *args: SimpleNamespace(status=ApiStatus.ERROR),
    )
    with pytest.raises(RuntimeError, match="Failed to enable Python logging"):
        enable_python_logging("ovphysx.failure")
    assert not _api._python_log_callback_transition
    assert _api._python_log_callback is None
    assert _api._python_log_logger_name is None
    assert len(_api._python_log_retained_callbacks) == 1
    ambiguous_owner = _api._python_log_retained_callbacks[0]

    monkeypatch.setattr(
        _api._lib,
        "ovphysx_set_log_callback",
        lambda *args: SimpleNamespace(status=ApiStatus.INVALID_ARGUMENT),
    )
    with pytest.raises(RuntimeError, match="Failed to enable Python logging"):
        enable_python_logging("ovphysx.invalid")
    assert _api._python_log_retained_callbacks == (ambiguous_owner,)
    assert not _api._python_log_callback_transition

    monkeypatch.setattr(
        _api._lib,
        "ovphysx_set_log_callback",
        lambda *args: SimpleNamespace(status=ApiStatus.SUCCESS),
    )
    disable_python_logging()
    assert not _api._python_log_retained_callbacks


def test_disable_python_logging_failure_preserves_callback(monkeypatch):
    """A failed native disable must retain the live CFUNCTYPE reference."""
    import ovphysx.api as _api

    from ovphysx import disable_python_logging, enable_python_logging
    from ovphysx.types import ApiStatus

    enable_python_logging("ovphysx.keepalive")
    previous_callback = _api._python_log_callback
    previous_name = _api._python_log_logger_name

    monkeypatch.setattr(
        _api._lib,
        "ovphysx_set_log_callback",
        lambda *args: SimpleNamespace(status=ApiStatus.ERROR),
    )
    try:
        disable_python_logging()
        assert _api._python_log_callback is previous_callback
        assert _api._python_log_logger_name == previous_name
    finally:
        monkeypatch.undo()
        disable_python_logging()


# ============================================================================
# End-to-end integration: native CARB_LOG_* -> Python logging
#
# This requires a PhysX instance so that Carbonite is initialized and the
# UserCallbackLogger is active.
# ============================================================================


def test_native_messages_flow_to_python_logging(physx_sdk_cpu, caplog):
    """Native CARB_LOG_* messages should arrive in Python logging via enable_python_logging.

    Full path:
      native CARB_LOG_* -> Carbonite ILogging -> UserCallbackLogger
        -> ctypes CFUNCTYPE -> Python logging.getLogger("ovphysx")
    """
    from ovphysx._bindings import _lib
    from ovphysx.types import LogLevel

    from ovphysx import (
        disable_python_logging,
        enable_python_logging,
        set_log_level,
    )

    original_level = _lib.ovphysx_get_log_level()

    try:
        set_log_level(LogLevel.VERBOSE)
        enable_python_logging()

        with caplog.at_level(logging.DEBUG, logger="ovphysx"):
            _lib.ovphysx_log_emit_test_messages()

        # Verify each expected test message arrived at the correct Python level
        error_msgs = [r for r in caplog.records if "[LogTest] ERROR" in r.message]
        warn_msgs = [r for r in caplog.records if "[LogTest] WARNING" in r.message]
        info_msgs = [r for r in caplog.records if "[LogTest] INFO" in r.message]
        verbose_msgs = [r for r in caplog.records if "[LogTest] VERBOSE" in r.message]

        assert len(error_msgs) >= 1, "Expected ERROR message from native"
        assert error_msgs[0].levelno == logging.ERROR

        assert len(warn_msgs) >= 1, "Expected WARNING message from native"
        assert warn_msgs[0].levelno == logging.WARNING

        assert len(info_msgs) >= 1, "Expected INFO message from native"
        assert info_msgs[0].levelno == logging.INFO

        assert len(verbose_msgs) >= 1, "Expected VERBOSE message from native"
        assert verbose_msgs[0].levelno == logging.DEBUG
    finally:
        disable_python_logging()
        set_log_level(original_level)


def test_python_logging_channel_filter_marshalling_and_delivery(physx_sdk_cpu, caplog):
    """The Python channel-filter view must reach native filtering intact."""
    from ovphysx._bindings import _lib
    from ovphysx.types import LogLevel

    from ovphysx import (
        disable_python_logging,
        enable_python_logging,
        flush_log,
        set_log_level,
    )

    original_level = _lib.ovphysx_get_log_level()
    logger_name = "ovphysx.filtered"
    try:
        set_log_level(LogLevel.VERBOSE)
        enable_python_logging(
            logger_name,
            min_severity=LogLevel.VERBOSE,
            channel_filter="omni_physx_sdk=error",
        )
        caplog.clear()
        with caplog.at_level(logging.DEBUG, logger=logger_name):
            _lib.ovphysx_log_emit_test_messages()
            flush_log()

        test_records = [
            record
            for record in caplog.records
            if record.name == logger_name and "[LogTest]" in record.message
        ]
        assert len(test_records) == 1
        assert "[LogTest] ERROR" in test_records[0].message
        assert test_records[0].ovphysx_channel == "omni_physx_sdk"

        enable_python_logging(
            logger_name,
            min_severity=LogLevel.VERBOSE,
            channel_filter="omni_physx_sdk=none",
        )
        caplog.clear()
        with caplog.at_level(logging.DEBUG, logger=logger_name):
            _lib.ovphysx_log_emit_test_messages()
            flush_log()

        assert not any(
            record.name == logger_name and "[LogTest]" in record.message
            for record in caplog.records
        )
    finally:
        disable_python_logging()
        set_log_level(original_level)


def test_python_logging_invalid_filter_preserves_bridge_state(physx_sdk_cpu, caplog):
    """Native filter validation must not publish or retain the rejected candidate."""
    import ovphysx.api as _api

    from ovphysx._bindings import _lib

    from ovphysx import disable_python_logging, enable_python_logging, flush_log

    logger_name = "ovphysx.filter.keep"
    enable_python_logging(logger_name)
    previous_callback = _api._python_log_callback
    previous_name = _api._python_log_logger_name
    previous_retained = _api._python_log_retained_callbacks
    try:
        with pytest.raises(RuntimeError, match="Failed to enable Python logging"):
            enable_python_logging(
                "ovphysx.filter.reject",
                channel_filter="missing_level",
            )

        assert _api._python_log_callback is previous_callback
        assert _api._python_log_logger_name == previous_name
        assert _api._python_log_retained_callbacks == previous_retained
        assert not _api._python_log_callback_transition

        caplog.clear()
        with caplog.at_level(logging.DEBUG, logger=logger_name):
            _lib.ovphysx_log_emit_test_messages()
            flush_log()
        assert any(
            record.name == logger_name and "[LogTest]" in record.message
            for record in caplog.records
        ), "The previously active native bridge must remain installed"
    finally:
        disable_python_logging()
