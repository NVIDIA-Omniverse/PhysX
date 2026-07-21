# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Tests for Python-side logging configuration.

Verifies that the ovphysx Python library uses the standard logging module
correctly, that the global log level API works, and that enable_python_logging
routes native messages into Python's logging system.
"""

import logging

import pytest

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

    This ensures the library doesn't emit warnings about 'No handlers could be
    found for logger' when the application hasn't configured logging.
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
    default is WARNING (2) and get_log_level() returns the same value as
    Python's LogLevel.WARNING, the two sides agree on what WARNING means.
    """
    from ovphysx.types import LogLevel

    from ovphysx import get_log_level

    assert get_log_level() == LogLevel.WARNING, (
        f"Expected default level to be LogLevel.WARNING ({LogLevel.WARNING}), " f"got {get_log_level()}"
    )


def test_log_level_constants_semantic_filtering(physx_sdk_cpu):
    """Python log level constants should produce correct C-side filtering.

    For each level, set the threshold, emit test messages from the C side,
    and verify a custom callback only receives messages at or above that level.
    This proves the Python constants control C-side filtering correctly —
    not just that they are in valid range.
    """
    from ovphysx._bindings import _lib, ovphysx_log_fn
    from ovphysx.types import LogLevel

    from ovphysx import set_log_level

    # Collect messages received by a custom C-level callback
    received = []

    @ovphysx_log_fn
    def collector(level, message, user_data):
        try:
            text = message.decode("utf-8") if isinstance(message, bytes) else str(message)
        except Exception:
            text = str(message)
        # Only collect our known test messages
        if "[LogTest]" in text:
            received.append((level, text))

    result = _lib.ovphysx_register_log_callback(collector, None)
    assert result.status == 0, "Failed to register test callback"

    try:
        # Known test messages emitted by ovphysx_log_emit_test_messages():
        #   ERROR:   "[LogTest] ERROR test message"
        #   WARNING: "[LogTest] WARNING test message"
        #   INFO:    "[LogTest] INFO test message"
        #   VERBOSE: "[LogTest] VERBOSE test message"

        # Define expected filtering for each level
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

            actual_levels = {level for level, _ in received}
            assert actual_levels == expected_levels, (
                f"At threshold {threshold}: expected levels {expected_levels}, "
                f"got {actual_levels} from messages {received}"
            )
    finally:
        _lib.ovphysx_unregister_log_callback(collector, None)
        set_log_level(LogLevel.WARNING)


# ============================================================================
# Public API surface: logging functions should be importable from ovphysx
# ============================================================================


def test_logging_api_exported():
    """set_log_level, get_log_level, enable/disable_python_logging should be importable."""
    from ovphysx import (
        disable_python_logging,
        enable_python_logging,
        get_log_level,
        set_log_level,
    )

    assert callable(set_log_level)
    assert callable(get_log_level)
    assert callable(enable_python_logging)
    assert callable(disable_python_logging)


# ============================================================================
# Global log level: set/get round-trip and default value
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


def test_default_log_level_is_warning():
    """Default global log level should be LogLevel.WARNING."""
    from ovphysx.types import LogLevel

    from ovphysx import get_log_level

    assert get_log_level() == LogLevel.WARNING


# ============================================================================
# PhysX class: log_level parameter was removed in favour of global API
# ============================================================================


def test_physx_init_no_log_level_param():
    """PhysX.__init__ should NOT have a log_level parameter.

    Log level is now controlled globally via ovphysx.set_log_level(), not
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


def test_usd_version_check_logger_messages(caplog):
    """Messages from usd_version_check should use the ovphysx.usd_version_check logger."""
    with caplog.at_level(logging.DEBUG, logger="ovphysx.usd_version_check"):
        from ovphysx.usd_version_check import check_usd_compatibility

        try:
            check_usd_compatibility()
        except Exception:
            pass  # Compatibility errors are expected in some test environments

    for record in caplog.records:
        assert (
            record.name == "ovphysx.usd_version_check"
        ), f"Expected logger name 'ovphysx.usd_version_check', got '{record.name}'"


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

    # Enable: bridge should become active with the default logger name
    enable_python_logging()
    assert _api._python_log_callback is not None, "Bridge should be active after enable"
    assert _api._python_log_logger_name == "ovphysx"

    # Disable: bridge should become inactive again
    disable_python_logging()
    assert _api._python_log_callback is None, "Bridge should be inactive after disable"
    assert _api._python_log_logger_name is None

    # Double disable should be a safe no-op (no exception, state unchanged)
    disable_python_logging()
    assert _api._python_log_callback is None


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

    # Second call should replace, not duplicate
    enable_python_logging("ovphysx.test2")
    assert _api._python_log_logger_name == "ovphysx.test2", "Bridge should now target test2 (replaced test1)"
    assert _api._python_log_callback is not None

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
