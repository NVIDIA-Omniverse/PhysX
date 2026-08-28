# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Pytest configuration for CPU-mode PhysX tests.

These tests run in a SEPARATE pytest invocation from the main (GPU) test suite.
They call PhysX.set_cpu_mode(True) before creating any instance, forcing the
process into CPU-only mode regardless of the USD stage settings.
See test_python_runtime.cmake for how the two invocations are orchestrated.
"""

import logging

import pytest


@pytest.fixture(scope="session")
def _cpu_session_instance():
    """Session-wide CPU PhysX instance (internal).

    Created once, reused across all CPU-mode tests. Carbonite/Python cannot
    be re-initialized after destroy, so one long-lived instance avoids the
    problematic destroy/recreate cycle.

    We intentionally do NOT call release() at session end — Carbonite
    shutdown can hang.  The OS reclaims all resources when the process exits.
    """
    from ovphysx import PhysX

    PhysX.set_cpu_mode(True)
    physx = PhysX()
    yield physx
    try:
        from test_utils import destroy_ovstage_test_attachments

        destroy_ovstage_test_attachments(physx)
    except Exception:
        pass
    try:
        physx.reset_stage()
        physx.wait_all()
    except RuntimeError:
        logging.getLogger(__name__).warning(
            "physx CPU session teardown: reset_stage() failed; process shutdown will reclaim remaining resources"
        )


@pytest.fixture(scope="function")
def physx_sdk(_cpu_session_instance):
    """Provide the shared CPU-mode PhysX instance for each test.

    Teardown is resilient to prevent cascading failures from error-path tests.
    """
    yield _cpu_session_instance
    try:
        _cpu_session_instance.wait_all()
    except RuntimeError:
        pass
    try:
        from test_utils import destroy_ovstage_test_attachments

        destroy_ovstage_test_attachments(_cpu_session_instance)
    except Exception:
        pass
    try:
        _cpu_session_instance.reset_stage()
        _cpu_session_instance.wait_all()
    except RuntimeError:
        logging.getLogger(__name__).warning(
            "physx_sdk teardown: reset_stage() failed — instance may be degraded for subsequent tests"
        )
    try:
        from test_utils import destroy_ovstage_test_attachments

        destroy_ovstage_test_attachments(_cpu_session_instance)
    except Exception:
        pass


@pytest.fixture(scope="function")
def physx_sdk_cpu(physx_sdk):
    """Backward-compatible alias for the CPU-mode fixture."""
    return physx_sdk
