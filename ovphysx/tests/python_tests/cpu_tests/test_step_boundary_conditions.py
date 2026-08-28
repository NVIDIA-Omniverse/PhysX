# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Boundary condition tests for step/step_n_sync and wait_op semantics.

Complements test_simulation.py (which covers basic dt validation, rapid queuing,
and timeout polling) without duplicating those tests.
"""

import os
import subprocess
import sys
import textwrap

import pytest
from test_utils import load_usd_with_ovstage

_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def data_path(filename):
    return os.path.join(_TEST_DIR, "data", filename)


def _load_basic(sdk):
    load_usd_with_ovstage(sdk, data_path("basic_simulation.usda"))
    sdk.wait_all()


# ---------------------------------------------------------------------------
# step_n_sync boundary: n values
# ---------------------------------------------------------------------------


def test_step_n_sync_n_equals_one(physx_sdk):
    """step_n_sync(n=1, ...) is the minimum valid call."""
    _load_basic(physx_sdk)
    physx_sdk.step_n_sync(n=1, dt=1.0 / 60.0)


def test_step_n_sync_n_equals_zero_raises(physx_sdk):
    """step_n_sync(n=0, ...) must raise RuntimeError (invalid step count)."""
    _load_basic(physx_sdk)
    with pytest.raises(RuntimeError):
        physx_sdk.step_n_sync(n=0, dt=1.0 / 60.0)


def test_step_n_sync_negative_n_raises(physx_sdk):
    """step_n_sync(n=-1, ...) must raise RuntimeError."""
    _load_basic(physx_sdk)
    with pytest.raises(RuntimeError):
        physx_sdk.step_n_sync(n=-1, dt=1.0 / 60.0)


def test_step_n_sync_large_n_succeeds(physx_sdk):
    """step_n_sync(n=50, ...) must complete successfully."""
    _load_basic(physx_sdk)
    physx_sdk.step_n_sync(n=50, dt=1.0 / 240.0)


# ---------------------------------------------------------------------------
# Non-finite dt: NaN rejection (negative / inf dt are covered in test_simulation.py)
# ---------------------------------------------------------------------------


def test_step_sync_nan_dt_raises(physx_sdk):
    """A NaN dt must be rejected rather than advancing the internal counter to
    a non-finite value."""
    _load_basic(physx_sdk)
    with pytest.raises(RuntimeError):
        physx_sdk.step_sync(float("nan"))


def test_step_n_sync_nan_dt_raises(physx_sdk):
    """step_n_sync() must reject a NaN dt."""
    _load_basic(physx_sdk)
    with pytest.raises(RuntimeError):
        physx_sdk.step_n_sync(n=2, dt=float("nan"))


# ---------------------------------------------------------------------------
# Interleaved step / step_sync
# ---------------------------------------------------------------------------


def test_step_and_step_sync_interleaved(physx_sdk):
    """Mixing async step + wait_op with step_sync on the same instance must work."""
    _load_basic(physx_sdk)
    dt = 1.0 / 60.0

    op = physx_sdk.step(dt)
    physx_sdk.wait_op(op)

    physx_sdk.step_sync(dt)

    op2 = physx_sdk.step(dt)
    physx_sdk.wait_op(op2)


# ---------------------------------------------------------------------------
# wait_op single-use semantics
# ---------------------------------------------------------------------------


def test_wait_op_rejects_consumed_op(physx_sdk):
    """Waiting on an already-consumed op_index must raise."""
    _load_basic(physx_sdk)
    op = physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_op(op)
    with pytest.raises(RuntimeError, match="op_index not found"):
        physx_sdk.wait_op(op)


def test_wait_op_defensively_rejects_competing_simulation_waiter():
    """Pin defensive single-consumer behavior in a hang-contained subprocess."""
    script = textwrap.dedent(r"""
        import os
        import sys
        import threading

        from ovphysx import PhysX
        from test_utils import destroy_ovstage_test_attachments, load_usd_with_ovstage

        PhysX.set_cpu_mode(True)
        sdk = PhysX()
        try:
            load_usd_with_ovstage(sdk, sys.argv[1])
            sdk.wait_all()
            op = sdk.step(1.0 / 60.0)
            start = threading.Barrier(2, timeout=10.0)
            outcomes = []

            def wait():
                start.wait()
                try:
                    sdk.wait_op(op)
                    outcomes.append("success")
                except RuntimeError as exc:
                    outcomes.append(str(exc))

            threads = [threading.Thread(target=wait, daemon=True) for _ in range(2)]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join(timeout=10.0)

            if any(thread.is_alive() for thread in threads):
                print("competing wait_op call did not return", file=sys.stderr, flush=True)
                os._exit(2)
            assert outcomes.count("success") == 1, outcomes
            errors = [outcome for outcome in outcomes if outcome != "success"]
            assert len(errors) == 1, outcomes
            assert "op_index not found" in errors[0], outcomes
        finally:
            destroy_ovstage_test_attachments(sdk)
            sdk.release()
        """)

    result = subprocess.run(
        [sys.executable, "-c", script, data_path("basic_simulation.usda")],
        cwd=os.path.join(_TEST_DIR, "python_tests"),
        capture_output=True,
        text=True,
        timeout=30.0,
        check=False,
    )
    assert result.returncode == 0, f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"


def test_wait_all_when_no_pending_ops_succeeds(physx_sdk):
    """wait_all() when the queue is empty must not raise."""
    _load_basic(physx_sdk)
    physx_sdk.step_sync(1.0 / 60.0)
    # After step_sync, no async ops are pending
    physx_sdk.wait_all()  # must not raise or hang


def test_wait_op_rejects_consumed_op_when_polling(physx_sdk):
    """Polling a consumed op with timeout=0 raises rather than timing out."""
    _load_basic(physx_sdk)

    op = physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_op(op, timeout_ns=None)

    # Consumed op: polling cannot reuse the index
    with pytest.raises(RuntimeError, match="op_index not found"):
        physx_sdk.wait_op(op, timeout_ns=0)


# ---------------------------------------------------------------------------
# Return types
# ---------------------------------------------------------------------------


def test_step_returns_int_op_index(physx_sdk):
    """step() must return an integer op_index."""
    _load_basic(physx_sdk)
    op = physx_sdk.step(1.0 / 60.0)
    assert isinstance(op, int)
    assert op > 0
    physx_sdk.wait_op(op)


def test_step_sync_returns_none(physx_sdk):
    """step_sync() must return None."""
    _load_basic(physx_sdk)
    result = physx_sdk.step_sync(1.0 / 60.0)
    assert result is None


def test_step_n_sync_returns_none(physx_sdk):
    """step_n_sync() must return None."""
    _load_basic(physx_sdk)
    result = physx_sdk.step_n_sync(n=3, dt=1.0 / 60.0)
    assert result is None
