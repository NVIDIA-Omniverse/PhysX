# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Boundary condition tests for step/step_sync and wait_task semantics.

Reuses the stepping regressions with the CPU fixture and adds wait boundaries.
"""

import os
import subprocess
import sys
import textwrap

import pytest
from test_simulation import test_step_large_dt_advances_full_time, test_step_variants
from test_utils import load_usd_with_ovstage
from ovphysx import ApiStatus, Task

_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def data_path(filename):
    return os.path.join(_TEST_DIR, "data", filename)


def _load_basic(sdk):
    load_usd_with_ovstage(sdk, data_path("basic_simulation.usda"))
    sdk.wait_all()


# ---------------------------------------------------------------------------
# Non-finite dt: NaN rejection (negative / inf dt are covered in test_simulation.py)
# ---------------------------------------------------------------------------


def test_step_sync_nan_dt_raises(physx_sdk):
    """A NaN dt must be rejected rather than advancing the internal counter to
    a non-finite value."""
    _load_basic(physx_sdk)
    with pytest.raises(RuntimeError):
        physx_sdk.step_sync(float("nan"))


# ---------------------------------------------------------------------------
# Interleaved step / step_sync
# ---------------------------------------------------------------------------


def test_step_and_step_sync_interleaved(physx_sdk):
    """Mixing async step + wait_task with step_sync on the same instance must work."""
    _load_basic(physx_sdk)
    dt = 1.0 / 60.0

    op = physx_sdk.step(dt)
    physx_sdk.wait_task(op)

    physx_sdk.step_sync(dt)

    op2 = physx_sdk.step(dt)
    physx_sdk.wait_task(op2)


# ---------------------------------------------------------------------------
# wait_task single-use semantics
# ---------------------------------------------------------------------------


def test_wait_task_rejects_consumed_op(physx_sdk):
    """Waiting on an already-consumed op_index must raise."""
    _load_basic(physx_sdk)
    op = physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_task(op)
    with pytest.raises(RuntimeError, match="already collected"):
        physx_sdk.wait_task(op)


def test_wait_task_collection_moves_between_serialized_threads():
    """Serialized waits may move between threads and still collect a task once."""
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
            instance_lock = threading.Lock()

            def wait():
                start.wait()
                with instance_lock:
                    try:
                        sdk.wait_task(op)
                        outcomes.append("success")
                    except RuntimeError as exc:
                        outcomes.append(str(exc))

            threads = [threading.Thread(target=wait, daemon=True) for _ in range(2)]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join(timeout=10.0)

            if any(thread.is_alive() for thread in threads):
                print("serialized wait_task call did not return", file=sys.stderr, flush=True)
                os._exit(2)
            assert outcomes.count("success") == 1, outcomes
            errors = [outcome for outcome in outcomes if outcome != "success"]
            assert len(errors) == 1, outcomes
            assert "already collected" in errors[0], outcomes
        finally:
            destroy_ovstage_test_attachments(sdk)
            sdk.destroy()
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


def test_wait_task_rejects_consumed_op_when_polling(physx_sdk):
    """Polling a consumed op with timeout=0 raises rather than timing out."""
    _load_basic(physx_sdk)

    op = physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_task(op, timeout_ns=None)

    # Consumed op: polling cannot reuse the index
    with pytest.raises(RuntimeError, match="already collected"):
        physx_sdk.wait_task(op, timeout_ns=0)


# ---------------------------------------------------------------------------
# Return types
# ---------------------------------------------------------------------------


def test_step_returns_task(physx_sdk):
    """step() returns an immutable receipt owned by its submitting instance."""
    _load_basic(physx_sdk)
    op = physx_sdk.step(1.0 / 60.0)
    assert isinstance(op, Task)
    assert op.status == ApiStatus.SUCCESS
    assert op.op_index > 0
    assert op.instance == physx_sdk.handle
    physx_sdk.wait_task(op)


def test_step_sync_returns_none(physx_sdk):
    """step_sync() must return None."""
    _load_basic(physx_sdk)
    result = physx_sdk.step_sync(1.0 / 60.0)
    assert result is None
