# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import os

import pytest
from test_utils import load_usd_with_ovstage

from ovphysx import OP_INDEX_ALL


# Parameterized cases for step boundary and nominal values.
@pytest.mark.parametrize(
    "dt, should_error",
    [
        # Valid cases (should succeed)
        (0.0, False),  # Zero dt (boundary case)
        (1.0 / 120.0, False),  # Common high-frequency update (120Hz)
        (1.0 / 60.0, False),  # Common update rate (60Hz)
        (0.016, False),  # Explicit 60Hz value
        (0.033, False),  # ~30Hz
        (0.001, False),  # Small dt
        (1e-10, False),  # Very small positive dt
        # Invalid cases (should error): dt must be >= 0.0
        # Note: Product correctly validates but returns generic error messages
        (-0.016, True),  # Negative dt
        (-1.0, True),  # Another negative dt
        (-0.001, True),  # Negative dt
    ],
)
def test_step_variants(physx_sdk, dt, should_error):
    """Validate step behavior with both valid and invalid dt values.

    API Requirements (per actual product behavior):
        - dt must be >= 0.0 (enforced, will raise RuntimeError if violated)

    This test validates both:
    1. Valid dt values work correctly
    2. Invalid (negative) dt values raise RuntimeError

    Covered APIs:
        PhysX.step, PhysX.wait_all

    Args:
        physx_sdk (PhysX): PhysX SDK fixture providing an initialized instance.
        dt (float): Simulation delta time for the step call.
        should_error (bool): Whether this combination should raise an error.

    Returns:
        None: Ensures valid cases succeed and invalid cases raise exceptions.
    """
    if should_error:
        # Invalid case - should raise RuntimeError
        # Note: Error messages are generic "Failed to step: Step failed"
        # C++ layer validates and logs detailed errors to stderr
        with pytest.raises(RuntimeError):
            physx_sdk.step(dt)
    else:
        # Valid case - should succeed. step() requires an attached stage
        # (rejects stage-less handles -- see NVBugs 6433668 MR review).
        test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        load_usd_with_ovstage(physx_sdk, os.path.join(test_dir, "data", "basic_simulation.usda"))
        physx_sdk.wait_all()
        physx_sdk.step(dt)
        physx_sdk.wait_all()


def test_step_negative_dt_error(physx_sdk):
    """Validate step raises error for negative dt.

    According to API requirements: dt must be >= 0.0

    Covered APIs:
        PhysX.step (dt validation)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Ensures negative dt is rejected.

    Note:
        Product correctly validates negative dt and logs detailed error to stderr,
        but Python wrapper returns generic "Failed to step: Step failed" message.
    """
    with pytest.raises(RuntimeError):
        physx_sdk.step(-0.016)


def test_step_infinity_dt_error(physx_sdk):
    """Validate step raises error for infinity dt with informative message.

    Covered APIs:
        PhysX.step (dt boundary validation)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Ensures infinity dt is rejected with clear error message.
    """
    with pytest.raises((RuntimeError, ValueError, OverflowError)) as exc_info:
        physx_sdk.step(float("inf"))

    error_msg = str(exc_info.value).lower()
    # Error message should be informative
    assert len(error_msg) > 0, "Error message should not be empty"


def test_step_extreme_valid_values(physx_sdk):
    """Validate step with extreme dt values.

    Tests that very small dt works, and very large dt is handled gracefully.

    Covered APIs:
        PhysX.step (boundary value testing)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Ensures extreme values are handled appropriately.
    """
    # step() requires an attached stage (rejects stage-less handles -- see
    # NVBugs 6433668 MR review).
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    load_usd_with_ovstage(physx_sdk, os.path.join(test_dir, "data", "basic_simulation.usda"))
    physx_sdk.wait_all()

    # Very small positive dt (should work)
    physx_sdk.step(1e-10)
    physx_sdk.wait_all()

    # Very large but finite dt - SDK should either accept or reject cleanly
    large_dt = 1e10
    large_dt_accepted = False
    large_dt_rejected = False

    try:
        physx_sdk.step(large_dt)
        physx_sdk.wait_all()
        large_dt_accepted = True
    except RuntimeError as e:
        large_dt_rejected = True
        # Validate error message is informative
        error_msg = str(e).lower()
        assert len(error_msg) > 0, "Error message should not be empty for invalid dt"

    # One of the two outcomes must occur (either accepted or rejected cleanly)
    assert large_dt_accepted or large_dt_rejected, "Large dt must either be accepted or rejected with RuntimeError"


def test_multiple_step_calls_rapid(physx_sdk):
    """Test multiple step calls in rapid succession.

    Covered APIs:
        PhysX.step (multiple rapid calls)
        PhysX.wait_all

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Ensures rapid step calls are handled correctly.
    """
    # step() requires an attached stage (rejects stage-less handles -- see
    # NVBugs 6433668 MR review).
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    load_usd_with_ovstage(physx_sdk, os.path.join(test_dir, "data", "basic_simulation.usda"))
    physx_sdk.wait_all()

    # Rapidly queue multiple steps
    ops = []
    for i in range(10):
        op = physx_sdk.step(0.016)
        ops.append(op)

    # All ops should have increasing indices
    for i in range(1, len(ops)):
        assert ops[i] > ops[i - 1], "Op indices should be strictly increasing"

    physx_sdk.wait_all()


def test_wait_op_timeout_boundary(physx_sdk):
    """Check wait_op timeout boundary behavior.

    Covered APIs:
        PhysX.step, PhysX.wait_op

    Args:
        physx_sdk (PhysX): PhysX SDK fixture providing an initialized instance.

    Returns:
        None: Ensures timeout mechanism works and operations complete properly.
    """
    # step() requires an attached stage (rejects stage-less handles -- see
    # NVBugs 6433668 MR review).
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    load_usd_with_ovstage(physx_sdk, os.path.join(test_dir, "data", "basic_simulation.usda"))
    physx_sdk.wait_all()

    op = physx_sdk.step(1.0 / 60.0)

    # A zero-timeout poll either consumes a completed op or leaves it pending.
    try:
        physx_sdk.wait_op(op, timeout_ns=0)
    except TimeoutError:
        # The poll did not consume the index, so finish with a blocking wait.
        physx_sdk.wait_op(op)


def test_wait_op_with_all_operations(physx_sdk):
    """Validate wait_op with OP_INDEX_ALL constant.

    Covered APIs:
        PhysX.wait_op (with ALL operations index)
        PhysX.step

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Ensures waiting for all operations works correctly.
    """
    # step() requires an attached stage (rejects stage-less handles -- see
    # NVBugs 6433668 MR review).
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    load_usd_with_ovstage(physx_sdk, os.path.join(test_dir, "data", "basic_simulation.usda"))
    physx_sdk.wait_all()

    # Queue multiple operations
    physx_sdk.step(0.016)
    physx_sdk.step(0.016)

    # Wait for all operations
    physx_sdk.wait_op(OP_INDEX_ALL)

    # Should complete without errors


def test_wait_op_invalid_index(physx_sdk):
    """Validate wait_op behavior with invalid operation index.

    Covered APIs:
        PhysX.wait_op (error handling)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates invalid operation indices are rejected.

    Note:
        API requires a valid, not-yet-consumed op_index. Invalid indices
        should raise RuntimeError.
    """
    with pytest.raises(RuntimeError, match="op_index not found"):
        physx_sdk.wait_op(999999, timeout_ns=1000000)  # 1ms timeout


def test_step_sync_negative_dt_error(physx_sdk):
    """step_sync() rejects negative dt."""
    with pytest.raises(RuntimeError):
        physx_sdk.step_sync(-0.016)


def test_step_n_sync_negative_dt_error(physx_sdk):
    """step_n_sync() rejects negative dt."""
    with pytest.raises(RuntimeError):
        physx_sdk.step_n_sync(1, -0.016)


def test_step_sync_valid_dt(physx_sdk):
    """step_sync() works with valid dt."""
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    load_usd_with_ovstage(physx_sdk, os.path.join(test_dir, "data", "basic_simulation.usda"))
    physx_sdk.wait_all()
    physx_sdk.step_sync(1 / 60)


def test_step_n_sync_valid_dt(physx_sdk):
    """step_n_sync() works with valid n_steps and dt."""
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    load_usd_with_ovstage(physx_sdk, os.path.join(test_dir, "data", "basic_simulation.usda"))
    physx_sdk.wait_all()
    physx_sdk.step_n_sync(3, 1 / 60)


def test_wait_op_timeout_variants(physx_sdk):
    """Validate wait_op with different timeout values.

    Covered APIs:
        PhysX.wait_op (timeout_ns parameter variants)
        PhysX.step

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates that different timeout values work as expected.
    """
    # step() requires an attached stage (rejects stage-less handles -- see
    # NVBugs 6433668 MR review).
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    load_usd_with_ovstage(physx_sdk, os.path.join(test_dir, "data", "basic_simulation.usda"))
    physx_sdk.wait_all()

    # Test 1: Infinite wait using None (should always succeed)
    op = physx_sdk.step(0.016)
    physx_sdk.wait_op(op, timeout_ns=None)  # Should not raise any exception

    # Test 2: Poll (timeout=0) on an already-consumed operation index
    op2 = physx_sdk.step(0.016)
    physx_sdk.wait_all()  # Ensure op2 is complete

    # wait_all() consumes pending op indices; waiting on op2 again should fail.
    with pytest.raises(RuntimeError, match="op_index not found"):
        physx_sdk.wait_op(op2, timeout_ns=0)

    # Test 3: Very long timeout (effectively infinite)
    op3 = physx_sdk.step(0.016)
    physx_sdk.wait_op(op3, timeout_ns=60_000_000_000)  # 60 seconds - should complete quickly
