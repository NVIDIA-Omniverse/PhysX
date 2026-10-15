# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import os

import pytest
from test_utils import load_usd_with_ovstage

from ovphysx import ApiStatus, Task
from ovphysx.types import SimObjectType


# Parameterized cases for step boundary and nominal values.
@pytest.mark.parametrize("step_method", ["step", "step_sync"])
@pytest.mark.parametrize(
    "dt, should_error",
    [
        # Valid cases
        (0.0, False),  # Zero dt (boundary case)
        (1.0 / 120.0, False),  # Common high-frequency update (120Hz)
        (1.0 / 60.0, False),  # Common update rate (60Hz)
        (0.016, False),  # Explicit 60Hz value
        (0.033, False),  # ~30Hz
        (0.001, False),  # Small dt
        (1e-10, False),  # Very small positive dt
        (1.0, False),
        (1.001, False),
        (2.0, False),
        # Invalid cases: dt must be finite and non-negative.
        (-0.016, True),  # Negative dt
        (-1.0, True),  # Another negative dt
        (-0.001, True),  # Negative dt
        (float("nan"), True),
        (float("inf"), True),
        (float("-inf"), True),
    ],
)
def test_step_variants(physx_sdk, dt, should_error, step_method):
    """Both stepping methods require a finite, non-negative timestep."""
    step = getattr(physx_sdk, step_method)
    if should_error:
        # Argument validation rejects submission before issuing a task.
        with pytest.raises(RuntimeError, match="finite"):
            step(dt)
    else:
        # step() rejects stage-less handles (NVBugs 6433668), so a stage is attached first.
        test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        load_usd_with_ovstage(physx_sdk, os.path.join(test_dir, "data", "basic_simulation.usda"))
        physx_sdk.wait_all()
        step(dt)
        physx_sdk.wait_all()


@pytest.mark.parametrize("step_method", ["step", "step_sync"])
@pytest.mark.parametrize("dt", [1.001, 2.0])
def test_step_large_dt_advances_full_time(physx_sdk, tmp_path, step_method, dt):
    """Neither stepping method rejects or clamps a timestep above one second."""
    scene = tmp_path / "constant_velocity.usda"
    scene.write_text('''#usda 1.0
(
    metersPerUnit = 1
)
def PhysicsScene "Scene" (
    prepend apiSchemas = ["PhysxSceneAPI"]
)
{
    float physics:gravityMagnitude = 0
    bool physxScene:enableGPUDynamics = true
    uniform token physxScene:broadphaseType = "GPU"
}
def Cube "Body" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysxRigidBodyAPI"]
)
{
    double size = 1
    vector3f physics:velocity = (1, 0, 0)
    float physxRigidBody:linearDamping = 0
}
''', encoding="utf-8")
    load_usd_with_ovstage(physx_sdk, str(scene))
    physx_sdk.wait_all()

    getattr(physx_sdk, step_method)(dt)
    physx_sdk.wait_all()

    with physx_sdk.read_sync(SimObjectType.RIGID_BODY, ["position"]) as result:
        assert len(result.groups) == 1
        positions = result.groups[0].tensors[0].numpy()
        assert positions.shape == (1, 3)
        assert positions[0].tolist() == pytest.approx([dt, 0.0, 0.0], abs=1e-6)


def test_step_negative_dt_error(physx_sdk):
    """Validate step raises error for negative dt.

    According to API requirements: dt must be finite and non-negative.

    Covered APIs:
        PhysX.step (dt validation)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Ensures negative dt is rejected.

    Note:
        Negative dt is rejected immediately with the native validation cause.
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
    # step() rejects stage-less handles (NVBugs 6433668), so a stage is attached first.
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    load_usd_with_ovstage(physx_sdk, os.path.join(test_dir, "data", "basic_simulation.usda"))
    physx_sdk.wait_all()

    # A very small positive dt must be accepted.
    physx_sdk.step(1e-10)
    physx_sdk.wait_all()

    # A very large but finite dt must be either accepted or rejected cleanly.
    large_dt = 1e10
    large_dt_accepted = False
    large_dt_rejected = False

    try:
        physx_sdk.step(large_dt)
        physx_sdk.wait_all()
        large_dt_accepted = True
    except RuntimeError as e:
        large_dt_rejected = True
        error_msg = str(e).lower()
        assert len(error_msg) > 0, "Error message should not be empty for invalid dt"

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
    # step() rejects stage-less handles (NVBugs 6433668), so a stage is attached first.
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    load_usd_with_ovstage(physx_sdk, os.path.join(test_dir, "data", "basic_simulation.usda"))
    physx_sdk.wait_all()

    ops = []
    for i in range(10):
        op = physx_sdk.step(0.016)
        ops.append(op)

    for i in range(1, len(ops)):
        assert ops[i].op_index > ops[i - 1].op_index, "Op indices should be strictly increasing"

    physx_sdk.wait_all()


def test_wait_task_timeout_boundary(physx_sdk):
    """Check wait_task timeout boundary behavior.

    Covered APIs:
        PhysX.step, PhysX.wait_task

    Args:
        physx_sdk (PhysX): PhysX SDK fixture providing an initialized instance.

    Returns:
        None: Ensures timeout mechanism works and operations complete properly.
    """
    # step() rejects stage-less handles (NVBugs 6433668), so a stage is attached first.
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    load_usd_with_ovstage(physx_sdk, os.path.join(test_dir, "data", "basic_simulation.usda"))
    physx_sdk.wait_all()

    op = physx_sdk.step(1.0 / 60.0)

    # A zero-timeout poll either consumes a completed op or leaves it pending.
    try:
        physx_sdk.wait_task(op, timeout_ns=0)
    except TimeoutError:
        # The poll did not consume the index, so finish with a blocking wait.
        physx_sdk.wait_task(op)


def test_wait_all_operations(physx_sdk):
    """Validate the separate wait_all entrypoint.

    Covered APIs:
        PhysX.wait_all
        PhysX.step

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Ensures waiting for all operations works correctly.
    """
    # step() rejects stage-less handles (NVBugs 6433668), so a stage is attached first.
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    load_usd_with_ovstage(physx_sdk, os.path.join(test_dir, "data", "basic_simulation.usda"))
    physx_sdk.wait_all()

    physx_sdk.step(0.016)
    physx_sdk.step(0.016)

    physx_sdk.wait_all()

    # Completes without raising.


def test_wait_task_invalid_index(physx_sdk):
    """Validate wait_task behavior with invalid operation index.

    Covered APIs:
        PhysX.wait_task (error handling)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates invalid operation indices are rejected.

    Note:
        API requires a valid, not-yet-consumed op_index. Invalid indices
        should raise RuntimeError.
    """
    with pytest.raises(RuntimeError, match="never issued"):
        physx_sdk.wait_task(Task(ApiStatus.SUCCESS, 999999, physx_sdk.handle), timeout_ns=1000000)  # 1ms timeout


def test_step_sync_negative_dt_error(physx_sdk):
    """step_sync() rejects negative dt."""
    with pytest.raises(RuntimeError):
        physx_sdk.step_sync(-0.016)


def test_step_sync_valid_dt(physx_sdk):
    """step_sync() works with valid dt."""
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    load_usd_with_ovstage(physx_sdk, os.path.join(test_dir, "data", "basic_simulation.usda"))
    physx_sdk.wait_all()
    physx_sdk.step_sync(1 / 60)


def test_wait_task_timeout_variants(physx_sdk):
    """Validate wait_task with different timeout values.

    Covered APIs:
        PhysX.wait_task (timeout_ns parameter variants)
        PhysX.step

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates that different timeout values work as expected.
    """
    # step() rejects stage-less handles (NVBugs 6433668), so a stage is attached first.
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    load_usd_with_ovstage(physx_sdk, os.path.join(test_dir, "data", "basic_simulation.usda"))
    physx_sdk.wait_all()

    # Test 1: infinite wait using None.
    op = physx_sdk.step(0.016)
    physx_sdk.wait_task(op, timeout_ns=None)

    # Test 2: poll (timeout=0) on an already-consumed operation index.
    op2 = physx_sdk.step(0.016)
    physx_sdk.wait_all()

    # wait_all() consumes pending op indices, so waiting on op2 again must fail.
    with pytest.raises(RuntimeError, match="already collected"):
        physx_sdk.wait_task(op2, timeout_ns=0)

    # Test 3: very long timeout (effectively infinite).
    op3 = physx_sdk.step(0.016)
    physx_sdk.wait_task(op3, timeout_ns=60_000_000_000)  # 60 seconds. The op completes long before that.
