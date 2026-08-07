# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import os

import numpy as np
import pytest
from ovphysx.types import LogLevel, TensorType
from test_utils import data_path

from ovphysx import (
    get_log_level,
    set_log_level,
)
from test_utils import load_usd_with_ovstage

# =============================================================================
# CORE API SURFACE TESTS
# =============================================================================


@pytest.mark.parametrize(
    "log_level",
    [
        LogLevel.NONE,
        LogLevel.ERROR,
        LogLevel.WARNING,
        LogLevel.INFO,
        LogLevel.VERBOSE,
    ],
)
def test_log_level_variants(physx_sdk, log_level):
    """Validate set_log_level with each valid level on a live instance.

    Covered APIs:
        ovphysx.set_log_level, ovphysx.get_log_level

    Args:
        physx_sdk: Shared PhysX SDK fixture
        log_level: Valid ovphysx log level constant.

    Returns:
        None: Ensures all valid log levels can be set and read back.
    """
    original = get_log_level()
    try:
        set_log_level(log_level)
        assert get_log_level() == log_level
    finally:
        set_log_level(original)


def test_invalid_log_level():
    """Validate that invalid log levels are rejected.

    Covered APIs:
        ovphysx.set_log_level

    Args:
        None

    Returns:
        None: Ensures invalid levels raise ValueError.
    """
    with pytest.raises(ValueError):
        set_log_level(-1)
    with pytest.raises(ValueError):
        set_log_level(999)


def test_multiple_bindings_stress(physx_sdk):
    """Validate creating and managing multiple tensor bindings simultaneously.

    Covered APIs:
        ovstage attach/update helper
        PhysX.create_tensor_binding (multiple)
        TensorBinding.destroy (multiple)
        PhysX.wait_all

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Stress test for multiple tensor bindings.
    """
    usd_path = data_path("boxes_falling_on_groundplane.usda")
    load_usd_with_ovstage(physx_sdk, usd_path)
    physx_sdk.wait_all()

    bindings = []

    # Create bindings for existing rigid bodies (Cube1-Cube9, plus more)
    # boxes_falling_on_groundplane.usda has Cube1-Cube16
    for i in range(1, 11):  # Bind to first 10 cubes
        binding = physx_sdk.create_tensor_binding(
            prim_paths=[f"/World/Cube{i}"],
            tensor_type=TensorType.RIGID_BODY_POSE,
        )
        bindings.append(binding)

    for binding in bindings:
        binding.destroy()


def test_integration_reset_with_bindings(physx_sdk):
    """Integration test: Reset operation with active tensor bindings.

    Covered APIs:
        ovstage attach/update helper
        PhysX.create_tensor_binding
        PhysX.reset (with active bindings)
        PhysX.wait_all
        TensorBinding.write (after reset - should fail)
        TensorBinding.read (after reset - should fail)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates that bindings become invalid after reset.
    """
    usd_path = data_path("boxes_falling_on_groundplane.usda")
    load_usd_with_ovstage(physx_sdk, usd_path)
    physx_sdk.wait_all()

    # Create tensor binding (synchronous - no wait needed)
    binding = physx_sdk.create_tensor_binding(
        prim_paths=["/World/Cube1"],
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )

    assert binding.count == 1, "Binding should have 1 prim before reset"
    assert binding.shape == (1, 6), "Binding shape should be (1, 6) before reset"

    # Reset stage (invalidates all bindings)
    physx_sdk.reset_stage()
    physx_sdk.wait_all()

    # Validate that binding operations fail after reset
    velocities = np.zeros((1, 6), dtype=np.float32)

    with pytest.raises(RuntimeError, match=r".*"):
        binding.write(velocities)

    with pytest.raises(RuntimeError, match=r".*"):
        binding.read(velocities)

    # Destroy should be safe to call (idempotent)
    binding.destroy()


def test_return_values_validity(physx_sdk):
    """Validate that return values are in expected ranges.

    Covered APIs:
        ovstage attach helper (return values)
        PhysX.step (return values)
        PhysX.reset (return values)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Ensures all return values are valid.
    """
    usd_path = os.path.join(os.path.dirname(__file__), "../data/api_surface_permutations.usda")

    # ovstage attach helper returns a synchronization op (no usd_handle anymore).
    op = load_usd_with_ovstage(physx_sdk, usd_path)
    assert op >= 0, "Op index should be non-negative"

    physx_sdk.wait_all()

    step_op = physx_sdk.step(0.016)
    assert step_op >= 0, "Step op should be non-negative"

    reset_op = physx_sdk.reset_stage()
    assert reset_op >= 0, "Reset op should be non-negative"

    physx_sdk.wait_all()


def test_error_messages_are_informative(physx_sdk):
    """Validate that error messages contain useful information.

    Covered APIs:
        ovstage population/attach (error handling)
        PhysX.wait_all (error reporting)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Ensures error messages are helpful for debugging.
    """
    try:
        load_usd_with_ovstage(physx_sdk, "/nonexistent/path/to/file.usda")
        physx_sdk.wait_all()
        pytest.fail("Should have raised RuntimeError")
    except RuntimeError as e:
        error_msg = str(e).lower()
        # Should mention the issue (file not found, path error, etc.)
        assert len(error_msg) > 0, "Error message should not be empty"
        has_useful_info = any(
            keyword in error_msg for keyword in ["file", "path", "not found", "nonexistent", "failed", "error"]
        )
        assert has_useful_info, f"Error message should be informative: {error_msg}"
