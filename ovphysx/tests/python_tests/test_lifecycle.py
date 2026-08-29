# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause


# Lifecycle tests that use the shared session-scoped PhysX instance.
# Tests that require create/release cycles (e.g. use-after-release,
# double-release, context manager exit) live in lifecycle_tests/ and
# run in a dedicated subprocess to avoid the Carbonite re-init limitation.

import gc
from ctypes import byref, c_int32

import pytest
from ovphysx._bindings import ovphysx_tensor_spec_t
from ovphysx.types import ApiStatus, TensorType
from test_utils import load_usd_with_ovstage


def test_operations_on_destroyed_binding(physx_sdk):
    """Test using binding after destroy() called.

    Covered APIs:
        ovstage attach/update helper
        PhysX.create_tensor_binding
        TensorBinding.destroy
        TensorBinding.write (on destroyed binding)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Ensures operations on destroyed bindings fail appropriately.
    """
    import os

    import numpy as np

    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "basic_simulation.usda")
    load_usd_with_ovstage(physx_sdk, usd_path)
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        prim_paths=["/World/envs/env0/table"],
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    # Synchronous - no wait() needed

    binding.destroy()

    # The binding should be invalidated after destroy()
    tensor = np.zeros((1, 7), dtype=np.float32)
    with pytest.raises(RuntimeError, match=r"(?i)(destroyed|invalid)"):
        binding.write(tensor)


def test_binding_double_destroy(physx_sdk):
    """Test calling destroy() twice on same binding.

    Covered APIs:
        ovstage attach/update helper
        PhysX.create_tensor_binding
        TensorBinding.destroy (idempotent behavior)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Ensures destroy() is idempotent.
    """
    import os

    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "basic_simulation.usda")
    load_usd_with_ovstage(physx_sdk, usd_path)
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        prim_paths=["/World/envs/env0/table"],
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    # Synchronous - no wait() needed
    binding.destroy()
    binding.destroy()  # Should be safe to call twice


def test_binding_context_manager_with_exception(physx_sdk):
    """Test TensorBinding context manager with exception handling.

    Covered APIs:
        ovstage attach/update helper
        PhysX.create_tensor_binding
        TensorBinding.__enter__
        TensorBinding.__exit__

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Ensures binding cleanup works even with exceptions.
    """
    import os

    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "basic_simulation.usda")
    load_usd_with_ovstage(physx_sdk, usd_path)
    physx_sdk.wait_all()

    try:
        with physx_sdk.create_tensor_binding(
            prim_paths=["/World/envs/env0/table"],
            tensor_type=TensorType.RIGID_BODY_POSE,
        ) as binding:
            assert binding.handle > 0
            assert binding.shape == (1, 7)  # One table, 7 components (pose)
            # Simulate error during usage
            raise ValueError("Intentional test error")
    except ValueError:
        pass  # Expected - binding should still be destroyed


def test_tensor_binding_resource_warning_without_explicit_destroy(physx_sdk):
    """Garbage-collected tensor bindings warn because native handles lived too long."""
    import os

    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "basic_simulation.usda")
    load_usd_with_ovstage(physx_sdk, usd_path)
    physx_sdk.wait_all()

    with pytest.warns(ResourceWarning, match="TensorBinding.*explicit destroy"):
        binding = physx_sdk.create_tensor_binding(
            prim_paths=["/World/envs/env0/table"],
            tensor_type=TensorType.RIGID_BODY_POSE,
        )
        handle = binding.handle
        del binding
        gc.collect()

    spec = ovphysx_tensor_spec_t()
    result = physx_sdk._lib.ovphysx_get_tensor_binding_spec(
        physx_sdk._omni_physx_sdk_handle.value,
        handle,
        byref(spec),
    )
    assert result.status == ApiStatus.NOT_FOUND


def test_contact_binding_resource_warning_without_explicit_destroy(physx_sdk):
    """Garbage-collected contact bindings warn because native handles lived too long."""
    import os

    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "boxes_falling_on_groundplane.usda")
    load_usd_with_ovstage(physx_sdk, usd_path)
    physx_sdk.wait_all()

    with pytest.warns(ResourceWarning, match="ContactBinding.*explicit destroy"):
        binding = physx_sdk.create_contact_binding(sensor_patterns=["/World/Cube1"])
        handle = binding._handle
        del binding
        gc.collect()

    sensor_count = c_int32()
    filter_count = c_int32()
    result = physx_sdk._lib.ovphysx_get_contact_binding_spec(
        physx_sdk._omni_physx_sdk_handle.value,
        handle,
        byref(sensor_count),
        byref(filter_count),
    )
    assert result.status == ApiStatus.NOT_FOUND
