# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# PARTIALLY DEPRECATED (tensor-binding-deprecation): the TensorBinding-lifecycle tests here retire with the binding. The attach-handle and ContactBinding tests stay.


# Lifecycle tests that use the shared session-scoped PhysX instance.
# Tests that require create/destroy cycles (e.g. use-after-destroy,
# repeated destruction and finalizer cleanup) live in lifecycle_tests/ and
# run in a dedicated subprocess to avoid the Carbonite re-init limitation.

import gc
from ctypes import byref, c_int32

import pytest
from ovphysx._bindings import ovphysx_tensor_spec_t
from ovphysx.types import ApiStatus, TensorType
from test_utils import destroy_ovstage_test_attachments, load_usd_with_ovstage


def test_get_attach_handle_reflects_current_attach(physx_sdk):
    """Test PhysX.get_attach_handle() against attach/detach/reattach (ADR-0016).

    Covered APIs:
        PhysX.get_attach_handle
        ovstage attach/detach helper

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Ensures the attach handle is nonzero while attached, zero once
        detached, and never repeats across a detach/reattach pair on the same
        instance.
    """
    import os

    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "basic_simulation.usda")

    assert physx_sdk.get_attach_handle() == 0

    load_usd_with_ovstage(physx_sdk, usd_path)
    physx_sdk.wait_all()
    first_handle = physx_sdk.get_attach_handle()
    assert first_handle != 0

    destroy_ovstage_test_attachments(physx_sdk)
    assert physx_sdk.get_attach_handle() == 0

    load_usd_with_ovstage(physx_sdk, usd_path)
    physx_sdk.wait_all()
    second_handle = physx_sdk.get_attach_handle()
    assert second_handle != 0
    assert second_handle != first_handle


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
    # Binding creation is synchronous, so no wait() is needed.

    binding.destroy()

    # The binding must be invalidated after destroy().
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
    # Binding creation is synchronous, so no wait() is needed.
    binding.destroy()
    binding.destroy()  # A second destroy() must be a no-op.


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
            raise ValueError("Intentional test error")
    except ValueError:
        pass  # Expected. The binding must still be destroyed.


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
