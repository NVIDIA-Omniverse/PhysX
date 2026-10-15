# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# TensorBinding tests retire with that API; attachment and ContactBinding tests remain.


# Lifecycle tests that use the shared session-scoped PhysX instance.
# Tests that require create/destroy cycles (e.g. use-after-destroy,
# repeated destruction and finalizer cleanup) live in lifecycle_tests/ and
# run in a dedicated subprocess to avoid the Carbonite re-init limitation.

import gc
from ctypes import byref, c_int32

import pytest
from ovphysx import Task
from ovphysx._bindings import ovphysx_tensor_spec_t
from ovphysx.types import ApiStatus, TensorType
from test_utils import destroy_ovstage_test_attachments, load_usd_with_ovstage


def test_native_same_stage_reattach_invalidates_python_cache(physx_sdk):
    # @implements REQ-PYTHON-FRAME-001
    # @covers AC-1
    import ctypes
    from test_utils import attach_usd_with_ovstage, data_path
    from ovphysx.utils.simulation import OvStageOutputCache

    assert not physx_sdk.is_stage_attached()
    stage = attach_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    cache = OvStageOutputCache(physx_sdk)
    try:
        assert physx_sdk.is_stage_attached()
        native = physx_sdk._lib
        receipt = native.ovphysx_detach_ovstage(physx_sdk.handle)
        assert receipt.status == ApiStatus.SUCCESS
        physx_sdk.wait_task(Task._from_native(receipt))
        assert not physx_sdk.is_stage_attached()
        assert physx_sdk._attached_ovstage is stage  # Native caller bypassed Python.
        receipt = native.ovphysx_attach_ovstage(physx_sdk.handle, ctypes.cast(stage._inst, ctypes.c_void_p), 1)
        assert receipt.status == ApiStatus.SUCCESS
        physx_sdk.wait_task(Task._from_native(receipt))
        assert physx_sdk.is_stage_attached()
        with pytest.raises(RuntimeError, match="detach or reattach"):
            cache.refresh()
        cache.close()
        cache.close()
        with OvStageOutputCache(physx_sdk) as replacement:
            replacement.refresh()
        physx_sdk.wait_task(physx_sdk.detach_ovstage())
        physx_sdk.wait_task(physx_sdk.detach_ovstage())
        assert not physx_sdk.is_stage_attached()
    finally:
        cache.close()
        physx_sdk.wait_task(physx_sdk.detach_ovstage())
        destroy_ovstage_test_attachments(physx_sdk)


def test_native_different_stage_attach_rejects_stale_python_keepalive(physx_sdk):
    # @implements REQ-PYTHON-FRAME-001
    # @covers AC-1
    import ctypes
    import ovstage
    from test_utils import attach_usd_with_ovstage, data_path
    from ovphysx.utils.simulation import OvStageOutputCache

    original = attach_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    with ovstage.Stage("native-replacement") as replacement:
        try:
            native = physx_sdk._lib
            receipt = native.ovphysx_detach_ovstage(physx_sdk.handle)
            assert receipt.status == ApiStatus.SUCCESS
            physx_sdk.wait_task(Task._from_native(receipt))
            receipt = native.ovphysx_attach_ovstage(
                physx_sdk.handle, ctypes.cast(replacement._inst, ctypes.c_void_p), 1
            )
            assert receipt.status == ApiStatus.SUCCESS
            physx_sdk.wait_task(Task._from_native(receipt))
            assert physx_sdk._attached_ovstage is original
            assert physx_sdk.is_stage_attached(replacement)
            assert not physx_sdk.is_stage_attached(original)
            with pytest.raises(RuntimeError, match="current ovstage.Stage object"):
                OvStageOutputCache(physx_sdk)
        finally:
            physx_sdk.wait_task(physx_sdk.detach_ovstage())
            destroy_ovstage_test_attachments(physx_sdk)


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
