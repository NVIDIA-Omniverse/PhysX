# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# DEPRECATED (tensor-binding-deprecation): a deprecated tensor-binding test. Removed with the binding.

"""Fresh-process regression test for NVBug 6504951 opaque-handle aliasing.

Instance handles and per-instance resource handles must not both count from 1.
If they do, then for a valid ``ovphysx_handle_t instance`` and
``ovphysx_tensor_spec_t out_spec``,
``ovphysx_get_tensor_binding_spec(instance, instance, &out_spec)`` returns an
``ovphysx_result_t`` with ``.status == OVPHYSX_API_SUCCESS`` and populates
``out_spec`` from the wrongly resolved first tensor binding. Every
ovphysx-owned object handle comes from one process-wide never-reused
sequence, so that specific lookup with a valid instance misses the
tensor-binding map and its returned ``ovphysx_result_t`` has
``.status == OVPHYSX_API_NOT_FOUND``.

Only ONE PhysX create+destroy cycle is permitted per lifecycle test file.
"""

import ctypes

from ovphysx import PhysX, _bindings
from ovphysx.types import ApiStatus, TensorType
from test_utils import data_path, load_usd_with_ovstage

_TABLE = "/World/envs/env0/table"


def test_instance_handle_is_not_a_tensor_binding_handle():
    PhysX.set_cpu_mode(True)
    physx = PhysX()
    try:
        load_usd_with_ovstage(physx, data_path("basic_simulation.usda"))
        physx.wait_all()

        binding = physx.create_tensor_binding(
            prim_paths=[_TABLE], tensor_type=TensorType.RIGID_BODY_POSE
        )
        sdk_handle = physx.handle
        binding_handle = binding.handle
        assert sdk_handle != binding_handle

        spec = _bindings.ovphysx_tensor_spec_t()

        live = _bindings._lib.ovphysx_get_tensor_binding_spec(
            sdk_handle, binding_handle, ctypes.byref(spec)
        )
        assert live.status == ApiStatus.SUCCESS

        # The call from the bug report. It must report NOT_FOUND and never
        # INVALID_ARGUMENT: there is no handle-kind validation and no dedicated
        # error code, the two numbers simply never collide.
        reproducer = _bindings._lib.ovphysx_get_tensor_binding_spec(
            sdk_handle, sdk_handle, ctypes.byref(spec)
        )
        assert reproducer.status == ApiStatus.NOT_FOUND

        # Serials are never returned to the sequence, so a destroyed binding's
        # handle can never be handed out to its replacement.
        binding.destroy()
        stale = _bindings._lib.ovphysx_get_tensor_binding_spec(
            sdk_handle, binding_handle, ctypes.byref(spec)
        )
        assert stale.status == ApiStatus.NOT_FOUND

        replacement = physx.create_tensor_binding(
            prim_paths=[_TABLE], tensor_type=TensorType.RIGID_BODY_POSE
        )
        assert replacement.handle != binding_handle
        replacement.destroy()
    finally:
        physx.destroy()
