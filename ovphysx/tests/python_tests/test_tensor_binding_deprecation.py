# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Compatibility smoke test for the deprecated tensor-binding API (REQ-CAPI-WRITE-001 AC-11).

The tensor-binding surface is deprecated in favor of ``PhysX.read`` / ``PhysX.write`` but
stays functional for the deprecation period. This is the per-language compat coverage: it
asserts BOTH that the deprecated entry point still works AND that it emits its
``DeprecationWarning`` naming the successor. Removed with the API.
See ovphysx/plc/plans/PLAN-tensor-binding-deprecation.md.
"""

# @implements REQ-CAPI-WRITE-001
# @covers AC-11
# @maps_to TEST-CAPI-WRITE-001

import warnings
from types import SimpleNamespace

import numpy as np
import pytest
from ovphysx.api import TensorBinding
from ovphysx.types import ApiStatus, TensorType
from test_utils import data_path, load_usd_with_ovstage


def _binding_deprecations(caught):
    """The recorded warnings that are the tensor-binding DeprecationWarning."""
    return [
        w for w in caught
        if issubclass(w.category, DeprecationWarning) and "tensor bindings are deprecated" in str(w.message)
    ]


def test_create_tensor_binding_emits_deprecation_warning(physx_sdk):
    """The deprecated factory emits a DeprecationWarning pointing at the successor."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    with pytest.warns(DeprecationWarning, match="tensor bindings are deprecated"):
        binding = physx_sdk.create_tensor_binding(
            pattern="/World/Cube[1-5]", tensor_type=TensorType.RIGID_BODY_POSE
        )
    binding.destroy()


def test_deprecated_binding_still_reads(physx_sdk):
    """The deprecated read path keeps working during the deprecation period."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.warmup()
    physx_sdk.wait_all()

    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        binding = physx_sdk.create_tensor_binding(
            pattern="/World/Cube[1-5]", tensor_type=TensorType.RIGID_BODY_POSE
        )

    poses = np.zeros(binding.shape, dtype=np.dtype(str(binding.dtype)))
    binding.read(poses)
    assert poses.shape == binding.shape
    assert poses.shape[0] > 0, "pattern /World/Cube[1-5] should match at least one body"
    # A real read fills every body's pose. A silent no-op would leave the allocated zeros.
    assert np.isfinite(poses).all()
    assert np.all(np.any(poses != 0.0, axis=1)), "read() left one or more poses untouched"
    binding.destroy()


def test_factory_and_constructor_each_warn_exactly_once(physx_sdk):
    """`_from_factory` buys exactly ONE DeprecationWarning, and nothing else asserts that.

    ``create_tensor_binding`` emits its own caller-precise warning and passes ``_from_factory=True``
    so the constructor does not warn again, giving exactly one warning. A direct ``TensorBinding(...)``
    (no ``_from_factory``) still warns, exactly once. ``pytest.warns`` passes for one OR two, so count
    explicitly: this guards a regression to double-warning on the factory and to no-warning on the
    constructor.
    """
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        binding = physx_sdk.create_tensor_binding(
            pattern="/World/Cube[1-5]", tensor_type=TensorType.RIGID_BODY_POSE
        )
    assert len(_binding_deprecations(caught)) == 1, "the factory must warn exactly once, not zero or twice"
    binding.destroy()

    # Direct construction bypasses the factory, so the constructor warns on its own, exactly once.
    # A mock sdk keeps this off the native path. The warning fires in __init__ before any native call.
    mock_sdk = SimpleNamespace(
        _omni_physx_sdk_handle=SimpleNamespace(value=1),
        _lib=SimpleNamespace(ovphysx_destroy_tensor_binding=lambda *args: SimpleNamespace(status=ApiStatus.SUCCESS)),
        _get_last_error=lambda: "",
    )
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        direct = TensorBinding(sdk=mock_sdk, handle=1, tensor_type=TensorType.RIGID_BODY_POSE, ndim=1, shape=(1,))
    assert len(_binding_deprecations(caught)) == 1, "a direct TensorBinding() must warn exactly once"
    direct.destroy()
