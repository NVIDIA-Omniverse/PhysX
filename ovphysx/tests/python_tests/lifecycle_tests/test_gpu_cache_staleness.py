# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# DEPRECATED (tensor-binding-deprecation): a deprecated tensor-binding test, removed with the binding.

"""GPU tensor cache staleness tests after SDK destroy. ONE create+destroy per file.

Verifies that the fast-path read/write cache correctly detects a destroyed SDK
handle and raises RuntimeError, even when the cache still holds a stale value.

Requires GPU. Runs in its own subprocess (see test_python_runtime.cmake).
"""

import os

import pytest
from ovphysx.types import TensorType

from ovphysx import PhysX, PhysXConfig
from test_utils import load_usd_with_ovstage

try:
    from test_utils import CudaArray
except ImportError:
    pytest.skip("CudaArray helper not available", allow_module_level=True)


def test_cache_staleness_after_sdk_destroy():
    """Warming read/write caches then destroying the SDK must raise on the next access.

    The fast path checks the live _omni_physx_sdk_handle (not the cached integer),
    so it must fail cleanly after PhysX.destroy() even though the cache still holds
    a stale handle value.

    Covers both read cache (RIGID_BODY_POSE) and write cache (RIGID_BODY_VELOCITY)
    in a single create+destroy cycle.
    """
    # Opt into DirectGPU since the test writes via a CudaArray (GPU-resident
    # write_binding). ovphysx does not auto-enable suppressReadback.
    physx = PhysX(
        config=PhysXConfig(
            carbonite_overrides={
                "/physics/suppressReadback": True,
            }
        ),
    )
    tests_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    usd_path = os.path.join(tests_dir, "data", "boxes_falling_on_groundplane.usda")
    load_usd_with_ovstage(physx, usd_path)
    physx.wait_all()
    physx.warmup()

    read_binding = physx.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    write_binding = physx.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )

    read_ga = CudaArray(read_binding.shape)
    write_ga = CudaArray(write_binding.shape)

    read_binding.read(read_ga.dltensor)
    assert read_binding._read_cache is not None

    write_binding.write(write_ga.dltensor)
    assert write_binding._write_cache is not None

    physx.destroy()

    with pytest.raises(RuntimeError, match="parent PhysX instance has been destroyed"):
        read_binding.read(read_ga.dltensor)

    with pytest.raises(RuntimeError, match="parent PhysX instance has been destroyed"):
        write_binding.write(write_ga.dltensor)
