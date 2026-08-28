# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""GPU tensor cache staleness tests after SDK release — ONE create+release per file.

Verifies that the fast-path read/write cache correctly detects a released SDK
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


def test_cache_staleness_after_sdk_release():
    """Warming read/write caches then releasing the SDK must raise on the next access.

    The fast path checks the live _omni_physx_sdk_handle (not the cached integer),
    so it must fail cleanly after PhysX.release() even though the cache still holds
    a stale handle value.

    Covers both read cache (RIGID_BODY_POSE) and write cache (RIGID_BODY_VELOCITY)
    in a single create+release cycle.
    """
    # Opt into DirectGPU since the test writes via a CudaArray (GPU-resident
    # write_binding). ovphysx no longer auto-enables suppressReadback (0.4.x);
    # see create_args doc-comment in ovphysx_types.h.
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
    physx.warmup_gpu()

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

    physx.release()

    with pytest.raises(RuntimeError, match="parent PhysX instance has been released"):
        read_binding.read(read_ga.dltensor)

    with pytest.raises(RuntimeError, match="parent PhysX instance has been released"):
        write_binding.write(write_ga.dltensor)
