# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Cache-after-release tests — ONE create+release.

Validates that the read/write cache correctly detects a released SDK and raises
rather than using stale handles.

Runs in its own subprocess (see test_python.cmake).
"""

import os

import numpy as np
import pytest
from ovphysx.types import TensorType

from ovphysx import PhysX
from test_utils import load_usd_with_ovstage


def _data_path(name):
    tests_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(tests_dir, "data", name)


def test_cache_after_release():
    """Read/write cache must raise after SDK release."""
    PhysX.set_cpu_mode(True)
    physx = PhysX()
    load_usd_with_ovstage(physx, _data_path("boxes_falling_on_groundplane.usda"))
    physx.wait_all()

    # -- Read cache --
    read_binding = physx.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    read_buf = np.zeros(read_binding.shape, dtype=np.float32)
    read_binding.read(read_buf)
    assert read_binding._read_cache is not None

    # -- Write cache --
    write_binding = physx.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )
    write_buf = np.zeros(write_binding.shape, dtype=np.float32)
    write_binding.write(write_buf)
    assert write_binding._write_cache is not None

    # -- Release SDK --
    physx.release()

    # -- Cached read must raise --
    with pytest.raises(RuntimeError, match="parent PhysX instance has been released"):
        read_binding.read(read_buf)

    # -- Cached write must raise --
    with pytest.raises(RuntimeError, match="parent PhysX instance has been released"):
        write_binding.write(write_buf)
