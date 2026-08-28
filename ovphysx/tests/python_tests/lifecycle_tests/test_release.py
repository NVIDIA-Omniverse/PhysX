# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Consolidated release/lifecycle tests — ONE create+release per file.

Carbonite/Python cannot be re-initialized after ovphysx_destroy_instance, so
all release-related assertions share a single PhysX instance.  The instance is
created once, exercised, then released; post-release behaviour is validated
after the release.

This file runs in its own subprocess (see test_python.cmake).
"""

import os

import numpy as np
import pytest
from ovphysx.types import TensorType

from ovphysx import PhysX
from test_utils import load_usd_with_ovstage


def test_release_comprehensive():
    """Comprehensive test for PhysX.release() behaviour.

    Covers:
        - Explicit release completes without error
        - Double release is safe (idempotent)
        - Operations after release raise RuntimeError/AttributeError
        - Binding operations after SDK release raise with clear message
    """
    physx = PhysX()
    assert physx is not None

    # -- Load a USD and create a binding before release --
    tests_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    usd_path = os.path.join(tests_dir, "data", "basic_simulation.usda")
    load_usd_with_ovstage(physx, usd_path)
    physx.wait_all()

    binding = physx.create_tensor_binding(
        prim_paths=["/World/envs/env0/table"],
        tensor_type=TensorType.RIGID_BODY_POSE,
    )

    # -- Explicit release --
    physx.release()

    # -- Double release must be safe --
    physx.release()

    # -- Use-after-release must raise for ALL public methods --
    msg = "has been released"
    with pytest.raises((RuntimeError, AttributeError), match=msg):
        physx.step(0.016)
    with pytest.raises((RuntimeError, AttributeError), match=msg):
        physx.step_sync(0.016)
    with pytest.raises((RuntimeError, AttributeError), match=msg):
        load_usd_with_ovstage(physx, "foo.usda")
    with pytest.raises((RuntimeError, AttributeError), match=msg):
        physx.reset_stage()
    with pytest.raises((RuntimeError, AttributeError), match=msg):
        physx.wait_all()
    with pytest.raises((RuntimeError, AttributeError), match=msg):
        physx.wait_op(0)
    with pytest.raises((RuntimeError, AttributeError), match=msg):
        physx.create_tensor_binding(pattern="/World/*")
    with pytest.raises((RuntimeError, AttributeError), match=msg):
        physx.warmup_gpu()

    # -- Binding operations after SDK release must raise --
    tensor = np.zeros(binding.shape, dtype=np.float32)
    with pytest.raises(RuntimeError, match="parent PhysX instance has been released"):
        binding.read(tensor)
    with pytest.raises(RuntimeError, match="parent PhysX instance has been released"):
        binding.write(tensor)
    with pytest.raises(RuntimeError, match="parent PhysX instance has been released"):
        binding.destroy()
