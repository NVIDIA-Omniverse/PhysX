# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Regression test for foreign DLPack capsules retained past PhysX release."""

import ctypes
import gc
import os

import warp as wp
import warp._src.dlpack as warp_dlpack
from ovphysx.types import TensorType
from test_utils import load_usd_with_ovstage

from ovphysx import PhysX


def test_warp_cache_survives_callback_teardown():
    """Cached descriptors must not call a foreign capsule destructor during late GC."""
    wp.init()
    PhysX.set_cpu_mode(True)
    physx = PhysX()
    tests_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    usd_path = os.path.join(tests_dir, "data", "boxes_falling_on_groundplane.usda")
    load_usd_with_ovstage(physx, usd_path)
    physx.wait_all()

    read_binding = physx.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    read_buffer = wp.zeros(read_binding.shape, dtype=wp.float32, device="cpu")
    read_binding.read(read_buffer)
    assert read_binding._read_cache is not None

    write_binding = physx.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )
    write_buffer = wp.zeros(write_binding.shape, dtype=wp.float32, device="cpu")
    write_binding.write(write_buffer)
    assert write_binding._write_cache is not None

    physx.release()

    assert warp_dlpack._dlpack_capsule_deleter is not None
    assert warp_dlpack._dlpack_tensor_deleter is not None
    warp_dlpack._dlpack_capsule_deleter = None
    warp_dlpack._dlpack_tensor_deleter = None
    # Overwrite freed libffi closure storage before collecting the caches;
    # this makes the old cache behavior fail reliably.
    heap_churn = [bytes(4096) for _ in range(20000)]
    heap_churn.extend((ctypes.c_double * 512)() for _ in range(2000))

    del read_binding
    del read_buffer
    del write_binding
    del write_buffer
    gc.collect()
    del heap_churn
