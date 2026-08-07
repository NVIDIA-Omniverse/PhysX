# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Fresh-process regression test for NVBug 6504951 SDF handle aliasing.

Instance and per-instance SDF-view handles both used to count from 1, so the
first instance handle resolved the first SDF view in that instance. Both now
draw from one process-wide never-reused sequence, so the instance handle misses
the SDF-view map.

Only ONE PhysX create+release cycle is permitted per lifecycle test file.
"""

import ctypes

from ovphysx import PhysX, PhysXConfig, _bindings
from ovphysx.types import ApiStatus
from test_utils import data_path, load_usd_with_ovstage

_CUBE_PATTERN = "/World/Cube"


def test_instance_handle_is_not_an_sdf_view_handle():
    physx = PhysX(
        config=PhysXConfig(
            carbonite_overrides={
                "/physics/suppressReadback": True,
            }
        ),
    )
    try:
        load_usd_with_ovstage(physx, data_path("sdf_cube.usda"))
        physx.wait_all()
        physx.warmup_gpu()

        sdf_view = physx.create_sdf_view(
            pattern=_CUBE_PATTERN, max_query_points=1
        )
        try:
            sdk_handle = physx.handle
            sdf_handle = sdf_view._handle
            assert sdk_handle != sdf_handle

            count = ctypes.c_uint32(0)
            live = _bindings._lib.ovphysx_sdf_view_get_count(
                sdk_handle, sdf_handle, ctypes.byref(count)
            )
            assert live.status == ApiStatus.SUCCESS
            assert count.value == 1

            reproducer = _bindings._lib.ovphysx_sdf_view_get_count(
                sdk_handle, sdk_handle, ctypes.byref(count)
            )
            assert reproducer.status == ApiStatus.NOT_FOUND
        finally:
            sdf_view.destroy()
    finally:
        physx.release()
