# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-WRITE-001
# @covers AC-9
# @maps_to TEST-CAPI-WRITE-001

"""Session-API pose round-trip: a known position and xyzw orientation written through the session read back unchanged."""

import os

import numpy as np
from ovphysx.types import SimObjectType
from session_helpers import read_column, write_uniform
from test_utils import attach_usd_with_ovstage

# data/ is tests/data, three levels up from this file (cpu_tests/).
_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def data_path(filename):
    return os.path.join(_TEST_DIR, "data", filename)


_RB_PATTERN = "/World/Cube*"  # rigid bodies in boxes_falling_on_groundplane.usda


def _load_boxes(sdk):
    stage = attach_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
    sdk.wait_all()
    return stage


class TestWarpStructuredTypes:

    def test_session_pose_write_reads_back(self, physx_sdk_cpu):
        """A known position and xyzw orientation written through the session read back unchanged."""
        stage = _load_boxes(physx_sdk_cpu)
        # Translation (1,2,3) and 90deg about +z, so quat (x,y,z,w) = (0,0,s,s).
        s = 2.0 ** -0.5
        expected = {"position": [1.0, 2.0, 3.0], "orientation": [0.0, 0.0, s, s]}
        for attribute, values in expected.items():
            assert write_uniform(physx_sdk_cpu, stage, SimObjectType.RIGID_BODY, attribute, values, pattern=_RB_PATTERN)
        for attribute, values in expected.items():
            rows = read_column(physx_sdk_cpu, stage, SimObjectType.RIGID_BODY, attribute, pattern=_RB_PATTERN)
            assert rows, attribute
            for path, row in rows.items():
                np.testing.assert_allclose(row, values, atol=1e-4, err_msg=f"{path} {attribute}")
