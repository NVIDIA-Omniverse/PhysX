# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-WRITE-001
# @covers AC-9
# @maps_to TEST-CAPI-WRITE-001

"""Session read/write GPU DirectGPU counterparts of the warp tensor-binding smoke tests."""

import numpy as np
from ovphysx.types import SimObjectType
from session_helpers import read_column, write_column, write_uniform
from test_utils import attach_usd_with_ovstage, data_path


_RB_PATTERN = "/World/Cube*"  # rigid bodies in boxes_falling_on_groundplane.usda


def _load_boxes(sdk):
    stage = attach_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
    sdk.wait_all()
    sdk.warmup()
    return stage


class TestWarpGpuShapeProperties3D:
    """Session DirectGPU round-trip for per-shape friction and restitution."""

    # Covers test_shape_friction_roundtrip_3d's check: per-shape friction and restitution round-trip on DirectGPU.
    def test_session_shape_friction_roundtrip(self, physx_sdk):
        """Per-shape static/dynamic friction and restitution written on a DirectGPU scene read back."""
        stage = attach_usd_with_ovstage(physx_sdk, data_path("simple_physics_scene.usda"))
        physx_sdk.wait_all()
        physx_sdk.warmup()
        # A per-shape row is padded to the widest body, so only its first shapeCount values are real.
        shape_counts = {
            path: int(row[0])
            for path, row in read_column(physx_sdk, stage, SimObjectType.RIGID_BODY, "shapeCount",
                                         pattern=_RB_PATTERN).items()
        }
        assert shape_counts and all(count >= 1 for count in shape_counts.values())

        # Values unlike the scene's defaults (0.5 / 0.5 / 0.0), so a dropped write shows.
        for attribute, value in (("staticFriction", 0.7), ("dynamicFriction", 0.4), ("restitution", 0.3)):

            def set_shapes(path, row, value=value):
                row[: shape_counts[path]] = value
                return row

            write_column(physx_sdk, stage, SimObjectType.RIGID_BODY, attribute, set_shapes, pattern=_RB_PATTERN)
            result = read_column(physx_sdk, stage, SimObjectType.RIGID_BODY, attribute, pattern=_RB_PATTERN)
            assert set(result) == set(shape_counts), attribute
            for path, row in result.items():
                np.testing.assert_allclose(row[: shape_counts[path]], value, atol=1e-4, err_msg=f"{attribute} {path}")


class TestWarpGpuStructuredTypes:
    """Session DirectGPU round-trip for rigid-body pose."""

    # Covers test_wp_transform_pose's check: a written pose, non-identity orientation included, reads back on DirectGPU.
    def test_session_pose_write_reads_back(self, physx_sdk):
        """A written position and orientation read back as written on a DirectGPU scene."""
        stage = _load_boxes(physx_sdk)
        # Translation (1,2,3) and 90deg about +z, so quat (x,y,z,w) = (0,0,s,s).
        s = 2.0 ** -0.5
        position, orientation = [1.0, 2.0, 3.0], [0.0, 0.0, s, s]
        written = write_uniform(physx_sdk, stage, SimObjectType.RIGID_BODY, "position", position,
                                pattern=_RB_PATTERN)
        assert written > 0
        assert write_uniform(physx_sdk, stage, SimObjectType.RIGID_BODY, "orientation", orientation,
                             pattern=_RB_PATTERN) == written

        positions = read_column(physx_sdk, stage, SimObjectType.RIGID_BODY, "position", pattern=_RB_PATTERN)
        orientations = read_column(physx_sdk, stage, SimObjectType.RIGID_BODY, "orientation", pattern=_RB_PATTERN)
        assert len(positions) == written and set(orientations) == set(positions)
        for path in positions:
            np.testing.assert_allclose(positions[path], position, atol=1e-4)
            np.testing.assert_allclose(orientations[path], orientation, atol=1e-4)
