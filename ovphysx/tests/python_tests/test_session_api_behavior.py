# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-READ-SELECT-001
# @covers AC-1 AC-3
# @maps_to TEST-READ-SELECT-001
# @implements REQ-CAPI-WRITE-001
# @covers AC-9
# @maps_to TEST-CAPI-WRITE-001

"""Session-API selection and lifecycle contracts: prim_paths selection, commit-after-close, and read/write without an attached stage."""

import ovstage
import pytest
from ovphysx.types import SimObjectType
from session_helpers import handles_for
from test_utils import attach_usd_with_ovstage, data_path, load_usd_with_ovstage


def test_session_prim_paths_select_each_existing_body_once(physx_sdk):
    """A prim_paths selection with absent and repeated paths reads each existing body exactly once."""
    stage = attach_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.warmup()  # DirectGPU serves simulated state only after the first step

    selection = handles_for(stage, ["/World/Cube3", "/World/Missing", "/World/Cube1", "/World/Cube2", "/World/Cube3"])
    with physx_sdk.read_sync(SimObjectType.RIGID_BODY, ["position"], prim_paths=selection) as result:
        with ovstage.PathDictionary(stage) as dictionary:
            paths = [
                dictionary.path_to_string(handle)
                for group in result.groups
                for handle in dictionary.get_paths(group.prim_list)[
                    group.prim_offset : group.prim_offset + group.prim_count
                ]
            ]
    assert sorted(paths) == ["/World/Cube1", "/World/Cube2", "/World/Cube3"]


def test_write_session_commit_after_close_raises(physx_sdk):
    """A group cannot be committed once its write session has closed."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.warmup()

    with physx_sdk.write_sync(SimObjectType.RIGID_BODY, "position") as session:
        group = session.groups[0]
    with pytest.raises(RuntimeError, match="closed"):
        session.commit_sync(group)


def test_session_read_and_write_without_stage_raise(physx_sdk):
    """With no ovstage attached, opening a session read or write raises."""
    with pytest.raises(RuntimeError, match="(?i)ovstage"):
        physx_sdk.read_sync(SimObjectType.RIGID_BODY, ["position"])
    with pytest.raises(RuntimeError, match="(?i)ovstage"):
        physx_sdk.write_sync(SimObjectType.RIGID_BODY, "position")
