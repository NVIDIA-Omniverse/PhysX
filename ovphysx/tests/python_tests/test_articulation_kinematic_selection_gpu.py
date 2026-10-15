# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-TENSOR-FK-001
# @covers AC-1 AC-3 AC-4
# @maps_to TEST-TENSOR-FK-001
# @implements REQ-PYTHON-TASK-001
# @covers AC-8

"""A kinematic update over a prim-set selection refreshes only the selected articulations (DirectGPU).

On DirectGPU a joint write leaves the links in place until the kinematic update, so the links that move
are exactly the ones the update refreshed. A partial selection must use the indexed dispatch; a selection
covering every articulation may use the scene-wide one. Both are checked by outcome: the unselected
articulation's links stay bit-identical. The CPU cases are C tests (test_articulation_kinematic_isolation.cpp),
since a CPU joint write already moves the links.
"""

import numpy as np
import ovstage
import pytest
from ovphysx.types import ObjectScope, SimObjectType
from test_utils import attach_usd_with_ovstage, data_path

_ARTI_A = "/World/articulation"
_ARTI_B = "/World/articulation2"

# (selection, articulations whose links must move). An articulation is named through its root, a link
# or a joint.
_SELECTIONS = [
    ({"pattern": _ARTI_A}, {_ARTI_A}),
    ({"paths": [_ARTI_B + "/articulationLink1"]}, {_ARTI_B}),
    ({"pattern": _ARTI_B + "/articulatedRevoluteJoint2"}, {_ARTI_B}),
    ({"pattern": "/World/articulation*"}, {_ARTI_A, _ARTI_B}),
    ({}, {_ARTI_A, _ARTI_B}),
]


def _selected_kinematic_update(sdk, *, paths=None, pattern=None):
    """Write every joint of both articulations, then run the kinematic update over the selection.

    Returns each link's position keyed by prim path, before the write and after the update.
    """
    stage = attach_usd_with_ovstage(sdk, data_path("two_articulations.usda"))
    # DirectGPU refuses a write before the first step; warmup() provides it.
    sdk.warmup()
    sdk.wait_all()

    def link_positions():
        out = {}
        with sdk.read_sync(SimObjectType.ARTICULATION_LINK, ["position"], scope=ObjectScope.ALL) as result:
            with ovstage.PathDictionary(stage) as dictionary:
                for group in result.groups:
                    handles = dictionary.get_paths(group.prim_list)
                    rows = np.asarray(group.tensors[0].numpy()).reshape(group.prim_count, -1)
                    for r in range(group.prim_count):
                        out[dictionary.path_to_string(handles[group.prim_offset + r])] = rows[r].copy()
        return out

    before = link_positions()
    # 17.19 degrees is about 0.3 rad. Commit publishes the whole group, so every tensor is filled.
    with sdk.write_sync(SimObjectType.ARTICULATION_JOINT, "jointPosition") as write:
        for group in write.groups:
            for tensor in group.tensors:
                tensor.assign(np.full(tuple(tensor.shape), 17.19, dtype=np.float32))
            write.commit_sync(group)

    handles = None
    if paths:
        with ovstage.PathDictionary(stage) as dictionary:
            handles = [dictionary.intern_path(p) for p in paths]
    sdk.update_articulations_kinematic(prim_paths=handles, pattern=pattern)
    return before, link_positions()


@pytest.mark.parametrize(("selection", "refreshed"), _SELECTIONS)
def test_kinematic_update_refreshes_only_the_selection_gpu(physx_sdk, selection, refreshed):
    before, after = _selected_kinematic_update(physx_sdk, **selection)
    for root in (_ARTI_A, _ARTI_B):
        links = [path for path in before if path.startswith(root + "/")]
        assert links, f"no links read under {root}"
        moved = [path for path in links if not np.array_equal(before[path], after[path])]
        if root in refreshed:
            assert moved, f"{root} was selected but none of its links moved"
        else:
            assert not moved, f"{root} was not selected but links moved: {moved}"
