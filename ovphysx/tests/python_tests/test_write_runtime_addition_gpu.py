# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""A DirectGPU write opened first after a runtime addition covers the added articulation."""

# @implements REQ-INPUT-CORE-001
# @covers AC-1
# @maps_to TEST-INPUT-CORE-001

import textwrap
from pathlib import Path

import numpy as np
import ovstage
from ovphysx.types import SimObjectType
from session_helpers import read_column, write_uniform
from test_utils import attach_usd_with_ovstage, data_path

DT = 1.0 / 60.0


def _copy_of_second_articulation():
    """`/World/articulation2` of two_articulations.usda as a new root prim, `/Added`."""
    source = Path(data_path("two_articulations.usda")).read_text()
    start = source.index('    def Xform "articulation2"')
    end = source.index("\n    }\n", start) + len("\n    }\n")
    block = textwrap.dedent(source[start:end])
    return "#usda 1.0\n" + block.replace("/World/articulation2/", "/Added/").replace('"articulation2"', '"Added"')


def test_root_write_first_after_runtime_addition_gpu(physx_sdk):
    """A read while the added articulation awaits insertion builds the view without it. A DirectGPU root write
    resolves its rows when it opens, so the open must rebuild that view to cover the added articulation."""
    stage = attach_usd_with_ovstage(physx_sdk, data_path("two_articulations.usda"))
    physx_sdk.step_sync(DT)  # no read yet, so the read below is the one that builds the view

    ovstage.population.add_usd_reference_from_string(stage, _copy_of_second_articulation(), "/")
    ovstage.population.apply_usd_changes(stage, ordinal=2)
    stage.advance_write_floor(ordinal=2).wait()
    physx_sdk.wait_task(physx_sdk.update_from_ovstage(2, 2))
    # A read that includes the pending articulation is omitted before it builds the view; select the others.
    assert len(read_column(physx_sdk, stage, SimObjectType.ARTICULATION, "rootPosition", pattern="/World/*")) == 2
    physx_sdk.step_sync(DT)

    written = write_uniform(physx_sdk, stage, SimObjectType.ARTICULATION, "rootPosition", [3.0, 3.0, 3.0])
    values = read_column(physx_sdk, stage, SimObjectType.ARTICULATION, "rootPosition")
    assert written == len(values) == 3, "the write must cover the added articulation, like the read after it"
    for row in values.values():
        np.testing.assert_allclose(row, 3.0, rtol=0, atol=1e-5)
