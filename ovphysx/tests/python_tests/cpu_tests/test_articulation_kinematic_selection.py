# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-TASK-001
# @covers AC-8

"""The prim-set selection arguments of update_articulations_kinematic follow the read and write rules.

Which articulations a selected update refreshes is checked on DirectGPU by
test_articulation_kinematic_selection_gpu.py and on CPU by the C tests in
test_articulation_kinematic_isolation.cpp: a CPU joint write already moves the links, so a Python-side
CPU check could not tell a refreshed articulation from an untouched one.
"""

import pytest
from ovphysx.types import ObjectScope, SimObjectType
from test_utils import attach_usd_with_ovstage, data_path


def test_kinematic_update_rejects_a_selection_with_active_scope(physx_sdk_cpu):
    """A selection has no scope, as for read and write."""
    attach_usd_with_ovstage(physx_sdk_cpu, data_path("two_articulations.usda"))
    with pytest.raises(ValueError, match="scope=ACTIVE"):
        physx_sdk_cpu.update_articulations_kinematic(pattern="/World/articulation", scope=ObjectScope.ACTIVE)


def test_kinematic_update_rejects_an_empty_pattern(physx_sdk_cpu):
    """An empty pattern is a caller error, not the whole set."""
    attach_usd_with_ovstage(physx_sdk_cpu, data_path("two_articulations.usda"))
    with pytest.raises(ValueError, match="pattern must be non-empty"):
        physx_sdk_cpu.update_articulations_kinematic(pattern="")


def test_kinematic_update_takes_a_held_query(physx_sdk_cpu):
    """A held articulation query is updated in place and stays open, so a graph capturing the update can replay it
    (ADR-0052); it cannot be combined with a selection, and another object type is refused."""
    attach_usd_with_ovstage(physx_sdk_cpu, data_path("two_articulations.usda"))
    physx_sdk_cpu.step_sync(1.0 / 60.0)
    with physx_sdk_cpu.query(SimObjectType.ARTICULATION).wait() as query:
        assert physx_sdk_cpu.update_articulations_kinematic(query=query) is None
        assert physx_sdk_cpu.update_articulations_kinematic(query=query) is None
        with pytest.raises(ValueError, match="either a held query or a selection"):
            physx_sdk_cpu.update_articulations_kinematic(query=query, pattern="/World/articulation")
    with physx_sdk_cpu.query(SimObjectType.RIGID_BODY).wait() as rigid:
        with pytest.raises(ValueError, match="must select articulations"):
            physx_sdk_cpu.update_articulations_kinematic(query=rigid)
