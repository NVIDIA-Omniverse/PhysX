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
from ovphysx.types import ObjectScope
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
