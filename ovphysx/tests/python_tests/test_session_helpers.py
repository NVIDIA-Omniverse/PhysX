# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""The session test helpers keep a joint's DOF rows together and every other object's rows apart.

A joint group repeats a joint's path once per DOF; a tendon group repeats a root prim's path once per tendon on
it. The helpers must group the first and not the second. The groups here are fakes over a real path
dictionary, so no simulation runs.
"""

from contextlib import contextmanager
from types import SimpleNamespace

import numpy as np
import ovstage
import pytest
import warp as wp
from ovphysx.types import SimObjectType
from session_helpers import read_column, write_uniform

_PRIM = "/World/robot"


class _Session:
    """What read_sync and write_sync hand back, as far as the helpers use it."""

    def __init__(self, groups):
        self.groups = groups

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        return False

    def commit_sync(self, _group, **_kwargs):
        pass


class _PhysX:
    def __init__(self, group):
        self._groups = [group]

    def read_sync(self, *_args, **_kwargs):
        return _Session(self._groups)

    def write_sync(self, *_args, **_kwargs):
        return _Session(self._groups)


@pytest.fixture
def stage():
    stage = ovstage.Stage("session-helpers")
    yield stage
    stage.destroy()


@contextmanager
def _group_on_one_prim(stage, column):
    """A fixed group over `column` whose rows all name _PRIM, as a joint's DOFs or a root's tendons do."""
    rows = int(column.shape[0])
    with ovstage.PathDictionary(stage) as paths:
        with paths.create_path_list([paths.intern_path(_PRIM)] * rows) as prim_list:
            yield SimpleNamespace(
                is_array=False, index_map=None, tensors=[column], prim_list=prim_list, prim_offset=0, prim_count=rows
            )


@pytest.mark.parametrize("object_type", [SimObjectType.FIXED_TENDON, SimObjectType.SPATIAL_TENDON])
def test_uniform_write_fills_every_tendon_on_one_prim(stage, object_type):
    column = wp.zeros(2, dtype=wp.float32, device="cpu")
    with _group_on_one_prim(stage, column) as group:
        assert write_uniform(_PhysX(group), stage, object_type, "tendonStiffness", [2.0]) == 2
    np.testing.assert_array_equal(column.numpy(), [2.0, 2.0])


def test_uniform_write_fills_every_tendon_limit_pair_on_one_prim(stage):
    column = wp.zeros((2, 2), dtype=wp.float32, device="cpu")
    with _group_on_one_prim(stage, column) as group:
        assert write_uniform(_PhysX(group), stage, SimObjectType.FIXED_TENDON, "tendonLimit", [-1.0, 1.0]) == 2
    np.testing.assert_array_equal(column.numpy(), [[-1.0, 1.0], [-1.0, 1.0]])


def test_a_joint_writes_and_reads_its_dof_rows_together(stage):
    column = wp.zeros(2, dtype=wp.float32, device="cpu")
    with _group_on_one_prim(stage, column) as group:
        physx = _PhysX(group)
        assert write_uniform(physx, stage, SimObjectType.ARTICULATION_JOINT, "jointPosition", [2.0, 3.0]) == 1
        values = read_column(physx, stage, SimObjectType.ARTICULATION_JOINT, "jointPosition")
    np.testing.assert_array_equal(column.numpy(), [2.0, 3.0])
    assert list(values) == [_PRIM]
    np.testing.assert_array_equal(values[_PRIM], [2.0, 3.0])
