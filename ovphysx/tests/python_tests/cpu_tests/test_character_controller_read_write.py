# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Public Python coverage for character-controller move and rigid-body pose."""

# @implements REQ-READ-CCT-001
# @covers AC-1 AC-2 AC-3 AC-5
# @maps_to TEST-READ-CCT-001
# @implements REQ-CAPI-WRITE-001
# @covers AC-15
# @maps_to TEST-CAPI-WRITE-001

from pathlib import Path

import numpy as np
import ovstage
import pytest
from ovphysx import PhysX, ReadOmission
from ovphysx.types import SimObjectType
from test_utils import attach_usd_with_ovstage


_SCENE = Path(__file__).parents[2] / "data" / "character_controller.usda"
_CONTROLLER = "/World/Controller"
_OTHER = "/World/Other"


def _rigid_positions(physx: PhysX, stage) -> dict[str, np.ndarray]:
    out: dict[str, np.ndarray] = {}
    with physx.read_sync(SimObjectType.RIGID_BODY, ["position"]) as result:
        with ovstage.PathDictionary(stage) as dictionary:
            for group in result.groups:
                handles = dictionary.get_paths(group.prim_list)
                values = np.asarray(group.tensors[0].numpy()).reshape(group.prim_count, -1)
                for row in range(group.prim_count):
                    index = handles[group.prim_offset + row]
                    out[dictionary.path_to_string(index)] = values[row].copy()
    return out


def _commit_move(physx: PhysX, stage, rows: np.ndarray, *, prim_paths=None, pattern=None) -> list[str]:
    paths: list[str] = []
    with physx.write_sync(
        SimObjectType.CHARACTER_CONTROLLER, "move", prim_paths=prim_paths, pattern=pattern
    ) as session:
        assert session.groups
        with ovstage.PathDictionary(stage) as dictionary:
            for group in session.groups:
                handles = dictionary.get_paths(group.prim_list)
                paths.extend(
                    dictionary.path_to_string(handle)
                    for handle in handles[group.prim_offset : group.prim_offset + group.prim_count]
                )
                target = group.tensors[0]
                block = np.asarray(rows, dtype=np.float32).reshape(target.shape)
                if isinstance(target, np.ndarray):
                    target[:] = block
                else:
                    target.assign(np.ascontiguousarray(block))
                session.commit_sync(group)
    return paths


def test_character_controller_move_and_selection(physx_sdk_cpu: PhysX) -> None:
    stage = attach_usd_with_ovstage(physx_sdk_cpu, str(_SCENE))
    physx_sdk_cpu.wait_all()

    start = _rigid_positions(physx_sdk_cpu, stage)
    assert _CONTROLLER in start and _OTHER in start
    np.testing.assert_allclose(start[_CONTROLLER], [0.0, 5.0, 0.0], atol=1e-4)
    np.testing.assert_allclose(start[_OTHER], [10.0, 5.0, 0.0], atol=1e-4)

    with ovstage.PathDictionary(stage) as dictionary:
        selected = [dictionary.intern_path(_CONTROLLER)]
    moved = _commit_move(physx_sdk_cpu, stage, np.array([[2.0, 0.0, 0.0]], np.float32), prim_paths=selected)
    moved += _commit_move(physx_sdk_cpu, stage, np.array([[1.0, 0.0, 0.0]], np.float32), prim_paths=selected)
    assert moved == [_CONTROLLER, _CONTROLLER]

    patterned = _commit_move(
        physx_sdk_cpu, stage, np.array([[0.0, 0.0, 1.0]], np.float32), pattern="/World/Other"
    )
    assert patterned == [_OTHER]

    # The kinematic target has not been integrated, so both actor poses are unchanged.
    held = _rigid_positions(physx_sdk_cpu, stage)
    np.testing.assert_allclose(held[_CONTROLLER], start[_CONTROLLER], atol=1e-4)
    np.testing.assert_allclose(held[_OTHER], start[_OTHER], atol=1e-4)

    physx_sdk_cpu.step_sync(1.0 / 60.0)
    after = _rigid_positions(physx_sdk_cpu, stage)
    np.testing.assert_allclose(after[_CONTROLLER], [3.0, 5.0, 0.0], atol=1e-3)
    np.testing.assert_allclose(after[_OTHER], [10.0, 5.0, 1.0], atol=1e-3)

    physx_sdk_cpu.step_sync(1.0 / 60.0)
    replayed = _rigid_positions(physx_sdk_cpu, stage)
    np.testing.assert_allclose(replayed[_CONTROLLER], after[_CONTROLLER], atol=1e-3)
    np.testing.assert_allclose(replayed[_OTHER], after[_OTHER], atol=1e-3)

    with pytest.raises(RuntimeError):
        with physx_sdk_cpu.write_sync(SimObjectType.CHARACTER_CONTROLLER, "position"):
            pass
    with pytest.raises(RuntimeError):
        _commit_move(physx_sdk_cpu, stage, np.array([[np.nan, 0.0, 0.0]], np.float32), prim_paths=selected)

    unchanged = _rigid_positions(physx_sdk_cpu, stage)
    np.testing.assert_allclose(unchanged[_CONTROLLER], after[_CONTROLLER], atol=1e-3)
    np.testing.assert_allclose(unchanged[_OTHER], after[_OTHER], atol=1e-3)


@pytest.mark.parametrize("pattern, expected_count", [(None, 2), (_CONTROLLER, 1), ("/World/Missing", 0)])
def test_character_controller_async_session_metadata(
    physx_sdk_cpu: PhysX, pattern: str | None, expected_count: int
) -> None:
    attach_usd_with_ovstage(physx_sdk_cpu, str(_SCENE))
    physx_sdk_cpu.wait_all()
    creation = physx_sdk_cpu.query(SimObjectType.CHARACTER_CONTROLLER, pattern=pattern)
    with pytest.raises(RuntimeError, match="not collected"):
        _ = creation.result
    with creation.wait() as query:
        assert query.metadata is not None
        assert query.metadata.total_prim_count == expected_count
        assert query.metadata.attributes == ()
        with physx_sdk_cpu.read(query, ["position", "move"], metadata=True).wait() as read:
            assert read.metadata is not None
            assert read.metadata.total_prim_count == expected_count
            assert len(read.metadata.attributes) == 2
            for attribute in read.metadata.attributes:
                assert attribute.attribute == 0
                assert attribute.group_count == 0
                assert attribute.omission_reasons == ReadOmission.UNSUPPORTED_ATTRIBUTE
            assert read.fetch_next().wait() is None
        with physx_sdk_cpu.write(query, ["move"], metadata=True).wait() as write:
            assert write.metadata is not None
            assert write.metadata.total_prim_count == expected_count
            assert len(write.metadata.attributes) == 1
            expected_groups = int(expected_count > 0)
            assert write.metadata.attributes[0].group_count == expected_groups
            group = write.fetch_next().wait()
            if expected_count:
                assert group is not None
                assert group.attribute_index == 0
                assert group.attribute == write.metadata.attributes[0].attribute
                assert group.prim_count == expected_count
            else:
                assert group is None
            assert write.fetch_next().wait() is None
    physx_sdk_cpu.wait_all()
