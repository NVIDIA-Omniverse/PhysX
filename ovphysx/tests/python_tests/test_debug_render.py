# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Python debug-render settings, geometry, and attached-stage ownership."""

# @implements REQ-CAPI-DEBUG-RENDER-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-6 AC-7 AC-8 AC-9
# @maps_to TEST-CAPI-DEBUG-RENDER-001

import numpy as np
import ovstage
import pytest
import warp as wp
from ovphysx._bindings import writability
from ovphysx.types import SimObjectType
from test_utils import attach_usd_with_ovstage, data_path

WRITABLE = 1
READ_ONLY = 4


def _to_host(t) -> np.ndarray:
    assert isinstance(t, wp.array)
    return t.numpy()


def _fill(t, block) -> None:
    assert isinstance(t, wp.array)
    t.assign(np.ascontiguousarray(block).reshape(t.shape))


def test_debug_render_writability():
    assert writability(int(SimObjectType.DEBUG_RENDER), "bodyAxes") == WRITABLE
    assert writability(int(SimObjectType.DEBUG_RENDER), "pointPosition") == READ_ONLY


def test_debug_render_settings_round_trip_and_empty_geometry(physx_sdk):
    """Settings work with no stage; geometry columns are empty without a matching scene."""
    physx = physx_sdk
    physx.detach_ovstage()
    physx.wait_all()

    with physx.write_sync(SimObjectType.DEBUG_RENDER, "enabled") as session:
        assert len(session.groups) == 1
        assert session.groups[0].attribute == 0
        _fill(session.groups[0].tensors[0], np.array([1], dtype=np.uint8))
        session.commit(session.groups[0]).wait()

    with physx.write_sync(SimObjectType.DEBUG_RENDER, "bodyAxes") as session:
        assert len(session.groups) == 1
        _fill(session.groups[0].tensors[0], np.array([2.5], dtype=np.float32))
        session.commit(session.groups[0]).wait()

    with physx.read_sync(
        SimObjectType.DEBUG_RENDER, ["bodyAxes", "linePosition", "lineColor"]
    ) as result:
        assert len(result.groups) == 3
        assert all(group.attribute == 0 for group in result.groups)
        assert float(_to_host(result.groups[0].tensors[0]).reshape(-1)[0]) == pytest.approx(2.5)
        assert int(_to_host(result.groups[1].tensors[0]).shape[0]) == 0
        assert int(_to_host(result.groups[2].tensors[0]).shape[0]) == 0


def test_debug_render_attached_geometry_and_attribute_tokens(physx_sdk):
    physx = physx_sdk
    stage = attach_usd_with_ovstage(physx, data_path("boxes_falling_on_groundplane.usda"))
    with physx.query(SimObjectType.DEBUG_RENDER, metadata=True).wait() as query:
        assert query.metadata.total_prim_count == 0
        assert query.metadata.attributes == ()
        with query.dictionary as dictionary:
            line_token = dictionary.intern_token("linePosition")

    try:
        for name, value, dtype in (
            ("enabled", 1, np.uint8),
            ("scale", 1.0, np.float32),
            ("collisionShapes", 1.0, np.float32),
        ):
            with physx.write_sync(SimObjectType.DEBUG_RENDER, name) as session:
                group = session.groups[0]
                with ovstage.PathDictionary(stage) as dictionary:
                    assert dictionary.token_to_string(group.attribute) == name
                _fill(group.tensors[0], np.array([value], dtype=dtype))
                session.commit(group).wait()

        physx.step(1.0 / 60.0)
        physx.wait_all()
        with physx.read_sync(
            SimObjectType.DEBUG_RENDER, ["linePosition", "lineColor"]
        ) as result:
            assert len(result.groups) == 2
            with result.dictionary as dictionary:
                assert [dictionary.token_to_string(group.attribute) for group in result.groups] == [
                    "linePosition", "lineColor"
                ]
            positions = _to_host(result.groups[0].tensors[0]).copy()
            assert positions.shape[0] > 0
            assert positions.shape[0] == result.groups[1].tensors[0].shape[0]

        with physx.read_tokens_sync(SimObjectType.DEBUG_RENDER, [line_token]) as result:
            np.testing.assert_array_equal(_to_host(result.groups[0].tensors[0]), positions)
    finally:
        for name in ("collisionShapes", "enabled"):
            with physx.write_sync(SimObjectType.DEBUG_RENDER, name) as session:
                group = session.groups[0]
                group.tensors[0].zero_()
                session.commit(group).wait()


def test_debug_render_query_and_read_are_invalidated_by_detach(physx_sdk):
    physx = physx_sdk
    attach_usd_with_ovstage(physx, data_path("boxes_falling_on_groundplane.usda"))
    with physx.query(SimObjectType.DEBUG_RENDER).wait() as query:
        with physx.read(query, ["scale"]).wait() as result:
            physx.detach_ovstage()
            physx.wait_all()
            with pytest.raises(RuntimeError, match="invalidated attachment"):
                _ = query.handle
            with pytest.raises(RuntimeError, match="invalidated attachment"):
                result.fetch_next()


def test_debug_render_detach_before_collection_has_no_stage(physx_sdk):
    physx = physx_sdk
    attach_usd_with_ovstage(physx, data_path("boxes_falling_on_groundplane.usda"))
    task = physx.detach_ovstage()
    with physx.write_sync(SimObjectType.DEBUG_RENDER, "enabled") as session:
        assert session.groups[0].attribute == 0
    physx.wait_task(task)


def test_debug_render_scope_and_culling_box_use_values(physx_sdk):
    """Scope and culling-box length live in the payload, not the tensor descriptor."""
    physx = physx_sdk
    physx.detach_ovstage()
    physx.wait_all()

    with pytest.raises(ValueError, match="scope and cullingBox"):
        physx.write_sync(SimObjectType.DEBUG_RENDER, "enabled", values=[1])

    with physx.write_sync(SimObjectType.DEBUG_RENDER, "scope", values=[11, 22]) as session:
        assert session.groups[0].tensors[0].shape == (4097,)
        session.commit(session.groups[0]).wait()
    with physx.read_sync(SimObjectType.DEBUG_RENDER, ["scope"]) as result:
        scope = _to_host(result.groups[0].tensors[0]).reshape(-1)
        assert scope.tolist() == [11, 22]

    with physx.write_sync(SimObjectType.DEBUG_RENDER, "scope", values=[]) as session:
        session.commit(session.groups[0]).wait()
    with physx.read_sync(SimObjectType.DEBUG_RENDER, ["scope"]) as result:
        assert int(_to_host(result.groups[0].tensors[0]).shape[0]) == 0

    box = np.array([-1.0, -2.0, -3.0, 4.0, 5.0, 6.0], dtype=np.float32)
    with physx.write_sync(SimObjectType.DEBUG_RENDER, "cullingBox", values=box) as session:
        assert session.groups[0].tensors[0].shape == (7,)
        session.commit(session.groups[0]).wait()
    with physx.read_sync(SimObjectType.DEBUG_RENDER, ["cullingBox"]) as result:
        stored = _to_host(result.groups[0].tensors[0]).reshape(-1)
        assert stored.tolist() == pytest.approx(box.tolist())

    with physx.write_sync(SimObjectType.DEBUG_RENDER, "cullingBox", values=[]) as session:
        session.commit(session.groups[0]).wait()
    with physx.read_sync(SimObjectType.DEBUG_RENDER, ["cullingBox"]) as result:
        assert int(_to_host(result.groups[0].tensors[0]).shape[0]) == 0
