# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tendon access before scene insertion must not leave later reads and writes empty (NVBug 6904267)."""

# @implements REQ-READ-TENDON-001
# @covers AC-5
# @maps_to TEST-READ-TENDON-001
# @implements REQ-CAPI-WRITE-001
# @covers AC-5a
# @maps_to TEST-CAPI-WRITE-001
# @implements REQ-INPUT-CORE-001
# @covers AC-1
# @maps_to TEST-INPUT-CORE-001

from pathlib import Path

import numpy as np
import ovstage
import pytest
from ovphysx.types import SimObjectType
from test_utils import attach_usd_with_ovstage, data_path


@pytest.fixture(params=["Fixed", "Spatial"])
def tendon_scene(request, physx_sdk, tmp_path):
    """One tendon with damping 12, and the USD of a second copy with damping 20 to add at runtime."""
    kind = request.param
    root = f"{kind}TendonTest"
    authored = 0 if kind == "Fixed" else 2
    source = Path(data_path(f"{root}.usda")).read_text()
    source = source.replace(f"physxTendon:body0:damping = {authored}", "physxTendon:body0:damping = 12")
    path = tmp_path / "tendons.usda"
    path.write_text(source)
    stage = attach_usd_with_ovstage(physx_sdk, str(path))
    object_type = SimObjectType.FIXED_TENDON if kind == "Fixed" else SimObjectType.SPATIAL_TENDON
    addition = "#usda 1.0\n" + source[source.index(f'def Xform "{root}"'):]
    addition = addition.replace(root, "AddedTendon").replace("damping = 12", "damping = 20")
    return stage, object_type, addition


def _add_at_runtime(physx, stage, addition):
    ovstage.population.add_usd_reference_from_string(stage, addition, "/")
    ovstage.population.apply_usd_changes(stage, ordinal=2)
    stage.advance_write_floor(ordinal=2).wait()
    physx.wait_task(physx.update_from_ovstage(2, 2))


def _assert_damping(physx, object_type, expected):
    with physx.read_sync(object_type, ["tendonDamping"], metadata=True) as result:
        assert result.metadata.total_prim_count == len(expected)
        assert result.metadata.attributes[0].omission_reasons == 0
        assert result.groups
        values = np.concatenate([g.tensors[0].numpy().reshape(-1) for g in result.groups])
        np.testing.assert_array_equal(np.sort(values), np.sort(expected))


def test_tendon_read_returns_authored_value_before_step(physx_sdk, tendon_scene):
    _, object_type, _ = tendon_scene
    _assert_damping(physx_sdk, object_type, [12])


@pytest.mark.parametrize("first_access", ["read", "discard_write", "commit_write"])
def test_tendon_access_before_first_step(physx_sdk, tendon_scene, first_access):
    """The first access is a read or a write session; later steps keep the complete tendon set."""
    _, object_type, _ = tendon_scene
    expected = 30 if first_access == "commit_write" else 12
    if first_access == "read":
        with physx_sdk.read_sync(object_type, ["tendonDamping"]):
            pass
    else:
        with physx_sdk.write_sync(object_type, ["tendonDamping"]) as session:
            assert session.groups
            if first_access == "commit_write":
                for group in session.groups:
                    group.tensors[0].fill_(expected)
                    session.commit_sync(group)
    for _ in range(5):
        physx_sdk.step_sync(1e-4)
    _assert_damping(physx_sdk, object_type, [expected])
    with physx_sdk.write_sync(object_type, ["tendonDamping"]) as session:
        assert session.groups
        for group in session.groups:
            group.tensors[0].fill_(40)
            session.commit_sync(group)
    _assert_damping(physx_sdk, object_type, [40])


@pytest.mark.parametrize("pattern", [None, "/DoesNotExist*"], ids=["whole_type", "empty_selection"])
def test_tendon_read_during_runtime_addition(physx_sdk, tendon_scene, pattern):
    """A read while an added articulation awaits insertion must not hide its tendon after the step."""
    stage, object_type, addition = tendon_scene
    physx_sdk.step_sync(1e-4)
    _assert_damping(physx_sdk, object_type, [12])
    _add_at_runtime(physx_sdk, stage, addition)
    with physx_sdk.read_sync(object_type, ["tendonDamping"], pattern=pattern):
        pass
    for _ in range(2):
        physx_sdk.step_sync(1e-4)
        _assert_damping(physx_sdk, object_type, [12, 20])


def _write_all(physx, object_type, attribute, value):
    """Fill every row of a whole-type write with `value`; return the number of rows written."""
    with physx.write_sync(object_type, [attribute]) as session:
        for group in session.groups:
            for tensor in group.tensors:
                tensor.fill_(value)
            session.commit_sync(group)
        return sum(group.prim_count for group in session.groups)


def _read_all(physx, object_type, attribute):
    with physx.read_sync(object_type, [attribute]) as result:
        rows = sum(group.prim_count for group in result.groups)
        values = [tensor.numpy().reshape(-1) for group in result.groups for tensor in group.tensors]
        return rows, np.concatenate(values) if values else np.zeros(0)


@pytest.mark.parametrize("write_kind", ["tendon", "joint", "joint_property", "root"])
def test_write_first_after_runtime_addition(physx_sdk, tendon_scene, write_kind):
    """A read during a runtime addition builds the articulation view before the insertion. A write that is the
    first access after the inserting step must still cover the added articulation."""
    stage, object_type, addition = tendon_scene
    physx_sdk.step_sync(1e-4)  # no read yet, so the read below is the one that builds the view
    _add_at_runtime(physx_sdk, stage, addition)
    with physx_sdk.read_sync(object_type, ["tendonDamping"]):
        pass
    physx_sdk.step_sync(1e-4)

    target_type, attribute, value = {
        "tendon": (object_type, "tendonDamping", 30.0),
        "joint": (SimObjectType.ARTICULATION_JOINT, "jointPosition", 0.05),
        "joint_property": (SimObjectType.ARTICULATION_JOINT, "jointStiffness", 123.0),
        "root": (SimObjectType.ARTICULATION, "rootPosition", 3.0),
    }[write_kind]
    written = _write_all(physx_sdk, target_type, attribute, value)
    rows, values = _read_all(physx_sdk, target_type, attribute)
    assert written == rows, "the write must cover the added articulation, like the read after it"
    if write_kind in ("tendon", "root"):
        assert rows == 2
    np.testing.assert_allclose(values, value, rtol=0, atol=1e-5)
