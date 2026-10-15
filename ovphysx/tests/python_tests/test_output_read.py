# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for the physics output read API (ADR-0007): PhysX.read_sync over an
ovstage-attached scene. The read is ovstage-native, so the scene is attached
through the ovstage backend (load_usd_with_ovstage).
"""

# @implements REQ-PYTHON-READ-001
# @covers AC-1 AC-3 AC-5
# @maps_to TEST-PYTHON-READ-001
# @implements REQ-CAPI-SLEEP-001
# @covers AC-2
# @maps_to TEST-CAPI-SLEEP-001

import warnings

import numpy as np
import pytest
import warp as wp
import ovstage
from ovphysx.types import ObjectScope, SimObjectType, TensorType
from test_utils import attach_usd_with_ovstage, data_path, load_usd_with_ovstage


def _col_width(t) -> int:
    """Trailing component width of a read column, device-agnostic."""
    return int(t.shape[-1]) if t.ndim >= 2 else 1


def _col_rows(t) -> int:
    """Row count (number of prims) of a read column, device-agnostic."""
    return int(t.shape[0])


def _is_read_column(t) -> bool:
    """Every Python output-read column uses the public Warp frontend."""
    return isinstance(t, wp.array)


def test_rigid_body_output_read(physx_sdk):
    """Query rigid bodies and read position + orientation as faithful column groups.

    The `physx_sdk` fixture is GPU-mode, so its Warp arrays are device-resident.
    The same public type and trailing-component shape are returned on a CPU sim.
    """
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    with physx_sdk.read_sync(
        SimObjectType.RIGID_BODY, ["position", "orientation"], scope=ObjectScope.ALL
    ) as result:
        groups = result.groups

        # At least a position (vec3) and an orientation (quat) column. Native
        # DLPack lanes are exposed uniformly as Warp's trailing dimension.
        assert len(groups) >= 2
        pos_groups = [g for g in groups if g.tensors and _col_width(g.tensors[0]) == 3]
        ori_groups = [g for g in groups if g.tensors and _col_width(g.tensors[0]) == 4]
        assert pos_groups, "expected a position (vec3) column"
        assert ori_groups, "expected an orientation (quat) column"

        g = pos_groups[0]
        assert g.object_type == SimObjectType.RIGID_BODY
        assert not g.is_array and not g.is_delete
        # A standalone-body fixed column: one tensor stacked over its prims.
        assert len(g.tensors) == 1
        data = g.tensors[0]
        assert _is_read_column(data)
        assert data.device.is_cuda
        assert _col_rows(data) > 0 and _col_width(data) == 3
        assert g.prim_count == _col_rows(data)
        # Interned identifiers come back as non-zero handles (valid within the block).
        assert g.attribute != 0
        assert g.prim_list != 0
        # A fixed column has no element-axis scatter.
        assert g.index_map is None


def _read_scalar_ints(physx_sdk, attribute):
    """Read one whole-articulation scalar-int32 metadata column and return its
    per-articulation values. Each attribute is read alone so the single group is
    unambiguously that attribute (all six share the same int32/scalar shape)."""
    with physx_sdk.read_sync(SimObjectType.ARTICULATION, [attribute], scope=ObjectScope.ALL) as result:
        groups = result.groups
        assert len(groups) == 1, attribute
        g = groups[0]
        assert g.object_type == SimObjectType.ARTICULATION
        assert not g.is_array and not g.is_delete
        assert len(g.tensors) == 1
        t = g.tensors[0]
        assert _is_read_column(t)
        # Host-resident even on the GPU-mode fixture (metatype-derived, no device copy).
        assert not t.device.is_cuda
        assert t.dtype == wp.int32
        assert _col_width(t) == 1
        assert _col_rows(t) == g.prim_count
        return t.numpy().reshape(-1).tolist()


def test_articulation_topology_metadata_read(physx_sdk):
    """Read the six whole-articulation topology-metadata scalars (replacing
    ovphysx_articulation_metadata_t) over a scene mixing a fixed- and a
    floating-base articulation: each comes back as a per-articulation int32
    column, and isFixedBase varies per row within one scene-wide read."""
    load_usd_with_ovstage(physx_sdk, data_path("mixed_base_articulations.usda"))
    for _ in range(2):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    # Two articulations (one fixed-base, one floating), two links each.
    link_counts = _read_scalar_ints(physx_sdk, "linkCount")
    assert len(link_counts) == 2
    assert all(count == 2 for count in link_counts)

    # The mixed-base capability the legacy homogeneous-view getter lacked: both base types in one read.
    is_fixed_base = _read_scalar_ints(physx_sdk, "isFixedBase")
    assert len(is_fixed_base) == 2
    assert set(is_fixed_base) == {0, 1}

    # Both articulations are a two-link chain on one revolute joint, so each has one inter-link joint,
    # one DOF and no tendons. Pinning the exact counts (rather than a vacuous >= 0) is what makes a
    # read-plumbing swap -- e.g. jointCount served by the wrong getter -- visible here. The specific
    # jointCount != dofCount separation is covered by the C++ spherical-joint fixture, since this fixture
    # is shared with the tensor-binding tests and stays all-revolute.
    expected_counts = {"dofCount": 1, "jointCount": 1, "fixedTendonCount": 0, "spatialTendonCount": 0}
    for attribute, want in expected_counts.items():
        values = _read_scalar_ints(physx_sdk, attribute)
        assert len(values) == 2
        assert all(value == want for value in values), (attribute, values)


def test_read_empty_when_no_match(physx_sdk):
    """A type with no objects in the scene yields an empty result, not an error."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    # The falling-boxes scene has no particle sets.
    with physx_sdk.read_sync(SimObjectType.PARTICLE_SET, ["points"]) as result:
        assert result.groups == []


def test_sleeping_state_is_host_uint8_and_accepts_tokens(physx_sdk):
    """DirectGPU exposes sleeping-disabled state through the ordinary read surface."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    with physx_sdk.read_sync(SimObjectType.RIGID_BODY, ["isSleeping"]) as result:
        assert result.groups == []
    physx_sdk.step_sync(1.0 / 60.0)

    with physx_sdk.read_sync(SimObjectType.RIGID_BODY, ["isSleeping"]) as result:
        assert len(result.groups) == 1
        group = result.groups[0]
        assert group.object_type == SimObjectType.RIGID_BODY
        assert not group.is_array and not group.is_delete
        assert len(group.tensors) == 1
        values = group.tensors[0]
        assert isinstance(values, wp.array)
        assert values.dtype == wp.uint8
        assert not values.device.is_cuda
        assert _col_width(values) == 1
        assert _col_rows(values) == group.prim_count > 0
        assert values.numpy().reshape(-1).tolist() == [0] * group.prim_count
        attribute_token = group.attribute
        expected = values.numpy().copy()

    physx_sdk.step_sync(1.0 / 60.0)
    assert (values.numpy() == expected).all()
    with physx_sdk.read_tokens_sync(SimObjectType.RIGID_BODY, [attribute_token]) as result:
        assert len(result.groups) == 1
        assert result.groups[0].attribute == attribute_token
        assert result.groups[0].tensors[0].dtype == wp.uint8
        assert (result.groups[0].tensors[0].numpy() == expected).all()


def test_read_tokens_accepts_attribute_token_from_read_group(physx_sdk):
    """An emitted attribute token can be fed directly into the public token overload."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    with physx_sdk.read_sync(
        SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL
    ) as named_result:
        attribute_token = next(group for group in named_result.groups if group.tensors).attribute

    with physx_sdk.read_tokens_sync(
        SimObjectType.RIGID_BODY, [attribute_token], scope=ObjectScope.ALL
    ) as token_result:
        token_group = next(
            group
            for group in token_result.groups
            if group.attribute == attribute_token and group.tensors
        )
        assert isinstance(token_group.tensors[0], wp.array)


# ---------------------------------------------------------------------------
# Prim-set selection (ADR-0025 / REQ-READ-SELECT-001): prim_paths / pattern narrow the
# read and the write to a prim set.
# ---------------------------------------------------------------------------

_CUBE_PATTERN = "/World/Cube*"
# The rigid bodies _CUBE_PATTERN names in boxes_falling_on_groundplane.usda.
_CUBE_PATHS = sorted(f"/World/Cube{i}" for i in range(1, 12))


def _handles_for(stage, paths):
    """Native ovx_primpath_t handles for `paths`. The explicit prim_paths selection takes handles, not
    strings -- a caller gathers these from a read group's prim_list or interns them through ovstage, which
    interning here stands in for."""
    with ovstage.PathDictionary(stage) as dictionary:
        return [dictionary.intern_path(p) for p in paths]


def _selected_read_paths(physx_sdk, stage, object_type, attribute, *, prim_paths=None, pattern=None):
    """Sorted, de-duplicated prim-path strings a (possibly selected) read covers."""
    out = []
    with physx_sdk.read_sync(object_type, [attribute], prim_paths=prim_paths, pattern=pattern) as result:
        with ovstage.PathDictionary(stage) as dictionary:
            for group in result.groups:
                handles = dictionary.get_paths(group.prim_list)
                out.extend(
                    dictionary.path_to_string(h)
                    for h in handles[group.prim_offset : group.prim_offset + group.prim_count]
                )
    return sorted(set(out))


def _selected_read_values(physx_sdk, stage, object_type, attribute, *, prim_paths=None, pattern=None):
    """{prim path -> value row (np.ndarray)} for a (possibly selected) read -- the value-parity oracle."""
    out = {}
    with physx_sdk.read_sync(object_type, [attribute], prim_paths=prim_paths, pattern=pattern) as result:
        with ovstage.PathDictionary(stage) as dictionary:
            for group in result.groups:
                handles = dictionary.get_paths(group.prim_list)
                vals = np.asarray(group.tensors[0].numpy()).reshape(group.prim_count, -1)
                for r in range(group.prim_count):
                    path = dictionary.path_to_string(handles[group.prim_offset + r])
                    out[path] = vals[r].copy()
    return out


def test_read_pattern_matches_tensor_binding(physx_sdk):
    """A pattern-narrowed read selects exactly the prims the glob names in the fixture.

    `/World/Cube*` resolves to the fixture's eleven cubes, Cube1..Cube11, asserted by equal resolved paths.
    A pattern-bound tensor binding resolves the glob through the same runtime PathPatternMatcher and selects
    the same prims.
    """
    stage = attach_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    read_paths = _selected_read_paths(
        physx_sdk, stage, SimObjectType.RIGID_BODY, "position", pattern=_CUBE_PATTERN
    )
    assert read_paths == _CUBE_PATHS

    # The deprecated pattern binding resolves the same prims; this comparison retires with the binding.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        binding = physx_sdk.create_tensor_binding(pattern=_CUBE_PATTERN, tensor_type=TensorType.RIGID_BODY_POSE)
        try:
            assert sorted(binding.prim_paths) == _CUBE_PATHS
        finally:
            binding.destroy()


def test_read_prim_paths_subset(physx_sdk):
    """An explicit prim_paths list reads exactly those prims -- a strict subset of the whole set."""
    stage = attach_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    everything = _selected_read_paths(physx_sdk, stage, SimObjectType.RIGID_BODY, "position")
    assert len(everything) >= 2, "need at least two rigid bodies for a meaningful subset"
    wanted = everything[:2]

    got = _selected_read_paths(
        physx_sdk, stage, SimObjectType.RIGID_BODY, "position", prim_paths=_handles_for(stage, wanted)
    )
    assert got == wanted


def test_read_selection_precedence(physx_sdk):
    """prim_paths wins over pattern when both are given (never combined)."""
    stage = attach_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    everything = _selected_read_paths(physx_sdk, stage, SimObjectType.RIGID_BODY, "position")
    assert everything, "expected at least one rigid body"
    wanted = everything[:1]

    # A pattern that would match everything is ignored because prim_paths is set.
    got = _selected_read_paths(
        physx_sdk, stage, SimObjectType.RIGID_BODY, "position",
        prim_paths=_handles_for(stage, wanted), pattern="/World/*"
    )
    assert got == wanted


def test_read_selection_empty_match(physx_sdk):
    """A selection that matches no prim is a valid empty read, not an error."""
    stage = attach_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    # A native handle interned for a path that is not in the scene: valid handle, but resolving to no live
    # object, so the selection matches nothing.
    with physx_sdk.read_sync(
        SimObjectType.RIGID_BODY, ["position"], prim_paths=_handles_for(stage, ["/World/DoesNotExist"])
    ) as result:
        assert result.groups == []


def test_write_selection_narrows_to_subset(physx_sdk):
    """A prim_paths-narrowed write covers ONLY the selected prims -- the same subset a selected read
    returns. The write inherits the query's selection, so it scatters to exactly those prims and
    leaves the unselected superset untouched: the selected bodies read back the written velocity, and
    every other body is bit-identical to its value before the write."""
    stage = attach_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    before = _selected_read_values(physx_sdk, stage, SimObjectType.RIGID_BODY, "linearVelocity")
    everything = sorted(before)
    assert len(everything) >= 2, "need at least two rigid bodies for a meaningful subset"
    wanted = everything[:2]
    target = np.array([1.0, 2.0, 3.0], dtype=np.float32)

    with ovstage.PathDictionary(stage) as dictionary:
        handles = [dictionary.intern_path(p) for p in wanted]
        with physx_sdk.write_sync(SimObjectType.RIGID_BODY, "linearVelocity", prim_paths=handles) as session:
            write_paths = []
            for group in session.groups:
                write_paths.extend(
                    dictionary.path_to_string(handle)
                    for handle in dictionary.get_paths(group.prim_list)[
                        group.prim_offset : group.prim_offset + group.prim_count
                    ]
                )
                tensor = group.tensors[0]
                block = np.tile(target, (tensor.shape[0], 1)).astype(np.float32)
                tensor.assign(np.ascontiguousarray(block).reshape(tensor.shape))
                cuda_tensor = next((t for t in group.tensors if t.size and t.device.is_cuda), None)
                if cuda_tensor is not None:
                    session.commit_sync(group, cuda_stream=int(wp.get_stream(cuda_tensor.device).cuda_stream or 1))
                else:
                    session.commit_sync(group)
    assert sorted(write_paths) == wanted
    physx_sdk.wait_all()

    after = _selected_read_values(physx_sdk, stage, SimObjectType.RIGID_BODY, "linearVelocity")
    assert set(after) == set(before)
    for path in everything:
        if path in wanted:
            np.testing.assert_allclose(after[path], target, rtol=1e-5, atol=1e-5, err_msg=path)
        else:
            assert np.array_equal(after[path], before[path]), f"unselected {path} changed"


def test_selection_rejects_active_scope(physx_sdk):
    """A selected query has no scope: it covers every selected object, sleeping ones included, so a
    selection combined with scope=ACTIVE is rejected rather than quietly widened to ALL (the C
    ovphysx_query_selection cannot express it). read_sync, read_tokens_sync and write_sync share the check. An empty
    prim_paths list with no pattern is not a selection, so ACTIVE still opens the whole-set ACTIVE query."""
    stage = attach_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()
    handles = _handles_for(stage, _selected_read_paths(physx_sdk, stage, SimObjectType.RIGID_BODY, "position")[:1])
    rigid, active = SimObjectType.RIGID_BODY, ObjectScope.ACTIVE

    with pytest.raises(ValueError, match="cannot be combined with scope=ACTIVE"):
        with physx_sdk.read_sync(rigid, ["position"], scope=active, prim_paths=handles):
            pass
    with pytest.raises(ValueError, match="cannot be combined with scope=ACTIVE"):
        with physx_sdk.read_sync(rigid, ["position"], scope=active, pattern=_CUBE_PATTERN):
            pass
    with pytest.raises(ValueError, match="cannot be combined with scope=ACTIVE"):
        with physx_sdk.write_sync(rigid, "position", scope=active, prim_paths=handles):
            pass

    with physx_sdk.read_sync(rigid, ["position"], scope=active, prim_paths=[]):
        pass


def test_read_empty_prim_paths_falls_back_to_pattern(physx_sdk):
    """An empty prim_paths list must fall back to the pattern, not become an invalid empty selection.

    prim_paths=[] means "no explicit paths": the native contract uses the pattern when
    prim_paths_count == 0. Regression -- the marshalling used to clear the pattern for any non-None
    prim_paths, turning a valid glob query into an empty selection that the C layer rejects.
    """
    stage = attach_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    pattern_only = _selected_read_paths(
        physx_sdk, stage, SimObjectType.RIGID_BODY, "position", pattern=_CUBE_PATTERN
    )
    assert pattern_only, "the glob should match some rigid bodies"

    empty_list = _selected_read_paths(
        physx_sdk, stage, SimObjectType.RIGID_BODY, "position", prim_paths=[], pattern=_CUBE_PATTERN
    )
    assert empty_list == pattern_only


def test_read_empty_pattern_is_rejected(physx_sdk):
    """An empty-string pattern is a caller error, rejected up front to match the C API (which returns
    INVALID_ARGUMENT), not silently treated as the whole set. pattern=None is the whole-set request."""
    attach_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    with pytest.raises(ValueError, match="pattern must be non-empty"):
        with physx_sdk.read_sync(SimObjectType.RIGID_BODY, ["position"], pattern=""):
            pass


def test_read_selection_value_parity(physx_sdk):
    """A selected read returns the same VALUES as the whole read for the selected prims.

    Path/count parity is not enough: a mis-index after selection compaction would emit the right prims
    with the wrong values. This compares element-wise against the whole read's values.
    """
    stage = attach_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    whole = _selected_read_values(physx_sdk, stage, SimObjectType.RIGID_BODY, "position")
    assert len(whole) >= 3
    wanted = sorted(whole)[:2]

    selected = _selected_read_values(
        physx_sdk, stage, SimObjectType.RIGID_BODY, "position", prim_paths=_handles_for(stage, wanted)
    )
    assert sorted(selected) == wanted
    for path in wanted:
        np.testing.assert_allclose(selected[path], whole[path], rtol=1e-5, atol=1e-5)


def test_read_selection_instancer(physx_sdk):
    """A selected read over a point instancer returns ONLY that instancer's array, with correct values.

    The instancer is a distinct array-group read path; the fixture also has a standalone rigid body, so
    selecting the instancer must drop the standalone AND keep the instance array's values.
    """
    stage = attach_usd_with_ovstage(physx_sdk, data_path("ovstage_output_instancer.usda"))
    for _ in range(10):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    whole = _selected_read_values(physx_sdk, stage, SimObjectType.RIGID_BODY, "position")
    assert "/World/Instances" in whole
    assert "/World/Fixed" in whole

    selected = _selected_read_values(
        physx_sdk, stage, SimObjectType.RIGID_BODY, "position",
        prim_paths=_handles_for(stage, ["/World/Instances"])
    )
    assert set(selected) == {"/World/Instances"}  # the standalone body is not emitted
    np.testing.assert_allclose(selected["/World/Instances"], whole["/World/Instances"], rtol=1e-5, atol=1e-5)


def test_write_selection_instancer(physx_sdk):
    """A selected write over one point instancer scatters only its instances; the standalone body is
    untouched (the planInstancerGroup gate)."""
    stage = attach_usd_with_ovstage(physx_sdk, data_path("ovstage_output_instancer.usda"))
    for _ in range(10):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    before = _selected_read_values(physx_sdk, stage, SimObjectType.RIGID_BODY, "position")
    assert {"/World/Instances", "/World/Fixed"} <= set(before)

    covered = []
    with ovstage.PathDictionary(stage) as dictionary:
        instancer_handles = [dictionary.intern_path("/World/Instances")]
        with physx_sdk.write_sync(SimObjectType.RIGID_BODY, "position", prim_paths=instancer_handles) as session:
            for group in session.groups:
                handles = dictionary.get_paths(group.prim_list)
                covered.extend(
                    dictionary.path_to_string(h)
                    for h in handles[group.prim_offset : group.prim_offset + group.prim_count]
                )
                # Shift every instance value +3 in the instancer-local frame (the frame the read publishes),
                # so the read-back is a clean displacement. assign() writes the aliased native column
                # device-or-host; a plain numpy() edit would not.
                tensor = group.tensors[0]
                target = (before["/World/Instances"].reshape(-1) + 3.0).astype(np.float32)
                tensor.assign(np.ascontiguousarray(target).reshape(tensor.shape))
                cuda_t = next((x for x in group.tensors if x.size and x.device.is_cuda), None)
                if cuda_t is not None:
                    session.commit_sync(group, cuda_stream=int(wp.get_stream(cuda_t.device).cuda_stream or 1))
                else:
                    session.commit_sync(group)
    assert covered == ["/World/Instances"]  # only the selected instancer's group

    after = _selected_read_values(physx_sdk, stage, SimObjectType.RIGID_BODY, "position")
    np.testing.assert_allclose(
        after["/World/Instances"], before["/World/Instances"].reshape(-1) + 3.0, rtol=1e-4, atol=1e-4
    )
    np.testing.assert_allclose(after["/World/Fixed"], before["/World/Fixed"], rtol=1e-5, atol=1e-5)


def test_selected_write_subsumes_legacy_indexed_write(physx_sdk):
    """The selected session write lands the state an indexed write of the same subset lands
    (REQ-CAPI-WRITE-001 AC-14): the selected bodies take the written linear velocity and every other body
    keeps its own, body for body. The deprecated indexed tensor-binding write of the same subset, from the
    same stepped baseline (re-attach resets it), lands the same state.
    """
    target = np.array([1.0, 2.0, 3.0], dtype=np.float32)

    def stepped():
        stage = attach_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
        for _ in range(3):
            physx_sdk.step(1.0 / 60.0)
        physx_sdk.wait_all()
        return stage

    stage = stepped()

    before = _selected_read_values(physx_sdk, stage, SimObjectType.RIGID_BODY, "linearVelocity")
    subset = sorted(before)[:2]
    # The indexed write's outcome: the listed rows hold the target, every other row is as it was.
    expected = {path: (target if path in subset else before[path]) for path in before}

    with physx_sdk.write_sync(
        SimObjectType.RIGID_BODY, "linearVelocity", prim_paths=_handles_for(stage, subset)
    ) as session:
        for group in session.groups:
            tensor = group.tensors[0]
            block = np.tile(target, (tensor.shape[0], 1)).astype(np.float32)
            tensor.assign(np.ascontiguousarray(block).reshape(tensor.shape))
            cuda_t = next((x for x in group.tensors if x.size and x.device.is_cuda), None)
            if cuda_t is not None:
                session.commit_sync(group, cuda_stream=int(wp.get_stream(cuda_t.device).cuda_stream or 1))
            else:
                session.commit_sync(group)
    physx_sdk.wait_all()
    selected = _selected_read_values(physx_sdk, stage, SimObjectType.RIGID_BODY, "linearVelocity")

    assert set(expected) == set(selected)
    for path in expected:
        np.testing.assert_allclose(selected[path], expected[path], rtol=1e-4, atol=1e-4)

    # The legacy indexed write over the same subset: full [N, 6] tensor, only the index set applied. This
    # comparison retires with the binding.
    stage = stepped()
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        binding = physx_sdk.create_tensor_binding(pattern=_CUBE_PATTERN, tensor_type=TensorType.RIGID_BODY_VELOCITY)
        try:
            order = list(binding.prim_paths)
            vel = np.zeros(binding.shape, dtype=np.float32)  # [N, 6] = linear (xyz) + angular (xyz)
            binding.read(vel)  # keep current angular; only the linear half changes
            idx = np.array([order.index(p) for p in subset], dtype=np.int32)
            for i in idx:
                vel[int(i)][0:3] = target
            binding.write(vel, indices=idx)
        finally:
            binding.destroy()
    physx_sdk.wait_all()
    legacy = _selected_read_values(physx_sdk, stage, SimObjectType.RIGID_BODY, "linearVelocity")

    assert set(legacy) == set(selected)
    for path in legacy:
        np.testing.assert_allclose(selected[path], legacy[path], rtol=1e-4, atol=1e-4)
