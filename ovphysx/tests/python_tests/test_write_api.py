# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for the app -> physics write API (ADR-0012): PhysX.write over an
ovstage-attached scene.

The oracle is the READ. Every value assertion writes a column and reads it back through
PhysX.read, because that is the check that pins the whole contract at once: the prim
ordering, the reframe, and the round trip through the backend. Asserting on the write
buffer alone would pass even if nothing reached the solver.
"""

# @implements REQ-CAPI-WRITE-001
# @covers AC-4 AC-5 AC-9
# @maps_to TEST-CAPI-WRITE-001
# @implements REQ-INPUT-DEVICE-001
# @covers AC-1 AC-4 AC-5
# @maps_to TEST-INPUT-DEVICE-001

import ctypes
import weakref
from types import SimpleNamespace

import numpy as np
import ovstage
import pytest
import warp as wp
from ovphysx.types import ObjectScope, SimObjectType
from test_utils import attach_usd_with_ovstage, data_path, load_usd_with_ovstage


def _to_host(t) -> np.ndarray:
    """A Warp column as host NumPy, whichever residency it came back as.

    Warp does any device copy: no torch, and no DLPack
    round trip through a framework that may not be installed.
    """
    assert isinstance(t, wp.array)
    return t.numpy()


def _fill(t, block) -> None:
    """Write `block` into a native-device Warp column.

    A non-empty column ALIASES the runtime's storage, so a copy here would be filled and
    dropped and the commit would publish the untouched original.
    """
    assert isinstance(t, wp.array)
    t.assign(np.ascontiguousarray(block).reshape(t.shape))


def _commit_after_fill(session, group) -> None:
    cuda_tensor = next((t for t in group.tensors if t.size and t.device.is_cuda), None)
    if cuda_tensor is None:
        session.commit(group)
        return
    stream = wp.get_stream(cuda_tensor.device)
    session.commit(group, cuda_stream=int(stream.cuda_stream or 1))


def _read_positions(physx, attr="position"):
    """path-independent: attribute values in prim order, as one host array."""
    with physx.read(SimObjectType.RIGID_BODY, [attr]) as result:
        assert result.groups, "expected at least one group"
        return np.concatenate([_to_host(g.tensors[0]).reshape(g.prim_count, -1) for g in result.groups])


def test_later_group_conversion_failure_drops_aliases_before_release():
    """A failed second group must not release storage under the first group's alias."""
    from ovphysx import _bindings
    from ovphysx.api import PhysX
    from ovphysx.dlpack import DLTensor
    from ovphysx.types import ApiStatus

    tensor = DLTensor()
    group = _bindings.ovstage_map_group_t()
    group.data.tensor_count = 1
    group.data.tensors = ctypes.pointer(tensor)
    alias_ref = None
    fetches = 0
    release_observations = []

    class Alias:
        pass

    class FakeLib:
        def ovphysx_query(self, handle, object_type, scope, out_query):
            out_query._obj.value = 1
            return SimpleNamespace(status=ApiStatus.SUCCESS)

        def ovphysx_write(self, handle, query, attribute, out_write):
            out_write._obj.value = 2
            return SimpleNamespace(status=ApiStatus.SUCCESS)

        def ovphysx_fetch_write_next(self, handle, write, out_group):
            nonlocal fetches
            fetches += 1
            out_group._obj.contents = group
            return SimpleNamespace(status=ApiStatus.SUCCESS)

        def ovphysx_release_write(self, handle, write):
            release_observations.append(("write", alias_ref() is None))

        def ovphysx_release_query(self, handle, query):
            release_observations.append(("query", alias_ref() is None))

    sdk = object.__new__(PhysX)
    sdk._lib = FakeLib()
    sdk._omni_physx_sdk_handle = SimpleNamespace(value=0x1234)
    sdk._check_valid = lambda: None
    sdk._drain_pending_read_releases = lambda: None
    sdk._get_last_error = lambda: "unused"

    def fake_convert(t, wp_mod, on_release):
        nonlocal alias_ref
        if alias_ref is None:
            alias = Alias()
            alias_ref = weakref.ref(alias)
            return alias
        raise RuntimeError("forced conversion failure on the second group")

    sdk._dltensor_to_warp_array = fake_convert

    with pytest.raises(RuntimeError, match="forced conversion failure"):
        sdk.write(SimObjectType.RIGID_BODY, "position")

    assert fetches == 2
    assert release_observations == [("write", True), ("query", True)]


def test_write_position_round_trips(physx_sdk):
    """A written position column comes back from the read unchanged (REQ-CAPI-WRITE-001)."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.step_sync(1.0 / 60.0)

    before = _read_positions(physx_sdk)
    assert before.shape[1] == 3

    # Values no simulation would produce, so a dropped or duplicated row is visible.
    target = np.arange(before.size, dtype=np.float32).reshape(before.shape) + 100.0

    with physx_sdk.write(SimObjectType.RIGID_BODY, "position") as w:
        assert w.groups, "expected at least one writable group"
        row = 0
        for g in w.groups:
            n = g.prim_count
            _fill(g.tensors[0], target[row : row + n])
            _commit_after_fill(w, g)
            row += n
        assert row == before.shape[0]

    after = _read_positions(physx_sdk)
    np.testing.assert_allclose(after, target, rtol=0, atol=1e-3)


def test_write_leaves_orientation_alone(physx_sdk):
    """position and orientation share one transform, so writing one must preserve the other."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(20):  # accumulate a non-identity orientation worth preserving
        physx_sdk.step_sync(1.0 / 60.0)

    quat_before = _read_positions(physx_sdk, "orientation")

    with physx_sdk.write(SimObjectType.RIGID_BODY, "position") as w:
        for g in w.groups:
            _fill(g.tensors[0], np.full((g.prim_count, 3), 42.0, dtype=np.float32))
            _commit_after_fill(w, g)

    quat_after = _read_positions(physx_sdk, "orientation")
    np.testing.assert_allclose(quat_after, quat_before, rtol=0, atol=1e-3)


def test_uncommitted_group_publishes_nothing(physx_sdk):
    """Leaving a filled group uncommitted discards it rather than leaking it to the solver."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.step_sync(1.0 / 60.0)

    before = _read_positions(physx_sdk)

    with physx_sdk.write(SimObjectType.RIGID_BODY, "position") as w:
        for g in w.groups:
            _fill(g.tensors[0], np.full((g.prim_count, 3), -12345.0, dtype=np.float32))
            # deliberately NOT committed

    after = _read_positions(physx_sdk)
    assert not np.any(np.isclose(after, -12345.0)), "an abandoned group reached the solver"
    np.testing.assert_allclose(after, before, rtol=0, atol=1e-6)


def test_double_commit_raises(physx_sdk):
    """Commit is the mutation, so a second commit of one group must not report success."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.step_sync(1.0 / 60.0)

    with physx_sdk.write(SimObjectType.RIGID_BODY, "position") as w:
        g = w.groups[0]
        w.commit(g)
        with pytest.raises(RuntimeError):
            w.commit(g)


def test_unwritable_attribute_raises(physx_sdk):
    """A name the type does not accept fails at open, not silently at commit."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.step_sync(1.0 / 60.0)

    with pytest.raises(RuntimeError):
        physx_sdk.write(SimObjectType.RIGID_BODY, "notAnAttribute")
    # Readable on other types, but rigid bodies do not produce it.
    with pytest.raises(RuntimeError):
        physx_sdk.write(SimObjectType.RIGID_BODY, "points")


def test_write_groups_match_the_read(physx_sdk):
    """The write covers the same prims, in the same order, as a read over the same query.

    This is what makes the round trip a no-repack edit: if the two enumerations could
    differ, a caller feeding a read column straight into a write group would silently
    scatter values onto the wrong prims.
    """
    stage = attach_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.step_sync(1.0 / 60.0)

    with ovstage.PathDictionary(stage) as paths:
        with physx_sdk.read(SimObjectType.RIGID_BODY, ["position"]) as r:
            read_shape = [
                (
                    tuple(paths.get_paths(g.prim_list)[g.prim_offset : g.prim_offset + g.prim_count]),
                    tuple(str(t.device) for t in g.tensors),
                )
                for g in r.groups
            ]

        with physx_sdk.write(SimObjectType.RIGID_BODY, "position") as w:
            assert all(isinstance(t, wp.array) for g in w.groups for t in g.tensors)
            assert any(t.device.is_cuda for g in w.groups for t in g.tensors)
            write_shape = [
                (
                    tuple(paths.get_paths(g.prim_list)[g.prim_offset : g.prim_offset + g.prim_count]),
                    tuple(str(t.device) for t in g.tensors),
                )
                for g in w.groups
            ]

    assert write_shape == read_shape


# ---------------------------------------------------------------------------
# Articulation joints.
#
# These are BINDING tests, not correctness tests. They prove the C and Python layers carry an
# array group end to end: N tensors per group, the right selector, the link refusal surfacing as
# an exception. They key by tensor INDEX, so they would not catch the read and write disagreeing
# about prim ORDER.
#
# That per-prim guarantee, and the angular-scale inversion in the joint path, are asserted in
# TestOvstageWriteScatter.cpp, where the values are compared per prim path and a flipped scale
# is shown to read back 57.3^2 times wrong.
# ---------------------------------------------------------------------------


def _read_joint_state(physx, attr="jointPosition"):
    """path -> that joint's per-axis values.

    Joint groups are ARRAY groups: one tensor PER PRIM, so the value for prim i is tensors[i], not a
    slice of tensors[0]. Reading them the fixed-group way would alias every joint onto one tensor and
    still produce plausible numbers.
    """
    out = {}
    with physx.read(SimObjectType.ARTICULATION_JOINT, [attr]) as result:
        for g in result.groups:
            assert len(g.tensors) == g.prim_count, "array group: one tensor per prim"
            for i, t in enumerate(g.tensors):
                out[i] = _to_host(t).reshape(-1)
    return out


def test_joint_position_round_trips(physx_sdk):
    """A written joint column survives the round trip through the Python and C layers.

    CartPole carries a revolute axis, so the value does cross the record scale, but the assertion
    here is that the plumbing preserves it, not that the scale is inverted correctly. The C++ suite
    owns that, with a flipped-scale case.
    """
    load_usd_with_ovstage(physx_sdk, data_path("CartPole.usda"))
    for _ in range(10):
        physx_sdk.step_sync(1.0 / 60.0)

    # The write derives its joint set from the cache the read fills, so read first.
    before = _read_joint_state(physx_sdk)
    assert before, "expected at least one joint"

    target = {}
    with physx_sdk.write(SimObjectType.ARTICULATION_JOINT, "jointPosition") as w:
        assert w.groups, "expected a writable joint group"
        for g in w.groups:
            assert len(g.tensors) == g.prim_count
            for i, t in enumerate(g.tensors):
                assert isinstance(t, wp.array)
                n = int(t.size)
                block = np.full(n, 0.25 * (i + 1), dtype=np.float32)
                _fill(t, block)
                target[i] = block
            _commit_after_fill(w, g)

    after = _read_joint_state(physx_sdk)
    assert set(after) == set(target)
    for i, want in target.items():
        np.testing.assert_allclose(after[i], want, rtol=0, atol=1e-2)


def test_articulation_link_attributes_are_refused_by_name(physx_sdk):
    """Links refuse per ATTRIBUTE, not per type.

    PhysX exposes no link POSE write, but a PxArticulationLink IS a PxRigidBody, so its mass and the
    rest of the per-body set are writable. Both halves are asserted here for the same reason the C++
    case asserts both: the refusals alone would also pass under a type-level refusal.
    """
    load_usd_with_ovstage(physx_sdk, data_path("CartPole.usda"))
    physx_sdk.step_sync(1.0 / 60.0)

    with pytest.raises(RuntimeError):
        physx_sdk.write(SimObjectType.ARTICULATION_LINK, "position")

    with physx_sdk.write(SimObjectType.ARTICULATION_LINK, "mass") as w:
        assert w.groups, "link mass is writable -- a link is a rigid body"
        tensors = [t for g in w.groups for t in g.tensors]
        assert tensors and all(t.size > 0 for t in tensors)
        assert all(isinstance(t, wp.array) and t.device.is_cpu for t in tensors)


# @implements REQ-INPUT-COVERAGE-001
# @covers AC-10
# @maps_to TEST-INPUT-COVERAGE-003
def test_joint_limit_on_a_free_axis_is_refused(physx_sdk):
    """jointLimit is CONDITIONAL, not unconditionally writable.

    A limit interval exists only on an eLIMITED axis, and PhysX refuses setMotion() on an in-scene
    articulation, so a FINITE limit aimed at a FREE axis cannot land: the commit is refused and
    NOTHING is written, not even the limited axis the write could accept (REQ-INPUT-COVERAGE-001
    AC-10). CartPole's poleJoint is a free revolute axis and cartJoint is a limited prismatic axis.
    This mirrors the C case in test_joint_limit_conditional.cpp. The refusal surfaces as an exception.
    """
    load_usd_with_ovstage(physx_sdk, data_path("CartPole.usda"))
    for _ in range(3):
        physx_sdk.step_sync(1.0 / 60.0)

    # The write derives its joint set from the cache the read fills, so read first.
    before = _read_joint_state(physx_sdk, "jointLimit")
    assert before, "expected at least one joint"
    # A free axis reads the +/-FLT_MAX unlimited sentinel. The fixture must expose one.
    unlimited = np.finfo(np.float32).max
    assert any(np.any(v >= unlimited) for v in before.values()), "CartPole must expose a free axis"

    # A finite limit on every DOF necessarily hits the free axis, so the commit is refused.
    with pytest.raises(RuntimeError):
        with physx_sdk.write(SimObjectType.ARTICULATION_JOINT, "jointLimit") as w:
            for g in w.groups:
                for t in g.tensors:
                    n = int(_to_host(t).reshape(-1).shape[0]) if not isinstance(t, np.ndarray) else t.size
                    _fill(t, np.tile(np.array([-30.0, 45.0], dtype=np.float32), n // 2))
                w.commit(g)

    # Nothing landed: every axis still reads its baseline (a refusal that half-applied would differ).
    after = _read_joint_state(physx_sdk, "jointLimit")
    assert set(after) == set(before)
    for i in before:
        np.testing.assert_array_equal(after[i], before[i])

    # The joint type IS writable on the same scene, which is what makes the refusal a statement
    # about a handful of ATTRIBUTES rather than about articulations.
    _read_joint_state(physx_sdk)
    with physx_sdk.write(SimObjectType.ARTICULATION_JOINT, "jointPosition") as w:
        assert w.groups


# ---------------------------------------------------------------------------
# Articulation root (ADR-0012).
#
# Binding tests, like the joint ones above: the C++ suite owns whether a written root pose actually
# moves the articulation, on a FLOATING-base fixture with gravity off. CartPole is fixed-base, so a
# value assertion here would be asserting against a base PhysX is entitled to hold in place.
# ---------------------------------------------------------------------------


def test_articulation_root_columns_have_the_right_shape(physx_sdk):
    """The four root names open and carry a FIXED group of the right width.

    Deliberately does NOT fill or commit: the shape contract is what the binding owns, and
    it holds at whatever residency the column came back as. The fill-and-commit half is the
    next case.
    """
    load_usd_with_ovstage(physx_sdk, data_path("CartPole.usda"))
    physx_sdk.step_sync(1.0 / 60.0)

    for attr, comp in (
        ("rootPosition", 3),
        ("rootOrientation", 4),
        ("rootLinearVelocity", 3),
        ("rootAngularVelocity", 3),
    ):
        with physx_sdk.write(SimObjectType.ARTICULATION, attr) as w:
            assert w.groups, f"expected a writable articulation group for {attr}"
            for g in w.groups:
                # FIXED shape: one tensor stacking every articulation, not one tensor per prim.
                assert len(g.tensors) == 1, f"{attr}: articulation groups are fixed, not array"
                assert g.prim_count >= 1
                t = g.tensors[0]
                assert isinstance(t, wp.array)
                assert t.shape == (g.prim_count, comp), f"{attr}: expected {comp} components"


def test_articulation_root_column_commits(physx_sdk):
    """Filling and committing a root column carries end to end through the Python and C layers.

    An IDENTITY orientation, so a green run leaves the scene where it found it: the C++ suite owns
    whether a root write moves the articulation, on a floating-base fixture with gravity off. CartPole
    is fixed-base, so a displacement assertion here would be asserting against a base PhysX is
    entitled to hold in place.
    """
    load_usd_with_ovstage(physx_sdk, data_path("CartPole.usda"))
    physx_sdk.step_sync(1.0 / 60.0)

    with physx_sdk.write(SimObjectType.ARTICULATION, "rootOrientation") as w:
        assert w.groups
        for g in w.groups:
            block = np.zeros((g.prim_count, 4), dtype=np.float32)
            block[:, 3] = 1.0
            _fill(g.tensors[0], block)
            _commit_after_fill(w, g)


def test_articulation_refuses_names_it_does_not_serve(physx_sdk):
    """A name only has to be unique WITHIN a type, so the type has to actually enforce its list."""
    load_usd_with_ovstage(physx_sdk, data_path("CartPole.usda"))
    physx_sdk.step_sync(1.0 / 60.0)

    # "position" is refused too: it is the RIGID spelling, and the articulation takes root* names.
    for attr in ("jointPosition", "mass", "points", "position"):
        with pytest.raises(RuntimeError):
            physx_sdk.write(SimObjectType.ARTICULATION, attr)
