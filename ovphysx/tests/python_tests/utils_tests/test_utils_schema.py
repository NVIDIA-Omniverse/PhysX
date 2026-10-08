# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Schema registry introspection and API-schema property snapshot/restore.

Two cases are not introspection. `get_world_position` is called nowhere else, so
AC-4's world-position clause is claimed here rather than in
test_utils_authoring.py; and `codeless`'s diagnostics are claimed here because
their unregistered-schemas half needs the child interpreters the AC-14 cases at
the bottom of this file already own. The refusal cases sit beside them: telling
USD's own refusal apart from an unknown identifier is the other half of the same
diagnosis.
"""

# @implements REQ-PYTHON-UTILS-001
# @covers AC-3 AC-4 AC-10 AC-12 AC-14

import json
import os
import subprocess
import sys

import pytest

pytest.importorskip("pxr", reason="ovphysx.utils requires a caller-supplied USD runtime")

from pxr import Gf, Sdf, Usd, UsdPhysics  # noqa: E402

from ovphysx import utils as physicsUtils  # noqa: E402
from ovphysx.utils import codeless, schema  # noqa: E402


def test_type_resolution_and_property_names(codeless_schemas):
    assert schema.get_tf_type_compatible("PhysxRigidBodyAPI")
    assert schema.get_tf_type_compatible(UsdPhysics.RigidBodyAPI)

    names = schema.get_schema_property_names("PhysxRigidBodyAPI")
    assert "physxRigidBody:disableGravity" in names

    # Inherited properties compose through the prim definition, which is why
    # schema.py reads Usd.SchemaRegistry rather than GetSchemaAttributeNames.
    revolute = schema.get_schema_property_names("PhysicsRevoluteJoint")
    assert "physics:body0" in revolute
    assert "physics:lowerLimit" in revolute

    assert schema.get_schema_prim_def("PhysxRigidBodyAPI")

    attributes = dict(schema.get_schema_attribute("PhysxRigidBodyAPI"))
    assert "physxRigidBody:disableGravity" in attributes
    assert attributes["physxRigidBody:disableGravity"].typeName == Sdf.ValueTypeNames.Bool

    relationships = dict(schema.get_schema_relationship(UsdPhysics.FilteredPairsAPI))
    assert "physics:filteredPairs" in relationships


def test_has_schema_matches_by_name(stage):
    prim = physicsUtils.add_rigid_box(stage, "/World/box")
    codeless.apply_api(prim, "PhysxRigidBodyAPI")
    assert schema.has_schema(prim, "PhysxRigidBodyAPI")
    assert not schema.has_schema(prim, "PhysxNoSuchAPI")


def test_remove_api_no_ops_but_reports_an_unknown_identifier(stage):
    """`remove_api` names the cause the way `apply_api` does (REQ AC-10).

    The no-op half matters as much as the raising half: `remove_collider` strips
    every approximation API `set_collider` can apply without asking which one is
    present, so a never-applied API has to stay a success.
    """
    prim = physicsUtils.add_rigid_box(stage, "/World/box")
    codeless.apply_api(prim, "PhysxRigidBodyAPI")

    assert codeless.remove_api(prim, "PhysxRigidBodyAPI")
    assert "PhysxRigidBodyAPI" not in prim.GetAppliedSchemas()
    assert codeless.remove_api(prim, "PhysxRigidBodyAPI")
    assert codeless.remove_api(prim, "PhysxSphereFillCollisionAPI")
    assert codeless.remove_api(prim, "PhysxCookedDataAPI", "convexHull")

    with pytest.raises(codeless.CodelessSchemaError) as excinfo:
        codeless.remove_api(prim, "PhysxNoSuchAPI")
    assert "does not name a known API schema" in str(excinfo.value)


def test_a_refused_application_is_not_reported_as_a_bad_identifier(stage):
    """USD's own refusal takes the bool path AC-10 documents for it.

    `Usd.Prim.ApplyAPI` and `RemoveAPI` raise one `Tf.ErrorException` for two
    unrelated things: an identifier the registry cannot resolve, and an
    operation on a schema it resolved and then refused to author. A layer that
    is not editable is the second, and only the registry tells them apart.
    """
    prim = physicsUtils.add_rigid_box(stage, "/World/box")
    codeless.apply_api(prim, "PhysxRigidBodyAPI")
    stage.GetRootLayer().SetPermissionToEdit(False)

    assert codeless.try_apply_api(prim, "PhysxCookedDataAPI", "convexHull") is False
    assert codeless.remove_api(prim, "PhysxRigidBodyAPI") is False
    assert "PhysxRigidBodyAPI" in prim.GetAppliedSchemas()

    # The raising half still names the cause, and the two causes stay distinct:
    # a refusal reports the layer, an unknown identifier reports the spelling.
    with pytest.raises(codeless.CodelessSchemaError) as excinfo:
        codeless.apply_api(prim, "PhysxCookedDataAPI", "sdf")
    assert "not editable" in str(excinfo.value)
    assert "does not name a known API schema" not in str(excinfo.value)

    with pytest.raises(codeless.CodelessSchemaError) as excinfo:
        codeless.try_apply_api(prim, "PhysxNoSuchAPI")
    assert "does not name a known API schema" in str(excinfo.value)


@pytest.mark.parametrize(
    "identifier, instance_name, expected",
    (
        ("PhysxRigidBodyAPI", "oops", "single-apply"),
        ("PhysxCookedDataAPI", None, "multiple-apply"),
        ("PhysxCookedDataAPI", "", "not an instance name"),
    ),
)
def test_an_instance_name_contradicting_the_schema_is_reported(
    stage, identifier, instance_name, expected
):
    """A known identifier can still make a call USD cannot resolve (REQ AC-10).

    USD raises the error it raises for an unknown identifier, so the registry
    has to answer for the pair rather than the identifier alone. Reporting a
    mistake in the call as a refusal would hide it behind a bool.
    """
    prim = physicsUtils.add_rigid_box(stage, "/World/box")

    for call in (codeless.try_apply_api, codeless.remove_api):
        with pytest.raises(codeless.CodelessSchemaError) as excinfo:
            call(prim, identifier, instance_name)
        assert expected in str(excinfo.value), str(excinfo.value)
        assert "does not name a known API schema" not in str(excinfo.value)


def test_a_refused_property_write_is_not_reported_as_a_write(stage):
    """A write USD refuses fails the call, whichever way USD reports it.

    `Usd.Attribute.Set` and `Usd.Relationship.SetTargets` answer `False` or
    raise, and both leave the property unauthored. Reporting success would hand
    a caller a stage it does not have.
    """
    prim = physicsUtils.add_rigid_box(stage, "/World/box")
    codeless.apply_api(prim, "PhysxRigidBodyAPI")
    UsdPhysics.FilteredPairsAPI.Apply(prim)
    stage.GetRootLayer().SetPermissionToEdit(False)

    with pytest.raises(codeless.CodelessSchemaError) as excinfo:
        codeless.set_attr(prim, "physxRigidBody:disableGravity", True)
    assert "not editable" in str(excinfo.value)
    assert not prim.GetAttribute("physxRigidBody:disableGravity").HasAuthoredValue()

    with pytest.raises(codeless.CodelessSchemaError) as excinfo:
        codeless.set_rel(prim, "physics:filteredPairs", "/World")
    assert "not editable" in str(excinfo.value)
    assert not prim.GetRelationship("physics:filteredPairs").HasAuthoredTargets()


def test_successful_property_writes_do_not_format_values(stage):
    """Successful writes must not pay for an unused diagnostic representation."""

    class UnprintableFloat(float):
        def __repr__(self):
            raise AssertionError("the attribute value was formatted")

    class UnprintableString(str):
        def __repr__(self):
            raise AssertionError("the relationship target was formatted")

    prim = physicsUtils.add_rigid_box(stage, "/World/box")
    codeless.apply_api(prim, "PhysxRigidBodyAPI")
    UsdPhysics.FilteredPairsAPI.Apply(prim)

    codeless.set_attr(
        prim,
        "physxRigidBody:sleepThreshold",
        UnprintableFloat(0.125),
    )
    codeless.set_rel(
        prim,
        "physics:filteredPairs",
        UnprintableString("/World/target"),
    )

    assert codeless.get_attr(prim, "physxRigidBody:sleepThreshold").Get() == pytest.approx(
        0.125
    )
    assert prim.GetRelationship("physics:filteredPairs").GetTargets() == [Sdf.Path("/World/target")]


@pytest.mark.parametrize(
    "operation",
    (
        pytest.param(
            lambda prim: codeless.apply_api(prim, "PhysxRigidBodyAPI"),
            id="apply-api",
        ),
        pytest.param(
            lambda prim: codeless.try_apply_api(prim, "PhysxRigidBodyAPI"),
            id="try-apply-api",
        ),
        pytest.param(
            lambda prim: codeless.remove_api(prim, "PhysxRigidBodyAPI"),
            id="remove-api",
        ),
        pytest.param(
            lambda prim: codeless.get_attr(prim, "physxRigidBody:disableGravity"),
            id="get-attr",
        ),
        pytest.param(
            lambda prim: codeless.set_attr(prim, "physxRigidBody:disableGravity", True),
            id="set-attr",
        ),
        pytest.param(
            lambda prim: codeless.set_attrs(prim, {"physxRigidBody:disableGravity": None}),
            id="set-attrs",
        ),
        pytest.param(
            lambda prim: codeless.set_rel(prim, "physics:filteredPairs", "/World"),
            id="set-rel",
        ),
    ),
)
def test_codeless_authoring_rejects_an_invalid_prim(stage, operation):
    prim = stage.GetPrimAtPath("/does/not/exist")
    assert not prim

    with pytest.raises(codeless.CodelessSchemaError, match="prim is not valid on this stage"):
        operation(prim)


def test_a_write_answering_false_is_refused_like_one_that_raises(stage, monkeypatch):
    """The other half of USD's refusal contract, which no layer state reaches.

    A non-editable layer makes both writes raise, so the `False` return is
    covered here instead. Patching USD's own methods is what it takes: nothing
    in the subpackage sits between the helper and the write.
    """
    prim = physicsUtils.add_rigid_box(stage, "/World/box")
    codeless.apply_api(prim, "PhysxRigidBodyAPI")
    UsdPhysics.FilteredPairsAPI.Apply(prim)

    monkeypatch.setattr(Usd.Attribute, "Set", lambda *args, **kwargs: False)
    monkeypatch.setattr(Usd.Relationship, "SetTargets", lambda *args, **kwargs: False)

    with pytest.raises(codeless.CodelessSchemaError):
        codeless.set_attr(prim, "physxRigidBody:disableGravity", True)
    with pytest.raises(codeless.CodelessSchemaError):
        codeless.set_rel(prim, "physics:filteredPairs", "/World")


def test_applied_api_reporting(stage):
    root = stage.DefinePrim("/World", "Xform")
    prim = physicsUtils.add_rigid_box(stage, "/World/box")
    codeless.apply_api(prim, "PhysxRigidBodyAPI")

    assert schema.has_schema(prim, "PhysxRigidBodyAPI")
    assert not schema.has_schema(prim, "PhysxCollisionAPI")
    assert schema.descendant_has_api("PhysxRigidBodyAPI", root)
    assert schema.ancestor_has_api("PhysxRigidBodyAPI", prim)


def test_multiple_apply_instances(stage):
    prim = physicsUtils.add_rigid_box(stage, "/World/box")
    codeless.apply_api(prim, "PhysxCookedDataAPI", "convexHull")
    codeless.apply_api(prim, "PhysxCookedDataAPI", "triangleMesh")

    instances = schema.get_schema_instances(prim, "PhysxCookedDataAPI")
    assert {"convexHull", "triangleMesh"} <= set(instances)


def test_property_cache_roundtrip(stage):
    prim = physicsUtils.add_rigid_box(stage, "/World/box")
    codeless.apply_api(prim, "PhysxRigidBodyAPI")
    codeless.set_attr(prim, "physxRigidBody:disableGravity", True)
    codeless.set_attr(prim, "physxRigidBody:sleepThreshold", 0.125)

    cache = schema.create_api_schema_property_cache("PhysxRigidBodyAPI", prim)
    assert cache

    schema.remove_api_schema_properties("PhysxRigidBodyAPI", prim)
    prim.RemoveAPI("PhysxRigidBodyAPI")
    assert not prim.GetAttribute("physxRigidBody:disableGravity").HasAuthoredValue()

    codeless.apply_api(prim, "PhysxRigidBodyAPI")
    schema.apply_api_schema_property_cache(cache, prim)
    assert codeless.get_attr(prim, "physxRigidBody:disableGravity").Get() is True
    assert codeless.get_attr(prim, "physxRigidBody:sleepThreshold").Get() == pytest.approx(
        0.125
    )

    # The snapshot records authored state only, so the restore authors the two
    # opinions the prim had and none of the API's other attributes. Recording
    # resolved values instead bakes every schema fallback into the layer, which
    # detaches the prim from any later change to those defaults.
    restored = {
        name.GetName()
        for name in prim.GetAuthoredProperties()
        if name.GetName().startswith("physxRigidBody:")
    }
    assert restored == {
        "physxRigidBody:disableGravity",
        "physxRigidBody:sleepThreshold",
    }, restored


def test_multiple_apply_cache_replays_onto_another_instance(stage):
    prim = physicsUtils.add_rigid_box(stage, "/World/box")
    codeless.apply_api(prim, "PhysxCookedDataAPI", "convexHull")
    codeless.set_attr(
        prim,
        codeless.instanced_name("physxCookedData", "convexHull", "buffer"),
        Sdf.ValueTypeNames.UCharArray.defaultValue,
    )

    cache = schema.create_multiple_api_schema_property_cache(
        "PhysxCookedDataAPI", prim, "physxCookedData", "convexHull"
    )
    assert cache

    codeless.apply_api(prim, "PhysxCookedDataAPI", "triangleMesh")
    schema.apply_api_schema_property_cache(cache, prim, "triangleMesh")
    assert prim.GetAttribute("physxCookedData:triangleMesh:buffer")


def test_multiple_apply_cache_records_authored_state_only(stage):
    # PhysxCookedDataAPI cannot show this: its one attribute has no fallback, so
    # an unauthored read is None either way. PhysxLimitAPI has four that resolve.
    prim = physicsUtils.add_rigid_box(stage, "/World/box")
    codeless.apply_api(prim, "PhysxLimitAPI", "rotX")
    stiffness = codeless.instanced_name("physxLimit", "rotX", "stiffness")
    codeless.set_attr(prim, stiffness, 4.0)

    cache = schema.create_multiple_api_schema_property_cache(
        "PhysxLimitAPI", prim, "physxLimit", "rotX"
    )
    schema.remove_multiple_api_schema_properties("PhysxLimitAPI", prim, "physxLimit", "rotX")
    # The removal has to be observed here, as its single-apply sibling's case
    # observes it: `restored == {stiffness}` below holds just as well for a
    # property that was never removed, since an unremoved one is still authored.
    assert not prim.GetAttribute(stiffness).HasAuthoredValue()
    prim.RemoveAPI("PhysxLimitAPI", "rotX")

    codeless.apply_api(prim, "PhysxLimitAPI", "rotX")
    schema.apply_api_schema_property_cache(cache, prim, "rotX")
    restored = {
        prop.GetName()
        for prop in prim.GetAuthoredProperties()
        if prop.GetName().startswith("physxLimit:")
    }
    assert restored == {stiffness}, restored


def test_relationship_snapshot(stage):
    box = physicsUtils.add_rigid_box(stage, "/World/box")
    other = physicsUtils.add_rigid_box(stage, "/World/other")
    physicsUtils.add_pair_filter(stage, [box.GetPath(), other.GetPath()])

    cache = schema.create_api_schema_property_cache(UsdPhysics.FilteredPairsAPI, box)
    assert cache

    schema.remove_api_schema_properties(UsdPhysics.FilteredPairsAPI, box)
    box.RemoveAPI(UsdPhysics.FilteredPairsAPI)

    UsdPhysics.FilteredPairsAPI.Apply(box)
    schema.apply_api_schema_property_cache(cache, box)
    targets = box.GetRelationship("physics:filteredPairs").GetTargets()
    assert Sdf.Path("/World/other") in targets


def test_untargeted_relationship_is_not_authored_by_a_restore(stage):
    """The relationship half of AC-12's property-cache over-authoring fix.

    The omni.physx original replayed every relationship the API declares
    through ``CreateFooRel().SetTargets(value)``, unconditionally. For a
    relationship with no targets that authors an explicit ``rel <name> = None``
    opinion onto a prim that had none -- the same orphan the attribute half
    produced from fallback-resolved values, arrived at by a different route.
    The fix is the ``if not targets`` skip in
    ``apply_api_schema_property_cache``; removing it fails this case.

    USD has no fallback for relationship targets, so there is no resolved
    default to exclude at capture time -- but the capture side still needs a
    guard of its own, because a fallback is not the whole of "unauthored". The
    companion case below is the other half: an authored-but-empty relationship
    composes to an empty target list as well, so the snapshot has to record
    ``HasAuthoredTargets()`` to tell this prim from that one.
    """
    box = physicsUtils.add_rigid_box(stage, "/World/box")
    UsdPhysics.FilteredPairsAPI.Apply(box)

    rel = box.GetRelationship("physics:filteredPairs")
    assert rel, "the API declares the relationship, so the prim carries it"
    assert list(rel.GetTargets()) == []
    assert not rel.HasAuthoredTargets()

    cache = schema.create_api_schema_property_cache(UsdPhysics.FilteredPairsAPI, box)
    # The snapshot holds one relationship entry, and records that it has no
    # authored targets as `None` rather than as an empty list. Asserting only
    # that the entry is falsy would hold for the authored-empty prim below too,
    # which is the case that has to come back rather than stay away.
    assert [name for name, _ in cache[1]] == ["physics:filteredPairs"]
    assert all(targets is None for _, targets in cache[1]), cache[1]

    schema.remove_api_schema_properties(UsdPhysics.FilteredPairsAPI, box)
    box.RemoveAPI(UsdPhysics.FilteredPairsAPI)
    UsdPhysics.FilteredPairsAPI.Apply(box)
    schema.apply_api_schema_property_cache(cache, box)

    assert not box.GetRelationship("physics:filteredPairs").HasAuthoredTargets()
    assert "physics:filteredPairs" not in {
        prop.GetName() for prop in box.GetAuthoredProperties()
    }
    # An exported layer is where an orphaned opinion outlives the process that
    # made it, and `rel physics:filteredPairs = None` is the spelling it takes.
    exported = stage.GetRootLayer().ExportToString()
    assert "physics:filteredPairs" not in exported


def test_authored_empty_relationship_survives_a_restore(stage):
    """The capture side lost an authored-empty relationship to that same fix.

    An unauthored relationship and one authored with an empty target list both
    compose to ``[]``, so a snapshot that stored ``GetTargets()`` alone recorded
    the same thing for both and ``apply_api_schema_property_cache``'s
    ``if not targets`` skip then dropped the authored one.
    ``rel physics:filteredPairs = None`` is a real opinion -- a caller authors
    it to override an inherited or referenced target list with nothing, and an
    exported layer carries it -- so discarding it loses an opinion rather than
    declining to invent one.

    The case above only shows that an unwanted empty relationship is not
    *created*. Nothing there shows an existing one *survives*, and both
    assertions hold against a snapshot that cannot tell the two apart, which is
    what made that coverage incomplete rather than merely thin.
    """
    box = physicsUtils.add_rigid_box(stage, "/World/box")
    UsdPhysics.FilteredPairsAPI.Apply(box)
    assert box.CreateRelationship("physics:filteredPairs").SetTargets([])

    rel = box.GetRelationship("physics:filteredPairs")
    assert list(rel.GetTargets()) == []
    assert rel.HasAuthoredTargets(), "the empty target list is the authored opinion"
    assert "rel physics:filteredPairs = None" in stage.GetRootLayer().ExportToString()

    cache = schema.create_api_schema_property_cache(UsdPhysics.FilteredPairsAPI, box)
    # The snapshot has to carry the authoredness and not just the list: the
    # empty list on its own is exactly what the unauthored prim snapshots as.
    snapshotted = dict(cache[1])["physics:filteredPairs"]
    assert snapshotted is not None, cache[1]
    assert list(snapshotted) == []

    schema.remove_api_schema_properties(UsdPhysics.FilteredPairsAPI, box)
    box.RemoveAPI(UsdPhysics.FilteredPairsAPI)
    # Observed, so that the restore below is restoring rather than finding the
    # opinion still in place.
    assert not box.GetRelationship("physics:filteredPairs").HasAuthoredTargets()
    assert "physics:filteredPairs" not in stage.GetRootLayer().ExportToString()

    UsdPhysics.FilteredPairsAPI.Apply(box)
    schema.apply_api_schema_property_cache(cache, box)

    restored = box.GetRelationship("physics:filteredPairs")
    assert restored.HasAuthoredTargets(), "the authored-empty opinion was dropped"
    assert list(restored.GetTargets()) == []
    assert "rel physics:filteredPairs = None" in stage.GetRootLayer().ExportToString()


def test_multiple_apply_cache_carries_relationship_authoredness(stage):
    """The multiple-apply capture had the same ambiguity and takes the same fix.

    `create_multiple_api_schema_property_cache` read
    `GetRelationship(instance).GetTargets()` verbatim, so it lost an
    authored-empty relationship instance for the same reason its single-apply
    sibling did. `PhysxTendonAttachmentAPI` is the schema this can be shown on:
    it is multiple-apply and declares a `parentLink` relationship, where
    `PhysxLimitAPI` -- the schema the other multiple-apply cases here use --
    declares none, and an assertion over an empty relationship list would hold
    whatever the capture did.
    """
    prim = physicsUtils.add_rigid_box(stage, "/World/box")
    codeless.apply_api(prim, "PhysxTendonAttachmentAPI", "t0")
    parent_link = codeless.instanced_name("physxTendon", "t0", "parentLink")

    # Unauthored first: recorded as `None`, under the instance-name template.
    unauthored = schema.create_multiple_api_schema_property_cache(
        "PhysxTendonAttachmentAPI", prim, "physxTendon", "t0"
    )
    assert dict(unauthored[1]) == {
        "physxTendon:__INSTANCE_NAME__:parentLink": None
    }, unauthored[1]

    assert prim.CreateRelationship(parent_link).SetTargets([])
    assert prim.GetRelationship(parent_link).HasAuthoredTargets()

    cache = schema.create_multiple_api_schema_property_cache(
        "PhysxTendonAttachmentAPI", prim, "physxTendon", "t0"
    )
    snapshotted = dict(cache[1])["physxTendon:__INSTANCE_NAME__:parentLink"]
    assert snapshotted is not None, cache[1]
    assert list(snapshotted) == []

    schema.remove_multiple_api_schema_properties(
        "PhysxTendonAttachmentAPI", prim, "physxTendon", "t0"
    )
    prim.RemoveAPI("PhysxTendonAttachmentAPI", "t0")
    assert not prim.GetRelationship(parent_link).HasAuthoredTargets()

    codeless.apply_api(prim, "PhysxTendonAttachmentAPI", "t0")
    schema.apply_api_schema_property_cache(cache, prim, "t0")

    restored = prim.GetRelationship(parent_link)
    assert restored.HasAuthoredTargets(), "the authored-empty instance was dropped"
    assert list(restored.GetTargets()) == []
    assert f"rel {parent_link} = None" in stage.GetRootLayer().ExportToString()


def test_world_position_is_the_centre_of_the_prims_world_bound(stage):
    """AC-4's world-position clause, asserted against the authored position.

    The prim has to be one with a bound. This case used a bare `add_rigid_xform`,
    which has no geometry, so `ComputeWorldBound` is empty and the helper answers
    the origin however the prim is translated -- which is why the old
    `is not None` assertion held without saying anything.
    """
    prim = physicsUtils.add_rigid_box(stage, "/World/box", position=Gf.Vec3f(1, 0, 0))
    position = physicsUtils.get_world_position(stage, prim.GetPath())
    assert tuple(position) == pytest.approx((1.0, 0.0, 0.0)), position


# The registration-ordering cases below need a process where the schema registry
# has NOT been built yet, which this one cannot supply: the session fixture
# registered before the first stage opened, and USD's registry is process-global
# and one-way. Each therefore runs in a child interpreter.

# Opens a stage first, which is what a host's USD import/export amounts to, then
# registers too late. Reports what each helper said rather than asserting, so the
# parent owns the expectations.
_LATE_REGISTRATION_CHILD = """
import json

from pxr import Usd

from ovphysx.utils import codeless

stage = Usd.Stage.CreateInMemory()
prim = stage.DefinePrim("/World/Box", "Cube")

outcome = {}
if codeless.schema_is_registered():
    # PXR_PLUGINPATH_NAME or ovphysx's own plugin path already carries the
    # schemas, so registration cannot be late here.
    outcome["preset"] = True
else:
    try:
        codeless.register_schemas(verify=True)
    except FileNotFoundError as exc:
        outcome["unstaged"] = str(exc)
    except codeless.CodelessSchemaError as exc:
        outcome["verify_error"] = str(exc)

    if "unstaged" not in outcome:
        try:
            codeless.apply_api(prim, "PhysxRigidBodyAPI")
        except codeless.CodelessSchemaError as exc:
            outcome["apply_error"] = str(exc)
        try:
            codeless.remove_api(prim, "PhysxRigidBodyAPI")
        except codeless.CodelessSchemaError as exc:
            outcome["remove_error"] = str(exc)

print("RESULT " + json.dumps(outcome))
"""

# Registers a tree carrying the PhysX schema root and not the Omni deformable
# one, in the supported order, so nothing here is late. Rebinding
# codeless_schema_paths on the package is how the child gets a half-staged
# install: register_schemas() imports that name at call time.
_HALF_STAGED_CHILD = """
import json
import sys
from pathlib import Path

import ovphysx

half_staged = Path(sys.argv[1])
ovphysx.codeless_schema_paths = lambda: sorted(half_staged.glob("*/resources"))

from ovphysx.utils import codeless
from pxr import Usd

outcome = {"roots": [path.parent.name for path in ovphysx.codeless_schema_paths()]}
try:
    codeless.register_schemas(verify=True)
except codeless.CodelessSchemaError as exc:
    outcome["verify_error"] = str(exc)

registry = Usd.SchemaRegistry()
outcome["physx_registered"] = bool(registry.FindAppliedAPIPrimDefinition("PhysxRigidBodyAPI"))
outcome["deformable_registered"] = bool(
    registry.FindAppliedAPIPrimDefinition("OmniPhysicsDeformableBodyAPI")
)
outcome["reports_registered"] = codeless.schema_is_registered()

stage = Usd.Stage.CreateInMemory()
prim = stage.DefinePrim("/World/Box", "Cube")
codeless.apply_api(prim, "PhysxRigidBodyAPI")
outcome["applied"] = "PhysxRigidBodyAPI" in [str(name) for name in prim.GetAppliedSchemas()]
try:
    codeless.apply_api(prim, "OmniPhysicsDeformableBodyAPI")
except codeless.CodelessSchemaError as exc:
    outcome["deformable_error"] = str(exc)

print("RESULT " + json.dumps(outcome))
"""

# Registers a half-staged tree with `verify` off and then registers the complete
# one. The default path must leave the schema registry unbuilt, or the second
# registration is silently too late. In "control" mode the second registration
# is omitted, which is what tells a layout carrying the deformable root from
# process start apart from a default path that really left the registry open.
_DEFAULT_PATH_CHILD = """
import json
import sys
from pathlib import Path

import ovphysx

complete = list(ovphysx.codeless_schema_paths())
half_staged = Path(sys.argv[1])
register_second = sys.argv[2] == "second"

ovphysx.codeless_schema_paths = lambda: sorted(half_staged.glob("*/resources"))

from ovphysx.utils import codeless

outcome = {"first_roots": [path.parent.name for path in ovphysx.codeless_schema_paths()]}
codeless.register_schemas()

if register_second:
    ovphysx.codeless_schema_paths = lambda: complete
    codeless.register_schemas()

from pxr import Usd

registry = Usd.SchemaRegistry()
outcome["physx_registered"] = bool(registry.FindAppliedAPIPrimDefinition("PhysxRigidBodyAPI"))
outcome["deformable_registered"] = bool(
    registry.FindAppliedAPIPrimDefinition("OmniPhysicsDeformableBodyAPI")
)

print("RESULT " + json.dumps(outcome))
"""

# Each helper below defines a codeless concrete prim. Without a registered
# definition, USD accepts the type token but creates an inert unknown prim.
_CONCRETE_PRIM_PREFLIGHT_CHILD = """
import json
import sys

from pxr import Usd

from ovphysx.utils import codeless, joints, particles

mode = sys.argv[1]
outcome = {}

stage = Usd.Stage.CreateInMemory()
registry = Usd.SchemaRegistry()
if registry.FindConcretePrimDefinition("PhysxParticleSystem"):
    outcome["preset"] = True
else:
    if mode == "late":
        try:
            codeless.register_schemas()
        except FileNotFoundError as exc:
            outcome["unstaged"] = str(exc)

    if "unstaged" not in outcome:
        cases = (
            ("particle", "PhysxParticleSystem"),
            ("gear", "PhysxPhysicsGearJoint"),
            ("rack", "PhysxPhysicsRackAndPinionJoint"),
        )
        for label, type_name in cases:
            case_stage = Usd.Stage.CreateInMemory()
            body = case_stage.DefinePrim("/World/body", "Xform")
            before = case_stage.GetRootLayer().ExportToString()
            try:
                if label == "particle":
                    particles.add_physx_particle_system(case_stage, "/World/particleSystem")
                elif label == "gear":
                    joints.create_joint(case_stage, "Gear", None, body)
                else:
                    joints.create_joint(case_stage, "RackAndPinion", None, body)
            except codeless.CodelessSchemaError as exc:
                outcome[label + "_error"] = str(exc)
            else:
                outcome[label + "_error"] = None
            outcome[label + "_unchanged"] = (
                case_stage.GetRootLayer().ExportToString() == before
            )
            outcome[label + "_type"] = type_name

print("RESULT " + json.dumps(outcome))
"""

# The supported ordering, with verification on: registering first must both
# satisfy verify and leave the schemas usable.
_EARLY_REGISTRATION_CHILD = """
from ovphysx.utils import codeless

codeless.register_schemas(verify=True)

from pxr import Usd

stage = Usd.Stage.CreateInMemory()
prim = stage.DefinePrim("/World/Box", "Cube")
codeless.apply_api(prim, "PhysxRigidBodyAPI")
assert "PhysxRigidBodyAPI" in [str(name) for name in prim.GetAppliedSchemas()]
print("OK")
"""


def _child_env():
    """The environment a shipped authoring process has, which this one does not.

    `import ovphysx` publishes its compiled schema plugins on
    OV_PXR_PLUGINPATH_2511, derived from OVPHYSX_LIB. That variable is read by
    ovphysx's own namespaced USD, and this suite runs against a py-enabled build
    of it, so a child inherits the schemas already registered at process start.
    No shipped configuration does: the wheel's USD is the py-less ovstage
    runtime, and an authoring process supplies a stock `usd-core`, which reads
    PXR_PLUGINPATH_NAME instead and so ignores the variable entirely. Dropping
    the three restores the layout a user gets, which is the one these cases are
    about; without it the unregistered-schemas branch never runs here.
    """
    env = dict(os.environ)
    for name in ("OV_PXR_PLUGINPATH_2511", "OVPHYSX_LIB", "PXR_PLUGINPATH_NAME"):
        env.pop(name, None)
    return env


def _run_child(script, *args):
    proc = subprocess.run(
        [sys.executable, "-c", script, *args],
        capture_output=True,
        text=True,
        env=_child_env(),
        timeout=120,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    return proc


def _child_outcome(proc):
    line = next(
        candidate for candidate in proc.stdout.splitlines() if candidate.startswith("RESULT ")
    )
    return json.loads(line[len("RESULT ") :])


def test_late_registration_is_reported_rather_than_silently_accepted(
    codeless_schemas, codeless_unavailable
):
    """A registration arriving after the registry was built raises under `verify`.

    Without `verify` USD gives no sign: `RegisterPlugins` still accepts the
    plugin roots and `Tf.Type.FindByName` still resolves the schema types, and
    only a later `ApplyAPI` fails (REQ AC-14).

    The two degradations below go through the conftest's fail-or-skip helper
    rather than `pytest.skip`, so they cannot take AC-14 out of the CI run and
    still report a pass.
    """
    outcome = _child_outcome(_run_child(_LATE_REGISTRATION_CHILD))

    if outcome.get("preset"):
        codeless_unavailable("the schemas are on this process's plugin path from the start")
    if "unstaged" in outcome:
        codeless_unavailable(
            f"codeless schemas not staged in this layout: {outcome['unstaged']}"
        )

    assert "verify_error" in outcome, outcome
    assert "PXR_PLUGINPATH_NAME" in outcome["verify_error"]


def test_registration_hint_offers_a_recovery_that_can_work(
    codeless_schemas, codeless_unavailable
):
    """The hint must not send the caller back to a call that cannot succeed.

    Reaching this branch means the registry is already built, so the advice has
    to be an ordering fix applied in a new process, not a `register_schemas()`
    call in this one (REQ AC-14). `remove_api` carries the same hint, since USD
    fails a removal against an unregistered schema set just as it fails an
    application (REQ AC-10).
    """
    outcome = _child_outcome(_run_child(_LATE_REGISTRATION_CHILD))

    if outcome.get("preset") or "unstaged" in outcome:
        codeless_unavailable("the unregistered-schemas branch is unreachable in this layout")

    for key in ("apply_error", "remove_error"):
        message = outcome[key]
        assert "cannot be repaired" in message, message
        assert "PXR_PLUGINPATH_NAME" in message, message


@pytest.mark.parametrize("mode", ("none", "late"))
def test_concrete_codeless_types_are_rejected_before_stage_mutation(codeless_schemas, codeless_unavailable, mode):
    """Unknown concrete types fail before defining inert prims (REQ AC-10)."""
    outcome = _child_outcome(_run_child(_CONCRETE_PRIM_PREFLIGHT_CHILD, mode))

    if outcome.get("preset"):
        codeless_unavailable("the schemas are on this process's plugin path from the start")
    if "unstaged" in outcome:
        codeless_unavailable(f"codeless schemas not staged in this layout: {outcome['unstaged']}")

    for label in ("particle", "gear", "rack"):
        message = outcome[label + "_error"]
        assert outcome[label + "_type"] in message, outcome
        assert "cannot be repaired" in message, message
        assert "PXR_PLUGINPATH_NAME" in message, message
        assert outcome[label + "_unchanged"], outcome


def test_verified_registration_before_any_stage_succeeds(codeless_schemas):
    """`verify` stays quiet on the supported ordering (REQ AC-14)."""
    proc = _run_child(_EARLY_REGISTRATION_CHILD)
    assert proc.stdout.strip().endswith("OK"), proc.stdout + proc.stderr


def test_half_staged_tree_is_reported_as_a_registration_failure(
    half_staged_schema_root, codeless_unavailable
):
    """One root registering while the other does not is not a spelling mistake.

    This is what the second sentinel identifier buys: probing only the PhysX
    root would answer that the schemas are registered, so the deformable helpers
    would fail with USD's own error and the diagnosis would send the caller
    looking at their identifier spelling (REQ AC-10, AC-14).
    """
    outcome = _child_outcome(_run_child(_HALF_STAGED_CHILD, str(half_staged_schema_root)))

    if outcome["deformable_registered"]:
        codeless_unavailable("this layout carries the deformable schemas from process start")

    # The half that is staged has to have registered, or the child is showing a
    # late registration rather than a partial one.
    assert outcome["roots"] == ["PhysxSchema"], outcome
    assert outcome["physx_registered"], outcome
    assert outcome["applied"], outcome

    assert "verify_error" in outcome, outcome
    assert "OmniPhysicsDeformableBodyAPI" in outcome["verify_error"], outcome
    assert "PhysxRigidBodyAPI" not in outcome["verify_error"], outcome
    assert outcome["reports_registered"] is False, outcome

    assert "deformable_error" in outcome, outcome
    message = outcome["deformable_error"]
    assert "cannot be repaired" in message, message
    assert "PXR_PLUGINPATH_NAME" in message, message
    assert "does not name a known API schema" not in message, message


def test_default_registration_leaves_the_schema_registry_unbuilt(
    half_staged_schema_root, codeless_unavailable
):
    """`register_schemas()` without `verify` must not touch the schema registry.

    USD builds that registry once and never rebuilds it, so a probe on the
    default path would lock out every later schema plugin registration in the
    process -- ovphysx's own and any other library's alike. That is the whole
    reason `verify` is opt-in rather than always on (REQ AC-14), and it only
    holds if the default path never reaches the probe.

    Observed the way a caller would meet it: register a partial tree with
    `verify` off, then register the complete one, and ask whether the root that
    arrived second is usable. The control child stops after the first
    registration, so a layout that already carries the deformable root at
    process start cannot be mistaken for a default path that left the registry
    open.
    """
    control = _child_outcome(
        _run_child(_DEFAULT_PATH_CHILD, str(half_staged_schema_root), "control")
    )
    if control["deformable_registered"]:
        codeless_unavailable("this layout carries the deformable schemas from process start")

    assert control["first_roots"] == ["PhysxSchema"], control
    assert control["physx_registered"], control

    outcome = _child_outcome(
        _run_child(_DEFAULT_PATH_CHILD, str(half_staged_schema_root), "second")
    )
    assert outcome["deformable_registered"], outcome
