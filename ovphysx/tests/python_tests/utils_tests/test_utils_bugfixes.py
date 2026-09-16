# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""The defects inherited from omni.physx.scripts, and their fixes.

Each case names the inherited defect it covers. The two subtree cases claim
AC-5's subtree clause as well, since the collider walk is where the
single-prim fixes either reach a hierarchy or do not.

The deformable-removal cases claim two AC-8 clauses. The first two claim the
removal below one level of children, which the deformable case in
`test_utils_authoring.py` cannot reach: its hierarchy puts the skin mesh
directly under the root, so it says nothing about a deeper one. The twenty
ownership cases claim the collision-API ownership clause, on the removal
path and on the `create_auto_*` paths, which run that teardown before
rebuilding, and in both directions: a collision API no deformable simulation
API accounts for is kept, and one it does account for is taken back. One of the
twenty puts the accounting simulation API below an immediate child, where the
ownership scan has to reach as far as the strip loop does. Six of
the twenty are about the paths the teardown exempts, which is what tells a
mesh path a build reuses from one it abandons; the last of those six pins the
limitation AC-8 records, that a moved collision path leaves a collider no
rebuild can attribute. Three run the same ownership question on a *property*
rather than an API: a property outlives the removal exactly when an API that
survives it declares it. A fourth asks what the strip reaches rather than what
it keeps: only a name a layer authored, and no name in a namespace that merely
starts like `physxDeformableBody`. One covers the curves simulation API, which
the teardown takes although no helper here applies it.

The point-type and non-Xformable cases are not defect fixes. They are the only
assertions anywhere that a helper which took a `carb.Float3` sequence accepts a
`Gf.Vec3f` (AC-6), and that a transform helper handed a non-Xformable logs
instead of raising. The bare-prim cases cover the runtime contracts and the
accepted-input annotation corrections for AC-4, so both ACs are claimed here.
"""

# @implements REQ-PYTHON-UTILS-001
# @covers AC-4 AC-5 AC-6 AC-8 AC-12

import subprocess
import sys
import textwrap
import typing

import pytest

pytest.importorskip("pxr", reason="ovphysx.utils requires a caller-supplied USD runtime")

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt  # noqa: E402

from ovphysx import utils as physicsUtils  # noqa: E402
from ovphysx.utils import codeless, particles  # noqa: E402
from ovphysx.utils.schema import get_schema_property_names  # noqa: E402


def test_removePairFilter_removes_rather_than_applies(stage):
    """omni.physx applied FilteredPairsAPI here, doing the opposite of its name."""
    box = physicsUtils.add_rigid_box(stage, "/World/box")
    other = physicsUtils.add_rigid_box(stage, "/World/other")

    physicsUtils.add_pair_filter(stage, [box.GetPath(), other.GetPath()])
    assert box.HasAPI(UsdPhysics.FilteredPairsAPI)
    assert other.HasAPI(UsdPhysics.FilteredPairsAPI)

    physicsUtils.remove_pair_filter(stage, [box.GetPath(), other.GetPath()])
    assert not box.HasAPI(UsdPhysics.FilteredPairsAPI)
    assert not other.HasAPI(UsdPhysics.FilteredPairsAPI)

    # Dropping the API is not enough: the relationship and its targets are a
    # separate authored opinion, so they have to go too or the prim still
    # filters in an exported layer.
    assert not box.GetRelationship("physics:filteredPairs")
    exported = stage.GetRootLayer().ExportToString()
    assert "physics:filteredPairs" not in exported, exported

    # A later add_pair_filter must not resurrect the removed target.
    third = physicsUtils.add_rigid_box(stage, "/World/third")
    physicsUtils.add_pair_filter(stage, [box.GetPath(), third.GetPath()])
    assert list(box.GetRelationship("physics:filteredPairs").GetTargets()) == [
        third.GetPath()
    ]


def test_addPairFilter_compares_paths_by_value(stage):
    """omni.physx used `otherPath is not path`, so duplicates self-filtered."""
    box = physicsUtils.add_rigid_box(stage, "/World/box")
    other = physicsUtils.add_rigid_box(stage, "/World/other")

    # Distinct paths: each filters against the other exactly once.
    physicsUtils.add_pair_filter(stage, [box.GetPath(), other.GetPath()])
    assert list(box.GetRelationship("physics:filteredPairs").GetTargets()) == [
        other.GetPath()
    ]

    # A list mixing str and Sdf.Path spellings of one prim must not self-filter.
    third = physicsUtils.add_rigid_box(stage, "/World/third")
    physicsUtils.add_pair_filter(stage, [str(third.GetPath()), Sdf.Path(third.GetPath())])
    targets = third.GetRelationship("physics:filteredPairs").GetTargets()
    assert third.GetPath() not in targets, targets

    # Two equal-but-distinct duplicate strings likewise. Built by concatenation
    # so the interpreter cannot intern them into one object, which is what makes
    # this an identity-versus-equality test rather than a tautology.
    fourth = physicsUtils.add_rigid_box(stage, "/World/fourth")
    first_spelling = "/World/fourth"
    second_spelling = "".join(["/World/", "fourth"])
    assert first_spelling is not second_spelling
    assert first_spelling == second_spelling
    physicsUtils.add_pair_filter(stage, [first_spelling, second_spelling])
    targets = fourth.GetRelationship("physics:filteredPairs").GetTargets()
    assert fourth.GetPath() not in targets, targets


def test_removeCollider_strips_approximation_apis_from_instanceable_prim(stage):
    """omni.physx only stripped these on the custom_execute_fn path."""
    physicsUtils.create_mesh_cube(stage, "/World/proto", 1.0)

    mesh_prim = physicsUtils.create_mesh_cube(stage, "/World/mesh", 1.0).GetPrim()
    physicsUtils.set_collider(mesh_prim, UsdPhysics.Tokens.convexDecomposition)
    assert "PhysxConvexDecompositionCollisionAPI" in mesh_prim.GetAppliedSchemas()

    xform = stage.DefinePrim("/World/instanceable", "Xform")
    xform.GetReferences().AddInternalReference(Sdf.Path("/World/proto"))
    xform.SetInstanceable(True)
    physicsUtils.set_collider(xform, UsdPhysics.Tokens.convexDecomposition)
    assert xform.IsInstanceable()
    assert "PhysxConvexDecompositionCollisionAPI" in xform.GetAppliedSchemas()

    physicsUtils.remove_collider(xform)
    assert "PhysxConvexDecompositionCollisionAPI" not in xform.GetAppliedSchemas()
    assert not xform.HasAPI(UsdPhysics.MeshCollisionAPI)

    physicsUtils.remove_collider(mesh_prim)
    assert "PhysxConvexDecompositionCollisionAPI" not in mesh_prim.GetAppliedSchemas()


def test_setCollider_guard_matches_the_recorded_schema_name(stage):
    """omni.physx guarded on "CollisionAPI"; USD records "PhysicsCollisionAPI"."""
    prim = physicsUtils.create_mesh_cube(stage, "/World/mesh", 1.0).GetPrim()

    # The guard has to test the name USD records, not the one the typed class is
    # spelled with, or it never matches and the early return is dead code.
    UsdPhysics.CollisionAPI.Apply(prim)
    assert "PhysicsCollisionAPI" in prim.GetAppliedSchemas()
    assert "CollisionAPI" not in prim.GetAppliedSchemas()

    physicsUtils.set_collider(prim, UsdPhysics.Tokens.convexHull)
    assert "PhysxConvexHullCollisionAPI" not in prim.GetAppliedSchemas()
    assert not prim.HasAPI(UsdPhysics.MeshCollisionAPI)

    # A second call on a prim set_collider itself authored is a no-op too, so one
    # approximation API cannot end up stacked on another.
    fresh = physicsUtils.create_mesh_cube(stage, "/World/fresh", 1.0).GetPrim()
    physicsUtils.set_collider(fresh, UsdPhysics.Tokens.convexHull)
    physicsUtils.set_collider(fresh, UsdPhysics.Tokens.meshSimplification)
    applied = fresh.GetAppliedSchemas()
    assert "PhysxConvexHullCollisionAPI" in applied
    assert "PhysxTriangleMeshSimplificationCollisionAPI" not in applied, applied
    assert (
        UsdPhysics.MeshCollisionAPI(fresh).GetApproximationAttr().Get()
        == UsdPhysics.Tokens.convexHull
    )


@pytest.mark.parametrize("no_collision", (False, True))
def test_setCollider_reads_the_no_collision_value(stage, no_collision):
    prim = physicsUtils.create_mesh_cube(stage, "/World/mesh", 1.0).GetPrim()
    prim.CreateAttribute("omni:no_collision", Sdf.ValueTypeNames.Bool).Set(no_collision)

    before = stage.GetRootLayer().ExportToString()
    physicsUtils.set_collider(prim, UsdPhysics.Tokens.convexHull)

    assert prim.HasAPI(UsdPhysics.CollisionAPI) == (not no_collision)
    if no_collision:
        assert stage.GetRootLayer().ExportToString() == before


@pytest.mark.parametrize(
    "initial_kinematic, requested_kinematic",
    ((False, True), (True, False)),
)
def test_set_physics_completes_a_preapplied_disabled_body(
    stage, initial_kinematic, requested_kinematic
):
    """A partial rigid-body API set must not make the operation a no-op."""
    prim = physicsUtils.create_mesh_cube(stage, "/World/mesh", 1.0).GetPrim()
    rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
    rigid_body.CreateRigidBodyEnabledAttr(False)
    rigid_body.CreateKinematicEnabledAttr(initial_kinematic)

    assert "PhysxRigidBodyAPI" not in prim.GetAppliedSchemas()
    physicsUtils.set_physics(prim, requested_kinematic)

    assert prim.HasAPI(UsdPhysics.RigidBodyAPI)
    assert "PhysxRigidBodyAPI" in prim.GetAppliedSchemas()
    assert rigid_body.GetRigidBodyEnabledAttr().Get() is True
    assert rigid_body.GetKinematicEnabledAttr().Get() is requested_kinematic


@pytest.mark.parametrize(
    "token, api",
    (
        ("sdf", "PhysxSDFMeshCollisionAPI"),
        ("sphereFill", "PhysxSphereFillCollisionAPI"),
    ),
)
def test_removeCollider_strips_the_sdf_and_sphere_fill_apis(stage, token, api):
    """omni.physx stripped only the hull and mesh-simplification APIs."""
    assert physicsUtils.MESH_APPROXIMATIONS[token] == api, "constant drifted"

    prim = physicsUtils.create_mesh_cube(stage, f"/World/{token}", 1.0).GetPrim()
    physicsUtils.set_collider(prim, token)
    assert api in prim.GetAppliedSchemas()

    physicsUtils.remove_collider(prim)
    assert api not in prim.GetAppliedSchemas(), prim.GetAppliedSchemas()
    assert not prim.HasAPI(UsdPhysics.CollisionAPI)
    assert not prim.HasAPI(UsdPhysics.MeshCollisionAPI)


def test_removeCollider_takes_the_approximation_opinions_with_the_apis(stage):
    """omni.physx dropped the API and left its properties authored as orphans.

    `Usd.Prim.RemoveAPI` and `codeless.remove_api` remove the `apiSchemas` entry
    only, so a prim re-approximated after a `remove_collider` kept the previous
    approximation's tuning and its cooked buffer -- opinions for APIs it no
    longer carries. This is the same defect class as `remove_pair_filter`
    leaving `physics:filteredPairs` authored, and it is fixed the same way.
    """
    token = physicsUtils.COOKED_DATA_TOKENS[0]
    buffer_name = codeless.instanced_name("physxCookedData", token, "buffer")

    prim = physicsUtils.create_mesh_cube(stage, "/World/mesh", 1.0).GetPrim()
    physicsUtils.set_collider(prim, UsdPhysics.Tokens.convexHull)
    codeless.set_attr(prim, "physxConvexHullCollision:hullVertexLimit", 42)
    codeless.set_attr(prim, "physxCollision:contactOffset", 0.5)
    codeless.set_attr(prim, "physxCollision:restOffset", 0.25)
    codeless.apply_api(prim, "PhysxCookedDataAPI", token)
    codeless.set_attr(prim, buffer_name, Vt.UCharArray([1, 2, 3]))
    assert prim.GetAttribute(buffer_name).HasAuthoredValue()

    physicsUtils.remove_collider(prim)
    hull_limit = prim.GetAttribute("physxConvexHullCollision:hullVertexLimit")
    assert not hull_limit.HasAuthoredValue(), hull_limit.Get()
    assert not prim.GetAttribute(buffer_name).HasAuthoredValue()

    # PhysxCollisionAPI is re-applied by every set_collider call, so its offsets
    # are not orphaned and must survive the cycle. The property cache's worked
    # example in schema.py depends on exactly that: it pairs remove_collider
    # with an explicit remove_api_schema_properties call because these two
    # values outlive the removal.
    physicsUtils.set_collider(prim, UsdPhysics.Tokens.convexDecomposition)
    assert "PhysxConvexDecompositionCollisionAPI" in prim.GetAppliedSchemas()
    assert not hull_limit.HasAuthoredValue(), hull_limit.Get()
    assert codeless.get_attr(prim, "physxCollision:contactOffset").Get() == pytest.approx(0.5)
    assert codeless.get_attr(prim, "physxCollision:restOffset").Get() == pytest.approx(0.25)

    # Nothing of the previous approximation reaches an exported layer either,
    # which is where an orphaned opinion outlives the process that made it.
    exported = stage.GetRootLayer().ExportToString()
    assert "physxConvexHullCollision" not in exported, exported
    assert "physxCookedData" not in exported, exported


def test_removeCollider_worked_example_from_the_property_cache_docstring(stage):
    """The example in `create_api_schema_property_cache`'s docstring, executed.

    It is the one place the split above is documented for a caller: the
    approximation's opinions go with the API, the two offsets do not, and the
    cache is what carries them across an explicit removal. An example that
    stopped holding would be worse than no example.
    """
    prim = physicsUtils.create_mesh_cube(stage, "/World/mesh", 1.0).GetPrim()
    physicsUtils.set_collider(prim, UsdPhysics.Tokens.convexHull)
    codeless.set_attr(prim, "physxCollision:contactOffset", 0.5)
    codeless.set_attr(prim, "physxCollision:restOffset", 0.25)
    codeless.set_attr(prim, "physxConvexHullCollision:hullVertexLimit", 42)

    tuning = physicsUtils.create_api_schema_property_cache("PhysxCollisionAPI", prim)

    physicsUtils.remove_collider(prim)
    physicsUtils.remove_api_schema_properties("PhysxCollisionAPI", prim)

    physicsUtils.set_collider(prim, UsdPhysics.Tokens.convexDecomposition)
    physicsUtils.apply_api_schema_property_cache(tuning, prim)

    assert (
        UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
        == UsdPhysics.Tokens.convexDecomposition
    )
    assert codeless.get_attr(prim, "physxCollision:contactOffset").Get() == pytest.approx(0.5)
    assert codeless.get_attr(prim, "physxCollision:restOffset").Get() == pytest.approx(0.25)
    authored = {
        prop.GetName()
        for prop in prim.GetAuthoredProperties()
        if prop.GetName().startswith("physx")
    }
    assert authored == {"physxCollision:contactOffset", "physxCollision:restOffset"}, authored


def _collision_schemas(prim):
    """The applied schemas a collider leaves behind, in USD's own spelling."""
    return [
        name
        for name in prim.GetAppliedSchemas()
        if "Collision" in name or "CookedData" in name
    ]


def _make_instanceable(stage, path, prototype_path):
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddInternalReference(Sdf.Path(prototype_path))
    prim.SetInstanceable(True)
    return prim


def test_removeColliderSubtree_leaves_the_whole_subtree_clean(stage):
    """omni.physx visited gprims only, so an instanceable prim kept its collider.

    This is also how remove_collider's own fixes reach a subtree: the sdf and
    sphere-fill APIs it added to the strip list, and the instanceable handling
    that the gprim-only walk made unreachable from here.
    """
    physicsUtils.create_mesh_cube(stage, "/World/proto", 1.0)
    root = UsdGeom.Xform.Define(stage, "/World/robot").GetPrim()
    stage.DefinePrim("/World/robot/inner", "Xform")

    approximations = {
        "/World/robot/inner/sdf": "sdf",
        "/World/robot/inner/sphereFill": "sphereFill",
        "/World/robot/hull": UsdPhysics.Tokens.convexHull,
        "/World/robot/simplified": UsdPhysics.Tokens.meshSimplification,
    }
    for path, approximation in approximations.items():
        mesh_prim = physicsUtils.create_mesh_cube(stage, path, 1.0).GetPrim()
        physicsUtils.set_collider(mesh_prim, approximation)
        assert physicsUtils.MESH_APPROXIMATIONS[approximation] in mesh_prim.GetAppliedSchemas()

    # Cooked data rides along with a collider and is stripped by the same call.
    cooked = stage.GetPrimAtPath("/World/robot/hull")
    codeless.apply_api(cooked, "PhysxCookedDataAPI", physicsUtils.COOKED_DATA_TOKENS[0])

    instanceable = _make_instanceable(stage, "/World/robot/instanceable", "/World/proto")
    physicsUtils.set_collider(instanceable, UsdPhysics.Tokens.convexDecomposition)
    assert "PhysxConvexDecompositionCollisionAPI" in instanceable.GetAppliedSchemas()

    # A prim that was never a collider, to show the walk does not author onto one.
    scope = stage.DefinePrim("/World/robot/scope", "Scope")

    physicsUtils.remove_collider_subtree(root)

    for prim in Usd.PrimRange(root):
        assert not _collision_schemas(prim), (prim.GetPath(), prim.GetAppliedSchemas())
        assert not prim.HasAPI(UsdPhysics.CollisionAPI), prim.GetPath()
        assert not prim.HasAPI(UsdPhysics.MeshCollisionAPI), prim.GetPath()
    assert not scope.GetAppliedSchemas()
    assert instanceable.IsInstanceable(), "the removal must not un-instance the prim"

    # An applied API reaches an exported layer as a `prepend apiSchemas` entry,
    # and a removed one as a `delete apiSchemas` opinion, which is what carries
    # the removal across the reference on the instanceable prim.
    exported = stage.GetRootLayer().ExportToString()
    assert "prepend apiSchemas" not in exported, exported
    assert "delete apiSchemas" in exported, exported


def test_removeRigidBodySubtree_undoes_setRigidBody(stage):
    """The rigid-body subtree walk is the collider walk plus the body itself.

    It is the path remove_rigid_body takes for an Xformable, so the instanceable
    prim above reaches it from there too.
    """
    physicsUtils.create_mesh_cube(stage, "/World/proto", 1.0)
    root = UsdGeom.Xform.Define(stage, "/World/body").GetPrim()
    mesh_prim = physicsUtils.create_mesh_cube(stage, "/World/body/mesh", 1.0).GetPrim()
    instanceable = _make_instanceable(stage, "/World/body/instanceable", "/World/proto")

    physicsUtils.set_rigid_body(root, UsdPhysics.Tokens.convexHull, False)
    assert root.HasAPI(UsdPhysics.RigidBodyAPI)
    assert "PhysxConvexHullCollisionAPI" in mesh_prim.GetAppliedSchemas()
    assert "PhysxConvexHullCollisionAPI" in instanceable.GetAppliedSchemas()

    physicsUtils.remove_rigid_body_subtree(root)
    assert not root.HasAPI(UsdPhysics.RigidBodyAPI)
    assert "PhysxRigidBodyAPI" not in root.GetAppliedSchemas()
    for prim in Usd.PrimRange(root):
        assert not _collision_schemas(prim), (prim.GetPath(), prim.GetAppliedSchemas())


def test_poisson_sample_mesh_distinguishes_success_from_failure(stage, monkeypatch):
    """omni.physx returned None on success, so it looked like the failure path."""
    physicsUtils.create_mesh_cube(stage, "/World/mesh", 1.0)

    first = physicsUtils.poisson_sample_mesh(stage, Sdf.Path("/World/mesh"))
    assert first, "success must return the particle system path"
    assert not first.isEmpty
    prim = stage.GetPrimAtPath("/World/mesh")
    assert "PhysxParticleSamplingAPI" in prim.GetAppliedSchemas()
    assert codeless.get_attr(prim, "physxParticleSampling:volume").Get() is True

    # Called twice on one stage it reuses the same particle system.
    physicsUtils.create_mesh_cube(stage, "/World/mesh2", 1.0)
    assert physicsUtils.poisson_sample_mesh(stage, Sdf.Path("/World/mesh2")) == first
    systems = [p for p in stage.Traverse() if p.GetTypeName() == "PhysxParticleSystem"]
    assert len(systems) == 1, systems

    # Failure returns an empty path, which is now distinguishable from success.
    # Injected at the creation call, on a stage that has no particle system to
    # find: `stage` above now has one, so the creation branch is unreachable there.
    monkeypatch.setattr(particles, "add_physx_particle_system", lambda *args, **kwargs: None)
    empty_stage = Usd.Stage.CreateInMemory()
    physicsUtils.create_mesh_cube(empty_stage, "/World/mesh3", 1.0)
    failed = particles.poisson_sample_mesh(empty_stage, Sdf.Path("/World/mesh3"))
    assert failed.isEmpty


def test_poisson_sample_mesh_reports_an_occupied_default_path(stage, caplog):
    """omni.physx raised AssertionError when the default path was already taken."""
    physicsUtils.create_mesh_cube(stage, "/World/mesh", 1.0)

    # The path the helper creates its particle system at, occupied by a prim of
    # another type. The traversal matches on the particle system type name only,
    # so it never sees this prim and the creation call is reached with its own
    # precondition already violated.
    taken = particles._get_default_particle_system_path(stage)
    stage.DefinePrim(taken, "Xform")
    assert not [p for p in stage.Traverse() if p.GetTypeName() == "PhysxParticleSystem"]

    with caplog.at_level("ERROR"):
        sampled = physicsUtils.poisson_sample_mesh(stage, Sdf.Path("/World/mesh"))
    assert sampled.isEmpty, sampled
    assert caplog.records, "expected a logged error, not an exception"

    # Failure means nothing was authored: neither the sampling request on the
    # mesh nor a particle system over the prim that was already there.
    mesh = stage.GetPrimAtPath("/World/mesh")
    assert "PhysxParticleSamplingAPI" not in mesh.GetAppliedSchemas()
    assert stage.GetPrimAtPath(taken).GetTypeName() == "Xform"


_OCCUPIED_PATH_HELPERS = (
    "add_physx_particle_system",
    "add_physx_particleset_points",
    "add_physx_particleset_pointinstancer",
)


def _call_with_occupied_path(stage, helper_name, path):
    """Call one of the three path-guarded particle helpers on `path`."""
    positions = [Gf.Vec3f(0.0)]
    velocities = [Gf.Vec3f(0.0)]
    if helper_name == "add_physx_particle_system":
        return particles.add_physx_particle_system(stage, path)
    if helper_name == "add_physx_particleset_points":
        return particles.add_physx_particleset_points(
            stage, path, positions, velocities, [1.0], "/World/particleSystem", True, False, 0, 1.0, 0.0
        )
    return particles.add_physx_particleset_pointinstancer(
        stage, Sdf.Path(path), positions, velocities, "/World/particleSystem", True, False, 0, 1.0, 0.0
    )


@pytest.mark.parametrize("helper_name", _OCCUPIED_PATH_HELPERS)
def test_particle_set_helpers_reject_an_occupied_path(stage, helper_name):
    """omni.physx guarded the target path with an assert, which -O compiles out."""
    occupied = UsdGeom.Cube.Define(stage, "/World/occupied").GetPrim()
    occupied.GetAttribute("size").Set(4.0)

    with pytest.raises(ValueError):
        _call_with_occupied_path(stage, helper_name, "/World/occupied")

    # Rejection means the prim that was there is untouched, which is the whole
    # of what the stripped assert cost: Define retypes it and keeps its own
    # authored properties, so the wreckage reads as a valid prim of both types.
    assert occupied.GetTypeName() == "Cube"
    assert occupied.GetAttribute("size").Get() == 4.0


def test_plane_collider_rejects_an_occupied_path(stage):
    """The helper must not retype a prim that already holds its target path."""
    occupied = UsdGeom.Cube.Define(stage, "/World/occupied").GetPrim()
    occupied.GetAttribute("size").Set(4.0)
    before = stage.GetRootLayer().ExportToString()

    with pytest.raises(ValueError, match="already held by a prim"):
        physicsUtils.add_plane_collider(stage, "/World/occupied", "Z")

    assert stage.GetRootLayer().ExportToString() == before
    assert occupied.GetTypeName() == "Cube"
    assert occupied.GetAttribute("size").Get() == 4.0
    assert not occupied.HasAPI(UsdPhysics.CollisionAPI)


def test_particle_set_path_guard_survives_optimized_python():
    """The guard has to be a statement, not an assert: -O strips the latter.

    A child interpreter is the only place this is observable -- `-O` is fixed
    when the process starts. It needs no schema registration, because the guard
    rejects before any codeless call is reached.
    """
    program = textwrap.dedent(
        """
        import sys
        from pxr import Usd, UsdGeom
        from ovphysx.utils import particles

        # Not an assert: -O would compile out the check that -O is in force.
        if __debug__:
            sys.exit("child interpreter is not running optimized")

        stage = Usd.Stage.CreateInMemory()
        prim = UsdGeom.Cube.Define(stage, "/World/occupied").GetPrim()
        for call in (
            lambda: particles.add_physx_particle_system(stage, "/World/occupied"),
            lambda: particles.add_physx_particleset_points(
                stage, "/World/occupied", [], [], [], "/World/ps", True, False, 0, 1.0, 0.0
            ),
        ):
            try:
                call()
            except ValueError:
                pass
            else:
                sys.exit("an occupied path was accepted")
            if prim.GetTypeName() != "Cube":
                sys.exit(f"the occupied prim was retyped to {prim.GetTypeName()}")
        """
    )
    completed = subprocess.run(
        [sys.executable, "-O", "-c", program],
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr


def test_auto_volume_deformable_hierarchy_reports_hex_mesh(stage, monkeypatch):
    """The fix returns False when a requested hex mesh could not be applied.

    The failure is injected at `codeless.try_apply_api`, which is the seam
    deformable.py's hexahedral apply goes through; USD itself refuses an apply
    only for reasons (an uneditable layer, an instance proxy) that would fail the
    applies before it too, so nothing on this stage can reach the branch.
    """
    stage.DefinePrim("/World/deformable", "Xform")
    physicsUtils.create_mesh_cube(stage, "/World/src", 1.0)

    assert (
        physicsUtils.create_auto_volume_deformable_hierarchy(
            stage,
            "/World/deformable",
            "/World/deformable/sim",
            "/World/deformable/collision",
            "/World/src",
            simulation_hex_mesh_enabled=True,
            cooking_src_simplification_enabled=False,
        )
        is True
    )
    assert "PhysxAutoDeformableHexahedralMeshAPI" in stage.GetPrimAtPath(
        "/World/deformable"
    ).GetAppliedSchemas()

    # A Gprim root is rejected rather than half-built.
    physicsUtils.create_mesh_cube(stage, "/World/gprim", 1.0)
    assert (
        physicsUtils.create_auto_volume_deformable_hierarchy(
            stage,
            "/World/gprim",
            "/World/gprim/sim",
            "/World/gprim/collision",
            "/World/src",
            simulation_hex_mesh_enabled=False,
            cooking_src_simplification_enabled=False,
        )
        is False
    )

    # A refused hexahedral apply reports failure instead of quietly leaving the
    # caller with the tetrahedral mesh they did not ask for. Only that one
    # identifier is refused, so every other apply in the call still runs.
    real_try_apply_api = codeless.try_apply_api

    def refuse_hex_mesh(prim, identifier, instance_name=None):
        if identifier == "PhysxAutoDeformableHexahedralMeshAPI":
            return False
        return real_try_apply_api(prim, identifier, instance_name)

    monkeypatch.setattr(codeless, "try_apply_api", refuse_hex_mesh)
    stage.DefinePrim("/World/nohex", "Xform")
    assert (
        physicsUtils.create_auto_volume_deformable_hierarchy(
            stage,
            "/World/nohex",
            "/World/nohex/sim",
            "/World/nohex/collision",
            "/World/src",
            simulation_hex_mesh_enabled=True,
            cooking_src_simplification_enabled=False,
        )
        is False
    )
    nohex = stage.GetPrimAtPath("/World/nohex")
    assert "PhysxAutoDeformableHexahedralMeshAPI" not in nohex.GetAppliedSchemas()

    # The same call without the hexahedral request still succeeds, so the False
    # above came from the hexahedral branch rather than from the patch at large.
    stage.DefinePrim("/World/tetonly", "Xform")
    assert (
        physicsUtils.create_auto_volume_deformable_hierarchy(
            stage,
            "/World/tetonly",
            "/World/tetonly/sim",
            "/World/tetonly/collision",
            "/World/src",
            simulation_hex_mesh_enabled=False,
            cooking_src_simplification_enabled=False,
        )
        is True
    )


@pytest.mark.parametrize("kind", ("volume", "surface"))
def test_auto_deformable_hierarchy_rejects_visual_without_points(stage, caplog, kind):
    """A missing default-time visual point array must fail before authoring."""
    stage.DefinePrim("/World/deformable", "Xform")
    UsdGeom.Mesh.Define(stage, "/World/deformable/visual")
    physicsUtils.create_mesh_cube(stage, "/World/src", 1.0)
    before = stage.GetRootLayer().ExportToString()

    with caplog.at_level("WARNING"):
        if kind == "volume":
            result = physicsUtils.create_auto_volume_deformable_hierarchy(
                stage,
                "/World/deformable",
                "/World/deformable/sim",
                "/World/deformable/collision",
                "/World/src",
                simulation_hex_mesh_enabled=False,
                cooking_src_simplification_enabled=False,
            )
        else:
            result = physicsUtils.create_auto_surface_deformable_hierarchy(
                stage,
                "/World/deformable",
                "/World/deformable/sim",
                "/World/src",
                cooking_src_simplification_enabled=False,
            )

    assert result is False
    assert stage.GetRootLayer().ExportToString() == before
    assert any(
        "/World/deformable/visual" in record.message and "no points at default time" in record.message
        for record in caplog.records
    )


@pytest.mark.parametrize(
    "kind, missing_property",
    (
        ("volume", "points"),
        ("volume", "tetVertexIndices"),
        ("surface", "points"),
        ("surface", "faceVertexCounts"),
        ("surface", "faceVertexIndices"),
    ),
)
def test_single_prim_deformable_rejects_missing_default_geometry(stage, caplog, kind, missing_property):
    """Required geometry must be present before deformable APIs are applied."""
    prim_path = "/World/deformable"
    if kind == "volume":
        mesh = UsdGeom.TetMesh.Define(stage, prim_path)
        if missing_property != "points":
            mesh.GetPointsAttr().Set(
                [
                    Gf.Vec3f(0, 0, 0),
                    Gf.Vec3f(1, 0, 0),
                    Gf.Vec3f(0, 1, 0),
                    Gf.Vec3f(0, 0, 1),
                ]
            )
        if missing_property != "tetVertexIndices":
            mesh.GetTetVertexIndicesAttr().Set([Gf.Vec4i(0, 1, 2, 3)])
    else:
        mesh = UsdGeom.Mesh.Define(stage, prim_path)
        if missing_property != "points":
            mesh.GetPointsAttr().Set([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(0, 1, 0)])
        if missing_property != "faceVertexCounts":
            mesh.GetFaceVertexCountsAttr().Set([3])
        if missing_property != "faceVertexIndices":
            mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2])

    before = stage.GetRootLayer().ExportToString()
    with caplog.at_level("WARNING"):
        if kind == "volume":
            result = physicsUtils.set_physics_volume_deformable_body(stage, mesh.GetPath())
        else:
            result = physicsUtils.set_physics_surface_deformable_body(stage, mesh.GetPath())

    assert result is False
    assert stage.GetRootLayer().ExportToString() == before
    assert any(
        prim_path in record.message
        and missing_property in record.message
        and "no value at default time" in record.message
        for record in caplog.records
    )


def test_api_walks_stop_at_a_real_reset_xform_stack(stage):
    """omni.physx guarded the walks on `xformOp:reset`, which USD does not define.

    A reset stack is the `!resetXformStack!` token inside the `xformOpOrder`
    array, so `HasAttribute("xformOp:reset")` was False on a prim that really had
    one and both walks ran straight through the boundary their docstrings promise
    to respect. The spelling is pinned here as well as the behaviour, because a
    guard that reads plausibly is what let this survive unnoticed.
    """
    mid = UsdGeom.Xform.Define(stage, "/Reset/mid")
    leaf = UsdGeom.Xform.Define(stage, "/Reset/mid/leaf")
    root = UsdGeom.Xform.Define(stage, "/Reset")
    mid.AddTranslateOp().Set(Gf.Vec3d(1, 2, 3))
    assert mid.SetResetXformStack(True)

    order = mid.GetPrim().GetAttribute("xformOpOrder").Get()
    assert list(order) == ["!resetXformStack!", "xformOp:translate"]
    assert not mid.GetPrim().HasAttribute("xformOp:reset")
    assert UsdGeom.Xformable(mid.GetPrim()).GetResetXformStack()

    # Ascent: the API sits above the boundary, so the leaf must not see it.
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    assert not physicsUtils.ancestor_has_api(UsdPhysics.RigidBodyAPI, leaf.GetPrim())

    # Descent: the API sits below the boundary, so the root must not see it.
    down = UsdGeom.Xform.Define(stage, "/Down")
    down_mid = UsdGeom.Xform.Define(stage, "/Down/mid")
    down_leaf = UsdGeom.Xform.Define(stage, "/Down/mid/leaf")
    down_mid.AddTranslateOp().Set(Gf.Vec3d(1, 2, 3))
    down_mid.SetResetXformStack(True)
    UsdPhysics.RigidBodyAPI.Apply(down_leaf.GetPrim())
    assert not physicsUtils.descendant_has_api(UsdPhysics.RigidBodyAPI, down.GetPrim())

    # The same two hierarchies without the reset, so the negatives above came
    # from the boundary rather than from the walks failing to find anything.
    plain = UsdGeom.Xform.Define(stage, "/Plain")
    UsdGeom.Xform.Define(stage, "/Plain/mid")
    plain_leaf = UsdGeom.Xform.Define(stage, "/Plain/mid/leaf")
    UsdPhysics.RigidBodyAPI.Apply(plain.GetPrim())
    assert physicsUtils.ancestor_has_api(UsdPhysics.RigidBodyAPI, plain_leaf.GetPrim())
    assert physicsUtils.descendant_has_api(UsdPhysics.RigidBodyAPI, plain.GetPrim())

    # A valid prim that is not Xformable reaches the guard and must not raise.
    scope = stage.DefinePrim("/Scope", "Scope")
    UsdPhysics.RigidBodyAPI.Apply(stage.DefinePrim("/Scope/child", "Xform"))
    assert physicsUtils.descendant_has_api(UsdPhysics.RigidBodyAPI, scope)


def _reference_extent(radius, height, axis, caps):
    """The extent a capsule, cylinder or cone really occupies, re-derived here.

    A cylinder and a cone are centred on the origin and reach ``height / 2``
    either side of it along ``axis``; a capsule adds a hemispherical cap of
    ``radius`` past each end of that. Across the axis all three reach
    ``radius``. Spelled out rather than taken from USD, so the case has a
    statement of its own to compare against.
    """
    along = radius + height / 2.0 if caps else height / 2.0
    reach = [radius, radius, radius]
    reach["XYZ".index(axis)] = along
    return [tuple(-v for v in reach), tuple(reach)]


def _usd_computed_bound(stage, cls, radius, height, axis):
    """USD's own local bound for the same shape, with no extent authored.

    An `extent` opinion is the first thing a bound query reads, so this defines
    the shape on a throwaway prim and leaves the attribute alone -- USD then
    computes the bound from the schema, which is the independent answer the
    authored extent has to agree with.
    """
    gprim = cls.Define(stage, f"/Reference/{cls.__name__}_{axis}")
    gprim.CreateRadiusAttr(radius)
    gprim.CreateHeightAttr(height)
    gprim.CreateAxisAttr(axis)
    assert not gprim.GetPrim().GetAttribute("extent").HasAuthoredValue()
    bound = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]
    ).ComputeLocalBound(gprim.GetPrim())
    aligned = bound.ComputeAlignedRange()
    return [tuple(aligned.GetMin()), tuple(aligned.GetMax())]


@pytest.mark.parametrize("axis", ("X", "Y", "Z"))
@pytest.mark.parametrize(
    "shape, add, cls, caps",
    (
        ("capsule", physicsUtils.add_capsule, UsdGeom.Capsule, True),
        ("cylinder", physicsUtils.add_cylinder, UsdGeom.Cylinder, False),
        ("cone", physicsUtils.add_cone, UsdGeom.Cone, False),
    ),
)
def test_shape_extents_are_axis_aware_and_the_right_size(stage, axis, shape, add, cls, caps):
    """omni.physx authored (-r, -r, -h) .. (r, r, h) for all three of these.

    That expression is wrong three ways at once: it is laid out as though the
    shape always ran along Z, so it ignores `axis`; it uses the full `height`
    where a cylinder and a cone reach only half of it either side of centre; and
    it drops the capsule's two hemispherical caps, which reach a further
    `radius` past each end. For radius 0.25, height 1.5 and axis Y it authored
    (-0.25, -0.25, -1.5) .. (0.25, 0.25, 1.5) for each of the three.
    """
    radius, height = 0.25, 1.5
    prim = add(stage, f"/World/{shape}", radius, height, axis)

    authored = prim.GetAttribute("extent").Get()
    assert authored, "the helper has to author an extent, not leave it to a consumer"
    authored = [tuple(p) for p in authored]

    expected = _reference_extent(radius, height, axis, caps)
    assert authored == [pytest.approx(p) for p in expected], (shape, axis, authored)

    # And it agrees with the bound USD computes for the same shape from the
    # schema alone, so the expectation above is not this test agreeing with
    # itself about geometry.
    assert authored == [
        pytest.approx(p) for p in _usd_computed_bound(stage, cls, radius, height, axis)
    ], (shape, axis, authored)

    # The old expression, pinned so this cannot pass against it. It differs on
    # every axis: along Z it is the wrong size, elsewhere the wrong shape.
    old = [(-radius, -radius, -height), (radius, radius, height)]
    assert authored != [pytest.approx(p) for p in old], (shape, axis, authored)


def _assert_subtree_has_no_deformable_state(root):
    """No prim below `root` keeps a deformable schema or an unowned property.

    Matches on the schema naming families and the property namespaces rather
    than on a list of names, so an API added to the deformable schemas and
    missed by the teardown fails here. `codeless.remove_api` drops the
    `apiSchemas` entry and leaves the properties authored, so the properties are
    checked separately from the schemas.

    A deformable *material* API is not a body API and the teardown leaves it, so
    the schema match excludes it rather than reading it as a leak.

    An `omniphysics:` property a still-applied API declares is that API's, not
    the removal's: `OmniPhysicsBodyAPI` shares three with
    `OmniPhysicsDeformableBodyAPI` and a caller can apply it in its own right.
    """
    for prim in Usd.PrimRange(root, Usd.PrimAllPrimsPredicate):
        applied = [
            name
            for name in prim.GetAppliedSchemas()
            if "Deformable" in name and "MaterialAPI" not in name
        ]
        assert not applied, (prim.GetPath(), prim.GetAppliedSchemas())

        owned = set()
        for name in prim.GetAppliedSchemas():
            owned.update(get_schema_property_names(name.split(":", 1)[0]))
        leftover = [
            prop.GetName()
            for prop in prim.GetAuthoredProperties()
            if prop.GetName().startswith(("deformablePose:", "physxDeformableBody:"))
            or (prop.GetName().startswith("omniphysics:") and prop.GetName() not in owned)
        ]
        assert not leftover, (prim.GetPath(), leftover)


def test_remove_deformable_body_cleans_the_whole_hierarchy(stage):
    """omni.physx iterated GetChildren(), so a nested skin prim kept its pose.

    The two `create_auto_*_deformable_hierarchy` helpers apply
    `OmniPhysicsDeformablePoseAPI` to every `UsdGeom.PointBased` prim in the
    root's subtree at any depth, so the removal has to walk the same range they
    author over rather than one level of it.
    """
    stage.DefinePrim("/World/deformable", "Xform")
    stage.DefinePrim("/World/deformable/group", "Xform")
    physicsUtils.create_mesh_cube(stage, "/World/deformable/group/skin", 1.0)
    physicsUtils.create_mesh_cube(stage, "/World/src", 1.0)

    assert (
        physicsUtils.create_auto_volume_deformable_hierarchy(
            stage,
            "/World/deformable",
            "/World/deformable/sim",
            "/World/deformable/collision",
            "/World/src",
            simulation_hex_mesh_enabled=False,
            cooking_src_simplification_enabled=False,
        )
        is True
    )

    # The prim the defect was reproduced on: a skinned visual mesh two levels
    # below the root, which the children-only walk never reached.
    root = stage.GetPrimAtPath("/World/deformable")
    skin = stage.GetPrimAtPath("/World/deformable/group/skin")
    assert "OmniPhysicsDeformablePoseAPI:default" in skin.GetAppliedSchemas()
    assert skin.GetAttribute("deformablePose:default:omniphysics:points").HasAuthoredValue()
    assert skin.GetAttribute("deformablePose:default:omniphysics:purposes").HasAuthoredValue()

    physicsUtils.remove_deformable_body(stage, "/World/deformable")

    _assert_subtree_has_no_deformable_state(root)

    # An exported layer is where a surviving opinion outlives the process. No
    # bind-pose property reaches it, and every deformable API name it still
    # mentions is mentioned by a `delete apiSchemas` opinion -- which is the
    # form the removal has to take -- rather than by a `prepend` one.
    exported = stage.GetRootLayer().ExportToString()
    assert "deformablePose" not in exported, exported
    prepends = [
        line for line in exported.splitlines()
        if "prepend apiSchemas" in line and "Deformable" in line
    ]
    assert not prepends, prepends
    assert "delete apiSchemas" in exported, exported


def test_remove_deformable_body_includes_the_prim_it_was_given(stage):
    """The children-only walk skipped the supplied prim as well as the deep ones.

    A caller may hand this the simulation mesh's own path rather than the root's
    -- that prim carries both `OmniPhysicsVolumeDeformableSimAPI` and a pose
    instance, and the original removed neither, because it only ever looked at
    children.
    """
    stage.DefinePrim("/World/deformable", "Xform")
    physicsUtils.create_mesh_cube(stage, "/World/src", 1.0)
    assert physicsUtils.create_auto_volume_deformable_hierarchy(
        stage,
        "/World/deformable",
        "/World/deformable/sim",
        "/World/deformable/collision",
        "/World/src",
        simulation_hex_mesh_enabled=False,
        cooking_src_simplification_enabled=False,
    )

    sim = stage.GetPrimAtPath("/World/deformable/sim")
    assert "OmniPhysicsVolumeDeformableSimAPI" in sim.GetAppliedSchemas()
    assert "OmniPhysicsDeformablePoseAPI:default" in sim.GetAppliedSchemas()

    physicsUtils.remove_deformable_body(stage, sim.GetPath())

    assert "OmniPhysicsVolumeDeformableSimAPI" not in sim.GetAppliedSchemas()
    assert "OmniPhysicsDeformablePoseAPI:default" not in sim.GetAppliedSchemas()
    assert not [
        prop.GetName()
        for prop in sim.GetAuthoredProperties()
        if prop.GetName().startswith("deformablePose:")
    ]

    # The root above it is untouched, so the sweep did not climb out of the
    # subtree it was given.
    root = stage.GetPrimAtPath("/World/deformable")
    assert "PhysxAutoDeformableBodyAPI" in root.GetAppliedSchemas()


def _build_deformable_hierarchy(
    stage,
    kind,
    root_path,
    cooking_src_path,
    simulation_name="sim",
    collision_name="collision",
):
    """Build one auto deformable hierarchy, returning its simulation mesh path.

    `collision_name` names the volume layout's separate collision mesh. Passing
    the same name for both builds the merged layout, where one tet mesh is the
    simulation and the collision geometry at once.
    """
    simulation_path = f"{root_path}/{simulation_name}"
    if kind == "volume":
        built = physicsUtils.create_auto_volume_deformable_hierarchy(
            stage,
            root_path,
            simulation_path,
            f"{root_path}/{collision_name}",
            cooking_src_path,
            simulation_hex_mesh_enabled=False,
            cooking_src_simplification_enabled=False,
        )
    else:
        built = physicsUtils.create_auto_surface_deformable_hierarchy(
            stage,
            root_path,
            simulation_path,
            cooking_src_path,
            cooking_src_simplification_enabled=False,
        )
    assert built is True, kind
    return simulation_path


def _add_unrelated_child_collider(stage, root_path):
    """A collider a caller authored inside a deformable root's subtree.

    Point-based on purpose: the `create_auto_*` bind-pose pass visits every
    `UsdGeom.PointBased` prim below the root.
    """
    collider = physicsUtils.create_mesh_cube(stage, f"{root_path}/rock", 1.0).GetPrim()
    UsdPhysics.CollisionAPI.Apply(collider)
    collider.GetAttribute("physics:collisionEnabled").Set(False)
    return collider


def _define_unit_tet_mesh(stage, path):
    """A one-tetrahedron `UsdGeom.TetMesh`, the smallest volume body input."""
    tet_mesh = UsdGeom.TetMesh.Define(stage, path)
    tet_mesh.GetPointsAttr().Set(
        [Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 0, 1)]
    )
    tet_mesh.GetTetVertexIndicesAttr().Set([Gf.Vec4i(0, 1, 2, 3)])
    return tet_mesh.GetPrim()


def _assert_child_collider_intact(collider):
    assert collider.HasAPI(UsdPhysics.CollisionAPI), collider.GetAppliedSchemas()
    enabled = collider.GetAttribute("physics:collisionEnabled")
    assert enabled.HasAuthoredValue(), collider.GetAuthoredProperties()
    assert enabled.Get() is False


@pytest.mark.parametrize("kind", ["volume", "surface"])
def test_remove_deformable_body_keeps_an_unrelated_child_collider(stage, kind):
    """A collider on a prim with no deformable simulation API keeps it (REQ AC-8).

    The properties authored under a caller's collision API survive the removal.
    """
    stage.DefinePrim("/World/deformable", "Xform")
    physicsUtils.create_mesh_cube(stage, "/World/src", 1.0)
    collider = _add_unrelated_child_collider(stage, "/World/deformable")
    simulation_path = _build_deformable_hierarchy(
        stage, kind, "/World/deformable", "/World/src"
    )

    physicsUtils.remove_deformable_body(stage, "/World/deformable")

    _assert_child_collider_intact(collider)
    exported = stage.GetRootLayer().ExportToString()
    assert "physics:collisionEnabled" in exported, exported

    simulation = stage.GetPrimAtPath(simulation_path)
    assert not [
        name for name in simulation.GetAppliedSchemas() if "Deformable" in name
    ], simulation.GetAppliedSchemas()
    if kind == "surface":
        # The surface helper gives the simulation mesh both APIs, so the
        # simulation API is the ownership evidence for its collision API.
        assert not simulation.HasAPI(UsdPhysics.CollisionAPI), simulation.GetAppliedSchemas()
    else:
        # The volume layout's collision mesh carries no simulation API, so a
        # direct removal cannot tell it from a caller's collider (REQ AC-8).
        collision = stage.GetPrimAtPath("/World/deformable/collision")
        assert collision.HasAPI(UsdPhysics.CollisionAPI), collision.GetAppliedSchemas()


@pytest.mark.parametrize("kind", ["volume", "surface"])
def test_rebuilding_a_deformable_hierarchy_keeps_an_unrelated_child_collider(stage, kind):
    """Both `create_auto_*` helpers run that teardown before they build."""
    stage.DefinePrim("/World/deformable", "Xform")
    physicsUtils.create_mesh_cube(stage, "/World/src", 1.0)
    collider = _add_unrelated_child_collider(stage, "/World/deformable")

    _build_deformable_hierarchy(stage, kind, "/World/deformable", "/World/src")
    _assert_child_collider_intact(collider)

    _build_deformable_hierarchy(stage, kind, "/World/deformable", "/World/src")
    _assert_child_collider_intact(collider)


def test_removing_a_merged_volume_body_takes_collision_from_its_one_mesh(stage):
    """`collision_tetmesh_path` may equal `simulation_tetmesh_path` (REQ AC-8).

    That layout gives one tet mesh the volume simulation API and a collision API
    the helper itself applied, so a removal takes both.
    """
    stage.DefinePrim("/World/deformable", "Xform")
    physicsUtils.create_mesh_cube(stage, "/World/src", 1.0)
    simulation_path = _build_deformable_hierarchy(
        stage, "volume", "/World/deformable", "/World/src", collision_name="sim"
    )
    merged = stage.GetPrimAtPath(simulation_path)
    assert merged.HasAPI(UsdPhysics.CollisionAPI), merged.GetAppliedSchemas()

    physicsUtils.remove_deformable_body(stage, "/World/deformable")

    assert not merged.GetAppliedSchemas(), merged.GetAppliedSchemas()


@pytest.mark.parametrize("kind", ["volume", "surface"])
def test_removing_a_body_takes_collision_a_caller_put_on_its_sim_mesh(stage, kind):
    """The cost AC-8 accepts: the simulation geometry's collider is not exempt.

    This prim is indistinguishable from the merged volume layout's one mesh,
    where the helper applied the collision API itself.
    """
    stage.DefinePrim("/World/deformable", "Xform")
    physicsUtils.create_mesh_cube(stage, "/World/src", 1.0)
    simulation_path = _build_deformable_hierarchy(
        stage, kind, "/World/deformable", "/World/src"
    )
    simulation = stage.GetPrimAtPath(simulation_path)
    UsdPhysics.CollisionAPI.Apply(simulation)
    simulation.GetAttribute("physics:collisionEnabled").Set(False)

    physicsUtils.remove_deformable_body(stage, "/World/deformable")

    assert not simulation.HasAPI(UsdPhysics.CollisionAPI), simulation.GetAppliedSchemas()
    assert not simulation.GetAttribute("physics:collisionEnabled").HasAuthoredValue()


@pytest.mark.parametrize("kind", ["volume", "surface"])
def test_rebuilding_at_a_new_mesh_path_leaves_no_collider_at_the_old_one(stage, kind):
    """A rebuild moves the simulation mesh; its collision API must not stay put.

    A collider left at the abandoned path is a second enabled collider in the
    subtree. The runtime binds whichever one its traversal reaches first, and
    rejects a surface body whose collider is not its simulation mesh.
    """
    stage.DefinePrim("/World/deformable", "Xform")
    physicsUtils.create_mesh_cube(stage, "/World/src", 1.0)

    def build(simulation_name):
        # The volume layout is built merged, since a separate collision mesh
        # carries no simulation API and so is never the helpers' to take.
        _build_deformable_hierarchy(
            stage,
            kind,
            "/World/deformable",
            "/World/src",
            simulation_name,
            collision_name=simulation_name,
        )

    build("simA")
    abandoned = stage.GetPrimAtPath("/World/deformable/simA")
    assert abandoned.HasAPI(UsdPhysics.CollisionAPI), abandoned.GetAppliedSchemas()
    # The rebuild's bind-pose pass reads this prim's points, which a generated
    # mesh only gains at cooking time.
    UsdGeom.PointBased(abandoned).GetPointsAttr().Set([Gf.Vec3f(0, 0, 0)])

    build("simB")

    assert not abandoned.HasAPI(UsdPhysics.CollisionAPI), abandoned.GetAppliedSchemas()
    rebuilt = stage.GetPrimAtPath("/World/deformable/simB")
    assert rebuilt.HasAPI(UsdPhysics.CollisionAPI), rebuilt.GetAppliedSchemas()


def test_rebuilding_a_moved_collision_path_leaves_its_collider(stage):
    """A volume layout's separate collision mesh is not attributable (REQ AC-8).

    It is the one collider these helpers author beside no simulation API, and
    nothing a rebuild can read names the layout its previous run authored, so a
    rebuild given a new collision path leaves the old collider enabled.
    """
    stage.DefinePrim("/World/deformable", "Xform")
    physicsUtils.create_mesh_cube(stage, "/World/src", 1.0)

    def build(suffix):
        _build_deformable_hierarchy(
            stage,
            "volume",
            "/World/deformable",
            "/World/src",
            f"sim{suffix}",
            collision_name=f"collision{suffix}",
        )

    build("A")
    abandoned = stage.GetPrimAtPath("/World/deformable/collisionA")
    assert abandoned.HasAPI(UsdPhysics.CollisionAPI), abandoned.GetAppliedSchemas()
    assert not any(
        "DeformableSim" in name for name in abandoned.GetAppliedSchemas()
    ), abandoned.GetAppliedSchemas()
    # The rebuild's bind-pose pass reads the points of every prim it does not
    # generate, and a generated mesh only gains them at cooking time.
    for name in ("simA", "collisionA"):
        points = UsdGeom.PointBased(stage.GetPrimAtPath(f"/World/deformable/{name}"))
        points.GetPointsAttr().Set([Gf.Vec3f(0, 0, 0)])

    build("B")

    assert abandoned.HasAPI(UsdPhysics.CollisionAPI), abandoned.GetAppliedSchemas()
    assert abandoned.GetAttribute("physics:collisionEnabled").Get() is True
    rebuilt = stage.GetPrimAtPath("/World/deformable/collisionB")
    assert rebuilt.HasAPI(UsdPhysics.CollisionAPI), rebuilt.GetAppliedSchemas()
    # The abandoned simulation mesh is attributable and does lose its APIs.
    old_simulation = stage.GetPrimAtPath("/World/deformable/simA")
    assert not any(
        "DeformableSim" in name for name in old_simulation.GetAppliedSchemas()
    ), old_simulation.GetAppliedSchemas()


def test_removing_a_root_cleans_a_whole_body_authored_below_the_children(stage):
    """Every prim of the subtree is torn down the way the supplied prim is (REQ AC-8).

    `set_physics_volume_deformable_body` applies the body, simulation and
    collision APIs to whatever path it is given, so a caller can author a
    standalone body deeper than an immediate child and remove a root above it.
    The auto-hierarchy APIs are applied here too, since a removal that reaches a
    prim has to take the whole family from it rather than part of it.
    """
    root = stage.DefinePrim("/World/deformable", "Xform")
    stage.DefinePrim("/World/deformable/group", "Xform")
    deep = _define_unit_tet_mesh(stage, "/World/deformable/group/tet")

    assert physicsUtils.set_physics_volume_deformable_body(stage, deep.GetPath()) is True
    for api in (
        "PhysxAutoDeformableBodyAPI",
        "PhysxAutoDeformableHexahedralMeshAPI",
        "PhysxAutoDeformableMeshSimplificationAPI",
    ):
        assert codeless.try_apply_api(deep, api), api
    codeless.set_attr(deep, "physxDeformableBody:autoDeformableBodyEnabled", True)
    assert deep.HasAPI(UsdPhysics.CollisionAPI), deep.GetAppliedSchemas()

    physicsUtils.remove_deformable_body(stage, root.GetPath())

    _assert_subtree_has_no_deformable_state(root)
    # `OmniPhysicsBodyAPI` names no deformable family, so the general oracle
    # cannot see it. It is a built-in of `OmniPhysicsDeformableBodyAPI` and has
    # to go when that one does.
    assert not deep.GetAppliedSchemas(), deep.GetAppliedSchemas()
    assert not deep.GetAttribute("physics:collisionEnabled").HasAuthoredValue()


def test_removing_a_body_takes_a_property_its_api_no_longer_carries(stage):
    """A property orphaned by an earlier partial teardown is still taken (REQ AC-8).

    `codeless.remove_api` drops the `apiSchemas` entry and leaves the properties
    authored, so a prim can hold a deformable property with no deformable API. A
    strip conditioned on the API being applied walks past it.
    """
    root = stage.DefinePrim("/World/deformable", "Xform")
    stage.DefinePrim("/World/deformable/group", "Xform")
    orphan = stage.DefinePrim("/World/deformable/group/orphan", "Xform")
    for prim in (root, orphan):
        prim.CreateAttribute("omniphysics:deformableBodyEnabled", Sdf.ValueTypeNames.Bool).Set(True)
        prim.CreateAttribute("omniphysics:mass", Sdf.ValueTypeNames.Float).Set(3.0)
        assert not prim.GetAppliedSchemas(), prim.GetAppliedSchemas()

    physicsUtils.remove_deformable_body(stage, root.GetPath())

    _assert_subtree_has_no_deformable_state(root)


def test_removing_a_body_keeps_a_body_api_the_caller_applied_itself(stage):
    """`OmniPhysicsBodyAPI` outlives the removal when the caller owns it (REQ AC-8).

    It is a built-in of `OmniPhysicsDeformableBodyAPI` and declares three of its
    properties, `omniphysics:simulationOwner` being a relationship rather than an
    attribute. Staying applied after that API is removed is what tells a caller's
    own application from the built-in, and all three properties go with it.
    """
    scene = UsdPhysics.Scene.Define(stage, "/World/scene").GetPrim()
    root = stage.DefinePrim("/World/deformable", "Xform")
    stage.DefinePrim("/World/deformable/group", "Xform")
    deep = stage.DefinePrim("/World/deformable/group/body", "Xform")
    for prim in (root, deep):
        assert codeless.try_apply_api(prim, "OmniPhysicsBodyAPI")
        assert codeless.try_apply_api(prim, "OmniPhysicsDeformableBodyAPI")
        codeless.set_attr(prim, "omniphysics:kinematicEnabled", True)
        codeless.set_attr(prim, "omniphysics:startsAsleep", True)
        prim.CreateRelationship("omniphysics:simulationOwner").SetTargets([scene.GetPath()])
        codeless.set_attr(prim, "omniphysics:mass", 7.0)

    physicsUtils.remove_deformable_body(stage, root.GetPath())

    _assert_subtree_has_no_deformable_state(root)
    for prim in (root, deep):
        assert prim.GetAppliedSchemas() == ["OmniPhysicsBodyAPI"], prim.GetAppliedSchemas()
        assert codeless.get_attr(prim, "omniphysics:kinematicEnabled").Get() is True
        assert codeless.get_attr(prim, "omniphysics:startsAsleep").Get() is True
        owner = prim.GetRelationship("omniphysics:simulationOwner")
        assert owner.GetTargets() == [scene.GetPath()], owner.GetTargets()
        # `omniphysics:mass` is the deformable body API's own, so it still goes.
        assert not prim.GetAttribute("omniphysics:mass").HasAuthoredValue()


def test_removing_a_body_touches_only_authored_properties(stage, monkeypatch):
    """The strip is offered no name a layer did not author (REQ AC-8).

    `omniphysics:kinematicEnabled` is declared by the API being removed and left
    at its schema default, and `physxDeformableBodyExtra` only looks like the
    namespace the strip takes. Neither reaches `RemoveProperty`.
    """
    root = stage.DefinePrim("/World/deformable", "Xform")
    body = stage.DefinePrim("/World/deformable/body", "Xform")
    assert codeless.try_apply_api(body, "OmniPhysicsDeformableBodyAPI")
    codeless.set_attr(body, "omniphysics:mass", 5.0)
    body.CreateAttribute("physxDeformableBody:autoDeformableBodyEnabled", Sdf.ValueTypeNames.Bool).Set(True)
    body.CreateAttribute("physxDeformableBodyExtra:keepMe", Sdf.ValueTypeNames.Bool).Set(True)
    assert not body.GetAttribute("omniphysics:kinematicEnabled").HasAuthoredValue()

    offered = []
    original = Usd.Prim.RemoveProperty
    monkeypatch.setattr(
        Usd.Prim, "RemoveProperty", lambda self, name: (offered.append(name), original(self, name))[1]
    )

    physicsUtils.remove_deformable_body(stage, root.GetPath())

    assert sorted(offered) == ["omniphysics:mass", "physxDeformableBody:autoDeformableBodyEnabled"]
    assert body.GetAttribute("physxDeformableBodyExtra:keepMe").Get() is True
    _assert_subtree_has_no_deformable_state(root)


def test_removing_a_body_takes_the_curves_simulation_api(stage):
    """The third simulation API is torn down like the other two (REQ AC-8).

    No helper here applies `OmniPhysicsCurvesDeformableSimAPI` -- the schema calls
    curves a work in progress -- but the teardown covers every deformable API, not
    only the ones this module authors.
    """
    root = stage.DefinePrim("/World/deformable", "Xform")
    stage.DefinePrim("/World/deformable/group", "Xform")
    curves = stage.DefinePrim("/World/deformable/group/curves", "Xform")
    assert codeless.try_apply_api(curves, "OmniPhysicsCurvesDeformableSimAPI")
    codeless.set_attr(curves, "omniphysics:restCrvVtxIndices", Vt.IntArray([0, 1]))
    codeless.set_attr(curves, "omniphysics:restShapePoints", Vt.Vec3fArray([Gf.Vec3f(0, 0, 0)]))
    codeless.set_attr(curves, "omniphysics:restNormals", Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)]))

    physicsUtils.remove_deformable_body(stage, root.GetPath())

    assert not curves.GetAppliedSchemas(), curves.GetAppliedSchemas()
    _assert_subtree_has_no_deformable_state(root)


def test_removing_a_body_that_only_inherits_the_body_api_takes_it(stage):
    """The same `OmniPhysicsBodyAPI` goes when it is only a built-in (REQ AC-8).

    Pins the other side of the ownership test, so keeping a caller's application
    cannot pass by keeping every application.
    """
    root = stage.DefinePrim("/World/deformable", "Xform")
    assert codeless.try_apply_api(root, "OmniPhysicsDeformableBodyAPI")
    codeless.set_attr(root, "omniphysics:kinematicEnabled", True)
    assert "OmniPhysicsBodyAPI" in root.GetAppliedSchemas(), root.GetAppliedSchemas()

    physicsUtils.remove_deformable_body(stage, root.GetPath())

    assert not root.GetAppliedSchemas(), root.GetAppliedSchemas()
    _assert_subtree_has_no_deformable_state(root)


def test_a_failed_build_keeps_collision_at_a_path_it_meant_to_reuse(stage):
    """No return after the teardown reapplies the collision API (REQ AC-8).

    A `UsdGeom.Camera` root passes the Imageable-and-not-Gprim check and is then
    refused by the mesh simplification step, so the build aborts between the
    teardown and the collision API it would have applied.
    """
    stage.DefinePrim("/World/cam", "Camera")
    physicsUtils.create_mesh_cube(stage, "/World/src", 1.0)
    simulation = UsdGeom.TetMesh.Define(stage, "/World/cam/sim").GetPrim()
    assert codeless.try_apply_api(simulation, "OmniPhysicsVolumeDeformableSimAPI")
    UsdPhysics.CollisionAPI.Apply(simulation)
    simulation.GetAttribute("physics:collisionEnabled").Set(False)

    assert physicsUtils.create_auto_volume_deformable_hierarchy(
        stage,
        "/World/cam",
        "/World/cam/sim",
        "/World/cam/collision",
        "/World/src",
        simulation_hex_mesh_enabled=False,
        cooking_src_simplification_enabled=True,
    ) is False

    _assert_child_collider_intact(simulation)


@pytest.mark.parametrize(
    "kind, generated_path, define",
    (
        ("volume", "/World/deformable/collision", UsdGeom.TetMesh.Define),
        ("surface", "/World/deformable/sim", UsdGeom.Mesh.Define),
    ),
)
def test_building_a_hierarchy_keeps_collision_authored_on_a_generated_prim(
    stage, kind, generated_path, define
):
    """A caller may author collision on a prim it hands a helper to generate.

    The build reapplies `UsdPhysics.CollisionAPI` at these paths, so the teardown
    exempts them: taking it there would delete the properties authored under it
    and give back the API alone (REQ AC-8).
    """
    stage.DefinePrim("/World/deformable", "Xform")
    physicsUtils.create_mesh_cube(stage, "/World/src", 1.0)
    generated = define(stage, generated_path).GetPrim()
    UsdPhysics.CollisionAPI.Apply(generated)
    generated.GetAttribute("physics:collisionEnabled").Set(False)

    _build_deformable_hierarchy(stage, kind, "/World/deformable", "/World/src")
    _assert_child_collider_intact(generated)

    _build_deformable_hierarchy(stage, kind, "/World/deformable", "/World/src")
    _assert_child_collider_intact(generated)


def test_removing_a_single_prim_volume_body_strips_its_collision_api(stage):
    """The one volume layout where the simulation API does mark ownership.

    `set_physics_volume_deformable_body` applies the collision API to the prim it
    is given, so a removal asked for that prim takes it back (REQ AC-8).
    """
    prim = _define_unit_tet_mesh(stage, "/World/tet")

    assert physicsUtils.set_physics_volume_deformable_body(stage, prim.GetPath()) is True
    assert prim.HasAPI(UsdPhysics.CollisionAPI), prim.GetAppliedSchemas()

    physicsUtils.remove_deformable_body(stage, prim.GetPath())

    assert not prim.HasAPI(UsdPhysics.CollisionAPI), prim.GetAppliedSchemas()


def test_verify_tetra_mesh_rejects_a_negative_index(caplog):
    """omni.physx tested `i >= len(points)` only, so a negative index passed.

    Python then indexes from the end instead of raising, so `-1` addressed a
    real point and the volume check that follows saw a well-formed tetrahedron.
    The five points here are chosen so that it does: the wrapped tetrahedron has
    a positive volume, which is what makes the range check the only thing in the
    helper that can object to it.
    """
    points = [
        Gf.Vec3f(0, 0, 0),
        Gf.Vec3f(1, 0, 0),
        Gf.Vec3f(0, 1, 0),
        Gf.Vec3f(0, 0, 1),
        Gf.Vec3f(-1, -1, -1),
    ]
    good = [0, 1, 2, 3]
    assert physicsUtils.calculate_tetra_volume(*[points[i] for i in good]) > 0.0
    with caplog.at_level("WARNING"):
        physicsUtils.verify_tetra_mesh(points, good)
    assert not caplog.records, [record.message for record in caplog.records]

    wrapped = [-1, 1, 2, 3]
    assert points[wrapped[0]] == points[4], "the negative index has to reach a real point"
    assert physicsUtils.calculate_tetra_volume(*[points[i] for i in wrapped]) > 0.0

    caplog.clear()
    with caplog.at_level("WARNING"):
        physicsUtils.verify_tetra_mesh(points, wrapped)
    assert len(caplog.records) == 1, [record.message for record in caplog.records]
    assert "invalid index -1" in caplog.records[0].message

    # Same warning voice and the same early return as an index past the end,
    # which is the behaviour the negative one is being brought in line with.
    caplog.clear()
    over = [len(points), 1, 2, 3]
    with caplog.at_level("WARNING"):
        physicsUtils.verify_tetra_mesh(points, over)
    assert len(caplog.records) == 1, [record.message for record in caplog.records]
    assert f"invalid index {len(points)}" in caplog.records[0].message
    assert physicsUtils.verify_tetra_mesh(points, wrapped) is None


@pytest.mark.parametrize(
    "shape, add_rigid, add_collider",
    (
        ("box", physicsUtils.add_rigid_box, physicsUtils.add_collider_box),
        ("cube", physicsUtils.add_rigid_cube, physicsUtils.add_collider_cube),
        ("sphere", physicsUtils.add_rigid_sphere, physicsUtils.add_collider_sphere),
        ("capsule", physicsUtils.add_rigid_capsule, physicsUtils.add_collider_capsule),
        ("cylinder", physicsUtils.add_rigid_cylinder, physicsUtils.add_collider_cylinder),
        ("cone", physicsUtils.add_rigid_cone, physicsUtils.add_collider_cone),
    ),
)
def test_add_rigid_shapes_apply_the_apis_at_zero_density(stage, shape, add_rigid, add_collider):
    """Every density gets the rigid body and mass APIs, zero included.

    `add_collider_*` is the static-collider spelling.
    """
    prim = add_rigid(stage, f"/World/{shape}_zero", density=0.0)

    assert prim.HasAPI(UsdPhysics.RigidBodyAPI), prim.GetAppliedSchemas()
    assert prim.HasAPI(UsdPhysics.MassAPI), prim.GetAppliedSchemas()
    assert prim.HasAPI(UsdPhysics.CollisionAPI), prim.GetAppliedSchemas()

    density = prim.GetAttribute("physics:density")
    assert density.HasAuthoredValue()
    assert density.Get() == pytest.approx(0.0)
    assert prim.GetAttribute("physics:velocity").HasAuthoredValue()
    assert prim.GetAttribute("physics:angularVelocity").HasAuthoredValue()

    other = add_rigid(stage, f"/World/{shape}_dense", density=7.0)
    assert other.HasAPI(UsdPhysics.RigidBodyAPI)
    assert other.GetAttribute("physics:density").Get() == pytest.approx(7.0)

    static = add_collider(stage, f"/World/{shape}_static")
    assert static.HasAPI(UsdPhysics.CollisionAPI)
    assert not static.HasAPI(UsdPhysics.RigidBodyAPI)
    assert not static.HasAPI(UsdPhysics.MassAPI)


def test_extract_triangle_surface_counts_occurrences_rather_than_parity():
    """The surface is the faces occurring exactly once, over any index list."""
    points = [
        Gf.Vec3f(0.0, 0.0, 0.0),
        Gf.Vec3f(1.0, 0.0, 0.0),
        Gf.Vec3f(0.0, 1.0, 0.0),
        Gf.Vec3f(0.0, 0.0, 1.0),
        Gf.Vec3f(0.0, 0.0, -1.0),
        Gf.Vec3f(1.0, 1.0, 1.0),
    ]
    shared = (0, 1, 2)
    three_tetrahedra = [*shared, 3, *shared, 4, *shared, 5]

    surface_points, surface_indices = physicsUtils.extract_triangle_surface_from_tetra(
        points, three_tetrahedra
    )
    assert len(surface_indices) == 9 * 3, len(surface_indices) // 3

    # The helper reindexes its output, so a face is identified by position.
    to_input = {
        out: inp
        for out, point in enumerate(surface_points)
        for inp, original in enumerate(points)
        if tuple(point) == tuple(original)
    }
    faces = {
        tuple(sorted(to_input[i] for i in surface_indices[at : at + 3]))
        for at in range(0, len(surface_indices), 3)
    }
    assert shared not in faces, sorted(faces)
    assert len(faces) == 9, sorted(faces)

    _, two_indices = physicsUtils.extract_triangle_surface_from_tetra(
        points, [*shared, 3, *shared, 4]
    )
    assert len(two_indices) == 6 * 3, len(two_indices) // 3


def test_bounding_box_diagonal_accepts_gf_vec3f():
    """The carb.Float3 annotation is gone, so a pxr vector type works."""
    diagonal = physicsUtils.compute_bounding_box_diagonal(
        [Gf.Vec3f(-1, -1, -1), Gf.Vec3f(1, 1, 1)]
    )
    assert diagonal == pytest.approx(12.0**0.5)


@pytest.mark.parametrize(
    "dimx, dimy",
    ((0, 1), (1, 0), (0, 0), (-1, 1), (1, -1), (2, -1), (-1, 2), (-1, -1), (0, -1), (-1, 0)),
)
def test_triangle_mesh_square_is_empty_for_non_positive_dimensions(dimx, dimy):
    assert physicsUtils.create_triangle_mesh_square(dimx, dimy) == ([], [])


@pytest.mark.parametrize(
    "create_mesh",
    (
        physicsUtils.create_tetra_voxel_box,
        physicsUtils.create_tetra_voxel_sphere,
        physicsUtils.create_triangle_mesh_cube,
    ),
)
@pytest.mark.parametrize("voxel_dim", (0, -1))
def test_voxel_meshes_are_empty_for_non_positive_resolution(create_mesh, voxel_dim):
    assert create_mesh(voxel_dim) == ([], [])


@pytest.mark.parametrize("points", ([], iter(())), ids=("list", "iterator"))
def test_bounding_box_diagonal_rejects_empty_iterables(points):
    with pytest.raises(ValueError, match="at least one point"):
        physicsUtils.compute_bounding_box_diagonal(points)


def test_transform_helpers_warn_on_non_xformable(stage, caplog):
    """carb.log_warn became standard logging, so caplog can see it."""
    scope = stage.DefinePrim("/World/scope", "Scope")
    with caplog.at_level("WARNING"):
        physicsUtils.set_or_add_translate_op(UsdGeom.Xformable(scope), Gf.Vec3f(1))
    assert caplog.records, "expected a logged warning, not an exception"


def test_setup_transform_accepts_a_bare_prim(stage):
    annotation = typing.get_type_hints(
        physicsUtils.setup_transform_as_scale_orient_translate
    )["xformable"]
    assert annotation == typing.Union[Usd.Prim, UsdGeom.Xformable]

    xform = UsdGeom.Xform.Define(stage, "/World/xform")
    xform.AddTranslateOp().Set(Gf.Vec3d(1, 2, 3))

    physicsUtils.setup_transform_as_scale_orient_translate(xform.GetPrim())

    xformable = UsdGeom.Xformable(xform)
    assert [op.GetOpName() for op in xformable.GetOrderedXformOps()] == [
        "xformOp:translate",
        "xformOp:orient",
        "xformOp:scale",
    ]
    assert xformable.GetLocalTransformation().ExtractTranslation() == Gf.Vec3d(1, 2, 3)


@pytest.mark.parametrize("src_as_prim", (False, True), ids=("src_xformable", "src_prim"))
@pytest.mark.parametrize("dst_as_prim", (False, True), ids=("dst_xformable", "dst_prim"))
@pytest.mark.parametrize("reset_stack", (False, True), ids=("inherit", "reset"))
def test_copy_transform_accepts_prims_and_xformables(stage, src_as_prim, dst_as_prim, reset_stack):
    src = UsdGeom.Xformable(UsdGeom.Xform.Define(stage, "/World/source"))
    src.AddTranslateOp().Set(Gf.Vec3d(1, 2, 3))
    src.AddRotateXYZOp().Set(Gf.Vec3f(20, 30, 40))
    src.AddScaleOp().Set(Gf.Vec3f(2, 3, 4))
    src.SetResetXformStack(reset_stack)
    expected_transform = src.GetLocalTransformation()
    source_ops = [op.GetOpName() for op in src.GetOrderedXformOps()]
    dst = UsdGeom.Xformable(UsdGeom.Xform.Define(stage, "/World/destination"))
    dst.AddTranslateOp().Set(Gf.Vec3d(-4, -5, -6))
    dst.SetResetXformStack(not reset_stack)

    physicsUtils.copy_transform_as_scale_orient_translate(
        src.GetPrim() if src_as_prim else src, dst.GetPrim() if dst_as_prim else dst
    )

    assert [op.GetOpName() for op in dst.GetOrderedXformOps()] == [
        "xformOp:translate",
        "xformOp:orient",
        "xformOp:scale",
    ]
    assert Gf.IsClose(dst.GetLocalTransformation(), expected_transform, 1e-5)
    assert dst.GetResetXformStack() == reset_stack
    assert src.GetLocalTransformation() == expected_transform
    assert [op.GetOpName() for op in src.GetOrderedXformOps()] == source_ops
    assert src.GetResetXformStack() == reset_stack

    annotations = typing.get_type_hints(physicsUtils.copy_transform_as_scale_orient_translate)
    assert annotations["src"] == typing.Union[Usd.Prim, UsdGeom.Xformable]
    assert annotations["dst"] == typing.Union[Usd.Prim, UsdGeom.Xformable]
