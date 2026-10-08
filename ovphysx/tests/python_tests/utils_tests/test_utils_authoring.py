# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""End-to-end authoring against a stock USD runtime and codeless PhysX schemas.

Authors one stage per case across every submodule family, then exports the
result to `.usda` and reopens it.
"""

# @implements REQ-PYTHON-UTILS-001
# @covers AC-4 AC-5 AC-6 AC-7 AC-8 AC-10 AC-12

import pytest

pytest.importorskip("pxr", reason="ovphysx.utils requires a caller-supplied USD runtime")

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

from ovphysx import utils as physicsUtils  # noqa: E402
from ovphysx.utils import codeless  # noqa: E402


def test_scene_and_shapes(stage):
    physicsUtils.add_physics_scene(stage, "/World/PhysicsScene")
    assert stage.GetPrimAtPath("/World/PhysicsScene").IsA(UsdPhysics.Scene)

    prim = physicsUtils.add_rigid_box(
        stage,
        "/World/box",
        size=Gf.Vec3f(0.5),
        position=Gf.Vec3f(0, 0, 5),
        density=250.0,
        lin_velocity=Gf.Vec3f(1, 0, 0),
    )
    assert prim.IsA(UsdGeom.Cube)
    assert prim.HasAPI(UsdPhysics.CollisionAPI)
    assert prim.HasAPI(UsdPhysics.RigidBodyAPI)
    assert prim.HasAPI(UsdPhysics.MassAPI)
    assert UsdPhysics.MassAPI(prim).GetDensityAttr().Get() == pytest.approx(250.0)
    assert UsdPhysics.RigidBodyAPI(prim).GetVelocityAttr().Get()[0] == pytest.approx(1.0)

    sphere = physicsUtils.add_collider_sphere(stage, "/World/sphere", radius=0.25)
    assert sphere.HasAPI(UsdPhysics.CollisionAPI)
    assert not sphere.HasAPI(UsdPhysics.RigidBodyAPI)

    plain = physicsUtils.add_box(stage, "/World/box")
    assert plain.GetPath() != prim.GetPath(), "path collision was not avoided"

    physicsUtils.add_ground_plane(
        stage, "/World/ground", "Z", 100.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5)
    )
    assert stage.GetPrimAtPath("/World/ground")


@pytest.mark.parametrize(
    "plain, collider, rigid, geom_type",
    (
        (
            physicsUtils.add_capsule,
            physicsUtils.add_collider_capsule,
            physicsUtils.add_rigid_capsule,
            UsdGeom.Capsule,
        ),
        (
            physicsUtils.add_cylinder,
            physicsUtils.add_collider_cylinder,
            physicsUtils.add_rigid_cylinder,
            UsdGeom.Cylinder,
        ),
        (
            physicsUtils.add_cone,
            physicsUtils.add_collider_cone,
            physicsUtils.add_rigid_cone,
            UsdGeom.Cone,
        ),
    ),
)
def test_capsule_cylinder_and_cone_constructors(stage, plain, collider, rigid, geom_type):
    name = geom_type.__name__
    prim = plain(stage, f"/World/{name}", radius=0.25, height=1.5, axis="Z")
    assert prim.IsA(geom_type)
    typed = geom_type(prim)
    assert typed.GetRadiusAttr().Get() == pytest.approx(0.25)
    assert typed.GetHeightAttr().Get() == pytest.approx(1.5)
    assert typed.GetAxisAttr().Get() == "Z"
    assert not prim.HasAPI(UsdPhysics.CollisionAPI)

    # AC-5 names the display color and the xform op stack.
    assert list(UsdGeom.Gprim(prim).GetDisplayColorAttr().Get()) == [Gf.Vec3f(1.0)]
    ops = [op.GetOpName() for op in UsdGeom.Xformable(prim).GetOrderedXformOps()]
    assert ops == ["xformOp:translate", "xformOp:orient", "xformOp:scale"]

    static = collider(stage, f"/World/{name}Collider", radius=0.25, height=1.5)
    assert static.IsA(geom_type)
    assert static.HasAPI(UsdPhysics.CollisionAPI)
    assert not static.HasAPI(UsdPhysics.RigidBodyAPI)

    dynamic = rigid(
        stage,
        f"/World/{name}Rigid",
        radius=0.25,
        height=1.5,
        density=7.5,
        lin_velocity=Gf.Vec3f(0, 0, -2),
        ang_velocity=Gf.Vec3f(0, 1, 0),
    )
    assert dynamic.HasAPI(UsdPhysics.CollisionAPI)
    assert dynamic.HasAPI(UsdPhysics.RigidBodyAPI)
    assert dynamic.HasAPI(UsdPhysics.MassAPI)
    assert UsdPhysics.MassAPI(dynamic).GetDensityAttr().Get() == pytest.approx(7.5)
    body = UsdPhysics.RigidBodyAPI(dynamic)
    assert body.GetVelocityAttr().Get() == Gf.Vec3f(0, 0, -2)
    assert body.GetAngularVelocityAttr().Get() == Gf.Vec3f(0, 1, 0)


def test_codeless_apply_and_property_roundtrip(stage):
    prim = physicsUtils.add_rigid_box(stage, "/World/box")

    codeless.apply_api(prim, "PhysxRigidBodyAPI")
    assert "PhysxRigidBodyAPI" in prim.GetAppliedSchemas()
    codeless.set_attr(prim, "physxRigidBody:disableGravity", True)
    assert codeless.get_attr(prim, "physxRigidBody:disableGravity").Get() is True

    # An attribute the caller did not author keeps its schema fallback.
    unset = codeless.get_attr(prim, "physxRigidBody:maxDepenetrationVelocity")
    assert not unset.HasAuthoredValue()
    assert unset.Get() is not None

    codeless.apply_api(prim, "PhysxCookedDataAPI", "convexHull")
    assert prim.HasAPI("PhysxCookedDataAPI", "convexHull")

    with pytest.raises(codeless.CodelessSchemaError):
        codeless.apply_api(prim, "PhysxNoSuchAPI")
    with pytest.raises(codeless.CodelessSchemaError):
        codeless.get_attr(prim, "physxRigidBody:notAnAttribute")

    assert codeless.schema_is_registered()
    # Registration is documented as idempotent: USD ignores a plugin root it has
    # already seen, so a second call registers nothing new rather than raising.
    assert codeless.register_schemas() == []


def test_materials_groups_filters_and_joints(stage):
    material = physicsUtils.add_rigid_body_material(
        stage, "/World/material", density=500.0, static_friction=0.7
    )
    assert stage.GetPrimAtPath("/World/material").HasAPI(UsdPhysics.MaterialAPI)
    # `is True`, not `is not None`: the helper returns False when the path is
    # occupied by something that is not a Material, and `False is not None`.
    assert material is True

    box = physicsUtils.add_rigid_box(stage, "/World/box")
    physicsUtils.add_physics_material_to_prim(stage, box, "/World/material")
    # Read the binding back. The purpose matters as much as the target: a
    # physics material bound under the default purpose would be overridden by
    # any render material on the same prim.
    binding = UsdShade.MaterialBindingAPI(box).GetDirectBinding("physics")
    assert binding.GetMaterialPath() == Sdf.Path("/World/material")

    physicsUtils.add_collision_group(stage, "/World/group")
    physicsUtils.add_collision_to_collision_group(stage, box.GetPath(), "/World/group")
    assert physicsUtils.is_in_collision_group(stage, box.GetPath(), "/World/group")
    physicsUtils.remove_collision_from_collision_group(
        stage, box.GetPath(), "/World/group"
    )
    assert not physicsUtils.is_in_collision_group(stage, box.GetPath(), "/World/group")

    other = physicsUtils.add_rigid_box(stage, "/World/other")
    physicsUtils.add_pair_filter(stage, [box.GetPath(), other.GetPath()])
    assert box.HasAPI(UsdPhysics.FilteredPairsAPI)
    physicsUtils.remove_pair_filter(stage, [box.GetPath(), other.GetPath()])
    assert not box.HasAPI(UsdPhysics.FilteredPairsAPI)

    joint = physicsUtils.create_joint(stage, "Revolute", box, other)
    assert joint.GetPrim().IsA(UsdPhysics.RevoluteJoint)


def test_collider_and_rigid_body_application_and_removal(stage):
    prim = physicsUtils.create_mesh_cube(stage, "/World/mesh", 1.0).GetPrim()
    physicsUtils.set_collider(prim, UsdPhysics.Tokens.convexHull)
    assert prim.HasAPI(UsdPhysics.CollisionAPI)
    assert "PhysxConvexHullCollisionAPI" in prim.GetAppliedSchemas()
    physicsUtils.remove_collider(prim)
    assert not prim.HasAPI(UsdPhysics.CollisionAPI)
    assert "PhysxConvexHullCollisionAPI" not in prim.GetAppliedSchemas()

    physicsUtils.set_rigid_body(prim, UsdPhysics.Tokens.convexHull, False)
    assert prim.HasAPI(UsdPhysics.RigidBodyAPI)
    physicsUtils.remove_rigid_body(prim)
    assert not prim.HasAPI(UsdPhysics.RigidBodyAPI)


def test_transform_helpers(stage):
    prim = physicsUtils.add_rigid_xform(stage, "/World/x", position=Gf.Vec3f(1, 2, 3))
    assert physicsUtils.get_translation(prim) == Gf.Vec3f(1, 2, 3)

    xformable = UsdGeom.Xformable(prim)
    physicsUtils.set_or_add_translate_op(xformable, Gf.Vec3f(4, 5, 6))
    assert physicsUtils.get_translation(prim) == Gf.Vec3f(4, 5, 6)
    physicsUtils.setup_transform_as_scale_orient_translate(xformable)
    names = [op.GetOpName() for op in xformable.GetOrderedXformOps()]
    assert names == ["xformOp:translate", "xformOp:orient", "xformOp:scale"]

    assert physicsUtils.get_axis_aligned_vector("Z", 2.0) == Gf.Vec3f(0, 0, 2)

    # The factor is the reciprocal of the stage's own metersPerUnit. Authoring an
    # unusual one and reading the number back is what shows the helper consults
    # the stage; `> 0.0` held for any constant.
    UsdGeom.SetStageMetersPerUnit(stage, 0.5)
    assert UsdGeom.GetStageMetersPerUnit(stage) == pytest.approx(0.5)
    assert physicsUtils.get_unit_scale_factor(stage) == pytest.approx(2.0)

    # The two bases are spelled out because the values are what callers port
    # against, and cross-checked against each other because a basis that is not
    # right-handed, or whose forward disagrees with get_forward_vector, would put a
    # caller's camera or joint frame in a mirrored world.
    for up_axis, expected in (
        ("Y", (Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, -1), Gf.Vec3d(1, 0, 0))),
        ("Z", (Gf.Vec3d(0, 0, 1), Gf.Vec3d(-1, 0, 0), Gf.Vec3d(0, 1, 0))),
    ):
        up, forward, right = physicsUtils.get_basis(up_axis)
        assert (up, forward, right) == expected
        assert Gf.Cross(forward, up) == right
        assert tuple(physicsUtils.get_forward_vector(up_axis)) == tuple(forward)

    # A non-Xformable warns rather than raising.
    scope = stage.DefinePrim("/World/scope", "Scope")
    physicsUtils.set_or_add_translate_op(UsdGeom.Xformable(scope), Gf.Vec3f(0))


def test_transform_rewrites_keep_composed_reads_out_of_change_blocks(stage, monkeypatch):
    source = UsdGeom.Xform.Define(stage, "/World/source")
    source_xformable = UsdGeom.Xformable(source)
    source_xformable.AddTranslateOp().Set(Gf.Vec3f(2, 3, 4))
    destination = UsdGeom.Xform.Define(stage, "/World/destination")

    def forbid_change_block():
        raise AssertionError("Usd composed reads must not run in Sdf.ChangeBlock")

    monkeypatch.setattr(Sdf, "ChangeBlock", forbid_change_block)

    physicsUtils.setup_transform_as_scale_orient_translate(source_xformable)
    physicsUtils.copy_transform_as_scale_orient_translate(source_xformable, destination)

    expected_names = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
    assert [op.GetOpName() for op in source_xformable.GetOrderedXformOps()] == expected_names
    destination_xformable = UsdGeom.Xformable(destination)
    assert [op.GetOpName() for op in destination_xformable.GetOrderedXformOps()] == expected_names
    assert destination_xformable.GetLocalTransformation() == source_xformable.GetLocalTransformation()


def test_custom_metadata(stage):
    prim = physicsUtils.add_rigid_box(stage, "/World/box")
    physicsUtils.set_local_space_velocities(prim, True)
    assert prim.GetCustomDataByKey(physicsUtils.METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES)
    physicsUtils.clear_local_space_velocities(prim)
    assert not prim.HasCustomDataKey(physicsUtils.METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES)


def test_mesh_helpers(stage):
    for maker, args in (
        (physicsUtils.create_mesh_cube, (1.0,)),
        (physicsUtils.create_mesh_cylinder, (2.0, 0.5)),
        (physicsUtils.create_mesh_cone, (2.0, 0.5)),
    ):
        name = f"/World/{maker.__name__}"
        usd_mesh = maker(stage, name, *args)
        prim = usd_mesh.GetPrim()
        assert prim.IsA(UsdGeom.Mesh), name
        counts = list(usd_mesh.GetFaceVertexCountsAttr().Get())
        indices = list(usd_mesh.GetFaceVertexIndicesAttr().Get())
        points = list(usd_mesh.GetPointsAttr().Get())
        assert points and counts and indices, name
        assert sum(counts) == len(indices), name

    # Reimplemented without numpy, which the wheel deliberately does not declare.
    points, indices = physicsUtils.create_tetra_voxel_box(2)
    assert points and indices and len(indices) % 4 == 0

    volume = physicsUtils.calculate_tetra_volume(
        Gf.Vec3f(0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 0, 1)
    )
    assert abs(volume) == pytest.approx(1.0 / 6.0)

    diagonal = physicsUtils.compute_bounding_box_diagonal(
        [Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 1, 1)]
    )
    assert diagonal == pytest.approx(3.0**0.5)

    surface_points, surface_indices = physicsUtils.extract_triangle_surface_from_tetra(
        points, indices
    )
    assert surface_points and surface_indices

    soup_points, soup_indices = physicsUtils.convert_tetra_to_triangle_soup(points, indices)
    assert soup_points and soup_indices

    square_points, square_indices = physicsUtils.create_triangle_mesh_square(2, 2)
    assert len(square_points) == 9 and len(square_indices) == 24


def test_tetra_mesh_validation_and_repair(caplog):
    """`verify_tetra_mesh` only logs, so the log is the only thing to assert on.

    Called without one it cannot fail for any input, well-formed or corrupt.
    Each of its three checks is given the input that trips it, and
    `fixup_tetra_mesh_volumes` is given the inverted tetrahedron the third one
    reports (REQ AC-6).
    """
    good_points, good_indices = physicsUtils.create_tetra_voxel_box(2)
    with caplog.at_level("WARNING"):
        physicsUtils.verify_tetra_mesh(good_points, good_indices)
    assert not caplog.records, [record.getMessage() for record in caplog.records]

    # An index count that is not a whole number of tetrahedra.
    caplog.clear()
    with caplog.at_level("WARNING"):
        physicsUtils.verify_tetra_mesh(good_points, list(good_indices)[:-1])
    assert "multiple of 4" in caplog.text, caplog.text

    # An index past the end of the point list.
    caplog.clear()
    with caplog.at_level("WARNING"):
        physicsUtils.verify_tetra_mesh(good_points, [0, 1, 2, len(good_points)])
    assert "invalid index" in caplog.text, caplog.text

    # An inverted tetrahedron: the unit corner one with its first two corners
    # swapped, which flips the sign of the signed volume.
    corner = [Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 0, 1)]
    inverted = [1, 0, 2, 3]
    assert physicsUtils.calculate_tetra_volume(
        corner[1], corner[0], corner[2], corner[3]
    ) == pytest.approx(-1.0 / 6.0)

    caplog.clear()
    with caplog.at_level("WARNING"):
        physicsUtils.verify_tetra_mesh(corner, inverted)
    assert "negative volume" in caplog.text, caplog.text

    # The repair re-winds it, so the volume comes back positive and the same
    # check now passes in silence.
    fixed = physicsUtils.fixup_tetra_mesh_volumes(corner, inverted)
    assert fixed == [0, 1, 2, 3], fixed
    assert physicsUtils.calculate_tetra_volume(
        *(corner[index] for index in fixed)
    ) == pytest.approx(1.0 / 6.0)

    caplog.clear()
    with caplog.at_level("WARNING"):
        physicsUtils.verify_tetra_mesh(corner, fixed)
    assert not caplog.records, [record.getMessage() for record in caplog.records]


def test_square_and_concave_mesh_helpers(stage):
    square = physicsUtils.create_mesh_square_axis(stage, "/World/square", "Y", 2.0)
    assert square.GetPrim().IsA(UsdGeom.Mesh)
    points = list(square.GetPointsAttr().Get())
    assert len(points) == 4
    # A Y-up square is flat in Y and faces up, and its winding is reversed
    # relative to the other two axes so that the front face points that way.
    assert all(point[1] == 0.0 for point in points), points
    assert {tuple(normal) for normal in square.GetNormalsAttr().Get()} == {(0.0, 1.0, 0.0)}
    assert list(square.GetFaceVertexCountsAttr().Get()) == [4]
    assert list(square.GetFaceVertexIndicesAttr().Get()) == [3, 2, 1, 0]
    assert square.GetDoubleSidedAttr().Get() is False

    # The Z-up square is the textured one, which is what add_quad_plane uses.
    textured = physicsUtils.create_mesh_square_axis(stage, "/World/textured", "Z", 1.0)
    st = UsdGeom.PrimvarsAPI(textured.GetPrim()).GetPrimvar("st")
    assert st
    assert len(st.Get()) == 4

    concave = physicsUtils.create_mesh_concave(stage, "/World/concave", 1.0)
    concave_points = list(concave.GetPointsAttr().Get())
    counts = list(concave.GetFaceVertexCountsAttr().Get())
    indices = list(concave.GetFaceVertexIndicesAttr().Get())
    assert len(concave_points) == 10
    assert len(concave.GetNormalsAttr().Get()) == len(concave_points)
    assert sum(counts) == len(indices)
    assert max(indices) < len(concave_points)
    # The two mid-plane points are what makes the shape concave rather than a
    # box: they sit at x == 0 and are pulled in towards the far face.
    dented = [point for point in concave_points if point[0] == 0.0]
    assert len(dented) == 2
    assert all(point[2] == pytest.approx(0.2) for point in dented), dented


def test_particles(stage):
    physicsUtils.add_physx_particle_system(
        stage, "/World/particleSystem", contact_offset=0.02
    )
    prim = stage.GetPrimAtPath("/World/particleSystem")
    assert prim.GetTypeName() == "PhysxParticleSystem"
    assert codeless.get_attr(prim, "contactOffset").Get() == pytest.approx(0.02)
    # An attribute the caller omitted is left unauthored.
    assert not codeless.get_attr(prim, "restOffset").HasAuthoredValue()

    physicsUtils.add_pbd_particle_material(stage, "/World/pbd", friction=0.3, density=900.0)
    assert "PhysxPBDMaterialAPI" in stage.GetPrimAtPath("/World/pbd").GetAppliedSchemas()

    positions, velocities = physicsUtils.create_particles_grid(
        Gf.Vec3f(0), 0.1, 2, 2, 2, Gf.Vec3f(0, 0, -1)
    )
    assert len(positions) == 8 and len(velocities) == 8
    assert velocities[0] == Gf.Vec3f(0, 0, -1)

    physicsUtils.add_physx_particleset_points(
        stage,
        "/World/points",
        positions,
        velocities,
        [0.1] * len(positions),
        "/World/particleSystem",
        True,
        False,
        0,
        1.0,
        0.0,
    )
    points_prim = stage.GetPrimAtPath("/World/points")
    assert points_prim.IsA(UsdGeom.Points)
    assert "PhysxParticleSetAPI" in points_prim.GetAppliedSchemas()

    # The Sdf.Path half of the domain this helper now shares with its two
    # siblings; the str half, which it used to refuse, has its own case below.
    physicsUtils.add_physx_particleset_pointinstancer(
        stage,
        Sdf.Path("/World/instancer"),
        positions,
        velocities,
        "/World/particleSystem",
        True,
        False,
        0,
        1.0,
        0.0,
    )
    assert stage.GetPrimAtPath("/World/instancer").IsA(UsdGeom.PointInstancer)

    physicsUtils.add_physx_particle_anisotropy(stage, "/World/particleSystem")
    physicsUtils.add_physx_particle_smoothing(stage, "/World/particleSystem")
    physicsUtils.add_physx_particle_isosurface(stage, "/World/particleSystem")
    physicsUtils.add_physx_diffuse_particles(stage, "/World/points")
    applied = stage.GetPrimAtPath("/World/particleSystem").GetAppliedSchemas()
    assert "PhysxParticleAnisotropyAPI" in applied
    assert "PhysxParticleSmoothingAPI" in applied
    assert "PhysxParticleIsosurfaceAPI" in applied
    assert "PhysxDiffuseParticlesAPI" in stage.GetPrimAtPath(
        "/World/points"
    ).GetAppliedSchemas()


def test_particleset_pointinstancer_accepts_a_str_path(stage):
    """The widened half: a str path, which this helper alone used to refuse.

    It built its prototype paths from `path.pathString` before converting,
    where its two siblings convert first, so a str raised `AttributeError` out
    of the helper rather than authoring anything. The prototypes are what the
    assertion turns on, since they are the part the old spelling reached for
    (REQ AC-7).
    """
    physicsUtils.add_physx_particle_system(stage, "/World/particleSystem")
    positions, velocities = physicsUtils.create_particles_grid(
        Gf.Vec3f(0), 0.1, 2, 1, 1, Gf.Vec3f(0)
    )

    prim = physicsUtils.add_physx_particleset_pointinstancer(
        stage,
        "/World/instancer",
        positions,
        velocities,
        "/World/particleSystem",
        True,
        False,
        0,
        1.0,
        0.0,
        num_prototypes=2,
    )

    assert prim.IsA(UsdGeom.PointInstancer)
    assert list(UsdGeom.PointInstancer(prim).GetPrototypesRel().GetTargets()) == [
        Sdf.Path("/World/instancer/particlePrototype0"),
        Sdf.Path("/World/instancer/particlePrototype1"),
    ]


def test_deformables(stage):
    physicsUtils.add_deformable_material(
        stage, "/World/deformableMaterial", youngs_modulus=5.0e5
    )
    assert "OmniPhysicsDeformableMaterialAPI" in stage.GetPrimAtPath(
        "/World/deformableMaterial"
    ).GetAppliedSchemas()
    physicsUtils.add_surface_deformable_material(
        stage, "/World/surfaceMaterial", surface_thickness=0.01
    )
    assert "OmniPhysicsSurfaceDeformableMaterialAPI" in stage.GetPrimAtPath(
        "/World/surfaceMaterial"
    ).GetAppliedSchemas()

    stage.DefinePrim("/World/deformable", "Xform")
    physicsUtils.create_mesh_cube(stage, "/World/src", 1.0)
    created = physicsUtils.create_auto_volume_deformable_hierarchy(
        stage,
        "/World/deformable",
        "/World/deformable/sim",
        "/World/deformable/collision",
        "/World/src",
        simulation_hex_mesh_enabled=True,
        cooking_src_simplification_enabled=True,
    )
    assert created is True
    applied = stage.GetPrimAtPath("/World/deformable").GetAppliedSchemas()
    assert "PhysxAutoDeformableHexahedralMeshAPI" in applied
    assert "PhysxAutoDeformableMeshSimplificationAPI" in applied

    physicsUtils.remove_deformable_body(stage, Sdf.Path("/World/deformable"))
    remaining = [
        name
        for name in stage.GetPrimAtPath("/World/deformable").GetAppliedSchemas()
        if "Deformable" in name
    ]
    assert not remaining, remaining


def test_surface_deformable_body_and_hierarchy(stage):
    points, indices = physicsUtils.create_triangle_mesh_square(2, 2)
    cloth = physicsUtils.create_mesh(
        stage,
        "/World/cloth",
        points,
        [Gf.Vec3f(0, 0, 1)] * len(points),
        indices,
        [3] * (len(indices) // 3),
    )
    assert physicsUtils.set_physics_surface_deformable_body(stage, Sdf.Path("/World/cloth"))
    cloth_prim = cloth.GetPrim()
    applied = cloth_prim.GetAppliedSchemas()
    assert "OmniPhysicsDeformableBodyAPI" in applied
    assert "OmniPhysicsSurfaceDeformableSimAPI" in applied
    assert cloth_prim.HasAPI(UsdPhysics.CollisionAPI)
    rest_points = codeless.get_attr(cloth_prim, "omniphysics:restShapePoints").Get()
    rest_indices = codeless.get_attr(cloth_prim, "omniphysics:restTriVtxIndices").Get()
    assert rest_points is not None and rest_indices is not None
    assert list(rest_points) == list(points)
    assert len(rest_indices) == len(indices) // 3

    # A quad mesh is refused, and refused before anything is authored: the
    # surface sim API is defined over triangles only.
    physicsUtils.create_mesh_cube(stage, "/World/quads", 1.0)
    assert not physicsUtils.set_physics_surface_deformable_body(stage, Sdf.Path("/World/quads"))
    assert not stage.GetPrimAtPath("/World/quads").GetAppliedSchemas()

    root = UsdGeom.Xform.Define(stage, "/World/cloak").GetPrim()
    skin = physicsUtils.create_mesh_cube(stage, "/World/cloak/skin", 1.0)
    physicsUtils.create_mesh_cube(stage, "/World/src", 1.0)
    assert physicsUtils.create_auto_surface_deformable_hierarchy(
        stage,
        "/World/cloak",
        "/World/cloak/sim",
        "/World/src",
        cooking_src_simplification_enabled=True,
        set_visibility_with_guide_purpose=True,
    )
    applied = root.GetAppliedSchemas()
    assert "PhysxAutoDeformableBodyAPI" in applied
    assert "PhysxAutoDeformableMeshSimplificationAPI" in applied
    assert "OmniPhysicsDeformableBodyAPI" in applied
    cooking_source = root.GetRelationship("physxDeformableBody:cookingSourceMesh")
    assert list(cooking_source.GetTargets()) == [Sdf.Path("/World/src")]

    sim = stage.GetPrimAtPath("/World/cloak/sim")
    assert sim.IsA(UsdGeom.Mesh)
    assert "OmniPhysicsSurfaceDeformableSimAPI" in sim.GetAppliedSchemas()
    assert sim.HasAPI(UsdPhysics.CollisionAPI)
    # The generated simulation mesh is hidden from rendering only because the
    # skin mesh is there to be rendered instead.
    assert UsdGeom.Imageable(sim).GetPurposeAttr().Get() == UsdGeom.Tokens.guide

    # The skin mesh keeps a bind-pose snapshot of its own points.
    assert "OmniPhysicsDeformablePoseAPI:default" in skin.GetPrim().GetAppliedSchemas()
    bind_pose = skin.GetPrim().GetAttribute("deformablePose:default:omniphysics:points")
    assert bind_pose.Get() is not None
    assert list(bind_pose.Get()) == list(skin.GetPointsAttr().Get())


def test_deformable_remove_side_helpers(stage):
    physicsUtils.create_mesh_cube(stage, "/World/src", 1.0)
    root = stage.DefinePrim("/World/deformable", "Xform")
    skin = physicsUtils.create_mesh_cube(stage, "/World/deformable/skin", 1.0)
    assert physicsUtils.create_auto_volume_deformable_hierarchy(
        stage,
        "/World/deformable",
        "/World/deformable/sim",
        "/World/deformable/collision",
        "/World/src",
        simulation_hex_mesh_enabled=True,
        cooking_src_simplification_enabled=True,
    )

    # The removal sweeps the whole physxDeformableBody namespace, not only the
    # properties the API declares, so a custom one in it goes too.
    root.CreateAttribute("physxDeformableBody:customTag", Sdf.ValueTypeNames.Float).Set(1.0)

    # Each sub-component helper drops its own API and leaves the auto body alone.
    physicsUtils.remove_auto_deformable_hexahedral_mesh(stage, Sdf.Path("/World/deformable"))
    assert "PhysxAutoDeformableHexahedralMeshAPI" not in root.GetAppliedSchemas()
    assert "PhysxAutoDeformableBodyAPI" in root.GetAppliedSchemas()

    physicsUtils.remove_auto_deformable_mesh_simplification(stage, Sdf.Path("/World/deformable"))
    assert "PhysxAutoDeformableMeshSimplificationAPI" not in root.GetAppliedSchemas()
    assert "PhysxAutoDeformableBodyAPI" in root.GetAppliedSchemas()

    assert physicsUtils.add_auto_deformable_mesh_simplification(
        stage, Sdf.Path("/World/deformable")
    )
    assert "PhysxAutoDeformableMeshSimplificationAPI" in root.GetAppliedSchemas()

    physicsUtils.remove_auto_deformable_body(stage, Sdf.Path("/World/deformable"))
    applied = root.GetAppliedSchemas()
    assert "PhysxAutoDeformableBodyAPI" not in applied
    assert "PhysxAutoDeformableMeshSimplificationAPI" not in applied
    # The auto body's properties go with it, the cooking source relationship
    # included, while the deformable body it fed stays: that is a separate API
    # and remove_deformable_body is what takes it.
    assert not root.GetAuthoredPropertiesInNamespace(["physxDeformableBody"])
    assert "OmniPhysicsDeformableBodyAPI" in applied

    physicsUtils.remove_deformable_body(stage, Sdf.Path("/World/deformable"))
    for prim in Usd.PrimRange(root):
        remaining = [name for name in prim.GetAppliedSchemas() if "Deformable" in name]
        assert not remaining, (prim.GetPath(), remaining)
    assert not skin.GetPrim().GetAuthoredPropertiesInNamespace(["deformablePose"])

    # This layout gives UsdPhysics.CollisionAPI to the collision mesh alone, and
    # nothing on the stage tells that prim from a caller's own child collider, so
    # a removal asked for the root keeps it. REQ AC-8 records the rule.
    assert stage.GetPrimAtPath("/World/deformable/collision").HasAPI(UsdPhysics.CollisionAPI)


def test_usda_roundtrip(stage):
    physicsUtils.add_physics_scene(stage, "/World/PhysicsScene")
    prim = physicsUtils.add_rigid_box(stage, "/World/box", density=42.0)
    path = prim.GetPath()
    codeless.apply_api(prim, "PhysxRigidBodyAPI")
    codeless.set_attr(prim, "physxRigidBody:disableGravity", True)

    text = stage.GetRootLayer().ExportToString()
    assert "PhysxRigidBodyAPI" in text

    layer = Sdf.Layer.CreateAnonymous(".usda")
    layer.ImportFromString(text)
    # Keep the reopened stage alive: the prim handle expires with it.
    reopened = Usd.Stage.Open(layer)
    reloaded = reopened.GetPrimAtPath(path)
    assert "PhysxRigidBodyAPI" in reloaded.GetAppliedSchemas()
    assert codeless.get_attr(reloaded, "physxRigidBody:disableGravity").Get() is True
    assert UsdPhysics.MassAPI(reloaded).GetDensityAttr().Get() == pytest.approx(42.0)
