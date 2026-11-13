# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Behavior of the generated Python population builder (ovphysx.population).

What the builder writes is verified against PhysX by the runtime's generated doctest suite; this
file covers the Python-only contract: lazy dependency import, DLPack dtypes of the published
columns (a contract bool must reach ovstage as kDLBool, not as a uint8 column), the contract rules
that need no PhysX, path-list ownership, and that a legitimately authored value equal to a
divergent raw schema fallback (zero friction) is accepted. The builders added with the compiled
ovstage cache, which no generated case covers, are read back from ovstage column by column.
"""

# @implements REQ-CAPI-POPULATION-001
# @covers AC-1 AC-2

import gc
import subprocess
import sys
import warnings

import pytest

import ovphysx.population as pop

K_DL_BOOL = 6
K_DL_FLOAT = 2


def test_import_does_not_pull_warp_or_ovstage():
    # A fresh interpreter: importing the module must not import its heavy dependencies.
    code = (
        "import sys, ovphysx.population as p; "
        "assert 'warp' not in sys.modules and 'ovstage' not in sys.modules, sorted(m for m in sys.modules if m in ('warp', 'ovstage')); "
        "assert p.resolved_defaults('Cube')['size'] == 2.0"
    )
    subprocess.run([sys.executable, "-c", code], check=True)


def test_resolved_defaults_follow_stage_units():
    scene = pop.resolved_defaults("PhysicsScene", meters_per_unit=0.01)
    assert scene["physics:gravityMagnitude"] == pytest.approx(981.0)
    assert tuple(scene["physics:gravityDirection"]) == (0.0, 0.0, -1.0)
    assert tuple(pop.resolved_defaults("PhysicsScene", up_axis="Y")["physics:gravityDirection"]) == (0.0, -1.0, 0.0)


def test_ancestor_paths():
    assert pop.ancestor_paths(["/World/Env/Box_0", "/World/Env/Box_1", "/Other"]) == ["/World", "/World/Env"]


def test_bool_columns_publish_as_dlpack_bool():
    wp = pytest.importorskip("warp")
    ovstage = pytest.importorskip("ovstage")
    tensor, arr = pop._tensor(wp, ovstage, "physics:collisionEnabled", [True, False], "bool", 2, 1)
    assert arr.dtype is wp.bool
    assert (tensor.dtype.code, tensor.dtype.bits, tensor.dtype.lanes) == (K_DL_BOOL, 8, 1)
    tensor, arr = pop._tensor(wp, ovstage, "physics:mass", [1.5], "float32", 1, 1)
    assert (tensor.dtype.code, tensor.dtype.bits) == (K_DL_FLOAT, 32)


def test_prebuilt_arrays_must_match_dtype():
    wp = pytest.importorskip("warp")
    ovstage = pytest.importorskip("ovstage")
    wrong = wp.array([1], dtype=wp.int32, device="cpu")
    with pytest.raises(TypeError, match="expected a float32 array"):
        pop._tensor(wp, ovstage, "physics:mass", wrong, "float32", 1, 1)


@pytest.fixture
def stage_and_paths():
    ovstage = pytest.importorskip("ovstage")
    pytest.importorskip("warp")
    with ovstage.Stage("ovphysx-population-builder") as stage:
        yield stage, ovstage.PathDictionary(stage)


def test_prebuilt_arrays_must_be_contiguous(stage_and_paths):
    # ovstage reads compact rows, so a strided view is rejected while the column is built, not by the
    # write op; a tuple row would otherwise fail inside the lane fold.
    wp = pytest.importorskip("warp")
    ovstage = pytest.importorskip("ovstage")
    _, paths = stage_and_paths
    every_other = wp.array([1.0, 2.0, 3.0, 4.0], dtype=wp.float32, device="cpu")[::2]
    with pytest.raises(TypeError, match="strided array"):
        pop._tensor(wp, ovstage, "physics:mass", every_other, "float32", 2, 1)
    every_other_quat = wp.zeros((4, 4), dtype=wp.float16, device="cpu")[::2]
    with pytest.raises(TypeError, match="strided array"):
        pop.PrimBatch(paths, ["/World/Instancer"]).define_point_instancer(
            proto_indices=[[0, 0]], orientations=[every_other_quat])


def test_rules_that_need_no_physx(stage_and_paths):
    stage, paths = stage_and_paths
    batch = pop.PrimBatch(paths, ["/World/Scene"])
    batch.define_physics_scene()
    with pytest.raises(ValueError, match="does not apply"):
        batch.apply_physics_rigid_body_api()

    batch = pop.PrimBatch(paths, ["/World/Box"])
    batch.define_cube()
    batch.apply_physx_collision_api()  # requires PhysicsCollisionAPI on the same prim
    with pytest.raises(ValueError, match="requires"):
        batch.write(stage, 1)

    # A Newton collider without the PhysicsCollisionAPI its schema builds in would not collide.
    batch = pop.PrimBatch(paths, ["/World/NewtonBox"])
    batch.define_cube()
    batch.apply_newton_collision_api()
    with pytest.raises(ValueError, match="requires PhysicsCollisionAPI"):
        batch.write(stage, 1)

    batch = pop.PrimBatch(paths, ["/World/Joint"])
    batch.define_physics_joint()
    batch.apply_physics_drive_api("rotX")
    batch.apply_physx_drive_performance_envelope_api("rotY")  # extends the same-instance drive only
    with pytest.raises(ValueError, match="PhysicsDriveAPI:rotY"):
        batch.write(stage, 1)


def test_transform_fields_are_rejected_on_non_xformable_types(stage_and_paths):
    stage, paths = stage_and_paths
    # The public define_<type> of a non-Xformable type has no transform keywords at all ...
    with pytest.raises(TypeError):
        pop.PrimBatch(paths, ["/World/Scene"]).define_physics_scene(reset_xform_stack=[True])
    # ... and the shared _define rejects every supplied transform field, the reset flag included.
    for local, reset, world in (([[1.0] * 16], None, None), (None, [True], None), (None, None, [[1.0] * 16])):
        batch = pop.PrimBatch(paths, ["/World/Scene"])
        with pytest.raises(ValueError, match="not Xformable"):
            batch._define("PhysicsScene", local, reset, world)


def test_batch_is_reusable_and_resends_the_full_prim(stage_and_paths):
    # Like the C++ builder, the payload columns survive write(); a retry or a second write must send
    # the whole prim again rather than only the structural columns appended last.
    stage, paths = stage_and_paths
    batch = pop.PrimBatch(paths, ["/World/Box"])
    batch.define_cube(size=[2.0])
    batch.apply_physics_rigid_body_api()
    batch.apply_physics_collision_api()
    payload = list(batch._writes)
    batch.write(stage, 1)
    assert batch._writes == payload
    batch.write(stage, 2)
    assert batch._writes == payload


def test_zero_friction_is_an_accepted_authored_value(stage_and_paths):
    # 0.0 is the raw schema fallback and parses differently from an absent column (0.5), but a value
    # does not encode authoredness: an explicitly authored zero-friction material must be written.
    stage, paths = stage_and_paths
    batch = pop.PrimBatch(paths, ["/World/Ice"])
    batch.define_material()
    batch.apply_physics_material_api(static_friction=[0.0], dynamic_friction=[0.0])
    batch.write(stage, 1)
    assert "physics:staticFriction" in pop.RAW_FALLBACK_DIVERGES


def test_write_releases_its_path_list(stage_and_paths):
    stage, paths = stage_and_paths
    pop.write_stage_units(stage, paths, 1, meters_per_unit=0.01)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        pop.create_rigid_body_cubes(stage, paths, ["/World/Box_0", "/World/Box_1"], 1, cube={"size": [2.0, 2.0]})
        gc.collect()
    leaked = [w for w in caught if issubclass(w.category, ResourceWarning)]
    assert not leaked, [str(w.message) for w in leaked]


def _read_back(stage, paths, prim, column, flatten=True):
    """The values the stage holds for one column of one prim, flattened in element order unless asked not to."""
    ovstage = pytest.importorskip("ovstage")
    values = []
    with paths.create_path_list_from_strings([prim]) as path_list:
        with stage.query_from_path_list(path_list) as query:
            with stage.read_attributes(query, [paths.intern_token(column)], ovstage.OrdinalRange.latest(1)) as read:
                read.wait()
                for group in read.groups():
                    with group:
                        for column in group.columns:
                            for index in range(column.tensor_count):
                                array = column.array(index)
                                values.append((array.reshape(-1) if flatten else array).copy())
    assert len(values) == 1, (prim, column, values)
    return values[0]


# The builders added with the compiled ovstage cache, each with every column it writes. Expected
# values are flattened in element order; "half" columns compare as numpy float16, "token" and
# "path" columns as the strings their ids intern. Values are exact in their column's precision.
_NEW_BUILDERS = [
    ("/World/Scope", lambda b: b.define_scope(), "Scope", None, {}),
    ("/World/Mesh/Subset", lambda b: b.define_geom_subset(indices=[[0, 2]], element_type=["face"]),
     "GeomSubset", None, {"indices": ("value", [0, 2]), "elementType": ("token", ["face"])}),
    ("/World/Instancer", lambda b: b.define_point_instancer(
        proto_indices=[[0, 1]], positions=[[[1, 2, 3], [4, 5, 6]]],
        orientations=[[[0.5, 0.5, 0.5, 0.5], [1, 0, 0, 0]]], velocities=[[[0, 0, 1], [0, 1, 0]]],
        angular_velocities=[[[1, 0, 0], [0, 0, 2]]], scales=[[[1, 1, 1], [2, 2, 2]]], ids=[[7, 9]],
        inactive_ids=[[9]], prototypes=[["/World/Proto"]], prototypes_usd_targets=[["/World/Proto"]]),
     "PointInstancer", None, {
         "protoIndices": ("value", [0, 1]), "positions": ("value", [1, 2, 3, 4, 5, 6]),
         "orientations": ("half", [0.5, 0.5, 0.5, 0.5, 1, 0, 0, 0]),
         "velocities": ("value", [0, 0, 1, 0, 1, 0]), "angularVelocities": ("value", [1, 0, 0, 0, 0, 2]),
         "scales": ("value", [1, 1, 1, 2, 2, 2]), "ids": ("value", [7, 9]),
         "usd-metadata:inactiveIds": ("value", [9]), "prototypes": ("path", ["/World/Proto"]),
         "prototypes:usd-targets": ("path", ["/World/Proto"])}),
    ("/World/JointInstancer", lambda b: b.define_physx_physics_joint_instancer(
        proto_indices=[[0]], body0_indices=[[0]], body1_indices=[[1]], local_pos0s=[[[1, 0, 0]]],
        local_rot0s=[[[0, 0, 0, 1]]], local_pos1s=[[[0, 1, 0]]], local_rot1s=[[[0.5, -0.5, 0.5, -0.5]]],
        prototypes=[["/World/JointProto"]], body0s=[["/World/A"]], body1s=[["/World/B"]]),
     "PhysxPhysicsJointInstancer", None, {
         "physics:protoIndices": ("value", [0]), "physics:body0Indices": ("value", [0]),
         "physics:body1Indices": ("value", [1]), "physics:localPos0s": ("value", [1, 0, 0]),
         "physics:localRot0s": ("half", [0, 0, 0, 1]), "physics:localPos1s": ("value", [0, 1, 0]),
         "physics:localRot1s": ("half", [0.5, -0.5, 0.5, -0.5]),
         "physics:prototypes": ("path", ["/World/JointProto"]), "physics:body0s": ("path", ["/World/A"]),
         "physics:body1s": ("path", ["/World/B"])}),
    ("/World/Group", lambda b: b.define_xform().apply_collection_api(
        "colliders", includes=[["/World/A"]], excludes=[["/World/B"]]),
     "Xform", "CollectionAPI:colliders", {
         "collection:colliders:includes": ("path", ["/World/A"]),
         "collection:colliders:excludes": ("path", ["/World/B"])}),
    ("/World/Robot", lambda b: b.define_xform().apply_physics_articulation_root_api()
     .apply_newton_articulation_root_api(self_collision_enabled=[True]),
     "Xform", "NewtonArticulationRootAPI", {"newton:selfCollisionEnabled": ("value", [True])}),
    ("/World/Box", lambda b: b.define_cube().apply_physics_collision_api().apply_newton_collision_api(
        contact_margin=[0.25], contact_gap=[0.5]),
     "Cube", "NewtonCollisionAPI", {"newton:contactMargin": ("value", [0.25]), "newton:contactGap": ("value", [0.5])}),
    ("/World/Hinge", lambda b: b.define_physics_revolute_joint().apply_newton_joint_api(
        velocity_limit=[3], armature=[0.125], friction=[0.25], damping=[0.5]),
     "PhysicsRevoluteJoint", "NewtonJointAPI", {
         "newton:velocityLimit": ("value", [3]), "newton:armature": ("value", [0.125]),
         "newton:friction": ("value", [0.25]), "newton:damping": ("value", [0.5])}),
    ("/World/Hull", lambda b: b.define_mesh().apply_physics_collision_api().apply_physics_mesh_collision_api()
     .apply_newton_collision_api().apply_newton_mesh_collision_api(max_hull_vertices=[32]),
     "Mesh", "NewtonMeshCollisionAPI", {"newton:maxHullVertices": ("value", [32])}),
    ("/World/Follower", lambda b: b.define_physics_revolute_joint().apply_newton_mimic_api(
        mimic_enabled=[True], mimic_coef0=[0.5], mimic_coef1=[2], mimic_joint=[["/World/Leader"]]),
     "PhysicsRevoluteJoint", "NewtonMimicAPI", {
         "newton:mimicEnabled": ("value", [True]), "newton:mimicCoef0": ("value", [0.5]),
         "newton:mimicCoef1": ("value", [2]), "newton:mimicJoint": ("path", ["/World/Leader"])}),
    ("/World/Scene", lambda b: b.define_physics_scene().apply_newton_scene_api(
        time_steps_per_second=[120], gravity_enabled=[False]),
     "PhysicsScene", "NewtonSceneAPI", {
         "newton:timeStepsPerSecond": ("value", [120]), "newton:gravityEnabled": ("value", [False])}),
    ("/World/Pushed", lambda b: b.define_xform().apply_physx_force_api(
        force_enabled=[True], world_frame_enabled=[False], force=[[1, 2, 3]], torque=[[4, 5, 6]],
        mode=["acceleration"]),
     "Xform", "PhysxForceAPI", {
         "physxForce:forceEnabled": ("value", [True]), "physxForce:worldFrameEnabled": ("value", [False]),
         "physxForce:force": ("value", [1, 2, 3]), "physxForce:torque": ("value", [4, 5, 6]),
         "physxForce:mode": ("token", ["acceleration"])}),
    ("/World/Mimic", lambda b: b.define_physics_revolute_joint().apply_physx_mimic_joint_api(
        "rotX", gearing=[2], offset=[0.25], natural_frequency=[5], damping_ratio=[0.75],
        reference_joint_axis=["rotY"], reference_joint=[["/World/Leader"]]),
     "PhysicsRevoluteJoint", "PhysxMimicJointAPI:rotX", {
         "physxMimicJoint:rotX:gearing": ("value", [2]), "physxMimicJoint:rotX:offset": ("value", [0.25]),
         "physxMimicJoint:rotX:naturalFrequency": ("value", [5]),
         "physxMimicJoint:rotX:dampingRatio": ("value", [0.75]),
         "physxMimicJoint:rotX:referenceJointAxis": ("token", ["rotY"]),
         "physxMimicJoint:rotX:referenceJoint": ("path", ["/World/Leader"])}),
    ("/World/Fluid", lambda b: b.define_material().apply_physx_pbdmaterial_api(
        friction=[0.25], particle_friction_scale=[2], damping=[0.5], viscosity=[0.125],
        vorticity_confinement=[1.5], surface_tension=[0.75], cohesion=[3], adhesion=[4],
        particle_adhesion_scale=[5], adhesion_offset_scale=[6], gravity_scale=[0.5], cfl_coefficient=[1],
        density=[1000]),
     "Material", "PhysxPBDMaterialAPI", {
         "physxPBDMaterial:friction": ("value", [0.25]), "physxPBDMaterial:particleFrictionScale": ("value", [2]),
         "physxPBDMaterial:damping": ("value", [0.5]), "physxPBDMaterial:viscosity": ("value", [0.125]),
         "physxPBDMaterial:vorticityConfinement": ("value", [1.5]),
         "physxPBDMaterial:surfaceTension": ("value", [0.75]), "physxPBDMaterial:cohesion": ("value", [3]),
         "physxPBDMaterial:adhesion": ("value", [4]), "physxPBDMaterial:particleAdhesionScale": ("value", [5]),
         "physxPBDMaterial:adhesionOffsetScale": ("value", [6]), "physxPBDMaterial:gravityScale": ("value", [0.5]),
         "physxPBDMaterial:cflCoefficient": ("value", [1]), "physxPBDMaterial:density": ("value", [1000])}),
    ("/World/Attachment", lambda b: b.define_xform().apply_physx_tendon_attachment_api(
        "t", gearing=[1.5], local_pos=[[1, 2, 3]], parent_attachment=["root"], parent_link=[["/World/Link"]]),
     "Xform", "PhysxTendonAttachmentAPI:t", {
         "physxTendon:t:gearing": ("value", [1.5]), "physxTendon:t:localPos": ("value", [1, 2, 3]),
         "physxTendon:t:parentAttachment": ("token", ["root"]), "physxTendon:t:parentLink": ("path", ["/World/Link"])}),
    ("/World/Leaf", lambda b: b.define_xform().apply_physx_tendon_attachment_leaf_api(
        "t", rest_length=[0.5], lower_limit=[-1], upper_limit=[1]),
     "Xform", "PhysxTendonAttachmentLeafAPI:t", {
         "physxTendon:t:restLength": ("value", [0.5]), "physxTendon:t:lowerLimit": ("value", [-1]),
         "physxTendon:t:upperLimit": ("value", [1])}),
    ("/World/Root", lambda b: b.define_xform().apply_physx_tendon_attachment_root_api(
        "t", tendon_enabled=[True], stiffness=[10], limit_stiffness=[20], damping=[1], offset=[0.25]),
     "Xform", "PhysxTendonAttachmentRootAPI:t", {
         "physxTendon:t:tendonEnabled": ("value", [True]), "physxTendon:t:stiffness": ("value", [10]),
         "physxTendon:t:limitStiffness": ("value", [20]), "physxTendon:t:damping": ("value", [1]),
         "physxTendon:t:offset": ("value", [0.25])}),
    ("/World/Axis", lambda b: b.define_physics_revolute_joint().apply_physx_tendon_axis_api(
        "t", gearing=[[1, 2]], force_coefficient=[[0.5, 0.25]]),
     "PhysicsRevoluteJoint", "PhysxTendonAxisAPI:t", {
         "physxTendon:t:gearing": ("value", [1, 2]), "physxTendon:t:forceCoefficient": ("value", [0.5, 0.25])}),
    ("/World/AxisRoot", lambda b: b.define_physics_revolute_joint().apply_physx_tendon_axis_root_api(
        "t", stiffness=[10], limit_stiffness=[20], damping=[1], offset=[0.25], rest_length=[0.5],
        lower_limit=[-1], upper_limit=[1], tendon_enabled=[True]),
     "PhysicsRevoluteJoint", "PhysxTendonAxisRootAPI:t", {
         "physxTendon:t:stiffness": ("value", [10]), "physxTendon:t:limitStiffness": ("value", [20]),
         "physxTendon:t:damping": ("value", [1]), "physxTendon:t:offset": ("value", [0.25]),
         "physxTendon:t:restLength": ("value", [0.5]), "physxTendon:t:lowerLimit": ("value", [-1]),
         "physxTendon:t:upperLimit": ("value", [1]), "physxTendon:t:tendonEnabled": ("value", [True])}),
]


@pytest.mark.parametrize("prim, build, prim_type, api, columns", _NEW_BUILDERS, ids=[e[0] for e in _NEW_BUILDERS])
def test_new_builders_write_their_columns(stage_and_paths, prim, build, prim_type, api, columns):
    # Each builder writes its type or API and every column given to it, with its value, type and order.
    np = pytest.importorskip("numpy")
    ovstage = pytest.importorskip("ovstage")
    stage, paths = stage_and_paths
    build(pop.PrimBatch(paths, [prim])).write(stage, 1)
    stage.advance_write_floor(ordinal=1).wait()
    # Token and path columns hold raw uint64 ids; resolving one takes the matching OVStage handle.
    prim_types = _read_back(stage, paths, prim, "usd-prim-type")
    assert [paths.token_to_string(ovstage.Token(int(t))) for t in prim_types] == [prim_type]
    if api is not None:
        schemas = [paths.token_to_string(ovstage.Token(int(t))) for t in _read_back(stage, paths, prim, "usd-schemas")]
        assert api in schemas, schemas
    for column, (kind, expected) in columns.items():
        values = _read_back(stage, paths, prim, column)
        if kind == "token":
            assert [paths.token_to_string(ovstage.Token(int(v))) for v in values] == expected, column
        elif kind == "path":
            assert [paths.path_to_string(ovstage.PrimPath(int(v))) for v in values] == expected, column
        elif kind == "half":
            assert values.dtype.itemsize == 2, column
            assert values.view(np.float16).tolist() == expected, column
        else:
            assert values.tolist() == expected, column


def test_array_columns_keep_their_tuple_width(stage_and_paths):
    # NVBug 7075162: an array element's tuple width travels in dtype.lanes. Published as an (n, lanes)
    # scalar tensor, two quaternions were stored as eight one-lane halves and read back as [8], not [2, 4].
    stage, paths = stage_and_paths
    pop.PrimBatch(paths, ["/World/Instancer"]).define_point_instancer(
        proto_indices=[[0, 0]], positions=[[[1, 2, 3], [4, 5, 6]]],
        orientations=[[[0, 0, 0, 1], [0.5, 0.5, 0.5, 0.5]]]).write(stage, 1)
    pop.PrimBatch(paths, ["/World/JointInstancer"]).define_physx_physics_joint_instancer(
        proto_indices=[[0, 0]], local_pos0s=[[[1, 0, 0], [0, 1, 0]]],
        local_rot0s=[[[0, 0, 0, 1], [0.5, -0.5, 0.5, -0.5]]]).write(stage, 1)
    stage.advance_write_floor(ordinal=1).wait()
    for prim, column, shape in (
        ("/World/Instancer", "protoIndices", (2,)),
        ("/World/Instancer", "positions", (2, 3)),
        ("/World/Instancer", "orientations", (2, 4)),
        ("/World/JointInstancer", "physics:localPos0s", (2, 3)),
        ("/World/JointInstancer", "physics:localRot0s", (2, 4)),
    ):
        assert _read_back(stage, paths, prim, column, flatten=False).shape == shape, column
