# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Ported from omni.physx.tests PhysxAutoVolumeDeformableBodyAPI.py for standalone ovruntime testing.
# Only the MemoryStage class is ported; the TestStage class requires Kit and is not portable.

import _physics_setup  # noqa: F401 - loads Carbonite + physics plugins + PhysX schemas

import unittest
import _physx
from physicsBase import PhysicsMemoryStageBaseTestCase, TestCategory
from omni.physx.scripts import deformableMeshUtils, deformableUtils
from pxr import Sdf, Usd, Gf, UsdGeom, UsdPhysics, PhysxSchema


def create_transform(translate=Gf.Vec3d(0.0),
                     rotate=Gf.Rotation(Gf.Quatd(1.0)),
                     scale=Gf.Vec3d(1.0),
                     pivot_pos=Gf.Vec3d(0.0),
                     pivot_orient=Gf.Rotation(Gf.Quatd(1.0))):
    return Gf.Transform(translate, rotate, scale, pivot_pos, pivot_orient)


def set_tetmesh_data(tetmesh):
    points, indices = deformableMeshUtils.createTetraVoxelBox(3)
    tet_indices = [Gf.Vec4i(*indices[i:i+4]) for i in range(0, len(indices), 4)]
    tetmesh.GetPointsAttr().Set(points)
    tetmesh.GetTetVertexIndicesAttr().Set(tet_indices)


def setup_xform(stage, path, transform: Gf.Transform):
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTransformOp().Set(transform.GetMatrix())
    return xform


def setup_cube_trimesh(stage, path, transform: Gf.Transform, dim: int):
    tri_points, tri_indices = deformableMeshUtils.createTriangleMeshCube(dim)
    skinmesh = UsdGeom.Mesh.Define(stage, path)
    skinmesh.AddTransformOp().Set(transform.GetMatrix())
    skinmesh.GetPointsAttr().Set(tri_points)
    skinmesh.GetFaceVertexCountsAttr().Set([3]*(len(tri_indices)//3))
    skinmesh.GetFaceVertexIndicesAttr().Set(tri_indices)
    return skinmesh


def get_schema_instances(prim: Usd.Prim, schema_type_name: str):
    return {s[len(schema_type_name) + 1:] for s in prim.GetAppliedSchemas() if s.startswith(schema_type_name)}


def has_pose(prim: Usd.Prim):
    return len(get_schema_instances(prim, "OmniPhysicsDeformablePoseAPI")) > 0


def get_pose_name_with_purpose(prim: Usd.Prim, purpose_name: str):
    pose_names = get_schema_instances(prim, "OmniPhysicsDeformablePoseAPI")
    for pose_name in pose_names:
        if purpose_name in prim.GetAttribute("deformablePose:" + pose_name + ":omniphysics:purposes").Get():
            return pose_name
    return None


def has_pose_with_purpose(prim: Usd.Prim, purpose_name: str):
    return get_pose_name_with_purpose(prim, purpose_name) is not None


def check_body_apis(test, stage, body_path, expect_body=True, expect_auto=True, cooking_src_path=None):
    prim = stage.GetPrimAtPath(body_path)
    test.assertTrue(prim.HasAPI("OmniPhysicsDeformableBodyAPI") == expect_body)
    test.assertTrue(prim.HasAPI("PhysxAutoDeformableBodyAPI") == expect_auto)
    test.assertTrue(prim.HasRelationship("physxDeformableBody:cookingSourceMesh") == (cooking_src_path is not None))
    if cooking_src_path is not None:
        cookingSrcRel = prim.GetRelationship("physxDeformableBody:cookingSourceMesh")
        test.assertTrue(len(cookingSrcRel.GetTargets()) == 1)
        test.assertTrue(cookingSrcRel.GetTargets()[0] == Sdf.Path(cooking_src_path))


def check_pose(test, prim, purpose_name, expect_purpose=True):
    test.assertTrue(has_pose(prim) == expect_purpose)
    test.assertTrue(has_pose_with_purpose(prim, purpose_name) == expect_purpose)


def check_sim_apis(test, stage, sim_path, expect_sim=True, expect_bindpose=True):
    prim = stage.GetPrimAtPath(sim_path)
    test.assertTrue(prim.IsA(UsdGeom.TetMesh))
    test.assertTrue(prim.HasAPI("OmniPhysicsVolumeDeformableSimAPI") == expect_sim)
    check_pose(test, prim, "bindPose", expect_bindpose)


def check_coll_apis(test, stage, coll_path, expect_coll=True, expect_bindpose=True):
    prim = stage.GetPrimAtPath(coll_path)
    test.assertTrue(prim.IsA(UsdGeom.TetMesh))
    test.assertTrue(prim.HasAPI(UsdPhysics.CollisionAPI) == expect_coll)
    check_pose(test, prim, "bindPose", expect_bindpose)


def check_skin_apis(test, stage, skin_paths, expect_bindpose=True):
    for skin_path in skin_paths:
        prim = stage.GetPrimAtPath(skin_path)
        check_pose(test, prim, "bindPose", expect_bindpose)


def check_pose_points(test, prim, purpose_name, expect_purpose=True):
    pose_name = get_pose_name_with_purpose(prim, purpose_name)
    test.assertTrue(expect_purpose == (pose_name is not None))
    if pose_name is not None:
        pose_point_attr = prim.GetAttribute("deformablePose:" + pose_name + ":omniphysics:points")
        pose_points = pose_point_attr.Get()
        test.assertTrue(len(pose_points) > 0)


def check_sim_attrs(test, stage, sim_path, expect_bindpose=True):
    prim = stage.GetPrimAtPath(sim_path)
    mesh = UsdGeom.TetMesh(prim)
    mesh_points = mesh.GetPointsAttr().Get()
    test.assertTrue(len(mesh_points) > 0)
    mesh_indices = mesh.GetTetVertexIndicesAttr().Get()
    test.assertTrue(len(mesh_indices) > 0)
    rest_shape_points = prim.GetAttribute("omniphysics:restShapePoints").Get()
    test.assertTrue(len(rest_shape_points) > 0)
    rest_shape_indices = prim.GetAttribute("omniphysics:restTetVtxIndices").Get()
    test.assertTrue(len(rest_shape_indices) > 0)
    check_pose_points(test, prim, "bindPose", expect_bindpose)


def check_coll_attrs(test, stage, coll_path, expect_bindpose=True):
    prim = stage.GetPrimAtPath(coll_path)
    mesh = UsdGeom.TetMesh(prim)
    mesh_points = mesh.GetPointsAttr().Get()
    test.assertTrue(len(mesh_points) > 0)
    mesh_indices = mesh.GetTetVertexIndicesAttr().Get()
    test.assertTrue(len(mesh_indices) > 0)
    check_pose_points(test, prim, "bindPose", expect_bindpose)


def check_skin_attrs(test, stage, skin_paths, expect_bindpose=True):
    for skin_path in skin_paths:
        prim = stage.GetPrimAtPath(skin_path)
        check_pose_points(test, prim, "bindPose", expect_bindpose)


def setup_simmesh(stage, path, transform: Gf.Transform, add_collision: bool = False):
    simmesh = UsdGeom.TetMesh.Define(stage, path)
    simmesh.AddTransformOp().Set(transform.GetMatrix())
    set_tetmesh_data(simmesh)
    simmesh.GetPrim().ApplyAPI("OmniPhysicsVolumeDeformableSimAPI")
    simmesh.GetPrim().GetAttribute("omniphysics:restShapePoints").Set(simmesh.GetPointsAttr().Get())
    simmesh.GetPrim().GetAttribute("omniphysics:restTetVtxIndices").Set(simmesh.GetTetVertexIndicesAttr().Get())

    if add_collision:
        UsdPhysics.CollisionAPI.Apply(simmesh.GetPrim())
        surfaceFaceVertexIndices = UsdGeom.TetMesh.ComputeSurfaceFaces(simmesh, Usd.TimeCode.Default())
        simmesh.GetSurfaceFaceVertexIndicesAttr().Set(surfaceFaceVertexIndices)

    return simmesh


class PhysxAutoVolumeDeformableBodyAPITestMemoryStage(PhysicsMemoryStageBaseTestCase):
    category = TestCategory.Core

    def new_stage(self, **kwargs):
        return super().new_stage(def_up_and_mpu=True, up=UsdGeom.Tokens.z, mpu=1.0)

    def step(self, num_steps=1):
        dtime = 1.0/60.0
        super().step(num_steps=num_steps, dt=dtime)

    def test_physx_volume_deformable_from_skin(self):
        stage = self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        xform = setup_xform(stage, "/xform", create_transform(translate=Gf.Vec3d(0.0, 0.0, 5.0)))
        skin_mesh = setup_cube_trimesh(stage, "/xform/skin_mesh", create_transform(scale=Gf.Vec3d(2.0)), 10)

        success = deformableUtils.create_auto_volume_deformable_hierarchy(stage,
            root_prim_path="/xform",
            simulation_tetmesh_path="/xform/sim_mesh",
            collision_tetmesh_path="/xform/sim_mesh",
            cooking_src_mesh_path="/xform/skin_mesh",
            simulation_hex_mesh_enabled=False,
            cooking_src_simplification_enabled=True
        )
        self.assertTrue(success)
        check_body_apis(self, stage, "/xform", cooking_src_path="/xform/skin_mesh")
        check_sim_apis(self, stage, "/xform/sim_mesh")
        check_coll_apis(self, stage, "/xform/sim_mesh")
        check_skin_apis(self, stage, ["/xform/skin_mesh"])

        success = _physx.acquire_physx_cooking_interface().cook_auto_deformable_body("/xform")
        self.assertTrue(success)
        check_sim_attrs(self, stage, "/xform/sim_mesh")
        check_coll_attrs(self, stage, "/xform/sim_mesh")
        check_skin_attrs(self, stage, ["/xform/skin_mesh"])

        # test instantiation
        sim_mesh = UsdGeom.TetMesh(stage.GetPrimAtPath("/xform/sim_mesh"))
        initial_point = sim_mesh.GetPointsAttr().Get()[0]

        # step and check that it's fallen a bit under gravity
        self.step(1)

        post_point = sim_mesh.GetPointsAttr().Get()[0]
        epsilon = 0.001
        self.assertTrue(initial_point[2] - post_point[2] > epsilon)

    def test_physx_volume_deformable_skin_meshes(self):
        stage = self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        xform0 = setup_xform(stage, "/xform0", create_transform(translate=Gf.Vec3d(0.0, 0.0, 5.0)))
        xform0_skin_mesh = setup_cube_trimesh(stage, "/xform0/skin_mesh", create_transform(scale=Gf.Vec3d(2.0)), 10)

        success = deformableUtils.create_auto_volume_deformable_hierarchy(stage,
            root_prim_path="/xform0",
            simulation_tetmesh_path="/xform0/sim_mesh",
            collision_tetmesh_path="/xform0/sim_mesh",
            cooking_src_mesh_path="/xform0/skin_mesh",
            simulation_hex_mesh_enabled=False,
            cooking_src_simplification_enabled=True
        )
        self.assertTrue(success)

        success = _physx.acquire_physx_cooking_interface().cook_auto_deformable_body("/xform0")
        self.assertTrue(success)

        xform1 = setup_xform(stage, "/xform1", create_transform(translate=Gf.Vec3d(5.0, 0.0, 5.0)))
        xform1_skin_mesh = setup_cube_trimesh(stage, "/xform1/skin_mesh", create_transform(scale=Gf.Vec3d(2.0)), 10)

        success = deformableUtils.create_auto_volume_deformable_hierarchy(stage,
            root_prim_path="/xform1",
            simulation_tetmesh_path="/xform1/sim_mesh",
            collision_tetmesh_path="/xform1/sim_mesh",
            cooking_src_mesh_path="/xform1/skin_mesh",
            simulation_hex_mesh_enabled=False,
            cooking_src_simplification_enabled=True
        )
        self.assertTrue(success)

        success = _physx.acquire_physx_cooking_interface().cook_auto_deformable_body("/xform1")
        self.assertTrue(success)

        xform2 = setup_xform(stage, "/xform2", create_transform(translate=Gf.Vec3d(0.0, 5.0, 5.0)))
        xform2.GetPrim().ApplyAPI("OmniPhysicsDeformableBodyAPI")

        # sim tetmesh
        tetmesh_transform = create_transform(translate=Gf.Vec3d(0.01), scale=Gf.Vec3d(2.0))
        xform2_sim_mesh = setup_simmesh(stage, "/xform2/simMesh", tetmesh_transform, add_collision=True)

        # test instantiation
        epsilon = 0.001

        xform0_sim_mesh = UsdGeom.TetMesh(stage.GetPrimAtPath("/xform0/sim_mesh"))
        xform0_initial_point = xform0_sim_mesh.GetPointsAttr().Get()[0]
        xform1_sim_mesh = UsdGeom.TetMesh(stage.GetPrimAtPath("/xform1/sim_mesh"))
        xform1_initial_point = xform1_sim_mesh.GetPointsAttr().Get()[0]
        xform2_initial_point = xform2_sim_mesh.GetPointsAttr().Get()[0]

        # step and check that it's fallen a bit under gravity
        self.step(1)

        xform0_post_point = xform0_sim_mesh.GetPointsAttr().Get()[0]
        self.assertTrue(xform0_initial_point[2] - xform0_post_point[2] > epsilon)
        xform1_post_point = xform1_sim_mesh.GetPointsAttr().Get()[0]
        self.assertTrue(xform1_initial_point[2] - xform1_post_point[2] > epsilon)
        xform2_post_point = xform2_sim_mesh.GetPointsAttr().Get()[0]
        self.assertTrue(xform2_initial_point[2] - xform2_post_point[2] > epsilon)

    def test_physx_volume_deformable_runtime_remove_body(self):
        stage = self.new_stage()
        UsdPhysics.Scene.Define(stage, "/physicsScene")
        xform = setup_xform(stage, "/xform", create_transform(translate=Gf.Vec3d(0.0, 0.0, 5.0)))
        skin_mesh = setup_cube_trimesh(stage, "/xform/skin_mesh", create_transform(scale=Gf.Vec3d(2.0)), 10)

        success = deformableUtils.create_auto_volume_deformable_hierarchy(stage,
            root_prim_path="/xform",
            simulation_tetmesh_path="/xform/sim_mesh",
            collision_tetmesh_path="/xform/sim_mesh",
            cooking_src_mesh_path="/xform/skin_mesh",
            simulation_hex_mesh_enabled=False,
            cooking_src_simplification_enabled=True
        )
        self.assertTrue(success)
        check_body_apis(self, stage, "/xform", cooking_src_path="/xform/skin_mesh")
        check_sim_apis(self, stage, "/xform/sim_mesh")
        check_coll_apis(self, stage, "/xform/sim_mesh")
        check_skin_apis(self, stage, ["/xform/skin_mesh"])

        success = _physx.acquire_physx_cooking_interface().cook_auto_deformable_body("/xform")
        self.assertTrue(success)
        check_sim_attrs(self, stage, "/xform/sim_mesh")
        check_coll_attrs(self, stage, "/xform/sim_mesh")
        check_skin_attrs(self, stage, ["/xform/skin_mesh"])

        sim_mesh = UsdGeom.TetMesh(stage.GetPrimAtPath("/xform/sim_mesh"))

        initial_point = sim_mesh.GetPointsAttr().Get()[0]

        # start simulation
        self.step(2)

        # test falling
        post_sim_point = sim_mesh.GetPointsAttr().Get()[0]
        epsilon = 0.001
        self.assertTrue(initial_point[2] - post_sim_point[2] > epsilon)

        # remove deformable body API
        xform.GetPrim().RemoveAPI("PhysxAutoDeformableBodyAPI")
        xform.GetPrim().RemoveAPI("OmniPhysicsDeformableBodyAPI")

        # resume simulation
        self.step(2)

        # test not falling
        post_remove_point = sim_mesh.GetPointsAttr().Get()[0]
        self.assertTrue(post_sim_point[2] == post_remove_point[2])


if __name__ == "__main__":
    unittest.main()
