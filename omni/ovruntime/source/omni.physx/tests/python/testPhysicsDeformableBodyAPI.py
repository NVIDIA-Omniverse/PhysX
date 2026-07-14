# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Ported from omni.physx.tests PhysicsDeformableBodyAPI.py for standalone ovruntime testing.
# Only the MemoryStage class is ported; the TestStage class requires Kit commands and is not portable.

import _physics_setup  # noqa: F401 - loads Carbonite + physics plugins + PhysX schemas

import unittest
import _physx
from physicsBase import PhysicsMemoryStageBaseTestCase, TestCategory
import physicsUtils
from omni.physx.scripts import deformableMeshUtils, deformableUtils
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade


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


def set_trimesh_data_from_tetmesh_surface(trimesh, tetmesh):
    tet_indices_flat = [e for vec in tetmesh.GetTetVertexIndicesAttr().Get() for e in vec]
    tri_points, tri_indices = deformableUtils.extractTriangleSurfaceFromTetra(tetmesh.GetPointsAttr().Get(), tet_indices_flat)
    trimesh.GetPointsAttr().Set(tri_points)
    trimesh.GetFaceVertexCountsAttr().Set([3]*(len(tri_indices)//3))
    trimesh.GetFaceVertexIndicesAttr().Set(tri_indices)


def compute_world_bounds(imageable: UsdGeom.Imageable) -> Gf.Range3d:
    """Compute world-space AABB from points and local-to-world transform."""
    prim = imageable.GetPrim()
    pointBased = UsdGeom.PointBased(prim)
    aabb = Gf.Range3d()
    if pointBased:
        transform = pointBased.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        points = pointBased.GetPointsAttr().Get()
        for p in points:
            aabb.UnionWith(transform.Transform(Gf.Vec3d(p)))
    else:
        boundable = UsdGeom.Boundable(prim)
        if boundable:
            extent = boundable.GetExtentAttr().Get()
            if extent:
                transform = boundable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                for corner in [
                    Gf.Vec3d(extent[0][0], extent[0][1], extent[0][2]),
                    Gf.Vec3d(extent[1][0], extent[0][1], extent[0][2]),
                    Gf.Vec3d(extent[0][0], extent[1][1], extent[0][2]),
                    Gf.Vec3d(extent[0][0], extent[0][1], extent[1][2]),
                    Gf.Vec3d(extent[1][0], extent[1][1], extent[0][2]),
                    Gf.Vec3d(extent[1][0], extent[0][1], extent[1][2]),
                    Gf.Vec3d(extent[0][0], extent[1][1], extent[1][2]),
                    Gf.Vec3d(extent[1][0], extent[1][1], extent[1][2]),
                ]:
                    aabb.UnionWith(transform.Transform(corner))
    return aabb


def setup_xform_body(stage, path, transform: Gf.Transform):
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTransformOp().Set(transform.GetMatrix())
    xform.GetPrim().ApplyAPI("OmniPhysicsDeformableBodyAPI")
    return xform


def setup_simmesh(stage, path, transform: Gf.Transform, add_collision: bool = False):
    simmesh = UsdGeom.TetMesh.Define(stage, path)
    simmesh.AddTransformOp().Set(transform.GetMatrix())
    set_tetmesh_data(simmesh)
    # add sim api
    simmesh.GetPrim().ApplyAPI("OmniPhysicsVolumeDeformableSimAPI")
    simmesh.GetPrim().GetAttribute("omniphysics:restShapePoints").Set(simmesh.GetPointsAttr().Get())
    simmesh.GetPrim().GetAttribute("omniphysics:restTetVtxIndices").Set(simmesh.GetTetVertexIndicesAttr().Get())

    # add collision
    if add_collision:
        UsdPhysics.CollisionAPI.Apply(simmesh.GetPrim())
        surfaceFaceVertexIndices = UsdGeom.TetMesh.ComputeSurfaceFaces(simmesh, Usd.TimeCode.Default())
        simmesh.GetSurfaceFaceVertexIndicesAttr().Set(surfaceFaceVertexIndices)

    return simmesh


def setup_skinmesh_from_tetmesh(stage, path, tetmesh, scalefactor):
    skinmesh = UsdGeom.Mesh.Define(stage, path)
    tetmesh_transform = Gf.Transform(tetmesh.GetLocalTransformation())
    tetmesh_transform.SetScale(tetmesh_transform.GetScale()*scalefactor)
    skinmesh.AddTransformOp().Set(tetmesh_transform.GetMatrix())
    set_trimesh_data_from_tetmesh_surface(skinmesh, tetmesh)
    return skinmesh


class PhysicsDeformableBodyAPITestMemoryStage(PhysicsMemoryStageBaseTestCase):
    category = TestCategory.Core

    def new_stage(self, **kwargs):
        return super().new_stage(def_up_and_mpu=True, up=UsdGeom.Tokens.z, mpu=1.0)

    def step(self, num_steps=1):
        dtime = 1.0/60.0
        super().step(num_steps=num_steps, dt=dtime)

    def test_volume_deformable_setup_simmesh(self):
        stage = self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # sim tetmesh
        transform = create_transform(translate=Gf.Vec3d(1.0), scale=Gf.Vec3d(5.0))
        tetmesh = UsdGeom.TetMesh.Define(stage, "/tetMesh")
        tetmesh.AddTransformOp().Set(transform.GetMatrix())
        set_tetmesh_data(tetmesh)

        # save point for reference
        initial_point = tetmesh.GetPointsAttr().Get()[0]

        # deformable body and sim api
        tetmesh.GetPrim().ApplyAPI("OmniPhysicsDeformableBodyAPI")
        tetmesh.GetPrim().ApplyAPI("OmniPhysicsVolumeDeformableSimAPI")
        tetmesh.GetPrim().GetAttribute("omniphysics:restShapePoints").Set(tetmesh.GetPointsAttr().Get())
        tetmesh.GetPrim().GetAttribute("omniphysics:restTetVtxIndices").Set(tetmesh.GetTetVertexIndicesAttr().Get())

        # need to add collision, currently no support without
        UsdPhysics.CollisionAPI.Apply(tetmesh.GetPrim())
        surfaceFaceVertexIndices = UsdGeom.TetMesh.ComputeSurfaceFaces(tetmesh, Usd.TimeCode.Default())
        tetmesh.GetSurfaceFaceVertexIndicesAttr().Set(surfaceFaceVertexIndices)

        # step and check that it's fallen a bit under gravity
        self.step(1)

        post_transform = tetmesh.GetLocalTransformation()
        post_point = tetmesh.GetPointsAttr().Get()[0]

        # simulation updates happen in mesh space
        self.assertTrue(transform.GetMatrix() == post_transform)
        epsilon = 0.00001
        self.assertTrue(initial_point[2] - post_point[2] > epsilon)

    def volume_deformable_setup_hierarch(self, use_scope):
        stage = self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # deformable body prim
        if use_scope:
            body_transform = create_transform()
            body = UsdGeom.Scope.Define(stage, "/body")
            body.GetPrim().ApplyAPI("OmniPhysicsDeformableBodyAPI")
        else:
            body_transform = create_transform(translate=Gf.Vec3d(0.1, -0.06, 0.035), scale=Gf.Vec3d(1.0, 2.0, 3.0))
            body = setup_xform_body(stage, "/body", body_transform)

        # sim tetmesh
        tetmesh_transform = create_transform(translate=Gf.Vec3d(0.01), scale=Gf.Vec3d(5.0))
        tetmesh = setup_simmesh(stage, "/body/simMesh", tetmesh_transform, add_collision=True)

        # save point for reference
        initial_point = tetmesh.GetPointsAttr().Get()[0]

        # step and check that it's fallen a bit under gravity
        self.step(1)

        if use_scope:
            post_body_transform = create_transform().GetMatrix()
        else:
            post_body_transform = body.GetLocalTransformation()

        post_tetmesh_transform = tetmesh.GetLocalTransformation()
        post_point = tetmesh.GetPointsAttr().Get()[0]

        # simulation updates happen in mesh space
        self.assertTrue(body_transform.GetMatrix() == post_body_transform)
        self.assertTrue(tetmesh_transform.GetMatrix() == post_tetmesh_transform)
        epsilon = 0.00001
        self.assertTrue(initial_point[2] - post_point[2] > epsilon)

    def test_volume_deformable_setup_hierarch(self):
        self.volume_deformable_setup_hierarch(False)
        self.volume_deformable_setup_hierarch(True)

    def volume_deformable_setup_skinmeshes(self, num_skinmeshes):
        stage = self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        meshscale = 0.5

        # deformable body prim
        xform_transform = create_transform(translate=Gf.Vec3d(0.0, 1.0, 0.0), scale=Gf.Vec3d(1.0, 2.0, 3.0))
        xform = setup_xform_body(stage, "/xform", xform_transform)

        # sim tetmesh
        tetmesh_transform = create_transform(translate=Gf.Vec3d(0.01), scale=Gf.Vec3d(meshscale))
        tetmesh = setup_simmesh(stage, "/xform/simMesh", tetmesh_transform, add_collision=True)

        # skin meshes
        skinmeshes = [None]*num_skinmeshes
        skinmesh_transforms = [None]*num_skinmeshes
        initial_skinmesh_points = [None]*num_skinmeshes
        post_skinmesh_transforms = [None]*num_skinmeshes
        post_skinmesh_points = [None]*num_skinmeshes

        scalefactor = 1.0
        for i in range(num_skinmeshes):
            scalefactor = scalefactor*0.95
            skinmeshes[i] = setup_skinmesh_from_tetmesh(stage, "/xform/triMesh_" + str(i), tetmesh, scalefactor)
            skinmesh_transforms[i] = skinmeshes[i].GetLocalTransformation()

        # save point for reference
        for i in range(num_skinmeshes):
            initial_skinmesh_points[i] = skinmeshes[i].GetPointsAttr().Get()[0]

        initial_tetmesh_point = tetmesh.GetPointsAttr().Get()[0]

        # step and check that it's fallen a bit under gravity
        self.step(1)

        post_xform_transform = xform.GetLocalTransformation()
        post_tetmesh_transform = tetmesh.GetLocalTransformation()

        for i in range(num_skinmeshes):
            post_skinmesh_transforms[i] = skinmeshes[i].GetLocalTransformation()
            post_skinmesh_points[i] = skinmeshes[i].GetPointsAttr().Get()[0]

        post_tetmesh_point = tetmesh.GetPointsAttr().Get()[0]

        # simulation updates happen in mesh space
        self.assertTrue(xform_transform.GetMatrix() == post_xform_transform)
        self.assertTrue(tetmesh_transform.GetMatrix() == post_tetmesh_transform)
        for i in range(num_skinmeshes):
            self.assertTrue(skinmesh_transforms[i] == post_skinmesh_transforms[i])

        epsilon = 0.00001
        for i in range(num_skinmeshes):
            self.assertTrue(initial_skinmesh_points[i][2] - post_skinmesh_points[i][2] > epsilon)

        self.assertTrue(initial_tetmesh_point[0] - post_tetmesh_point[0] < epsilon)
        self.assertTrue(initial_tetmesh_point[1] - post_tetmesh_point[1] < epsilon)
        self.assertTrue(initial_tetmesh_point[2] - post_tetmesh_point[2] > epsilon)

    def test_volume_deformable_setup_skinmeshes(self):
        self.volume_deformable_setup_skinmeshes(1)
        self.volume_deformable_setup_skinmeshes(2)
        self.volume_deformable_setup_skinmeshes(3)

    def volume_deformable_setup_material(self, purpose):
        stage = self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        physicsUtils.add_ground_plane(stage, "/groundPlane", UsdGeom.GetStageUpAxis(stage), 10.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        meshscale = 0.5

        # deformable body prim
        xform = setup_xform_body(stage, "/xform", create_transform())

        # sim tetmesh
        tetmesh_transform = create_transform(translate=Gf.Vec3d(0.0, 0.0, meshscale/2.0), scale=Gf.Vec3d(meshscale))
        tetmesh = setup_simmesh(stage, "/xform/simMesh", tetmesh_transform, add_collision=True)

        # debugging
        skinmesh_debug = setup_skinmesh_from_tetmesh(stage, "/xform/triMesh", tetmesh, 0.95)

        # materials
        materials = [None]*2

        for i in range(2):
            material_prim = stage.DefinePrim("/deformableMaterial_" + str(i), "Material")
            materials[i] = UsdShade.Material(material_prim)
            material_prim.ApplyAPI("OmniPhysicsDeformableMaterialAPI")
            material_prim.GetAttribute("omniphysics:density").Set(1e3)
            material_prim.GetAttribute("omniphysics:youngsModulus").Set(2e4 * (10**i))
            material_prim.GetAttribute("omniphysics:poissonsRatio").Set(0.45)

        binding = UsdShade.MaterialBindingAPI.Apply(tetmesh.GetPrim())

        # apply soft material
        binding.Bind(materials[0], UsdShade.Tokens.weakerThanDescendants, purpose)
        initial_world_extents_soft = compute_world_bounds(tetmesh)
        self.step(100)
        post_world_extents_soft = compute_world_bounds(tetmesh)

        _physx.acquire_physx_interface().reset_simulation()

        # apply hard material
        binding.Bind(materials[1], UsdShade.Tokens.weakerThanDescendants, purpose)
        initial_world_extents_hard = compute_world_bounds(tetmesh)
        self.step(100)
        post_world_extents_hard = compute_world_bounds(tetmesh)

        # check bounds, initial state, collision with ground
        self.assertTrue(initial_world_extents_soft.GetMin() == initial_world_extents_hard.GetMin())
        self.assertTrue(initial_world_extents_soft.GetMax() == initial_world_extents_hard.GetMax())
        self.assertTrue(post_world_extents_soft.GetMin()[2] > -0.05 and post_world_extents_soft.GetMin()[2] < 0.05)
        self.assertTrue(post_world_extents_hard.GetMin()[2] > -0.05 and post_world_extents_hard.GetMin()[2] < 0.05)

        # soft deformable top should be lower than hard deformable top
        self.assertTrue(post_world_extents_hard.GetMax()[2] > post_world_extents_soft.GetMax()[2] + 0.05)

    def test_volume_deformable_setup_material(self):
        self.volume_deformable_setup_material("physics")
        self.volume_deformable_setup_material("")

    def test_volume_deformable_runtime_remove_body(self):
        stage = self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # deformable body prim
        body_transform = create_transform(translate=Gf.Vec3d(0.1, -0.06, 0.035), scale=Gf.Vec3d(1.0, 2.0, 3.0))
        xform = setup_xform_body(stage, "/body", body_transform)

        # sim tetmesh
        tetmesh_transform = create_transform(translate=Gf.Vec3d(0.01), scale=Gf.Vec3d(5.0))
        tetmesh = setup_simmesh(stage, "/body/simMesh", tetmesh_transform, add_collision=True)

        initial_point = tetmesh.GetPointsAttr().Get()[0]

        # start simulation
        self.step(2)

        # test falling
        post_sim_point = tetmesh.GetPointsAttr().Get()[0]
        epsilon = 0.00001
        self.assertTrue(initial_point[2] - post_sim_point[2] > epsilon)

        # remove deformable body API
        xform.GetPrim().RemoveAPI("OmniPhysicsDeformableBodyAPI")

        # resume simulation
        self.step(2)

        # test not falling
        post_remove_point = tetmesh.GetPointsAttr().Get()[0]
        self.assertTrue(post_sim_point[2] == post_remove_point[2])


if __name__ == "__main__":
    unittest.main()
