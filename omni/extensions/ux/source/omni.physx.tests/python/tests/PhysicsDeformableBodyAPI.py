# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx import get_physx_interface
from omni.physxtests import utils
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory, PhysicsBaseAsyncTestCase
from omni.physx.scripts import deformableMeshUtils, deformableUtils, physicsUtils
import omni.kit.commands
import omni.kit.undo
from pxr import Sdf, Usd, Gf, UsdShade, UsdGeom, UsdPhysics, UsdUtils, Vt, PhysicsSchemaTools


def create_transform(translate = Gf.Vec3d(0.0),
                        rotate = Gf.Rotation(Gf.Quatd(1.0)),
                        scale = Gf.Vec3d(1.0),
                        pivot_pos = Gf.Vec3d(0.0),
                        pivot_orient = Gf.Rotation(Gf.Quatd(1.0))):
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
    obb = imageable.ComputeWorldBound(Usd.TimeCode.Default(), purpose1=imageable.GetPurposeAttr().Get())
    return obb.ComputeAlignedBox()

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
        collision = UsdPhysics.CollisionAPI.Apply(simmesh.GetPrim())
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

#debug class:
#class PhysicsDeformableBodyAPITestDebugStage(PhysicsBaseAsyncTestCase):
class PhysicsDeformableBodyAPITestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    async def new_stage(self):
        class_name = self.__class__.__name__
        if class_name == 'PhysicsDeformableBodyAPITestDebugStage':
            return await utils.new_stage_setup(def_up_and_mpu=True, up=UsdGeom.Tokens.z, mpu=1.0)
        else:
            return await super().new_stage(def_up_and_mpu=True, up=UsdGeom.Tokens.z, mpu=1.0)

    def step(self, num_steps):
        dtime = 1.0/60.0
        class_name = self.__class__.__name__
        if class_name == 'PhysicsDeformableBodyAPITestDebugStage':
            physx_interface = get_physx_interface()
            time = 0.0
            for i in range(num_steps):
                physx_interface.update_simulation(dtime, time)
                physx_interface.update_transformations(True, True, True, False)
                time = time + dtime
        else:
            super().step(num_steps=num_steps, dt=dtime)

    def check_prims_and_apis(self, root_prim, skin_mesh,
        exp_body_api=True, exp_sim_mesh=True, exp_rest_shape_api=True, exp_sim_bind_pose_api=True, exp_skin_bind_pose_api=True):
        (   body_api, sim_mesh, rest_shape_api,
            sim_bind_pose_api, skin_bind_pose_api   ) = get_prims_and_apis(root_prim, skin_mesh)
        self.assertTrue(exp_body_api == bool(body_api))
        self.assertTrue(exp_sim_mesh == bool(sim_mesh))
        self.assertTrue(exp_rest_shape_api == bool(rest_shape_api))
        self.assertTrue(exp_sim_bind_pose_api == bool(sim_bind_pose_api))
        self.assertTrue(exp_skin_bind_pose_api == bool(skin_bind_pose_api))

    def check_absent_prims_and_apis(self, root_prim, skin_mesh,
        exp_body_api=False, exp_sim_mesh=False, exp_rest_shape_api=False, exp_sim_bind_pose_api=False, exp_skin_bind_pose_api=False):
        self.check_prims_and_apis(root_prim, skin_mesh,
            exp_body_api=exp_body_api, exp_sim_mesh=exp_sim_mesh,
            exp_rest_shape_api=exp_rest_shape_api, exp_sim_bind_pose_api=exp_sim_bind_pose_api,
            exp_skin_bind_pose_api=exp_skin_bind_pose_api)

    def check_attributes(self, root_prim, skin_mesh):
        (   body_api, sim_mesh, rest_shape_api,
            sim_bind_pose_api, skin_bind_pose_api   ) = get_prims_and_apis(root_prim, skin_mesh)

        (   sim_mesh_points, sim_mesh_indices, rest_shape_points, rest_shape_indices,
            sim_bind_pose_points, skin_bind_pose_points ) = get_attributes(sim_mesh, rest_shape_api, sim_bind_pose_api, skin_bind_pose_api)
        self.assertTrue(len(sim_mesh_points) > 0)
        self.assertTrue(len(sim_mesh_indices) > 0)
        self.assertTrue(len(rest_shape_points) > 0)
        self.assertTrue(len(rest_shape_indices) > 0)
        self.assertTrue(len(sim_bind_pose_points) == len(sim_mesh_points))
        self.assertTrue(len(skin_bind_pose_points) == len(skin_mesh.GetPointsAttr().Get()))


    async def test_volume_deformable_setup_simmesh(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # sim tetmesh
        transform = create_transform(translate = Gf.Vec3d(1.0), scale = Gf.Vec3d(5.0))
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
        collision = UsdPhysics.CollisionAPI.Apply(tetmesh.GetPrim())
        surfaceFaceVertexIndices = UsdGeom.TetMesh.ComputeSurfaceFaces(tetmesh, Usd.TimeCode.Default())
        tetmesh.GetSurfaceFaceVertexIndicesAttr().Set(surfaceFaceVertexIndices)

        # step and check that it's fallen a bit under gravity
        self.step(1)

        post_transform = tetmesh.GetLocalTransformation()
        post_point= tetmesh.GetPointsAttr().Get()[0]

        # simulation updates happen in mesh space
        self.assertTrue(transform.GetMatrix() == post_transform)
        epsilon = 0.00001
        self.assertTrue(initial_point[2] - post_point[2] > epsilon)

    async def volume_deformable_setup_hierarch(self, use_scope):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # deformable body prim
        if use_scope:
            body_transform = create_transform()
            body = UsdGeom.Scope.Define(stage, "/body")
            body.GetPrim().ApplyAPI("OmniPhysicsDeformableBodyAPI")
        else:
            body_transform = create_transform(translate = Gf.Vec3d(0.1, -0.06, 0.035), scale = Gf.Vec3d(1.0, 2.0, 3.0))
            body = setup_xform_body(stage, "/body", body_transform)

        # sim tetmesh
        tetmesh_transform = create_transform(translate = Gf.Vec3d(0.01), scale = Gf.Vec3d(5.0))
        tetmesh = setup_simmesh(stage, "/body/simMesh", tetmesh_transform, add_collision = True)

        # save point for reference
        initial_point = tetmesh.GetPointsAttr().Get()[0]

        # step and check that it's fallen a bit under gravity
        self.step(1)

        if use_scope:
            post_body_transform = create_transform().GetMatrix()
        else:
            post_body_transform = body.GetLocalTransformation()

        post_tetmesh_transform = tetmesh.GetLocalTransformation()
        post_point= tetmesh.GetPointsAttr().Get()[0]

        # simulation updates happen in mesh space
        self.assertTrue(body_transform.GetMatrix() == post_body_transform)
        self.assertTrue(tetmesh_transform.GetMatrix() == post_tetmesh_transform)
        epsilon = 0.00001
        self.assertTrue(initial_point[2] - post_point[2] > epsilon)

    async def test_volume_deformable_setup_hierarch(self):
        await self.volume_deformable_setup_hierarch(False)
        await self.volume_deformable_setup_hierarch(True)

    async def volume_deformable_setup_skinmeshes(self, num_skinmeshes):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        meshscale = 0.5

        # deformable body prim
        xform_transform = create_transform(translate = Gf.Vec3d(0.0,1.0,0.0), scale = Gf.Vec3d(1.0, 2.0, 3.0))
        xform = setup_xform_body(stage, "/xform", xform_transform)
        
        # sim tetmesh
        tetmesh_transform = create_transform(translate = Gf.Vec3d(0.01), scale = Gf.Vec3d(meshscale))
        tetmesh = setup_simmesh(stage, "/xform/simMesh", tetmesh_transform, add_collision = True)

        # skin meshes
        skinmeshes = [None]*num_skinmeshes
        skinmesh_transforms = [None]*num_skinmeshes
        initial_skinmesh_points = [None]*num_skinmeshes
        post_skinmesh_transforms = [None]*num_skinmeshes
        post_skinmesh_points = [None]*num_skinmeshes

        scalefactor = 1.0
        for i in range(num_skinmeshes):
            #make scale a little smaller than the previous for matrjoschka skin meshes
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
            post_skinmesh_points[i]= skinmeshes[i].GetPointsAttr().Get()[0]

        post_tetmesh_point= tetmesh.GetPointsAttr().Get()[0]

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


    async def test_volume_deformable_setup_skinmeshes(self):
        await self.volume_deformable_setup_skinmeshes(1)
        await self.volume_deformable_setup_skinmeshes(2)
        await self.volume_deformable_setup_skinmeshes(3)

    async def volume_deformable_setup_material(self, purpose):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        physicsUtils.add_ground_plane(stage, "/groundPlane", UsdGeom.GetStageUpAxis(stage), 10.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        meshscale = 0.5

        # deformable body prim
        xform = setup_xform_body(stage, "/xform", create_transform())

        # sim tetmesh
        tetmesh_transform = create_transform(translate = Gf.Vec3d(0.0, 0.0, meshscale/2.0), scale = Gf.Vec3d(meshscale))
        tetmesh = setup_simmesh(stage, "/xform/simMesh", tetmesh_transform, add_collision = True)

        #debugging
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

        get_physx_interface().reset_simulation()

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

    async def test_volume_deformable_setup_material(self):
        await self.volume_deformable_setup_material("physics")
        await self.volume_deformable_setup_material("")

    async def test_volume_deformable_runtime_remove_body(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # deformable body prim
        body_transform = create_transform(translate = Gf.Vec3d(0.1, -0.06, 0.035), scale = Gf.Vec3d(1.0, 2.0, 3.0))
        xform = setup_xform_body(stage, "/body", body_transform)

        # sim tetmesh
        tetmesh_transform = create_transform(translate = Gf.Vec3d(0.01), scale = Gf.Vec3d(5.0))
        tetmesh = setup_simmesh(stage, "/body/simMesh", tetmesh_transform, add_collision = True)

        initial_point = tetmesh.GetPointsAttr().Get()[0]

        #start simulation
        self.step(2)

        #test falling
        post_sim_point = tetmesh.GetPointsAttr().Get()[0]
        epsilon = 0.00001
        self.assertTrue(initial_point[2] - post_sim_point[2] > epsilon)

        #remove deformable body API
        xform.GetPrim().RemoveAPI("OmniPhysicsDeformableBodyAPI")

        #resume simulation
        self.step(2)

        #test not falling
        post_remove_point = tetmesh.GetPointsAttr().Get()[0]
        self.assertTrue(post_sim_point[2] == post_remove_point[2])


##########################################################################################
##########################################################################################
##########################################################################################

class PhysicsDeformableBodyAPITestStage(PhysicsBaseAsyncTestCase):
    category = TestCategory.Core

    async def new_stage(self):
        class_name = self.__class__.__name__
        return await utils.new_stage_setup(def_up_and_mpu=True, up=UsdGeom.Tokens.z, mpu=1.0)

    def step(self, num_steps):
        dtime = 1.0/60.0
        physx_interface = get_physx_interface()
        time = 0.0
        for i in range(num_steps):
            physx_interface.update_simulation(dtime, time)
            physx_interface.update_transformations(True, True, True, False)
            time = time + dtime

    async def test_volume_deformable_setup_material_command(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")
        physicsUtils.add_ground_plane(stage, "/groundPlane", UsdGeom.GetStageUpAxis(stage), 10.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        meshscale = 0.5

        # deformable body prim
        xform = setup_xform_body(stage, "/xform", create_transform())

        # sim tetmesh
        tetmesh_transform = create_transform(translate = Gf.Vec3d(0.0, 0.0, meshscale/2.0), scale = Gf.Vec3d(meshscale))
        tetmesh = setup_simmesh(stage, "/xform/simMesh", tetmesh_transform, add_collision = True)

        #debugging
        skinmesh_debug = setup_skinmesh_from_tetmesh(stage, "/xform/triMesh", tetmesh, 0.95)

        # sim with default material
        initial_world_extents_default = compute_world_bounds(tetmesh)
        self.step(100)
        post_world_extents_default = compute_world_bounds(tetmesh)
        get_physx_interface().reset_simulation()

        # material
        success = omni.kit.commands.execute("AddDeformableMaterial", stage=stage, path="/deformableMaterial",
            density=1e3, youngs_modulus=1e4, poissons_ratio=0.45, dynamic_friction=0.25) # default is 5e5 on m stage
        self.assertTrue(success)
        material_prim = stage.GetPrimAtPath("/deformableMaterial")
        material_prim.ApplyAPI("OmniPhysicsDeformableMaterialAPI")

        # bind material
        omni.kit.commands.execute("BindMaterial", prim_path=tetmesh.GetPrim().GetPath(),
            material_path="/deformableMaterial", strength=UsdShade.Tokens.weakerThanDescendants)
        binding = UsdShade.MaterialBindingAPI.Apply(tetmesh.GetPrim())

        #sim with softer than default material
        initial_world_extents_softer = compute_world_bounds(tetmesh)
        self.step(100)
        post_world_extents_softer = compute_world_bounds(tetmesh)

        # check bounds, initial state, collision with ground
        self.assertTrue(initial_world_extents_softer.GetMin() == initial_world_extents_default.GetMin())
        self.assertTrue(initial_world_extents_softer.GetMax() == initial_world_extents_default.GetMax())
        self.assertTrue(post_world_extents_softer.GetMin()[2] > -0.005 and post_world_extents_softer.GetMin()[2] < 0.05)
        self.assertTrue(post_world_extents_default.GetMin()[2] > -0.005 and post_world_extents_default.GetMin()[2] < 0.05)

        # softer deformable top should be lower than default deformable top
        print(f"default {post_world_extents_default.GetMax()[2]}, soft {post_world_extents_softer.GetMax()[2]}")
        self.assertTrue(post_world_extents_default.GetMax()[2] > post_world_extents_softer.GetMax()[2] + 0.02)
