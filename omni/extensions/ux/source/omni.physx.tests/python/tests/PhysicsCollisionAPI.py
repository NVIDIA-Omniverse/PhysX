# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.test
import omni.kit.stage_templates
import omni.usd
from omni.physx.scripts.physicsUtils import *
from pxr import UsdGeom, UsdShade, Sdf, Gf, UsdPhysics, PhysxSchema, Usd, UsdUtils
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physxtests import utils
from omni.physx.scripts import physicsUtils
from omni.physx.scripts import utils as physxUtils
import unittest
from omni.physx import get_physxunittests_interface, get_physx_simulation_interface
from omni.physxcommands import SetStaticColliderCommand, RemoveStaticColliderCommand
from omni.physxtests.utils import rigidbody
import os

class PhysicsCollisionAPITestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    def setup_multimaterial_mesh(self, materialCount, stage, simplified):
        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        material_scope_path = defaultPrimPath + "/Looks"
        UsdGeom.Scope.Define(stage, material_scope_path)

        # Trianglemesh materials
        for i in range(materialCount):
            mtl_path = material_scope_path + "/OmniPBR" + str(i)

            mu = 0.0 + (i % 10) * 0.1

            mat_prim = stage.DefinePrim(mtl_path, "Material")
            material_prim = UsdShade.Material.Get(stage, mat_prim.GetPath())
            material = UsdPhysics.MaterialAPI.Apply(material_prim.GetPrim())
            material.CreateRestitutionAttr().Set(mu)

        # Triangle mesh with multiple materials
        stripSize = 100.0
        path = defaultPrimPath + "/triangleMesh"
        self._mesh_path = path
        mesh = UsdGeom.Mesh.Define(stage, path)
        halfSize = 500.0        

        # Fill in VtArrays
        points = []
        normals = []
        indices = []
        vertexCounts = []

        for i in range(materialCount):
            subset = UsdGeom.Subset.Define(stage, path + "/subset" + str(i))
            subset.CreateElementTypeAttr().Set("face")
            subset_indices = [i]
            bindingAPI = UsdShade.MaterialBindingAPI.Apply(subset.GetPrim())
            materialPrim = UsdShade.Material(stage.GetPrimAtPath(material_scope_path + "/OmniPBR" + str(i)))
            bindingAPI.Bind(materialPrim, UsdShade.Tokens.weakerThanDescendants, "physics")
        
            points.append(Gf.Vec3f(-stripSize/2.0 + stripSize * i, -halfSize, 0.0))
            points.append(Gf.Vec3f(-stripSize/2.0 + stripSize * (i + 1), -halfSize, 0.0))
            points.append(Gf.Vec3f(-stripSize/2.0 + stripSize * (i + 1), halfSize, 0.0))
            points.append(Gf.Vec3f(-stripSize/2.0 + stripSize * i, halfSize, 0.0))
            
            for j in range(4):
                normals.append(Gf.Vec3f(0, 0, 1))
                indices.append(j + i * 4)                

            subset.CreateIndicesAttr().Set(subset_indices)
            vertexCounts.append(4)

        mesh.CreateFaceVertexCountsAttr().Set(vertexCounts)
        mesh.CreateFaceVertexIndicesAttr().Set(indices)
        mesh.CreatePointsAttr().Set(points)
        mesh.CreateDoubleSidedAttr().Set(False)
        mesh.CreateNormalsAttr().Set(normals)
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())

        if simplified:
            meshCollisionAPI.CreateApproximationAttr().Set("meshSimplification")
        else:
            meshCollisionAPI.CreateApproximationAttr().Set("none")

    async def test_physics_multi_material(self):
        stage = await self.new_stage()

        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        # Physics scene
        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")        
    
        self.setup_multimaterial_mesh(10, stage, False)

        self.step()

        utils.check_stats(self, {"numTriMeshShapes": 1, "numStaticRigids": 1})

    async def test_physics_multi_material_simplified(self):
        stage = await self.new_stage()

        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        # Physics scene
        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")        
    
        self.setup_multimaterial_mesh(10, stage, True)

        self.step()

        utils.check_stats(self, {"numTriMeshShapes": 1, "numStaticRigids": 1})


    async def test_physics_convex_decomposition_material(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # material
        materialPath = "/Material"
        UsdShade.Material.Define(stage, materialPath)

        physicsMaterialPath = "/PhysicsMaterial"
        mat = UsdShade.Material.Define(stage, physicsMaterialPath)
        UsdPhysics.MaterialAPI.Apply(mat.GetPrim())

        # box0 without a PhysicsMaterialAPI added
        box0CollisionPath = "/box0"

        cubeGeom = create_mesh_cube(stage, box0CollisionPath, 10.0)        
        UsdPhysics.CollisionAPI.Apply(cubeGeom.GetPrim())
        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(cubeGeom.GetPrim())
        meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")

        rel = cubeGeom.GetPrim().CreateRelationship("material:binding", False)
        rel.SetTargets([Sdf.Path(materialPath)])

        # box1 with a PhysicsMaterialAPI added
        box1CollisionPath = "/box1"

        cubeGeom = create_mesh_cube(stage, box1CollisionPath, 10.0)        
        UsdPhysics.CollisionAPI.Apply(cubeGeom.GetPrim())
        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(cubeGeom.GetPrim())
        meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")

        rel = cubeGeom.GetPrim().CreateRelationship("material:binding", False)
        rel.SetTargets([Sdf.Path(physicsMaterialPath)])

        self.step()

        utils.check_stats(self, {"numConvexShapes": 2, "numStaticRigids": 2})

        materials_list = get_physxunittests_interface().get_materials_paths(box0CollisionPath)
        self.assertTrue(len(materials_list) == 0)

        materials_list = get_physxunittests_interface().get_materials_paths(box1CollisionPath)
        self.assertTrue(len(materials_list) == 1)
        self.assertTrue(materials_list[0] == physicsMaterialPath)


    async def test_physics_collision_material(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # material
        materialPath = "/Material"
        UsdShade.Material.Define(stage, materialPath)

        physicsMaterialPath = "/PhysicsMaterial"
        mat = UsdShade.Material.Define(stage, physicsMaterialPath)
        UsdPhysics.MaterialAPI.Apply(mat.GetPrim())

        # box0 without a PhysicsMaterialAPI added
        box0CollisionPath = "/box0"

        cubeGeom = UsdGeom.Cube.Define(stage, box0CollisionPath)
        UsdPhysics.CollisionAPI.Apply(cubeGeom.GetPrim())

        rel = cubeGeom.GetPrim().CreateRelationship("material:binding", False)
        rel.SetTargets([Sdf.Path(materialPath)])

        # box1 with a PhysicsMaterialAPI added as material:binding
        box1CollisionPath = "/box1"

        cubeGeom = UsdGeom.Cube.Define(stage, box1CollisionPath)
        UsdPhysics.CollisionAPI.Apply(cubeGeom.GetPrim())

        rel = cubeGeom.GetPrim().CreateRelationship("material:binding", False)
        rel.SetTargets([Sdf.Path(physicsMaterialPath)])

        # box2 with a PhysicsMaterialAPI added as material:binding:physics
        box2CollisionPath = "/box2"

        cubeGeom = UsdGeom.Cube.Define(stage, box2CollisionPath)
        UsdPhysics.CollisionAPI.Apply(cubeGeom.GetPrim())

        rel = cubeGeom.GetPrim().CreateRelationship("material:binding:physics", False)
        rel.SetTargets([Sdf.Path(physicsMaterialPath)])

        self.step()

        utils.check_stats(self, {"numBoxShapes": 3, "numStaticRigids": 3})

        materials_list = get_physxunittests_interface().get_materials_paths(box0CollisionPath)
        self.assertTrue(len(materials_list) == 0)

        materials_list = get_physxunittests_interface().get_materials_paths(box1CollisionPath)
        self.assertTrue(len(materials_list) == 1)
        self.assertTrue(materials_list[0] == physicsMaterialPath)        

        materials_list = get_physxunittests_interface().get_materials_paths(box2CollisionPath)
        self.assertTrue(len(materials_list) == 1)
        self.assertTrue(materials_list[0] == physicsMaterialPath)                

    async def test_physics_collision_not_existing_material_prim(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # material without the prim
        materialPath = "/Material"

        # box0 without a PhysicsMaterialAPI added
        box0CollisionPath = "/box0"

        cubeGeom = UsdGeom.Cube.Define(stage, box0CollisionPath)
        UsdPhysics.CollisionAPI.Apply(cubeGeom.GetPrim())

        rel = cubeGeom.GetPrim().CreateRelationship("material:binding", False)
        rel.SetTargets([Sdf.Path(materialPath)])

        self.step()

        utils.check_stats(self, {"numBoxShapes": 1, "numStaticRigids": 1})

        materials_list = get_physxunittests_interface().get_materials_paths(box0CollisionPath)
        self.assertTrue(len(materials_list) == 0)

    async def test_physics_contact_offset_error(self):
        stage = await self.new_stage()

        cube = UsdGeom.Cube.Define(stage, "/World/cube").GetPrim()
        UsdPhysics.CollisionAPI.Apply(cube)
        collision_api = PhysxSchema.PhysxCollisionAPI.Apply(cube)
        collision_api.GetContactOffsetAttr().Set(0.2)
        collision_api.GetRestOffsetAttr().Set(0.1)

        self.step()
        
        message = f"Collision contact offset must be positive and greater then restOffset, prim: {cube.GetPrimPath()}"
        with utils.ExpectMessage(self, message):
            collision_api.GetContactOffsetAttr().Set(-1.0)
        with utils.ExpectMessage(self, message):
            collision_api.GetContactOffsetAttr().Set(0.1)
        collision_api.GetContactOffsetAttr().Set(0.3)
        
    async def test_physics_rest_offset_error(self):
        stage = await self.new_stage()

        cube = UsdGeom.Cube.Define(stage, "/World/cube").GetPrim()
        UsdPhysics.CollisionAPI.Apply(cube)
        collision_api = PhysxSchema.PhysxCollisionAPI.Apply(cube)
        collision_api.GetContactOffsetAttr().Set(0.2)
        collision_api.GetRestOffsetAttr().Set(0.1)

        self.step()
        
        message = f"Collision rest offset must be lesser then contact offset, prim: {cube.GetPrimPath()}"
        with utils.ExpectMessage(self, message):
            collision_api.GetRestOffsetAttr().Set(0.3)
        collision_api.GetRestOffsetAttr().Set(-0.1)        

    async def test_mesh_approximations_set_collider(self):
        for name, api in physxUtils.MESH_APPROXIMATIONS.items():
            print(name)
            stage = await self.new_stage()
            path = "/meshCube"
            cube = physicsUtils.create_mesh_cube(stage, path, 20.0)
            prim = cube.GetPrim()
            physxUtils.setCollider(prim, name)

            # check api presence
            if api is not None:
                inst = api.Get(stage, path)
                self.assertTrue(inst is not None)

            # check param is set
            mesh_api = UsdPhysics.MeshCollisionAPI.Get(stage, path)
            self.assertTrue(mesh_api is not None)
            self.assertTrue(mesh_api.GetApproximationAttr().Get() == name)

            # check UI errors
            omni.usd.get_context().get_selection().set_selected_prim_paths([path], True)
            await self.wait(1)

    async def test_OM_79338_GPU_collision_new_object_passthrough(self):
        for use_GPU in (True, False):
            with self.subTest(use_GPU=use_GPU):
                stage = await self.new_stage()
                physics_scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
                physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
                physx_scene.CreateEnableGPUDynamicsAttr().Set(use_GPU)
                if use_GPU:
                    physx_scene.CreateBroadphaseTypeAttr().Set("GPU")
                else:
                    physx_scene.CreateBroadphaseTypeAttr().Set("MBP")

                self.step(1)
                table_prim = physicsUtils.add_rigid_box(stage, "/World/base_link", Gf.Vec3f(200, 1, 100))
                UsdPhysics.ArticulationRootAPI.Apply(table_prim)
                PhysxSchema.PhysxArticulationAPI.Apply(table_prim)
                fixed_base_joint = UsdPhysics.FixedJoint.Define(stage, "/World/base_link/FixedJoint")
                fixed_base_joint.CreateBody1Rel().SetTargets(["/World/base_link"])
                box_prim = physicsUtils.add_box(stage, "/World/base_link_01", Gf.Vec3f(10, 10, 10), position=Gf.Vec3f(0, 5.5, 0))

                self.step(1)
                UsdPhysics.CollisionAPI.Apply(box_prim)
                UsdPhysics.RigidBodyAPI.Apply(box_prim)
                self.step(10)

                position = box_prim.GetAttribute('xformOp:translate').Get()
                self.assertFloatIterableAlmostEqual(position, Gf.Vec3f(0, 0.55, 0), rel_tol=1)

    async def test_runtime_mesh_changes(self):
        stage = await self.new_stage()

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        halfSize = 100
        points = [
            Gf.Vec3f(-halfSize, 0.0, -halfSize),
            Gf.Vec3f(halfSize, 0.0, -halfSize),
            Gf.Vec3f(halfSize, 0.0, halfSize),
            Gf.Vec3f(-halfSize, 0.0, halfSize),
        ]
        indices = [0, 2, 1, 0, 3, 2]
        normals = []
        vertexCounts = [3, 3]

        # Create the mesh
        mesh = physicsUtils.create_mesh(stage, "/meshActor", points, normals, indices, vertexCounts)
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())        

        # setup a rigid body
        radius = 5.0
        position = Gf.Vec3f(0.0, 50.0, 50.0)
        sphere_prim = physicsUtils.add_rigid_sphere(stage, "/sphere", radius, position)

        for _ in range(50):
            self.step()
            
        current_pos = physicsUtils.get_translation(sphere_prim)        
        self.assertTrue(current_pos[1] > 0.0)
        print(current_pos)

        sphere_prim.GetAttribute("xformOp:translate").Set(position)

        halfSize = 10
        points = [
            Gf.Vec3f(-halfSize, 0.0, -halfSize),
            Gf.Vec3f(halfSize, 0.0, -halfSize),
            Gf.Vec3f(halfSize, 0.0, halfSize),
            Gf.Vec3f(-halfSize, 0.0, halfSize),
        ]

        # Create the mesh again
        mesh = physicsUtils.create_mesh(stage, "/meshActor", points, normals, indices, vertexCounts)
        mesh.GetPrim().RemoveAPI(UsdPhysics.CollisionAPI)
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())        

        for _ in range(50):
            self.step()
            
        current_pos = physicsUtils.get_translation(sphere_prim)        
        print(current_pos)
        self.assertTrue(current_pos[1] < 0.0)


class PhysicsCollisionAPITestAsyncRB(rigidbody.AsyncTestCase):
    category = TestCategory.Kit

    async def test_physics_commands_SetStaticCollider_base(self):
        await self.base_apply_command_test(
            lambda primPath: SetStaticColliderCommand.execute(primPath, "convexHull"),
            {"numBoxShapes": 1, "numStaticRigids": 1},
            {"numSphereShapes": 1, "numStaticRigids": 2},
            {"numCapsuleShapes": 1, "numStaticRigids": 3},
            lambda primPath: RemoveStaticColliderCommand.execute(primPath),
            {"numBoxShapes": 0, "numSphereShapes": 0, "numCapsuleShapes": 0, "numStaticRigids": 0}
        )

    async def test_physics_commands_SetStaticCollider_undoredo(self):
        await self.base_undoredo_command_test(
            lambda primPath: SetStaticColliderCommand.execute(primPath, "convexHull")
        )

    async def test_physics_userpath_set_static_collider(self):
        for approx in rigidbody.approximations:
            print(approx)
            await self.base_basic_userpath_command_test(
                lambda primPath: SetStaticColliderCommand.execute(primPath, approx),
                {"numBoxShapes": 1, "numStaticRigids": 1}
            )

    async def test_physics_collision_instancing(self):
        schema_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, "../../../../data/")))
        schema_folder = schema_folder.replace("\\", "/") + "/"

        # stage with scene graph instanced cubes with rotation
        # if parsed correctly they should not collide
        stage = Usd.Stage.Open(schema_folder + "colliders_colliding.usda")

        UsdUtils.StageCache.Get().Insert(stage)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        get_physx_simulation_interface().attach_stage(stage_id)

        cube1 = stage.GetPrimAtPath("/World/Cube1") 
        cube2 = stage.GetPrimAtPath("/World/Cube1_01")

        translate1 = cube1.GetAttribute("xformOp:translate").Get()
        translate2 = cube2.GetAttribute("xformOp:translate").Get()

        for _ in range(10):
            get_physx_simulation_interface().simulate(1.0/60.0, 0.0)
            get_physx_simulation_interface().fetch_results()

        new_translate1 = cube1.GetAttribute("xformOp:translate").Get()
        new_translate2 = cube2.GetAttribute("xformOp:translate").Get()

        self.assertTrue(Gf.IsClose(translate1, new_translate1, 1e-3))  # Should be close by
        self.assertTrue(Gf.IsClose(translate2, new_translate2, 1e-3))  # Should be close by

        get_physx_simulation_interface().detach_stage()

        UsdUtils.StageCache.Get().Erase(stage)
