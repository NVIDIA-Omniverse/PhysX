# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import omni.kit.test
import omni.kit.stage_templates
from omni.physx.scripts.physicsUtils import *
from omni.physx import get_physx_interface, get_physx_simulation_interface
from omni.physx.bindings._physx import SimulationEvent, ContactEventType
from pxr import UsdGeom, UsdUtils, UsdShade, Sdf, Gf, UsdPhysics, PhysxSchema, PhysicsSchemaTools
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
import unittest


class PhysxContactReportAPITestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    def clear_contact_data(self):
        self.num_contact_found = 0
        self.num_contact_lost = 0
        self.num_contact_data = 0
                
        self.contact_found = False
        self.contact_lost = False
        
        self.contact_data_reported = 0            


    def setup_scene(self, stage, scenario, restitution):
        self.clear_contact_data()
        
        cache = UsdUtils.StageCache.Get()
        self.stage_id = cache.GetId(stage).ToLongInt()
         
        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        # Physics scene
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(981.0)

        # Plane
        add_ground_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        # Sphere material
        materialPath = "/physicsMaterial"
        matShade = UsdShade.Material.Define(stage, materialPath)
        material = UsdPhysics.MaterialAPI.Apply(matShade.GetPrim())
        material.CreateStaticFrictionAttr().Set(0.5)
        material.CreateDynamicFrictionAttr().Set(0.5)
        material.CreateRestitutionAttr().Set(restitution)
        material.CreateDensityAttr().Set(1000.0)

        if scenario == "point_instancer":
            geomPointInstancerPath = "/pointinstancer"

            # Box instanced
            boxActorPath = geomPointInstancerPath + "/boxActor"
            size = 10.0
            cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
            cubePrim = stage.GetPrimAtPath(boxActorPath)
            cubeGeom.CreateSizeAttr(size)
            UsdPhysics.CollisionAPI.Apply(cubePrim)
            UsdPhysics.RigidBodyAPI.Apply(cubePrim)
            contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(cubePrim)
            contactReportAPI.CreateThresholdAttr().Set(3000)

            # Add material        
            add_physics_material_to_prim(stage, stage.GetPrimAtPath(Sdf.Path(boxActorPath)), Sdf.Path(materialPath))

            # indices
            meshIndices = [0]
            positions = [Gf.Vec3f(0.0, 0.0, 100.0)]
            orientations = [Gf.Quath(1.0, 0.0, 0.0, 0.0)]
            linearVelocities = [Gf.Vec3f(0.0)]
            angularVelocities = [Gf.Vec3f(0.0)]

            # Create point instancer
            shapeList = UsdGeom.PointInstancer.Define(stage, geomPointInstancerPath)
            meshList = shapeList.GetPrototypesRel()
            # add mesh reference to point instancer
            meshList.AddTarget(boxActorPath)

            shapeList.GetProtoIndicesAttr().Set(meshIndices)
            shapeList.GetPositionsAttr().Set(positions)
            shapeList.GetOrientationsAttr().Set(orientations)
            shapeList.GetVelocitiesAttr().Set(linearVelocities)
            shapeList.GetAngularVelocitiesAttr().Set(angularVelocities)       
        elif scenario == "simple_sphere":
            spherePath = "/sphereActor"
            radius = 10.0
            color = Gf.Vec3f(71.0 / 255.0, 165.0 / 255.0, 1.0)
            sphere = add_rigid_sphere(self.stage, spherePath, radius, Gf.Vec3f(0.0, 0.0, 100.0), Gf.Quatf(1.0), color, 1000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.0))

            # apply contact report        
            contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(sphere.GetPrim())
            contactReportAPI.CreateThresholdAttr().Set(3000)

            # Add material        
            add_physics_material_to_prim(stage, stage.GetPrimAtPath(Sdf.Path(spherePath)), Sdf.Path(materialPath))
        elif scenario == "convex_decomposition":
            # concave mesh for convex decomposition
            mesh_path = "/World/concaveMesh"
            concaveGeom = create_mesh_concave(self.stage, mesh_path, 10.0)
            concaveGeom.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 100.0))
            UsdPhysics.RigidBodyAPI.Apply(concaveGeom.GetPrim())
            UsdPhysics.CollisionAPI.Apply(concaveGeom.GetPrim())
            meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(concaveGeom.GetPrim())
            meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")

            # apply contact report        
            contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(concaveGeom.GetPrim())
            contactReportAPI.CreateThresholdAttr().Set(3000)

            # Add material        
            add_physics_material_to_prim(self.stage, concaveGeom.GetPrim(), Sdf.Path(materialPath))
        elif scenario == "sdf":
            # concave mesh for SDF
            mesh_path = "/World/sdfMesh"
            concaveGeom = create_mesh_concave(self.stage, mesh_path, 10.0)
            concaveGeom.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 100.0))
            UsdPhysics.RigidBodyAPI.Apply(concaveGeom.GetPrim())
            UsdPhysics.CollisionAPI.Apply(concaveGeom.GetPrim())
            meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(concaveGeom.GetPrim())
            PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(concaveGeom.GetPrim())
            meshCollisionAPI.CreateApproximationAttr().Set("sdf")

            # apply contact report        
            contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(concaveGeom.GetPrim())
            contactReportAPI.CreateThresholdAttr().Set(3000)
            
            # Add material
            add_physics_material_to_prim(self.stage, concaveGeom.GetPrim(), Sdf.Path(materialPath))

    def _on_simulation_event(self, event):                
        if event.type == int(SimulationEvent.CONTACT_FOUND):            
            self.contact_found = True
            self.num_contact_found = self.num_contact_found + 1            
            self.num_contact_data = self.num_contact_data + event.payload['numContactData']
            stageId = int(event.payload['stageId'][0])
            self.assertTrue(int(self.stage_id) == stageId)
        if event.type == int(SimulationEvent.CONTACT_LOST):            
            self.contact_lost = True
            self.num_contact_lost = self.num_contact_lost + 1
            stageId = int(event.payload['stageId'][0])
            self.assertTrue(int(self.stage_id) == stageId)
        if event.type == int(SimulationEvent.CONTACT_PERSISTS):            
            self.num_contact_data = self.num_contact_data + event.payload['numContactData']
        if event.type == int(SimulationEvent.CONTACT_DATA):
            self.contact_data_reported = self.contact_data_reported + 1
            self.faceIndex1 = event.payload['faceIndex1']
            self.material1 = PhysicsSchemaTools.decodeSdfPath(event.payload['material1'][0], event.payload['material1'][1])

    def _on_contact_report_event(self, contact_headers, contact_data):    
        for contact_header in contact_headers:
            if contact_header.type == ContactEventType.CONTACT_FOUND:
                self.contact_found = True
                self.num_contact_found = self.num_contact_found + 1
            elif contact_header.type == ContactEventType.CONTACT_LOST:
                self.contact_lost = True
                self.num_contact_lost = self.num_contact_lost + 1
            
            self.num_contact_data = self.num_contact_data + contact_header.num_contact_data
            stageId = contact_header.stage_id
            self.assertTrue(self.stage_id == stageId)
            
            contact_data_offset = contact_header.contact_data_offset
            num_contact_data = contact_header.num_contact_data
            
            self.contact_data_reported = self.contact_data_reported + len(contact_data)
            
            for index in range(contact_data_offset, contact_data_offset + num_contact_data, 1):
                self.faceIndex1 = contact_data[index].face_index1
                self.material1 = PhysicsSchemaTools.intToSdfPath(contact_data[index].material1)

    async def contact_report_found_lost(self):
        self.contact_found = False
        self.contact_lost = False

        self.stage = await self.new_stage()
        
        self.setup_scene(self.stage, "simple_sphere", 1.0)
        
        for _ in range(100):
            self.step()

        self.assertTrue(self.contact_found)
        self.assertTrue(self.contact_lost)
        print("Compare contact data reported: " + str(self.contact_data_reported) + " vs num: " + str(self.num_contact_data))
        self.assertTrue(self.contact_data_reported == self.num_contact_data)

    async def contact_report_found_lost_point_instancer(self):
        self.contact_found = False
        self.contact_lost = False

        self.stage = await self.new_stage()
        
        self.setup_scene(self.stage, "point_instancer", 1.0)
        
        for _ in range(100):
            self.step()

        self.assertTrue(self.contact_found)
        self.assertTrue(self.contact_lost)
        self.assertTrue(self.contact_data_reported == self.num_contact_data)
        
    async def contact_report_found_lost_convex_decomposition(self):        
        self.stage = await self.new_stage()
        
        self.setup_scene(self.stage, "convex_decomposition", 1.0)
        
        for _ in range(30):
            self.step()
                        
        self.assertTrue(self.contact_found)
        self.assertTrue(self.contact_lost)
        self.assertTrue(self.num_contact_data > 4)
        self.assertTrue(self.num_contact_found == 1)
        self.assertTrue(self.num_contact_lost == 1)
        self.assertTrue(self.contact_data_reported == self.num_contact_data)

    async def contact_report_found_lost_sdf(self):
        self.stage = await self.new_stage()
        
        self.setup_scene(self.stage, "sdf", 1.0)
        
        for _ in range(30):
            self.step()
            
        self.assertTrue(self.contact_found)
        self.assertTrue(self.contact_lost)
        self.assertTrue(self.num_contact_data == 4)
        self.assertTrue(self.num_contact_found == 1)
        self.assertTrue(self.num_contact_lost == 1)
        self.assertTrue(self.contact_data_reported == self.num_contact_data)

    async def contact_report_multi_material(self):
        self.clear_contact_data()

        self.faceIndex1 = 0
        self.material1 = Sdf.Path()

        stage = await self.new_stage()

        cache = UsdUtils.StageCache.Get()
        self.stage_id = cache.GetId(stage).ToLongInt()

        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        # Physics scene
        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")

        materialCount = 10        
    
        material_scope_path = defaultPrimPath + "/Looks"
        UsdGeom.Scope.Define(stage, material_scope_path)

        # Trianglemesh materials
        mtl_path_1 = Sdf.Path()
        for i in range(materialCount):
            mtl_path = material_scope_path + "/OmniPBR" + str(i)

            mu = 0.0 + (i % 10) * 0.1

            mat_prim = stage.DefinePrim(mtl_path, "Material")
            material_prim = UsdShade.Material.Get(stage, mat_prim.GetPath())
            material = UsdPhysics.MaterialAPI.Apply(material_prim.GetPrim())
            material.CreateRestitutionAttr().Set(mu)
            if i == 1:
                mtl_path_1 = mat_prim.GetPath()

        # Sphere        
        size = 25.0
        position = Gf.Vec3f(100, 0.0, 250.0)
        sphere_prim = add_rigid_sphere(stage, "/sphere", size, position)

        # apply contact report            
        contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(sphere_prim)
        contactReportAPI.CreateThresholdAttr().Set(200000)

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
            rel = subset.GetPrim().CreateRelationship("material:binding:physics", False)
            rel.SetTargets([Sdf.Path(material_scope_path + "/OmniPBR" + str(i))])

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
        meshCollisionAPI.CreateApproximationAttr().Set("none")

        for _ in range(100):
            self.step()

        self.assertTrue(self.contact_found)
        self.assertTrue(self.faceIndex1 == 1)
        self.assertTrue(self.material1 == mtl_path_1)
        self.assertTrue(self.contact_data_reported == self.num_contact_data)

    async def lost_contacts_prim_delete_static(self):

        self.stage = await self.new_stage()
        
        self.setup_scene(self.stage, "simple_sphere", 0.0)
        
        for _ in range(200):
            self.step()

        self.assertTrue(self.contact_found)        

        self.contact_found = False
        self.contact_lost = False

        defaultPrimPath = str(self.stage.GetDefaultPrim().GetPath())
        self.stage.RemovePrim(defaultPrimPath + "/groundPlane")        

        for _ in range(5):
            self.step()

        self.assertTrue(self.contact_found == False)
        self.assertTrue(self.contact_lost)

    async def lost_contacts_prim_delete_dynamic(self):
        self.stage = await self.new_stage()
        
        self.setup_scene(self.stage, "simple_sphere", 0.0)
        
        for _ in range(200):
            self.step()

        self.assertTrue(self.contact_found)        

        self.contact_found = False
        self.contact_lost = False

        defaultPrimPath = str(self.stage.GetDefaultPrim().GetPath())
        self.stage.RemovePrim(defaultPrimPath + "/sphereActor")        

        for _ in range(5):
            self.step()

        self.assertTrue(self.contact_found == False)
        self.assertTrue(self.contact_lost)

    async def lost_contacts_prim_change_dynamic_to_static(self):
        self.stage = await self.new_stage()
        
        self.setup_scene(self.stage, "simple_sphere", 0.0)
        
        for _ in range(200):
            self.step()

        self.assertTrue(self.contact_found)        

        self.contact_found = False
        self.contact_lost = False

        defaultPrimPath = str(self.stage.GetDefaultPrim().GetPath())
        rbo = UsdPhysics.RigidBodyAPI.Get(self.stage, defaultPrimPath + "/sphereActor")
        rbo.GetRigidBodyEnabledAttr().Set(False)

        for _ in range(5):
            self.step()

        self.assertTrue(self.contact_found == False)
        self.assertTrue(self.contact_lost)

    async def lost_contacts_prim_delete_static_convex_decomposition(self):
        self.stage = await self.new_stage()
        
        self.setup_scene(self.stage, "convex_decomposition", 0.0)
        
        for _ in range(200):
            self.step()

        self.assertTrue(self.contact_found)        

        self.contact_found = False
        self.contact_lost = False
        self.num_contact_found = 0
        self.num_contact_lost = 0

        defaultPrimPath = str(self.stage.GetDefaultPrim().GetPath())
        self.stage.RemovePrim(defaultPrimPath + "/groundPlane")        

        for _ in range(5):
            self.step()

        self.assertTrue(self.contact_found == False)
        self.assertTrue(self.contact_lost)
        self.assertTrue(self.num_contact_lost == 1)

    async def lost_contacts_prim_delete_dynamic_convex_decomposition(self):
        self.stage = await self.new_stage()
        
        self.setup_scene(self.stage, "convex_decomposition", 0.0)
        
        for _ in range(200):
            self.step()

        self.assertTrue(self.contact_found)        

        self.contact_found = False
        self.contact_lost = False
        self.num_contact_found = 0
        self.num_contact_lost = 0
        
        self.stage.RemovePrim("/World/concaveMesh")        

        for _ in range(5):
            self.step()

        self.assertTrue(self.contact_found == False)
        self.assertTrue(self.contact_lost)
        self.assertTrue(self.num_contact_lost == 1)
        
    async def lost_contacts_prim_delete_static_sdf(self):
        self.stage = await self.new_stage()
        
        self.setup_scene(self.stage, "sdf", 0.0)
        
        for _ in range(200):
            self.step()

        self.assertTrue(self.contact_found)

        self.contact_found = False
        self.contact_lost = False
        self.num_contact_found = 0
        self.num_contact_lost = 0

        defaultPrimPath = str(self.stage.GetDefaultPrim().GetPath())
        self.stage.RemovePrim(defaultPrimPath + "/groundPlane")

        for _ in range(5):
            self.step()

        self.assertTrue(self.contact_found == False)
        self.assertTrue(self.contact_lost)
        self.assertTrue(self.num_contact_lost == 1)

    async def lost_contacts_prim_delete_dynamic_sdf(self):
        self.stage = await self.new_stage()
        
        self.setup_scene(self.stage, "sdf", 0.0)
        
        for _ in range(200):
            self.step()

        self.assertTrue(self.contact_found)

        self.contact_found = False
        self.contact_lost = False
        self.num_contact_found = 0
        self.num_contact_lost = 0
        
        self.stage.RemovePrim("/World/sdfMesh")

        for _ in range(5):
            self.step()

        self.assertTrue(self.contact_found == False)
        self.assertTrue(self.contact_lost)
        self.assertTrue(self.num_contact_lost == 1)
        
    async def contact_kinekine_report_found_lost(self):                
        self.clear_contact_data()
        
        stage = await self.new_stage()

        cache = UsdUtils.StageCache.Get()
        self.stage_id = cache.GetId(stage).ToLongInt()

        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        # Physics scene
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(981.0)
        sceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        sceneAPI.CreateReportKinematicKinematicPairsAttr(False)

        spherePath0 = "/sphereActor0"
        radius = 50.0
        sphere0 = add_rigid_sphere(stage, spherePath0, radius, Gf.Vec3f(0.0), Gf.Quatf(1.0))
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(sphere0.GetPrim())
        rigidBodyAPI.CreateKinematicEnabledAttr(True)

        # apply contact report        
        contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(sphere0.GetPrim())
        contactReportAPI.CreateThresholdAttr().Set(3000)

        spherePath1 = "/sphereActor1"
        sphere1 = add_rigid_sphere(stage, spherePath1, radius, Gf.Vec3f(0.0, 0.0, 200.0), Gf.Quatf(1.0))
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(sphere1.GetPrim())
        rigidBodyAPI.CreateKinematicEnabledAttr(True)

        cache = UsdUtils.StageCache.Get()
        stage_id = cache.GetId(stage).ToLongInt()

        get_physx_simulation_interface().detach_stage()

        # test with kine kine disabled - no report should be send
        get_physx_simulation_interface().attach_stage(stage_id)
        
        self.step()

        self.assertTrue(not self.contact_found)
        self.assertTrue(not self.contact_lost)        

        sphere0.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 0.0, 180.0))
        self.contact_found = False
        self.contact_lost = False

        for _ in range(5):
            self.step()

        self.assertTrue(not self.contact_found)
        self.assertTrue(not self.contact_lost)        

        get_physx_simulation_interface().detach_stage()

        sphere0.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 0.0, 0.0))        

        # enable kine kine
        sceneAPI.CreateReportKinematicKinematicPairsAttr(True)

        get_physx_simulation_interface().attach_stage(stage_id)

        self.step()

        self.assertTrue(not self.contact_found)
        self.assertTrue(not self.contact_lost)        

        sphere0.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 0.0, 180.0))
        self.contact_found = False
        self.contact_lost = False

        for _ in range(5):
            self.step()

        self.assertTrue(self.contact_found)
        self.assertTrue(not self.contact_lost)        

        sphere0.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 0.0, 0.0))
        self.contact_found = False
        self.contact_lost = False

        for _ in range(5):
            self.step()

        self.assertTrue(not self.contact_found)
        self.assertTrue(self.contact_lost)   

    async def contact_kinestatic_report_found_lost(self):
        self.clear_contact_data()

        stage = await self.new_stage()

        cache = UsdUtils.StageCache.Get()
        self.stage_id = cache.GetId(stage).ToLongInt()

        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        # Physics scene
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(981.0)
        sceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        sceneAPI.CreateReportKinematicStaticPairsAttr(False)

        spherePath0 = "/sphereActor0"
        radius = 50.0
        sphere0 = add_rigid_sphere(stage, spherePath0, radius, Gf.Vec3f(0.0), Gf.Quatf(1.0))
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(sphere0.GetPrim())
        rigidBodyAPI.CreateKinematicEnabledAttr(True)

        # apply contact report        
        contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(sphere0.GetPrim())
        contactReportAPI.CreateThresholdAttr().Set(3000)

        spherePath1 = "/sphereActor1"
        sphere1 = add_rigid_sphere(stage, spherePath1, radius, Gf.Vec3f(0.0, 0.0, 200.0), Gf.Quatf(1.0))
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(sphere1.GetPrim())
        rigidBodyAPI.CreateRigidBodyEnabledAttr(False)

        cache = UsdUtils.StageCache.Get()
        stage_id = cache.GetId(stage).ToLongInt()

        get_physx_simulation_interface().detach_stage()

        # test with kine static disabled - no report should be send
        get_physx_simulation_interface().attach_stage(stage_id)
        
        self.step()

        self.assertTrue(not self.contact_found)
        self.assertTrue(not self.contact_lost)        

        sphere0.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 0.0, 180.0))
        self.contact_found = False
        self.contact_lost = False

        for _ in range(5):
            self.step()

        self.assertTrue(not self.contact_found)
        self.assertTrue(not self.contact_lost)        

        get_physx_simulation_interface().detach_stage()

        sphere0.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 0.0, 0.0))        

        # enable kine static
        sceneAPI.CreateReportKinematicStaticPairsAttr(True)

        get_physx_simulation_interface().attach_stage(stage_id)

        self.step()

        self.assertTrue(not self.contact_found)
        self.assertTrue(not self.contact_lost)        

        sphere0.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 0.0, 180.0))
        self.contact_found = False
        self.contact_lost = False

        for _ in range(5):
            self.step()

        self.assertTrue(self.contact_found)
        self.assertTrue(not self.contact_lost)        

        sphere0.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 0.0, 0.0))
        self.contact_found = False
        self.contact_lost = False

        for _ in range(5):
            self.step()

        self.assertTrue(not self.contact_found)
        self.assertTrue(self.contact_lost)   
                
    async def test_physics_contact_report_found_lost_batched(self):        
        simulation_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)

        await self.contact_report_found_lost()

        simulation_event_sub = None
                
    async def test_physics_contact_report_found_lost_point_instancer_batched(self):
        simulation_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)

        await self.contact_report_found_lost_point_instancer()

        simulation_event_sub = None
                
    async def test_physics_contact_report_found_lost_convex_decomposition_batched(self):
        simulation_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)

        await self.contact_report_found_lost_convex_decomposition()
        
        simulation_event_sub = None
        
    async def test_physics_contact_report_found_lost_sdf_batched(self):
        simulation_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)

        await self.contact_report_found_lost_sdf()
        
        simulation_event_sub = None

    async def test_physics_contact_report_multi_material_batched(self):
        simulation_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)

        await self.contact_report_multi_material()

        simulation_event_sub = None
        
    async def test_physics_lost_contacts_prim_delete_static_batched(self):
        simulation_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)

        await self.lost_contacts_prim_delete_static()

        simulation_event_sub = None
        
    async def test_physics_lost_contacts_prim_delete_dynamic_batched(self):
        simulation_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)
        
        await self.lost_contacts_prim_delete_dynamic()

        simulation_event_sub = None
        
    async def test_physics_lost_contacts_prim_change_dynamic_to_static(self):
        simulation_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)
        
        await self.lost_contacts_prim_change_dynamic_to_static()

        simulation_event_sub = None        

    async def test_physics_lost_contacts_prim_delete_static_convex_decomposition_batched(self):
        simulation_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)

        await self.lost_contacts_prim_delete_static_convex_decomposition()

        simulation_event_sub = None

    async def test_physics_lost_contacts_prim_delete_dynamic_convex_decomposition_batched(self):
        simulation_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)

        await self.lost_contacts_prim_delete_dynamic_convex_decomposition()

        simulation_event_sub = None

    async def test_physics_lost_contacts_prim_delete_static_sdf_batched(self):
        simulation_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)

        await self.lost_contacts_prim_delete_static_sdf()

        simulation_event_sub = None

    async def test_physics_lost_contacts_prim_delete_dynamic_sdf_batched(self):
        simulation_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)

        await self.lost_contacts_prim_delete_dynamic_sdf()

        simulation_event_sub = None        
                
    async def test_physics_contact_kinekine_report_found_lost_batched(self):                
        simulation_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)

        await self.contact_kinekine_report_found_lost()

        simulation_event_sub = None

    async def test_physics_contact_kinestatic_report_found_lost_batched(self):
        simulation_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)

        await self.contact_kinestatic_report_found_lost()

        simulation_event_sub = None

    async def test_physics_contact_report_found_lost_immediate_api(self):
        self.stage = await self.new_stage()
        
        self.setup_scene(self.stage, "simple_sphere", 1.0)
        
        for _ in range(50):
            self.step()
            contact_headers, contact_data = get_physx_simulation_interface().get_contact_report()
            if len(contact_headers) == 1:
                if contact_headers[0].type == ContactEventType.CONTACT_FOUND:
                    self.contact_found = True
                if contact_headers[0].type == ContactEventType.CONTACT_LOST:
                    self.contact_lost = True
                    
                self.contact_data_reported = self.contact_data_reported + contact_headers[0].num_contact_data
                self.num_contact_data = self.num_contact_data + len(contact_data)

        self.assertTrue(self.contact_found)
        self.assertTrue(self.contact_lost)
        print("Compare contact data reported: " + str(self.contact_data_reported) + " vs num: " + str(self.num_contact_data))
        self.assertTrue(self.contact_data_reported == self.num_contact_data)

    async def test_physics_contact_report_disable_setting(self):
        self.stage = await self.new_stage()
        
        DISABLE_CONTACT_PROCESSING = omni.physx.bindings._physx.SETTING_DISABLE_CONTACT_PROCESSING

        settingsInterface = carb.settings.get_settings()
        disable_contact_proc = carb.settings.get_settings().get_as_bool(DISABLE_CONTACT_PROCESSING)
        settingsInterface.set_bool(DISABLE_CONTACT_PROCESSING, True)

        self.setup_scene(self.stage, "simple_sphere", 1.0)
        
        for _ in range(50):
            self.step()
            contact_headers, contact_data = get_physx_simulation_interface().get_contact_report()
            if len(contact_headers) == 1:
                if contact_headers[0].type == ContactEventType.CONTACT_FOUND:
                    self.contact_found = True
                if contact_headers[0].type == ContactEventType.CONTACT_LOST:
                    self.contact_lost = True
                    
                self.contact_data_reported = self.contact_data_reported + contact_headers[0].num_contact_data
                self.num_contact_data = self.num_contact_data + len(contact_data)

        self.assertTrue(not self.contact_found)
        self.assertTrue(not self.contact_lost)
        
        settingsInterface.set_bool(DISABLE_CONTACT_PROCESSING, disable_contact_proc)

    async def test_physics_contact_report_multiple_scenes(self):
        self.stage = await self.new_stage()
        
        self.setup_scene(self.stage, "simple_sphere", 1.0)
        
        UsdPhysics.Scene.Define(self.stage, Sdf.Path("/physicsScene1"))
        
        sphere_rigid_body_api = UsdPhysics.RigidBodyAPI.Get(self.stage, Sdf.Path("/sphereActor"))        
        ground_collision_api = UsdPhysics.CollisionAPI.Get(self.stage, Sdf.Path("/groundPlane/CollisionPlane"))
        
        sphere_rigid_body_api.GetSimulationOwnerRel().SetTargets([ Sdf.Path("/physicsScene1") , Sdf.Path("/physicsScene")])
        ground_collision_api.GetSimulationOwnerRel().SetTargets([ Sdf.Path("/physicsScene") , Sdf.Path("/physicsScene1")])
        
        for _ in range(50):
            self.step()
            contact_headers, contact_data = get_physx_simulation_interface().get_contact_report()
            if len(contact_headers) == 1:
                if contact_headers[0].type == ContactEventType.CONTACT_FOUND:
                    self.contact_found = True
                if contact_headers[0].type == ContactEventType.CONTACT_LOST:
                    self.contact_lost = True
                    
                self.contact_data_reported = self.contact_data_reported + contact_headers[0].num_contact_data
                self.num_contact_data = self.num_contact_data + len(contact_data)

        self.assertTrue(not self.contact_found)
        self.assertTrue(not self.contact_lost)
        
    async def test_physics_contact_report_point_instancer_immediate_api(self):
        self.stage = await self.new_stage()
        
        self.setup_scene(self.stage, "point_instancer", 1.0)
        
        for _ in range(50):
            self.step()
            contact_headers, contact_data = get_physx_simulation_interface().get_contact_report()
            if len(contact_headers) == 1:
                if contact_headers[0].type == ContactEventType.CONTACT_FOUND:
                    self.contact_found = True
                if contact_headers[0].type == ContactEventType.CONTACT_LOST:
                    self.contact_lost = True
                    
                self.assertTrue(contact_headers[0].proto_index0 == 0)
                self.contact_data_reported = self.contact_data_reported + contact_headers[0].num_contact_data
                self.num_contact_data = self.num_contact_data + len(contact_data)

        self.assertTrue(self.contact_found)
        self.assertTrue(self.contact_lost)
        print("Compare contact data reported: " + str(self.contact_data_reported) + " vs num: " + str(self.num_contact_data))
        self.assertTrue(self.contact_data_reported == self.num_contact_data)

    async def test_contact_report_found_lost_runtime_add_remove(self):
        self.contact_found = False
        self.contact_lost = False

        simulation_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)

        self.stage = await self.new_stage()
        
        self.setup_scene(self.stage, "simple_sphere", 1.0)

        defaultPrimPath = str(self.stage.GetDefaultPrim().GetPath())
        sphere_prim = self.stage.GetPrimAtPath(defaultPrimPath + "/sphereActor")
        sphere_prim.RemoveAPI(PhysxSchema.PhysxContactReportAPI)

        pos_attr = sphere_prim.GetAttribute("xformOp:translate")
        init_pos = pos_attr.Get()
        
        for _ in range(50):
            self.step()

        self.assertFalse(self.contact_found)
        self.assertFalse(self.contact_lost)

        PhysxSchema.PhysxContactReportAPI.Apply(sphere_prim)
        
        pos_attr.Set(init_pos)

        self.contact_found = False
        self.contact_lost = False
        
        for _ in range(50):
            self.step()

        self.assertTrue(self.contact_found)
        self.assertTrue(self.contact_lost)

        sphere_prim.RemoveAPI(PhysxSchema.PhysxContactReportAPI)
        
        pos_attr.Set(init_pos)

        self.contact_found = False
        self.contact_lost = False
        
        for _ in range(50):
            self.step()

        self.assertFalse(self.contact_found)
        self.assertFalse(self.contact_lost)

        self.assertTrue(self.contact_data_reported == self.num_contact_data)

        simulation_event_sub = None
