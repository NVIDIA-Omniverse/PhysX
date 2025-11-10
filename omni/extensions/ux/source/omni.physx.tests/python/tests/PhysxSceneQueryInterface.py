# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import omni.physx.scripts.utils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physx import get_physx_scene_query_interface
from omni.physxtests import utils
from pxr import Usd, Gf, Sdf, UsdGeom, UsdShade, UsdPhysics, UsdUtils, PhysxSchema, PhysicsSchemaTools
import math

class PhysxSceneQueryInterfaceTestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    def setup_stage(self, stage, scenario = "simple"):
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        self.defaultPrimPath = str(stage.GetDefaultPrim().GetPath())
        
        UsdPhysics.Scene.Define(stage, self.defaultPrimPath + "/physicsScene")

        # Sphere material
        materialPath = "/physicsMaterial"
        matShade = UsdShade.Material.Define(stage, materialPath)
        UsdPhysics.MaterialAPI.Apply(matShade.GetPrim())
        
        if scenario == "simple":
            boxActorPath = "/boxActor0"
            size = Gf.Vec3f(100.0)
            position = Gf.Vec3f(0.0)
            orientation = Gf.Quatf(1.0)
            physicsUtils.add_rigid_box(stage, boxActorPath, size, position, orientation)

            boxActorPath = "/boxActor1"
            size = Gf.Vec3f(100.0)
            position = Gf.Vec3f(0.0, -200.0, 0.0)
            orientation = Gf.Quatf(1.0)
            physicsUtils.add_rigid_box(stage, boxActorPath, size, position, orientation)

        elif scenario == "mesh_cleanup":
            halfSize = 50
            points = [
                Gf.Vec3f(-halfSize, -halfSize, 0.0),
                Gf.Vec3f(halfSize, -halfSize, 0.0),
                Gf.Vec3f(halfSize, halfSize, 0.0),
                Gf.Vec3f(-halfSize, halfSize, 0.0),
            ]
            indices = [0, 1, 2, 1, 1 , 1, 2 , 2, 2, 0, 2, 3]
            normals = []
            vertexCounts = [3,3,3,3]

            # Create the mesh
            mesh = physicsUtils.create_mesh(stage, "/meshActor", points, normals, indices, vertexCounts)
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())

        elif scenario == "point_instancer":
            geomPointInstancerPath = "/pointinstancer"

            # Box instanced
            boxActorPath = geomPointInstancerPath + "/boxActor"
            size = 100.0
            cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
            cubePrim = stage.GetPrimAtPath(boxActorPath)
            cubeGeom.CreateSizeAttr(size)
            UsdPhysics.CollisionAPI.Apply(cubePrim)
            UsdPhysics.RigidBodyAPI.Apply(cubePrim)

            # Add material        
            physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath(Sdf.Path(boxActorPath)), Sdf.Path(materialPath))

            # indices
            meshIndices = [0]
            positions = [Gf.Vec3f(0.0)]
            orientations = [Gf.Quath(1.0)]
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
            
        self.step()

    def setup_stage_multi_material(self, stage, default_material):
        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        self.defaultPrimPath = str(stage.GetDefaultPrim().GetPath())
        
        UsdPhysics.Scene.Define(stage, self.defaultPrimPath + "/physicsScene")

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        # materials
        materialPath0 = defaultPrimPath + "/material0"
        matShade = UsdShade.Material.Define(stage, materialPath0)
        UsdPhysics.MaterialAPI.Apply(matShade.GetPrim())

        materialPath1 = defaultPrimPath + "/material1"
        matShade = UsdShade.Material.Define(stage, materialPath1)
        UsdPhysics.MaterialAPI.Apply(matShade.GetPrim())

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

        for i in range(5):
            if i == 2:
                subset = UsdGeom.Subset.Define(stage, path + "/subset" + str(i))
                subset.CreateElementTypeAttr().Set("face")
                subset_indices = [i]
                subset.CreateIndicesAttr().Set(subset_indices)
                rel = subset.GetPrim().CreateRelationship("material:binding:physics", False)
                rel.SetTargets([Sdf.Path(materialPath0)])
            else:
                subset = UsdGeom.Subset.Define(stage, path + "/subset" + str(i))
                subset.CreateElementTypeAttr().Set("face")
                subset_indices = [i]
                subset.CreateIndicesAttr().Set(subset_indices)

            points.append(Gf.Vec3f(-stripSize/2.0 + stripSize * i, -halfSize, 0.0))
            points.append(Gf.Vec3f(-stripSize/2.0 + stripSize * (i + 1), -halfSize, 0.0))
            points.append(Gf.Vec3f(-stripSize/2.0 + stripSize * (i + 1), halfSize, 0.0))
            points.append(Gf.Vec3f(-stripSize/2.0 + stripSize * i, halfSize, 0.0))
            
            for j in range(4):
                normals.append(Gf.Vec3f(0, 0, 1))
                indices.append(j + i * 4)                            
            vertexCounts.append(4)

        mesh.CreateFaceVertexCountsAttr().Set(vertexCounts)
        mesh.CreateFaceVertexIndicesAttr().Set(indices)
        mesh.CreatePointsAttr().Set(points)
        mesh.CreateDoubleSidedAttr().Set(False)
        mesh.CreateNormalsAttr().Set(normals)
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
        meshCollisionAPI.CreateApproximationAttr().Set("none")
        if not default_material:
            rel = mesh.GetPrim().CreateRelationship("material:binding:physics", False)
            rel.SetTargets([Sdf.Path(materialPath1)])

        self.step()


    def create_cube_mesh(self, stage, size, position=Gf.Vec3d(0.0, 0.0, 0.0), orientation=Gf.Quatf(1.0)):
        prim_path = self.defaultPrimPath + "/mesh"
        cubeMesh = UsdGeom.Mesh.Define(stage, prim_path)

        faceVertexCounts = [4, 4, 4, 4, 4, 4]
        faceVertexIndices = [0, 1, 3, 2, 4, 5, 7, 6, 10, 11, 13, 12, 14, 15, 9, 8, 17, 23, 21, 19, 22, 16, 18, 20]
        points = [
            Gf.Vec3f(-1, 1, -1), Gf.Vec3f(1, 1, -1), Gf.Vec3f(-1, 1, 1), Gf.Vec3f(1, 1, 1),
            Gf.Vec3f(-1, -1, 1), Gf.Vec3f(1, -1, 1), Gf.Vec3f(-1, -1, -1), Gf.Vec3f(1, -1, -1),
            Gf.Vec3f(-1, 1, -1), Gf.Vec3f(1, 1, -1), Gf.Vec3f(-1, 1, 1), Gf.Vec3f(1, 1, 1),
            Gf.Vec3f(-1, -1, 1), Gf.Vec3f(1, -1, 1), Gf.Vec3f(-1, -1, -1), Gf.Vec3f(1, -1, -1),
            Gf.Vec3f(-1, 1, -1), Gf.Vec3f(1, 1, -1), Gf.Vec3f(-1, 1, 1), Gf.Vec3f(1, 1, 1),
            Gf.Vec3f(-1, -1, 1), Gf.Vec3f(1, -1, 1), Gf.Vec3f(-1, -1, -1), Gf.Vec3f(1, -1, -1),
        ]

        cubeMesh.CreateFaceVertexCountsAttr(faceVertexCounts)
        cubeMesh.CreateFaceVertexIndicesAttr(faceVertexIndices)
        cubeMesh.CreatePointsAttr(points)

        cubeMesh.AddTranslateOp().Set(position)
        cubeMesh.AddOrientOp().Set(orientation)
        cubeMesh.AddScaleOp().Set(Gf.Vec3f(size))

        return cubeMesh

    geoms_list = ["sphere", "cube", "capsule", "cylinder", "cone", "mesh"]

    def create_geometry(self, stage, geometry_type, size=100.0, position=Gf.Vec3d(0.0, 0.0, 0.0), orientation=Gf.Quatf(1.0)):
        geom_path = Sdf.Path(self.defaultPrimPath + '/' + geometry_type)

        if geometry_type == "mesh":
            shape = self.create_cube_mesh(stage, size, position, orientation)
            return shape
        elif geometry_type == "sphere":
            shape = UsdGeom.Sphere.Define(stage, geom_path)
            shape.CreateRadiusAttr().Set(size)
        elif geometry_type == "cube":
            shape = UsdGeom.Cube.Define(stage, geom_path)
            shape.CreateSizeAttr().Set(size)
        elif geometry_type == "capsule":
            shape = UsdGeom.Capsule.Define(stage, geom_path)
            shape.CreateRadiusAttr().Set(size * 0.5)
            shape.CreateHeightAttr().Set(size)
            shape.CreateAxisAttr("Y")
        elif geometry_type == "cylinder":
            shape = UsdGeom.Cylinder.Define(stage, geom_path)
            shape.CreateRadiusAttr().Set(size * 0.5)
            shape.CreateHeightAttr().Set(size)
            shape.CreateAxisAttr("Y")
        elif geometry_type == "cone":
            shape = UsdGeom.Cone.Define(stage, geom_path)
            shape.CreateRadiusAttr().Set(size * 0.5)
            shape.CreateHeightAttr().Set(size)
            shape.CreateAxisAttr("Y")

        shape.AddTranslateOp().Set(position)
        shape.AddOrientOp().Set(orientation)
        shape.AddScaleOp().Set(Gf.Vec3f(1.0))
        return shape

    async def test_physics_raycast_closest(self):
        stage = await self.new_stage()

        self.setup_stage(stage)
        
        tolerance = 0.001

        hitInfo = get_physx_scene_query_interface().raycast_closest(Gf.Vec3f(0.0, 200.0, 0.0), Gf.Vec3f(0.0, -1.0, 0.0), 1000.0)
        if hitInfo["hit"]:
            hitPos = Gf.Vec3f(hitInfo["position"].x, hitInfo["position"].y, hitInfo["position"].z)
            hitNormal = Gf.Vec3f(hitInfo["normal"].x, hitInfo["normal"].y, hitInfo["normal"].z)
            hitCollision = hitInfo["collision"]
            hitRigidBody = hitInfo["rigidBody"]
            self.assertTrue(Gf.IsClose(hitPos, Gf.Vec3f(0.0, 50.0, 0.0), tolerance))
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 1.0, 0.0), tolerance))
            self.assertTrue(hitCollision == self.defaultPrimPath + "/boxActor0")
            self.assertTrue(hitRigidBody == self.defaultPrimPath + "/boxActor0")
            self.assertTrue(hitInfo["protoIndex"] == 0xFFFFFFFF)
        else:
            print("Hit not found.")
            self.assertTrue(False)

    async def test_physics_raycast_closest_point_instancer(self):
        stage = await self.new_stage()

        self.setup_stage(stage, "point_instancer")
        
        tolerance = 0.001

        hitInfo = get_physx_scene_query_interface().raycast_closest(Gf.Vec3f(0.0, 200.0, 0.0), Gf.Vec3f(0.0, -1.0, 0.0), 1000.0)
        if hitInfo["hit"]:
            hitPos = Gf.Vec3f(hitInfo["position"].x, hitInfo["position"].y, hitInfo["position"].z)
            hitNormal = Gf.Vec3f(hitInfo["normal"].x, hitInfo["normal"].y, hitInfo["normal"].z)
            hitCollision = hitInfo["collision"]
            hitRigidBody = hitInfo["rigidBody"]
            self.assertTrue(Gf.IsClose(hitPos, Gf.Vec3f(0.0, 50.0, 0.0), tolerance))
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 1.0, 0.0), tolerance))
            self.assertTrue(hitCollision == "/pointinstancer/boxActor")
            self.assertTrue(hitRigidBody == "/pointinstancer/boxActor")
            self.assertTrue(hitInfo["protoIndex"] == 0)
        else:
            print("Hit not found.")
            self.assertTrue(False)
            
    async def test_physics_raycast_any(self):
        stage = await self.new_stage()

        self.setup_stage(stage)
        
        tolerance = 0.001

        hitInfo = get_physx_scene_query_interface().raycast_any(Gf.Vec3f(0.0, 200.0, 0.0), Gf.Vec3f(0.0, -1.0, 0.0), 1000.0)
        if hitInfo:
            print("Hit found.")
        else:
            print("Hit not found.")
            self.assertTrue(False)

    async def test_physics_raycast_all(self):
        stage = await self.new_stage()

        self.setup_stage(stage)
        
        tolerance = 0.001

        self.numHits = 0

        def report_all_hits(hit):
            hitNormal = Gf.Vec3f(hit.normal.x, hit.normal.y, hit.normal.z)
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 1.0, 0.0), tolerance))
            self.numHits = self.numHits + 1
            return True

        numHits = get_physx_scene_query_interface().raycast_all(Gf.Vec3f(0.0, 200.0, 0.0), Gf.Vec3f(0.0, -1.0, 0.0), 1000.0, report_all_hits)
        if numHits and self.numHits == 2:
            print("Hits found.")
        else:
            print("Hits not found. Num hits: " + str(self.numHits))
            self.assertTrue(False)   

    async def test_physics_raycast_closest_multi_material(self):
        stage = await self.new_stage()

        self.setup_stage_multi_material(stage, False)

        tolerance = 0.001

        hitInfo = get_physx_scene_query_interface().raycast_closest(Gf.Vec3f(200.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0)
        if hitInfo["hit"]:
            hitPos = Gf.Vec3f(hitInfo["position"].x, hitInfo["position"].y, hitInfo["position"].z)
            hitNormal = Gf.Vec3f(hitInfo["normal"].x, hitInfo["normal"].y, hitInfo["normal"].z)
            hitCollision = hitInfo["collision"]
            hitRigidBody = hitInfo["rigidBody"]
            hitMaterial = hitInfo["material"]
            self.assertTrue(Gf.IsClose(hitPos, Gf.Vec3f(200.0, 0.0, 0.0), tolerance))
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 0.0, 1.0), tolerance))
            self.assertTrue(hitInfo["faceIndex"] == 2)
            self.assertTrue(hitCollision == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitRigidBody == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitMaterial == self.defaultPrimPath + "/material0")
        else:
            print("Hit not found.")
            self.assertTrue(False)

        hitInfo = get_physx_scene_query_interface().raycast_closest(Gf.Vec3f(100.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0)
        if hitInfo["hit"]:
            hitPos = Gf.Vec3f(hitInfo["position"].x, hitInfo["position"].y, hitInfo["position"].z)
            hitNormal = Gf.Vec3f(hitInfo["normal"].x, hitInfo["normal"].y, hitInfo["normal"].z)
            hitCollision = hitInfo["collision"]
            hitRigidBody = hitInfo["rigidBody"]
            hitMaterial = hitInfo["material"]
            self.assertTrue(Gf.IsClose(hitPos, Gf.Vec3f(100.0, 0.0, 0.0), tolerance))
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 0.0, 1.0), tolerance))
            self.assertTrue(hitInfo["faceIndex"] == 1)
            self.assertTrue(hitCollision == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitRigidBody == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitMaterial == self.defaultPrimPath + "/material1")
        else:
            print("Hit not found.")
            self.assertTrue(False)

        hitInfo = get_physx_scene_query_interface().raycast_closest(Gf.Vec3f(300.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0)
        if hitInfo["hit"]:
            hitPos = Gf.Vec3f(hitInfo["position"].x, hitInfo["position"].y, hitInfo["position"].z)
            hitNormal = Gf.Vec3f(hitInfo["normal"].x, hitInfo["normal"].y, hitInfo["normal"].z)
            hitCollision = hitInfo["collision"]
            hitRigidBody = hitInfo["rigidBody"]
            hitMaterial = hitInfo["material"]
            self.assertTrue(Gf.IsClose(hitPos, Gf.Vec3f(300.0, 0.0, 0.0), tolerance))
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 0.0, 1.0), tolerance))
            self.assertTrue(hitInfo["faceIndex"] == 3)
            self.assertTrue(hitCollision == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitRigidBody == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitMaterial == self.defaultPrimPath + "/material1")
        else:
            print("Hit not found.")
            self.assertTrue(False)

    async def test_physics_raycast_closest_multi_material_default(self):
        stage = await self.new_stage()

        self.setup_stage_multi_material(stage, True)

        tolerance = 0.001

        hitInfo = get_physx_scene_query_interface().raycast_closest(Gf.Vec3f(200.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0)
        if hitInfo["hit"]:
            hitPos = Gf.Vec3f(hitInfo["position"].x, hitInfo["position"].y, hitInfo["position"].z)
            hitNormal = Gf.Vec3f(hitInfo["normal"].x, hitInfo["normal"].y, hitInfo["normal"].z)
            hitCollision = hitInfo["collision"]
            hitRigidBody = hitInfo["rigidBody"]
            hitMaterial = hitInfo["material"]
            self.assertTrue(Gf.IsClose(hitPos, Gf.Vec3f(200.0, 0.0, 0.0), tolerance))
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 0.0, 1.0), tolerance))
            self.assertTrue(hitInfo["faceIndex"] == 2)
            self.assertTrue(hitCollision == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitRigidBody == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitMaterial == self.defaultPrimPath + "/material0")
        else:
            print("Hit not found.")
            self.assertTrue(False)

        hitInfo = get_physx_scene_query_interface().raycast_closest(Gf.Vec3f(100.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0)
        if hitInfo["hit"]:
            hitPos = Gf.Vec3f(hitInfo["position"].x, hitInfo["position"].y, hitInfo["position"].z)
            hitNormal = Gf.Vec3f(hitInfo["normal"].x, hitInfo["normal"].y, hitInfo["normal"].z)
            hitCollision = hitInfo["collision"]
            hitRigidBody = hitInfo["rigidBody"]
            hitMaterial = hitInfo["material"]
            self.assertTrue(Gf.IsClose(hitPos, Gf.Vec3f(100.0, 0.0, 0.0), tolerance))
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 0.0, 1.0), tolerance))
            self.assertTrue(hitInfo["faceIndex"] == 1)
            self.assertTrue(hitCollision == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitRigidBody == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitMaterial == '')
        else:
            print("Hit not found.")
            self.assertTrue(False)

        hitInfo = get_physx_scene_query_interface().raycast_closest(Gf.Vec3f(300.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0)
        if hitInfo["hit"]:
            hitPos = Gf.Vec3f(hitInfo["position"].x, hitInfo["position"].y, hitInfo["position"].z)
            hitNormal = Gf.Vec3f(hitInfo["normal"].x, hitInfo["normal"].y, hitInfo["normal"].z)
            hitCollision = hitInfo["collision"]
            hitRigidBody = hitInfo["rigidBody"]
            hitMaterial = hitInfo["material"]
            self.assertTrue(Gf.IsClose(hitPos, Gf.Vec3f(300.0, 0.0, 0.0), tolerance))
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 0.0, 1.0), tolerance))
            self.assertTrue(hitInfo["faceIndex"] == 3)
            self.assertTrue(hitCollision == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitRigidBody == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitMaterial == '')
        else:
            print("Hit not found.")
            self.assertTrue(False)

    async def test_physics_raycast_all_multi_material(self):
        stage = await self.new_stage()

        self.setup_stage_multi_material(stage, False)

        tolerance = 0.001

        self.material = ''
        self.faceIndex = 0

        def report_all_hits(hit):
            hitNormal = Gf.Vec3f(hit.normal.x, hit.normal.y, hit.normal.z)
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 0.0, 1.0), tolerance))
            self.assertTrue(hit.material == self.material)
            self.assertTrue(hit.face_index == self.faceIndex)
            return True
        
        self.material = self.defaultPrimPath + "/material0"
        self.faceIndex = 2
        numHits = get_physx_scene_query_interface().raycast_all(Gf.Vec3f(200.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0, report_all_hits)
        self.assertTrue(numHits == 1)

        self.material = self.defaultPrimPath + "/material1"
        self.faceIndex = 1
        numHits = get_physx_scene_query_interface().raycast_all(Gf.Vec3f(100.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0, report_all_hits)
        self.assertTrue(numHits == 1)

        self.material = self.defaultPrimPath + "/material1"
        self.faceIndex = 3
        numHits = get_physx_scene_query_interface().raycast_all(Gf.Vec3f(300.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0, report_all_hits)
        self.assertTrue(numHits == 1)

    async def test_physics_raycast_all_multi_material_default(self):
        stage = await self.new_stage()

        self.setup_stage_multi_material(stage, True)

        tolerance = 0.001

        self.material = ''
        self.faceIndex = 0

        def report_all_hits(hit):
            hitNormal = Gf.Vec3f(hit.normal.x, hit.normal.y, hit.normal.z)
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 0.0, 1.0), tolerance))
            self.assertTrue(hit.material == self.material)
            self.assertTrue(hit.face_index == self.faceIndex)
            return True
        
        self.material = self.defaultPrimPath + "/material0"
        self.faceIndex = 2
        numHits = get_physx_scene_query_interface().raycast_all(Gf.Vec3f(200.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0, report_all_hits)
        self.assertTrue(numHits == 1)

        self.material = ''
        self.faceIndex = 1
        numHits = get_physx_scene_query_interface().raycast_all(Gf.Vec3f(100.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0, report_all_hits)
        self.assertTrue(numHits == 1)

        self.material = ''
        self.faceIndex = 3
        numHits = get_physx_scene_query_interface().raycast_all(Gf.Vec3f(300.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0, report_all_hits)
        self.assertTrue(numHits == 1)

    def eval_sweep_test_closest(self, hitInfo, desired_position: (Gf.Vec3f, Gf.Vec3d) = Gf.Vec3f(0.0, 50.0, 0.0), desired_normal: (Gf.Vec3f, Gf.Vec3d) = Gf.Vec3f(0.0, 1.0, 0.0), point_instancer: bool = False):
        tolerance = 0.001
        if hitInfo["hit"]:
            hitPos = Gf.Vec3f(hitInfo["position"].x, hitInfo["position"].y, hitInfo["position"].z)
            hitNormal = Gf.Vec3f(hitInfo["normal"].x, hitInfo["normal"].y, hitInfo["normal"].z)
            hitCollision = hitInfo["collision"]
            hitRigidBody = hitInfo["rigidBody"]
            self.assertTrue(Gf.IsClose(hitPos, desired_position, tolerance))
            self.assertTrue(Gf.IsClose(hitNormal, desired_normal, tolerance))
            if point_instancer:
                self.assertTrue(hitCollision == "/pointinstancer/boxActor")
                self.assertTrue(hitRigidBody == "/pointinstancer/boxActor")
                self.assertTrue(hitInfo["protoIndex"] == 0)
            else:
                self.assertTrue(hitCollision == self.defaultPrimPath + "/boxActor0")
                self.assertTrue(hitRigidBody == self.defaultPrimPath + "/boxActor0")
                self.assertTrue(hitInfo["protoIndex"] == 0xFFFFFFFF)
        else:
            print("Hit not found.")
            self.assertTrue(False)

    async def test_physics_sweep_sphere_closest(self):
        stage = await self.new_stage()

        self.setup_stage(stage)

        self.eval_sweep_test_closest(
            get_physx_scene_query_interface().sweep_sphere_closest(10.0, Gf.Vec3f(0.0, 200.0, 0.0), Gf.Vec3f(0.0, -1.0, 0.0), 1000.0))

    async def test_physics_sweep_box_closest(self):
        stage = await self.new_stage()

        self.setup_stage(stage)
        
        self.eval_sweep_test_closest(
            get_physx_scene_query_interface().sweep_box_closest(Gf.Vec3f(50.0, 50.0, 50.0), Gf.Vec3f(0.0, 500.0, 0.0), 
                                                                      carb.Float4( -0.364705, 0.1159176, -0.2798498, 0.8804757), Gf.Vec3f(0.0, -1.0, 0.0), 1000.0))

    async def test_physics_sweep_mesh_closest(self):
        stage = await self.new_stage()

        self.setup_stage(stage)

        cubePrim = self.create_cube_mesh(stage, 100.0, Gf.Vec3d(0.0, 500.0, 0.0), Gf.Quatf(0.8804757, -0.364705, 0.1159176, -0.2798498))
        physicsUtils._add_rigid(cubePrim.GetPrim(), 1.0, Gf.Vec3f(0.0), Gf.Vec3f(0.0))
        path_tuple = PhysicsSchemaTools.encodeSdfPath(cubePrim.GetPrim().GetPrimPath())

        self.eval_sweep_test_closest(
            get_physx_scene_query_interface().sweep_mesh_closest(path_tuple[0], path_tuple[1], Gf.Vec3f(0.0, -1.0, 0.0), 1000.0))

    async def test_physics_sweep_shape_closest(self):
        for geometry in self.geoms_list:
            print(f"Geometry type: {geometry}")
            stage = await self.new_stage()
            self.setup_stage(stage)
            position =  Gf.Vec3f(0.0, 50.0, 0.0)
            if geometry == "cube" or geometry == "mesh":
                orientation = Gf.Quatf(0.8804757, -0.364705, 0.1159176, -0.2798498)
            elif geometry == "cylinder":
                orientation = Gf.Quatf(0.9238795, 0.3826834, 0, 0)
                position = Gf.Vec3f(0.0, 50.0, -0.193687)
            elif geometry == "cone":
                orientation = Gf.Quatf(0, 1, 0, 0)
            else:
                orientation = Gf.Quatf()
            shape = self.create_geometry(stage, geometry, 100.0, Gf.Vec3d(0.0, 500.0, 0.0), orientation)
            physicsUtils._add_rigid(shape.GetPrim(), 1.0, Gf.Vec3f(0.0), Gf.Vec3f(0.0))
            path_tuple = []
            path_tuple = PhysicsSchemaTools.encodeSdfPath(shape.GetPrim().GetPrimPath())

            self.eval_sweep_test_closest(
                get_physx_scene_query_interface().sweep_shape_closest(path_tuple[0], path_tuple[1], Gf.Vec3f(0.0, -1.0, 0.0), 1000.0),
                desired_position=position)

    async def test_physics_sweep_sphere_closest_point_instancer(self):
        stage = await self.new_stage()

        self.setup_stage(stage, "point_instancer")

        self.eval_sweep_test_closest(
            get_physx_scene_query_interface().sweep_sphere_closest(10.0, Gf.Vec3f(0.0, 200.0, 0.0), Gf.Vec3f(0.0, -1.0, 0.0), 1000.0), 
            point_instancer=True)

    async def test_physics_sweep_box_closest_point_instancer(self):
        stage = await self.new_stage()

        self.setup_stage(stage, "point_instancer")
        
        self.eval_sweep_test_closest(
            get_physx_scene_query_interface().sweep_box_closest(Gf.Vec3f(50.0, 50.0, 50.0), Gf.Vec3f(0.0, 500.0, 0.0), 
                                                                      carb.Float4( -0.364705, 0.1159176, -0.2798498, 0.8804757), Gf.Vec3f(0.0, -1.0, 0.0), 1000.0),
                                                                      point_instancer=True)

    async def test_physics_sweep_mesh_closest_point_instancer(self):
        stage = await self.new_stage()

        self.setup_stage(stage, "point_instancer")

        cubePrim = self.create_cube_mesh(stage, 100.0, Gf.Vec3d(0.0, 500.0, 0.0), Gf.Quatf(0.8804757, -0.364705, 0.1159176, -0.2798498))
        physicsUtils._add_rigid(cubePrim.GetPrim(), 1.0, Gf.Vec3f(0.0), Gf.Vec3f(0.0))
        path_tuple = PhysicsSchemaTools.encodeSdfPath(cubePrim.GetPrim().GetPrimPath())

        self.eval_sweep_test_closest(
            get_physx_scene_query_interface().sweep_mesh_closest(path_tuple[0], path_tuple[1], Gf.Vec3f(0.0, -1.0, 0.0), 1000.0),
            point_instancer=True)

    async def test_physics_sweep_shape_closest_point_instancer(self):
        for geometry in self.geoms_list:
            print(f"Geometry type: {geometry}")
            stage = await self.new_stage()
            self.setup_stage(stage, "point_instancer")
            position =  Gf.Vec3f(0.0, 50.0, 0.0)
            if geometry == "cube" or geometry == "mesh":
                orientation = Gf.Quatf(0.8804757, -0.364705, 0.1159176, -0.2798498)
            elif geometry == "cylinder":
                orientation = Gf.Quatf(0.9238795, 0.3826834, 0, 0)
                position = Gf.Vec3f(0.0, 50.0, -0.193687)
            elif geometry == "cone":
                orientation = Gf.Quatf(0, 1, 0, 0)
            else:
                orientation = Gf.Quatf()
            shape = self.create_geometry(stage, geometry, 100.0, Gf.Vec3d(0.0, 500.0, 0.0), orientation)
            physicsUtils._add_rigid(shape.GetPrim(), 1.0, Gf.Vec3f(0.0), Gf.Vec3f(0.0))
            path_tuple = []
            path_tuple = PhysicsSchemaTools.encodeSdfPath(shape.GetPrim().GetPrimPath())

            self.eval_sweep_test_closest(
                get_physx_scene_query_interface().sweep_shape_closest(path_tuple[0], path_tuple[1], Gf.Vec3f(0.0, -1.0, 0.0), 1000.0),
                desired_position=position, point_instancer=True)

    async def test_physics_sweep_sphere_any(self):
        stage = await self.new_stage()

        self.setup_stage(stage)

        if get_physx_scene_query_interface().sweep_sphere_any(10.0, Gf.Vec3f(0.0, 200.0, 0.0), Gf.Vec3f(0.0, -1.0, 0.0), 1000.0):
            print("Hit found.")
        else:
            print("Hit not found.")
            self.assertTrue(False)

    async def test_physics_sweep_box_any(self):
        stage = await self.new_stage()

        self.setup_stage(stage)
        
        if (get_physx_scene_query_interface().sweep_box_any(Gf.Vec3f(50.0, 50.0, 50.0), Gf.Vec3f(0.0, 500.0, 0.0), 
                                                                      carb.Float4( -0.364705, 0.1159176, -0.2798498, 0.8804757), Gf.Vec3f(0.0, -1.0, 0.0), 1000.0)):
            print("Hit found.")
        else:
            print("Hit not found.")
            self.assertTrue(False)

    async def test_physics_sweep_mesh_any(self):
        stage = await self.new_stage()

        self.setup_stage(stage)

        cubePrim = self.create_cube_mesh(stage, 100.0, Gf.Vec3d(0.0, 500.0, 0.0), Gf.Quatf(0.8804757, -0.364705, 0.1159176, -0.2798498))
        physicsUtils._add_rigid(cubePrim.GetPrim(), 1.0, Gf.Vec3f(0.0), Gf.Vec3f(0.0))
        path_tuple = PhysicsSchemaTools.encodeSdfPath(cubePrim.GetPrim().GetPrimPath())

        if get_physx_scene_query_interface().sweep_mesh_any(path_tuple[0], path_tuple[1], Gf.Vec3f(0.0, -1.0, 0.0), 1000.0):
            print("Hit found.")
        else:
            print("Hit not found.")
            self.assertTrue(False)

    async def test_physics_sweep_shape_any(self):
        for geometry in self.geoms_list:
            print(f"Geometry type: {geometry}")
            stage = await self.new_stage()
            self.setup_stage(stage)
            if geometry == "cube" or geometry == "mesh":
                orientation = Gf.Quatf(0.8804757, -0.364705, 0.1159176, -0.2798498)
            elif geometry == "cylinder":
                orientation = Gf.Quatf(0.9238795, 0.3826834, 0, 0)
            elif geometry == "cone":
                orientation = Gf.Quatf(0, 1, 0, 0)
            else:
                orientation = Gf.Quatf()
            shape = self.create_geometry(stage, geometry, 100.0, Gf.Vec3d(0.0, 500.0, 0.0), orientation)
            physicsUtils._add_rigid(shape.GetPrim(), 1.0, Gf.Vec3f(0.0), Gf.Vec3f(0.0))
            path_tuple = []
            path_tuple = PhysicsSchemaTools.encodeSdfPath(shape.GetPrim().GetPrimPath())

            if get_physx_scene_query_interface().sweep_shape_any(path_tuple[0], path_tuple[1], Gf.Vec3f(0.0, -1.0, 0.0), 1000.0):
                print("Hit found.")
            else:
                print("Hit not found.")
                self.assertTrue(False)

    def report_all_hits(self, hit):
        tolerance = 0.001
        hitNormal = Gf.Vec3f(hit.normal.x, hit.normal.y, hit.normal.z)
        self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 1.0, 0.0), tolerance))
        self.numHits = self.numHits + 1
        return True
    
    def eval_sweep_test_all(self, found_hit):
        if not found_hit:
            print("No hits found.")
            self.assertTrue(False)
        if self.numHits and self.numHits == 2:
            print("Hits found.")
        else:
            print("Incorrect number of hits found: " + str(self.numHits))
            self.assertTrue(False)

    async def test_physics_sweep_sphere_all(self):
        stage = await self.new_stage()

        self.setup_stage(stage)

        self.numHits = 0

        self.eval_sweep_test_all(
            get_physx_scene_query_interface().sweep_sphere_all(10.0, Gf.Vec3f(0.0, 200.0, 0.0), Gf.Vec3f(0.0, -1.0, 0.0), 1000.0, self.report_all_hits))

    async def test_physics_sweep_box_all(self):
        stage = await self.new_stage()

        self.setup_stage(stage)
        
        self.numHits = 0

        self.eval_sweep_test_all(
            get_physx_scene_query_interface().sweep_box_all(Gf.Vec3f(50.0, 50.0, 50.0), Gf.Vec3f(0.0, 500.0, 0.0), 
                                                                      carb.Float4( -0.364705, 0.1159176, -0.2798498, 0.8804757), Gf.Vec3f(0.0, -1.0, 0.0), 
                                                                      1000.0, self.report_all_hits))

    async def test_physics_sweep_mesh_all(self):
        stage = await self.new_stage()

        self.setup_stage(stage)

        self.numHits = 0

        cubePrim = self.create_cube_mesh(stage, 100.0, Gf.Vec3d(0.0, 500.0, 0.0), Gf.Quatf(0.8804757, -0.364705, 0.1159176, -0.2798498))
        physicsUtils._add_rigid(cubePrim.GetPrim(), 1.0, Gf.Vec3f(0.0), Gf.Vec3f(0.0))
        path_tuple = PhysicsSchemaTools.encodeSdfPath(cubePrim.GetPrim().GetPrimPath())

        self.eval_sweep_test_all(
            get_physx_scene_query_interface().sweep_mesh_all(path_tuple[0], path_tuple[1], Gf.Vec3f(0.0, -1.0, 0.0), 1000.0, self.report_all_hits))

    async def test_physics_sweep_shape_all(self):
        for geometry in self.geoms_list:
            print(f"Geometry type: {geometry}")
            stage = await self.new_stage()
            self.setup_stage(stage)
            if geometry == "cube" or geometry == "mesh":
                orientation = Gf.Quatf(0.8804757, -0.364705, 0.1159176, -0.2798498)
            elif geometry == "cylinder":
                orientation = Gf.Quatf(0.9238795, 0.3826834, 0, 0)
            elif geometry == "cone":
                orientation = Gf.Quatf(0, 1, 0, 0)
            else:
                orientation = Gf.Quatf()
            shape = self.create_geometry(stage, geometry, 100.0, Gf.Vec3d(0.0, 500.0, 0.0), orientation)
            physicsUtils._add_rigid(shape.GetPrim(), 1.0, Gf.Vec3f(0.0), Gf.Vec3f(0.0))
            path_tuple = []
            path_tuple = PhysicsSchemaTools.encodeSdfPath(shape.GetPrim().GetPrimPath())

            self.numHits = 0

            self.eval_sweep_test_all(
                get_physx_scene_query_interface().sweep_shape_all(path_tuple[0], path_tuple[1], Gf.Vec3f(0.0, -1.0, 0.0), 1000.0, self.report_all_hits))


    async def test_physics_sweep_closest_multi_material(self):
        stage = await self.new_stage()

        self.setup_stage_multi_material(stage, False)

        tolerance = 0.001

        hitInfo = get_physx_scene_query_interface().sweep_sphere_closest(10.0, Gf.Vec3f(200.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0)
        if hitInfo["hit"]:
            hitPos = Gf.Vec3f(hitInfo["position"].x, hitInfo["position"].y, hitInfo["position"].z)
            hitNormal = Gf.Vec3f(hitInfo["normal"].x, hitInfo["normal"].y, hitInfo["normal"].z)
            hitCollision = hitInfo["collision"]
            hitRigidBody = hitInfo["rigidBody"]
            hitMaterial = hitInfo["material"]
            self.assertTrue(Gf.IsClose(hitPos, Gf.Vec3f(200.0, 0.0, 0.0), tolerance))
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 0.0, 1.0), tolerance))
            self.assertTrue(hitInfo["faceIndex"] == 2)
            self.assertTrue(hitCollision == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitRigidBody == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitMaterial == self.defaultPrimPath + "/material0")
        else:
            print("Hit not found.")
            self.assertTrue(False)

        hitInfo = get_physx_scene_query_interface().sweep_sphere_closest(10.0, Gf.Vec3f(100.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0)
        if hitInfo["hit"]:
            hitPos = Gf.Vec3f(hitInfo["position"].x, hitInfo["position"].y, hitInfo["position"].z)
            hitNormal = Gf.Vec3f(hitInfo["normal"].x, hitInfo["normal"].y, hitInfo["normal"].z)
            hitCollision = hitInfo["collision"]
            hitRigidBody = hitInfo["rigidBody"]
            hitMaterial = hitInfo["material"]
            self.assertTrue(Gf.IsClose(hitPos, Gf.Vec3f(100.0, 0.0, 0.0), tolerance))
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 0.0, 1.0), tolerance))
            self.assertTrue(hitInfo["faceIndex"] == 1)
            self.assertTrue(hitCollision == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitRigidBody == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitMaterial == self.defaultPrimPath + "/material1")
        else:
            print("Hit not found.")
            self.assertTrue(False)

        hitInfo = get_physx_scene_query_interface().sweep_sphere_closest(10.0, Gf.Vec3f(300.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0)
        if hitInfo["hit"]:
            hitPos = Gf.Vec3f(hitInfo["position"].x, hitInfo["position"].y, hitInfo["position"].z)
            hitNormal = Gf.Vec3f(hitInfo["normal"].x, hitInfo["normal"].y, hitInfo["normal"].z)
            hitCollision = hitInfo["collision"]
            hitRigidBody = hitInfo["rigidBody"]
            hitMaterial = hitInfo["material"]
            self.assertTrue(Gf.IsClose(hitPos, Gf.Vec3f(300.0, 0.0, 0.0), tolerance))
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 0.0, 1.0), tolerance))
            self.assertTrue(hitInfo["faceIndex"] == 3)
            self.assertTrue(hitCollision == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitRigidBody == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitMaterial == self.defaultPrimPath + "/material1")
        else:
            print("Hit not found.")
            self.assertTrue(False)

    async def test_physics_sweep_closest_multi_material_default(self):
        stage = await self.new_stage()

        self.setup_stage_multi_material(stage, True)

        tolerance = 0.001

        hitInfo = get_physx_scene_query_interface().sweep_sphere_closest(10.0, Gf.Vec3f(200.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0)
        if hitInfo["hit"]:
            hitPos = Gf.Vec3f(hitInfo["position"].x, hitInfo["position"].y, hitInfo["position"].z)
            hitNormal = Gf.Vec3f(hitInfo["normal"].x, hitInfo["normal"].y, hitInfo["normal"].z)
            hitCollision = hitInfo["collision"]
            hitRigidBody = hitInfo["rigidBody"]
            hitMaterial = hitInfo["material"]
            self.assertTrue(Gf.IsClose(hitPos, Gf.Vec3f(200.0, 0.0, 0.0), tolerance))
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 0.0, 1.0), tolerance))
            self.assertTrue(hitInfo["faceIndex"] == 2)
            self.assertTrue(hitCollision == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitRigidBody == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitMaterial == self.defaultPrimPath + "/material0")
        else:
            print("Hit not found.")
            self.assertTrue(False)

        hitInfo = get_physx_scene_query_interface().sweep_sphere_closest(10.0, Gf.Vec3f(100.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0)
        if hitInfo["hit"]:
            hitPos = Gf.Vec3f(hitInfo["position"].x, hitInfo["position"].y, hitInfo["position"].z)
            hitNormal = Gf.Vec3f(hitInfo["normal"].x, hitInfo["normal"].y, hitInfo["normal"].z)
            hitCollision = hitInfo["collision"]
            hitRigidBody = hitInfo["rigidBody"]
            hitMaterial = hitInfo["material"]
            self.assertTrue(Gf.IsClose(hitPos, Gf.Vec3f(100.0, 0.0, 0.0), tolerance))
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 0.0, 1.0), tolerance))
            self.assertTrue(hitInfo["faceIndex"] == 1)
            self.assertTrue(hitCollision == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitRigidBody == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitMaterial == '')
        else:
            print("Hit not found.")
            self.assertTrue(False)

        hitInfo = get_physx_scene_query_interface().sweep_sphere_closest(10.0, Gf.Vec3f(300.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0)
        if hitInfo["hit"]:
            hitPos = Gf.Vec3f(hitInfo["position"].x, hitInfo["position"].y, hitInfo["position"].z)
            hitNormal = Gf.Vec3f(hitInfo["normal"].x, hitInfo["normal"].y, hitInfo["normal"].z)
            hitCollision = hitInfo["collision"]
            hitRigidBody = hitInfo["rigidBody"]
            hitMaterial = hitInfo["material"]
            self.assertTrue(Gf.IsClose(hitPos, Gf.Vec3f(300.0, 0.0, 0.0), tolerance))
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 0.0, 1.0), tolerance))
            self.assertTrue(hitInfo["faceIndex"] == 3)
            self.assertTrue(hitCollision == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitRigidBody == self.defaultPrimPath + "/triangleMesh")
            self.assertTrue(hitMaterial == '')
        else:
            print("Hit not found.")
            self.assertTrue(False)

    async def test_physics_sweep_all_multi_material(self):
        stage = await self.new_stage()

        self.setup_stage_multi_material(stage, False)

        tolerance = 0.001

        self.material = ''
        self.faceIndex = 0

        def report_all_hits(hit):
            hitNormal = Gf.Vec3f(hit.normal.x, hit.normal.y, hit.normal.z)
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 0.0, 1.0), tolerance))
            self.assertTrue(hit.material == self.material)
            self.assertTrue(hit.face_index == self.faceIndex)
            return True
        
        self.material = self.defaultPrimPath + "/material0"
        self.faceIndex = 2
        numHits = get_physx_scene_query_interface().sweep_sphere_all(10.0, Gf.Vec3f(200.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0, report_all_hits)
        self.assertTrue(numHits == 1)

        self.material = self.defaultPrimPath + "/material1"
        self.faceIndex = 1
        numHits = get_physx_scene_query_interface().sweep_sphere_all(10.0, Gf.Vec3f(100.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0, report_all_hits)
        self.assertTrue(numHits == 1)

        self.material = self.defaultPrimPath + "/material1"
        self.faceIndex = 3
        numHits = get_physx_scene_query_interface().sweep_sphere_all(10.0, Gf.Vec3f(300.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0, report_all_hits)
        self.assertTrue(numHits == 1)

    async def test_physics_sweep_all_multi_material_default(self):
        stage = await self.new_stage()

        self.setup_stage_multi_material(stage, True)

        tolerance = 0.001

        self.material = ''
        self.faceIndex = 0

        def report_all_hits(hit):
            hitNormal = Gf.Vec3f(hit.normal.x, hit.normal.y, hit.normal.z)
            self.assertTrue(Gf.IsClose(hitNormal, Gf.Vec3f(0.0, 0.0, 1.0), tolerance))
            self.assertTrue(hit.material == self.material)
            self.assertTrue(hit.face_index == self.faceIndex)
            return True
        
        self.material = self.defaultPrimPath + "/material0"
        self.faceIndex = 2
        numHits = get_physx_scene_query_interface().sweep_sphere_all(10.0, Gf.Vec3f(200.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0, report_all_hits)
        self.assertTrue(numHits == 1)

        self.material = ''
        self.faceIndex = 1
        numHits = get_physx_scene_query_interface().sweep_sphere_all(10.0, Gf.Vec3f(100.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0, report_all_hits)
        self.assertTrue(numHits == 1)

        self.material = ''
        self.faceIndex = 3
        numHits = get_physx_scene_query_interface().sweep_sphere_all(10.0, Gf.Vec3f(300.0, 0.0, 100.0), Gf.Vec3f(0.0, 0.0, -1.0), 1000.0, report_all_hits)
        self.assertTrue(numHits == 1)

    async def test_physics_overlap_box_any(self):
        stage = await self.new_stage()

        self.setup_stage(stage)
        
        hitInfo = get_physx_scene_query_interface().overlap_box_any(Gf.Vec3f(100.0), Gf.Vec3f(0.0),  carb.Float4(0.0, 0.0, 0.0, 1.0))
        if hitInfo:
            print("Hit found.")
        else:
            print("Hit not found.")
            self.assertTrue(False)

    async def test_physics_overlap_box(self):
        stage = await self.new_stage()

        self.setup_stage(stage)
        
        hitInfo = get_physx_scene_query_interface().overlap_box(Gf.Vec3f(100.0), Gf.Vec3f(0.0),  carb.Float4(0.0, 0.0, 0.0, 1.0), None, True)
        if hitInfo:
            print("Hit found.")
        else:
            print("Hit not found.")
            self.assertTrue(False)

        tolerance = 0.001

        self.numHits = 0

        def report_all_hits(hit):
            self.numHits = self.numHits + 1
            self.assertTrue(hit.protoIndex == 0xFFFFFFFF)
            return True

        numHits = get_physx_scene_query_interface().overlap_box(Gf.Vec3f(1000.0), Gf.Vec3f(0.0),  carb.Float4(0.0, 0.0, 0.0, 1.0), report_all_hits)
        if numHits == 2 and self.numHits == 2:
            print("Hits found.")
        else:
            print("Hits not found. Num hits: " + str(self.numHits))
            self.assertTrue(False)
            
    async def test_physics_overlap_box_point_instancer(self):
        stage = await self.new_stage()

        self.setup_stage(stage, "point_instancer")
        
        hitInfo = get_physx_scene_query_interface().overlap_box(Gf.Vec3f(100.0), Gf.Vec3f(0.0),  carb.Float4(0.0, 0.0, 0.0, 1.0), None, True)
        if hitInfo:
            print("Hit found.")
        else:
            print("Hit not found.")
            self.assertTrue(False)

        tolerance = 0.001

        self.numHits = 0

        def report_all_hits(hit):
            self.numHits = self.numHits + 1
            self.assertTrue(hit.protoIndex == 0)
            return True

        numHits = get_physx_scene_query_interface().overlap_box(Gf.Vec3f(1000.0), Gf.Vec3f(0.0),  carb.Float4(0.0, 0.0, 0.0, 1.0), report_all_hits)
        if numHits == 1 and self.numHits == 1:
            print("Hits found.")
        else:
            print("Hits not found. Num hits: " + str(self.numHits))
            self.assertTrue(False)
            
    async def test_physics_overlap_sphere_any(self):
        stage = await self.new_stage()

        self.setup_stage(stage)
        
        hitInfo = get_physx_scene_query_interface().overlap_sphere_any(100.0, Gf.Vec3f(0.0))
        if hitInfo:
            print("Hit found.")
        else:
            print("Hit not found.")
            self.assertTrue(False)

    async def test_physics_overlap_sphere(self):
        stage = await self.new_stage()

        self.setup_stage(stage)
        
        hitInfo = get_physx_scene_query_interface().overlap_sphere(100.0, Gf.Vec3f(0.0), None, True)
        if hitInfo:
            print("Hit found.")
        else:
            print("Hit not found.")
            self.assertTrue(False)

        tolerance = 0.001

        self.numHits = 0

        def report_all_hits(hit):
            self.numHits = self.numHits + 1
            return True

        numHits = get_physx_scene_query_interface().overlap_sphere(1000.0, Gf.Vec3f(0.0), report_all_hits)
        if numHits == 2 and self.numHits == 2:
            print("Hits found.")
        else:
            print("Hits not found. Num hits: " + str(self.numHits))
            self.assertTrue(False)  


    async def test_physics_overlap_mesh_any(self):
        stage = await self.new_stage()

        self.setup_stage(stage)

        self.create_cube_mesh(stage, 100.0)
        
        path_tuple = []
        path_tuple = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(self.defaultPrimPath + '/mesh'))
        hitInfo = get_physx_scene_query_interface().overlap_mesh_any(path_tuple[0], path_tuple[1])

        if hitInfo:
            print("Hit found.")
        else:
            print("Hit not found.")
            self.assertTrue(False)

    async def test_physics_overlap_mesh(self):
        stage = await self.new_stage()

        self.setup_stage(stage)
        
        self.create_cube_mesh(stage, 1000.0)
        
        path_tuple = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(self.defaultPrimPath + '/mesh'))
        hitInfo = get_physx_scene_query_interface().overlap_mesh(path_tuple[0], path_tuple[1], None, True)

        if hitInfo:
            print("Hit found.")
        else:
            print("Hit not found.")
            self.assertTrue(False)

        self.numHits = 0

        def report_all_hits(hit):
            self.numHits = self.numHits + 1
            return True

        numHits = get_physx_scene_query_interface().overlap_mesh(path_tuple[0], path_tuple[1], report_all_hits)
        if numHits == 2 and self.numHits == 2:
            print("Hits found.")
        else:
            print("Hits not found. Num hits: " + str(self.numHits))
            self.assertTrue(False)             

    async def test_physics_overlap_shape_any(self):
        geoms_list = ["sphere", "cube", "capsule", "cylinder", "cone", "mesh"]
        for geometry in geoms_list:
            stage = await self.new_stage()
            self.setup_stage(stage)

            geom_path = Sdf.Path(self.defaultPrimPath + '/' + geometry)
            path_tuple = []
            path_tuple = PhysicsSchemaTools.encodeSdfPath(geom_path)
                        
            if geometry == "mesh":
                self.create_cube_mesh(stage, 100.0)                
            elif geometry == "sphere":
                UsdGeom.Sphere.Define(stage, geom_path)
            elif geometry == "cube":
                UsdGeom.Cube.Define(stage, geom_path)
            elif geometry == "capsule":
                UsdGeom.Capsule.Define(stage, geom_path)
            elif geometry == "cylinder":
                UsdGeom.Cylinder.Define(stage, geom_path)
            elif geometry == "cone":
                UsdGeom.Cone.Define(stage, geom_path)

            hitInfo = get_physx_scene_query_interface().overlap_shape_any(path_tuple[0], path_tuple[1])

            if hitInfo:
                print("Hit found.")
            else:
                print("Hit not found.")
                self.assertTrue(False)

    async def test_physics_overlap_shape(self):
        geoms_list = ["sphere", "cube", "capsule", "cylinder", "cone", "mesh"]
        for geometry in geoms_list:
            stage = await self.new_stage()
            self.setup_stage(stage)

            geom_path = Sdf.Path(self.defaultPrimPath + '/' + geometry)
            path_tuple = []
            path_tuple = PhysicsSchemaTools.encodeSdfPath(geom_path)
                        
            if geometry == "mesh":
                self.create_cube_mesh(stage, 1000.0)                
            elif geometry == "sphere":
                sphere = UsdGeom.Sphere.Define(stage, geom_path)
                sphere.CreateRadiusAttr().Set(300)
            elif geometry == "cube":
                cube = UsdGeom.Cube.Define(stage, geom_path)
                cube.CreateSizeAttr().Set(300)
            elif geometry == "capsule":
                capsule = UsdGeom.Capsule.Define(stage, geom_path)
                capsule.CreateRadiusAttr().Set(200)
                capsule.CreateHeightAttr().Set(200)
            elif geometry == "cylinder":
                cylinder = UsdGeom.Cylinder.Define(stage, geom_path)
                cylinder.CreateRadiusAttr().Set(200)
                cylinder.CreateHeightAttr().Set(200)
            elif geometry == "cone":
                cone = UsdGeom.Cone.Define(stage, geom_path)
                cone.CreateRadiusAttr().Set(2000)
                cone.CreateHeightAttr().Set(2000)
                
            hitInfo = get_physx_scene_query_interface().overlap_shape(path_tuple[0], path_tuple[1], None, True)

            if hitInfo:
                print("Hit found.")
            else:
                print("Hit not found. " + str(geom_path))
                self.assertTrue(False)

            self.numHits = 0

            def report_all_hits(hit):
                self.numHits = self.numHits + 1
                return True

            numHits = get_physx_scene_query_interface().overlap_shape(path_tuple[0], path_tuple[1], report_all_hits)
            if numHits == 2 and self.numHits == 2:
                print("Hits found.")
            else:
                print("Hits not found. Num hits: " + str(self.numHits) + " " + str(geom_path))
                self.assertTrue(False)             

    async def test_physics_material_face_cleanup(self):
        stage = await self.new_stage()

        self.setup_stage(stage, "mesh_cleanup")
        
        hitInfo = get_physx_scene_query_interface().raycast_closest(Gf.Vec3f(-1.0, 0.0, 10.0), Gf.Vec3f(0.0, 0.0, -1.0), 100.0)
        if hitInfo["hit"]:
            self.assertTrue(hitInfo["faceIndex"] == 3)
        else:
            print("Hit not found.")
            self.assertTrue(False)
