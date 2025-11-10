# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests import utils
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Vt, UsdPhysics
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
import carb
import math
from omni.physx.scripts.physicsUtils import get_initial_collider_pairs


use_tri_mesh_as_ground_plane = False

def joint_transform(bodyPos, bodyRot, p):
    trRotInv = Gf.Rotation(bodyRot).GetInverse()
    return trRotInv.TransformDir(p - bodyPos)

def scene_setup(stage):
    # set up axis to z
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)

    # light
    sphereLight = UsdLux.SphereLight.Define(stage, Sdf.Path("/SphereLight"))
    sphereLight.CreateRadiusAttr(150)
    sphereLight.CreateIntensityAttr(30000)
    sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

    # Physics scene
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(981.0)

def ground_plane_setup(stage):
    if use_tri_mesh_as_ground_plane:
        # (Graphics) Plane mesh
        # Trimesh
        shapeColor = Gf.Vec3f(0.5)
        entityPlane = physicsUtils.create_mesh_square_axis(stage, Sdf.Path("/trimesh"), "Z", 1500.0)
        entityPlane.CreateDisplayColorAttr().Set([shapeColor])
        prim = stage.GetPrimAtPath(Sdf.Path("/trimesh"))
        collisionAPI = UsdPhysics.CollisionAPI.Apply(prim)
        collisionAPI.CreateApproximationShapeAttr("none")
    else:
        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

def rigid_box_setup(stage):
    size = 50.0
    density = 1000
    color = Gf.Vec3f(71.0 / 255.0, 165.0 / 255.0, 1.0)
    position = Gf.Vec3f(50.0, 50.0, 200.0)
    orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    linVelocity = Gf.Vec3f(0.0)
    angularVelocity = Gf.Vec3f(0.0)
    cubePath = "/boxActor"

    physicsUtils.add_rigid_box(
        stage,
        cubePath,
        Gf.Vec3f(size, size, size),
        position,
        orientation,
        color,
        density,
        linVelocity,
        angularVelocity,
    )

def rigid_sphere_setup(stage):
    radius = 25.0
    density = 1000.0
    color = Gf.Vec3f(71.0 / 255.0, 125.0 / 255.0, 1.0)
    position = Gf.Vec3f(-50.0, -50.0, 200.0)
    orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    linVelocity = Gf.Vec3f(0.0)
    angularVelocity = Gf.Vec3f(0.0)
    spherePath = "/sphereActor"

    physicsUtils.add_rigid_sphere(stage, spherePath, radius, position, orientation, color, density, linVelocity, angularVelocity)

def rigid_capsule_setup(stage):
    radius = 25.0
    height = 50.0
    density = 1000.0
    color = Gf.Vec3f(71.0 / 255.0, 85.0 / 255.0, 1.0)
    position = Gf.Vec3f(50.0, -50.0, 200.0)
    orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    linVelocity = Gf.Vec3f(0.0)
    angularVelocity = Gf.Vec3f(0.0)
    capsulePath = "/capsuleActor"

    physicsUtils.add_rigid_capsule(
        stage, capsulePath, radius, height, "Z", position, orientation, color, density, linVelocity, angularVelocity
    )

def rigid_cylinder_setup(stage):
    radius = 25.0
    height = 50.0
    density = 1.0
    color = Gf.Vec3f(1.0, 0.0, 0.0)
    position = Gf.Vec3f(-50.0, 50.0, 200.0)
    orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    linVelocity = Gf.Vec3f(0.0)
    angularVelocity = Gf.Vec3f(0.0)
    path = "/cylinder"

    physicsUtils.add_rigid_cylinder(
        stage, path, radius, height, "X", position, orientation, color, density, linVelocity, angularVelocity
    )

def rigid_cone_setup(stage):
    radius = 25.0
    height = 50.0
    density = 1000.0
    color = Gf.Vec3f(71.0 / 255.0, 85.0 / 255.0, 1.0)
    position = Gf.Vec3f(-100.0, 100.0, 200.0)
    orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    linVelocity = Gf.Vec3f(0.0)
    angularVelocity = Gf.Vec3f(0.0)
    conePath = "/coneActor"

    physicsUtils.add_rigid_cone(
        stage, conePath, radius, height, "Z", position, orientation, color, density, linVelocity, angularVelocity
    )

def create_mesh_face_subset(mesh, faceIndices):
    return UsdGeom.Subset.CreateGeomSubset(mesh, "mySubset", "face", Vt.IntArray(faceIndices))

class PhysicsUtilsTestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core
    
    async def test_physics_utils(self):
        stage = await self.new_stage()

        scene_setup(stage)

        # expect one static body with plane
        ground_plane_setup(stage)
        self.step()
        utils.check_stats(self, {"numPlaneShapes": 1, "numStaticRigids": 1, "numDynamicRigids": 0 })

        # add dynamic box
        rigid_box_setup(stage)
        self.step()
        utils.check_stats(self, { "numPlaneShapes": 1, "numBoxShapes": 1, "numStaticRigids": 1, "numDynamicRigids": 1 })

        # add dynamic sphere
        rigid_sphere_setup(stage)
        self.step()
        utils.check_stats(self, { "numPlaneShapes": 1, "numBoxShapes": 1, "numSphereShapes": 1, "numStaticRigids": 1, "numDynamicRigids": 2 })

        # add dynamic capsule
        rigid_capsule_setup(stage)
        self.step()
        utils.check_stats(self, { "numPlaneShapes": 1, "numBoxShapes": 1, "numSphereShapes": 1, "numCapsuleShapes": 1, "numStaticRigids": 1, "numDynamicRigids": 3 })

        # add dynamic cylinder
        rigid_cylinder_setup(stage)
        self.step()
        utils.check_stats(self, { "numPlaneShapes": 1, "numBoxShapes": 1, "numSphereShapes": 1, "numCapsuleShapes": 1, "numCylinderShapes": 1, "numStaticRigids": 1, "numDynamicRigids": 4 })

        # add dynamic cone
        rigid_cone_setup(stage)
        self.step()
        utils.check_stats(self, { "numPlaneShapes": 1, "numBoxShapes": 1, "numSphereShapes": 1, "numCapsuleShapes": 1, "numCylinderShapes": 1, "numConeShapes": 1, "numStaticRigids": 1, "numDynamicRigids": 5 })

    async def test_compute_bounding_box_diagonal(self):        
        box_size = carb.Float3(2,3,4)
        points = [ carb.Float3(box_size.x, box_size.y, box_size.z),
            carb.Float3(box_size.x, -box_size.y, -box_size.z),
            carb.Float3(box_size.x, box_size.y, -box_size.z),
            carb.Float3(box_size.x, -box_size.y, box_size.z),
            carb.Float3(-box_size.x, -box_size.y, -box_size.z),
            carb.Float3(-box_size.x, box_size.y, -box_size.z),
            carb.Float3(-box_size.x, -box_size.y, box_size.z),
            carb.Float3(-box_size.x, box_size.y, box_size.z)          
        ]
        diag = physicsUtils.compute_bounding_box_diagonal(points)
        diag_comp = math.sqrt((box_size.x * 2) ** 2 + (box_size.y * 2) ** 2 + (box_size.z * 2) ** 2)
        self.assertTrue(diag == diag_comp)
        
    async def test_cylinder_mesh(self):
        stage = await self.new_stage()

        scene_setup(stage)
        
        cylinder0ActorPath = "/World/cylinder0Actor"

        radius = 50.0
        height = 100.0

        physicsUtils.create_mesh_cylinder(stage, cylinder0ActorPath, height, radius)
        cylinder0Prim = stage.GetPrimAtPath(cylinder0ActorPath)
        
        physicsUtils.add_mass(stage, cylinder0ActorPath, 10.0)

        UsdPhysics.CollisionAPI.Apply(cylinder0Prim)
        mesh_api = UsdPhysics.MeshCollisionAPI.Apply(cylinder0Prim)
        mesh_api.CreateApproximationAttr(UsdPhysics.Tokens.convexHull)

        self.step()
        utils.check_stats(self, {"numConvexShapes": 1, "numStaticRigids": 1})
        
    async def test_cone_mesh(self):
        stage = await self.new_stage()

        scene_setup(stage)
        
        cone0ActorPath = "/World/cone0Actor"

        radius = 50.0
        height = 100.0

        physicsUtils.create_mesh_cone(stage, cone0ActorPath, height, radius)
        cone0Prim = stage.GetPrimAtPath(cone0ActorPath)
        
        physicsUtils.add_density(stage, cone0ActorPath, 10.0)

        UsdPhysics.CollisionAPI.Apply(cone0Prim)
        mesh_api = UsdPhysics.MeshCollisionAPI.Apply(cone0Prim)
        mesh_api.CreateApproximationAttr(UsdPhysics.Tokens.convexHull)

        self.step()
        utils.check_stats(self, {"numConvexShapes": 1, "numStaticRigids": 1})        
        
    async def test_cube_ground_plane(self):
        stage = await self.new_stage()

        scene_setup(stage)
        
        cubeActorPath = "/World/cubeActor"

        physicsUtils.add_cube_ground_plane(stage, cubeActorPath, "Y", 100, Gf.Vec3f(0.0), Gf.Vec3f(1.0))

        self.step()
        utils.check_stats(self, {"numBoxShapes": 1, "numStaticRigids": 1})         
        
    async def test_collision_group_utils(self):
        stage = await self.new_stage()
        
        size = Gf.Vec3f(100.0)
        cube = physicsUtils.add_rigid_box(stage, "/cube", size)
        
        collision_group = UsdPhysics.CollisionGroup.Define(stage, "/World/CollisionGroup").GetPrim()
        
        physicsUtils.add_collision_to_collision_group(stage, cube.GetPrimPath(), collision_group.GetPrimPath())
        
        self.assertTrue(physicsUtils.is_in_collision_group(stage, cube.GetPrimPath(), collision_group.GetPrimPath()))
        
        physicsUtils.remove_collision_from_collision_group(stage, cube.GetPrimPath(), collision_group.GetPrimPath())
        
        self.assertTrue(not physicsUtils.is_in_collision_group(stage, cube.GetPrimPath(), collision_group.GetPrimPath()))        
        
    async def test_add_joint_fixed_utils(self):
        stage = await self.new_stage()
        
        size = Gf.Vec3f(100.0)
        cube = physicsUtils.add_rigid_box(stage, "/cube", size)
        cube2 = physicsUtils.add_rigid_box(stage, "/cube2", size)
        
        physicsUtils.add_joint_fixed(stage, "/fixedJoint", cube.GetPrimPath(), cube2.GetPrimPath(), Gf.Vec3f(0.0), Gf.Quatf(1.0), Gf.Vec3f(0.0), Gf.Quatf(1.0), 1e5, 1e5)
        physicsUtils.add_joint_fixed(stage, "/fixedJoint", cube.GetPrimPath(), None, Gf.Vec3f(0.0), Gf.Quatf(1.0), Gf.Vec3f(0.0), Gf.Quatf(1.0), 1e5, 1e5)
        
        self.step()
        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numConstraints": 2})         

    async def test_get_initial_collider_pairs(self):
        stage = await self.new_stage()

        size = Gf.Vec3f(100.0)
        cube = physicsUtils.add_rigid_box(stage, "/cube", size)
        cube2 = physicsUtils.add_rigid_box(stage, "/cube2", size)

        collider_pairs = get_initial_collider_pairs(stage)

        self.assertTrue(len(collider_pairs) == 1)
        self.assertTrue((str(cube.GetPrimPath()), str(cube2.GetPrimPath())) in collider_pairs)
