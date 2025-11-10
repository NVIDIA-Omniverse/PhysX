# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physxtests import utils
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics, PhysicsSchemaTools
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory


use_tri_mesh_as_ground_plane = False

def joint_transform(bodyPos, bodyRot, p):
    trRotInv = Gf.Rotation(bodyRot).GetInverse()
    return trRotInv.TransformDir(p - bodyPos)


def create_mesh_square_axis(stage, path, axis, halfSize):
    if axis == "X":
        points = [
            Gf.Vec3f(0.0, -halfSize, -halfSize),
            Gf.Vec3f(0.0, halfSize, -halfSize),
            Gf.Vec3f(0.0, halfSize, halfSize),
            Gf.Vec3f(0.0, -halfSize, halfSize),
        ]
        normals = [Gf.Vec3f(1, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(1, 0, 0)]
        indices = [0, 1, 2, 3]
        vertexCounts = [4]

        # Create the mesh
        return PhysicsSchemaTools.createMesh(stage, path, points, normals, indices, vertexCounts)
    elif axis == "Y":
        points = [
            Gf.Vec3f(-halfSize, 0.0, -halfSize),
            Gf.Vec3f(halfSize, 0.0, -halfSize),
            Gf.Vec3f(halfSize, 0.0, halfSize),
            Gf.Vec3f(-halfSize, 0.0, halfSize),
        ]
        normals = [Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 1, 0)]
        indices = [0, 1, 2, 3]
        vertexCounts = [4]

        # Create the mesh
        return PhysicsSchemaTools.createMesh(stage, path, points, normals, indices, vertexCounts)

    points = [
        Gf.Vec3f(-halfSize, -halfSize, 0.0),
        Gf.Vec3f(halfSize, -halfSize, 0.0),
        Gf.Vec3f(halfSize, halfSize, 0.0),
        Gf.Vec3f(-halfSize, halfSize, 0.0),
    ]
    normals = [Gf.Vec3f(0, 0, 1), Gf.Vec3f(0, 0, 1), Gf.Vec3f(0, 0, 1), Gf.Vec3f(0, 0, 1)]
    indices = [0, 1, 2, 3]
    vertexCounts = [4]

    # Create the mesh
    mesh = PhysicsSchemaTools.createMesh(stage, path, points, normals, indices, vertexCounts)

    # text coord
    texCoords = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying)
    texCoords.Set([(0, 0), (1, 0), (1, 1), (0, 1)])

    return mesh


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
        entityPlane = create_mesh_square_axis(stage, Sdf.Path("/trimesh"), "Z", 1500.0)
        entityPlane.CreateDisplayColorAttr().Set([shapeColor])
        prim = stage.GetPrimAtPath(Sdf.Path("/trimesh"))
        collisionAPI = UsdPhysics.CollisionAPI.Apply(prim)
        collisionAPI.CreateApproximationShapeAttr("none")
    else:
        # Plane
        PhysicsSchemaTools.addGroundPlane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))


def rigid_box_setup(stage):
    size = 50.0
    density = 1000.0
    color = Gf.Vec3f(71.0 / 255.0, 165.0 / 255.0, 1.0)
    orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    linVelocity = Gf.Vec3f(0.0)
    angularVelocity = Gf.Vec3f(0.0)

    position = Gf.Vec3f(50.0, 50.0, 200.0)
    cubePath = "/boxActor"

    PhysicsSchemaTools.addRigidBox(
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

    PhysicsSchemaTools.addRigidSphere(
        stage, spherePath, radius, position, orientation, color, density, linVelocity, angularVelocity
    )

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

    PhysicsSchemaTools.addRigidCapsule(
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

    PhysicsSchemaTools.addRigidCylinder(
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

    PhysicsSchemaTools.addRigidCone(
        stage, conePath, radius, height, "Z", position, orientation, color, density, linVelocity, angularVelocity
    )


class PhysicsSchemaToolsTestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    async def test_physics_schema_tools(self):
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
