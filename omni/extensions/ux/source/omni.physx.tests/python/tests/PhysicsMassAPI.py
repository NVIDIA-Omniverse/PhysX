# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physx.scripts.utils import setRigidBody
from omni.physx import get_physxunittests_interface, get_physx_simulation_interface, get_physx_interface
from omni.physxtests.testBases.massTestBase import MassTestBase
from pxr import UsdGeom, UsdShade, Sdf, Gf, UsdPhysics, PhysicsSchemaTools, UsdUtils, PhysxSchema
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, PhysicsKitStageAsyncTestCase, TestCategory
import carb
import omni.usd
import omni.kit.app
import omni.kit.test
import omni.kit.commands
import omni.kit.stage_templates
from omni.physx.bindings import _physx
from omni.physxcommands import SetRigidBodyCommand
from omni.physxtests import utils


class PhysicsMassAPITestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase, MassTestBase):
    category = TestCategory.Core

    async def test_physics_MassAPI_primitives(self):
        stage = await self.mass_test_stage_setup()

        # box
        boxActorPath = "/boxActor"
        size = 10.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        position = Gf.Vec3f(0.0)
        orientation = Gf.Quatf(1.0)
        com = Gf.Vec3f(0.0, 0.0, 0.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        UsdPhysics.MassAPI.Apply(cubePrim)

        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, com=com, scale_com=True)

        # Sphere
        sphereActorPath = "/sphereActor"
        radius = 10.0
        scale = Gf.Vec3f(1.0)
        position = Gf.Vec3f(0.0, 0.0, 500.0)

        sphereGeom = UsdGeom.Sphere.Define(stage, sphereActorPath)
        spherePrim = stage.GetPrimAtPath(sphereActorPath)
        sphereGeom.CreateRadiusAttr(radius)
        sphereGeom.AddTranslateOp().Set(position)
        sphereGeom.AddOrientOp().Set(orientation)
        sphereGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(spherePrim)
        UsdPhysics.RigidBodyAPI.Apply(spherePrim)
        UsdPhysics.MassAPI.Apply(spherePrim)

        await self.check_shape_mass_properties("sphere", sphereActorPath, Gf.Vec3f(radius, 0.0, 0.0), scale, com=com, scale_com=True)

        # Capsule
        capsuleActorPath = "/capsuleActor"
        radius = 10.0
        height = 10.0
        scale = Gf.Vec3f(1.0, 1.0, 1.0)
        position = Gf.Vec3f(-500.0, 0.0, 0.0)

        capsuleGeom = UsdGeom.Capsule.Define(stage, capsuleActorPath)
        capsulePrim = stage.GetPrimAtPath(capsuleActorPath)
        capsuleGeom.CreateHeightAttr(height)
        capsuleGeom.CreateRadiusAttr(radius)
        capsuleGeom.CreateAxisAttr("Y")
        capsuleGeom.AddTranslateOp().Set(position)
        capsuleGeom.AddOrientOp().Set(orientation)
        capsuleGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(capsulePrim)
        UsdPhysics.RigidBodyAPI.Apply(capsulePrim)
        UsdPhysics.MassAPI.Apply(capsulePrim)

        await self.check_shape_mass_properties("capsule", capsuleActorPath, Gf.Vec3f(radius, height, 0.0), scale, com=com, scale_com=True)

    async def test_physics_MassAPI_density(self):
        stage = await self.mass_test_stage_setup()

        # box collisionAPI density has a priority over body density
        boxActorPath = "/boxXformActor1"
        boxCollisionPath = "/boxXformActor1/box"
        bodyXform = UsdGeom.Xform.Define(stage, boxActorPath)
        bodyPrim = stage.GetPrimAtPath(boxActorPath)

        size = 10.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        position = Gf.Vec3f(-500.0)
        orientation = Gf.Quatf(1.0)
        com = Gf.Vec3f(0.0, 0.0, 0.0)
        density = 0.002

        bodyXform.AddTranslateOp().Set(position)
        bodyXform.AddOrientOp().Set(orientation)

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
        massAPI.CreateDensityAttr(density)

        UsdPhysics.RigidBodyAPI.Apply(bodyPrim)
        massAPI = UsdPhysics.MassAPI.Apply(bodyPrim)
        massAPI.CreateDensityAttr(0.004)

        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, density=density, com=com, scale_com=True)

        density = 0.002

        # box material density if nothing else is defined
        UsdShade.Material.Define(stage, "/physicsMaterial")
        physMaterial = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath("/physicsMaterial"))
        physMaterial.CreateDensityAttr().Set(density)

        boxActorPath = "/boxXformActor2"
        boxCollisionPath = "/boxXformActor2/box"
        bodyXform = UsdGeom.Xform.Define(stage, boxActorPath)
        bodyPrim = stage.GetPrimAtPath(boxActorPath)

        size = 10.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        position = Gf.Vec3f(500.0)
        orientation = Gf.Quatf(1.0)
        com = Gf.Vec3f(0.0, 0.0, 0.0)

        bodyXform.AddTranslateOp().Set(position)
        bodyXform.AddOrientOp().Set(orientation)

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsUtils.add_physics_material_to_prim(stage, cubePrim, Sdf.Path("/physicsMaterial"))
        massAPI = UsdPhysics.MassAPI.Apply(cubePrim)

        UsdPhysics.RigidBodyAPI.Apply(bodyPrim)
        massAPI = UsdPhysics.MassAPI.Apply(bodyPrim)

        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, density=density, com=com, scale_com=True)

    async def test_physics_MassAPI_mass(self):
        stage = await self.mass_test_stage_setup()

        # box
        boxActorPath = "/boxActor"
        size = 100.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        position = Gf.Vec3f(0.0)
        orientation = Gf.Quatf(1.0)
        com = Gf.Vec3f(0.0, 0.0, 0.0)
        mass = 6.0

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
        massAPI.CreateMassAttr(mass)

        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, mass=mass, com=com, scale_com=True)

        mass = 12.0
        massAPI.GetMassAttr().Set(mass)
        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, mass=mass, com=com, scale_com=True)

        # box
        boxActorPath = "/boxXformActor0"
        boxCollisionPath = "/boxXformActor0/box"
        bodyXform = UsdGeom.Xform.Define(stage, boxActorPath)
        bodyPrim = stage.GetPrimAtPath(boxActorPath)

        size = 100.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        position = Gf.Vec3f(500.0)
        orientation = Gf.Quatf(1.0)
        com = Gf.Vec3f(0.0, 0.0, 0.0)
        mass = 6.0

        bodyXform.AddTranslateOp().Set(position)
        bodyXform.AddOrientOp().Set(orientation)

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(bodyPrim)
        massAPI = UsdPhysics.MassAPI.Apply(bodyPrim)
        massAPI.CreateMassAttr(mass)

        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, mass=mass, com=com, scale_com=True)

        # box
        boxActorPath = "/boxXformActor1"
        boxCollisionPath = "/boxXformActor1/box"
        bodyXform = UsdGeom.Xform.Define(stage, boxActorPath)
        bodyPrim = stage.GetPrimAtPath(boxActorPath)

        size = 100.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        position = Gf.Vec3f(-500.0)
        com = Gf.Vec3f(0.0, 0.0, 0.0)
        mass = 6.0

        bodyXform.AddTranslateOp().Set(position)
        bodyXform.AddOrientOp().Set(orientation)

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
        massAPI.CreateMassAttr(mass)

        UsdPhysics.RigidBodyAPI.Apply(bodyPrim)
        massAPI = UsdPhysics.MassAPI.Apply(bodyPrim)

        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, mass=mass, com=com, scale_com=True)

        # box - mass precedence, top massAPI has a priority
        boxActorPath = "/boxXformActor2"
        boxCollisionPath = "/boxXformActor2/box"
        bodyXform = UsdGeom.Xform.Define(stage, boxActorPath)
        bodyPrim = stage.GetPrimAtPath(boxActorPath)

        size = 100.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        position = Gf.Vec3f(-1500.0)
        com = Gf.Vec3f(0.0, 0.0, 0.0)
        mass = 6.0

        bodyXform.AddTranslateOp().Set(position)
        bodyXform.AddOrientOp().Set(orientation)

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
        massAPI.CreateMassAttr(16.0)

        UsdPhysics.RigidBodyAPI.Apply(bodyPrim)
        massAPI = UsdPhysics.MassAPI.Apply(bodyPrim)
        massAPI.CreateMassAttr(mass)

        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, mass=mass, com=com, scale_com=True)

    async def test_physics_MassAPI_mass_force_load(self):
        stage = await self.mass_test_stage_setup()

        # box
        boxActorPath = "/boxActor"
        size = 100.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        position = Gf.Vec3f(0.0)
        orientation = Gf.Quatf(1.0)
        com = Gf.Vec3f(0.0, 0.0, 0.0)
        mass = 6.0

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
        massAPI.CreateMassAttr(mass)
        
        get_physx_interface().force_load_physics_from_usd()
        
        actor_mass_info = get_physxunittests_interface().get_mass_information(boxActorPath)
        self.assertAlmostEqual(actor_mass_info["mass"], 6.0, delta=0.1)

        mass = 12.0
        massAPI.GetMassAttr().Set(mass)
        actor_mass_info = get_physxunittests_interface().get_mass_information(boxActorPath)
        self.assertAlmostEqual(actor_mass_info["mass"], 12.0, delta=0.1)

    async def test_physics_MassAPI_inertia(self):
        stage = await self.mass_test_stage_setup()

        # box
        boxActorPath = "/boxActor0"
        size = 100.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        position = Gf.Vec3f(0.0)
        orientation = Gf.Quatf(1.0)
        com = Gf.Vec3f(0.0, 0.0, 0.0)
        inertia = Gf.Vec3f(5.0, 6.0, 7.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
        massAPI.CreateDiagonalInertiaAttr(inertia)

        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, inertia=inertia, com=com, scale_com=True)

        # box
        boxActorPath = "/boxActor1"
        size = 100.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        position = Gf.Vec3f(1000.0)
        orientation = Gf.Quatf(1.0)
        com = Gf.Vec3f(0.0, 0.0, 0.0)
        mass = 4.0
        inertia = Gf.Vec3f(5.0, 6.0, 7.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
        massAPI.CreateDiagonalInertiaAttr(inertia)
        massAPI.CreateMassAttr(mass)

        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, mass=mass, inertia=inertia, com=com, scale_com=True)

        # box
        boxActorPath = "/boxXformActor0"
        boxCollisionPath = "/boxXformActor0/box"
        bodyXform = UsdGeom.Xform.Define(stage, boxActorPath)
        bodyPrim = stage.GetPrimAtPath(boxActorPath)

        size = 100.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        position = Gf.Vec3f(500.0)
        orientation = Gf.Quatf(1.0)
        com = Gf.Vec3f(0.0, 0.0, 0.0)
        inertia = Gf.Vec3f(5.0, 6.0, 7.0)

        bodyXform.AddTranslateOp().Set(position)
        bodyXform.AddOrientOp().Set(orientation)

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(bodyPrim)
        massAPI = UsdPhysics.MassAPI.Apply(bodyPrim)
        massAPI.CreateDiagonalInertiaAttr(inertia)

        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, inertia=inertia, com=com, scale_com=True)

        # box
        boxActorPath = "/boxXformActor1"
        boxCollisionPath = "/boxXformActor1/box"
        bodyXform = UsdGeom.Xform.Define(stage, boxActorPath)
        bodyPrim = stage.GetPrimAtPath(boxActorPath)

        size = 100.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        position = Gf.Vec3f(-500.0)
        com = Gf.Vec3f(0.0, 0.0, 0.0)
        inertia = Gf.Vec3f(5.0, 6.0, 7.0)

        bodyXform.AddTranslateOp().Set(position)
        bodyXform.AddOrientOp().Set(orientation)

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
        massAPI.CreateDiagonalInertiaAttr(inertia)

        UsdPhysics.RigidBodyAPI.Apply(bodyPrim)
        massAPI = UsdPhysics.MassAPI.Apply(bodyPrim)

        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, inertia=inertia, com=com, scale_com=True)

        # box
        boxActorPath = "/boxActor2"
        size = 100.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        position = Gf.Vec3f(0.0)
        orientation = Gf.Quatf(1.0)
        com = Gf.Vec3f(0.0, 0.0, 0.0)
        inertia = Gf.Vec3f(0.0, 0.0, 0.0)  # should do autocompute

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
        massAPI.CreateDiagonalInertiaAttr(inertia)

        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, com=com, scale_com=True)

    async def test_physics_MassAPI_com(self):
        stage = await self.mass_test_stage_setup()

        # box
        boxActorPath = "/boxActor0"
        size = 100.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        position = Gf.Vec3f(0.0)
        orientation = Gf.Quatf(1.0)
        com = Gf.Vec3f(0.0, -25.0, 0.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
        massAPI.CreateCenterOfMassAttr(com)  # scale is applied

        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, com=com, scale_com=True)

        # test runtime change
        com = Gf.Vec3f(0.0, 25.0, 0.0)
        massAPI.CreateCenterOfMassAttr(com)  # scale is applied

        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, com=com, scale_com=True)

        # box
        boxActorPath = "/boxXformActor0"
        boxCollisionPath = "/boxXformActor0/box"
        bodyXform = UsdGeom.Xform.Define(stage, boxActorPath)
        bodyPrim = stage.GetPrimAtPath(boxActorPath)

        size = 100.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        position = Gf.Vec3f(500.0)
        orientation = Gf.Quatf(1.0)
        com = Gf.Vec3f(0.0, -50.0, 0.0)

        bodyXform.AddTranslateOp().Set(position)
        bodyXform.AddOrientOp().Set(orientation)

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(bodyPrim)
        massAPI = UsdPhysics.MassAPI.Apply(bodyPrim)
        massAPI.CreateCenterOfMassAttr(com)  # scale is not applied

        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, com=com, scale_com=False)

        # box
        boxActorPath = "/boxXformActor1"
        boxCollisionPath = "/boxXformActor1/box"
        bodyXform = UsdGeom.Xform.Define(stage, boxActorPath)
        bodyPrim = stage.GetPrimAtPath(boxActorPath)

        size = 100.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        position = Gf.Vec3f(-500.0)
        com = Gf.Vec3f(0.0, -25.0, 0.0)

        bodyXform.AddTranslateOp().Set(position)
        bodyXform.AddOrientOp().Set(orientation)

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
        massAPI.CreateCenterOfMassAttr(com)  # scale is applied

        UsdPhysics.RigidBodyAPI.Apply(bodyPrim)
        massAPI = UsdPhysics.MassAPI.Apply(bodyPrim)

        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, com=com, scale_com=True)

    async def test_physics_MassAPI_compounds(self):
        stage = await self.mass_test_stage_setup()

        # box
        boxActorPath = "/boxXformActor0"
        bodyXform = UsdGeom.Xform.Define(stage, boxActorPath)
        bodyPrim = stage.GetPrimAtPath(boxActorPath)
        position = Gf.Vec3f(500.0)
        orientation = Gf.Quatf(1.0)
        bodyXform.AddTranslateOp().Set(position)
        bodyXform.AddOrientOp().Set(orientation)

        UsdPhysics.RigidBodyAPI.Apply(bodyPrim)
        UsdPhysics.MassAPI.Apply(bodyPrim)

        size = 10.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)

        boxCollisionPath = boxActorPath + "/box0"

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(Gf.Vec3f(100.0, 0.0, 0.0))
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)

        boxCollisionPath = boxActorPath + "/box1"

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(Gf.Vec3f(-100.0, 0.0, 0.0))
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)

        await self.check_mass_properties(
            boxActorPath, (6.0 * 2.0) / self.kilogramsPerUnit,
            inertia=Gf.Vec3f(1300.0 / self.kilogramsPerUnit, 121000.0 / self.kilogramsPerUnit, 120500.0 / self.kilogramsPerUnit),
            com=Gf.Vec3f(0.0, 0.0, 0.0)
        )

        # box
        boxActorPath = "/boxXformActor1"
        bodyXform = UsdGeom.Xform.Define(stage, boxActorPath)
        bodyPrim = stage.GetPrimAtPath(boxActorPath)
        position = Gf.Vec3f(-500.0)
        orientation = Gf.Quatf(1.0)

        bodyXform.AddTranslateOp().Set(position)
        bodyXform.AddOrientOp().Set(orientation)

        UsdPhysics.RigidBodyAPI.Apply(bodyPrim)
        UsdPhysics.MassAPI.Apply(bodyPrim)

        size = 10.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        boxCollisionPath = boxActorPath + "/box0"

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(Gf.Vec3f(300.0, 0.0, 0.0))
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)

        boxCollisionPath = boxActorPath + "/box1"

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(Gf.Vec3f(-100.0, 0.0, 0.0))
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)

        await self.check_mass_properties(
            boxActorPath, (6.0 * 2.0) / self.kilogramsPerUnit,
            inertia=Gf.Vec3f(1300.0 / self.kilogramsPerUnit, 481000.0 / self.kilogramsPerUnit, 480500.0 / self.kilogramsPerUnit),
            com=Gf.Vec3f(100.0, 0.0, 0.0)
        )

    async def test_physics_MassAPI_compounds_rotate(self):
        stage = await self.mass_test_stage_setup()

        # box
        boxActorPath = "/boxXformActor0"
        bodyXform = UsdGeom.Xform.Define(stage, boxActorPath)
        bodyPrim = stage.GetPrimAtPath(boxActorPath)
        position = Gf.Vec3f(0.0)
        orientation = Gf.Quatf(1.0)
        bodyXform.AddTranslateOp().Set(position)
        bodyXform.AddOrientOp().Set(orientation)

        UsdPhysics.RigidBodyAPI.Apply(bodyPrim)
        UsdPhysics.MassAPI.Apply(bodyPrim)

        size = 10.0
        scale = Gf.Vec3f(3.0, 2.0, 3.0)
        orientation = Gf.Quatf(1.0)

        boxCollisionPath = boxActorPath + "/box0"

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(Gf.Vec3f(100.0, 20.0, 10.0))
        cube0RotateOp = cubeGeom.AddRotateXYZOp()
        cube0RotateOp.Set(Gf.Vec3f(0.0, 0.0, 45.0))
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)

        boxCollisionPath = boxActorPath + "/box1"

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(Gf.Vec3f(-100.0, 20.0, 10.0))
        cube1RotateOp = cubeGeom.AddRotateXYZOp()
        cube1RotateOp.Set(Gf.Vec3f(0.0, 0.0, 45.0))
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)

        physxUT = get_physxunittests_interface()
        physxUT.update(1.0 / 60.0, 1.0 / 60.0)
        massInfo0 = physxUT.get_mass_information(boxActorPath)

        get_physx_simulation_interface().detach_stage()

        cube0RotateOp.Set(Gf.Vec3f(0.0, 90.0, 45.0))
        cube1RotateOp.Set(Gf.Vec3f(0.0, 0.0, 45.0))

        cache = UsdUtils.StageCache.Get()
        stage_id = cache.GetId(self._stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(stage_id)

        physxUT.update(1.0 / 60.0, 1.0 / 60.0)
        massInfo1 = physxUT.get_mass_information(boxActorPath)

        self.assertFloatIterableAlmostEqual(massInfo0["inertia"], massInfo1["inertia"], rel_tol=0.001)

    async def test_physics_MassAPI_compounds_trigger(self):
        stage = await self.mass_test_stage_setup()

        # box
        boxActorPath = "/boxXformActor0"
        UsdGeom.Xform.Define(stage, boxActorPath)
        bodyPrim = stage.GetPrimAtPath(boxActorPath)
        UsdPhysics.RigidBodyAPI.Apply(bodyPrim)
        UsdPhysics.MassAPI.Apply(bodyPrim)

        size = 10.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)

        boxCollisionPath = boxActorPath + "/box0"

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)

        boxCollisionPath = boxActorPath + "/box1"

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(Gf.Vec3f(-100.0, 0.0, 0.0))
        cubeGeom.AddScaleOp().Set(scale)
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        PhysxSchema.PhysxTriggerAPI.Apply(cubePrim)

        await self.check_mass_properties(
            boxActorPath, 6.0 / self.kilogramsPerUnit,
            com=Gf.Vec3f(0.0, 0.0, 0.0)
        )

    async def test_physics_MassAPI_trigger(self):
        stage = await self.mass_test_stage_setup()

        # box
        boxActorPath = "/boxXformActor0"
        UsdGeom.Xform.Define(stage, boxActorPath)
        bodyPrim = stage.GetPrimAtPath(boxActorPath)
        UsdPhysics.RigidBodyAPI.Apply(bodyPrim)
        UsdPhysics.MassAPI.Apply(bodyPrim)

        size = 10.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)

        boxCollisionPath = boxActorPath + "/box0"

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubePrim = stage.GetPrimAtPath(boxCollisionPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        PhysxSchema.PhysxTriggerAPI.Apply(cubePrim)

        await self.check_mass_properties(
            boxActorPath, 1.0,
            com=Gf.Vec3f(0.0, 0.0, 0.0)
        )

    async def test_physics_MassAPI_mass_update(self):
        stage = await self.mass_test_stage_setup()

        # create rigid body xform
        actorPath = "/actor"
        actorPrim = UsdGeom.Xform.Define(stage, actorPath)

        size = 100.0

        # create four box shapes
        for i in range(4):
            # box
            boxActorPath = actorPath + "/box" + str(i)

            cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
            cubeGeom.CreateSizeAttr(size)

        setRigidBody(actorPrim.GetPrim(), "", False)

        cube_mass = self.calculate_shape_mass("cube", Gf.Vec3f(size))

        # mass is default density 1000*volume = 1000* 1*1*1  (100 size but in cm thats 1m)
        await self.check_mass_properties(actorPath, cube_mass * 4.0)

        # add one more box
        boxActorPath = actorPath + "/box5"
        size = 100.0

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        await self.check_mass_properties(actorPath, cube_mass * 5.0)

        # remove the original 4 boxes and check the mass updates
        for i in range(4):
            # box
            boxActorPath = actorPath + "/box" + str(i)

            stage.RemovePrim(boxActorPath)
            await self.check_mass_properties(actorPath, cube_mass * (5 - i - 1))

        # remove the last box and then remove the actor, actor should not trigger update - crash would happen
        boxActorPath = actorPath + "/box5"
        stage.RemovePrim(boxActorPath)
        stage.RemovePrim(actorPath)
        get_physxunittests_interface().update(1.0 / 60.0, 1.0 / 60.0)

    async def test_physics_MassAPI_point_instancer(self):
        stage = await self.mass_test_stage_setup()
        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        # Box instanced
        boxActorPath = defaultPrimPath + "/boxActor"
        size = 1.0
        scale = Gf.Vec3f(8.0, 100.0, 25.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddScaleOp().Set(scale)
        cubeGeom.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 25.0))
        UsdPhysics.CollisionAPI.Apply(cubeGeom.GetPrim())

        geomPointInstancerPath = defaultPrimPath + "/pointinstancer"
        # indices
        meshIndices = [0, 0]
        positions = [Gf.Vec3f(0.0), Gf.Vec3f(25.0, 0.0, 0.0)]
        orientations = [Gf.Quath(1.0), Gf.Quath(1.0)]

        # Create point instancer
        shapeList = UsdGeom.PointInstancer.Define(stage, geomPointInstancerPath)
        meshList = shapeList.GetPrototypesRel()
        # add mesh reference to point instancer
        meshList.AddTarget(boxActorPath)

        shapeList.GetProtoIndicesAttr().Set(meshIndices)
        shapeList.GetPositionsAttr().Set(positions)
        shapeList.GetOrientationsAttr().Set(orientations)

        # set the rigid body on the point instancer itself
        UsdPhysics.RigidBodyAPI.Apply(shapeList.GetPrim())

        await self.check_mass_properties(
            geomPointInstancerPath, 40.0 / self.kilogramsPerUnit,
            inertia=Gf.Vec3f(35416.7 / self.kilogramsPerUnit, 8546.67 / self.kilogramsPerUnit, 39796.7 / self.kilogramsPerUnit),
            com=Gf.Vec3f(12.5, 0.0, 625.0)
        )

    async def test_physics_convex_decomposition(self):
        stage = await self.mass_test_stage_setup()

        # top level xform
        actorPath = "/xformActor"
        bodyXform = UsdGeom.Xform.Define(stage, actorPath)
        position = Gf.Vec3f(500.0)
        orientation = Gf.Quatf(1.0)

        bodyXform.AddTranslateOp().Set(position)
        bodyXform.AddOrientOp().Set(orientation)

        UsdPhysics.RigidBodyAPI.Apply(bodyXform.GetPrim())

        boxCollisionPath = actorPath + "/box0"

        cubeGeom = physicsUtils.create_mesh_cube(stage, boxCollisionPath, 10.0)
        cubeGeom.AddTranslateOp().Set(Gf.Vec3f(10.0, 0.0, 0.0))
        UsdPhysics.CollisionAPI.Apply(cubeGeom.GetPrim())
        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(cubeGeom.GetPrim())
        meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")

        boxCollisionPath = actorPath + "/box1"

        cubeGeom = physicsUtils.create_mesh_cube(stage, boxCollisionPath, 10.0)
        cubeGeom.AddTranslateOp().Set(Gf.Vec3f(-10.0, 0.0, 0.0))
        UsdPhysics.CollisionAPI.Apply(cubeGeom.GetPrim())
        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(cubeGeom.GetPrim())
        meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")

        self.step()
        self.step()
        self.step()

        await self.check_mass_properties(
            actorPath, (8.0 * 2.0) / self.kilogramsPerUnit,
            inertia=Gf.Vec3f(1066.7 / self.kilogramsPerUnit, 2666.7 / self.kilogramsPerUnit, 2666.7 / self.kilogramsPerUnit),
            com=Gf.Vec3f(0.0, 0.0, 0.0)
        )
        stage.RemovePrim(boxCollisionPath)

        self.step()
        self.step()

        await self.check_mass_properties(
            actorPath, 8.0 / self.kilogramsPerUnit,
            inertia=Gf.Vec3f(533.3 / self.kilogramsPerUnit, 533.3 / self.kilogramsPerUnit, 533.3 / self.kilogramsPerUnit),
            com=Gf.Vec3f(10.0, 0.0, 0.0)
        )

    # OM-39538
    async def test_physics_convex_decomposition_mass(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # top level xform
        actorPath = "/xformActor"
        bodyXform = UsdGeom.Xform.Define(stage, actorPath)
        position = Gf.Vec3f(500.0)
        orientation = Gf.Quatf(1.0)

        bodyXform.AddTranslateOp().Set(position)
        bodyXform.AddOrientOp().Set(orientation)

        UsdPhysics.RigidBodyAPI.Apply(bodyXform.GetPrim())

        boxCollisionPath = actorPath + "/box0"

        cubeGeom = physicsUtils.create_mesh_concave(stage, boxCollisionPath, 10.0)
        UsdPhysics.CollisionAPI.Apply(cubeGeom.GetPrim())
        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(cubeGeom.GetPrim())
        meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")
        massAPI = UsdPhysics.MassAPI.Apply(cubeGeom.GetPrim())
        massAPI.GetMassAttr().Set(5.0)

        await self.check_mass_properties(
            actorPath, 5.0)

    # OM-39538
    async def test_physics_convex_decomposition_density(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # top level xform
        actorPath = "/xformActor"
        bodyXform = UsdGeom.Xform.Define(stage, actorPath)
        position = Gf.Vec3f(500.0)
        orientation = Gf.Quatf(1.0)

        bodyXform.AddTranslateOp().Set(position)
        bodyXform.AddOrientOp().Set(orientation)

        UsdPhysics.RigidBodyAPI.Apply(bodyXform.GetPrim())

        boxCollisionPath = actorPath + "/box0"

        cubeGeom = physicsUtils.create_mesh_concave(stage, boxCollisionPath, 10.0)
        UsdPhysics.CollisionAPI.Apply(cubeGeom.GetPrim())
        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(cubeGeom.GetPrim())
        meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")

        actor_mass_info = await self.get_mass_information(actorPath)
        self.assertAlmostEqual(actor_mass_info["mass"], 7, delta=0.5)
  
    async def test_physics_convex_decomposition_material_density(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # top level xform
        actorPath = "/xformActor"
        bodyXform = UsdGeom.Xform.Define(stage, actorPath)
        position = Gf.Vec3f(500.0)
        orientation = Gf.Quatf(1.0)

        bodyXform.AddTranslateOp().Set(position)
        bodyXform.AddOrientOp().Set(orientation)

        UsdPhysics.RigidBodyAPI.Apply(bodyXform.GetPrim())

        boxCollisionPath = actorPath + "/box0"

        cubeGeom = physicsUtils.create_mesh_concave(stage, boxCollisionPath, 10.0)
        UsdPhysics.CollisionAPI.Apply(cubeGeom.GetPrim())
        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(cubeGeom.GetPrim())
        meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")

        # box material density if nothing else is defined
        UsdShade.Material.Define(stage, "/physicsMaterial")
        physMaterial = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath("/physicsMaterial"))
        physMaterial.CreateDensityAttr().Set(0.002)
        
        physicsUtils.add_physics_material_to_prim(stage, cubeGeom.GetPrim(), Sdf.Path("/physicsMaterial"))

        actor_mass_info = await self.get_mass_information(actorPath)
        self.assertAlmostEqual(actor_mass_info["mass"], 14, delta=0.5)
  
        physMaterial.CreateDensityAttr().Set(0.004)        
        self.step()
        
        actor_mass_info = await self.get_mass_information(actorPath)
        self.assertAlmostEqual(actor_mass_info["mass"], 28, delta=0.5)
  
    async def test_physics_box_material_density(self):
        stage = await self.new_stage()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # top level xform
        actorPath = "/xformActor"
        bodyXform = UsdGeom.Xform.Define(stage, actorPath)
        position = Gf.Vec3f(500.0)
        orientation = Gf.Quatf(1.0)

        bodyXform.AddTranslateOp().Set(position)
        bodyXform.AddOrientOp().Set(orientation)

        UsdPhysics.RigidBodyAPI.Apply(bodyXform.GetPrim())

        boxCollisionPath = actorPath + "/box0"

        cubeGeom = UsdGeom.Cube.Define(stage, boxCollisionPath)
        cubeGeom.CreateSizeAttr(10)
        UsdPhysics.CollisionAPI.Apply(cubeGeom.GetPrim())

        # box material density if nothing else is defined
        UsdShade.Material.Define(stage, "/physicsMaterial")
        physMaterial = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath("/physicsMaterial"))
        physMaterial.CreateDensityAttr().Set(0.002)
        
        physicsUtils.add_physics_material_to_prim(stage, cubeGeom.GetPrim(), Sdf.Path("/physicsMaterial"))

        actor_mass_info = await self.get_mass_information(actorPath)
        self.assertAlmostEqual(actor_mass_info["mass"], 2, delta=0.5)
  
        physMaterial.CreateDensityAttr().Set(0.004)        
        self.step()
        
        actor_mass_info = await self.get_mass_information(actorPath)
        self.assertAlmostEqual(actor_mass_info["mass"], 4, delta=0.5)
  
    async def test_physics_MassAPI_mass_no_inertia_explicit_mass(self):
        stage = await self.mass_test_stage_setup()

        # box
        boxActorPath = "/boxXformActor0"
        bodyXform = UsdGeom.Xform.Define(stage, boxActorPath)

        mass = 1.0
        com = Gf.Vec3f(0.0, 0.0, 0.0)

        UsdPhysics.RigidBodyAPI.Apply(bodyXform.GetPrim())
        massAPI = UsdPhysics.MassAPI.Apply(bodyXform.GetPrim())
        massAPI.CreateMassAttr().Set(mass)

        # when mass is defined but not inertia, default to a sphere with radius 0.1
        await self.check_shape_mass_properties("sphere", boxActorPath, Gf.Vec3f(0.1 / self.metersPerUnit), Gf.Vec3f(1.0), mass=mass, com=com, scale_com=True)

    async def test_physics_MassAPI_mass_no_inertia_no_mass(self):
        stage = await self.mass_test_stage_setup()

        # box
        boxActorPath = "/boxXformActor0"
        bodyXform = UsdGeom.Xform.Define(stage, boxActorPath)

        mass = 1.0
        com = Gf.Vec3f(0.0, 0.0, 0.0)

        UsdPhysics.RigidBodyAPI.Apply(bodyXform.GetPrim())

        # when mass is not defined and not inertia, default to a sphere with radius 0.1, mass defaults to 1.0
        await self.check_shape_mass_properties("sphere", boxActorPath, Gf.Vec3f(0.1 / self.metersPerUnit), Gf.Vec3f(1.0), mass=mass, com=com, scale_com=True)

    async def test_physics_MassAPI_kinematic_mass_no_inertia(self):
        stage = await self.mass_test_stage_setup()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # box
        boxActorPath = "/boxXformActor0"
        bodyXform = UsdGeom.Xform.Define(stage, boxActorPath)

        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(bodyXform.GetPrim())
        rigidBodyAPI.CreateKinematicEnabledAttr().Set(True)

        await self.check_mass_properties(
            boxActorPath, 1.0
        )

    async def test_physics_MassAPI_kinematic_mass(self):
        stage = await self.mass_test_stage_setup()

        UsdPhysics.Scene.Define(stage, "/physicsScene")

        # box
        boxActorPath = "/boxActor"
        size = 10.0
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        position = Gf.Vec3f(0.0)
        orientation = Gf.Quatf(1.0)
        com = Gf.Vec3f(0.0, 0.0, 0.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        physicsAPI.CreateKinematicEnabledAttr().Set(True)
        UsdPhysics.MassAPI.Apply(cubePrim)

        await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, com=com, scale_com=True)

    async def test_physics_MassAPI_unit_scales(self):
        stage = await self.mass_test_stage_setup()
        lastCalculatedMass = -1.0

        for j in range(2):
            if j == 0:
                kgpu = 1000.0
            else:
                kgpu = 1.0
            for i in range(2):
                if i == 0:
                    mpu = 0.01
                else:
                    mpu = 1.0

                self.set_units(stage, mpu, kgpu)

                ind = j * 2 + i

                boxActorPath = "/boxActor" + str(ind)
                size = 10.0 / self.metersPerUnit
                scale = Gf.Vec3f(1.0, 2.0, 3.0)
                position = Gf.Vec3f(i * 200.0, j * 200.0, 0.0)
                orientation = Gf.Quatf(1.0)
                com = Gf.Vec3f(0.0, 0.0, 0.0)

                cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
                cubePrim = stage.GetPrimAtPath(boxActorPath)
                cubeGeom.CreateSizeAttr(size)
                cubeGeom.AddTranslateOp().Set(position)
                cubeGeom.AddOrientOp().Set(orientation)
                cubeGeom.AddScaleOp().Set(scale)

                UsdPhysics.CollisionAPI.Apply(cubePrim)
                UsdPhysics.RigidBodyAPI.Apply(cubePrim)
                UsdPhysics.MassAPI.Apply(cubePrim)

                calculatedMass = await self.check_shape_mass_properties("cube", boxActorPath, Gf.Vec3f(size), scale, com=com, scale_com=True)
                calculatedMass = calculatedMass * kgpu  # normalize mass by kilogramsPerUnit for comparison

                if ind > 0:
                    self.assertAlmostEqual(calculatedMass, lastCalculatedMass, delta=0.01)
                lastCalculatedMass = calculatedMass

    async def test_physics_get_mass_space_inertia(self):
        epsilon = 1e-5
        diagonal = Gf.Vec3f(10.0, 20.0, 30.0)
        matrix = Gf.Matrix3f(diagonal)

        mass_diagonal, principal_axes = PhysicsSchemaTools.getMassSpaceInertia(matrix)

        Gf.IsClose(mass_diagonal, diagonal, epsilon)
        Gf.IsClose(principal_axes.GetImaginary(), Gf.Vec3f(1.0, 0.0, 0.0), epsilon)
        self.assertTrue(abs(principal_axes.GetReal() - 1.0) < epsilon)


async def base_setup():
    await utils.new_stage_setup()
    stage = omni.usd.get_context().get_stage()
    omni.kit.commands.execute("AddPhysicsScene", stage=stage, path='/physicsScene')
    return stage


class PhysicsMassAPIKitTest(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core

    async def check_mass_api(self, prim, mass, inertia=None, com=None):
        physxUT = get_physxunittests_interface()
        physxUT.update(1.0 / 60.0, 1.0 / 60.0)
        massInfo = physxUT.get_mass_information(prim)

        epsilon = 0.1

        self.assertTrue(abs(massInfo["mass"] - mass) < epsilon)
        if inertia is not None:
            self.assertTrue(abs(massInfo["inertia"].x - inertia.x) < epsilon)
            self.assertTrue(abs(massInfo["inertia"].y - inertia.y) < epsilon)
            self.assertTrue(abs(massInfo["inertia"].z - inertia.z) < epsilon)
        if com is not None:
            self.assertTrue(abs(massInfo["com"].x - com.x) < epsilon)
            self.assertTrue(abs(massInfo["com"].y - com.y) < epsilon)
            self.assertTrue(abs(massInfo["com"].z - com.z) < epsilon)


    async def test_physics_MassAPI_RB_selection(self):
        stage = await base_setup()

        boxActorPath = "/boxActor"
        size = 10.0
        position = Gf.Vec3f(0.0)
        orientation = Gf.Quatf(1.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        scale = Gf.Vec3f(1.0, 2.0, 3.0)
        cubeGeom.AddScaleOp().Set(scale)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)        
        massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
        density = 2000.0 / 100.0 ** 3.0  # in kg/CM3
        massAPI.CreateDensityAttr(density)  
        # mass needs to be 10 * 20 * 30 * density
        mass = size ** 3.0 * scale[0] * scale[1] * scale[2] * density
        await self.check_mass_api(boxActorPath, mass)

        # select the prim:
        omni.kit.commands.execute("SelectPrims", old_selected_paths=[], new_selected_paths=[boxActorPath], expand_in_stage=False)

        # Kit update:
        await omni.kit.app.get_app().next_update_async()

        # step physics:
        physxUTI = get_physxunittests_interface()
        physxUTI.update(1.0 / 60.0, 1.0 / 60.0)

        # ensure mass still the same:
        await self.check_mass_api(boxActorPath, mass)


    async def test_physics_MassAPI_RB_multi_selection(self):
        stage = await base_setup()

        boxActorPath0 = "/boxActor0"
        size = 100.0
        position = Gf.Vec3f(0.0)
        orientation = Gf.Quatf(1.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath0)        
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)

        boxActorPath1 = "/boxActor1"
        position = Gf.Vec3f(100.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath1)        
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)

        # select the prim:
        omni.kit.commands.execute("SelectPrims", old_selected_paths=[], new_selected_paths=[boxActorPath0, boxActorPath1], expand_in_stage=False)
        SetRigidBodyCommand.execute(boxActorPath0, "convexHull", False)
        SetRigidBodyCommand.execute(boxActorPath1, "convexHull", False)

        # Kit update:
        await omni.kit.app.get_app().next_update_async()

        # step physics:
        physxUTI = get_physxunittests_interface()
        physxUTI.update(1.0 / 60.0, 1.0 / 60.0)

        # ensure mass still the same:
        await self.check_mass_api(boxActorPath0, 1000.0)
        await self.check_mass_api(boxActorPath1, 1000.0)
