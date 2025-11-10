# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physxcommands import SetRigidBodyCommand
from omni.physx.scripts import physicsUtils
from omni.physxtests import utils
from omni.physxtests.utils import rigidbody
from omni.physx.bindings._physx import ErrorEvent
from omni.physx import get_physx_simulation_interface, get_physx_interface
from pxr import Gf, UsdPhysics, UsdUtils, UsdGeom, Sdf, PhysicsSchemaTools
import unittest
import omni.timeline
import omni.kit
from omni.physxtests.utils.physicsBase import TestCategory
import carb

class PhysxSimulationInterfaceTestAsyncRB(rigidbody.AsyncTestCase):
    category = TestCategory.Kit

    async def test_physx_simulate(self):
        stage = await utils.new_stage_setup()

        simulateI = get_physx_simulation_interface()

        cubePrim = physicsUtils.add_cube(stage, "/cubeActor", 100.0, Gf.Vec3f(0.0, 0.5, 0.0))
        SetRigidBodyCommand.execute(cubePrim.GetPath(), "", False)

        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, -1.0, 0.0))
        scene.CreateGravityMagnitudeAttr().Set(981)

        stageId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        # attach to the simulation interface
        simulateI.attach_stage(stageId)

        # simulate through the simulation interface
        for i in range(100):
            simulateI.simulate(1.0/60.0, i * 1.0/60.0)
            simulateI.fetch_results()

        self.assertTrue(physicsUtils.get_translation(cubePrim)[1] < 0.0)

        simulateI.detach_stage()

    async def test_physx_is_running(self):
        stage = await utils.new_stage_setup()

        simulateI = get_physx_simulation_interface()

        cubePrim = physicsUtils.add_cube(stage, "/cubeActor", 100.0, Gf.Vec3f(0.0, 0.5, 0.0))
        SetRigidBodyCommand.execute(cubePrim.GetPath(), "", False)

        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, -1.0, 0.0))
        scene.CreateGravityMagnitudeAttr().Set(981)

        stageId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        self.assertTrue(not get_physx_interface().is_running())

        # attach to the simulation interface
        simulateI.attach_stage(stageId)

        self.assertTrue(get_physx_interface().is_running())

        # simulate through the simulation interface
        for i in range(10):
            simulateI.simulate(1.0/60.0, i * 1.0/60.0)
            simulateI.fetch_results()        

        self.assertTrue(get_physx_interface().is_running())
        
        simulateI.detach_stage()

        self.assertTrue(not get_physx_interface().is_running())

    async def test_physx_re_attach(self):
        stage = await utils.new_stage_setup()

        simulateI = get_physx_simulation_interface()

        cubePrim = physicsUtils.add_cube(stage, "/cubeActor", 100.0, Gf.Vec3f(0.0, 0.5, 0.0))
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.Scene.Define(stage, "/physicsScene")

        stageId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        # attach to the simulation interface
        simulateI.attach_stage(stageId)

        # simulate through the simulation interface
        for i in range(5):
            simulateI.simulate(1.0/60.0, i * 1.0/60.0)
            simulateI.fetch_results()

        self.assertTrue(physicsUtils.get_translation(cubePrim)[1] < 0.0)

        message = 'Stage ' + str(stageId) + ' already attached.'
        with utils.ExpectMessage(self, message):
            simulateI.attach_stage(stageId)        

        simulateI.detach_stage()

    async def test_physx_get_stage_attach(self):
        stage = await utils.new_stage_setup()

        simulateI = get_physx_simulation_interface()

        cubePrim = physicsUtils.add_cube(stage, "/cubeActor", 100.0, Gf.Vec3f(0.0, 0.5, 0.0))
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.Scene.Define(stage, "/physicsScene")

        stageId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        # attach to the simulation interface
        simulateI.attach_stage(stageId)

        attached_stage = simulateI.get_attached_stage()
        self.assertTrue(attached_stage == stageId)

        simulateI.detach_stage()        

        attached_stage = simulateI.get_attached_stage()
        self.assertTrue(attached_stage == 0)


    async def test_physx_simulate_block_stage_update(self):
        stage = await utils.new_stage_setup()

        simulateI = get_physx_simulation_interface()

        cubePrim = physicsUtils.add_cube(stage, "/cubeActor", 100.0, Gf.Vec3f(0.0, 0.5, 0.0))
        SetRigidBodyCommand.execute(cubePrim.GetPath(), "", False)

        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, -1.0, 0.0))
        scene.CreateGravityMagnitudeAttr().Set(981)

        stageId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        # attach IPhysxSimulation interface, its expected to be updated that way        
        simulateI.attach_stage(stageId)

        # step through play, this should not move the box
        omni.timeline.get_timeline_interface().play()
        for i in range(20):
            await omni.kit.app.get_app().next_update_async()

        self.assertTrue(physicsUtils.get_translation(cubePrim)[1] > 0.0)
        omni.timeline.get_timeline_interface().stop()

        # detach the simulation interface, this should allow back the update through play
        simulateI.detach_stage()        

        omni.timeline.get_timeline_interface().play()
        for i in range(20):
            await omni.kit.app.get_app().next_update_async()

        self.assertTrue(physicsUtils.get_translation(cubePrim)[1] < 0.0)
        omni.timeline.get_timeline_interface().stop()


class PhysxSimulationInterfaceTestPointInstancer(rigidbody.AsyncTestCase):
    category = TestCategory.Kit

    async def test_physx_simulate_point_instancers(self):
        stage = await utils.new_stage_setup()
        utils.physics_scene_setup(stage)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        geomPointInstancerPath = defaultPrimPath + "/pointinstancer"

        physicsUtils.add_ground_plane(stage, defaultPrimPath + "groundPlane", "Y", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        cubePrim1 = physicsUtils.add_cube(stage, geomPointInstancerPath + "/boxActor1", 100.0)
        cubePrim2 = physicsUtils.add_cube(stage, geomPointInstancerPath + "/boxActor2", 100.0)
        SetRigidBodyCommand.execute(cubePrim1.GetPath(), "", False)
        SetRigidBodyCommand.execute(cubePrim2.GetPath(), "", False)

        # indices
        meshIndices = [0, 0, 1, 1]
        positions = [Gf.Vec3f(-150, 50.0, 0.0), Gf.Vec3f(150.0, 50.0, 0.0), Gf.Vec3f(0.0, 50.0, -150.0), Gf.Vec3f(0.0, 50.0, 150.0)]
        orientations = [Gf.Quath(1.0, 0.0, 0.0, 0.0), Gf.Quath(1.0, 0.0, 0.0, 0.0), Gf.Quath(1.0, 0.0, 0.0, 0.0), Gf.Quath(1.0, 0.0, 0.0, 0.0)]
        linearVelocities = [Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(0.0, 0.0, 0.0)]
        angularVelocities = [Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(0.0, 0.0, 0.0)]

        shapeList = UsdGeom.PointInstancer.Define(stage, Sdf.Path(geomPointInstancerPath))
        meshList = shapeList.GetPrototypesRel()

        # add mesh reference to point instancer
        meshList.AddTarget(cubePrim1.GetPath())
        meshList.AddTarget(cubePrim2.GetPath())

        shapeList.GetProtoIndicesAttr().Set(meshIndices)
        shapeList.GetPositionsAttr().Set(positions)
        shapeList.GetOrientationsAttr().Set(orientations)
        shapeList.GetVelocitiesAttr().Set(linearVelocities)
        shapeList.GetAngularVelocitiesAttr().Set(angularVelocities)

        simulateI = get_physx_simulation_interface()
        stageId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        simulateI.attach_stage(stageId)
        point_instancer_path_encoded = PhysicsSchemaTools.sdfPathToInt(geomPointInstancerPath)

        impulse = carb.Float3(0.0, 100000.0, 0.0)

        step = 0
        for _ in range(5):
            simulateI.apply_force_at_pos_instanced(stageId, point_instancer_path_encoded, impulse, carb.Float3(*positions[0]), "Impulse", 0)
            simulateI.apply_force_at_pos_instanced(stageId, point_instancer_path_encoded, impulse, carb.Float3(*positions[3]), "Impulse", 3)
            simulateI.simulate(1.0 / 60.0, step * 1.0 / 60.0)
            simulateI.fetch_results()
            step += 1

        newPositions = shapeList.GetPositionsAttr().Get()
        self.assertNotAlmostEqual(positions[0][1], newPositions[0][1], delta=0.01)
        self.assertAlmostEqual(positions[1][1], newPositions[1][1], delta=0.01)
        self.assertAlmostEqual(positions[2][1], newPositions[2][1], delta=0.01)
        self.assertNotAlmostEqual(positions[3][1], newPositions[3][1], delta=0.01)

        newLinearVelocities = shapeList.GetVelocitiesAttr().Get()
        self.assertNotAlmostEqual(linearVelocities[0][1], newLinearVelocities[0][1], delta=0.01)
        self.assertAlmostEqual(linearVelocities[1][1], newLinearVelocities[1][1], delta=0.1)
        self.assertAlmostEqual(linearVelocities[2][1], newLinearVelocities[2][1], delta=0.1)
        self.assertNotAlmostEqual(linearVelocities[3][1], newLinearVelocities[3][1], delta=0.01)

        torque = carb.Float3(0.0, 100000000.0, 0.0)

        for _ in range(5):
            simulateI.apply_torque_instanced(stageId, point_instancer_path_encoded, torque, 0)
            simulateI.apply_torque_instanced(stageId, point_instancer_path_encoded, torque, 3)
            simulateI.simulate(1.0 / 60.0, step * 1.0 / 60.0)
            simulateI.fetch_results()
            step += 1

        newOrientations = shapeList.GetOrientationsAttr().Get()
        self.assertNotAlmostEqual(orientations[0].GetReal(), newOrientations[0].GetReal(), delta=0.001)
        self.assertAlmostEqual(orientations[1].GetReal(), newOrientations[1].GetReal(), delta=0.001)
        self.assertAlmostEqual(orientations[2].GetReal(), newOrientations[2].GetReal(), delta=0.001)
        self.assertNotAlmostEqual(orientations[3].GetReal(), newOrientations[3].GetReal(), delta=0.001)

        newAngularVelocities = shapeList.GetAngularVelocitiesAttr().Get()
        self.assertNotAlmostEqual(angularVelocities[0][1], newAngularVelocities[0][1], delta=0.01)
        self.assertAlmostEqual(angularVelocities[1][1], newAngularVelocities[1][1], delta=0.01)
        self.assertAlmostEqual(angularVelocities[2][1], newAngularVelocities[2][1], delta=0.01)
        self.assertNotAlmostEqual(angularVelocities[3][1], newAngularVelocities[3][1], delta=0.01)

        # Omitting the instance parameter should put all to sleep.
        simulateI.put_to_sleep_instanced(stageId, point_instancer_path_encoded)

        # Wake up two.
        simulateI.wake_up_instanced(stageId, point_instancer_path_encoded, 1)
        simulateI.wake_up_instanced(stageId, point_instancer_path_encoded, 2)

        self.assertTrue(simulateI.is_sleeping_instanced(stageId, point_instancer_path_encoded, 0))
        self.assertFalse(simulateI.is_sleeping_instanced(stageId, point_instancer_path_encoded, 1))
        self.assertFalse(simulateI.is_sleeping_instanced(stageId, point_instancer_path_encoded, 2))
        self.assertTrue(simulateI.is_sleeping_instanced(stageId, point_instancer_path_encoded, 3))

        simulateI.detach_stage()
