# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physx import get_physx_simulation_interface, get_physx_interface
from omni.physxtests import utils
from pxr import Gf, UsdGeom, UsdPhysics, UsdUtils
import os


class PointInstancerTestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    def base_setup(self, stage, config: str = "dynamic", static_kinematic=False):
        self.stage = stage
        self.default_prim_path = str(stage.GetDefaultPrim().GetPath())
        self.boxes = []
        self.instancer = None
        self.scene = UsdPhysics.Scene.Define(stage, self.default_prim_path + "/physicsScene")
        instancerPath = self.default_prim_path + "/pointinstancer"
        self.create_box_point_instancer(instancerPath, config=config, static_kinematic=static_kinematic)

    def create_proto_box(self, path, dynamic=True, color=Gf.Vec3f(71.0 / 255.0, 165.0 / 255.0, 1.0), size=25.0):
        self.assertFalse(self.stage.GetPrimAtPath(path))
        # Box instanced
        cubeGeom = UsdGeom.Cube.Define(self.stage, path)
        cubePrim = self.stage.GetPrimAtPath(path)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddScaleOp().Set(Gf.Vec3f(1.0))
        cubeGeom.CreateDisplayColorAttr().Set([color])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        if dynamic:
            UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        self.boxes.append(cubeGeom)

    def create_box_point_instancer(self, path, config: str = "dynamic", static_kinematic=False):
        """
        Creates a point instancer with two instances that are dynamic, static or mixed based on the config string
        config can be: 'dynamic', 'static', or 'mixed' - mixed means one dynamic one static instance
        """
        boxActorPath = path + "/boxActor"
        boxActorPathMixed = None
        if config == "static":
            if static_kinematic:
                self.create_proto_box(boxActorPath, dynamic=True)
                physicsAPI = UsdPhysics.RigidBodyAPI.Apply(self.boxes[0].GetPrim())
                physicsAPI.CreateKinematicEnabledAttr().Set(True)
            else:
                self.create_proto_box(boxActorPath, dynamic=False)
        else:  # dynamic or mixed create a dynamic box
            self.create_proto_box(boxActorPath, dynamic=True)

        if config == "mixed":
            boxActorPathMixed = path + "/boxActorMixed"
            self.create_proto_box(boxActorPathMixed, dynamic=False)

        # indices
        meshIndices = [0, 0]
        self.positions = [Gf.Vec3f(-125.0, 0.0, 500.0), Gf.Vec3f(125.0, 0.0, 500.0)]
        self.orientations = [Gf.Quath(1.0, 0.0, 0.0, 0.0), Gf.Quath(0.8660254, 0.0, 0.5, 0.0)]
        linearVelocities = [Gf.Vec3f(0.0, 5.0, 0.0), Gf.Vec3f(0.0, 0.0, 9.0)]
        angularVelocities = [Gf.Vec3f(0.0, 10.0, 0.0), Gf.Vec3f(10.0, 0.0, 0.0)]

        # Create point instancer
        instancer = UsdGeom.PointInstancer.Define(self.stage, path)
        meshList = instancer.GetPrototypesRel()
        # add mesh reference to point instancer
        meshList.AddTarget(boxActorPath)
        if config == "mixed":
            meshList.AddTarget(boxActorPathMixed)
            meshIndices[1] = 1

        instancer.GetProtoIndicesAttr().Set(meshIndices)
        instancer.GetPositionsAttr().Set(self.positions)
        instancer.GetOrientationsAttr().Set(self.orientations)
        instancer.GetVelocitiesAttr().Set(linearVelocities)
        instancer.GetAngularVelocitiesAttr().Set(angularVelocities)

        self.instancer = instancer

    # OM-26384
    async def test_physics_point_instancer_static(self):
        stage = await self.new_stage()
        self.base_setup(stage, config="static")

        self.step()

        utils.check_stats(self, {"numBoxShapes": 2, "numStaticRigids": 2})

        get_physx_simulation_interface().detach_stage()

        UsdPhysics.RigidBodyAPI.Apply(self.boxes[0].GetPrim())

        cache = UsdUtils.StageCache.Get()
        stage_id = cache.GetId(self._stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(stage_id)

        self.step()

        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2})

    # OM-26384
    async def test_physics_point_instancer_mixed_dynamic_static(self):
        stage = await self.new_stage()

        self.base_setup(stage, config="mixed")

        self.step()

        utils.check_stats(self, {"numBoxShapes": 2, "numStaticRigids": 1, "numDynamicRigids": 1})

    async def test_physics_point_instancer_mixed_dynamic_kinematic(self):
        stage = await self.new_stage()

        self.base_setup(stage, config="mixed", static_kinematic=True)

        self.step()

        utils.check_stats(self, {"numBoxShapes": 2, "numStaticRigids": 1, "numDynamicRigids": 1})

    async def test_physics_point_instancer_deactivation(self):
        stage = await self.new_stage()

        self.base_setup(stage)

        # indices
        meshIndices = [0, 0, 0, 0]
        positions = [Gf.Vec3f(200.0), Gf.Vec3f(400.0), Gf.Vec3f(600.0), Gf.Vec3f(800.0)]
        orientations = [Gf.Quath(1.0), Gf.Quath(1.0), Gf.Quath(1.0), Gf.Quath(1.0)]

        self.instancer.GetProtoIndicesAttr().Set(meshIndices)
        self.instancer.GetPositionsAttr().Set(positions)
        self.instancer.GetOrientationsAttr().Set(orientations)

        self.instancer.DeactivateId(2)

        self.step()

        utils.check_stats(self, {"numBoxShapes": 3, "numDynamicRigids": 3})

        self.instancer.DeactivateId(3)

        self.step()

        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2})

        self.instancer.ActivateId(2)

        self.step()

        utils.check_stats(self, {"numBoxShapes": 3, "numDynamicRigids": 3})

    async def test_physics_point_instancer_collisions(self):
        stage = await self.new_stage()
        self.base_setup(stage, config="static")

        # indices
        meshIndices = [0, 0, 0, 0]
        positions = [Gf.Vec3f(200.0), Gf.Vec3f(400.0), Gf.Vec3f(600.0), Gf.Vec3f(800.0)]
        orientations = [Gf.Quath(1.0), Gf.Quath(1.0), Gf.Quath(1.0), Gf.Quath(1.0)]

        self.instancer.GetProtoIndicesAttr().Set(meshIndices)
        self.instancer.GetPositionsAttr().Set(positions)
        self.instancer.GetOrientationsAttr().Set(orientations)

        # set the rigid body on the point instancer itself
        UsdPhysics.RigidBodyAPI.Apply(self.instancer.GetPrim())

        self.step()

        utils.check_stats(self, {"numBoxShapes": 4, "numDynamicRigids": 1})

        self.instancer.DeactivateId(3)

        self.step()

        utils.check_stats(self, {"numBoxShapes": 3, "numDynamicRigids": 1})

        self.instancer.ActivateId(3)

        self.step()

        utils.check_stats(self, {"numBoxShapes": 4, "numDynamicRigids": 1})

    async def test_physics_point_instancer_initial_velocities(self):
        stage = await self.new_stage()

        self.base_setup(stage, config="dynamic")

        # indices
        meshIndices = [0, 0, 0, 0]
        positions = [Gf.Vec3f(200.0), Gf.Vec3f(400.0), Gf.Vec3f(600.0), Gf.Vec3f(800.0)]
        orientations = [Gf.Quath(1.0), Gf.Quath(1.0), Gf.Quath(1.0), Gf.Quath(1.0)]
        velocities = [Gf.Vec3f(1.0), Gf.Vec3f(2.0), Gf.Vec3f(3.0), Gf.Vec3f(4.0)]

        self.instancer.GetProtoIndicesAttr().Set(meshIndices)
        self.instancer.GetPositionsAttr().Set(positions)
        self.instancer.GetOrientationsAttr().Set(orientations)
        self.instancer.GetVelocitiesAttr().Set(velocities)
        self.instancer.GetAngularVelocitiesAttr().Set(velocities)

        get_physx_interface().start_simulation()
        self.step()

        utils.check_stats(self, {"numBoxShapes": 4, "numDynamicRigids": 4})

        get_physx_interface().reset_simulation()

        linearVelocities = []
        angularVelocities = []
        linearVelocities = self.instancer.GetVelocitiesAttr().Get()
        angularVelocities = self.instancer.GetAngularVelocitiesAttr().Get()

        self.assertTrue(len(velocities) == len(linearVelocities))
        self.assertTrue(len(velocities) == len(angularVelocities))

        for (vel, linVel, angVel) in zip(velocities, linearVelocities, angularVelocities):
            for i in range(3):
                self.assertTrue(vel[i] == linVel[i])
                self.assertTrue(vel[i] == angVel[i])

    async def _run_reset_transform_test(self, config, use_kinematic_static=False):
        stage = await self.new_stage()

        self.base_setup(stage, config=config, static_kinematic=use_kinematic_static)

        get_physx_interface().start_simulation()
        for _ in range(5):
            self.step()

        # move the positions and rotations
        # the dynamic actors will reset to initial
        new_pos = []
        new_orient = []
        for pos in self.positions:
            new_pos.append(pos + Gf.Vec3f(1.2, 1.3, 1.5))
        for orient in self.orientations:
            new_orient.append(orient * Gf.Quath(0.5, 0.5, 0.5, 0.5))

        self.instancer.GetPositionsAttr().Set(new_pos)
        self.instancer.GetOrientationsAttr().Set(new_orient)

        for _ in range(5):
            self.step()

        get_physx_interface().reset_simulation()

        should_reset = [True] * 2

        if config == "static":
            should_reset = [False] * 2
        if config == "mixed":
            should_reset[1] = False

        reset_pos = self.instancer.GetPositionsAttr().Get()
        reset_orient = self.instancer.GetOrientationsAttr().Get()

        for i in range(len(reset_pos)):
            if should_reset[i]:
                self.assertEqual(reset_pos[i], self.positions[i])
                self.assertEqual(reset_orient[i], self.orientations[i])
            else:
                self.assertNotEqual(reset_pos[i], self.positions[i])
                self.assertEqual(reset_orient[i], self.orientations[i])

    async def test_physics_point_instancer_reset_transformation_dynamic(self):
        await self._run_reset_transform_test(config="dynamic")
        # pointinstancer will reset nondynamics, so the following two test configs
        # are currently not supported:
        # await self._run_reset_transform_test(config="mixed")
        # await self._run_reset_transform_test(config="mixed", use_kinematic_static=True)

    async def test_physics_point_instancer_invalid_index(self):
        stage = await self.new_stage()

        self.base_setup(stage, config="static")

        # indices
        meshIndices = [0, 0, 5, 5]
        positions = [Gf.Vec3f(-125.0, 0.0, 500.0), Gf.Vec3f(125.0, 0.0, 500.0)]
        orientations = [Gf.Quath(1.0, 0.0, 0.0, 0.0), Gf.Quath(0.8660254, 0.0, 0.5, 0.0)]
        linearVelocities = [Gf.Vec3f(0.0), Gf.Vec3f(0.0, 0.0, 0.0)]
        angularVelocities = [Gf.Vec3f(0.0, 10.0, 0.0), Gf.Vec3f(0.0)]

        self.instancer.GetProtoIndicesAttr().Set(meshIndices)
        self.instancer.GetPositionsAttr().Set(positions)
        self.instancer.GetOrientationsAttr().Set(orientations)
        self.instancer.GetVelocitiesAttr().Set(linearVelocities)
        self.instancer.GetAngularVelocitiesAttr().Set(angularVelocities)

        self.step()

        utils.check_stats(self, {"numBoxShapes": 2, "numStaticRigids": 2})

        meshIndices = [0, 0, 5, 5, 8]
        self.instancer.GetProtoIndicesAttr().Set(meshIndices)

        self.step()

        utils.check_stats(self, {"numBoxShapes": 2, "numStaticRigids": 2})

    # OM-42086
    async def test_physics_point_instancer_hierarchy_transformation(self):
        stage = await self.new_stage()

        self.base_setup(stage, config="dynamic")
        # disable gravity
        self.scene.CreateGravityMagnitudeAttr(0.0)

        # indices
        meshIndices = [0, 0]
        positions = [Gf.Vec3f(-125.0, 0.0, 500.0), Gf.Vec3f(125.0, 0.0, 500.0)]
        orientations = [Gf.Quath(1.0, 0.0, 0.0, 0.0), Gf.Quath(0.8660254, 0.0, 0.5, 0.0)]
        linearVelocities = [Gf.Vec3f(0.0), Gf.Vec3f(0.0, 0.0, 0.0)]
        angularVelocities = [Gf.Vec3f(0.0, 10.0, 0.0), Gf.Vec3f(0.0)]

        # Create point instancer
        self.instancer.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 500.0))
        self.instancer.GetProtoIndicesAttr().Set(meshIndices)
        self.instancer.GetPositionsAttr().Set(positions)
        self.instancer.GetOrientationsAttr().Set(orientations)
        self.instancer.GetVelocitiesAttr().Set(linearVelocities)
        self.instancer.GetAngularVelocitiesAttr().Set(angularVelocities)

        self.step()

        # check that translate op did not affect local-space instancer positions
        new_positions = self.instancer.GetPositionsAttr().Get()
        for new_pos, old_pos in zip(new_positions, positions):
            self.assertEqual(new_pos, old_pos)

        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2})

    # OM-43430
    async def test_physics_point_instancer_prototype_outside(self):
        stage = await self.new_stage()

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")

        geomPointInstancerPath = defaultPrimPath + "/pointinstancer"

        # Box instanced
        boxActorPath = defaultPrimPath + "/boxActor"

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)

        UsdPhysics.RigidBodyAPI.Apply(cubeGeom.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cubeGeom.GetPrim())

        # indices
        meshIndices = [0, 0]
        positions = [Gf.Vec3f(-125.0, 0.0, 500.0), Gf.Vec3f(125.0, 0.0, 500.0)]
        orientations = [Gf.Quath(1.0), Gf.Quath(1.0)]

        # Create point instancer
        shapeList = UsdGeom.PointInstancer.Define(stage, geomPointInstancerPath)

        meshList = shapeList.GetPrototypesRel()
        # add mesh reference to point instancer
        meshList.AddTarget(boxActorPath)

        shapeList.GetProtoIndicesAttr().Set(meshIndices)
        shapeList.GetPositionsAttr().Set(positions)
        shapeList.GetOrientationsAttr().Set(orientations)

        self.step()

        utils.check_stats(self, {"numBoxShapes": 3, "numDynamicRigids": 3})

    async def test_physics_point_instancer_prototype_scene_instanced(self):
        schema_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, "../../../../data/")))
        schema_folder = schema_folder.replace("\\", "/") + "/"

        print(schema_folder + "point_instancer_blast.usda")

        await self.new_stage(file_to_load=schema_folder + "point_instancer_blast.usda")

        self.step()

        utils.check_stats(self, {"numConvexShapes": 1, "numDynamicRigids": 1})

    # OM-48268
    async def test_physics_point_instancer_prototype_added_test_arrays(self):
        stage = await self.new_stage()

        self.base_setup(stage, config="dynamic")

        # run the simulation for a bit then reset it
        self.step(100, 1.0 / 60.0, True)

        # add another instance to the scene
        meshIndices = self.instancer.GetProtoIndicesAttr().Get()
        positions = self.instancer.GetPositionsAttr().Get()
        orientations = self.instancer.GetOrientationsAttr().Get()
        linearVelocities = self.instancer.GetVelocitiesAttr().Get()
        angularVelocities = self.instancer.GetAngularVelocitiesAttr().Get()

        ml = list(meshIndices)
        ml.append(0)
        meshIndices = ml

        ps = list(positions)
        ps.append(Gf.Vec3f(500.0))
        positions = ps

        ori = list(orientations)
        ori.append(Gf.Quath(1.0))
        orientations = ori

        lv = list(linearVelocities)
        lv.append(Gf.Vec3f(0.0))
        linearVelocities = lv

        av = list(linearVelocities)
        av.append(Gf.Vec3f(0.0))
        angularVelocities = lv

        self.instancer.GetProtoIndicesAttr().Set(meshIndices)
        self.instancer.GetPositionsAttr().Set(positions)
        self.instancer.GetOrientationsAttr().Set(orientations)
        self.instancer.GetVelocitiesAttr().Set(linearVelocities)
        self.instancer.GetAngularVelocitiesAttr().Set(angularVelocities)

        self.step()  # run the simulation again, it should not crash

        utils.check_stats(self, {"numBoxShapes": 3})

    async def test_physics_point_instancer_time_sampled_values(self):
        stage = await self.new_stage()

        self.base_setup(stage, config="dynamic")

        # indices
        meshIndices = [0, 0, 0, 0]
        positions = [Gf.Vec3f(200.0), Gf.Vec3f(400.0), Gf.Vec3f(600.0), Gf.Vec3f(800.0)]
        orientations = [Gf.Quath(1.0), Gf.Quath(1.0), Gf.Quath(1.0), Gf.Quath(1.0)]
        velocities = [Gf.Vec3f(1.0), Gf.Vec3f(2.0), Gf.Vec3f(3.0), Gf.Vec3f(4.0)]

        # store as time sampled values
        self.instancer.GetProtoIndicesAttr().Clear()
        self.instancer.GetProtoIndicesAttr().Set(value = meshIndices, time = 50)
        self.instancer.GetPositionsAttr().Clear()
        self.instancer.GetPositionsAttr().Set(value = positions, time = 50)
        self.instancer.GetOrientationsAttr().Clear()
        self.instancer.GetOrientationsAttr().Set(value = orientations, time = 50)
        self.instancer.GetVelocitiesAttr().Clear()
        self.instancer.GetVelocitiesAttr().Set(value = velocities, time = 50)
        self.instancer.GetAngularVelocitiesAttr().Clear()
        self.instancer.GetAngularVelocitiesAttr().Set(value = velocities, time = 50)

        get_physx_interface().start_simulation()
        
        for _ in range(10):
            self.step()
            
        resetPositions = self.instancer.GetPositionsAttr().Get()
        for (pos, rpos) in zip(positions, resetPositions):
            self.assertTrue(pos[1] > rpos[1])

        utils.check_stats(self, {"numBoxShapes": 4, "numDynamicRigids": 4})

        get_physx_interface().reset_simulation()

        linearVelocities = []
        angularVelocities = []
        resetPositions = []
        linearVelocities = self.instancer.GetVelocitiesAttr().Get()
        angularVelocities = self.instancer.GetAngularVelocitiesAttr().Get()
        resetPositions = self.instancer.GetPositionsAttr().Get()

        self.assertTrue(len(velocities) == len(linearVelocities))
        self.assertTrue(len(velocities) == len(angularVelocities))
        self.assertTrue(len(positions) == len(resetPositions))
        
        for (vel, linVel, angVel) in zip(velocities, linearVelocities, angularVelocities):
            for i in range(3):
                self.assertTrue(vel[i] == linVel[i])
                self.assertTrue(vel[i] == angVel[i])
                
        for (pos, rpos) in zip(positions, resetPositions):
            for i in range(3):
                self.assertTrue(pos[i] == rpos[i])

    # OM-75944
    async def test_physics_point_instancer_colliders_without_rb(self):
        self.stage = await self.new_stage()
        defpath = str(self.stage.GetDefaultPrim().GetPath())

        UsdPhysics.Scene.Define(self.stage, f"{defpath}/physicsScene")

        instpath = f"{defpath}/pointInstancer"
        protopath = f"{instpath}/proto"

        UsdGeom.Xform.Define(self.stage, protopath)

        for i in range(1, 3):
            colpath = f"{protopath}/col{i}"
            cubeGeom = UsdGeom.Cube.Define(self.stage, colpath)
            cubeGeom.CreateSizeAttr(25.0)
            cubeGeom.AddScaleOp().Set(Gf.Vec3f(1.0))
            cubePrim = self.stage.GetPrimAtPath(colpath)
            UsdPhysics.CollisionAPI.Apply(cubePrim)

        instancer = UsdGeom.PointInstancer.Define(self.stage, instpath)
        instancer.GetPrototypesRel().AddTarget(protopath)

        instancer.GetProtoIndicesAttr().Set([0])
        instancer.GetPositionsAttr().Set([Gf.Vec3f(0.0)])
        instancer.GetOrientationsAttr().Set([Gf.Quath(1.0, 0.0, 0.0, 0.0)])

        self.step()

        self._check_physx_object_counts({"numStaticRigids": 1, "numBoxShapes": 2})
