# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import omni.kit.test
import omni.kit.commands
from pxr import Gf, Sdf, Vt, UsdGeom, Usd, UsdPhysics, UsdLux, PhysxSchema
import omni.physx
from omni.physxcommands import AddGroundPlaneCommand, SetRigidBodyCommand, SetStaticColliderCommand
from omni.physxtests import utils
from omni.physx import get_physx_interface, get_physx_cooking_interface
from omni.physxui import get_physxui_interface
import unittest
from omni.physxtests.utils.physicsBase import PhysicsBaseAsyncTestCase, TestCategory
from omni.physxtests.utils.embeddedData import EmbeddedData
import omni.usd
from omni.physx.scripts import particleUtils, deformableUtils, physicsUtils


class PhysxPhysicsAttachmentTestAsync(PhysicsBaseAsyncTestCase):
    category = TestCategory.Kit
    @classmethod
    def setUpClass(self):
        # init for attributes that are not stage-dependent:
        # default number of places to check float equality:
        self._places = 3
        # carb settings and bloky dev mode:
        self._settings = carb.settings.get_settings()
        self._prim_type_list = ['Cone', 'Cube', 'Cylinder', 'Sphere', 'Torus']

    # runs before each test case
    async def setUp(self):
        await super().setUp()
        self._baseWasSetup = False

    # runs after each test case and runs base_terminate if setup was called:
    async def tearDown(self):
        if self._baseWasSetup:
            await self.base_terminate()
        omni.kit.commands.execute("SelectPrims", old_selected_paths=[], new_selected_paths=[], expand_in_stage=False)
        await super().tearDown()

    async def base_setup(self):
        self._baseWasSetup = True
        self.fail_on_log_error = True
        self._stage = await utils.new_stage_setup()
        sphereLight = UsdLux.SphereLight.Define(self._stage, Sdf.Path("/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        self._upAxis = UsdGeom.GetStageUpAxis(self._stage)
        self._defaultPrimPath = self._stage.GetDefaultPrim().GetPath()

        # add physics scene
        omni.kit.commands.execute("AddPhysicsScene", stage=self._stage, path='/World/PhysicsScene')

        # create a material (make a bit squishier for clearer deformation results)
        self._deformable_body_material_path = '/World/DeformableBodyMaterial'
        omni.kit.commands.execute("AddDeformableBodyMaterial",
                                  stage=self._stage, path=self._deformable_body_material_path,
                                  youngsModulus=5000.0)

    async def base_terminate(self):
        pass

    def add_groundplane(self):
        AddGroundPlaneCommand.execute(self._stage, '/CollisionPlane',
                                      self._upAxis, 1000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    async def _runAddDeformableBodyComponentCommand(self, skin_mesh_path: Sdf.Path=Sdf.Path(), collision_mesh_path: Sdf.Path=Sdf.Path(), simulation_mesh_path: Sdf.Path=Sdf.Path()) -> bool:
        # make path for softbody
        self.assertTrue(bool(skin_mesh_path))

        # create softbody:
        success = omni.kit.commands.execute(
            "AddDeformableBodyComponent",
            skin_mesh_path=skin_mesh_path,
            collision_mesh_path=collision_mesh_path,
            simulation_mesh_path=simulation_mesh_path)

        # set deformable body material
        physicsUtils.add_physics_material_to_prim(self._stage, self._stage.GetPrimAtPath(skin_mesh_path), self._deformable_body_material_path)

        # this is a workaround for hang in logger while async cooking, would be nice to get at the bottom of this
        # tests that check for the cooked meshes should still call cook_deformable_body_mesh separately
        get_physx_cooking_interface().cook_deformable_body_mesh(str(skin_mesh_path))
        return success

    @staticmethod
    def set_prim_translation(prim: Usd.Prim, translateVec: Gf.Vec3d):
        translate_mtx = Gf.Matrix4d().SetTranslate(translateVec)
        omni.kit.commands.execute("TransformPrim", path=prim.GetPath(), new_transform_matrix=translate_mtx)

    def _create_mesh_prims(self, prim_type_list: list) -> list:
        mesh_list = []
        for prim_type in prim_type_list:
            path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/" + prim_type, True))
            omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type=prim_type)
            mesh = UsdGeom.Mesh.Get(self._stage, path)
            self.assertTrue(mesh)
            mesh_list.append(mesh)
        return mesh_list

    async def _create_mesh_primitives(self, prim_type_list, starting_height=60.0):
        mesh_list = self._create_mesh_prims(prim_type_list)

        height = starting_height
        offset = 150
        origin = Gf.Vec3d(-offset * 3 / 2, height, -offset * 3 / 2)
        for i in range(3):
            for j in range(3):
                index = i * 3 + j
                if index < len(mesh_list):
                    self.set_prim_translation(mesh_list[index].GetPrim(), origin + Gf.Vec3d(i * offset, 0, j * offset))

        return mesh_list

    @staticmethod
    def _get_time_step():
        return 1.0 / 60.0

    def _start(self):
        physx_interface = get_physx_interface()
        physx_interface.start_simulation()

    def _step(self, numSteps):
        physx_interface = get_physx_interface()
        time = 0.0
        dtime = self._get_time_step()
        for i in range(numSteps):
            physx_interface.update_simulation(dtime, time)
            physx_interface.update_transformations(True, True, True, False)
            time = time + dtime

    def _reset(self):
        physx_interface = get_physx_interface()
        physx_interface.reset_simulation()


    async def test_deformableBody_rigidBody_attachment(self):
        for geometry_type in ['mesh', 'shape']:
            for api_type in ['collider', 'rigid_body']:
                await self.base_setup()
                starting_height = 170.0
                prim_type_list = ['Cone']
                coneMesh, = await self._create_mesh_primitives(prim_type_list, starting_height=starting_height)

                translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, starting_height, 0))
                omni.kit.commands.execute("TransformPrim", path=coneMesh.GetPath(), new_transform_matrix=translate_mtx)

                await self._runAddDeformableBodyComponentCommand(skin_mesh_path=coneMesh.GetPath())
                soft_body_translation = coneMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
                self.assertTrue(soft_body_translation[1] == starting_height)

                cubeGeometry = None
                if geometry_type == 'mesh':
                    prim_type_list = ['Cube']
                    cubeGeometry, = await self._create_mesh_primitives(prim_type_list, starting_height=starting_height)
                    translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, starting_height + 30.0, 0))
                    omni.kit.commands.execute("TransformPrim", path=cubeGeometry.GetPath(), new_transform_matrix=translate_mtx)
                elif geometry_type == 'shape':
                    cubeGeometry = omni.physx.scripts.physicsUtils.add_cube(self._stage, "/cubeShape", 100.0, Gf.Vec3f(0.0, starting_height + 30.0, 0.0))

                if api_type == 'collider':
                    SetStaticColliderCommand.execute(cubeGeometry.GetPath(), "")
                elif api_type == 'rigid_body':
                    SetRigidBodyCommand.execute(cubeGeometry.GetPath(), "", True)

                target_attachment_path = coneMesh.GetPath().AppendElementString("rigidAttachment")
                target_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False))
                omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=target_attachment_path, actor0_path=coneMesh.GetPath(), actor1_path=cubeGeometry.GetPath())

                self._start()
                self._step(30)

                # if the softbody is attached, it will not fall to the ground
                soft_body_translation = coneMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
                pause_height = soft_body_translation[1]
                self.assertTrue(pause_height > (starting_height / 2.0))


    async def test_deformableBody_deformableBody_attachment(self):
        await self.base_setup()
        self.add_groundplane()
        cone_height = 170.0
        cube_height = cone_height + 30.0
        sphere_height = cone_height - 90.0
        prim_type_list = ['Cone', 'Sphere']
        coneMesh, sphereMesh = await self._create_mesh_primitives(prim_type_list, starting_height=cone_height)

        translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, cone_height, 0))
        omni.kit.commands.execute("TransformPrim", path=coneMesh.GetPath(), new_transform_matrix=translate_mtx)

        translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, sphere_height, 0))
        omni.kit.commands.execute("TransformPrim", path=sphereMesh.GetPath(), new_transform_matrix=translate_mtx)

        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=coneMesh.GetPath())
        soft_body_0_translation = coneMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertTrue(soft_body_0_translation[1] == cone_height)

        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=sphereMesh.GetPath())
        soft_body_1_translation = sphereMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertTrue(soft_body_1_translation[1] == sphere_height)

        cubePrim = omni.physx.scripts.physicsUtils.add_cube(self._stage, "/cubeActor", 100.0, Gf.Vec3f(0.0, cube_height, 0.0))
        SetRigidBodyCommand.execute(cubePrim.GetPath(), "", False)

        physicsBodyAPI = UsdPhysics.RigidBodyAPI.Get(self._stage, cubePrim.GetPath())
        physicsBodyAPI.CreateKinematicEnabledAttr(True)

        target_attachment_path = coneMesh.GetPath().AppendElementString("rigidAttachment")
        target_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False))
        omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=target_attachment_path, actor0_path=coneMesh.GetPath(), actor1_path=cubePrim.GetPath())

        target_attachment_path = coneMesh.GetPath().AppendElementString("deformableAttachment")
        target_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False))
        omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=target_attachment_path, actor0_path=coneMesh.GetPath(), actor1_path=sphereMesh.GetPath())

        self._start()
        self._step(30)

        # if the softbody is attached, it will not fall to the ground
        soft_body_1_translation = sphereMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        pause_height = soft_body_1_translation[1]
        self.assertTrue(pause_height > (sphere_height / 2.0))


    async def test_deformableBody_static_attachment(self):
        await self.base_setup()
        self.add_groundplane()
        starting_height = 170.0
        prim_type_list = ['Cone']
        coneMesh, = await self._create_mesh_primitives(prim_type_list, starting_height=starting_height)

        translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, starting_height, 0))
        omni.kit.commands.execute("TransformPrim", path=coneMesh.GetPath(), new_transform_matrix=translate_mtx)

        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=coneMesh.GetPath())
        soft_body_translation = coneMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertTrue(soft_body_translation[1] == starting_height)

        cubePrim = omni.physx.scripts.physicsUtils.add_cube(self._stage, "/cubeActor", 100.0, Gf.Vec3f(0.0, starting_height + 30.0, 0.0))
        SetRigidBodyCommand.execute(cubePrim.GetPath(), "", False)

        physicsBodyAPI = UsdPhysics.RigidBodyAPI.Get(self._stage, cubePrim.GetPath())
        physicsBodyAPI.CreateKinematicEnabledAttr(True)

        target_attachment_path = coneMesh.GetPath().AppendElementString("worldAttachment")
        target_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False))
        omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=target_attachment_path, actor0_path=coneMesh.GetPath(), actor1_path=cubePrim.GetPath())
        await self.wait(1)

        # remove actor 1 rel, this will change the attachment to world static attachment
        attachment = PhysxSchema.PhysxPhysicsAttachment.Get(self._stage, target_attachment_path)
        attachment.CreateActor1Rel().ClearTargets(False)
        # allow sufficient time for the attachment authoring listener to process the attachment
        await self.wait(1)

        # remove rigid cube to be sure that there is no rigid body that the attachment can be attached to
        self._stage.RemovePrim(cubePrim.GetPath())

        self._start()
        self._step(30)

        # if the softbody is attached, it will not fall to the ground
        soft_body_translation = coneMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        pause_height = soft_body_translation[1]
        self.assertTrue(pause_height > (starting_height / 2.0))


    async def test_deformable_body_rigid_body_attachment_enable_disable(self):
        await self.base_setup()

        initial_translation=Gf.Vec3f(0.0, 170.0, 0.0)
        moved_delta = 10.0

        # create deformable body
        cone_mesh, = await self._create_mesh_primitives(['Cone'])
        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=cone_mesh.GetPath())
        physicsUtils.set_or_add_translate_op(cone_mesh, initial_translation)

        # create kinematic rigid body
        cube_prim = physicsUtils.add_cube(self._stage, "/cubeActor", 100.0, initial_translation + Gf.Vec3f(0.0, 50.0, 0.0))
        SetRigidBodyCommand.execute(cube_prim.GetPath(), "", True)

        # create attachment
        attachment_path = cone_mesh.GetPath().AppendElementString("rigidAttachment")
        omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=attachment_path, actor0_path=cone_mesh.GetPath(), actor1_path=cube_prim.GetPath())
        attachment = PhysxSchema.PhysxPhysicsAttachment.Get(self._stage, attachment_path)
        self.assertTrue(attachment)

        # simulate and check that deformable body is attached (it will not fall)
        self._start()
        self._step(10)
        new_translation = physicsUtils.get_translation(cone_mesh.GetPrim())
        self.assertTrue(new_translation[1] > initial_translation[1] - moved_delta)
        self._reset()

        # disable attachment and simulate, then check that deformable body is not attached
        attachment.GetAttachmentEnabledAttr().Set(False)
        self._start()
        self._step(10)
        new_translation = physicsUtils.get_translation(cone_mesh.GetPrim())
        self.assertTrue(new_translation[1] < initial_translation[1] - moved_delta)
        self._reset()


    @staticmethod
    def getWorldBounds(imageable: UsdGeom.Imageable) -> Gf.Range3d:
        obb = imageable.ComputeWorldBound(Usd.TimeCode.Default(), purpose1=imageable.GetPurposeAttr().Get())
        return obb.ComputeAlignedBox()

    #@unittest.skip("OM-76592")
    async def test_deformableBody_rigidBody_attachment_filtering(self):
        await self.base_setup()
        prim_type_list = ['Cone']
        coneMesh, = self._create_mesh_prims(prim_type_list)

        physicsUtils.setup_transform_as_scale_orient_translate(coneMesh)
        initial_orient = Gf.Quatd(Gf.Rotation(Gf.Vec3d(1,0,0), 120).GetQuat())
        initial_scale = Gf.Vec3d(0.5, 1.5, 0.5)
        initial_translation = Gf.Vec3d(0, 180.0, -72.0)
        physicsUtils.set_or_add_scale_orient_translate(coneMesh, initial_scale, initial_orient, initial_translation)
        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=coneMesh.GetPath())

        cubePrim = omni.physx.scripts.physicsUtils.add_cube(self._stage, "/cubeActor", 100.0)
        initial_orient = Gf.Quatd(0.70710677, 0.70710677, 0, 0)
        initial_scale = Gf.Vec3d(300, 300, 300)
        initial_translation = Gf.Vec3d(0, 0, 0)
        physicsUtils.set_or_add_scale_orient_translate(cubePrim, initial_scale, initial_orient, initial_translation)
        SetRigidBodyCommand.execute(cubePrim.GetPath(), "", True)

        target_attachment_path = coneMesh.GetPath().AppendElementString("rigidAttachment")
        target_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False))
        omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=target_attachment_path, actor0_path=coneMesh.GetPath(), actor1_path=cubePrim.GetPath())

        self._start()
        self._step(40)

        # softbody should collide with the rigidbody
        cubeGeom = UsdGeom.Cube(cubePrim)
        box_bounds = self.getWorldBounds(cubeGeom)
        soft_body_pos = coneMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertFalse(box_bounds.Contains(soft_body_pos))

        self._reset()

        # fully filter the softbody so that it no longer collides with the rigidbody
        attachment = PhysxSchema.PhysxPhysicsAttachment.Define(self._stage, target_attachment_path)
        autoApi = PhysxSchema.PhysxAutoAttachmentAPI.Apply(attachment.GetPrim())
        autoApi.GetCollisionFilteringOffsetAttr().Set(200.0)

        self._start()
        self._step(40)

        # softbody will penetrate the rigidbody
        box_bounds = self.getWorldBounds(cubeGeom)
        soft_body_pos = coneMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertTrue(box_bounds.Contains(soft_body_pos))


    async def test_deformableBody_deformableBody_attachment_filtering(self):
        await self.base_setup()
        prim_type_list = ['Cone', 'Cube']
        coneMesh, CubeMesh= await self._create_mesh_primitives(prim_type_list)

        physicsUtils.setup_transform_as_scale_orient_translate(coneMesh)
        initial_orient = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1,0,0), 120).GetQuat())
        initial_scale = Gf.Vec3d(0.25, 0.75, 0.25)
        initial_translation = Gf.Vec3d(0, 107, 360)
        physicsUtils.set_or_add_scale_orient_translate(coneMesh, initial_scale, initial_orient, initial_translation)
        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=coneMesh.GetPath())

        physicsUtils.setup_transform_as_scale_orient_translate(CubeMesh)
        initial_orient = Gf.Quatd(1, 0, 0, 0)
        initial_scale = Gf.Vec3d(1, 1, 1)
        initial_translation = Gf.Vec3d(0, 47.46, 361.458)
        physicsUtils.set_or_add_scale_orient_translate(CubeMesh, initial_scale, initial_orient, initial_translation)
        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=CubeMesh.GetPath())

        deformable_attachment_path = coneMesh.GetPath().AppendElementString("deformableAttachment")
        deformable_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(deformable_attachment_path), False))
        omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=deformable_attachment_path, actor0_path=coneMesh.GetPath(), actor1_path=CubeMesh.GetPath())

        cubePrim = omni.physx.scripts.physicsUtils.add_cube(self._stage, "/cubeAnchor", 100.0)
        physicsUtils.setup_transform_as_scale_orient_translate(cubePrim)
        initial_orient = Gf.Quatd(1, 0, 0, 0)
        initial_scale = Gf.Vec3d(100, 100, 100)
        initial_translation = Gf.Vec3d(0, -42, 361.458)
        physicsUtils.set_or_add_scale_orient_translate(cubePrim, initial_scale, initial_orient, initial_translation)
        SetRigidBodyCommand.execute(cubePrim.GetPath(), "", True)

        rigid_attachment_path = CubeMesh.GetPath().AppendElementString("rigidAttachment")
        rigid_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(rigid_attachment_path), False))
        omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=rigid_attachment_path, actor0_path=CubeMesh.GetPath(), actor1_path=cubePrim.GetPath())

        self._start()
        self._step(40)

        # softbody should collide with the softbody
        box_bounds = self.getWorldBounds(CubeMesh)
        soft_body_pos = coneMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertFalse(box_bounds.Contains(soft_body_pos))

        self._reset()

        # fully filter the softbody so that it no longer collides with the other softbody
        attachment = PhysxSchema.PhysxPhysicsAttachment.Define(self._stage, deformable_attachment_path)
        autoApi = PhysxSchema.PhysxAutoAttachmentAPI.Apply(attachment.GetPrim())
        autoApi.GetCollisionFilteringOffsetAttr().Set(200.0)

        self._start()
        self._step(40)

        # softbody will penetrate the rigidbody
        box_bounds = self.getWorldBounds(CubeMesh)
        soft_body_pos = coneMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertTrue(box_bounds.Contains(soft_body_pos))


    async def test_get_attachments_api(self):
        await self.base_setup()

        #create deformable body mesh
        deformable_body, = await self._create_mesh_primitives(['Sphere'])
        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=deformable_body.GetPath())

        #create rigid body cubes
        rigid_body_0 = omni.physx.scripts.physicsUtils.add_cube(self._stage, "/Cube0")
        rigid_body_1 = omni.physx.scripts.physicsUtils.add_cube(self._stage, "/Cube1")

        #create attachments
        attachment_path0 = "/attachment_0"
        attachment_path1 = "/attachment_1"
        omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=attachment_path0, actor0_path=deformable_body.GetPath(), actor1_path=rigid_body_0.GetPath())
        omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=attachment_path1, actor0_path=deformable_body.GetPath(), actor1_path=rigid_body_1.GetPath())

        #give physx ui a chance to register the new attachment
        await omni.kit.app.get_app().next_update_async()

        #excercise physxui get_attachments
        sphere_attachment_paths = get_physxui_interface().get_attachments(str(deformable_body.GetPath()))['attachments']
        cube0_attachment_paths = get_physxui_interface().get_attachments(str(rigid_body_0.GetPath()))['attachments']
        cube1_attachment_paths = get_physxui_interface().get_attachments(str(rigid_body_1.GetPath()))['attachments']

        #test consistency of results
        self.assertTrue(len(sphere_attachment_paths) == 2 and sphere_attachment_paths[0] == attachment_path0 and sphere_attachment_paths[1] == attachment_path1)
        self.assertTrue(len(cube0_attachment_paths) == 1 and cube0_attachment_paths[0] == attachment_path0)
        self.assertTrue(len(cube1_attachment_paths) == 1 and cube1_attachment_paths[0] == attachment_path1)


    async def test_deformable_attachment_multi_scenes(self):
        await self.base_setup()

        scene1Path = Sdf.Path('/World/PhysicsScene1')
        scene = UsdPhysics.Scene.Define(self._stage, scene1Path)

        scene2Path = Sdf.Path('/World/PhysicsScene2')
        scene = UsdPhysics.Scene.Define(self._stage, scene2Path)

        cone_height = 170.0
        cube_height = cone_height + 30.0
        sphere_height = cone_height - 90.0
        prim_type_list = ['Cone', 'Sphere']
        coneMesh, sphereMesh = await self._create_mesh_primitives(prim_type_list, starting_height=cone_height)

        translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, cone_height, 0))
        omni.kit.commands.execute("TransformPrim", path=coneMesh.GetPath(), new_transform_matrix=translate_mtx)

        translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, sphere_height, 0))
        omni.kit.commands.execute("TransformPrim", path=sphereMesh.GetPath(), new_transform_matrix=translate_mtx)

        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=coneMesh.GetPath())
        soft_body_0_translation = coneMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertTrue(soft_body_0_translation[1] == cone_height)

        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=sphereMesh.GetPath())
        soft_body_1_translation = sphereMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertTrue(soft_body_1_translation[1] == sphere_height)

        cubePrim = omni.physx.scripts.physicsUtils.add_cube(self._stage, "/cubeActor", 100.0, Gf.Vec3f(0.0, cube_height, 0.0))
        SetStaticColliderCommand.execute(cubePrim.GetPath())

        # set cone simulation owner to scene 1
        prim = self._stage.GetPrimAtPath(coneMesh.GetPath())
        deformableAPI = PhysxSchema.PhysxDeformableBodyAPI(prim)
        PhysxSchema.PhysxDeformableAPI(deformableAPI).GetSimulationOwnerRel().SetTargets([scene1Path])

        target_attachment_path = coneMesh.GetPath().AppendElementString("rigidAttachment")
        error_message = "Attachment " + str(target_attachment_path) + " was not created! The prims attached must belong to the same scene."
        with utils.ExpectMessage(self, error_message):
            omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=target_attachment_path, actor0_path=coneMesh.GetPath(), actor1_path=cubePrim.GetPath())
            self._start()
            self._step(1)
            self._reset()

        # set cube simulation owner to scene 1
        collisionAPI = UsdPhysics.CollisionAPI.Get(self._stage, cubePrim.GetPath())
        collisionAPI.CreateSimulationOwnerRel().SetTargets([scene2Path, scene1Path])
        target_attachment_path = coneMesh.GetPath().AppendElementString("rigidAttachment")
        error_message = "Attachment " + str(target_attachment_path) + " was not created! The prims attached must belong to the same scene."
        with utils.ExpectMessage(self, error_message, False):
            omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=target_attachment_path, actor0_path=coneMesh.GetPath(), actor1_path=cubePrim.GetPath())
            self._start()
            self._step(1)
            self._reset()

        # set sphere simulation owner to scene 1
        prim = self._stage.GetPrimAtPath(sphereMesh.GetPath())
        deformableAPI = PhysxSchema.PhysxDeformableBodyAPI(prim)
        PhysxSchema.PhysxDeformableAPI(deformableAPI).GetSimulationOwnerRel().SetTargets([scene1Path])
        target_attachment_path = coneMesh.GetPath().AppendElementString("deformableAttachment")
        error_message = "Attachment " + str(target_attachment_path) + " was not created! The prims attached must belong to the same scene."
        with utils.ExpectMessage(self, error_message, False):
            omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=target_attachment_path, actor0_path=coneMesh.GetPath(), actor1_path=sphereMesh.GetPath())
            self._start()
            self._step(1)
            self._reset()


class PhysxPhysicsAttachmentClothTestAsync(PhysicsBaseAsyncTestCase):
    category = TestCategory.Kit
    @classmethod
    def setUpClass(self):
        # init for attributes that are not stage-dependent:
        # default number of places to check float equality:
        self._places = 3
        # carb settings and bloky dev mode:
        self._settings = carb.settings.get_settings()
        self._prim_type_list = ["Cone", "Cube", "Cylinder", "Sphere", "Torus"]

    # runs before each test case
    async def setUp(self):
        await super().setUp()
        self._baseWasSetup = False

    # runs after each test case and runs base_terminate if setup was called:
    async def tearDown(self):
        if self._baseWasSetup:
            await self.base_terminate()
        omni.kit.commands.execute("SelectPrims", old_selected_paths=[], new_selected_paths=[], expand_in_stage=False)
        await super().tearDown()

    async def base_setup(self):
        self._baseWasSetup = True
        self.fail_on_log_error = True
        self._stage = await utils.new_stage_setup()
        sphereLight = UsdLux.SphereLight.Define(self._stage, Sdf.Path("/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        self._upAxis = UsdGeom.GetStageUpAxis(self._stage)
        self._defaultPrimPath = self._stage.GetDefaultPrim().GetPath()

        # add physics scene
        omni.kit.commands.execute("AddPhysicsScene", stage=self._stage, path="/World/PhysicsScene")

        # create a material (make a bit squishier for clearer deformation results)
        self._deformable_body_material_path = "/World/DeformableBodyMaterial"
        omni.kit.commands.execute(
            "AddDeformableBodyMaterial",
            stage=self._stage,
            path=self._deformable_body_material_path,
            youngsModulus=5000.0,
        )

    async def base_terminate(self):
        pass

    def add_groundplane(self):
        AddGroundPlaneCommand.execute(
            self._stage, "/CollisionPlane", self._upAxis, 1000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5)
        )

    async def _runAddDeformableBodyComponentCommand(
        self,
        skin_mesh_path: Sdf.Path = Sdf.Path(),
        collision_mesh_path: Sdf.Path = Sdf.Path(),
        simulation_mesh_path: Sdf.Path = Sdf.Path(),
    ) -> bool:
        # make path for softbody
        self.assertTrue(bool(skin_mesh_path))

        # create softbody:
        success = omni.kit.commands.execute(
            "AddDeformableBodyComponent",
            skin_mesh_path=skin_mesh_path,
            collision_mesh_path=collision_mesh_path,
            simulation_mesh_path=simulation_mesh_path,
        )

        # set deformable body material
        physicsUtils.add_physics_material_to_prim(
            self._stage, self._stage.GetPrimAtPath(skin_mesh_path), self._deformable_body_material_path
        )

        # this is a workaround for hang in logger while async cooking, would be nice to get at the bottom of this
        # tests that check for the cooked meshes should still call cook_deformable_body_mesh separately
        get_physx_cooking_interface().cook_deformable_body_mesh(str(skin_mesh_path))
        return success

    @staticmethod
    def set_prim_translation(prim: Usd.Prim, translateVec: Gf.Vec3d):
        translate_mtx = Gf.Matrix4d().SetTranslate(translateVec)
        omni.kit.commands.execute("TransformPrim", path=prim.GetPath(), new_transform_matrix=translate_mtx)

    def _create_mesh_prims(self, prim_type_list: list) -> list:
        mesh_list = []
        for prim_type in prim_type_list:
            path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/" + prim_type, True))
            omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type=prim_type)
            mesh = UsdGeom.Mesh.Get(self._stage, path)
            self.assertTrue(mesh)
            mesh_list.append(mesh)
        return mesh_list

    async def _create_mesh_primitives(self, prim_type_list, starting_height=60.0):
        mesh_list = self._create_mesh_prims(prim_type_list)

        height = starting_height
        offset = 150
        origin = Gf.Vec3d(-offset * 3 / 2, height, -offset * 3 / 2)
        for i in range(3):
            for j in range(3):
                index = i * 3 + j
                if index < len(mesh_list):
                    self.set_prim_translation(mesh_list[index].GetPrim(), origin + Gf.Vec3d(i * offset, 0, j * offset))

        return mesh_list

    def _create_tesselated_plane_mesh(self, u_resolution=10, v_resolution=10, scale=100.0):

        success, mesh_path = omni.kit.commands.execute(
            "CreateMeshPrimWithDefaultXform",
            prim_type="Plane",
            u_patches=u_resolution,
            v_patches=v_resolution,
            u_verts_scale=1,
            v_verts_scale=1,
            half_scale=scale * 0.5,
        )
        self.assertTrue(success)

        return UsdGeom.Mesh.Get(self._stage, mesh_path)

    @staticmethod
    def _get_time_step():
        return 1.0 / 60.0

    def _start(self):
        physx_interface = get_physx_interface()
        physx_interface.start_simulation()

    def _step(self, numSteps):
        physx_interface = get_physx_interface()
        time = 0.0
        dtime = self._get_time_step()
        for i in range(numSteps):
            physx_interface.update_simulation(dtime, time)
            physx_interface.update_transformations(True, True, True, False)
            time = time + dtime

    def _reset(self):
        physx_interface = get_physx_interface()
        physx_interface.reset_simulation()

    def set_cloth(self, mesh):
        omni.kit.commands.execute("AddParticleClothComponent", prim_path=mesh.GetPrim().GetPath())
        # check cloth API got applied
        cloth = PhysxSchema.PhysxParticleClothAPI.Get(self._stage, mesh.GetPath())
        self.assertTrue(cloth)
        return cloth

    async def test_particleCloth_rigidBody_attachment(self):
        for geometry_type in ['mesh', 'shape']:
            for api_type in ['collider', 'rigid_body']:
                await self.base_setup()
                starting_height = 170.0
                prim_type_list = ['Sphere']
                sphereMesh, = await self._create_mesh_primitives(prim_type_list, starting_height=starting_height)

                translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, starting_height, 0))
                omni.kit.commands.execute("TransformPrim", path=sphereMesh.GetPath(), new_transform_matrix=translate_mtx)

                self.set_cloth(mesh=sphereMesh)
                particle_cloth_translation = sphereMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
                self.assertTrue(particle_cloth_translation[1] == starting_height)

                cubeGeometry = None
                if geometry_type == 'mesh':
                    prim_type_list = ['Cube']
                    cubeGeometry, = await self._create_mesh_primitives(prim_type_list, starting_height=starting_height)
                    translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, starting_height + 60.0, 0))
                    omni.kit.commands.execute("TransformPrim", path=cubeGeometry.GetPath(), new_transform_matrix=translate_mtx)
                elif geometry_type == 'shape':
                    cubeGeometry = omni.physx.scripts.physicsUtils.add_cube(self._stage, "/cubeShape", 100.0, Gf.Vec3f(0.0, starting_height + 60.0, 0.0))

                if api_type == 'collider':
                    SetStaticColliderCommand.execute(cubeGeometry.GetPath(), "")
                elif api_type == 'rigid_body':
                    SetRigidBodyCommand.execute(cubeGeometry.GetPath(), "", True)

                target_attachment_path = sphereMesh.GetPath().AppendElementString("rigidAttachment")
                target_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False))
                omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=target_attachment_path, actor0_path=sphereMesh.GetPath(), actor1_path=cubeGeometry.GetPath())

                self._start()
                self._step(30)

                # if the particle cloth is attached, it will not fall to the ground
                particle_cloth_translation = sphereMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
                pause_height = particle_cloth_translation[1]
                self.assertTrue(pause_height > (starting_height / 2.0))

    # Reenable this test since TC is now green
    # @unittest.skip("OM-43150")
    async def test_deformableBody_particleCloth_attachment(self):
        await self.base_setup()
        self.add_groundplane()
        cone_height = 170.0
        cube_height = cone_height + 30.0
        sphere_height = cone_height - 90.0
        prim_type_list = ["Cone", "Sphere"]
        coneMesh, sphereMesh = await self._create_mesh_primitives(prim_type_list, starting_height=cone_height)

        translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, cone_height, 0))
        omni.kit.commands.execute("TransformPrim", path=coneMesh.GetPath(), new_transform_matrix=translate_mtx)

        translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, sphere_height, 0))
        omni.kit.commands.execute("TransformPrim", path=sphereMesh.GetPath(), new_transform_matrix=translate_mtx)

        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=coneMesh.GetPath())
        soft_body_translation = coneMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertTrue(soft_body_translation[1] == cone_height)

        self.set_cloth(mesh=sphereMesh)
        particle_cloth_translation = sphereMesh.ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        ).ExtractTranslation()
        self.assertTrue(particle_cloth_translation[1] == sphere_height)

        cubePrim = omni.physx.scripts.physicsUtils.add_cube(
            self._stage, "/cubeActor", 100.0, Gf.Vec3f(0.0, cube_height, 0.0)
        )
        SetRigidBodyCommand.execute(cubePrim.GetPath(), "", False)

        physicsBodyAPI = UsdPhysics.RigidBodyAPI.Get(self._stage, cubePrim.GetPath())
        physicsBodyAPI.CreateKinematicEnabledAttr(True)

        target_attachment_path = coneMesh.GetPath().AppendElementString("rigidAttachment")
        target_attachment_path = Sdf.Path(
            omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False)
        )
        omni.kit.commands.execute(
            "CreatePhysicsAttachment",
            target_attachment_path=target_attachment_path,
            actor0_path=coneMesh.GetPath(),
            actor1_path=cubePrim.GetPath(),
        )

        target_attachment_path = coneMesh.GetPath().AppendElementString("deformableAttachment")
        target_attachment_path = Sdf.Path(
            omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False)
        )
        omni.kit.commands.execute(
            "CreatePhysicsAttachment",
            target_attachment_path=target_attachment_path,
            actor0_path=coneMesh.GetPath(),
            actor1_path=sphereMesh.GetPath(),
        )

        self._start()
        self._step(30)

        # if the particle cloth is attached, it will not fall to the ground
        particle_cloth_translation = sphereMesh.ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        ).ExtractTranslation()
        pause_height = particle_cloth_translation[1]
        self.assertTrue(pause_height > (sphere_height / 2.0))

    async def test_particleCloth_static_attachment(self):
        await self.base_setup()
        self.add_groundplane()
        starting_height = 170.0
        prim_type_list = ["Sphere"]
        (sphereMesh,) = await self._create_mesh_primitives(prim_type_list, starting_height=starting_height)

        translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, starting_height, 0))
        omni.kit.commands.execute("TransformPrim", path=sphereMesh.GetPath(), new_transform_matrix=translate_mtx)

        self.set_cloth(mesh=sphereMesh)
        particle_cloth_translation = sphereMesh.ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        ).ExtractTranslation()
        self.assertTrue(particle_cloth_translation[1] == starting_height)

        cubePrim = omni.physx.scripts.physicsUtils.add_cube(
            self._stage, "/cubeActor", 100.0, Gf.Vec3f(0.0, starting_height + 60.0, 0.0)
        )
        SetRigidBodyCommand.execute(cubePrim.GetPath(), "", False)

        physicsBodyAPI = UsdPhysics.RigidBodyAPI.Get(self._stage, cubePrim.GetPath())
        physicsBodyAPI.CreateKinematicEnabledAttr(True)

        target_attachment_path = sphereMesh.GetPath().AppendElementString("worldAttachment")
        target_attachment_path = Sdf.Path(
            omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False)
        )
        omni.kit.commands.execute(
            "CreatePhysicsAttachment",
            target_attachment_path=target_attachment_path,
            actor0_path=sphereMesh.GetPath(),
            actor1_path=cubePrim.GetPath(),
        )
        await self.wait(1)

        # remove actor 1 rel, this will change the attachment to world static attachment
        attachment = PhysxSchema.PhysxPhysicsAttachment.Get(self._stage, target_attachment_path)
        attachment.CreateActor1Rel().ClearTargets(False)
        # allow sufficient time for the attachment authoring listener to process the attachment
        await self.wait(1)

        # remove rigid cube to be sure that there is no rigid body that the attachment can be attached to
        self._stage.RemovePrim(cubePrim.GetPath())

        self._start()
        self._step(30)

        # if the particle cloth is attached, it will not fall to the ground
        particle_cloth_translation = sphereMesh.ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        ).ExtractTranslation()
        pause_height = particle_cloth_translation[1]
        self.assertTrue(pause_height > (starting_height / 2.0))

    async def test_particle_cloth_rigid_body_attachment_enable_disable(self):
        await self.base_setup()

        initial_translation = Gf.Vec3f(0.0)
        initial_orient = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1,0,0), 90.0).GetQuat())
        moved_delta = 10.0

        # create particle cloth
        plane_mesh = self._create_tesselated_plane_mesh()
        physicsUtils.set_or_add_scale_orient_translate(plane_mesh, Gf.Vec3f(1.0), initial_orient, initial_translation)
        self.set_cloth(mesh=plane_mesh)

        # create kinematic rigid body
        cube_prim = physicsUtils.add_cube(self._stage, "/cubeActor", 100.0, initial_translation + Gf.Vec3f(0.0, 100.0, 0.0))
        self.assertTrue(omni.kit.commands.execute("SetRigidBodyCommand", path=cube_prim.GetPath(), approximationShape="", kinematic=True))

        # create attachment
        attachment_path = plane_mesh.GetPath().AppendElementString("rigidAttachment")
        omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=attachment_path, actor0_path=plane_mesh.GetPath(), actor1_path=cube_prim.GetPath())
        attachment = PhysxSchema.PhysxPhysicsAttachment.Get(self._stage, attachment_path)
        self.assertTrue(attachment)

        # simulate and check that deformable body is attached (it will not fall)
        self._start()
        self._step(10)
        new_translation = physicsUtils.get_translation(plane_mesh.GetPrim())
        self.assertTrue(new_translation[1] > initial_translation[1] - moved_delta)
        self._reset()

        # disable attachment and simulate, then check that deformable body is not attached
        attachment.GetAttachmentEnabledAttr().Set(False)
        self._start()
        self._step(10)
        new_translation = physicsUtils.get_translation(plane_mesh.GetPrim())
        self.assertTrue(new_translation[1] < initial_translation[1] - moved_delta)
        self._reset()

    @staticmethod
    def getWorldBounds(imageable: UsdGeom.Imageable) -> Gf.Range3d:
        obb = imageable.ComputeWorldBound(Usd.TimeCode.Default(), purpose1=imageable.GetPurposeAttr().Get())
        return obb.ComputeAlignedBox()

    async def test_particleCloth_rigidBody_attachment_filtering(self):
        await self.base_setup()

        # create particle cloth
        plane_mesh = self._create_tesselated_plane_mesh(u_resolution=100, v_resolution=100)

        initial_orient = Gf.Quatd(1, 0, 0, 0)
        initial_scale = Gf.Vec3d(1, 1, 1.5)
        initial_translation = Gf.Vec3d(0, 100.0, -100.0)
        physicsUtils.set_or_add_scale_orient_translate(plane_mesh, initial_scale, initial_orient, initial_translation)
        self.set_cloth(mesh=plane_mesh)

        cubePrim = omni.physx.scripts.physicsUtils.add_cube(self._stage, "/cubeActor", 100.0)
        initial_orient = Gf.Quatd(1, 0, 0, 0)
        initial_scale = Gf.Vec3d(120, 300, 100)
        initial_translation = Gf.Vec3d(0, 0, 0)
        physicsUtils.set_or_add_scale_orient_translate(cubePrim, initial_scale, initial_orient, initial_translation)
        self.assertTrue(omni.kit.commands.execute("SetRigidBodyCommand", path=cubePrim.GetPath(), approximationShape="", kinematic=True))

        target_attachment_path = plane_mesh.GetPath().AppendElementString("rigidAttachment")
        target_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False))
        omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=target_attachment_path, actor0_path=plane_mesh.GetPath(), actor1_path=cubePrim.GetPath())

        self._start()
        self._step(75)

        # particle cloth should collide with the rigidbody
        particle_cloth_tol = Gf.Vec3d(0.0, 0.0, -10.0)
        cubeGeom = UsdGeom.Cube(cubePrim)
        box_bounds = self.getWorldBounds(cubeGeom)
        particle_cloth_pos = plane_mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertFalse(box_bounds.Contains(particle_cloth_pos + particle_cloth_tol))

        self._reset()

        # fully filter the particle cloth so that it no longer collides with the rigidbody
        attachment = PhysxSchema.PhysxPhysicsAttachment.Define(self._stage, target_attachment_path)
        autoApi = PhysxSchema.PhysxAutoAttachmentAPI.Apply(attachment.GetPrim())
        autoApi.GetCollisionFilteringOffsetAttr().Set(300.0)

        self._start()
        self._step(75)

        # particle cloth will penetrate the rigidbody
        box_bounds = self.getWorldBounds(cubeGeom)
        particle_cloth_pos = plane_mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertTrue(box_bounds.Contains(particle_cloth_pos + particle_cloth_tol))

    async def test_particleCloth_deformableBody_attachment_filtering(self):
        await self.base_setup()

        # create particle cloth
        plane_mesh = self._create_tesselated_plane_mesh(u_resolution=100, v_resolution=100)

        # create cube mesh
        prim_type_list = ['Cube']
        CubeMesh, = await self._create_mesh_primitives(prim_type_list)

        initial_orient = Gf.Quatd(1, 0, 0, 0)
        initial_scale = Gf.Vec3d(1, 2, 2.5)
        initial_translation = Gf.Vec3d(0, 160, -210)
        physicsUtils.set_or_add_scale_orient_translate(plane_mesh, initial_scale, initial_orient, initial_translation)
        self.set_cloth(mesh=plane_mesh)

        initial_orient = Gf.Quatd(1, 0, 0, 0)
        initial_scale = Gf.Vec3d(2, 4, 2)
        initial_translation = Gf.Vec3d(0, 0, 0)
        physicsUtils.set_or_add_scale_orient_translate(CubeMesh, initial_scale, initial_orient, initial_translation)
        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=CubeMesh.GetPath())

        deformable_attachment_path = plane_mesh.GetPath().AppendElementString("deformableAttachment")
        deformable_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(deformable_attachment_path), False))
        omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=deformable_attachment_path, actor0_path=plane_mesh.GetPath(), actor1_path=CubeMesh.GetPath())

        cubePrim = omni.physx.scripts.physicsUtils.add_cube(self._stage, "/cubeAnchor", 100.0)
        initial_orient = Gf.Quatd(1, 0, 0, 0)
        initial_scale = Gf.Vec3d(100, 100, 100)
        initial_translation = Gf.Vec3d(0, -230, 0)
        physicsUtils.set_or_add_scale_orient_translate(cubePrim, initial_scale, initial_orient, initial_translation)
        self.assertTrue(omni.kit.commands.execute("SetRigidBodyCommand", path=cubePrim.GetPath(), approximationShape="", kinematic=True))

        rigid_attachment_path = plane_mesh.GetPath().AppendElementString("rigidAttachment")
        rigid_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(rigid_attachment_path), False))
        omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=rigid_attachment_path, actor0_path=CubeMesh.GetPath(), actor1_path=cubePrim.GetPath())

        cubePrim = omni.physx.scripts.physicsUtils.add_cube(self._stage, "/cubeAnchor1", 100.0)
        initial_orient = Gf.Quatd(1, 0, 0, 0)
        initial_scale = Gf.Vec3d(100, 100, 100)
        initial_translation = Gf.Vec3d(0, 230, 0)
        physicsUtils.set_or_add_scale_orient_translate(cubePrim, initial_scale, initial_orient, initial_translation)
        self.assertTrue(omni.kit.commands.execute("SetRigidBodyCommand", path=cubePrim.GetPath(), approximationShape="", kinematic=True))

        rigid_attachment_path = plane_mesh.GetPath().AppendElementString("rigidAttachment1")
        rigid_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(rigid_attachment_path), False))
        omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=rigid_attachment_path, actor0_path=CubeMesh.GetPath(), actor1_path=cubePrim.GetPath())

        self._start()
        self._step(75)

        # particle cloth should collide with the rigidbody
        particle_cloth_tol = Gf.Vec3d(0.0, 0.0, -10.0)
        box_bounds = self.getWorldBounds(CubeMesh)
        particle_cloth_pos = plane_mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertFalse(box_bounds.Contains(particle_cloth_pos + particle_cloth_tol))

        self._reset()

        # fully filter the particle cloth so that it no longer collides with the rigidbody
        attachment = PhysxSchema.PhysxPhysicsAttachment.Define(self._stage, deformable_attachment_path)
        autoApi = PhysxSchema.PhysxAutoAttachmentAPI.Apply(attachment.GetPrim())
        autoApi.GetCollisionFilteringOffsetAttr().Set(300.0)

        self._start()
        self._step(75)

        # particle cloth will penetrate the rigidbody
        box_bounds = self.getWorldBounds(CubeMesh)
        particle_cloth_pos = plane_mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertTrue(box_bounds.Contains(particle_cloth_pos + particle_cloth_tol))

    async def test_particleCloth_attachment_multi_scenes(self):
        await self.base_setup()

        scene1Path = Sdf.Path('/World/PhysicsScene1')
        scene = UsdPhysics.Scene.Define(self._stage, scene1Path)

        scene2Path = Sdf.Path('/World/PhysicsScene2')
        scene = UsdPhysics.Scene.Define(self._stage, scene2Path)

        # set particle system simulation owner to scene 1
        particleUtils.add_physx_particle_system(stage=self._stage, particle_system_path='/World/particleSystem', simulation_owner=scene1Path)

        cone_height = 170.0
        cube_height = cone_height + 30.0
        sphere_height = cone_height - 90.0
        prim_type_list = ["Cone", "Sphere"]
        coneMesh, sphereMesh = await self._create_mesh_primitives(prim_type_list, starting_height=cone_height)

        translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, cone_height, 0))
        omni.kit.commands.execute("TransformPrim", path=coneMesh.GetPath(), new_transform_matrix=translate_mtx)

        translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, sphere_height, 0))
        omni.kit.commands.execute("TransformPrim", path=sphereMesh.GetPath(), new_transform_matrix=translate_mtx)

        await self._runAddDeformableBodyComponentCommand(skin_mesh_path=sphereMesh.GetPath())
        soft_body_translation = sphereMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertTrue(soft_body_translation[1] == sphere_height)

        self.set_cloth(mesh=coneMesh)
        particle_cloth_translation = coneMesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertTrue(particle_cloth_translation[1] == cone_height)

        cubePrim = omni.physx.scripts.physicsUtils.add_cube(self._stage, "/cubeActor", 100.0, Gf.Vec3f(0.0, cube_height, 0.0))
        SetStaticColliderCommand.execute(cubePrim.GetPath())

        target_attachment_path = coneMesh.GetPath().AppendElementString("rigidAttachment")
        error_message = "Attachment " + str(target_attachment_path) + " was not created! The prims attached must belong to the same scene."
        with utils.ExpectMessage(self, error_message):
            omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=target_attachment_path, actor0_path=coneMesh.GetPath(), actor1_path=cubePrim.GetPath())
            self._start()
            self._step(1)
            self._reset()

        # set cube simulation owner to scene 1
        collisionAPI = UsdPhysics.CollisionAPI.Get(self._stage, cubePrim.GetPath())
        collisionAPI.CreateSimulationOwnerRel().SetTargets([scene2Path, scene1Path])
        target_attachment_path = coneMesh.GetPath().AppendElementString("rigidAttachment")
        error_message = "Attachment " + str(target_attachment_path) + " was not created! The prims attached must belong to the same scene."
        with utils.ExpectMessage(self, error_message, False):
            omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=target_attachment_path, actor0_path=coneMesh.GetPath(), actor1_path=cubePrim.GetPath())
            self._start()
            self._step(1)
            self._reset()

        # set sphere simulation owner to scene 1
        prim = self._stage.GetPrimAtPath(sphereMesh.GetPath())
        deformableAPI = PhysxSchema.PhysxDeformableBodyAPI(prim)
        PhysxSchema.PhysxDeformableAPI(deformableAPI).GetSimulationOwnerRel().SetTargets([scene1Path])
        target_attachment_path = coneMesh.GetPath().AppendElementString("deformableAttachment")
        error_message = "Attachment " + str(target_attachment_path) + " was not created! The prims attached must belong to the same scene."
        with utils.ExpectMessage(self, error_message, False):
            omni.kit.commands.execute("CreatePhysicsAttachment", target_attachment_path=target_attachment_path, actor0_path=coneMesh.GetPath(), actor1_path=sphereMesh.GetPath())
            self._start()
            self._step(1)
            self._reset()
