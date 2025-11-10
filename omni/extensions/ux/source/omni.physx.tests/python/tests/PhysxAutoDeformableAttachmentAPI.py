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
import omni.physx.scripts.utils as core_utils
from omni.usd.commands.usd_commands import DeletePrimsCommand


class PhysicsDeformableAttachmentTestAsync(PhysicsBaseAsyncTestCase):
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
        self._stage = await utils.new_stage_setup(def_up_and_mpu=True, up=UsdGeom.Tokens.z, mpu=1.0)
        sphereLight = UsdLux.SphereLight.Define(self._stage, Sdf.Path("/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        self._upAxis = UsdGeom.GetStageUpAxis(self._stage)
        self._defaultPrimPath = self._stage.GetDefaultPrim().GetPath()

        # add physics scene
        omni.kit.commands.execute("AddPhysicsScene", stage=self._stage, path='/World/PhysicsScene')

    async def base_terminate(self):
        pass

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

    def add_groundplane(self):
        AddGroundPlaneCommand.execute(self._stage, '/CollisionPlane', self._upAxis, 1000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    @staticmethod
    def create_transform(translate = Gf.Vec3d(0.0),
                                     rotate = Gf.Rotation(Gf.Quatd(1.0)),
                                     scale = Gf.Vec3d(1.0),
                                     pivot_pos = Gf.Vec3d(0.0),
                                     pivot_orient = Gf.Rotation(Gf.Quatd(1.0))):
        return Gf.Transform(translate, rotate, scale, pivot_pos, pivot_orient)

    @staticmethod
    def set_prim_translation(prim: Usd.Prim, translate_vec: Gf.Vec3d):
        translate_mtx = Gf.Matrix4d().SetTranslate(translate_vec)
        omni.kit.commands.execute("TransformPrim", path=prim.GetPath(), new_transform_matrix=translate_mtx)

    def create_mesh_prim(self, prim_type : str, path : Sdf.Path) -> list:
        omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type=prim_type, prim_path=path)
        mesh = UsdGeom.Mesh.Get(self._stage, path)
        self.assertTrue(mesh)
        return mesh

    def create_deformable_actor(self, deformable_type, starting_height = 0.0):
        root_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/deformable", True))
        xform = UsdGeom.Xform.Define(self._stage, root_path)
        xform.AddTransformOp().Set(self.create_transform().GetMatrix())
        mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(root_path) + "/Cone", True))
        mesh = self.create_mesh_prim('Cone', mesh_path)

        if deformable_type == 'volume':
            sim_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(root_path) + "/sim_mesh", True))
            coll_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(root_path) + "/coll_mesh", True))
            success = omni.kit.commands.execute("CreateAutoVolumeDeformableHierarchy",
                root_prim_path=root_path,
                simulation_tetmesh_path=sim_mesh_path,
                collision_tetmesh_path=coll_mesh_path,
                cooking_src_mesh_path=mesh_path,
                simulation_hex_mesh_enabled=True,
                cooking_src_simplification_enabled=True
            )
        elif deformable_type == 'surface':
            sim_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(root_path) + "/sim_mesh", True))
            success = omni.kit.commands.execute("CreateAutoSurfaceDeformableHierarchy",
                root_prim_path=root_path,
                simulation_mesh_path=sim_mesh_path,
                cooking_src_mesh_path=mesh_path,
                cooking_src_simplification_enabled=False
            )

        self.set_prim_translation(xform.GetPrim(), Gf.Vec3d(0, 0, starting_height))
        get_physx_cooking_interface().cook_auto_deformable_body(str(root_path))
        return xform

    def create_xform_actor(self, xform_type, starting_height = 0.0, child_xform_height = 0.0):
        xform_actor = None
        starting_pos = Gf.Vec3d(0, 0, starting_height)

        if 'mesh' in xform_type:
            if 'parentXform' in xform_type:
                path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/xform/Cube", True))
                xform_actor = self.create_mesh_prim('Cube', path)
            else:
                path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/Cube", True))
                xform_actor = self.create_mesh_prim('Cube', path)
            self.set_prim_translation(xform_actor.GetPrim(), starting_pos)
        elif 'shape' in xform_type:
            if 'parentXform' in xform_type:
                xform_actor = omni.physx.scripts.physicsUtils.add_cube(self._stage, "/xform/cubeShape", 1.0, starting_pos)
            else:
                xform_actor = omni.physx.scripts.physicsUtils.add_cube(self._stage, "/cubeShape", 1.0, starting_pos)
        elif 'xform' in xform_type:
            path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/xform", True))
            xform = UsdGeom.Xform.Define(self._stage, path)
            xform.AddTransformOp().Set(self.create_transform().GetMatrix())
            xform_actor = xform
            self.set_prim_translation(xform_actor.GetPrim(), starting_pos)

        if 'childXform' in xform_type:
            path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(xform_actor.GetPath()) + "/xform", True))
            xform = UsdGeom.Xform.Define(self._stage, path)
            translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, 0, child_xform_height))
            xform_actor = xform
            omni.kit.commands.execute("TransformPrim", path=xform.GetPath(), new_transform_matrix=translate_mtx)

        if 'rigid' in xform_type:
            SetRigidBodyCommand.execute(path=xform_actor.GetPath(), approximationShape="convexHull", kinematic=True)
        if 'collider' in xform_type:
            SetStaticColliderCommand.execute(path=xform_actor.GetPath(), approximationShape="convexHull")

        return xform_actor

    async def do_delete_undo(self, target_path: Sdf.Path, deformable_sim: UsdGeom.PointBased, starting_height):
        # delete attachment
        deletePrimsCmd = DeletePrimsCommand(paths=[target_path])
        deletePrimsCmd.do()

        self._step(30)

        # target removed, deformable will fall to the ground
        translate_height = core_utils.get_world_position(self._stage, deformable_sim.GetPath())[2]
        self.assertTrue(translate_height < (starting_height / 2.0))

        deletePrimsCmd.undo()
        await self.wait(1)

        self._step(50)

        # target restored, deformable will reattach to xform actor
        translate_height = core_utils.get_world_position(self._stage, deformable_sim.GetPath())[2]
        self.assertTrue(translate_height > (starting_height / 2.0))


    async def test_vtx_xform_attachments_basic(self):
    # Attachments are connected directly to the rigidbody/collider
        for test_type in ['attachment', 'move', 'delete_undo_attachment', 'delete_undo_xform_actor', 'undo_redo']:
            for deformable_type in ['volume', 'surface']:
                for xform_type in ['mesh_rigid', 'mesh_collider', 'shape_rigid', 'shape_collider', 'xform']:
                    await self.base_setup()
                    starting_height = 1.7

                    deformable_actor = self.create_deformable_actor(deformable_type, starting_height)
                    translate_vec = deformable_actor.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
                    self.assertTrue(translate_vec[2] == starting_height)
                    deformable_sim = UsdGeom.PointBased(self._stage.GetPrimAtPath(str(deformable_actor.GetPath()) + "/sim_mesh"))

                    xform_actor = self.create_xform_actor(xform_type, starting_height + 0.6)

                    target_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/deformableAttachment", True))
                    omni.kit.commands.execute("CreateAutoDeformableAttachment",
                                                target_attachment_path=target_attachment_path,
                                                attachable0_path=deformable_actor.GetPath(),
                                                attachable1_path=xform_actor.GetPath())
                    await self.wait(1)

                    self._start()
                    self._step(30)

                    if test_type == 'attachment':
                        # if the deformable is attached, it will not fall to the ground
                        translate_vec = core_utils.get_world_position(self._stage, deformable_sim.GetPath())
                        self.assertTrue(translate_vec[2] > (starting_height / 2.0))
                    elif test_type == 'move':
                        # move xform actor
                        translate_mtx = Gf.Matrix4d()
                        if 'shape' in xform_type:
                            translate_mtx.SetScale(Gf.Vec3d(1.0))
                        move_delta = 0.5
                        translate_mtx.SetTranslateOnly(Gf.Vec3d(move_delta, 0, starting_height + 0.6))
                        omni.kit.commands.execute("TransformPrim", path=xform_actor.GetPath(), new_transform_matrix=translate_mtx)

                        self._step(50)

                        # if the deformable is attached, it will move with the xform actor
                        translate_vec = core_utils.get_world_position(self._stage, deformable_sim.GetPath())
                        self.assertTrue(translate_vec[0] > (move_delta / 2.0))
                    elif test_type == 'delete_undo_attachment':
                        await self.do_delete_undo(target_attachment_path, deformable_sim, starting_height)
                    elif test_type == 'delete_undo_xform_actor':
                        await self.do_delete_undo(xform_actor.GetPath(), deformable_sim, starting_height)
                    elif test_type == 'undo_redo':
                        # test undo and redo
                        # this test is an edge case because this sequence of events is highly unusual
                        omni.kit.undo.undo()
                        await self.wait(1)

                        self._step(30)

                        # undo attachment, deformable will fall to the ground
                        translate_vec = core_utils.get_world_position(self._stage, deformable_sim.GetPath())
                        self.assertTrue(translate_vec[2] < (starting_height / 2.0))

                        omni.kit.undo.redo()
                        await self.wait(1)

                        self._step(30)

                        # redo attachment, a new attachment is recreated based on the new position of the deformable
                        # since the deformable has fallen too far from the xform actor, no attachment points are computed
                        # the exception is attachment to a xform (world attachment) and in this case, the attachment points are computed based on the current position of the deformable actor
                        translate_vec = core_utils.get_world_position(self._stage, deformable_sim.GetPath())
                        if xform_type == 'xform':
                            self.assertTrue(translate_vec[2] > 0)
                        else:
                            self.assertTrue(translate_vec[2] < 0)


    async def test_vtx_xform_attachments_with_child_xform(self):
    # Attachments are connected to a child xform of the rigidbody/collider
        for test_type in ['attachment', 'move', 'delete_undo_attachment', 'delete_undo_xform_actor']:
            for deformable_type in ['volume', 'surface']:
                for xform_type in ['mesh_rigid_childXform', 'mesh_collider_childXform', 'shape_rigid_childXform', 'shape_collider_childXform']:
                    await self.base_setup()
                    starting_height = 1.7

                    deformable_actor = self.create_deformable_actor(deformable_type, starting_height)
                    translate_vec = deformable_actor.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
                    self.assertTrue(translate_vec[2] == starting_height)
                    deformable_sim = UsdGeom.PointBased(self._stage.GetPrimAtPath(str(deformable_actor.GetPath()) + "/sim_mesh"))

                    xform_actor = self.create_xform_actor(xform_type, starting_height + 130, child_xform_height = -1)

                    target_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/deformableAttachment", True))
                    omni.kit.commands.execute("CreateAutoDeformableAttachment",
                                                target_attachment_path=target_attachment_path,
                                                attachable0_path=deformable_actor.GetPath(),
                                                attachable1_path=xform_actor.GetPath())
                    await self.wait(1)

                    self._start()
                    self._step(30)

                    if test_type == 'attachment':
                        # if the deformable is attached, it will not fall to the ground
                        translate_vec = core_utils.get_world_position(self._stage, deformable_sim.GetPath())
                        self.assertTrue(translate_vec[2] > (starting_height / 2.0))
                    elif test_type == 'move':
                        # move xform actor
                        move_delta = 0.5
                        translate_mtx = Gf.Matrix4d().SetTranslate(Gf.Vec3d(move_delta, 0, -1))
                        omni.kit.commands.execute("TransformPrim", path=xform_actor.GetPath(), new_transform_matrix=translate_mtx)

                        self._step(50)

                        # if the deformable is attached, it will move with the xform actor
                        translate_vec = core_utils.get_world_position(self._stage, deformable_sim.GetPath())
                        self.assertTrue(translate_vec[0] > (move_delta / 2.0))

                        # move parent actor
                        if 'shape' in xform_type:
                            translate_mtx.SetScale(Gf.Vec3d(1.0))
                        move_delta = 0.5
                        translate_mtx.SetTranslateOnly(Gf.Vec3d(move_delta, 0, starting_height + 1.3))
                        omni.kit.commands.execute("TransformPrim", path=xform_actor.GetPrim().GetParent().GetPath(), new_transform_matrix=translate_mtx)

                        self._step(50)

                        # if the deformable is attached, it will move with the parent actor
                        translate_vec = core_utils.get_world_position(self._stage, deformable_sim.GetPath())
                        self.assertTrue(translate_vec[0] > move_delta)
                    elif test_type == 'delete_undo_attachment':
                        await self.do_delete_undo(target_attachment_path, deformable_sim, starting_height)
                    elif test_type == 'delete_undo_xform_actor':
                        await self.do_delete_undo(xform_actor.GetPath(), deformable_sim, starting_height)


    async def test_vtx_xform_attachments_rigid_with_multiple_colliders(self):
    # Attachments are connected to a collider of a rigidbody with multiple colliders
        for test_type in ['attachment', 'move', 'delete_undo_attachment', 'delete_undo_xform_actor']:
            for deformable_type in ['volume', 'surface']:
                for collider0_type in ['shape_collider_parentXform', 'mesh_collider_parentXform']:
                    for collider1_type in ['shape_collider_parentXform', 'mesh_collider_parentXform']:
                        await self.base_setup()
                        starting_height = 1.7

                        deformable_actor = self.create_deformable_actor(deformable_type, starting_height)
                        translate_vec = deformable_actor.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
                        self.assertTrue(translate_vec[2] == starting_height)
                        deformable_sim = UsdGeom.PointBased(self._stage.GetPrimAtPath(str(deformable_actor.GetPath()) + "/sim_mesh"))

                        root_actor = self.create_xform_actor('xform_rigid', 0)
                        non_attached_actor = self.create_xform_actor(collider0_type, starting_height + 1.8)
                        xform_actor = self.create_xform_actor(collider1_type, starting_height + 0.6)

                        target_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/deformableAttachment", True))
                        omni.kit.commands.execute("CreateAutoDeformableAttachment",
                                                    target_attachment_path=target_attachment_path,
                                                    attachable0_path=deformable_actor.GetPath(),
                                                    attachable1_path=xform_actor.GetPath())
                        await self.wait(1)

                        self._start()
                        self._step(30)

                        if test_type == 'attachment':
                            # if the deformable is attached, it will not fall to the ground
                            translate_vec = core_utils.get_world_position(self._stage, deformable_sim.GetPath())
                            self.assertTrue(translate_vec[2] > (starting_height / 2.0))
                        elif test_type == 'move':
                            # move xform actor
                            translate_mtx = Gf.Matrix4d()
                            if 'shape' in collider1_type:
                                translate_mtx.SetScale(Gf.Vec3d(1.0))
                            move_delta = 0.5

                            translate_mtx.SetTranslateOnly(Gf.Vec3d(move_delta, 0, starting_height + 0.6))
                            omni.kit.commands.execute("TransformPrim", path=xform_actor.GetPath(), new_transform_matrix=translate_mtx)

                            self._step(50)

                            # if the deformable is attached, it will move with the xform actor
                            translate_vec = core_utils.get_world_position(self._stage, deformable_sim.GetPath())
                            self.assertTrue(translate_vec[0] > (move_delta / 2.0))

                            # move root actor
                            move_delta = 0.5
                            translate_mtx.SetTranslate(Gf.Vec3d(move_delta, 0, 0))
                            omni.kit.commands.execute("TransformPrim", path=root_actor.GetPrim().GetPath(), new_transform_matrix=translate_mtx)

                            self._step(50)

                            # if the deformable is attached, it will move with the root actor
                            translate_vec = core_utils.get_world_position(self._stage, deformable_sim.GetPath())
                            self.assertTrue(translate_vec[0] > move_delta)

                            # move collider 0 which should not have any effect on attachment points
                            if 'shape' in collider0_type:
                                translate_mtx.SetScale(Gf.Vec3d(1.0))
                            move_negative_delta = -1.0
                            translate_mtx.SetTranslateOnly(Gf.Vec3d(move_negative_delta, 0, starting_height + 1.8))
                            omni.kit.commands.execute("TransformPrim", path=non_attached_actor.GetPrim().GetPath(), new_transform_matrix=translate_mtx)

                            self._step(50)

                            # if the deformable is attached, it will not move with the non attached actor
                            translate_vec = core_utils.get_world_position(self._stage, deformable_sim.GetPath())
                            self.assertTrue(translate_vec[0] > move_delta)
                        elif test_type == 'delete_undo_attachment':
                            await self.do_delete_undo(target_attachment_path, deformable_sim, starting_height)
                        elif test_type == 'delete_undo_xform_actor':
                            await self.do_delete_undo(xform_actor.GetPath(), deformable_sim, starting_height)


    async def test_deformable_deformable_attachments(self):
        for test_type in ['attachment', 'delete_undo_attachment', 'delete_undo_deformable_actor']:
            for deformable0_type in ['volume', 'surface']:
                for deformable1_type in ['volume', 'surface']:
                    # surface/surface attachment is not supported yet
                    if deformable0_type == 'surface' and deformable1_type == 'surface':
                        continue

                    await self.base_setup()
                    starting_height = 1.7

                    deformable0_actor = self.create_deformable_actor(deformable0_type, starting_height + 1.0)
                    deformable1_actor = self.create_deformable_actor(deformable1_type, starting_height)
                    #set smaller mass for bottom deformable, so we don't get that much stretching
                    deformable0_actor.GetPrim().GetAttribute("omniphysics:mass").Set(1.0)
                    deformable1_actor.GetPrim().GetAttribute("omniphysics:mass").Set(0.1)
                    translate_vec = deformable1_actor.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
                    self.assertTrue(translate_vec[2] == starting_height)
                    deformable1_sim = UsdGeom.PointBased(self._stage.GetPrimAtPath(str(deformable1_actor.GetPath()) + "/sim_mesh"))

                    xform_actor = self.create_xform_actor('xform', starting_height + 1.5)

                    xform_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/xformAttachment", True))
                    omni.kit.commands.execute("CreateAutoDeformableAttachment",
                                                target_attachment_path=xform_attachment_path,
                                                attachable0_path=deformable0_actor.GetPath(),
                                                attachable1_path=xform_actor.GetPath())

                    deformable_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/deformableAttachment", True))
                    omni.kit.commands.execute("CreateAutoDeformableAttachment",
                                                target_attachment_path=deformable_attachment_path,
                                                attachable0_path=deformable0_actor.GetPath(),
                                                attachable1_path=deformable1_actor.GetPath())

                    # set attachment offset
                    scope = UsdGeom.Scope.Define(self._stage, deformable_attachment_path)
                    scope.GetPrim().ApplyAPI("PhysxAutoDeformableAttachmentAPI")
                    scope.GetPrim().GetAttribute("physxAutoDeformableAttachment:deformableVertexOverlapOffset").Set(0.01)
                    scope.GetPrim().GetAttribute("physxAutoDeformableAttachment:collisionFilteringOffset").Set(0.5)
                    await self.wait(1)

                    self._start()
                    self._step(30)

                    if test_type == 'attachment':
                        # if the deformable is attached, it will not fall to the ground
                        translate_vec = core_utils.get_world_position(self._stage, deformable1_sim.GetPath())
                        self.assertTrue(translate_vec[2] > (starting_height / 3.0))
                    elif test_type == 'delete_undo_attachment':
                        await self.do_delete_undo(deformable_attachment_path, deformable1_sim, starting_height)
                    elif test_type == 'delete_undo_deformable_actor':
                        await self.do_delete_undo(deformable0_actor.GetPath(), deformable1_sim, starting_height)
