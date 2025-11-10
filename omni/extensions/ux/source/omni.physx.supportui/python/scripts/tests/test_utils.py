# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import os
import carb
import omni.ext
from omni.kit.test.async_unittest import AsyncTestCase
import omni.kit.undo
from omni.kit.widget.layers.layer_settings import LayerSettings
import omni.ui as ui
import asyncio
from omni.physx.scripts import physicsUtils
from omni.physx.scripts import utils as baseUtils
from omni.physxtests import utils
from omni.physx import get_physxunittests_interface
import omni.physx.bindings._physx as pb
from omni.physxsupportui import get_physx_supportui_interface
import omni.physxsupportui.bindings._physxSupportUi as pxsupportui
from pxr import PhysicsSchemaTools, PhysxSchema, UsdPhysics, Gf, UsdGeom
from timeit import default_timer as timer
from ..utils import ui_wait


class SupportUiAsyncTestCase(AsyncTestCase):
    def setUp(self):
        super().setUp()
        self._stepper = utils.PhysicsStepper()
        self._stage = None
        self._supportui_event_sub = None

        self._static_simplifications_type_token_dict = {
            int(pxsupportui.SupportUiStaticColliderSimplificationType.NONE):
                UsdPhysics.Tokens.none,
            int(pxsupportui.SupportUiStaticColliderSimplificationType.MESH):
                UsdPhysics.Tokens.meshSimplification,
        }
        self._dynamic_simplifications_type_token_dict = {
            int(pxsupportui.SupportUiDynamicColliderSimplificationType.CONVEX_HULL):
                UsdPhysics.Tokens.convexHull,
            int(pxsupportui.SupportUiDynamicColliderSimplificationType.CONVEX_DECOMPOSITION):
                UsdPhysics.Tokens.convexDecomposition,
            int(pxsupportui.SupportUiDynamicColliderSimplificationType.SDF):
                PhysxSchema.Tokens.sdf,
        }

        settings = carb.settings.get_settings()

        # enable supportui toolbar
        settings.set_bool(pxsupportui.SETTINGS_ACTION_BAR_ENABLED, True)

        # disable triggering of cooking
        self._cooking_bak = settings.get_as_bool(pxsupportui.SETTINGS_ASYNC_COOKING_AT_STAGE_LOAD)
        settings.set_bool(pxsupportui.SETTINGS_ASYNC_COOKING_AT_STAGE_LOAD, False)

        # disable avoid changing existing colliders
        self._avoid_changing_existing_coll_bak = settings.get_as_bool(pxsupportui.SETTINGS_AVOID_CHANGING_EXISTING_COLLIDERS)
        settings.set_bool(pxsupportui.SETTINGS_AVOID_CHANGING_EXISTING_COLLIDERS, False)

        # disable any collider debug visualization
        settings.set_int(pb.SETTING_DISPLAY_COLLIDERS, 0)

    async def tearDown(self):
        await self._close_stage()
        await utils.new_stage_setup()  # TODO FIXME HACK: this prevents crash at test exit (because of dangling SDF VtValues)

        self._stepper = None

        settings = carb.settings.get_settings()
        settings.set_bool(pxsupportui.SETTINGS_ASYNC_COOKING_AT_STAGE_LOAD, self._cooking_bak)
        settings.set_bool(pxsupportui.SETTINGS_AVOID_CHANGING_EXISTING_COLLIDERS, self._avoid_changing_existing_coll_bak)

        self._static_simplifications_type_token_dict = None
        self._dynamic_simplifications_type_token_dict = None
        super().tearDown()

    async def create_simple_test_stage(self, num_static=1, num_dynamic=1):
        stage = await utils.new_stage_setup()
        up_axis = UsdGeom.GetStageUpAxis(stage)
        up_offset = baseUtils.getAxisAlignedVector(up_axis, 25)
        for i in range(num_static):
            physicsUtils.add_cube(stage, f"/Cube{i}", Gf.Vec3f(10.0), up_offset)
        up_offset = baseUtils.getAxisAlignedVector(up_axis, 15)
        for i in range(num_dynamic):
            physicsUtils.add_rigid_cube(stage, f"/CubeActor{i}", Gf.Vec3f(10.0), up_offset)
        physicsUtils.add_ground_plane(stage, "/GroundPlane", up_axis, 50, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
        utils.physics_scene_setup(stage)
        self._stage = stage
        self.assertIsNotNone(self._stage)

    def get_physics_stats(self):
        return get_physxunittests_interface().get_physics_stats()

    def _on_supportui_event(self, event):
        def decode_prim_path():
            prim_path_encoded = event.payload['primPath']
            prim_path = PhysicsSchemaTools.decodeSdfPath(prim_path_encoded[0], prim_path_encoded[1])
            return prim_path

        def check_coll_simplification_type(prim_path, coll_type, simplification_type):
            if coll_type is False:
                simplifications_type_token_dict = self._static_simplifications_type_token_dict
            else:
                simplifications_type_token_dict = self._dynamic_simplifications_type_token_dict
            if simplifications_type_token_dict is None:
                return False
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                return False
            mesh_api = UsdPhysics.MeshCollisionAPI.Get(stage, prim_path)
            if mesh_api is not None:
                approxToken = mesh_api.GetApproximationAttr().Get()
                if approxToken is not None and approxToken == simplifications_type_token_dict[simplification_type]:
                    return True
            return False

        if event.type == int(pxsupportui.SupportUiEventType.COLLIDER_CREATED):
            coll_type = event.payload['colliderType']
            simplification_type = event.payload['simplificationType']
            num_remaining_coll_tasks = event.payload['numRemainingCollTasks']
            num_total_coll_tasks = event.payload['numTotalCollTasks']
            prim_path = decode_prim_path()

            # double check that the collider type created is what was intended
            self.assertTrue(check_coll_simplification_type(prim_path, coll_type, simplification_type))

            # add to the creation dict: {prim_path (string) : simplification_type (int from C++ enum)}
            target_map = self._colliders_static_created if coll_type is False else self._colliders_dynamic_created
            target_map[str(prim_path)] = simplification_type

            # signal coll. creation has finished
            if (num_remaining_coll_tasks == 0 and not self._task_finished.done()):
                self._task_finished.set_result(True)

        elif event.type == int(pxsupportui.SupportUiEventType.RIGID_BODY_CREATED):
            self._rigid_bodies_created.add(str(decode_prim_path()))

        elif event.type == int(pxsupportui.SupportUiEventType.RIGID_BODY_REMOVED):
            self._rigid_bodies_removed.add(str(decode_prim_path()))

    async def _load_supportui_test_scene(self, filename):
        self._supportui_event_sub = None
        self._colliders_static_created = dict()
        self._colliders_dynamic_created = dict()
        self._rigid_bodies_created = set()
        self._rigid_bodies_removed = set()
        self._task_finished = asyncio.Future()

        event_stream = get_physx_supportui_interface().get_event_stream()
        self.assertIsNotNone(event_stream)
        self._supportui_event_sub = event_stream.create_subscription_to_pop(self._on_supportui_event)

        data_path = "../../../../../data/tests/SupportUiTests"
        tests_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, data_path)))
        tests_folder = tests_folder.replace("\\", "/") + "/"
        file_path = tests_folder + filename

        usd_context = omni.usd.get_context()
        await usd_context.open_stage_async(file_path)
        self._stage = usd_context.get_stage()
        self.assertIsNotNone(self._stage)

    async def _close_stage(self):
        omni.usd.get_context().get_selection().clear_selected_prim_paths()
        await ui_wait()
        self._supportui_event_sub = None
        self._stage = None

    async def _step(self, min_steps=5, timeout=5):
        await self._stepper.play_and_step_and_pause(min_steps, timeout)

    async def _pump(self, wait_for_coll_task=False):
        if wait_for_coll_task:
            timeout = 10  # in seconds
            start = timer()
            while timeout > 0 and not self._task_finished.done():
                await omni.kit.app.get_app().next_update_async()
                timeout -= timer() - start
        await self._step()

    async def _select_and_focus_property_window(self, paths):
        # select prim and focus property window so the widgets are built
        omni.usd.get_context().get_selection().set_selected_prim_paths(paths, False)
        window = ui.Workspace.get_window("Property")
        window.focus()
        await ui_wait(20)

    async def _command_test(
            self,
            stage_filename,
            prim_paths,
            cmd,
            custom_exe,
            ph_stats_after_cmd,
            additional_checks=None,
            wait_for_coll_task=True
    ):
        auto_coll_bak = carb.settings.get_settings().get_as_bool(pxsupportui.SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED)
        carb.settings.get_settings().set_bool(pxsupportui.SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED, False)

        await self._load_supportui_test_scene(stage_filename)
        await self._pump()
        exp_init_stats = self.get_physics_stats()

        if (custom_exe is not None):
            custom_exe(prim_paths)
        utils.execute_and_check(self, cmd, stage=self._stage, prim_paths=prim_paths)
        await self._pump(wait_for_coll_task)
        exp_stats_after_cmd = self.get_physics_stats()
        utils.check_stats(self, ph_stats_after_cmd)

        if additional_checks is not None:
            self.assertTrue(
                additional_checks(
                    self._colliders_static_created,
                    self._colliders_dynamic_created,
                    self._rigid_bodies_created,
                    self._rigid_bodies_removed,
                    [prim.GetPath() for prim in self._stage.Traverse() if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
                )
            )

        omni.kit.undo.undo()
        await self._pump()
        utils.check_stats(self, exp_init_stats)

        omni.kit.undo.redo()
        await self._pump()
        utils.check_stats(self, exp_stats_after_cmd)

        carb.settings.get_settings().set_bool(pxsupportui.SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED, auto_coll_bak)

        await self._close_stage()

    async def _after_load_test(self, stage_filename, ph_stats_after_load, additional_checks=None):
        await self._load_supportui_test_scene(stage_filename)
        await self._pump(True)
        exp_after_load_stats = self.get_physics_stats()
        utils.check_stats(self, ph_stats_after_load)
        if additional_checks is not None:
            self.assertTrue(
                additional_checks(
                    self._colliders_static_created,
                    self._colliders_dynamic_created,
                    self._rigid_bodies_created,
                    self._rigid_bodies_removed,
                    [prim.GetPath() for prim in self._stage.Traverse() if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
                )
            )
        await self._close_stage()
