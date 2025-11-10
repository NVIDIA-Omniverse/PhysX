# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import unittest
import omni.usd
import omni.kit.ui_test as ui_test
import omni.physxsupportui.bindings._physxSupportUi as pxsupportui
from omni.physxsupportui import get_physx_supportui_interface
from pxr import PhysicsSchemaTools, UsdPhysics
from omni.physxtests.utils.physicsBase import PhysicsBaseAsyncTestCase
from .test_utils import SupportUiAsyncTestCase
from ..utils import ui_wait

from .tests_inspector import *

class SupportUiTests(SupportUiAsyncTestCase):
    @unittest.skip("crash at exit")
    async def test_ui_property_list_refresh(self):
        def get_prop_frames():
            physics_frame = ui_test.find("Property//Frame/**/CollapsableFrame[*].title=='Physics'")
            top_stack = physics_frame.find("*/ZStack[0]/VStack[0]/*/VStack[0]")
            collider_frame = top_stack.find("*/ZStack[0]/CollapsableFrame[*].title=='Collider'")
            rb_frame = top_stack.find("*/ZStack[0]/CollapsableFrame[*].title=='Rigid Body'")
            return collider_frame, rb_frame

        await self.create_simple_test_stage()
        await ui_wait(5)

        # select a prim
        prim_paths = ["/World/Cube0"]
        await self._select_and_focus_property_window(prim_paths)

        # create a dynamic collider together with a rigid body
        for path in prim_paths:
            get_physx_supportui_interface().create_colliders(
                PhysicsSchemaTools.sdfPathToInt(path),
                pxsupportui.SupportUiColliderType.DYNAMIC
            )
        omni.kit.commands.execute("CreateCollidersCommand", stage=self._stage, prim_paths=prim_paths)
        await ui_wait(20)

        # check if collider and rigid body section exists in the property list
        collider_frame, rb_frame = get_prop_frames()
        self.assertTrue(
            collider_frame is not None and collider_frame._widget.visible is True,
            "Collider section must be present in the property list!"
        )
        self.assertTrue(
            rb_frame is not None and rb_frame._widget.visible is True,
            "Rigid Body section must be present in the property list!"
        )

        # now check refresh after physics components removal
        omni.kit.commands.execute("ClearPhysicsComponentsCommand", stage=self._stage, prim_paths=prim_paths)
        await ui_wait(20)

        # check if collider and rigid body section now does not exist in the property list
        collider_frame, rb_frame = get_prop_frames()
        self.assertTrue(
            collider_frame is None or collider_frame._widget.visible is False,
            "Collider section must NOT be present in the property list!"
        )
        self.assertTrue(
            rb_frame is None or rb_frame._widget.visible is False,
            "Rigid Body section must NOT be present in the property list!"
        )

        await self._close_stage()

    async def _test_colliders_creation(
        self,
        scene_name,
        paths,
        collider_type,
        simplification_type,
        exp_stats,
        additional_checks=None
    ):
        def create_colliders(prim_paths):
            for path in prim_paths:
                get_physx_supportui_interface().create_colliders(PhysicsSchemaTools.sdfPathToInt(path), collider_type)

        settings = carb.settings.get_settings()
        simplification_type_bak = settings.get_as_int(pxsupportui.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE)

        if collider_type == pxsupportui.SupportUiColliderType.STATIC:
            settings.set_int(pxsupportui.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE, simplification_type)
        else:
            settings.set_int(pxsupportui.SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE, simplification_type)

        await self._command_test(
            scene_name,
            paths,
            "CreateCollidersCommand",
            lambda prim_paths: create_colliders(prim_paths),
            exp_stats,
            additional_checks
        )

        if collider_type == pxsupportui.SupportUiColliderType.STATIC:
            settings.set_int(pxsupportui.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE, simplification_type_bak)
        else:
            settings.set_int(pxsupportui.SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE, simplification_type_bak)

    # "TestScene.usda" prims legend:
    #  {'numDynamicRigids': 2, 'numStaticRigids': 3, 'numConvexShapes': 1, 'numTriMeshShapes': 3, 'numBoxShapes': 0})
    # - "/World/Cylinder" - no RB and no Coll
    # - "/World/Xform/Cube" - RB and dynamic coll. type - convex decomposition
    # - "/World/Cube2" - no RB and static coll. type - mesh simplification
    # - "/World/Cube3" - no RB and static coll. type - triangle mesh
    # - "/World/Sphere" - RB and no collision

    async def test_static_colliders_creation(self):  # also includes conversion test from dynamic to static
        scene_name = "TestScene.usda"
        prims = ["/World/Cylinder", "/World/Xform/Cube", "/World/Cube2", "/World/Cube3", "/World/Plane"]
        collider_type = pxsupportui.SupportUiColliderType.STATIC
        exp_stats = {'numDynamicRigids': 0, 'numStaticRigids': 6, 'numConvexShapes': 0, 'numTriMeshShapes': 6, 'numBoxShapes': 0}
        simplification_type = int(pxsupportui.SupportUiStaticColliderSimplificationType.NONE)

        def add_check(sta_coll, dyn_coll, rb_add, rb_rem, rb_prim_paths):
            return (
                len(sta_coll) == 4
                and all(st == simplification_type for st in sta_coll.values())
                and len(dyn_coll) == 0
                and len(rb_add) == 0
                and len(rb_rem) == 2
                and len(rb_prim_paths) == 0
            )

        await self._test_colliders_creation(scene_name, prims, collider_type, simplification_type, exp_stats, add_check)

        simplification_type = int(pxsupportui.SupportUiStaticColliderSimplificationType.MESH)
        await self._test_colliders_creation(scene_name, prims, collider_type, simplification_type, exp_stats, add_check)

    async def test_dynamic_colliders_creation(self):  # also includes conversion test from static to dynamic
        scene_name = "TestScene2.usda"
        prims = ["/World/Xform", "/World/Box", "/World/_PhysicsScene", "/World/_SphereLight", "/World/Cylinder"]
        collider_type = pxsupportui.SupportUiColliderType.DYNAMIC
        exp_stats = {'numDynamicRigids': 3, 'numStaticRigids': 1, 'numConvexShapes': 4, 'numTriMeshShapes': 1, 'numBoxShapes': 1}
        simplification_type = int(pxsupportui.SupportUiDynamicColliderSimplificationType.CONVEX_HULL)
        # the following prims must contain (newly created) rigid bodies
        rb_on_prims = ["/World/Xform", "/World/Box", "/World/Cylinder"]

        def add_check(sta_coll, dyn_coll, rb_add, rb_rem, rb_prim_paths):
            return (
                len(sta_coll) == 0
                and len(dyn_coll) == 5
                and all(st == simplification_type for st in dyn_coll.values())
                and len(rb_add) == 3
                and len(rb_rem) == 0
                and len(rb_prim_paths) == len(rb_on_prims) and sorted(rb_prim_paths) == sorted(rb_on_prims)
            )

        await self._test_colliders_creation(scene_name, prims, collider_type, simplification_type, exp_stats, add_check)

        simplification_type = int(pxsupportui.SupportUiDynamicColliderSimplificationType.CONVEX_DECOMPOSITION)
        exp_stats = {'numDynamicRigids': 3, 'numStaticRigids': 1, 'numConvexShapes': 19, 'numTriMeshShapes': 1, 'numBoxShapes': 1}        
        await self._test_colliders_creation(scene_name, prims, collider_type, simplification_type, exp_stats, add_check)

        simplification_type = int(pxsupportui.SupportUiDynamicColliderSimplificationType.SDF)
        exp_stats = {'numDynamicRigids': 3, 'numStaticRigids': 1, 'numConvexShapes': 0, 'numTriMeshShapes': 5, 'numBoxShapes': 1}
        await self._test_colliders_creation(scene_name, prims, collider_type, simplification_type, exp_stats, add_check)

        # check the rigid body selection mode
        settings = carb.settings.get_settings()
        context = omni.usd.get_context()
        selection = context.get_selection()

        rb_sel_mode_bak = settings.get_as_bool(pxsupportui.SETTINGS_RIGID_BODY_SELECTION_MODE_ENABLED)
        settings.set_bool(pxsupportui.SETTINGS_RIGID_BODY_SELECTION_MODE_ENABLED, True)

        selection.clear_selected_prim_paths()
        selection.set_selected_prim_paths(prims, False)  # select all prims
        await ui_wait(5)

        # Now, only rb prims should be selected
        selected_prim_paths = selection.get_selected_prim_paths()
        self.assertTrue(sorted(rb_on_prims) == sorted(selected_prim_paths), "All prims with rigid bodies must get selected!")

        settings.set_bool(pxsupportui.SETTINGS_RIGID_BODY_SELECTION_MODE_ENABLED, rb_sel_mode_bak)

    async def test_auto_coll_creation_after_stage_open(self):
        scene_name = "TestScene3.usda"
        simplification_type_static = int(pxsupportui.SupportUiStaticColliderSimplificationType.MESH)
        simplification_type_dynamic = int(pxsupportui.SupportUiDynamicColliderSimplificationType.CONVEX_HULL)

        settings = carb.settings.get_settings()
        auto_coll_creation_bak = settings.get_as_bool(pxsupportui.SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED)
        simplification_type_static_bak = settings.get_as_int(pxsupportui.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE)
        simplification_type_dynamic_bak = settings.get_as_int(pxsupportui.SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE)

        settings.set_int(pxsupportui.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE, simplification_type_static)
        settings.set_int(pxsupportui.SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE, simplification_type_dynamic)
        settings.set_bool(pxsupportui.SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED, True)

        def add_check(sta_coll, dyn_coll, rb_add, rb_rem, rb_prim_paths):
            return (
                len(sta_coll) == 6
                and len(dyn_coll) == 0
                and all(st == simplification_type_static for st in sta_coll.values())
                and len(rb_add) == 0
                and len(rb_rem) == 0
                and len(rb_prim_paths) == 0
            )

        await self._after_load_test(
            scene_name,
            {'numDynamicRigids': 0, 'numStaticRigids': 6, 'numConvexShapes': 0, 'numTriMeshShapes': 5, 'numBoxShapes': 1},
            add_check
        )

        settings.set_bool(pxsupportui.SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED, auto_coll_creation_bak)
        settings.set_int(pxsupportui.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE, simplification_type_static_bak)
        settings.set_int(pxsupportui.SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE, simplification_type_dynamic_bak)

    async def test_avoid_changing_existing_colliders(self):

        def check_collider_types():
            for prim in self._stage.Traverse():
                prim_path = prim.GetPath().pathString
                if prim_path in all_prims_coll_dict.keys():
                    coll = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
                    #  print(f"{prim_path}: {coll}")
                    self.assertIsNotNone(coll)
                    self.assertTrue(coll == all_prims_coll_dict[prim_path])

        settings = carb.settings.get_settings()

        avoid_changing_existing_coll = settings.get_as_bool(pxsupportui.SETTINGS_AVOID_CHANGING_EXISTING_COLLIDERS)
        settings.set_bool(pxsupportui.SETTINGS_AVOID_CHANGING_EXISTING_COLLIDERS, True)

        # create a stage where we will have enough static and dynamic actors
        await self.create_simple_test_stage(
            len(self._static_simplifications_type_token_dict.keys()),
            len(self._dynamic_simplifications_type_token_dict.keys())
        )
        await ui_wait(1)

        all_prims_coll_dict = dict()  # all prims and expected collider types

        # create all types of colliders
        # static
        simplification_type_static_bak = settings.get_as_int(pxsupportui.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE)
        counter = 0
        for key, value in self._static_simplifications_type_token_dict.items():
            prim_path = f"/World/Cube{counter}"
            settings.set_int(pxsupportui.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE, key)
            get_physx_supportui_interface().create_colliders(
                PhysicsSchemaTools.sdfPathToInt(prim_path),
                pxsupportui.SupportUiColliderType.STATIC
            )
            all_prims_coll_dict[prim_path] = value
            counter += 1
        settings.set_int(pxsupportui.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE, simplification_type_static_bak)

        # dynamic
        simplification_type_dynamic_bak = settings.get_as_int(pxsupportui.SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE)
        counter = 0
        for key, value in self._dynamic_simplifications_type_token_dict.items():
            prim_path = f"/World/CubeActor{counter}"
            settings.set_int(pxsupportui.SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE, key)
            get_physx_supportui_interface().create_colliders(
                PhysicsSchemaTools.sdfPathToInt(prim_path),
                pxsupportui.SupportUiColliderType.DYNAMIC
            )
            all_prims_coll_dict[prim_path] = value
            counter += 1
        settings.set_int(pxsupportui.SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE, simplification_type_dynamic_bak)

        # finally all colliders will get created
        await ui_wait(1)
        # check if all colliders got created correctly
        check_collider_types()

        # convert all colliders to static
        for prim_path in all_prims_coll_dict.keys():
            get_physx_supportui_interface().create_colliders(
                PhysicsSchemaTools.sdfPathToInt(prim_path),
                pxsupportui.SupportUiColliderType.STATIC
            )
        await ui_wait(5)
        # all dynamic colliders are supported in static version so colliders should stay the same
        check_collider_types()

        # convert all colliders to dynamic
        for prim_path in all_prims_coll_dict.keys():
            get_physx_supportui_interface().create_colliders(
                PhysicsSchemaTools.sdfPathToInt(prim_path),
                pxsupportui.SupportUiColliderType.DYNAMIC
            )
        await ui_wait(5)

        # update the dictionary with expected result - all static (triangle and mesh simplification) unsupported
        new_dyn_coll_type = self._dynamic_simplifications_type_token_dict[simplification_type_dynamic_bak]
        for i in range(len(self._static_simplifications_type_token_dict.items())):
            all_prims_coll_dict[f"/World/Cube{i}"] = new_dyn_coll_type
        check_collider_types()

        # restore settings
        settings.set_bool(pxsupportui.SETTINGS_AVOID_CHANGING_EXISTING_COLLIDERS, avoid_changing_existing_coll)

        await self._close_stage()

    async def test_remove_physics_components(self):
        scene_name = "TestScene.usda"
        await self._command_test(
            scene_name,
            ["/World"],
            "ClearPhysicsComponentsCommand",
            None,
            {'numDynamicRigids': 0, 'numStaticRigids': 0, 'numConvexShapes': 0, 'numTriMeshShapes': 0, 'numBoxShapes': 0},
            None,
            False
        )

    async def test_rigid_body_manipulator_selector(self):
        from omni.kit.manipulator.selector.extension import get_manipulator_selector
        from omni.kit.manipulator.transform.settings_constants import c
        from ..rigid_body_transform_manipulator import RigidBodyTransformManipulator

        settings = carb.settings.get_settings()

        auto_coll_creation_bak = settings.get_as_bool(pxsupportui.SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED)
        settings.set_bool(pxsupportui.SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED, False)

        rb_manipulator_enabled_bak = settings.get_as_bool(pxsupportui.SETTINGS_CUSTOM_MANIPULATOR_ENABLED)
        settings.set_bool(pxsupportui.SETTINGS_CUSTOM_MANIPULATOR_ENABLED, True)

        await self.create_simple_test_stage()
        await self._step()

        module_name = 'omni.physxsupportui'
        context = omni.usd.get_context()
        selection = context.get_selection()

        def find_rb_manipulator():
            selector = get_manipulator_selector("")
            for k, v in selector._manipulators.items():
                if k == module_name:
                    for m in v:
                        if isinstance(m, RigidBodyTransformManipulator):
                            return m
            return None

        rb_manipulator = find_rb_manipulator()
        self.assertTrue(rb_manipulator is not None, "rigid body manipulator must exist and be registered to the system!")
        self.assertTrue(rb_manipulator._rigid_body_manipulator_enabled, "rigid body manipulator is supposed to be enabled now!")

        # make sure Move op is active
        settings.set(c.TRANSFORM_OP_SETTING, c.TRANSFORM_OP_MOVE)

        # select a rigid body and check current manipulator
        selection.set_selected_prim_paths(["/World/CubeActor0"], True)
        await self._step()
        self.assertTrue(rb_manipulator.enabled, "rigid body manipulator is supposed to be active now!")

        # check that manipulator gets disabled when SCALE op is active
        settings.set(c.TRANSFORM_OP_SETTING, c.TRANSFORM_OP_SCALE)
        await self._step()
        self.assertFalse(rb_manipulator.enabled, "rigid body manipulator should not be active in scaling mode!")
        settings.set(c.TRANSFORM_OP_SETTING, c.TRANSFORM_OP_ROTATE)
        await self._step()
        self.assertTrue(rb_manipulator.enabled, "rigid body manipulator is supposed to be active now!")
        settings.set(c.TRANSFORM_OP_SETTING, c.TRANSFORM_OP_MOVE)
        await self._step()
        self.assertTrue(rb_manipulator.enabled, "rigid body manipulator is supposed to be active now!")

        # select a non rigid body and check current manipulator
        selection.set_selected_prim_paths(["/World/Cube0"], True)
        await self._step()
        self.assertFalse(rb_manipulator.enabled, "rigid body manipulator should not be active now!")

        # turn off rb manipulator and select a rigid body and check current manipulator
        settings.set_bool(pxsupportui.SETTINGS_CUSTOM_MANIPULATOR_ENABLED, False)
        self.assertFalse(rb_manipulator._rigid_body_manipulator_enabled, "rigid body manipulator is supposed to be disabled now!")

        selection.set_selected_prim_paths(["/World/CubeActor0"], True)
        await self._step()
        self.assertFalse(rb_manipulator.enabled, "rigid body manipulator is supposed to be disabled now!")

        # turn rb manipulator back on and select a rigid body and check
        settings.set_bool(pxsupportui.SETTINGS_CUSTOM_MANIPULATOR_ENABLED, True)
        self.assertTrue(rb_manipulator._rigid_body_manipulator_enabled, "rigid body manipulator is supposed to be enabled now!")
        selection.clear_selected_prim_paths()

        selection.set_selected_prim_paths(["/World/Cube0"], True)
        await self._step()
        self.assertFalse(rb_manipulator.enabled, "rigid body manipulator should not be active now!")

        selection.set_selected_prim_paths(["/World/CubeActor0"], True)
        await self._step()
        self.assertTrue(rb_manipulator.enabled, "rigid body manipulator is supposed to be active now!")

        # turn off rb manipulator
        settings.set_bool(pxsupportui.SETTINGS_CUSTOM_MANIPULATOR_ENABLED, False)
        self.assertFalse(rb_manipulator._rigid_body_manipulator_enabled, "rigid body manipulator is supposed to be disabled now!")

        selection.clear_selected_prim_paths()
        await self._step()
        self.assertFalse(rb_manipulator.enabled, "rigid body manipulator should not be active now!")

        settings.set_bool(pxsupportui.SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED, auto_coll_creation_bak)
        settings.set_bool(pxsupportui.SETTINGS_CUSTOM_MANIPULATOR_ENABLED, rb_manipulator_enabled_bak)

        await self._close_stage()
