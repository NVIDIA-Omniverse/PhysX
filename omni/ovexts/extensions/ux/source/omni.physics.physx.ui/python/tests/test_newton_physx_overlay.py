# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.kit.ui_test as ui_test
import omni.kit.window.property as p
import omni.ui as ui
import omni.usd
from omni.kit.property.physics import Manager
from omni.kit.property.physics.widgets import MainFrameWidget
from omni.physx.scripts import physicsUtils
from omni.physxtests.utils.physicsBase import TestCategory
from omni.physxtestsvisual.utils import TestCase
from pxr import PhysxSchema, Sdf, Tf, UsdGeom, UsdPhysics
from carb.input import MouseEventType

from omni.physics.physxui.utils import DisableByCallbackBuilder


class NewtonPhysXOverlayTest(TestCase):
    category = TestCategory.Core

    async def _select_and_focus_property_window(self, paths):
        omni.usd.get_context().get_selection().set_selected_prim_paths(paths, False)
        ui.Workspace.get_window("Property").focus()
        await self.wait(20)

    async def _test_newton_physx_visual(self, prim, suffix, expand_advanced=False):
        await self._select_and_focus_property_window([prim.GetPath().pathString])
        await self.setup_docked_test("Property", width=790, height=2048)
        await self.wait(3)

        if expand_advanced:
            for title in ("Advanced", "Advanced - PhysX"):
                frame = ui_test.find(f"Property//Frame/**/CollapsableFrame[*].title=='{title}'")
                if frame is not None:
                    frame.widget.collapsed = False
            await self.wait(5)

        widget = p.get_window()._widgets_top[Manager.scheme][MainFrameWidget.name]
        target_height = widget._collapsable_frame.computed_height
        target_scroll = widget._collapsable_frame.screen_position_y

        self.force_resize_window("Property", 790, target_height)
        await self.wait(3)

        p.get_window()._window_frame.vertical_scrollbar_policy = ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF
        p.get_window()._window_frame.scroll_y = target_scroll
        await self.wait(20)

        self.force_resize_window("Property", 800, target_height)
        await ui_test.input.emulate_mouse(MouseEventType.MOVE, ui_test.Vec2(0, 0))
        await self.wait(20)

        return await self.do_visual_test(img_suffix=suffix)

    def _apply_api_schema(self, prim, schema_name):
        """Apply a codeless API schema directly via apiSchemas metadata, bypassing TfType lookup."""
        existing = prim.GetMetadata("apiSchemas")
        prepended = list(existing.prependedItems) if existing else []
        if schema_name not in prepended:
            prepended.append(schema_name)
            op = Sdf.TokenListOp()
            op.prependedItems = prepended
            prim.SetMetadata("apiSchemas", op)

    def _overlay_for_attr(self, attr_name):
        identifier = "newton_physx_overlay_" + attr_name.replace(":", "_")
        return ui_test.find(f"Property//Frame/**/Rectangle[*].identifier=='{identifier}'")

    async def _create_scene_with_newton(self, stage):
        path = omni.usd.get_stage_next_free_path(stage, "/physicsScene", True)
        prim = UsdPhysics.Scene.Define(stage, path).GetPrim()
        self._apply_api_schema(prim, "NewtonSceneAPI")
        PhysxSchema.PhysxSceneAPI.Apply(prim)
        return prim

    async def test_newton_physx_overlay_scene(self):
        """Screenshot test: Newton/PhysX priority overlay states on a PhysicsScene prim.

        Three states captured:
          1. Neither authored  — Newton row overlayed (PhysX default in effect).
          2. Newton authored   — overlay gone (Newton in control).
          3. PhysX deprecated attr authored — overlay back on Newton, PhysX row overlayed.
        """
        stage = await self.new_stage()
        prim = await self._create_scene_with_newton(stage)
        await self._select_and_focus_property_window([prim.GetPath().pathString])

        self.assertTrue(await self._test_newton_physx_visual(prim, "_newton_physx_scene_neither_authored"))

        prim.GetAttribute("newton:timeStepsPerSecond").Set(500.0)
        await self.wait(5)
        self.assertTrue(await self._test_newton_physx_visual(prim, "_newton_physx_scene_newton_authored"))

        PhysxSchema.PhysxSceneAPI.Get(stage, prim.GetPath()).GetTimeStepsPerSecondAttr().Set(60.0)
        await self.wait(5)
        self.assertTrue(await self._test_newton_physx_visual(prim, "_newton_physx_scene_physx_authored"))

        stage = await self.new_stage()

    async def test_newton_physx_overlay_dynamic(self):
        """Overlay visibility updates correctly when attrs are authored and deauthored.

        Priority states exercised:
          - Neither authored  → Newton overlay on,  PhysX overlay off  (priority 3: PhysX default)
          - Newton authored   → Newton overlay off, PhysX overlay on   (priority 2: Newton fallback)
          - Newton deauthored → Newton overlay on,  PhysX overlay off  (back to priority 3)
          - PhysX authored    → Newton overlay on,  PhysX overlay off  (priority 1: PhysX authored)
          - Both authored     → Newton overlay on,  PhysX overlay off  (priority 1 still wins)
        """
        stage = await self.new_stage()
        prim = await self._create_scene_with_newton(stage)
        await self._select_and_focus_property_window([prim.GetPath().pathString])
        await self.wait(5)

        newton_overlay = self._overlay_for_attr("newton:timeStepsPerSecond")
        physx_overlay = self._overlay_for_attr("physxScene:timeStepsPerSecond")
        self.assertIsNotNone(newton_overlay, "Newton overlay widget not found")
        self.assertIsNotNone(physx_overlay, "PhysX overlay widget not found")

        # Neither authored: PhysX default in effect.
        self.assertTrue(newton_overlay.widget.visible)
        self.assertFalse(physx_overlay.widget.visible)

        # Author Newton: Newton takes control.
        prim.GetAttribute("newton:timeStepsPerSecond").Set(500.0)
        await self.wait(5)
        self.assertFalse(newton_overlay.widget.visible)
        self.assertTrue(physx_overlay.widget.visible)

        # Deauthor Newton: back to PhysX default.
        prim.GetAttribute("newton:timeStepsPerSecond").Clear()
        await self.wait(5)
        self.assertTrue(newton_overlay.widget.visible)
        self.assertFalse(physx_overlay.widget.visible)

        # Author PhysX deprecated attr: PhysX wins via priority 1.
        PhysxSchema.PhysxSceneAPI.Get(stage, prim.GetPath()).GetTimeStepsPerSecondAttr().Set(60.0)
        await self.wait(5)
        self.assertTrue(newton_overlay.widget.visible)
        self.assertFalse(physx_overlay.widget.visible)

        # Author Newton too: PhysX still wins (priority 1 beats priority 2).
        prim.GetAttribute("newton:timeStepsPerSecond").Set(500.0)
        await self.wait(5)
        self.assertTrue(newton_overlay.widget.visible)
        self.assertFalse(physx_overlay.widget.visible)

        stage = await self.new_stage()

    async def test_newton_physx_reset_deauthors(self):
        """Reset button deauthors the Newton attribute rather than setting its default value.

        _remove_if_default=True on the model causes set_default() to call attr.Clear().
        """
        stage = await self.new_stage()
        prim = await self._create_scene_with_newton(stage)
        await self._select_and_focus_property_window([prim.GetPath().pathString])
        await self.wait(5)

        attr = prim.GetAttribute("newton:timeStepsPerSecond")
        attr.Set(500.0)
        await self.wait(5)
        self.assertTrue(attr.HasAuthoredValue())

        prim_key = (prim.GetPath().pathString,)
        registry_key = (id(stage), prim_key, "newton:timeStepsPerSecond")
        entry = DisableByCallbackBuilder._registry.get(registry_key)
        self.assertIsNotNone(entry, "Model not found in DisableByCallbackBuilder registry")
        model, _ = entry
        model.set_default()
        await self.wait(5)

        self.assertFalse(attr.HasAuthoredValue())

        stage = await self.new_stage()

    async def test_newton_physx_overlay_collision(self):
        """Screenshot test: Newton collision overlays (contactMargin, contactGap, maxHullVertices).

        Three states captured:
          1. Neither authored       — all Newton collision rows overlayed.
          2. Newton contactMargin authored — contactMargin overlay gone.
          3. PhysX restOffset authored    — contactMargin overlayed again.
        """
        stage = await self.new_stage()

        mesh = physicsUtils.create_mesh_cube(stage, "/meshActor", 100.0)
        prim = mesh.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim)
        self._apply_api_schema(prim, "NewtonCollisionAPI")
        self._apply_api_schema(prim, "NewtonMeshCollisionAPI")

        await self._select_and_focus_property_window([prim.GetPath().pathString])

        self.assertTrue(await self._test_newton_physx_visual(prim, "_newton_physx_collision_neither_authored", expand_advanced=True))

        prim.GetAttribute("newton:contactMargin").Set(0.01)
        await self.wait(5)
        self.assertTrue(await self._test_newton_physx_visual(prim, "_newton_physx_collision_margin_authored", expand_advanced=True))

        PhysxSchema.PhysxCollisionAPI.Apply(prim)
        PhysxSchema.PhysxCollisionAPI.Get(stage, prim.GetPath()).GetRestOffsetAttr().Set(0.02)
        await self.wait(5)
        self.assertTrue(await self._test_newton_physx_visual(prim, "_newton_physx_collision_physx_authored", expand_advanced=True))

        stage = await self.new_stage()
