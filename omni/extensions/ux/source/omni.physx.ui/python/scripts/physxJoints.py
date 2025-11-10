# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb.settings
from omni.usdphysicsui.scripts.physicsViewportOverlayPrimInfo import PhysicsUIPrimInfoOverlay, PhysicsUIViewportPrim
from omni.usdphysicsui.scripts.physicsViewportOverlayJoints import JointEditManipulator
from omni.usdphysicsui.scripts.physicsViewportOverlayInfobox import PhysicsUIFloatingInfobox
from omni.usdphysicsui.scripts.physicsViewportOverlayShared import *
from pxr import Usd, PhysxSchema
import math
from enum import auto, IntEnum
from omni.ui import scene as sc
from omni.ui import color as cl
from pathlib import Path


class PhysxUIGearJointEditManipulator(JointEditManipulator):

    infobox_title = "GEAR JOINT PROPERTIES"

    @classmethod
    def get_is_prim_valid_target(cls, prim: Usd.Prim) -> bool:
        if not super().get_is_prim_valid_target(prim):
            return False
               
        return prim.IsA(PhysxSchema.PhysxPhysicsGearJoint)

    def __init__(self, viewport_overlay, prim, **kwargs):
        self._gear_joint = PhysxSchema.PhysxPhysicsGearJoint(prim)

        super().__init__(viewport_overlay, prim, **kwargs)

        for limit_type in self.Limit:
            self._limits[limit_type] = [-math.inf, math.inf]

        self._hinges_rel_attr = [self._gear_joint.GetHinge0Rel(), self._gear_joint.GetHinge1Rel()]
        self._ratio_attr = self._gear_joint.GetGearRatioAttr()
    
        self._hinges = [None, None]
        self._ratio : float = 1.0

    def _refresh_ui_joint_bounds(self) -> bool:
        updated = super()._refresh_ui_joint_bounds()
        if updated:
            # Never show the aligned bbox
            self._attachments[1]._ui_transform_bbox_aligned.visible = False

        return updated

    def refresh_data(self):
        for n in range(2):
            if not self.simulation_active or self._hinges[n] is None:
                hinge = self._hinges_rel_attr[n].GetTargets()
                self._hinges[n] = None if len(hinge) != 1 else self.stage.GetPrimAtPath(hinge[0])

        self._ratio = self._ratio_attr.Get()
        super().refresh_data()

    def hinges_swap(self):
        properties = []
        values = []

        for hinge in range(2):
            other = (hinge + 1) % 2
            properties.append(self._hinges_rel_attr[hinge])
            values.append(self._hinges_rel_attr[other].GetTargets())

        if self._ratio > 0.0:
            properties.append(self._ratio_attr)
            values.append(1.0 / self._ratio)

        self._write_values_to_usd(properties, values)

    class SwapHingesClickGesture(PhysicsUIClickGesture):
        def __init__(self, manipulator, toggle_group):
            super().__init__(manipulator, toggle_group)

        def on_ended(self):
            super().on_ended()
            target = self.manipulator.get_targets()[0]
            target.hinges_swap()


    class InfoboxSubsectionGearHinges(PhysicsUIFloatingInfobox.Subsection):
        class TextEntries(IntEnum):
            HINGE_0 = 0
            HINGE_1 = auto()
            GEAR_RATIO = auto()

        def __init__(self, info_box):
            text_entries = [None] * len(self.TextEntries)
            text_entries[self.TextEntries.HINGE_0] = "Hinge 0:"
            text_entries[self.TextEntries.HINGE_1] = "Hinge 1:"
            text_entries[self.TextEntries.GEAR_RATIO] = "Gear Ratio:"
            self._toggle_swap = PhysicsUIToggleGroup()
            self._swap_image_icon = None
            super().__init__(info_box, "Hinges", text_entries, 2)

        def get_is_active(self) -> bool:
            if not super().get_is_active():
                return False

            manipulators: list[PhysicsUIViewportPrim] = self.manipulator.get_targets()

            if (len(manipulators) == 1 and isinstance(manipulators[0], PhysxUIGearJointEditManipulator) and self.manipulator.get_targets()[0].prim is not None):
                return True
            return False

        def refresh(self):
            if self.get_is_active():
                manipulator: PhysxUIGearJointEditManipulator = self.manipulator.get_targets()[0]
                if manipulator is None or manipulator._attachments[0] is None or manipulator._attachments[1] is None:
                    return

                self._swap_image_icon.visible = not manipulator.simulation_active

                text_entries = [None] * len(self.TextEntries)

                if manipulator._hinges[0] is not None:
                    text_entries[self.TextEntries.HINGE_0] = text_truncate(manipulator._hinges[0].GetPrimPath().pathString, 30)
                else: 
                    text_entries[self.TextEntries.HINGE_0] = "-"

                if manipulator._hinges[1] is not None:
                    text_entries[self.TextEntries.HINGE_1] = text_truncate(manipulator._hinges[1].GetPrimPath().pathString, 30)
                else: 
                    text_entries[self.TextEntries.HINGE_1] = "-"

                text_entries[self.TextEntries.GEAR_RATIO] = float_to_string(manipulator._ratio)

                self.set_text(text_entries, 1)

            super().refresh()

        def _make_ui_shapes(self):
            super()._make_ui_shapes()

            self._text_entries_transform[1][0].transform = self._text_entries_transform[1][0].transform * sc.Matrix44().get_translation_matrix(-12, 0, 0)
            self._text_entries_transform[1][1].transform = self._text_entries_transform[1][1].transform * sc.Matrix44().get_translation_matrix(-12, 0, 0)
            with self._text_entries_transform[1][0]:
                with sc.Transform(transform=sc.Matrix44().get_translation_matrix(8, -round(self._style.font_size * (1.0 + self._style.font_line_spacing) * 0.5) - 1, -0.00000001)):
                    filename = str(Path(__file__).parents[3].joinpath("icons", "physicsJoint", "swap.svg"))
                    self._swap_image_icon = sc.Image(filename, color=cl("#FFFFFFFF"), width=16, height=16, gesture=[PhysxUIGearJointEditManipulator.SwapHingesClickGesture(self.manipulator, self._toggle_swap), PhysicsUIHoverGesture(self.manipulator, self._toggle_swap)])
                    toggles = self._toggle_swap.get_manipulator_toggles(self.manipulator)
                    toggles.clear()
                    toggles.append(ColorToggle(self._swap_image_icon, COLOR_TEXT_HIGHLIGHT, self._style.font_color))

    @classmethod
    def get_infobox_subsections(cls) -> list[PhysicsUIFloatingInfobox.Subsection]:
        return JointEditManipulator.get_infobox_subsections() + [cls.InfoboxSubsectionGearHinges]

class PhysxUIRackAndPinionJointEditManipulator(JointEditManipulator):

    infobox_title = "RACK AND PINION JOINT PROPERTIES"

    @classmethod
    def get_is_prim_valid_target(cls, prim: Usd.Prim) -> bool:
        if not super().get_is_prim_valid_target(prim):
            return False
               
        return prim.IsA(PhysxSchema.PhysxPhysicsRackAndPinionJoint)

    def __init__(self, viewport_overlay, prim, **kwargs):
        self._rack_and_pinion_joint = PhysxSchema.PhysxPhysicsRackAndPinionJoint(prim)

        super().__init__(viewport_overlay, prim, **kwargs)

        for limit_type in self.Limit:
            self._limits[limit_type] = [-math.inf, math.inf]

        self._hinge_rel_attr = self._rack_and_pinion_joint.GetHingeRel()
        self._prismatic_rel_attr = self._rack_and_pinion_joint.GetPrismaticRel()
        self._ratio_attr = self._rack_and_pinion_joint.GetRatioAttr()
    
        self._hinge = None
        self._prismatic = None
        self._ratio : float = 1.0

    def _refresh_ui_joint_bounds(self) -> bool:
        updated = super()._refresh_ui_joint_bounds()
        if updated:
            # Never show the aligned bbox
            self._attachments[1]._ui_transform_bbox_aligned.visible = False

        return updated

    def refresh_data(self):
        if not self.simulation_active or self._hinge is None:
            hinge = self._hinge_rel_attr.GetTargets()
            self._hinge = None if len(hinge) != 1 else self.stage.GetPrimAtPath(hinge[0])

        if not self.simulation_active or self._prismatic is None:
            prismatic = self._prismatic_rel_attr.GetTargets()
            self._prismatic = None if len(prismatic) != 1 else self.stage.GetPrimAtPath(prismatic[0])

        self._ratio = self._ratio_attr.Get()
        super().refresh_data()


    class InfoboxSubsectionRackAndPinionJoints(PhysicsUIFloatingInfobox.Subsection):
        class TextEntries(IntEnum):
            HINGE = 0
            PRISMATIC = auto()
            GEAR_RATIO = auto()

        def __init__(self, info_box):
            text_entries = [None] * len(self.TextEntries)
            text_entries[self.TextEntries.HINGE] = "Hinge:"
            text_entries[self.TextEntries.PRISMATIC] = "Prismatic Joint:"
            text_entries[self.TextEntries.GEAR_RATIO] = "Ratio:"
            super().__init__(info_box, "Hinges", text_entries, 2)

        def get_is_active(self) -> bool:
            if not super().get_is_active():
                return False

            manipulators: list[PhysicsUIViewportPrim] = self.manipulator.get_targets()

            if (len(manipulators) == 1 and isinstance(manipulators[0], PhysxUIRackAndPinionJointEditManipulator) and self.manipulator.get_targets()[0].prim is not None):
                return True
            return False

        def refresh(self):
            if self.get_is_active():
                manipulator: PhysxUIRackAndPinionJointEditManipulator = self.manipulator.get_targets()[0]
                if manipulator is None or manipulator._attachments[0] is None or manipulator._attachments[1] is None:
                    return

                text_entries = [None] * len(self.TextEntries)

                if manipulator._hinge is not None:
                    text_entries[self.TextEntries.HINGE] = text_truncate(manipulator._hinge.GetPrimPath().pathString, 30)
                else: 
                    text_entries[self.TextEntries.HINGE] = "-"

                if manipulator._prismatic is not None:
                    text_entries[self.TextEntries.PRISMATIC] = text_truncate(manipulator._prismatic.GetPrimPath().pathString, 30)
                else: 
                    text_entries[self.TextEntries.PRISMATIC] = "-"

                text_entries[self.TextEntries.GEAR_RATIO] = float_to_string(manipulator._ratio)

                self.set_text(text_entries, 1)

            super().refresh()

    @classmethod
    def get_infobox_subsections(cls) -> list[PhysicsUIFloatingInfobox.Subsection]:
        return JointEditManipulator.get_infobox_subsections() + [cls.InfoboxSubsectionRackAndPinionJoints]

class PhysxUIJointManager():
    def __init__(self):
        self._registered_prim_property_types = []
        self._refresh_prim_property_types_registered()

        self._settings_subscriptions = []
        self._settings_subscriptions.append(carb.settings.get_settings().subscribe_to_node_change_events(
            '/persistent/physics/visualizationDisplayJoints', self._on_settings_changed
        ))

    def _refresh_prim_property_types_registered(self):
        self._registered_prim_property_types.clear()

        if carb.settings.get_settings().get('/persistent/physics/visualizationDisplayJoints'):
            self._registered_prim_property_types.append(PhysicsUIPrimInfoOverlay.register_prim_property_type(PhysxUIGearJointEditManipulator))
            self._registered_prim_property_types.append(PhysicsUIPrimInfoOverlay.register_prim_property_type(PhysxUIRackAndPinionJointEditManipulator))

    def _on_settings_changed(self, item, event_type):
        self._refresh_prim_property_types_registered()

    def destroy(self):
        for registry in self._registered_prim_property_types:
            registry.unregister()

        self._registered_prim_property_types.clear()

    def __del__(self):
        self.destroy()
