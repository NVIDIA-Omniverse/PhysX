# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.commands
import omni.usd
from pxr import Usd, Sdf
from carb.eventdispatcher import get_eventdispatcher
import omni.kit.window.property
import collections.abc
from .physicsViewportOverlayShared import *
from .physicsViewportOverlayInfobox import PhysicsUIFloatingInfobox
from enum import IntEnum, auto
from collections.abc import Iterable


class PhysicsUIViewportPrim(PhysicsUIManipulator):
    """
    Generic template for showing information of a prim.

    Specific variations should be derived from this class and be registered on the PhysicsUIPrimInfoOverlay.
    """

    infobox_title = "PRIM PROPERTIES"


    class SubsectionTargetBasic(PhysicsUIFloatingInfobox.Subsection):

        class TextEntries(IntEnum):
            PATH = 0
            TYPE = auto()

        def __init__(self, info_box):
            text_entries = [None] * len(self.TextEntries)
            text_entries[self.TextEntries.PATH] = "Path:"
            text_entries[self.TextEntries.TYPE] = "Type:"
            super().__init__(info_box, PhysicsUIViewportPrim.infobox_title, text_entries, 2)

        def refresh(self):
            if len(self.manipulator._targets) == 1:
                self.set_title(self.manipulator._targets[0].get_infobox_title())
            else:
                self.set_title(PhysicsUIViewportPrim.infobox_title)

            new_basic_values_text = [None] * len(self.TextEntries)
            if self.manipulator._targets is None or len(self.manipulator._targets) == 0 or self.manipulator._targets[0]._prim == None:
                new_basic_values_text[self.TextEntries.PATH] = "-"
                new_basic_values_text[self.TextEntries.TYPE] = "-"
            elif len(self.manipulator._targets) > 1:
                new_basic_values_text[self.TextEntries.PATH] = "Multiple selected"
                new_basic_values_text[self.TextEntries.TYPE] = "-"
            else:
                prim = self.manipulator._targets[0]._prim
                new_basic_values_text[self.TextEntries.PATH] = text_truncate(prim.GetPrimPath().pathString, 35)
                new_basic_values_text[self.TextEntries.TYPE] = prim.GetTypeName()

            self.set_text(new_basic_values_text, 1)

            super().refresh()

    @classmethod
    def get_infobox_subsections(cls) -> list[PhysicsUIFloatingInfobox.Subsection]:
        return [cls.SubsectionTargetBasic]

    @property
    def prim(self) -> Usd.Prim:
        return self._prim

    def __init__(self, viewport_overlay: PhysicsUIViewportOverlay, prim: Usd.Prim, **kwargs):
        super().__init__(viewport_overlay, **kwargs)
        self._prim: Usd.Prim = prim

    def destroy(self):
        if self._prim is not None:
            self._prim = None
            self.viewport_overlay.remove_prim_property_manipulator(self)
            super().destroy()

    @property
    def prim_xform(self):
        return None

    def get_infobox_view_offset(self) -> tuple[float, float]:
        return (0.0, 0.0)

    def get_infobox_screen_offset(self) -> tuple[float, float]:
        return (10.0, -10.0)

    def get_infobox_world_position(self) -> tuple[float, float, float]:
        return None

    def get_infobox_title(self) -> str:
        return self.infobox_title

    def on_usd_prim_paths_info_changed(self, paths : set[Sdf.Path]):
        if self._prim is not None and self._prim.GetPath() in paths:
            if not self.get_is_valid():
                self.destroy()
            else:
                self.refresh_data()

    def on_usd_prim_paths_resynced(self, paths : set[Sdf.Path]):
        if self._prim is not None and self._prim.GetPath() in paths:
            if not self.get_is_valid():
                self.destroy()
            else:
                self.refresh_data()

    def refresh_data(self):
        if self in self.viewport_overlay.get_info_box_targets():
            self.viewport_overlay.refresh_info_box()

    def get_is_valid(self) -> bool:
        if not super().get_is_valid():
            return False
        return self.get_is_prim_valid_target(self._prim)

    def _make_ui_shapes(self):
        pass

    @classmethod
    def get_is_prim_valid_target(cls, prim: Usd.Prim):
        return (prim is not None and prim.IsValid())

    @classmethod
    def overrides_type(cls, other_type) -> bool:
        # Default is to have derived types override ancestor types.
        return (other_type is None or issubclass(cls, other_type))

    def on_build(self):
        super().on_build()

        if not self.get_is_valid():
            self.destroy()
            return

        self._make_ui_shapes()
        self.refresh_data()


class PhysicsPrimFloatingInfoboxManipulator(PhysicsUIFloatingInfobox):

    def __init__(self, viewport_overlay):
        super().__init__(viewport_overlay)
        self._targets = []

    def destroy(self):
        self._targets : list[PhysicsUIViewportPrim] = []
        super().destroy()

    # Can be a single target or an array.
    def set_targets(self, targets):
        if targets is None:
            targets = []
        elif not isinstance(targets, collections.abc.Sized):
            targets = [targets]

        if len(self._targets) == len(targets):
            target_update = False
            for target in targets:
                if target not in self._targets:
                    target_update = True
                    break
        else:
            target_update = True

        if target_update:
            subsections = []
            for target in targets:
                for subsection in target.get_infobox_subsections():
                    if subsection not in subsections:
                        subsections.append(subsection)
            self._world_position_current = None
            self._targets = targets.copy()
            self.set_subsection_types(subsections)

    def get_targets(self):
        return self._targets

    def get_is_active(self) -> bool:
        if super().get_is_active() and self.get_targets() is not None and len(self.get_targets()) > 0:
            return True
        return False

    def get_view_offset(self) -> tuple[float, float]:
        targets = self.get_targets()
        if len(targets) == 1:
            return targets[0].get_infobox_view_offset()
        else:
            return (0.0, 0.0)

    def get_screen_offset(self) -> tuple[float, float]:
        targets = self.get_targets()
        if len(targets) == 1:
            return targets[0].get_infobox_screen_offset()
        else:
            return (10.0, -10.0)

    def get_world_position(self) -> tuple[float, float, float]:
        targets = self.get_targets()
        if len(targets) > 0:
            return targets[0].get_infobox_world_position()
        else:
            return None

    def refresh(self):
        new_targets = []

        active_hover = self.viewport_overlay.manager.gesture_manager.active_hover
        active_gesture = self.viewport_overlay.manager.gesture_manager.active_gesture

        if active_gesture is not None and isinstance(active_gesture.manipulator, PhysicsUIViewportPrim):
            new_targets.append(active_gesture.manipulator)
        elif active_hover is not None and isinstance(active_hover.manipulator, PhysicsUIViewportPrim):
            new_targets.append(active_hover.manipulator)
        elif self.stage is not None:
            for selected_path in self.viewport_overlay._selection.get_selected_prim_paths():
                prim = self.stage.GetPrimAtPath(selected_path)
                if prim is not None and prim in self.viewport_overlay._prim_property_manipulators:
                    manipulator = self.viewport_overlay._prim_property_manipulators[prim]
                    if manipulator is not None:
                        new_targets.append(manipulator)

        self.set_targets(new_targets)
        super().refresh()

class PhysicsUIPrimInfoOverlay(PhysicsUIViewportOverlay):
    """
    Overlay for drawing UI manipulators for individuals prims (as PhysicsUIViewportPrim) in the scene and display their information.

    Other locations may register PhysicsUIViewportPrim subtypes that the overlay will use.
    """
    _prim_property_manipulator_type_registry : list[PhysicsUIViewportPrim] = []

    class PrimInfoTypeRegistryHandle():
        def __init__(self, prim_property_manipulator_type) -> None:
            self.manipulator_type = prim_property_manipulator_type

        def unregister(self):
            PhysicsUIPrimInfoOverlay._prim_property_manipulator_type_registry.remove(self.manipulator_type)
            self.manipulator_type = None

        def __del__(self):
            if self.manipulator_type is not None:
                self.unregister()

    @classmethod
    def register_prim_property_type(cls, prim_property_manipulator_type):
        cls._prim_property_manipulator_type_registry.append(prim_property_manipulator_type)
        return __class__.PrimInfoTypeRegistryHandle(prim_property_manipulator_type)

    def __init__(self, main_viewport_overlay):
        super().__init__(main_viewport_overlay)
        self._info_box = None
        self._stage_event_sub = None
        self._prim_property_manipulators : dict[Usd.Prim, PhysicsUIViewportPrim]= {}
        self._selection = self.usd_context.get_selection()

        self._stage_event_sub = [
            get_eventdispatcher().observe_event(
                observer_name="omni.usdphysics.ui:PhysicsUIPrimInfoOverlay",
                event_name=self.usd_context.stage_event_name(event),
                on_event=func,
                order=1
            )
            for event, func in (
                (omni.usd.StageEventType.OPENED, lambda _: self._on_stage_opened()),
                (omni.usd.StageEventType.CLOSING, lambda _: self._on_stage_closed()),
                (omni.usd.StageEventType.SELECTION_CHANGED, lambda _: self.refresh_prim_manipulators()),
                (omni.usd.StageEventType.SIMULATION_STOP_PLAY, lambda _: self._on_stage_simulation_stop_play()),
            )
        ]

        with self.manager.scene_view_world.scene:
            self._info_box = PhysicsPrimFloatingInfoboxManipulator(self)

        self._current_registry = PhysicsUIPrimInfoOverlay._prim_property_manipulator_type_registry.copy()

        self._on_stage_opened()

    def destroy(self):
        self._stage_event_sub = None
        self._on_stage_closed()
        if self._info_box is not None:
            self._info_box.destroy()
            self._info_box = None
        super().destroy()

    def get_info_box_targets(self):
        if self._info_box is not None:
            return self._info_box.get_targets()
        return []

    def refresh_info_box(self):
        if self._info_box is not None:
            self._info_box.refresh()

    @classmethod
    def _get_prim_desired_manipulator_type(cls, prim: Usd.Prim):
        desired_type = None
        for prim_type in cls._prim_property_manipulator_type_registry:
            if prim_type.overrides_type(desired_type) and prim_type.get_is_prim_valid_target(prim):
                desired_type = prim_type

        return desired_type

    def get_prim_property_manipulator(self, prim : Usd.Prim):
        return self._prim_property_manipulators.get(prim)

    def remove_prim_property_manipulator(self, prim_manipulator : PhysicsUIViewportPrim):
        for prim, manipulator in self._prim_property_manipulators.items():
            if manipulator == prim_manipulator:
                self._prim_property_manipulators[prim] = None   # .pop(prim)
                if manipulator in self.get_info_box_targets():
                    self.refresh_info_box()
                break

    def refresh_prim_manipulators(self, prim_paths: Iterable[Sdf.Path] = None):
        if not self.stage:
            self._clear_prim_property_manipulators()
            return

        selected_prim_paths = self._selection.get_selected_prim_paths()

        # If no paths are provided, reevaluate all existing manipulators and use selection for creating new.
        if prim_paths is None:
            for manipulator in self._prim_property_manipulators.values():
                if (manipulator is not None and manipulator.prim.GetPath() not in selected_prim_paths):
                    manipulator.destroy()
            prim_paths = selected_prim_paths

        for prim_path in prim_paths:
            prim = self.stage.GetPrimAtPath(prim_path)
            manipulator = self.get_prim_property_manipulator(prim)
            desired_type = self._get_prim_desired_manipulator_type(prim)
            if manipulator is not None:
                if desired_type is None or type(manipulator) is not desired_type:
                    # Destroy immediately.
                    manipulator.destroy()
                else:
                    # Our current type matches the desired type.
                    continue

            if desired_type is not None:
                with self.manager.scene_view_world.scene:
                    self._prim_property_manipulators[prim] = desired_type(self, prim)

        # Clean up any leftover entries in our dictionary that doesn't point to a manipulator.
        expired_manipulator_prims = []
        for prim, manipulator in self._prim_property_manipulators.items():
            if manipulator is None:
                expired_manipulator_prims.append(prim)

        for expired_prim in expired_manipulator_prims:
            self._prim_property_manipulators.pop(expired_prim)

        self.manager.refresh_usd_change_listener()
        self.refresh_info_box()

    def _on_stage_simulation_stop_play(self):
        for manipulator in self._prim_property_manipulators.values():
            if manipulator is not None:
                manipulator.refresh_data()

    def _clear_prim_property_manipulators(self):
        for manipulator in self._prim_property_manipulators.values():
            if manipulator is not None:
                manipulator.destroy()
        self._prim_property_manipulators.clear()
        self.manager.refresh_usd_change_listener()
        self.refresh_info_box()

    def _on_stage_opened(self):
        if self.stage is None:
            return

        self.refresh_prim_manipulators()

    def _on_stage_closed(self):
        self._clear_prim_property_manipulators()

    def on_kit_update(self):
        if not components_equal(self._current_registry, PhysicsUIPrimInfoOverlay._prim_property_manipulator_type_registry):
            self.refresh_prim_manipulators()
            self._current_registry = PhysicsUIPrimInfoOverlay._prim_property_manipulator_type_registry.copy()

        # When simulation is running, we don't track USD changes but simply refresh all data each frame.
        if self.simulation_active:
            for manipulator in self._prim_property_manipulators.values():
                if manipulator is not None:
                    manipulator.refresh_data()

    def subscribes_to_usd_prim_paths_info_changed(self) -> bool:
        return len(self._prim_property_manipulators) > 0

    def on_usd_prim_paths_info_changed(self, changed_paths: set[Sdf.Path]):
        for manipulator in self._prim_property_manipulators.values():
            if manipulator is not None:
                manipulator.on_usd_prim_paths_info_changed(changed_paths)

    def subscribes_to_usd_prim_paths_resynced(self) -> bool:
        return len(self._prim_property_manipulators) > 0

    def on_usd_prim_paths_resynced(self, resynced_paths: set[Sdf.Path]):
        for manipulator in self._prim_property_manipulators.values():
            if manipulator is not None:
                manipulator.on_usd_prim_paths_resynced(resynced_paths)

        self.refresh_prim_manipulators(resynced_paths)
