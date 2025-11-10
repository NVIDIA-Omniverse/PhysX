# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb.settings
import carb.profiler
import omni.ext
import omni.usd
import omni.timeline
import asyncio
import omni.usdphysicsui
from omni.physxui import get_physxui_interface, get_physxui_private_interface
import omni.physx.bindings._physx as pb
from omni.physx.bindings._physx import ErrorEvent, SimulationEvent
from omni.physxui.bindings._physxUI import release_physx_ui_interface, release_physx_ui_interface_scripting
from omni.physxui.bindings._physxUI import release_physx_ui_private_interface, release_physx_ui_private_interface_scripting
from omni.physx import get_physx_interface
from .menu import PhysicsMenu, can_show_all, can_show_any
from .physxDebugView import PhysXDebugView, remove_schema
from .physxProgressView import PhysXProgressView
from .settings import PhysicsPreferences, PhysicsSettings
import omni.kit.window.preferences.scripts.preferences_window as preferences_window
import omni.kit.notification_manager as nm
import omni.kit.actions.core as ac
from omni.kit.viewport.utility import get_active_viewport_window
from .input import InputManager
from .utils import register_stage_update_node
from omni.physxuicommon import windowmenuitem
from omni.debugdraw import get_debug_draw_interface
from .physxViewportOverlays import PhysxUIViewportOverlays, PhysxUIMouseInteraction
from .collisionGroups import CollisionGroupsWindow
from .physicsViewportInfoBox import *
import omni.physxui.scripts.commands

from .physxJoints import PhysxUIJointManager


_extension_instance = None


def get_physicsui_instance():
    global _extension_instance
    return _extension_instance


def get_input_manager():
    return get_physicsui_instance()._input_manager

class PhysxUIExtension(omni.ext.IExt):
    def __init__(self):
        super().__init__()
        self._physxDebugView = None
        self._physxProgressView = None
        self._viewport_overlays = None
        self._input_manager = None
        self.__view_change_sub = None

    def on_startup(self):
        global _extension_instance
        _extension_instance = self

        # init interface cache
        self._physxui_iface = get_physxui_interface()
        self._physxui_private_iface = get_physxui_private_interface()

        events = get_physx_interface().get_error_event_stream()
        self._error_event_sub = events.create_subscription_to_pop(self._on_error_event)
        self._error_notification = None
        self._async_task_too_many_errors = None

        development_mode = carb.settings.get_settings().get("physics/developmentMode")
        self._physxDebugView = PhysXDebugView(development_mode)
        self._physxProgressView = PhysXProgressView()
        self._preferences = preferences_window.register_page(PhysicsPreferences())
        self._input_manager = InputManager()

        self._collision_groups = CollisionGroupsWindow()
        if self._collision_groups:
            self._collision_groups.on_startup()

        self._settings_menu = windowmenuitem.WindowMenuItem("Settings", lambda: PhysicsSettings(), True)
        if not development_mode:
            self._settings_menu.hide_window()

        omni.ui.Workspace.set_show_window_fn(PhysicsSettings.get_window_name(), self._settings_menu.set_window_visibility)

        def on_viewport_change(viewport_api):
            cam_pos = viewport_api.transform.ExtractTranslation()
            get_physxui_interface().set_camera_pos(cam_pos)

        viewport_window = get_active_viewport_window()
        if viewport_window:
            self.__view_change_sub = viewport_window.viewport_api.subscribe_to_view_change(on_viewport_change)

        self._menu = PhysicsMenu()
        self._menu.on_startup()

        self._debugDraw = get_debug_draw_interface()

        self._physx_viewport_menu = None

        omni.usdphysicsui.get_usdphysicsui_instance().unregister_physics_viewport_menu()
        try:
            # We have an optional dependency on VP2 "omni.kit.viewport.menubar.display" that will
            # not get loaded during kit VP1 L1 tests, and in turn will throw an exception here
            from .physicsViewportMenu import PhysxViewportMenu
            self._physx_viewport_menu = PhysxViewportMenu()
            self._physx_viewport_menu.register_with_viewport()
        except:
            pass

        # On 104 branch, viewport2 destroyed, so we need poll check
        self._viewport_overlays = PhysxUIViewportOverlays(viewport2_destroyed_poll_checking=True)
        self._viewport_overlays.on_startup()

        self._show_sim_output_wnd_action = None
        self._register_actions("omni.physx.ui")

        self._joint_manager = PhysxUIJointManager()


    def _register_actions(self, ext_id):
        self._show_sim_output_wnd_action = ac.get_action_registry().register_action(
            ext_id,
            "show_simulation_output_settings_window",
            lambda: carb.settings.get_settings().set_bool(pb.SETTING_DISPLAY_SIMULATION_OUTPUT, True),
            display_name="Show Simulation Output Settings Window",
            description="Shows Simulation Output Settings Window",
            tag="Physics Tools",
        )

    def _unregister_actions(self):
        if self._show_sim_output_wnd_action:
            ac.get_action_registry().deregister_action(self._show_sim_output_wnd_action)
            self._show_sim_output_wnd_action = None

    # Allows overriding mouse interaction state externally.
    def mouse_interaction_override_toggle(self, state: PhysxUIMouseInteraction):
        if self._viewport_overlays:
            self._viewport_overlays._mouse_interaction_state = state

    def create_text_info_box(self, text = [], text_color=INFO_BOX_TEXT_COLOR_DEFAULT, text_font_size = INFO_BOX_TEXT_FONT_SIZE_DEFAULT, text_alignment = ui.Alignment.CENTER, background_color = INFO_BOX_BACKGROUND_COLOR_DEFAULT, screen_coords = [0, 0], height = -1, width = -1):
        """
        Creates a text box on in main viewport at the provided coordinates as an offset from the camera center.

        -   If background_color is not None, it will draw the text within a rectangle.
        -   If the height or width input is -1, it will attempt to match the dimension of the box based on the size of the text.
        -   The 'text' input can be an array, in which case each entry will appear as a separate line.
        -   The 'text_color' and 'text_font_size' can also be arrays, in which case they will be used as per-line values.
            Note: if using array input for either of these, their length must match the text array.

        You must keep a reference to the PhysicsInfoBoxTextManipulator returned and call destroy() prior to delete/release.
        """
        if self._viewport_overlays:
            return self._viewport_overlays.create_text_info_box(text, text_color, text_font_size, text_alignment, background_color, screen_coords, height, width)
        else:
            return None

    async def post_too_many_errors_notification(self):
        await asyncio.sleep(5)
        self._error_notification = nm.post_notification(
            "PhysX has reported too many errors, simulation has been stopped.",
            duration=5,
            status=nm.NotificationStatus.WARNING)
        await asyncio.sleep(2)
        carb.log_error("PhysX has reported too many errors, simulation has been stopped.")
        self._async_task_too_many_errors = None

    def _on_error_event(self, event):
        prefixValue = ""
        if event.type == int(ErrorEvent.USD_LOAD_ERROR):
            prefixValue = "Physics USD Load: "
        elif event.type == int(ErrorEvent.PHYSX_ERROR):
            prefixValue = "PhysX error: "
        elif event.type == int(ErrorEvent.PHYSX_TOO_MANY_ERRORS):
            omni.timeline.get_timeline_interface().stop()
            # we post the notification later because at this point we've probably flooded
            # notification manager, so we must wait for its queue to empty if we want to
            # make sure we're properly showing this notification as last one on screen
            if self._async_task_too_many_errors:
                self._async_task_too_many_errors.cancel()
            self._async_task_too_many_errors = asyncio.ensure_future(self.post_too_many_errors_notification())
            return
        elif event.type == int(ErrorEvent.PHYSX_CUDA_ERROR):
            self._error_notification = nm.post_notification(
                "PhysX GPU Cuda context error reported, simulation will be stopped.",
                duration=5,
                status=nm.NotificationStatus.WARNING)
            omni.timeline.get_timeline_interface().stop()
            return

        self._error_notification = nm.post_notification(
            prefixValue + event.payload['errorString'],
            duration=5,
            status=nm.NotificationStatus.WARNING)

    def on_shutdown(self):
        self._unregister_actions()

        # Deregister the function that shows the window from omni.ui
        omni.ui.Workspace.set_show_window_fn(PhysicsSettings.get_window_name(), None)

        if self._collision_groups:
            self._collision_groups.on_shutdown()
            self._collision_groups = None

        self._viewport_overlays.on_shutdown()
        self._viewport_overlays = None
        self._error_notification = None
        if self._async_task_too_many_errors:
            self._async_task_too_many_errors.cancel()
        self._async_task_too_many_errors = None
        self._physxProgressView.on_shutdown()
        self._physxProgressView = None

        self._physxDebugView.on_shutdown()
        self._physxDebugView = None

        if self._physx_viewport_menu is not None:
            self._physx_viewport_menu.unregister_from_viewport()
        self._physx_viewport_menu = None

        preferences_window.unregister_page(self._preferences)
        self._preferences.on_shutdown()
        self._preferences = None
        self._settings_menu.on_shutdown()
        self._settings_menu = None

        self._menu.on_shutdown()
        self._menu = None

        self._error_event_sub = None

        self._stage_update_node = None

        self._joint_manager.destroy()
        self._joint_manager = None

        release_physx_ui_interface(self._physxui_iface)
        release_physx_ui_interface_scripting(self._physxui_iface)
        self._physxui_iface = None
        release_physx_ui_private_interface(self._physxui_private_iface)
        release_physx_ui_private_interface_scripting(self._physxui_private_iface)
        self._physxui_private_iface = None

        self.__view_change_sub = None

        global _extension_instance
        _extension_instance = None
