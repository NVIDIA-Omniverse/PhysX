# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ext
import carb.settings
import omni.ui as ui
import omni.physx.bindings._physx as physx_bindings
from omni.kit.viewport.menubar.core.model.category_model import CategoryCustomItem, CategoryStateItem
from omni.kit.viewport.menubar.core import ViewportMenuDelegate, SelectableMenuItem
from .simulationInfo import SimulationInfoWindow
from omni.usdphysicsui import PhysicsViewportMenuHelper
from .simulationDataVisualizer import SimulationDataVisualizerWindowManager


# The 'Physics' viewport 2.0 menu to visualize debug colliders, tendons and such
class PhysxViewportMenu:
    def __init__(self):
        self._simulation_info_wnd = None
        self._simulation_data_visualizer_manager = None
        self._display_instance = None

    def register_with_viewport(self):
        self._helper = PhysicsViewportMenuHelper()
        self._simulation_info_wnd = None
        self._simulation_data_visualizer_manager = None

        def _build_menu():

            settings = carb.settings.acquire_settings_interface()

            # some helper functions

            def create_menu_simple_bool_item(text, setting) -> CategoryStateItem:
                return self._helper.create_menu_simple_bool_item(text, setting)

            def simulation_collision_store_setting_deprecated(item, setting):
                self._helper.simulation_collision_store_setting_deprecated(item, setting)

            def deformable_mesh_type_store_setting(item, setting):
                self._helper.deformable_mesh_type_store_setting(item, setting)

            def none_selected_all_store_setting(item, setting):
                self._helper.none_selected_all_store_setting(item, setting)

            def safe_subscribe_to_setting_change(sub, items, setting):
                self._helper.safe_subscribe_to_setting_change(sub, items, setting)

            def create_radioboxes_for_setting(setting, radioboxes, map_text_to_setting_value_fn):
                self._helper.create_radioboxes_for_setting(setting, radioboxes, map_text_to_setting_value_fn)

            def create_checkbox_for_setting(text, setting):
                self._helper.create_checkbox_for_setting(text, setting)

            def _build_coll_simpl_distance_slider_menu_item():
                from omni.kit.viewport.menubar.core import (
                    SliderMenuDelegate,
                    SettingModelWithDefaultValue
                )
                ui.MenuItem(
                    "Simplify at Distance",
                    hide_on_click=False,
                    delegate=SliderMenuDelegate(
                        model=SettingModelWithDefaultValue(
                            physx_bindings.SETTING_DEBUG_VIS_SIMPLIFY_AT_DISTANCE,
                            settings.get_as_float(physx_bindings.SETTING_DEBUG_VIS_SIMPLIFY_AT_DISTANCE),
                            draggable=True
                        ),
                        min=1,
                        max=10000,
                        step=1.0,
                        tooltip="Simplify colliders to AABBs at specified distance from active camera",
                        has_reset=True,
                    ),
                )

            # this item does not have C++ implementation as others
            def create_menu_sim_output_bool_item(text, setting) -> CategoryStateItem:
                def show_overlay(model, item):
                    show = model.get_value_as_bool()
                    settings.set(setting, show)
                    if self._simulation_info_wnd is None:
                        self._simulation_info_wnd = SimulationInfoWindow()
                        self._simulation_info_wnd.set_pos(55, 100)
                    if show:
                        self._simulation_info_wnd.show()
                    else:
                        self._simulation_info_wnd.hide()

                # also set default value with whatever we have in the settings
                newItem = SelectableMenuItem(text, ui.SimpleBoolModel(settings.get_as_bool(setting)))
                newItem.model.add_value_changed_fn(lambda model: show_overlay(model, newItem))
                # set overlay visibility based on current setting
                show_overlay(newItem.model, newItem)
                return newItem

            # this item does not have C++ implementation as others
            def create_menu_sim_data_visualizer_bool_item(text, setting) -> CategoryStateItem:
                def show_overlay(model, item):
                    show = model.get_value_as_bool()
                    settings.set(setting, show)
                    if show:
                        if self._simulation_data_visualizer_manager is None:
                            self._simulation_data_visualizer_manager = SimulationDataVisualizerWindowManager()
                    else:
                        if self._simulation_data_visualizer_manager is not None:
                            self._simulation_data_visualizer_manager.destroy()
                            self._simulation_data_visualizer_manager = None


                # also set default value with whatever we have in the settings
                newItem = SelectableMenuItem(text, ui.SimpleBoolModel(settings.get_as_bool(setting)))
                newItem.model.add_value_changed_fn(lambda model: show_overlay(model, newItem))
                # set overlay visibility based on current setting
                show_overlay(newItem.model, newItem)
                return newItem

            # Main menu construction code
            with ui.Menu("Physics", delegate=ViewportMenuDelegate()):
                jointsItem = create_menu_simple_bool_item("Joints", physx_bindings.SETTING_DISPLAY_JOINTS)
                # first time, by passing None as subscription, we just set the setting changes notification as active
                safe_subscribe_to_setting_change(None, [jointsItem], physx_bindings.SETTING_DISPLAY_JOINTS)

                simulationOutputItem = create_menu_sim_output_bool_item("Simulation Settings", physx_bindings.SETTING_DISPLAY_SIMULATION_OUTPUT)
                # first time, by passing None as subscription, we just set the setting changes notification as active
                safe_subscribe_to_setting_change(None, [simulationOutputItem], physx_bindings.SETTING_DISPLAY_SIMULATION_OUTPUT)

                simulationDataVisualizerItem = create_menu_sim_data_visualizer_bool_item("Simulation Data Visualizer", physx_bindings.SETTING_DISPLAY_SIMULATION_DATA_VISUALIZER)
                # first time, by passing None as subscription, we just set the setting changes notification as active
                safe_subscribe_to_setting_change(None, [simulationDataVisualizerItem], physx_bindings.SETTING_DISPLAY_SIMULATION_DATA_VISUALIZER)

                deformable_beta_enabled = carb.settings.get_settings().get(physx_bindings.SETTING_ENABLE_DEFORMABLE_BETA)

                with ui.Menu("Colliders", delegate=ViewportMenuDelegate()):
                    create_radioboxes_for_setting(physx_bindings.SETTING_DISPLAY_COLLIDERS, ["None", "Selected", "All"], none_selected_all_store_setting)
                    _build_coll_simpl_distance_slider_menu_item()
                    ui.Separator()
                    create_checkbox_for_setting("Normals", physx_bindings.SETTING_DISPLAY_COLLIDER_NORMALS)
                with ui.Menu("Mass Properties", delegate=ViewportMenuDelegate()):
                    create_radioboxes_for_setting(physx_bindings.SETTING_DISPLAY_MASS_PROPERTIES, ["None", "Selected", "All"], none_selected_all_store_setting)
                with ui.Menu("Tendons", delegate=ViewportMenuDelegate()):
                    create_radioboxes_for_setting(physx_bindings.SETTING_DISPLAY_TENDONS, ["None", "Selected", "All"], none_selected_all_store_setting)
                # DEPRECATED
                if not deformable_beta_enabled:
                    with ui.Menu("Deformable Body (deprecated)", delegate=ViewportMenuDelegate()):
                        create_radioboxes_for_setting(physx_bindings.SETTING_DISPLAY_DEFORMABLE_BODIES, ["None", "Selected", "All"], none_selected_all_store_setting)
                        ui.Separator()
                        create_radioboxes_for_setting(physx_bindings.SETTING_DISPLAY_DEFORMABLE_BODY_TYPE, ["Simulation", "Collision"], simulation_collision_store_setting_deprecated)
                    with ui.Menu("Attachments (deprecated)", delegate=ViewportMenuDelegate()):
                        create_radioboxes_for_setting(physx_bindings.SETTING_DISPLAY_ATTACHMENTS, ["None", "Selected", "All"], none_selected_all_store_setting)
                        ui.Separator()
                        create_checkbox_for_setting("Hide Actor 0", physx_bindings.SETTING_DISPLAY_ATTACHMENTS_HIDE_ACTOR_0)
                        create_checkbox_for_setting("Hide Actor 1", physx_bindings.SETTING_DISPLAY_ATTACHMENTS_HIDE_ACTOR_1)
                #~DEPRECATED
                if deformable_beta_enabled:
                    with ui.Menu("Deformables (beta)", delegate=ViewportMenuDelegate()):
                        create_radioboxes_for_setting(physx_bindings.SETTING_DISPLAY_DEFORMABLES, ["None", "Selected", "All"], none_selected_all_store_setting)
                        ui.Separator()
                        create_radioboxes_for_setting(
                            physx_bindings.SETTING_DISPLAY_DEFORMABLE_MESH_TYPE,
                            [
                                "Simulation: Default Pose",
                                "Simulation: Bind Pose",
                                "Simulation: Rest Shape",
                                "Collision: Default Pose",
                                "Collision: Bind Pose",
                            ],
                            deformable_mesh_type_store_setting
                        )
                        ui.Separator()
                        create_checkbox_for_setting("Attachments", physx_bindings.SETTING_DISPLAY_DEFORMABLE_ATTACHMENTS)
                with ui.Menu("Particles", delegate=ViewportMenuDelegate()):
                    create_radioboxes_for_setting(physx_bindings.SETTING_DISPLAY_PARTICLES, ["None", "Selected", "All"], none_selected_all_store_setting)
                    ui.Separator()
                    create_checkbox_for_setting("Particles", physx_bindings.SETTING_DISPLAY_PARTICLES_SHOW_PARTICLE_SET_PARTICLES)
                    create_checkbox_for_setting("Fluid Surface", physx_bindings.SETTING_DISPLAY_PARTICLES_SHOW_FLUID_SURFACE)
                    create_checkbox_for_setting("Cloth Particles (deprecated)", physx_bindings.SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_PARTICLES)
                    create_checkbox_for_setting("Cloth Mesh", physx_bindings.SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH)
                    ui.Separator()
                    create_checkbox_for_setting("Diffuse Particles", physx_bindings.SETTING_DISPLAY_PARTICLES_SHOW_DIFFUSE)

        self._display_instance = omni.kit.viewport.menubar.display.get_instance()
        self._physics_menu = CategoryCustomItem("Physics", _build_menu)
        self._display_instance.register_custom_category_item("Show By Type", self._physics_menu)

    def unregister_from_viewport(self):
        if self._simulation_info_wnd is not None:
            self._simulation_info_wnd.undock()
            self._simulation_info_wnd.destroy()
            self._simulation_info_wnd = None

        if self._simulation_data_visualizer_manager is not None:
            self._simulation_data_visualizer_manager.destroy()
            self._simulation_data_visualizer_manager=None
        if self._display_instance is not None:
            self._display_instance.deregister_custom_category_item("Show By Type", self._physics_menu)
            self._physics_menu = None
            self._helper = None
