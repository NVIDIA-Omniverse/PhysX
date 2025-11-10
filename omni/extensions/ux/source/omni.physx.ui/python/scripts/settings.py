# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.usd
from omni import ui
from omni.physx import get_physx_interface, get_physx_cooking_interface
import carb
import carb.settings
import carb.profiler
from carb.eventdispatcher import get_eventdispatcher
from omni.kit.window.preferences.scripts.preferences_window import PreferenceBuilder
from omni.kit.widget.settings import SettingsWidgetBuilder
from omni.kit.widget.settings.settings_widget import SettingType, create_setting_widget, create_setting_widget_combo
import omni.physx.bindings._physx as physx_bindings
from functools import partial
from omni.physx.scripts.pythonUtils import ScopeGuard

BUTTON_HEIGHT = 30


def get_no_stage_err_msg(path):
    return f"You are setting a per-stage physics settings {path} without an active stage. This setting will be reset when a new stage is loaded!"


def build_section(name, build_func):
    with ui.CollapsableFrame(name, height=0):
        with ui.HStack():
            ui.Spacer(width=20)
            with ui.VStack():
                build_func()


class PhysicsPreferences(PreferenceBuilder):
    def __init__(self):
        super().__init__("Physics")
        self._widgets = {}
        self._line_height = 20
        self._developer_mode = carb.settings.get_settings().get("physics/developmentMode")

    def on_shutdown(self):
        self._widgets = {}

    def _add_setting_combo_and_label(self, name, path, items, **kwargs):
        with ui.HStack(height=27):
            self.label(name)
            create_setting_widget_combo(path, items, **kwargs)

    def _add_setting(self, setting_type, name, path, range_from=0, range_to=0, speed=1, **kwargs):
        widget = self.create_setting_widget(name, path, setting_type, range_from=range_from, range_to=range_to, speed=speed, **kwargs)
        self._widgets[path] = widget

        if setting_type in [SettingType.INT, SettingType.FLOAT]:
            def value_changed(model):
                val = model.as_int if setting_type == SettingType.INT else model.as_float
                if val < range_from:
                    model.set_value(range_from)
                elif val > range_to:
                    model.set_value(range_to)

            widget.model.add_value_changed_fn(value_changed)

        return widget

    def _build_simulator_ui(self):
        self._line_height = 23
        self._add_setting(SettingType.INT, "Num Simulation Threads", physx_bindings.SETTING_NUM_THREADS, 0, 256)
        self._add_setting(SettingType.INT, "Stop simulation after number of (PhysX) errors", physx_bindings.SETTING_MAX_NUMBER_OF_PHYSX_ERRORS, 1, 4096)
        self._add_setting(SettingType.BOOL, "Use PhysX CPU Dispatcher", physx_bindings.SETTING_PHYSX_DISPATCHER)
        self._add_setting(SettingType.BOOL, "Expose PhysX SDK Profiler Data", physx_bindings.SETTING_EXPOSE_PROFILER_DATA)
        self._add_setting(SettingType.BOOL, "Expose USD Prim Names toPhysX SDK Names", physx_bindings.SETTING_EXPOSE_PRIM_PATH_NAMES)
        self._add_setting(SettingType.BOOL, "Simulate empty scene", physx_bindings.SETTING_SIMULATE_EMPTY_SCENE)

    def _add_release_local_mesh_cache(self):
        button = ui.Button("Release Local Mesh Cache", height=BUTTON_HEIGHT, width=300)

        def on_click():
            get_physx_cooking_interface().release_local_mesh_cache()
        button.set_clicked_fn(on_click)

    def _build_local_mesh_cache_ui(self):
        self._line_height = 23
        self._add_setting(SettingType.BOOL, "Enable Local Mesh Cache (Takes Effect on Sim Start)", physx_bindings.SETTING_USE_LOCAL_MESH_CACHE)
        self._add_setting(SettingType.INT, "Local Mesh Cache Size MB (Takes Effect on Sim Start)", physx_bindings.SETTING_LOCAL_MESH_CACHE_SIZE_MB, 16, 4096)
        self._add_setting(SettingType.BOOL, "Enable Ujitso Collision Cooking", physx_bindings.SETTING_UJITSO_COLLISION_COOKING)
        self._add_setting(SettingType.INT, "Ujitso Cooking Max Process Count", physx_bindings.SETTING_UJITSO_COOKING_MAX_PROCESS_COUNT, 1, 128)
        self._add_release_local_mesh_cache()

    def _build_general_settings(self):
        self._add_setting(SettingType.BOOL, "Reset Simulation on Stop", physx_bindings.SETTING_RESET_ON_STOP)
        self._add_setting(SettingType.BOOL, "Use Active CUDA Context (Requires Restart)", physx_bindings.SETTING_USE_ACTIVE_CUDA_CONTEXT)
        self._add_setting_combo_and_label("Use Physics Scene Multi-GPU Mode", physx_bindings.SETTING_PHYSICS_SCENE_MULTIGPU_MODE, {"Disabled": 0, "Use All Devices": 1, "Use All Devices Except First": 2})
        self._add_setting(SettingType.INT, "Add Menu Selection Prim Limit", physx_bindings.SETTING_ADDMENU_SELECTION_LIMIT, 0, 4294967295)
        self._add_setting(SettingType.INT, "Add Menu Subtree Prim Limit", physx_bindings.SETTING_ADDMENU_SUBTREE_LIMIT, 0, 4294967295)
        self._add_setting(SettingType.BOOL, "Enable Attachment Authoring (Requires Stage Reload)", physx_bindings.SETTING_ENABLE_ATTACHMENT_AUTHORING)
        self._add_setting(SettingType.BOOL, "Enable Particle Authoring (Requires Stage Reload)", physx_bindings.SETTING_ENABLE_PARTICLE_AUTHORING)
        self._add_setting(SettingType.BOOL, "Enable Deformable Schema Beta (Requires Restart)", physx_bindings.SETTING_ENABLE_DEFORMABLE_BETA)
        self._add_setting_combo_and_label(
            "Default Physics Simulator",
            physx_bindings.SETTING_DEFAULT_SIMULATOR,
            {
                "None": "None",
                "PhysX": "PhysX",
            },
        )

        def clicked(*_):
            get_physx_interface().reset_settings_in_preferences()

        ui.Button("Reset Physics Preferences", height=BUTTON_HEIGHT, width=300, clicked_fn=clicked)

    def build(self):
        with ui.VStack(height=0):
            build_section("General", self._build_general_settings)
            ui.Spacer(height=5)
            build_section("Simulator", self._build_simulator_ui)
            ui.Spacer(height=5)
            build_section("Local Mesh Cache", self._build_local_mesh_cache_ui)


class PhysicsSettings(ui.Window):
    def __init__(self):
        super().__init__(PhysicsSettings.get_window_name(), dockPreference=ui.DockPreference.RIGHT_TOP)
        self.deferred_dock_in("Stage", ui.DockPolicy.DO_NOTHING)

        usd_context = omni.usd.get_context()
        self._stage_event_sub = get_eventdispatcher().observe_event(
            observer_name="omni.physx.ui:PhysicsSettings",
            event_name=usd_context.stage_event_name(omni.usd.StageEventType.OPENED),
            on_event=lambda _: self._refresh_from_usd()
        )

        self._write_guard = ScopeGuard()
        self._buttons = {}
        self._build()

    @staticmethod
    def get_window_name() -> str:
        return "Physics Stage Settings"

    def on_shutdown(self):
        self._buttons = {}
        self._stage_event_sub = None

    def _build(self):
        from omni.kit.widget.settings import get_style
        self.frame.set_style(get_style())

        with self.frame:
            with ui.ScrollingFrame(
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            ):
                with ui.VStack(spacing=5):
                    build_section("Update", self._build_update_ui)
                    ui.Spacer(height=5)
                    build_section("Simulator", self._build_simulator_ui)
                    ui.Spacer(height=5)
                    build_section("Collision", self._build_collision_ui)
                    ui.Spacer(height=5)
                    build_section("Mouse Interaction", self._build_mouse_interaction_ui)
                    ui.Spacer(height=5)
                    build_section("Debug Visualization", self._build_debug_vis_ui)

    def _is_default(self, path, curr_val):
        def_val = carb.settings.get_settings().get("/defaults" + path)
        return def_val == curr_val

    def _refresh_from_usd(self):
        stage = omni.usd.get_context().get_stage()
        physics_settings = stage.GetRootLayer().customLayerData.get("physicsSettings")

        with self._write_guard:
            get_physx_interface().reset_settings_in_stage()

        for btn in self._buttons.values():
            btn.visible = False

        if physics_settings:
            for path, value in physics_settings.items():
                carb.settings.get_settings().set(path, value)
                if path in self._buttons:
                    self._buttons[path].visible = True

    def _write_to_usd(self, path, _):
        if self._write_guard.is_guarded():
            return

        stage = omni.usd.get_context().get_stage()
        if not stage:
            carb.log_warn(get_no_stage_err_msg(path))
            return

        val = carb.settings.get_settings().get(path)

        # changing it to default, delete it from usd instead (if present)
        if self._is_default(path, val):
            self._delete_from_usd(path)
            return

        custom_layer_data = stage.GetRootLayer().customLayerData

        if "physicsSettings" not in custom_layer_data:
            custom_layer_data["physicsSettings"] = {}

        custom_layer_data["physicsSettings"][path] = val

        btn = self._buttons.get(path)
        if btn:
            btn.visible = True

        try:
            # Before writing let's check if an asyncio event loop exists on current thread
            # Joint state asset validator changes this setting and gets invoked on non-main asyncio thread
            import asyncio
            asyncio.get_event_loop()
            stage.GetRootLayer().customLayerData = custom_layer_data
        except Exception as e:
            carb.log_info("Skip writing customLayerData")

    def _delete_from_usd(self, path):
        stage = omni.usd.get_context().get_stage()
        custom_layer_data = stage.GetRootLayer().customLayerData

        if "physicsSettings" in custom_layer_data:
            custom_layer_data["physicsSettings"].pop(path, None)

        btn = self._buttons.get(path)
        if btn:
            btn.visible = False
        try:
            # Before writing let's check if an asyncio event loop exists on current thread
            # Joint state asset validator changes this setting and gets invoked on non-main asyncio thread
            import asyncio
            asyncio.get_event_loop()
            stage.GetRootLayer().customLayerData = custom_layer_data
        except Exception as e:
            carb.log_info("Skip writing customLayerData")

    def _build_update_ui(self):
        self._add_setting(SettingType.BOOL, "Update to USD", physx_bindings.SETTING_UPDATE_TO_USD)
        self._add_setting(SettingType.BOOL, "Update Velocities to USD", physx_bindings.SETTING_UPDATE_VELOCITIES_TO_USD)
        self._add_setting(SettingType.BOOL, "Output Velocities in Local Space", physx_bindings.SETTING_OUTPUT_VELOCITIES_LOCAL_SPACE)
        self._add_setting(SettingType.BOOL, "Update Particles to USD", physx_bindings.SETTING_UPDATE_PARTICLES_TO_USD)
        self._add_setting(SettingType.BOOL, "Update Residuals to USD", physx_bindings.SETTING_UPDATE_RESIDUALS_TO_USD)

    def _build_collision_ui(self):
        self._add_setting(SettingType.BOOL, "Approximate Cones With Convex Meshes", physx_bindings.SETTING_COLLISION_APPROXIMATE_CONES)
        self._add_setting(SettingType.BOOL, "Approximate Cylinders With Convex Meshes", physx_bindings.SETTING_COLLISION_APPROXIMATE_CYLINDERS)

    def _build_simulator_ui(self):
        self._add_setting(SettingType.INT, "Min Simulation Frame Rate", physx_bindings.SETTING_MIN_FRAME_RATE, 1, 1000)
        self._add_setting(SettingType.FLOAT, "Joint Body Transform Check Tolerance", physx_bindings.SETTING_JOINT_BODY_TRANSFORM_CHECK_TOLERANCE, 0.0, 10000000.0)
        self._add_setting(SettingType.BOOL, "Enable Extended Joint Angles", physx_bindings.SETTING_ENABLE_EXTENDED_JOINT_ANGLES, tooltip=(
            "\nSupport for joint angle computation in range (-360, 360)"
            "\nfor revolute joints and D6 joints that have limits on a"
            "\nrotational degree of freedom. This allows, for example, to"
            "\ndefine revolute joint rotational limits in range (-360, 360)."
            "\nAffects non-articulation joints only."))

    def _build_mouse_interaction_ui(self):
        self._add_setting(SettingType.BOOL, "Mouse Interaction Enabled", physx_bindings.SETTING_MOUSE_INTERACTION_ENABLED)
        self._add_setting(SettingType.BOOL, "Mouse Grab", physx_bindings.SETTING_MOUSE_GRAB)
        self._add_setting(SettingType.BOOL, "Mouse Grab Ignore Invisible", physx_bindings.SETTING_MOUSE_GRAB_IGNORE_INVISBLE)
        self._add_setting(SettingType.BOOL, "Mouse Grab With Force", physx_bindings.SETTING_MOUSE_GRAB_WITH_FORCE)
        self._add_setting(SettingType.FLOAT, "Mouse Grab Force Coeff", physx_bindings.SETTING_MOUSE_PICKING_FORCE, 0.0, 10.0)
        self._add_setting(SettingType.FLOAT, "Mouse Push Acceleration", physx_bindings.SETTING_MOUSE_PUSH, 0.0, 1000.0)

    def _build_debug_vis_ui(self):
        self._add_setting(SettingType.FLOAT, "Simplify Debug Visualization at Distance", physx_bindings.SETTING_DEBUG_VIS_SIMPLIFY_AT_DISTANCE, 1.0, 10000.0)
        self._add_setting(SettingType.BOOL, "Query USDRT For Stage Traversal", physx_bindings.SETTING_DEBUG_VIS_QUERY_USDRT_FOR_TRAVERSAL)

    def _reset_setting(self, path):
        self._delete_from_usd(path)
        self._refresh_from_usd()

    def _add_setting(self, setting_type, name, path, range_from=0, range_to=0, speed=1, tooltip=""):
        with ui.HStack(skip_draw_when_clipped=True):
            SettingsWidgetBuilder._create_label(name, path, tooltip, additional_label_kwargs={"width": 220})
            widget, model = create_setting_widget(path, setting_type, range_from, range_to, speed)
            model.add_value_changed_fn(partial(self._write_to_usd, path))
            btn = SettingsWidgetBuilder._build_reset_button(path)
            btn.set_mouse_pressed_fn(lambda *_, path=path: self._reset_setting(path))
            btn.set_tooltip("Click to clear from USD")
            self._buttons[path] = btn
        ui.Spacer(height=3)


def load_physics_settings_from_stage(stage):
    physics_settings = stage.GetRootLayer().customLayerData.get("physicsSettings")
    get_physx_interface().reset_settings_in_stage()
    if physics_settings:
        for path, value in physics_settings.items():
            carb.settings.get_settings().set(path, value)
