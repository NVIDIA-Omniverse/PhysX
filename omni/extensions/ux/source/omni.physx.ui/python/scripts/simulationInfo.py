# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from enum import IntEnum, auto
from functools import partial
import carb.input
import omni.appwindow
from omni import ui
from omni.ui import color as cl
import omni.physx.bindings._physx as ph


class TargetOutput(IntEnum):
    USD = auto()
    FABRIC_CPU = auto()
    FABRIC_GPU = auto()


class SimulationInfoWindow(ui.Window):
    FABRIC_EXTENSION_NAME = "omni.physx.fabric"
    SETTING_FABRIC_ENABLED = "physics/fabricEnabled"
    SETTING_FABRIC_UPDATE_TRANSFORMATIONS = "/physics/fabricUpdateTransformations"
    SETTING_FABRIC_UPDATE_POINTS = "physics/fabricUpdatePoints"
    SETTING_FABRIC_USE_GPU_INTEROP = "physics/fabricUseGPUInterop"
    SETTING_OMNI_HYDRA_USE_SCENE_GRAPH_INSTANCING = "persistent/omnihydra/useSceneGraphInstancing"
    WARNING_COLOR = cl.yellow
    ENABLED_COLOR = cl.lightgreen
    WINDOW_STYLE = {
        "Window": {
            # "background_color": cl.shade(cl("#25282ABB")), <- enable if you want transparent window background
            "border_color": 0x0,
            "border_width": 0,
            "border_radius": 5
        }
    }
    OUTPUT_BUTTON_STYLE = {"Button:checked": {"background_color": ENABLED_COLOR}, "Button.Label:checked": {"color": cl.black}}
    NONE = ("None", None)
    USD = ("USD", None)
    FABRIC_WARNING = " (Properties not updated!)"
    FABRIC = ("Fabric", FABRIC_WARNING)
    FABRIC_CPU = ("Fabric CPU", FABRIC_WARNING)
    FABRIC_GPU = ("Fabric GPU", FABRIC_WARNING)

    def __init__(self):

        super().__init__(
            "Simulation Settings",
            visible=False,
            flags=ui.WINDOW_FLAGS_NO_FOCUS_ON_APPEARING,
        )

        self.frame.set_style(self.WINDOW_STYLE)
        self.frame.set_build_fn(self.build)

        self._settings = carb.settings.get_settings()

        # get extension manager and register for fabric extension load/uload events
        app = omni.kit.app.get_app_interface()
        self._extension_manager = app.get_extension_manager()
        self._fabric_ext_active = self._extension_manager.is_extension_enabled(self.FABRIC_EXTENSION_NAME)
        self._extensions_subs = (
            self._extension_manager.subscribe_to_extension_enable(
                partial(self._fabric_extension_changed, True),
                partial(self._fabric_extension_changed, False),
                ext_name=self.FABRIC_EXTENSION_NAME,
                hook_name="omni.physx.ui fabric cache extension listener",
            )
        )

        self.set_visibility_changed_fn(self._window_visiblity_changed_fn)

        # subscribe to all settings we are interested in
        def sub_setting(setting):
            return omni.kit.app.SettingChangeSubscription(setting, self._setting_changed)

        self._settings_subs = (
            sub_setting(ph.SETTING_UPDATE_TO_USD),
            sub_setting(ph.SETTING_RESET_ON_STOP),
            sub_setting(ph.SETTING_OVERRIDE_GPU),
            sub_setting(ph.SETTING_PVD_ENABLED),
            sub_setting(self.SETTING_OMNI_HYDRA_USE_SCENE_GRAPH_INSTANCING),
            sub_setting(self.SETTING_FABRIC_ENABLED),
            sub_setting(self.SETTING_FABRIC_UPDATE_TRANSFORMATIONS),
            sub_setting(self.SETTING_FABRIC_UPDATE_POINTS),
            sub_setting(self.SETTING_FABRIC_USE_GPU_INTEROP),
        )

    def destroy(self):
        self._extensions_subs = None
        self._settings_subs = None
        self._settings = None

    def set_pos(self, x, y):
        self.position_x, self.position_y = x, y

    def show(self):
        if not self.docked:
            self.width = -1
            self.height = -1
        self.visible = True

    def hide(self):
        self.visible = False

    def _window_visiblity_changed_fn(self, visible):
        # handle the case when user closes the window by a cross
        if not visible:
            window_enabled = self._settings.get_as_bool(ph.SETTING_DISPLAY_SIMULATION_OUTPUT)
            if window_enabled:
                self._settings.set_bool(ph.SETTING_DISPLAY_SIMULATION_OUTPUT, False)

    def toggle_fabric_ext(self, enable):
        if (enable and not self._fabric_ext_active) or (not enable and self._fabric_ext_active):
            omni.kit.commands.execute("ToggleExtension", ext_id=self.FABRIC_EXTENSION_NAME, enable=enable)

    def _fabric_extension_changed(self, loaded: bool, ext_id: str):
        self._fabric_ext_active = loaded
        self.frame.rebuild() if self.frame is not None else self.build()

    def _setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self.frame.rebuild() if self.frame is not None else self.build()

    def enable_scene_graph_instancing(self):
        self._settings.set_bool(self.SETTING_OMNI_HYDRA_USE_SCENE_GRAPH_INSTANCING, True)

    def set_simulation_output(self, output: TargetOutput):
        self.toggle_fabric_ext(output != TargetOutput.USD)  # need to enable Fabric extension?
        self._settings.set_bool(ph.SETTING_UPDATE_TO_USD, output == TargetOutput.USD)
        self._settings.set_bool(self.SETTING_FABRIC_ENABLED, output == TargetOutput.FABRIC_CPU or output == TargetOutput.FABRIC_GPU)
        self._settings.set_bool(self.SETTING_FABRIC_USE_GPU_INTEROP, output == TargetOutput.FABRIC_GPU)
        self._settings.set_bool(self.SETTING_FABRIC_UPDATE_TRANSFORMATIONS, output == TargetOutput.FABRIC_CPU or output == TargetOutput.FABRIC_GPU)
        self._settings.set_bool(self.SETTING_FABRIC_UPDATE_POINTS, output == TargetOutput.FABRIC_CPU or output == TargetOutput.FABRIC_GPU)

    def is_fabric_setting_on(self, fabric_setting) -> bool:
        if not self._fabric_ext_active:
            return False
        if not self._settings.get_as_bool(self.SETTING_FABRIC_ENABLED):
            return False
        if not self._settings.get_as_bool(fabric_setting):
            return False
        return True

    def build(self):
        def build_section(name, build_func):
            with ui.CollapsableFrame(name, height=0):
                with ui.HStack():
                    ui.Spacer(width=25)
                    with ui.VStack():
                        build_func()
                    ui.Spacer(width=5)

        def build_output_buttons():
            def create_button(text, init_value, target: TargetOutput):
                button = ui.Button(
                    text,
                    checked=init_value,
                    tooltip=f"Set simulation target to {text}",
                    mouse_pressed_fn=lambda x, y, b, m, t=target: self.set_simulation_output(t),
                    style=self.OUTPUT_BUTTON_STYLE
                )
                return button

            with ui.HStack():
                create_button(
                    "USD",
                    self._settings.get_as_bool(ph.SETTING_UPDATE_TO_USD) and not self.is_fabric_setting_on(self.SETTING_FABRIC_ENABLED),
                    TargetOutput.USD
                )
                ui.Spacer(width=5)
                create_button(
                    "Fabric CPU",
                    self.is_fabric_setting_on(self.SETTING_FABRIC_ENABLED) and not self.is_fabric_setting_on(self.SETTING_FABRIC_USE_GPU_INTEROP),
                    TargetOutput.FABRIC_CPU
                )
                ui.Spacer(width=5)
                create_button(
                    "Fabric GPU",
                    self.is_fabric_setting_on(self.SETTING_FABRIC_USE_GPU_INTEROP),
                    TargetOutput.FABRIC_GPU
                )

        def build_line(label, value, add_value=None):
            with ui.HStack():
                ui.Label(label, width=80)
                ui.Spacer(width=5)
                ui.Label(value, width=0, style={"color": self.ENABLED_COLOR})
                if add_value is not None:
                    ui.Label(add_value, style={"color": self.WARNING_COLOR})

        def build_toggle_line(label, eval_fnc, change_fnc):
            value = eval_fnc()
            with ui.HStack():
                ui.Label(label, width=80)
                ui.Spacer(width=5)
                ui.Button(
                    "Enabled" if value else "Disabled",
                    checked=value,
                    mouse_pressed_fn=lambda x, y, b, m: change_fnc(not eval_fnc()),
                    style=self.OUTPUT_BUTTON_STYLE
                )
                ui.Spacer(width=5)

        def build_hydra_section():
            ui.Label(
                "For rigid bodies with instanced geometries, enable scene graph instancing\n"
                "in omni.hydra.ui extension. Then reload the stage.",
                width=0,
                style={"color": self.WARNING_COLOR}
            )
            ui.Button(
                "Enable scene graph instancing now",
                style={"background_color": self.WARNING_COLOR, "color": cl.black, "margin_width": 0},
                mouse_pressed_fn=lambda x, y, b, m: self.enable_scene_graph_instancing()
            )

        def build_simulation_output_section():
            def fabric_mode_cpu_gpu():
                if self._settings.get_as_bool(self.SETTING_FABRIC_USE_GPU_INTEROP):
                    return self.FABRIC_GPU
                return self.FABRIC_CPU

            mode = self.NONE
            if self._settings.get_as_bool(ph.SETTING_UPDATE_TO_USD):
                mode = self.USD

            fabric_cpu_gpu = fabric_mode_cpu_gpu()

            rigid_body = mode
            if self.is_fabric_setting_on(self.SETTING_FABRIC_UPDATE_TRANSFORMATIONS):
                rigid_body = self.FABRIC

            deformable_particles = mode
            if self.is_fabric_setting_on(self.SETTING_FABRIC_UPDATE_POINTS):
                deformable_particles = fabric_cpu_gpu

            build_output_buttons()

            build_line("Rigid Body", *rigid_body)
            build_line("Deformable Bodies", *deformable_particles)
            build_line("Particles", *deformable_particles)

        def build_more_simulation_settings_section():
            with ui.VStack():
                gpu_override = self._settings.get_as_int(ph.SETTING_OVERRIDE_GPU)
                if gpu_override in (0, 1):
                    build_line("Simulation", "Forced to GPU" if gpu_override == 1 else "Forced to CPU")
                if self._settings.get_as_bool(ph.SETTING_PVD_ENABLED):
                    build_line("PVD is", "Enabled")
            build_toggle_line(
                "Reset Simulation on Stop",
                lambda: self._settings.get_as_bool(ph.SETTING_RESET_ON_STOP),
                lambda enable: self._settings.set_bool(ph.SETTING_RESET_ON_STOP, enable))

        instancing_used = False
        if not self._settings.get_as_bool(self.SETTING_OMNI_HYDRA_USE_SCENE_GRAPH_INSTANCING):
            stage = omni.usd.get_context().get_stage()
            if stage:
                masters = stage.GetPrototypes()
                if len(masters) > 0:
                    instancing_used = True

        if not self.docked:
            self.width = -1
            self.height = -1

        with self.frame:
            with ui.VStack(height=0, spacing=1):
                if instancing_used:
                    build_section("Instancing is Used", build_hydra_section)
                    ui.Spacer(width=0, height=3)
                build_section("Simulation Output", build_simulation_output_section)
                ui.Spacer(width=0, height=3)
                build_section("More Simulation Settings", build_more_simulation_settings_section)
