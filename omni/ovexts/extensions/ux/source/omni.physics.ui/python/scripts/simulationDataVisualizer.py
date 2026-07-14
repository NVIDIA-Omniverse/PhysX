# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import math
from collections import deque
from collections.abc import Sequence
from pathlib import Path
from typing import Any, Callable

from carb import log_error
from carb.eventdispatcher import Event, get_eventdispatcher
from carb.settings import get_settings
from pxr import Sdf, Tf, Usd

import omni.kit.clipboard
from omni import ui
from omni.kit.notification_manager import post_notification
from omni.physics.core import (
    DebugDataItemType,
    Simulation,
    SimulationEvent,
    SimulationId,
    SimulationRegistryEventType,
    get_physics_interaction_interface,
    get_physics_interface,
    get_physics_simulation_interface,
    get_physics_stage_update_interface,
)
from omni.ui import color as cl
from omni.usd import StageEventType, UsdContext
from omni.usd import get_context as get_usd_context

from omni.usdphysicsui import PhysicsViewportMenuHelper

from .utils import float_to_string, register_stage_update_node


SETTINGS_UI_PHYSICS_SIMULATION_DATA_VISUALIZER_ENABLED = "/persistent/physics/ui/simulation_data_visualizer_enabled"
SETTINGS_UI_WINDOW_OPACITY = "/persistent/app/viewport/ui/background/opacity"

icons_folder = Path(__file__).parents[4].joinpath("icons", "simDataVis")
icon_plot = str(icons_folder.joinpath("plot.svg"))


class PhysicsDataTrace:
    """
    Stores data input and generates sample outputs with a defined sample period.

    The sample outputs are used to draw graphs that may need a different format than the input (e.g. quaternions vs axis-angle),
    and a different frequency since too many datapoints will cause visual jitter.
    """

    def __init__(self, data_num_components, samples_num_components=None, samples_num=500, sample_period=0.05) -> None:
        self._data_num_components = data_num_components
        self._samples_num = samples_num
        self._samples_num_components = data_num_components if samples_num_components is None else samples_num_components
        self._sample_period = sample_period
        self.reset()

    def reset(self):
        self._input_accumulator = [0.0] * self._data_num_components
        self._input_accumulator_time_elapsed = 0.0

        # The raw input datapoints stored in a list of tuples.
        self._trace_datapoints: list = []

        # Generated sample output as deques for plotting graphs.
        self._trace_samples: list[deque[float]] = []
        for _ in range(self._samples_num_components):
            self._trace_samples.append(deque[float]([0.0], maxlen=self._samples_num))

    # Overridable function for data types that may need further output processing (e.g. quaternions to axis-angle).
    def process_data(self, data: Sequence[float]) -> Sequence[float]:
        return data

    def set_data(self, data: tuple[float, ...]):
        sample = self.process_data(data)
        if len(self._trace_datapoints) == 0:
            # This means that we haven't set any yet. Reset to this data.
            for n in range(self._samples_num_components):
                self._trace_samples[n] = deque[float]([sample[n]], maxlen=self._samples_num)

            self._input_accumulator = [0.0] * self._data_num_components
            self._trace_datapoints = [data]
            self._input_accumulator_time_elapsed = 0.0
        else:
            # Changes while simulation is paused. Adjust the most recent datapoint.
            for n in range(self._samples_num_components):
                self._trace_samples[n][-1] = sample[n]
            self._trace_datapoints[-1] = data

    # Appends a datapoint to the accumulator. Returns true if a new output was added to the sampler.
    def append(self, delta_time, data: tuple[float, ...]) -> bool:
        if len(self._trace_datapoints) == 0:
            self.set_data(data)
        else:
            self._trace_datapoints.append(data)

        self._input_accumulator_time_elapsed += delta_time
        input_accumulator = self._input_accumulator
        sample_period = self._sample_period
        if self._input_accumulator_time_elapsed >= sample_period:
            # Time we've exceeded the interval.
            excess_time = self._input_accumulator_time_elapsed - sample_period

            for component in range(self._data_num_components):
                input_accumulator[component] += (delta_time - excess_time) * data[component]
                input_accumulator[component] /= sample_period

            sample = self.process_data(input_accumulator)

            for n in range(self._samples_num_components):
                self._trace_samples[n].append(sample[n])

            for component in range(self._data_num_components):
                input_accumulator[component] = excess_time * data[component]
            self._input_accumulator_time_elapsed = excess_time

            return True
        else:
            for component in range(self._data_num_components):
                input_accumulator[component] += delta_time * data[component]
            return False

    def get_samples(self):
        return self._trace_samples

    def get_sample(self, index):
        return self._trace_samples[index]

    def get_data(self) -> list:
        return self._trace_datapoints

    @property
    def num_components(self):
        return self._samples_num_components


class PhysicsDataTraceAxisAngleFromQuaternion(PhysicsDataTrace):
    def __init__(self) -> None:
        super().__init__(4)

    def process_data(self, data):
        length = math.sqrt(data[0] ** 2 + data[1] ** 2 + data[2] ** 2)
        if length == 0.0:
            return (1.0, 0.0, 0.0, 0.0)

        angle = math.degrees(2.0 * math.acos(data[3]))
        if angle > 180.0:
            angle = angle - 360.0
        return (data[0] / length, data[1] / length, data[2] / length, angle)


class PhysicsDataTraceSingle(PhysicsDataTrace):
    def __init__(self) -> None:
        super().__init__(1)

    def set_data(self, data: float):
        return super().set_data((data,))

    def append(self, delta_time, data) -> bool:
        return super().append(delta_time, (data,))


class PhysicsDataTraceVectorWithMagnitude(PhysicsDataTrace):
    def __init__(self) -> None:
        super().__init__(3, 4)

    def process_data(self, data):
        return (
            *data,
            math.sqrt(data[0] ** 2 + data[1] ** 2 + data[2] ** 2),
        )


class SimulationDataVisualizerWindow(ui.Window):
    WINDOW_WIDTH_MIN = 500
    WINDOW_TITLE = "Simulation Data Visualizer"

    PROPERTY_COMPONENT_COLORS = [cl("#AA5555"), cl("#71A376"), cl("#4F7DA0"), cl("AAAA55"), cl("AA55AA"), cl("55AAAA")]
    WINDOW_COLOR = cl("#444444")
    FRAME_COLOR = cl("#323434")
    FIELD_BORDER_RADIUS = 2
    PLOT_BACKGROUND_COLOR = cl("#1F2123")
    SECTION_SPACING = 2
    TOOL_BUTTON_SIZE = 20
    TOOL_BUTTON_STYLE = {
        "background_color": 0x0,
        "padding": 0,
        "margin": 0,
        "color": 0xFFFFFFFF,
        "Tooltip": {"background_color": 0xFF222222, "color": 0xFFDDDDDD},
    }
    TOOL_BUTTON_STYLE_ACTIVATED = {
        "background_color": cl("#1F2123"),
        "padding": 0,
        "margin": 0,
        "color": 0xFFFFFFFF,
        "Tooltip": {"background_color": 0xFF222222, "color": 0xFFDDDDDD},
    }
    PROPERTY_NAME_WIDTH = 100
    PROPERTY_VALUE_WIDTH = (
        WINDOW_WIDTH_MIN - PROPERTY_NAME_WIDTH - 2 * FIELD_BORDER_RADIUS - TOOL_BUTTON_SIZE - 2 - 50
    )  # Latter is for scrollbar and window margin.
    PROPERTY_VALUE_BACKGROUND_COLOR = cl("#1F2123")

    active_style = {}

    @classmethod
    def refresh_style(cls):
        def set_color_opacity(color: int, opacity: float):
            return color % (0x01000000) + min(255, int(opacity * 256.0)) * 0x01000000

        opacity = get_settings().get_as_float(SETTINGS_UI_WINDOW_OPACITY)

        cls.WINDOW_COLOR = set_color_opacity(cls.WINDOW_COLOR, opacity)
        cls.FRAME_COLOR = set_color_opacity(cls.FRAME_COLOR, opacity)
        cls.PROPERTY_VALUE_BACKGROUND_COLOR = set_color_opacity(cls.PROPERTY_VALUE_BACKGROUND_COLOR, opacity)
        cls.PLOT_BACKGROUND_COLOR = set_color_opacity(cls.PLOT_BACKGROUND_COLOR, opacity)

        for n in range(len(cls.PROPERTY_COMPONENT_COLORS)):
            cls.PROPERTY_COMPONENT_COLORS[n] = set_color_opacity(cls.PROPERTY_COMPONENT_COLORS[n], opacity)

        cls.active_style = {
            "Rectangle::hover_highlight": {
                "background_color": 0x0,
                "border_radius": cls.FIELD_BORDER_RADIUS,
                "margin": 0,
                "padding": 0,
            },
            "Rectangle::hover_highlight:hovered": {"background_color": 0x59FFFFFF},
            "Rectangle::property_value": {
                "background_color": cls.PROPERTY_VALUE_BACKGROUND_COLOR,
                "border_color": 0x0,
                "border_width": 0,
                "border_radius": cls.FIELD_BORDER_RADIUS,
            },
            "Rectangle::plot": {"background_color": cls.PLOT_BACKGROUND_COLOR},
            "Rectangle::plot_overlay": {"background_color": cl("#00000000")},
            "Frame::section": {"background_color": cls.FRAME_COLOR},
            "CollapsableFrame::section": {"background_color": cls.FRAME_COLOR},
            "Label::property_name": {"margin": 2},
            "Label::property_value": {"margin": 2, "color": cl("#868E8F")},
            "Tooltip": {"background_color": 0xFF222222, "color": 0xFFDDDDDD},
        }

    class DataProperties:
        def __init__(
            self,
            window,
            name: str,
            components: int = 3,
            component_names: Sequence[str] | None = None,
            component_colors: Sequence[int] | None = None,
            unit_name: str | tuple[str, ...] = "",
            doc: str = "",
        ):
            self._window: SimulationDataVisualizerWindow = window
            self._values = [None] * components
            self._ui_values = [None] * components
            self._name: str = name
            self._ui_label_name = None
            self.unit_name: str | tuple[str, ...] = unit_name
            self._doc: str = doc

            if component_names is None:
                self.component_names = ("X", "Y", "Z", "W")[0:components] if components > 1 else None
            else:
                self.component_names = component_names

            if component_colors is None:
                self._component_colors = (
                    SimulationDataVisualizerWindow.PROPERTY_COMPONENT_COLORS[0:components] if components > 1 else None
                )
            else:
                self._component_colors = component_colors

            self._components_toggle_callback: Callable = None
            self._components_enabled = [True] * components
            self._components_color_rectangles = [None] * components
            self._components_toggle_rectangles = [None] * components

        @property
        def name(self) -> str:
            return self._name

        @name.setter
        def name(self, name: str):
            self._name = name
            if self._ui_label_name is not None:
                self._ui_label_name.text = name

        @property
        def num_components(self):
            return len(self._values)

        def set_components_toggle_callback(self, cb: Callable):
            self._components_toggle_callback = cb
            for rectangle in self._components_toggle_rectangles:
                if rectangle is not None:
                    rectangle.visible = cb is not None
                    rectangle.enabled = cb is not None

        def _toggle_component(self, component, button):
            if button == 1:  # Right click
                enable_all = True
                # If all others are disabled, we enable all, otherwise we disable all others.
                for n in range(self.num_components):
                    if n != component and self._components_enabled[n]:
                        enable_all = False
                        break

                for n in range(self.num_components):
                    self._components_enabled[n] = (n == component) or enable_all
                    self._components_color_rectangles[n].visible = (n == component) or enable_all
            elif button == 0:  # Left click
                self._components_enabled[component] = not self._components_enabled[component]
                self._components_color_rectangles[component].visible = self._components_enabled[component]

            if self._components_toggle_callback:
                self._components_toggle_callback()

        def build_ui(self):
            with ui.HStack():
                BORDER_RADIUS = SimulationDataVisualizerWindow.FIELD_BORDER_RADIUS
                self._ui_label_name = ui.Label(
                    self._name,
                    width=SimulationDataVisualizerWindow.PROPERTY_NAME_WIDTH,
                    name="property_name",
                    tooltip=self._doc,
                )
                ui.Spacer(tooltip=self._doc)
                if self.num_components == 1:
                    with ui.ZStack(width=SimulationDataVisualizerWindow.PROPERTY_VALUE_WIDTH):
                        ui.Rectangle(name="property_value")
                        with ui.HStack(tooltip=self._doc):
                            ui.Spacer(width=BORDER_RADIUS)
                            self._ui_values[0] = ui.Label(
                                "-", elided_text=True, alignment=ui.Alignment.RIGHT, name="property_value"
                            )
                            ui.Spacer(width=BORDER_RADIUS)
                else:
                    RECT_WIDTH = 12
                    with ui.HStack(
                        width=SimulationDataVisualizerWindow.PROPERTY_VALUE_WIDTH, spacing=BORDER_RADIUS + 1
                    ):
                        for index in range(self.num_components):
                            with ui.ZStack():
                                ui.Rectangle(name="property_value")
                                with ui.ZStack(width=RECT_WIDTH + BORDER_RADIUS * 2):
                                    component_color_style = SimulationDataVisualizerWindow.active_style.get(
                                        "Rectangle::property_value"
                                    ).copy()
                                    component_color_style.update({"background_color": self._component_colors[index]})
                                    self._components_color_rectangles[index] = ui.Rectangle(style=component_color_style)
                                    ui.Label(
                                        self.component_names[index][0],
                                        alignment=ui.Alignment.CENTER,
                                        width=RECT_WIDTH + BORDER_RADIUS * 2,
                                        name="property_name",
                                    )
                                    self._components_toggle_rectangles[index] = ui.Rectangle(
                                        visible=(self._components_toggle_callback is not None),
                                        enabled=(self._components_toggle_callback is not None),
                                        width=RECT_WIDTH + BORDER_RADIUS * 2,
                                        tooltip="Left click: toggle this component\nRight click: enable this and toggle all others.",
                                        name="hover_highlight",
                                        alignment=ui.Alignment.LEFT_TOP,
                                        mouse_pressed_fn=lambda x, y, b, m, component=index: self._toggle_component(
                                            component, b
                                        ),
                                    )
                                with ui.HStack():
                                    ui.Spacer(width=RECT_WIDTH + BORDER_RADIUS)
                                    with ui.ZStack(tooltip=self._doc):
                                        block_style = SimulationDataVisualizerWindow.active_style.get(
                                            "Rectangle::property_value"
                                        ).copy()
                                        block_style.update({"border_radius": 0, "border_width": BORDER_RADIUS})
                                        ui.Rectangle(style=block_style)
                                        self._ui_values[index] = ui.Label(
                                            "-", elided_text=True, alignment=ui.Alignment.RIGHT, name="property_value"
                                        )
                                    ui.Spacer(width=BORDER_RADIUS)

            self._values = [None] * self.num_components

        def set_value(self, values: tuple):
            for index in range(len(values)):
                if self._ui_values[index] is not None:
                    if self._values[index] != values[index]:
                        self._values[index] = values[index]
                        if values[index] is None:
                            self._ui_values[index].text = "-"
                        else:
                            self._ui_values[index].text = (
                                str(values[index])
                                if not isinstance(values[index], float)
                                else float_to_string(values[index], min(12, int(32 / len(self._values))))
                            ) + (self.unit_name[index] if isinstance(self.unit_name, tuple) else self.unit_name)

        @property
        def value(self) -> tuple:
            return self._values

    class DataProperty(DataProperties):
        def __init__(self, window, name: str, unit_name: str = "", doc: str = ""):
            super().__init__(window, name, 1, unit_name=unit_name, doc=doc)

        def set_value(self, value):
            super().set_value((value,))

        @property
        def value(self):
            return super().value[0]

    class DataPropertyAxisAngleFromQuaternion(DataProperties):
        def __init__(self, window, name: str, doc: str = ""):
            super().__init__(
                window,
                name,
                4,
                ("X", "Y", "Z", "Angle"),
                unit_name=("", "", "", "°"),
                doc=doc + (" " if doc else "") + "(quaternion input as axis-angle)",
            )

        def set_value(self, values: tuple[float, float, float, float]):
            length = math.sqrt(values[0] ** 2 + values[1] ** 2 + values[2] ** 2)
            if length == 0.0:
                super().set_value((1.0, 0.0, 0.0, 0.0))
                return

            angle = math.degrees(2.0 * math.acos(values[3]))
            if angle > 180.0:
                angle = angle - 360.0
            super().set_value((values[0] / length, values[1] / length, values[2] / length, angle))

    class DataPropertyVectorWithMagnitude(DataProperties):
        def __init__(self, window, name: str, doc: str = ""):
            super().__init__(window, name, 4, ("X", "Y", "Z", "Magnitude"), unit_name="", doc=doc)

        def set_value(self, values: tuple[float, float, float]):
            magnitude = math.sqrt(values[0] ** 2 + values[1] ** 2 + values[2] ** 2)
            super().set_value((*values, magnitude))

    class DataPlot:
        PLOT_STYLE_MARGIN = 2
        PLOT_STYLE_HEIGHT = 100

        def __init__(self, data_property, data_trace: PhysicsDataTrace | None = None):
            self._window: SimulationDataVisualizerWindow = data_property._window
            self._ui_label_min = None
            self._ui_label_max = None
            self._ui_plot_pre_data_spacer = None
            self._ui_plots = []
            self._ui_baseline = None
            self._ui_plot_overlay = None

            self._ui_plot_pre_sim_rect = None

            self._ui_plot_inspector_hstack = None
            self._ui_plot_inspector_line = None
            self._ui_plot_inspector_x = None
            self.data_property: SimulationDataVisualizerWindow.DataProperties = data_property
            self.data_property.set_components_toggle_callback(self._component_filter_cb)

            if data_trace is not None:
                if data_trace.num_components != self.data_property.num_components:
                    log_error(f"SimulationDataVisualizer: Failed to create plot for property {data_property}")
                self._data_trace = data_trace
            else:
                if isinstance(data_property, SimulationDataVisualizerWindow.DataProperty):
                    self._data_trace = PhysicsDataTraceSingle()
                elif isinstance(data_property, SimulationDataVisualizerWindow.DataPropertyAxisAngleFromQuaternion):
                    self._data_trace = PhysicsDataTraceAxisAngleFromQuaternion()
                elif isinstance(data_property, SimulationDataVisualizerWindow.DataPropertyVectorWithMagnitude):
                    self._data_trace = PhysicsDataTraceVectorWithMagnitude()
                else:
                    self._data_trace = PhysicsDataTrace(self.data_property.num_components)

            self._limit_min = [math.inf] * self._data_trace.num_components
            self._limit_max = [-math.inf] * self._data_trace.num_components

            self._limit_min_refresh_timer = [0] * self._data_trace.num_components
            self._limit_max_refresh_timer = [0] * self._data_trace.num_components

        def _component_filter_cb(self):
            self.refresh_plot()

        def _copy_csv_to_clipboard(self):
            csv = self.data_property.name
            data = self._data_trace.get_data()
            trace_time = self._window._simulation_times
            if len(data) <= len(trace_time):
                # If not having data for all time steps, adjust the index to the first available data point.
                trace_time = trace_time[-len(data):]

            csv += "\nTime"
            if self.data_property.component_names is not None:
                for component in range(self._data_trace.num_components):
                    if not self.data_property._components_enabled[component]:
                        continue
                    csv += "," + self.data_property.component_names[component]

            for time, dataplot in zip(trace_time, data):
                csv += "\n" + str(time)

                sample = self._data_trace.process_data(dataplot)
                for component in range(self._data_trace.num_components):
                    if not self.data_property._components_enabled[component]:
                        continue
                    csv += "," + str(sample[component])

            omni.kit.clipboard.copy(csv)
            post_notification("Copied plot CSV to clipboard.")

        def _set_inspection_x(self, x):
            self._ui_plot_inspector_x = x
            if x is not None:
                local_offset = x - self._ui_plot_overlay.screen_position_x
                width = self._ui_plot_overlay.computed_width

                # -1 since offset is zero-based.
                relative_offset = max(min(local_offset / (width - 1), 1.0), 0.0)
                tooltip = self.data_property.name + "\n"

                data_trace = self._data_trace
                current_time = self._window._simulation_time_current
                # The simulation time reduced to the most recent sample. This yields the time at the rightmost part of the plot.
                sampled_time = math.floor(current_time / data_trace._sample_period) * data_trace._sample_period
                # Determine the time at x by using the offset.
                inspected_time = max(
                    0.0, sampled_time - (1.0 - relative_offset) * data_trace._sample_period * (data_trace._samples_num - 1)
                )
                tooltip += "Time:\t" + float_to_string(inspected_time, precision=4)

                data = self._data_trace.get_data()
                trace_time = self._window._simulation_times
                data_index = 0
                if current_time > 0.0:
                    # Attempt to find the index of the last datapoint right before the inspected time. Search by initially assuming that times are evenly distributed
                    data_index = int(inspected_time / current_time)

                    while data_index > 0 and trace_time[data_index] > inspected_time:
                        # Iterate back to the first index before the inspected time.
                        data_index -= 1

                    # Increase by one temporarily and verify that our next datapoint is indeed after the inspected time.
                    data_index += 1
                    trace_time_length = len(trace_time)
                    while data_index < trace_time_length and trace_time[data_index] < inspected_time:
                        data_index += 1
                    data_index -= 1

                    if len(data) <= trace_time_length:
                        # If not having data for all time steps, adjust the index to the last available data point.
                        data_index = data_index - (trace_time_length - len(data))

                if data_index < 0:
                    tooltip += "\nData unavailable."
                else:
                    inspected_data = data[data_index]

                    # Weight the output between this index and the next.
                    if data_index + 1 < len(data):
                        weight = (inspected_time - trace_time[data_index]) / (
                            trace_time[data_index + 1] - trace_time[data_index]
                        )
                        inspected_data = [*inspected_data]
                        for component in range(len(inspected_data)):
                            inspected_data[component] = (
                                inspected_data[component] * (1.0 - weight) + data[data_index + 1][component] * weight
                            )

                    inspected_sample = self._data_trace.process_data(inspected_data)

                    for component in range(self._data_trace.num_components):
                        if not self.data_property._components_enabled[component]:
                            continue
                        tooltip += "\n"
                        if self.data_property.component_names is not None:
                            tooltip += self.data_property.component_names[component] + ":\t"

                        tooltip += float_to_string(inspected_sample[component], precision=4)

                self._ui_plot_overlay.tooltip = tooltip
                self._ui_plot_inspector_line.visible = True
                # Using spacing seems to be the most reliable way to dynamically adjust the offsets.
                self._ui_plot_inspector_hstack.spacing = max(0, min(local_offset, width - 1))
            else:
                self._ui_plot_inspector_line.visible = False
                self._ui_plot_overlay.tooltip = (
                    "Left click: inspect value at point.\nRight click: copy CSV to clipboard."
                )

        def _mouse_moved(self, x: float, y: float, modifiers: int, is_pressed: bool, *args):
            if is_pressed:
                self._set_inspection_x(x)
            else:
                self._set_inspection_x(None)

        def _mouse_hovered(self, is_hovered: bool):
            if not is_hovered:
                self._set_inspection_x(None)

        def _mouse_released(self, x, y, button, modifiers):
            if button == 0:
                self._set_inspection_x(None)

        def _mouse_pressed(self, x, y, button, modifiers):
            if button == 0:
                self._set_inspection_x(x)
            elif button == 1:
                self._copy_csv_to_clipboard()

        def _toggle_plot_visibility(self):
            if self.data_property.name not in SimulationDataVisualizerWindow.SharedStates.plots_enabled:
                SimulationDataVisualizerWindow.SharedStates.plots_enabled.add(self.data_property.name)
            else:
                SimulationDataVisualizerWindow.SharedStates.plots_enabled.remove(self.data_property.name)
            self._plot_stack.visible = (
                self.data_property.name in SimulationDataVisualizerWindow.SharedStates.plots_enabled
            )
            self._ui_toggle_plot_visibility_button.style = (
                SimulationDataVisualizerWindow.TOOL_BUTTON_STYLE_ACTIVATED
                if self.data_property.name in SimulationDataVisualizerWindow.SharedStates.plots_enabled
                else SimulationDataVisualizerWindow.TOOL_BUTTON_STYLE
            )

        def build_ui(self, colors=None):
            with ui.HStack():
                self.data_property.build_ui()

                ui.Spacer(width=2)

                with ui.ZStack(width=SimulationDataVisualizerWindow.TOOL_BUTTON_SIZE):
                    self._ui_toggle_plot_visibility_button = ui.ToolButton(
                        None,
                        clicked_fn=self._toggle_plot_visibility,
                        image_url=icon_plot,
                        fill_policy=ui.FillPolicy.PRESERVE_ASPECT_CROP,
                        image_width=SimulationDataVisualizerWindow.TOOL_BUTTON_SIZE,
                        image_height=SimulationDataVisualizerWindow.TOOL_BUTTON_SIZE,
                        style=(
                            SimulationDataVisualizerWindow.TOOL_BUTTON_STYLE_ACTIVATED
                            if self.data_property.name in SimulationDataVisualizerWindow.SharedStates.plots_enabled
                            else SimulationDataVisualizerWindow.TOOL_BUTTON_STYLE
                        ),
                    )
                    self._ui_toggle_plot_visibility = ui.Rectangle(
                        height=SimulationDataVisualizerWindow.TOOL_BUTTON_SIZE,
                        width=SimulationDataVisualizerWindow.TOOL_BUTTON_SIZE,
                        tooltip="Toggle plot",
                        name="hover_highlight",
                        alignment=ui.Alignment.LEFT_TOP,
                    )

            self._plot_stack = ui.HStack(
                height=100 + 2 * self.PLOT_STYLE_MARGIN,
                visible=self.data_property.name in SimulationDataVisualizerWindow.SharedStates.plots_enabled,
            )
            with self._plot_stack:
                with ui.ZStack():
                    if colors is None:
                        colors = self.data_property._component_colors
                        if colors is None:
                            if self._data_trace.num_components == 1:
                                colors = SimulationDataVisualizerWindow.PROPERTY_COMPONENT_COLORS[1:2]
                            else:
                                colors = SimulationDataVisualizerWindow.PROPERTY_COMPONENT_COLORS[
                                    0 : self._data_trace.num_components
                                ]

                    ui.Rectangle(name="plot")

                    with ui.VStack(spacing=0):
                        ui.Spacer(height=self.PLOT_STYLE_MARGIN)
                        with ui.HStack(spacing=0):
                            ui.Spacer(width=self.PLOT_STYLE_MARGIN)
                            with ui.ZStack():
                                with ui.HStack(spacing=0):
                                    self._ui_plot_pre_data_spacer = ui.Spacer(width=ui.Percent(100))
                                    with ui.ZStack():
                                        data_plots = []
                                        data_trace_samples = self._data_trace.get_samples()
                                        for value in range(self._data_trace.num_components):
                                            data_plots.append(
                                                ui.Plot(
                                                    ui.Type.LINE,
                                                    -1,
                                                    1,
                                                    *data_trace_samples[value],
                                                    width=ui.Percent(100),
                                                    height=self.PLOT_STYLE_HEIGHT,
                                                    style={
                                                        "color": colors[value],
                                                        "background_color": 0x00000000,
                                                        "margin": 0,
                                                        "padding": 0,
                                                    },
                                                )
                                            )
                                self._ui_plots = data_plots

                                self._ui_baseline = ui.Plot(
                                    ui.Type.LINE,
                                    0,
                                    1,
                                    0.0,
                                    0.0,
                                    width=ui.Percent(100),
                                    height=self.PLOT_STYLE_HEIGHT,
                                    style={
                                        "color": 0x44FFFFFF,
                                        "background_color": 0x00000000,
                                        "margin": 0,
                                        "padding": 0,
                                    },
                                )


                                with ui.ZStack(content_clipping=True):
                                    self._ui_plot_pre_sim_rect = ui.Rectangle(style={"background_color": 0x06FFFFFF})

                                    self._ui_plot_overlay = ui.Rectangle(
                                        mouse_pressed_fn=self._mouse_pressed,
                                        name="plot_overlay",
                                        mouse_released_fn=self._mouse_released,
                                        mouse_moved_fn=self._mouse_moved,
                                        mouse_hovered_fn=self._mouse_hovered,
                                        width=ui.Percent(100),
                                        height=self.PLOT_STYLE_HEIGHT #  - 2 * self.PLOT_STYLE_MARGIN,
                                    )
                                    self._ui_plot_inspector_hstack = ui.HStack()
                                    with self._ui_plot_inspector_hstack:
                                        ui.Spacer(width=0)
                                        self._ui_plot_inspector_line = ui.Line(
                                            visible=False,
                                            width=1,
                                            height=self.PLOT_STYLE_HEIGHT, #  - 2 * self.PLOT_STYLE_MARGIN,
                                            alignment=ui.Alignment.H_CENTER,
                                            style={"color": 0x44FFFFFF},
                                        )
                            ui.Spacer(width=self.PLOT_STYLE_MARGIN)
                        ui.Spacer(height=self.PLOT_STYLE_MARGIN)

                    self._set_inspection_x(None)

                    self._ui_label_max = ui.Label(
                        "1.0", style={"font_size": 12.0, "margin": 3.0}, width=0, alignment=ui.Alignment.LEFT_TOP
                    )
                    self._ui_label_min = ui.Label(
                        "-1.0", style={"font_size": 12.0, "margin": 3.0}, width=0, alignment=ui.Alignment.LEFT_BOTTOM
                    )

                    self.refresh_plot()

        def _refresh_limits(self):
            if isinstance(self.data_property, SimulationDataVisualizerWindow.DataPropertyAxisAngleFromQuaternion):
                return -180.0, 180.0

            data = self._data_trace.get_samples()
            data_max = -math.inf
            data_min = math.inf

            for data_trace_component in range(self._data_trace.num_components):
                component_data = data[data_trace_component]

                # Check if the newest added number match or exceed the current limits.
                if component_data[-1] <= self._limit_min[data_trace_component]:
                    # Set the new limit.
                    self._limit_min[data_trace_component] = component_data[-1]
                    # Set the timer until next time we have to evaluate the limit for potential reduction.
                    # We know that it will not be relevant until the current value has been dropped from the data. Add a modifier for reducing the frequency of reevaluation.
                    self._limit_min_refresh_timer[data_trace_component] = len(component_data) + int(
                        len(component_data) / 5 + 0.5
                    )
                else:
                    # Otherwise, reduce the counter by one.
                    self._limit_min_refresh_timer[data_trace_component] -= 1
                    if self._limit_min_refresh_timer[data_trace_component] <= 0:
                        # Reevaluate the limit by cycling all current data for the component.
                        self._limit_min[data_trace_component] = math.inf
                        for index in range(len(component_data)):
                            if self._limit_min[data_trace_component] >= component_data[index]:
                                self._limit_min[data_trace_component] = component_data[index]
                                self._limit_min_refresh_timer[data_trace_component] = index + int(
                                    len(component_data) / 5 + 0.5
                                )

                # Same process for max limit.
                if component_data[-1] >= self._limit_max[data_trace_component]:
                    self._limit_max_refresh_timer[data_trace_component] = len(component_data) + int(
                        len(component_data) / 5 + 0.5
                    )
                    self._limit_max[data_trace_component] = component_data[-1]
                else:
                    self._limit_max_refresh_timer[data_trace_component] -= 1
                    if self._limit_max_refresh_timer[data_trace_component] <= 0:
                        self._limit_max[data_trace_component] = -math.inf
                        for index in range(len(component_data)):
                            if self._limit_max[data_trace_component] <= component_data[index]:
                                self._limit_max[data_trace_component] = component_data[index]
                                self._limit_max_refresh_timer[data_trace_component] = index + int(
                                    len(component_data) / 5 + 0.5
                                )

                # Only factor it into the visual output if the component plot is actually visible.
                if (
                    data_trace_component >= self.data_property.num_components
                    or self.data_property._components_enabled[data_trace_component]
                ):
                    if self._limit_min[data_trace_component] < data_min:
                        data_min = self._limit_min[data_trace_component]
                    if self._limit_max[data_trace_component] > data_max:
                        data_max = self._limit_max[data_trace_component]

            # Note that this function only accepts positive numbers.
            def adjust_limit(limit: float):
                if limit > 0.0001:
                    range_factor = math.pow(10.0, math.floor(math.log10(limit)))
                    return math.ceil(limit / range_factor) * range_factor
                else:
                    return 0.0001

            # If having negatives, set offset of min and max to always match distance from baseline. This makes the graphs easier to read.
            if data_min < 0.0:
                if data_min < -data_max:
                    data_min = -adjust_limit(-data_min)
                    data_max = -data_min
                else:
                    data_max = adjust_limit(data_max)
                    data_min = -data_max
            elif data_max > 0.0:
                data_max = adjust_limit(data_max)
                data_min = 0.0
            else:
                data_min = -1.0
                data_max = 1.0

            return data_min, data_max

        def reset_data(self):
            self._data_trace.reset()
            self._limit_min = [math.inf] * self._data_trace.num_components
            self._limit_max = [-math.inf] * self._data_trace.num_components

            self.refresh_plot()

        def refresh_plot(self):
            if self._ui_label_min is None:
                return

            data_min, data_max = self._refresh_limits()

            unit_name = "" if isinstance(self.data_property.unit_name, tuple) else self.data_property.unit_name
            self._ui_label_min.text = float_to_string(data_min, precision=4) + unit_name
            self._ui_label_max.text = float_to_string(data_max, precision=4) + unit_name

            data_samples = self._data_trace.get_samples()
            
            self._ui_plot_pre_data_spacer.width = ui.Percent(100.0 -  ((len(data_samples[0]) - 1) / (self._data_trace._samples_num - 1)) * 100.0)

            for n in range(self._data_trace.num_components):
                self._ui_plots[n].scale_min = data_min
                self._ui_plots[n].scale_max = data_max
                self._ui_plots[n].set_data(*data_samples[n])
                if n < self.data_property.num_components:
                    self._ui_plots[n].visible = self.data_property._components_enabled[n]

            self._ui_baseline.scale_min = data_min
            self._ui_baseline.scale_max = data_max

            pre_sim_width = 1.0 - self._window._simulation_time_current / (
                (self._data_trace._samples_num - 1) * self._data_trace._sample_period
            )

            if pre_sim_width < 0.0:
                self._ui_plot_pre_sim_rect.visible = False
            else:
                self._ui_plot_pre_sim_rect.visible = True
                self._ui_plot_pre_sim_rect.width = ui.Percent(100.0 * pre_sim_width)

            self._set_inspection_x(self._ui_plot_inspector_x)

        def set_value(self, values):
            self.data_property.set_value(values)

        def append_data(self, time_delta, values):
            if time_delta > 0.0:
                if self._data_trace.append(time_delta, values):
                    self.refresh_plot()
            else:
                self._data_trace.set_data(values)
                self.refresh_plot()

            self.data_property.set_value(values)

    class Monitor:
        def __init__(self, window):
            self._window: SimulationDataVisualizerWindow = window

        @property
        def stage(self):
            return self._window._stage

        @property
        def prim(self):
            return self._window._prim

        @property
        def prim_path(self):
            return self._window._prim_path

        def refresh_simulation_data(self, delta_time=0.0):
            pass

        def reset_plots(self):
            pass

        def refresh_data(self):
            self.refresh_simulation_data()

        def build_ui(self):
            pass

    class MonitorPrimBasic(Monitor):
        def __init__(self, window):
            super().__init__(window)

            class DataPropertyPath(SimulationDataVisualizerWindow.DataProperty):

                icon_unlocked = str(icons_folder.joinpath("window_unlocked.svg"))
                icon_locked = str(icons_folder.joinpath("window_locked.svg"))
                icon_new_window = str(icons_folder.joinpath("window_new.svg"))

                def __init__(self, window, name):
                    super().__init__(window, name)
                    self._window_lock_button = None
                    self._window_new_button = None

                def _refresh_locked_state(self):
                    if window._prim_locked:
                        self._window_lock_button.style = SimulationDataVisualizerWindow.TOOL_BUTTON_STYLE_ACTIVATED
                        self._window_lock_button.image_url = self.icon_locked
                    else:
                        self._window_lock_button.style = SimulationDataVisualizerWindow.TOOL_BUTTON_STYLE
                        self._window_lock_button.image_url = self.icon_unlocked

                def toggle_locked(self):
                    window._prim_locked = not window._prim_locked
                    self._refresh_locked_state()
                    if not window._prim_locked:
                        window._refresh_target()

                def new_window(self):
                    window._prim_locked = True
                    self._refresh_locked_state()
                    window._window_manager.add_window(window)

                def build_ui(self):
                    with ui.HStack():
                        super().build_ui()

                        ui.Spacer(width=2)

                        with ui.ZStack(width=SimulationDataVisualizerWindow.TOOL_BUTTON_SIZE):
                            self._window_lock_button = ui.ToolButton(
                                None,
                                clicked_fn=self.toggle_locked,
                                image_url=self.icon_unlocked,
                                fill_policy=ui.FillPolicy.PRESERVE_ASPECT_CROP,
                                image_width=SimulationDataVisualizerWindow.TOOL_BUTTON_SIZE,
                                image_height=SimulationDataVisualizerWindow.TOOL_BUTTON_SIZE,
                                style=SimulationDataVisualizerWindow.TOOL_BUTTON_STYLE,
                            )
                            self._refresh_locked_state()
                            ui.Rectangle(
                                height=SimulationDataVisualizerWindow.TOOL_BUTTON_SIZE,
                                width=SimulationDataVisualizerWindow.TOOL_BUTTON_SIZE,
                                tooltip="Lock window to this prim",
                                name="hover_highlight",
                                alignment=ui.Alignment.LEFT_TOP,
                            )

                        ui.Spacer(width=2)

                        with ui.ZStack(width=SimulationDataVisualizerWindow.TOOL_BUTTON_SIZE):
                            self._window_new_button = ui.ToolButton(
                                None,
                                clicked_fn=self.new_window,
                                image_url=self.icon_new_window,
                                fill_policy=ui.FillPolicy.PRESERVE_ASPECT_CROP,
                                image_width=20,
                                image_height=20,
                                style=SimulationDataVisualizerWindow.TOOL_BUTTON_STYLE,
                            )
                            ui.Rectangle(
                                height=SimulationDataVisualizerWindow.TOOL_BUTTON_SIZE,
                                width=SimulationDataVisualizerWindow.TOOL_BUTTON_SIZE,
                                tooltip="Create new window (locks the current)",
                                name="hover_highlight",
                                alignment=ui.Alignment.LEFT_TOP,
                            )

            self._property_path = DataPropertyPath(window, "Path")

        def refresh_data(self):
            super().refresh_data()
            self._property_path.set_value(self.prim_path if (self.prim is not None and self.prim.IsValid()) else None)

        def build_ui(self):
            super().build_ui()

            with ui.Frame(height=0, name="section"):
                with ui.HStack():
                    with ui.VStack(spacing=SimulationDataVisualizerWindow.SECTION_SPACING):
                        self._property_path.build_ui()

    class MonitorSubsection(Monitor):
        def __init__(self, window):
            super().__init__(window)

        def build_collapsable_section(self, name, build_func, build_header_func=None):
            def on_collapsed_changed(collapsed):
                if not collapsed:
                    SimulationDataVisualizerWindow.SharedStates.sections_expanded.add(name)
                else:
                    SimulationDataVisualizerWindow.SharedStates.sections_expanded.discard(name)

            with ui.CollapsableFrame(
                name,
                opaque_for_mouse_events=True,
                height=0,
                name="section",
                collapsed=(name not in SimulationDataVisualizerWindow.SharedStates.sections_expanded),
                collapsed_changed_fn=on_collapsed_changed,
                build_header_fn=build_header_func,
            ):
                with ui.HStack():
                    with ui.VStack(spacing=SimulationDataVisualizerWindow.SECTION_SPACING):
                        build_func()

    """
    Template for a derived MonitorSubsection class: 
    class MonitorSubsectionDerived(MonitorSubsection):
        def __init__(self, window):
            super().__init__(window)

        def refresh_simulation_data(self, delta_time = 0.0):
            super().refresh_simulation_data(delta_time)

        def refresh_data(self):
            super().refresh_data()

        def _build_section_content_ui(self):
            self._plot_world_position.build_ui()
            self._plot_world_orientation.build_ui()

        def build_ui(self):
            super().build_ui()
            self.build_collapsable_section("Section Name", self._build_section_content_ui)
    """

    class MonitorPrimSimulationData(Monitor):
        def __init__(self, window, debug_data_name: str, debug_data_info: dict):
            super().__init__(window)

            self.identifier = debug_data_name
            self._datatype = debug_data_info.get("type", DebugDataItemType.UNDEFINED)

            if self._datatype == DebugDataItemType.FLOAT:
                self._data_property = SimulationDataVisualizerWindow.DataProperty(
                    window, debug_data_name, doc=debug_data_info.get("doc", "")
                )
                self._plot = SimulationDataVisualizerWindow.DataPlot(self._data_property)
            elif self._datatype == DebugDataItemType.VECTOR:
                self._data_property = SimulationDataVisualizerWindow.DataPropertyVectorWithMagnitude(
                    window, debug_data_name, doc=debug_data_info.get("doc", "")
                )
                self._plot = SimulationDataVisualizerWindow.DataPlot(self._data_property)
            elif self._datatype == DebugDataItemType.POINT:
                self._data_property = SimulationDataVisualizerWindow.DataProperties(
                    window, debug_data_name, 3, doc=debug_data_info.get("doc", "")
                )
                self._plot = SimulationDataVisualizerWindow.DataPlot(self._data_property)
            elif self._datatype == DebugDataItemType.QUATERNION:
                self._data_property = SimulationDataVisualizerWindow.DataPropertyAxisAngleFromQuaternion(
                    window, debug_data_name, doc=debug_data_info.get("doc", "")
                )
                self._plot = SimulationDataVisualizerWindow.DataPlot(self._data_property)
            else:
                self._plot = None
                self._data_property = SimulationDataVisualizerWindow.DataProperty(
                    window, debug_data_name, doc=debug_data_info.get("doc", "")
                )

            self.refresh_simulation_data(0.0)

        def refresh_simulation_data(self, delta_time=0.0):
            value = self._window._debug_data.get(self.identifier, {}).get("value", None)
            if value is None:
                return

            if self._plot is not None:
                self._plot.append_data(delta_time, value)
            else:
                self._data_property.set_value(value)

            super().refresh_simulation_data(delta_time)

        def reset_plots(self):
            if self._plot is not None:
                self._plot.reset_data()

        def build_ui(self):
            super().build_ui()

            if self._plot is not None:
                self._plot.build_ui()
            else:
                self._data_property.build_ui()

    class SharedStates:
        sections_expanded = set()
        plots_enabled = set()

    def __init__(self, window_manager, title):
        super().__init__(
            title, visible=True, flags=ui.WINDOW_FLAGS_NO_SCROLLBAR, width=self.WINDOW_WIDTH_MIN, height=500
        )

        self._physics_interaction_interface = get_physics_interaction_interface()
        if self._physics_interaction_interface is None:
            log_error(
                "Failed to initialize SimulationDataVisualizer window: Physics interaction interface is unavailable"
            )
            return
        if get_physics_simulation_interface() is None:
            log_error(
                "Failed to initialize SimulationDataVisualizer window: Physics simulation interface is unavailable"
            )
            return
        physics_stage_update_interface = get_physics_stage_update_interface()
        if physics_stage_update_interface is None:
            log_error(
                "Failed to initialize SimulationDataVisualizer window: Physics stage update interface is unavailable"
            )
            return

        self._window_manager: SimulationDataVisualizerWindowManager = window_manager
        self.frame.set_build_fn(self.build)

        self._usd_context: UsdContext = get_usd_context()
        self._stage_event_sub = [
            get_eventdispatcher().observe_event(
                observer_name="omni.physics.ui:SimulationDataVisualizer",
                event_name=self._usd_context.stage_event_name(event),
                on_event=func,
            )
            for event, func in (
                (StageEventType.OPENED, self._attach_stage),
                (StageEventType.CLOSING, self._detach_stage),
                (StageEventType.SELECTION_CHANGED, self._refresh_target),
            )
        ]
        self._stage_update_node = None
        self._stage: Usd.Stage = None

        # Get the physics stage update interface
        self._simulation_event_sub = (
            physics_stage_update_interface.get_simulation_event_stream().create_subscription_to_pop(
                self._on_simulation_event
            )
        )

        physics = get_physics_interface()
        self._simulation_registry_sub = physics.subscribe_simulation_registry_events(self._on_simulation_registry_event)

        self._simulation_step_sub = None
        self._simulation_time_current = 0.0
        self._simulation_updated = False
        self._simulation_times = [0.0]
        self._usd_object_changed_listener = None

        self._data_monitor_basic: __class__.MonitorPrimBasic = None
        self._prim_simulation_data_monitors: dict[str, __class__.MonitorPrimSimulationData] = {}
        self._debug_data: dict[str, Any] = {}

        self._prim_path: Sdf.Path | None = None
        self._prim: Usd.Prim | None = None
        self._prim_locked = False

        self.set_visibility_changed_fn(self._window_visiblity_changed_fn)

        self._attach_stage()

    def _window_visiblity_changed_fn(self, visible):
        # handle the case when user closes the window by a cross
        if not visible:
            self._window_manager.refresh()

    def _on_simulation_registry_event(self, event_type: SimulationRegistryEventType, id: SimulationId, name: str):
        self._refresh_monitor_simulation_data()

    def _refresh_monitor_data_monitors(self):
        for monitor_key in list(self._prim_simulation_data_monitors.keys()):
            if monitor_key not in self._debug_data.keys():
                self._prim_simulation_data_monitors.pop(monitor_key)

        for key, value in self._debug_data.items():
            if key not in self._prim_simulation_data_monitors.keys():
                self._prim_simulation_data_monitors[key] = __class__.MonitorPrimSimulationData(
                    self, key, value
                )

    def _refresh_monitor_data(self):
        if self._prim_path is not None:
            self._debug_data = self._physics_interaction_interface.get_prim_debug_data(self._prim_path.pathString)
        else:
            self._debug_data = {}

        if self._debug_data.keys() != self._prim_simulation_data_monitors.keys():
            self._refresh_monitor_data_monitors()
            self.frame.rebuild()
            return

        if self._data_monitor_basic is not None:
            self._data_monitor_basic.refresh_data()
        for monitor in self._prim_simulation_data_monitors.values():
            if monitor is not None:
                monitor.refresh_data()

    def _refresh_monitor_simulation_data(self, time_delta=0.0):
        if self._prim_path is not None:
            self._debug_data = self._physics_interaction_interface.get_prim_debug_data(self._prim_path.pathString)
        else:
            self._debug_data = {}

        if self._debug_data.keys() != self._prim_simulation_data_monitors.keys():
            self._refresh_monitor_data_monitors()
            self.frame.rebuild()
            return

        for monitor in self._prim_simulation_data_monitors.values():
            monitor.refresh_simulation_data(time_delta)

    def _reset_monitor_data(self):
        for monitor in self._prim_simulation_data_monitors.values():
            monitor.reset_plots()

        self._refresh_monitor_data()

    def _refresh_target(self, event: Event | None = None):
        if (self._prim_locked and self._prim is not None) or self._stage is None:
            return

        selected_prim_path = self._usd_context.get_selection().get_selected_prim_paths()
        if len(selected_prim_path) > 0:
            # Always use the most recently (last) selected prim.
            selected_prim_path = Sdf.Path(selected_prim_path[-1])
        else:
            selected_prim_path = None

        self._set_target_path(selected_prim_path)

    def _set_target_path(self, prim_path: Sdf.Path):
        if prim_path == self._prim_path:
            return

        self._prim_path = prim_path

        if prim_path is not None:
            self._prim = self._stage.GetPrimAtPath(prim_path)

        else:
            self._prim = None

        self._reset_monitor_data()

    def _on_simulation_step(self, delta_time, context):
        # To support manual stepping, we use this to track if the simulation has been updated.
        self._simulation_updated = True

    def _on_stage_update(self, current_time, delta_time):
        if not self._simulation_updated:
            return

        self._simulation_times.append(self._simulation_times[-1] + delta_time)
        self._simulation_time_current += delta_time

        self._refresh_monitor_simulation_data(delta_time)

        self._simulation_updated = False

    def _attach_stage(self, event: Event | None = None):
        self._stage = self._usd_context.get_stage()
        if self._stage is not None:
            self._attach_usd_change_listener()
            self._simulation_step_sub = get_physics_simulation_interface().subscribe_physics_on_step_events(
                False, 0, self._on_simulation_step
            )
            self.stage_update_node = register_stage_update_node(
                    "simDataVisualization", on_update_fn=self._on_stage_update, priority=12
                )
        self._refresh_target()

    def _detach_stage(self, event: Event | None = None):
        self._simulation_step_sub = None
        self._stage_update_node = None
        self._stage = None
        self._set_target_path(None)
        self._revoke_usd_change_listener()

    def _attach_usd_change_listener(self):
        if self._usd_object_changed_listener is None and self._stage is not None:
            self._usd_object_changed_listener = Tf.Notice.Register(
                Usd.Notice.ObjectsChanged, self._on_usd_objects_changed, self._stage
            )

    def _revoke_usd_change_listener(self):
        if self._usd_object_changed_listener is not None:
            self._usd_object_changed_listener.Revoke()
            self._usd_object_changed_listener = None

    def _on_usd_objects_changed(self, notice, stage):
        if self._prim is None:
            return

        if len(notice.GetResyncedPaths()) > 0:
            # If paths have been resynced, it means that the path of our target prim could have changed or been deleted.
            if not self._prim.IsValid():
                # Set to none here. The selection changed event will make sure that the new path is selected.
                self._set_target_path(None)
                return

        for changed_path in notice.GetChangedInfoOnlyPaths():
            changed_prim_path = changed_path.GetPrimPath()
            if self._prim_path.HasPrefix(changed_prim_path) or changed_prim_path == self._prim_path:
                self._refresh_monitor_data()
                break

    def _on_simulation_event(self, event):
        if event.type == int(SimulationEvent.RESUMED):
            # During simulation we only listen for specific attributes that are then queried each frame.
            self._revoke_usd_change_listener()
            if self._simulation_time_current == 0.0:
                self._refresh_monitor_simulation_data()
        elif event.type == int(SimulationEvent.PAUSED) or event.type == int(SimulationEvent.STOPPED):
            if event.type == int(SimulationEvent.STOPPED):
                self._simulation_time_current = 0.0
                self._simulation_updated = False
                self._simulation_times.clear()
                self._simulation_times.append(0.0)
                self._reset_monitor_data()
            self._attach_usd_change_listener()

    def build(self):
        if not self.visible:
            return

        self.frame.set_style(
            {
                "Window": {
                    "background_color": self.WINDOW_COLOR,
                    "border_color": 0x0,
                    "border_width": 0,
                    "border_radius": 5,
                }
            }
        )

        with self.frame:
            vstack_outer = ui.VStack(spacing=5, style=__class__.active_style)
            with vstack_outer:
                if self._data_monitor_basic is None:
                    self._data_monitor_basic = __class__.MonitorPrimBasic(self)
                self._data_monitor_basic.build_ui()
                self._data_monitor_basic.refresh_data()
                with ui.ScrollingFrame():
                    with ui.VStack(height=0, spacing=5):
                        for monitor in self._prim_simulation_data_monitors.values():
                            monitor.build_ui()
                            monitor.refresh_data()

    def destroy(self):
        self._detach_stage()
        self._stage_event_sub = None
        self._simulation_event_sub = None
        self._simulation_step_sub = None
        self._simulation_registry_sub = None
        self.set_visibility_changed_fn(None)
        super().destroy()


class SimulationDataVisualizerWindowManager:
    def __init__(self) -> None:
        self._windows: list[SimulationDataVisualizerWindow] = []

        self._enabled_setting_sub = get_settings().subscribe_to_node_change_events(
           SETTINGS_UI_PHYSICS_SIMULATION_DATA_VISUALIZER_ENABLED, self._on_enabled_setting_changed
        )        
        self._background_opacity_setting_sub = None
        self._menu_builder_registered = False
        self._register_viewport_menu_builder()
        self._enabled = False
        self._set_enabled(get_settings().get_as_bool(SETTINGS_UI_PHYSICS_SIMULATION_DATA_VISUALIZER_ENABLED))

    @staticmethod
    def _build_simulation_data_visualizer_menu_item(settings, helper):
        import omni.ui as ui
        from omni.kit.viewport.menubar.core import SelectableMenuItem

        def on_value_changed(model, menu_item):
            show = model.get_value_as_bool()
            settings.set(SETTINGS_UI_PHYSICS_SIMULATION_DATA_VISUALIZER_ENABLED, show)

        # Create the menu item
        menu_item = SelectableMenuItem(
            "Simulation Data Visualizer",
            ui.SimpleBoolModel(settings.get_as_bool(SETTINGS_UI_PHYSICS_SIMULATION_DATA_VISUALIZER_ENABLED))
        )
        menu_item.model.add_value_changed_fn(lambda model: on_value_changed(model, menu_item))

        # Subscribe to setting changes to keep menu in sync
        helper.safe_subscribe_to_setting_change(None, [menu_item], SETTINGS_UI_PHYSICS_SIMULATION_DATA_VISUALIZER_ENABLED)

    def _register_viewport_menu_builder(self):
        """Register the menu builder with PhysicsViewportMenuHelper."""
        try:
            PhysicsViewportMenuHelper.register_additional_menu_builder(self._build_simulation_data_visualizer_menu_item)
            self._menu_builder_registered = True
        except ImportError:
            pass

    def _on_enabled_setting_changed(self, item, event_type):
        self._set_enabled(get_settings().get_as_bool(SETTINGS_UI_PHYSICS_SIMULATION_DATA_VISUALIZER_ENABLED))

    def _set_enabled(self, enabled: bool):
        if self._enabled == enabled:
            return
        self._enabled = enabled
        if enabled:
            self._background_opacity_setting_sub = get_settings().subscribe_to_node_change_events(
                SETTINGS_UI_WINDOW_OPACITY, self._on_opacity_setting_changed
            )
            SimulationDataVisualizerWindow.refresh_style()
            if len(self._windows) == 0:
                self.add_window()
        else:
            self._background_opacity_setting_sub = None
            for window in self._windows:
                window.destroy()
            self._windows.clear()

    def add_window(self, creator_window: SimulationDataVisualizerWindow = None):
        window_name = SimulationDataVisualizerWindow.WINDOW_TITLE
        suffix = 1
        window = ui.Workspace.get_window(window_name)
        while window is not None:
            if not isinstance(window, SimulationDataVisualizerWindow):
                # Even if the window was destroyed, it will still show up but only as a handle. In this case we just overwrite it.
                break

            suffix += 1
            window_name = SimulationDataVisualizerWindow.WINDOW_TITLE + " #" + str(suffix)
            window = ui.Workspace.get_window(window_name)

        window = SimulationDataVisualizerWindow(self, window_name)

        self._windows.append(window)

        if creator_window is not None:
            if creator_window.docked:
                window.deferred_dock_in(creator_window.title, ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
            else:
                window.position_x = creator_window.position_x + 40
                window.position_y = creator_window.position_y + 40
                window.width = creator_window.width
                window.height = creator_window.height
                window.focus()
        else:
            window.deferred_dock_in("Content", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)

    def refresh(self):
        windows_visible = []
        for window in self._windows:
            if window.visible:
                windows_visible.append(window)
            else:
                window.destroy()

        self._windows = windows_visible

        if len(self._windows) < 1:
            get_settings().set_bool(SETTINGS_UI_PHYSICS_SIMULATION_DATA_VISUALIZER_ENABLED, False)

    def _on_opacity_setting_changed(self, item, event_type):
        SimulationDataVisualizerWindow.refresh_style()
        for window in self._windows:
            if window is not None and window.visible:
                window.frame.rebuild()

    def destroy(self):
        for window in self._windows:
            window.destroy()
        self._windows.clear()
        if self._background_opacity_setting_sub is not None:
            self._background_opacity_setting_sub = None
        if self._enabled_setting_sub is not None:
            self._enabled_setting_sub = None
        if self._menu_builder_registered:
            PhysicsViewportMenuHelper.unregister_additional_menu_builder(self._build_simulation_data_visualizer_menu_item)
            self._menu_builder_registered = False

    def __del__(self):
        self.destroy()
