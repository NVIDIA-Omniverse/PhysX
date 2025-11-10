# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb.input
from carb.eventdispatcher import get_eventdispatcher
from carb.settings import get_settings
import omni.appwindow
from omni import ui
from omni.physx.bindings._physx import SimulationEvent, SETTING_DISPLAY_SIMULATION_DATA_VISUALIZER
import omni.usd
from omni.ui import color as cl
from omni.physx.scripts import propertyQueryRigidBody
from .physicsViewportShared import *
from pxr import Usd, Gf, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Tf
import collections
from collections.abc import Iterable, Sequence
import math
from .utils import register_stage_update_node
from pathlib import Path
from typing import Callable
import copy
import omni.kit.clipboard
import omni.kit.notification_manager as nm
from usdrt import Usd as UsdRt, Rt as UsdRtGeom, UsdPhysics as UsdRtPhysics, Gf as UsdRtGf


SETTINGS_UI_WINDOW_OPACITY = "/persistent/app/viewport/ui/background/opacity"


def rotation_to_usdrt(value):
    if isinstance(value, Gf.Rotation):
        return UsdRtGf.Rotation(value.GetQuat())
    return value

def vec3_to_usdrt(value):
    if isinstance(value, Gf.Vec3d) or isinstance(value, Gf.Vec3f) or isinstance(value, UsdRtGf.Vec3f):
        return UsdRtGf.Vec3d(*value)
    return value

def vec4_to_usdrt(value):
    if isinstance(value, Gf.Vec4d) or isinstance(value, Gf.Vec4f) or isinstance(value, UsdRtGf.Vec4f):
        return UsdRtGf.Vec4d(*value)
    return value

def matrix4_to_usdrt(value):
    if isinstance(value, Gf.Matrix4d) or isinstance(value, Gf.Matrix4f) or isinstance(value, UsdRtGf.Matrix4f):
        # return UsdRtGf.Matrix4d().Set(*value[0], *value[1], *value[2], *value[3])
        return UsdRtGf.Matrix4d().Set(*value[0], *value[1], *value[2], *value[3])
    return value

def matrix3_to_usdrt(value):
    if isinstance(value, Gf.Matrix3d) or isinstance(value, Gf.Matrix3f) or isinstance(value, UsdRtGf.Matrix3f):
        return UsdRtGf.Matrix3d().Set(*value[0], *value[1], *value[2])
    return value

def quaternion_to_usdrt(value):
    if (isinstance(value, Gf.Quatd) or isinstance(value, Gf.Quatf) or isinstance(value, Gf.Quaternion)
        or isinstance(value, UsdRtGf.Quatf) or isinstance(value, UsdRtGf.Quaternion)):
        return UsdRtGf.Quatd(value.GetReal(), UsdRtGf.Vec3d(*value.GetImaginary()))
    return value

def axis_token_to_index(axis : UsdGeom.Tokens) -> int:
    if axis == UsdGeom.Tokens.x:
        return 0
    elif axis == UsdGeom.Tokens.y:
        return 1
    return 2


class PhysicsDataTrace():
    """
    Stores data input and generates sample outputs with a defined sample period.

    The sample outputs are used to draw graphs that may need a different format than the input (e.g. quaternions vs Eulers),
    and a different frequency since too many datapoints will cause visual jitter.
    """
    def __init__(self, data_num_components, samples_num_components = None, samples_num = 500, sample_period=0.05) -> None:
        self._data_num_components = data_num_components
        self._samples_num = samples_num
        self._samples_num_components = data_num_components if samples_num_components is None else samples_num_components
        self._sample_period = sample_period
        self.reset()

    def reset(self):
        self._input_accumulator = [0.0] * self._data_num_components
        self._input_accumulator_time_elapsed = 0.0

        # The raw input datapoints stored in a list of tuples.
        self._trace_datapoints : list = [(0.0,) * self._data_num_components]

        # Generated sample output as deques for plotting graphs.
        self._trace_samples : list[collections.deque[float]] = []
        for _ in range(self._samples_num_components):
            self._trace_samples.append(collections.deque([0.0] * self._samples_num, maxlen=self._samples_num))

    # Overridable function for data types that may need further output processing (e.g. quaternions to Eulers).
    def process_data(self, data : Sequence[float]) -> Sequence[float]:
        return data

    def set_data(self, data : tuple[float, ...]):
        sample = self.process_data(data)
        if self._input_accumulator_time_elapsed == 0.0:
            # This means that we haven't set any yet. Reset to this data.
            for n in range(self._samples_num_components):
                self._trace_samples[n] = collections.deque([sample[n]] * self._samples_num, maxlen=self._samples_num)

            self._input_accumulator = [0.0] * self._data_num_components
            self._trace_datapoints = [data]
        else:
            # Manipulating while simulation is paused. Adjust the most recent datapoint.
            for n in range(self._samples_num_components):
                self._trace_samples[n][-1] = sample[n]
            self._trace_datapoints[-1] = data

    # Appends a datapoint to the accumulator. Returns true if a new output was added to the sampler.
    def append(self, delta_time, data : tuple[float, ...]) -> bool:
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


class PhysicsDataTraceEulersFromQuaternion(PhysicsDataTrace):
    def __init__(self, rotation_order : tuple[int, int, int] =  (0, 1, 2)) -> None:
        self.rotation_order = rotation_order
        super().__init__(4, 3)

    @property
    def rotation_order(self) -> tuple[int, int, int]:
        return self._rotation_order

    @rotation_order.setter
    def rotation_order(self, rotation_order : tuple[int, int, int] = (0, 1, 2)):
        self._rotation_order = rotation_order
        axes = (UsdRtGf.Vec3d.XAxis(), UsdRtGf.Vec3d.YAxis(), UsdRtGf.Vec3d.ZAxis())
        self._rotation_axes = (axes[rotation_order[0]], axes[rotation_order[1]], axes[rotation_order[2]])

    def process_data(self, data):
        rotation = UsdRtGf.Rotation(UsdRtGf.Quatd(data[3], UsdRtGf.Vec3d(data[0], data[1], data[2])))
        return (*rotation.Decompose(*self._rotation_axes),)

    def append(self, delta_time, data : Gf.Quaternion | Gf.Quatd | Gf.Quatf | UsdRtGf.Quatd | UsdRtGf.Quatf) -> bool:
        return super().append(delta_time, (*data.GetImaginary(), data.GetReal()))


class PhysicsDataTraceAngleFromQuaternion(PhysicsDataTrace):
    def __init__(self, axis : int) -> None:
        self.axis = axis
        super().__init__(4, 1)

    @property
    def axis(self):
        return self._axis

    @axis.setter
    def axis(self, axis : int):
        self._axis = axis
        axes = (UsdRtGf.Vec3d.XAxis(), UsdRtGf.Vec3d.YAxis(), UsdRtGf.Vec3d.ZAxis())
        self._rotation_axes = (axes[self._axis], axes[(self._axis + 1) % 3], axes[(self._axis + 2) % 3])

    def process_data(self, data):
        rotation = UsdRtGf.Rotation(UsdRtGf.Quatd(data[3], UsdRtGf.Vec3d(data[0], data[1], data[2])))
        return (rotation.Decompose(*self._rotation_axes)[0],)

    def append(self, delta_time, data : Gf.Quaternion | Gf.Quatd | Gf.Quatf | UsdRtGf.Quatd | UsdRtGf.Quatf) -> bool:
        return super().append(delta_time, (*data.GetImaginary(), data.GetReal()) if data.GetReal() > 0.0
                                  else (*-data.GetImaginary(), -data.GetReal())
                                  )


class PhysicsDataTraceVector(PhysicsDataTrace):
    def __init__(self) -> None:
        super().__init__(3)


class PhysicsDataTraceSingle(PhysicsDataTrace):
    def __init__(self) -> None:
        super().__init__(1)

    def set_data(self, data : float):
        return super().set_data((data,))

    def append(self, delta_time, data) -> bool:
        return super().append(delta_time, (data,))


class SimulationDataVisualizerWindow(ui.Window):
    WINDOW_WIDTH_MIN = 500
    WINDOW_TITLE = "Simulation Data Visualizer"

    PROPERTY_COMPONENT_COLORS = [cl("#AA5555"), cl("#71A376"), cl("#4F7DA0"), cl("AAAA55"), cl("AA55AA"), cl("55AAAA")]
    WINDOW_COLOR = cl("#444444")
    FRAME_COLOR = cl("#323434")
    FIELD_BORDER_RADIUS = 2
    PLOT_BACKGROUND_COLOR = cl("#1F2123")
    SECTION_SPACING = 2
    PROPERTY_NAME_WIDTH = 100
    PROPERTY_VALUE_WIDTH = WINDOW_WIDTH_MIN - PROPERTY_NAME_WIDTH - 2 * FIELD_BORDER_RADIUS - 50
    PROPERTY_VALUE_BACKGROUND_COLOR = cl("#1F2123")

    active_style = {}

    @classmethod
    def refresh_style(cls):
        def set_color_opacity(color : int, opacity : float):
            return color % (0x01000000) + min(255, int(opacity * 256.0)) * 0x01000000

        opacity = get_settings().get_as_float(SETTINGS_UI_WINDOW_OPACITY)

        cls.WINDOW_COLOR = set_color_opacity(cls.WINDOW_COLOR, opacity)
        cls.FRAME_COLOR = set_color_opacity(cls.FRAME_COLOR, opacity)
        cls.PROPERTY_VALUE_BACKGROUND_COLOR = set_color_opacity(cls.PROPERTY_VALUE_BACKGROUND_COLOR, opacity)
        cls.PLOT_BACKGROUND_COLOR = set_color_opacity(cls.PLOT_BACKGROUND_COLOR, opacity)

        for n in range(len(cls.PROPERTY_COMPONENT_COLORS)):
            cls.PROPERTY_COMPONENT_COLORS[n] = set_color_opacity(cls.PROPERTY_COMPONENT_COLORS[n], opacity)

        cls.active_style = {
            "Rectangle::hover_highlight": {"background_color": 0x0, "border_radius": cls.FIELD_BORDER_RADIUS, "margin": 0, "padding": 0},
            "Rectangle::hover_highlight:hovered": {"background_color": 0x59FFFFFF},
            "Rectangle::property_value": {"background_color": cls.PROPERTY_VALUE_BACKGROUND_COLOR, "border_color": 0x0,"border_width": 0,"border_radius": cls.FIELD_BORDER_RADIUS},
            "Rectangle::plot": {"background_color": cls.PLOT_BACKGROUND_COLOR},
            "Rectangle::plot_overlay": {"background_color": cl("#00000000")},
            "Frame::section": {"background_color": cls.FRAME_COLOR},
            "CollapsableFrame::section": {"background_color": cls.FRAME_COLOR},
            "Label::property_name": {"margin": 2},
            "Label::property_value": {"margin": 2, "color": cl("#868E8F")},
            "Tooltip": {"background_color": 0xFF222222, "color": 0xFFDDDDDD},
            }

    class DataProperties():
        def __init__(self, window, name : str, components : int = 3, component_names : Sequence[str] | None = None, component_colors : Sequence[int] | None = None, unit_name : str = ""):
            self._window : SimulationDataVisualizerWindow = window
            self._values = [None] * components
            self._ui_values = [None] * components
            self._name : str = name
            self._ui_label_name = None
            self.unit_name : str = unit_name

            if component_names is None:
                self.component_names = ("X", "Y", "Z", "W")[0:components] if components > 1 else None
            else:
                self.component_names = component_names

            if component_colors is None:
                self._component_colors = SimulationDataVisualizerWindow.PROPERTY_COMPONENT_COLORS[0:components] if components > 1 else None
            else:
                self._component_colors = component_colors

            self._components_toggle_callback : Callable = None
            self._components_enabled = [True] * components
            self._components_color_rectangles = [None] * components
            self._components_toggle_rectangles = [None] * components

        @property
        def name(self) -> str:
            return self._name

        @name.setter
        def name(self, name : str):
            self._name = name
            if self._ui_label_name is not None:
                self._ui_label_name.text = name

        @property
        def num_components(self):
            return len(self._values)

        def set_components_toggle_callback(self, cb : Callable):
            self._components_toggle_callback = cb
            for rectangle in self._components_toggle_rectangles:
                if rectangle is not None:
                    rectangle.visible = (cb is not None)
                    rectangle.enabled = (cb is not None)

        def _toggle_component(self, component, button):
            if button == 1: # Right click
                enable_all = True
                # If all others are disabled, we enable all, otherwise we disable all others.
                for n in range(self.num_components):
                    if n != component and self._components_enabled[n]:
                        enable_all = False
                        break

                for n in range(self.num_components):
                    self._components_enabled[n] = (n == component) or enable_all
                    self._components_color_rectangles[n].visible = (n == component) or enable_all
            elif button == 0: # Left click
                self._components_enabled[component] = not self._components_enabled[component]
                self._components_color_rectangles[component].visible = self._components_enabled[component]

            if self._components_toggle_callback:
                self._components_toggle_callback()

        def build_ui(self):
            with ui.HStack():
                BORDER_RADIUS = SimulationDataVisualizerWindow.FIELD_BORDER_RADIUS
                self._ui_label_name = ui.Label(self._name, width=SimulationDataVisualizerWindow.PROPERTY_NAME_WIDTH, name="property_name")
                ui.Spacer()
                if self.num_components == 1:
                    with ui.ZStack(width=SimulationDataVisualizerWindow.PROPERTY_VALUE_WIDTH):
                        ui.Rectangle(name="property_value")
                        with ui.HStack():
                            ui.Spacer(width=BORDER_RADIUS)
                            self._ui_values[0] = ui.Label("-", elided_text=True, alignment=ui.Alignment.RIGHT, name="property_value")
                            ui.Spacer(width=BORDER_RADIUS)
                else:
                    RECT_WIDTH = 12
                    with ui.HStack(width=SimulationDataVisualizerWindow.PROPERTY_VALUE_WIDTH, spacing=BORDER_RADIUS + 1):
                        for index in range(self.num_components):
                            with ui.ZStack():
                                ui.Rectangle(name="property_value")
                                with ui.ZStack(width=RECT_WIDTH + BORDER_RADIUS * 2):
                                    component_color_style = SimulationDataVisualizerWindow.active_style.get("Rectangle::property_value").copy()
                                    component_color_style.update({"background_color": self._component_colors[index]})
                                    self._components_color_rectangles[index] = ui.Rectangle(style=component_color_style)
                                    ui.Label(self.component_names[index][0], alignment=ui.Alignment.CENTER,
                                             width=RECT_WIDTH + BORDER_RADIUS * 2, name="property_name")
                                    self._components_toggle_rectangles[index] = ui.Rectangle(visible=(self._components_toggle_callback is not None),
                                                                                             enabled=(self._components_toggle_callback is not None),
                                                                                             width=RECT_WIDTH + BORDER_RADIUS * 2,
                                                                                             tooltip="Left click: toggle this component\nRight click: enable this and toggle all others.",
                                                                                             name="hover_highlight", alignment=ui.Alignment.LEFT_TOP,
                                                                                             mouse_pressed_fn= lambda x, y, b, m, component=index: self._toggle_component(component, b))
                                with ui.HStack():
                                    ui.Spacer(width=RECT_WIDTH + BORDER_RADIUS)
                                    with ui.ZStack():
                                        block_style = SimulationDataVisualizerWindow.active_style.get("Rectangle::property_value").copy()
                                        block_style.update({"border_radius": 0, "border_width": BORDER_RADIUS})
                                        ui.Rectangle(style=block_style)
                                        self._ui_values[index] = ui.Label("-", elided_text=True, alignment=ui.Alignment.RIGHT, name="property_value")
                                    ui.Spacer(width=BORDER_RADIUS)

        def set_value(self, values : tuple):
            if values is None:
                values = [None] * len(self._values)

            for index in range(len(values)):
                if self._ui_values[index] is not None:
                    if self._values[index] != values[index]:
                        if values[index] is None:
                            self._ui_values[index].text = "-"
                        else:
                            self._ui_values[index].text = (float_to_string(values[index], min(12, int(32 / len(self._values)))) if isinstance(values[index], float) else str(values[index])) + self.unit_name

            self._values = values

        @property
        def value(self) -> tuple:
            return self._values

    class DataProperty(DataProperties):
        def __init__(self, window, name : str, unit_name : str = ""):
            super().__init__(window, name, 1, unit_name = unit_name)

        def set_value(self, value):
            super().set_value((value,))

        @property
        def value(self):
            return super().value[0]

    class DataPropertyEulersFromQuaternion(DataProperties):
        def __init__(self, window, name : str, rotation_order : tuple[int, int, int] = (0, 1, 2)):
            self.rotation_order = rotation_order
            super().__init__(window, name, 3, unit_name = "°")

        @property
        def rotation_order(self) -> tuple[int, int, int]:
            return self._rotation_order

        @rotation_order.setter
        def rotation_order(self, rotation_order : tuple[int, int, int] = (0, 1, 2)):
            self._rotation_order = rotation_order
            axes = (UsdRtGf.Vec3d.XAxis(), UsdRtGf.Vec3d.YAxis(), UsdRtGf.Vec3d.ZAxis())
            self._rotation_axes = (axes[rotation_order[0]], axes[rotation_order[1]], axes[rotation_order[2]])

        @property
        def value(self) -> tuple[float, float, float]:
            return super().value

        def set_value(self, values : UsdRtGf.Quatd):
            rotation = UsdRtGf.Rotation(values)
            decomposed = (*rotation.Decompose(*self._rotation_axes),)
            values = (*decomposed,)
            super().set_value(values)

    class DataPropertyAngleFromQuaternion(DataProperties):
        def __init__(self, window, name : str, axis : int = 0):
            self.axis = axis
            super().__init__(window, name, 1, unit_name = "°")

        @property
        def axis(self):
            return self._axis

        @axis.setter
        def axis(self, axis : int):
            self._axis = axis
            axes = (UsdRtGf.Vec3d.XAxis(), UsdRtGf.Vec3d.YAxis(), UsdRtGf.Vec3d.ZAxis())
            self._rotation_axes = (axes[self._axis], axes[(self._axis + 1) % 3], axes[(self._axis + 2) % 3])

        @property
        def value(self):
            return super().value[0]

        def set_value(self, values : UsdRtGf.Quatd):
            rotation = UsdRtGf.Rotation(values)
            decomposed = (*rotation.Decompose(*self._rotation_axes),)
            value = (*decomposed,)[0:1]
            super().set_value(value)

    class DataPlot():
        PLOT_STYLE_MARGIN = 2
        PLOT_STYLE_HEIGHT = 100

        def __init__(self, data_property, data_trace : PhysicsDataTrace | None = None):
            self._window : SimulationDataVisualizerWindow = data_property._window
            self._ui_label_min = None
            self._ui_label_max = None
            self._ui_plots = []
            self._ui_baseline = None
            self._ui_plot_overlay = None

            self._ui_plot_pre_sim_rect = None

            self._ui_plot_inspector_hstack = None
            self._ui_plot_inspector_line = None
            self._ui_plot_inspector_x = None
            self.data_property : SimulationDataVisualizerWindow.DataProperties = data_property
            self.data_property.set_components_toggle_callback(self._component_filter_cb)

            if data_trace is not None:
                if data_trace.num_components != self.data_property.num_components:
                    carb.log_error(f"Failed to create plot for property {data_property}")
                self._data_trace = data_trace
            else:
                if isinstance(data_property, SimulationDataVisualizerWindow.DataProperty):
                    self._data_trace = PhysicsDataTraceSingle()
                elif isinstance(data_property, SimulationDataVisualizerWindow.DataPropertyEulersFromQuaternion):
                    self._data_trace = PhysicsDataTraceEulersFromQuaternion(data_property.rotation_order)
                elif isinstance(data_property, SimulationDataVisualizerWindow.DataPropertyAngleFromQuaternion):
                    self._data_trace = PhysicsDataTraceAngleFromQuaternion(data_property.axis)
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
            nm.post_notification("Copied plot CSV to clipboard.")

        def _set_inspection_x(self, x):
            self._ui_plot_inspector_x = x
            if x is not None:
                local_offset = (x - self._ui_plot_overlay.screen_position_x - self.PLOT_STYLE_MARGIN)
                width = self._ui_plot_overlay.computed_width - self.PLOT_STYLE_MARGIN * 2

                # -1 since offset is zero-based.
                relative_offset = max(min(local_offset / (width - 1), 1.0), 0.0)
                tooltip = self.data_property.name + "\n"

                data_trace = self._data_trace
                current_time = self._window._simulation_time_current
                # The simulation time reduced to the most recent sample. This yields the time at the rightmost part of the plot.
                sampled_time = math.floor(current_time / data_trace._sample_period) * data_trace._sample_period
                # Determine the time at x by using the offset.
                inspected_time = max(0.0, sampled_time - (1.0 - relative_offset) * data_trace._sample_period * data_trace._samples_num)
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
                    data_length = len(trace_time)
                    while data_index < data_length and trace_time[data_index] < inspected_time:
                        data_index += 1
                    data_index -= 1

                inspected_data = data[data_index]

                # Weight the output between this index and the next.
                if data_index + 1 < len(data):
                    weight = (inspected_time - trace_time[data_index]) / (trace_time[data_index + 1] - trace_time[data_index])
                    inspected_data = [*inspected_data]
                    for component in range(len(inspected_data)):
                        inspected_data[component] = inspected_data[component] * (1.0 - weight) + data[data_index + 1][component] * weight

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
                self._ui_plot_inspector_hstack.spacing = max(0, min(local_offset, width -1))
            else:
                self._ui_plot_inspector_line.visible = False
                self._ui_plot_overlay.tooltip = "Left click: inspect value at point.\nRight click: copy CSV to clipboard."

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

        def build_ui(self, colors = None):
            self.data_property.build_ui()
            with ui.HStack(height=100 + 2 * self.PLOT_STYLE_MARGIN):
                with ui.ZStack():
                    if colors is None:
                        colors = self.data_property._component_colors
                        if colors is None:
                            if self._data_trace.num_components == 1:
                                colors = SimulationDataVisualizerWindow.PROPERTY_COMPONENT_COLORS[1:2]
                            else:
                                colors = SimulationDataVisualizerWindow.PROPERTY_COMPONENT_COLORS[0:self._data_trace.num_components]

                    ui.Rectangle(name="plot")

                    data_plots = []
                    for value in range(self._data_trace.num_components):
                        data_plots.append(ui.Plot(
                            ui.Type.LINE,
                            -1,
                            1,
                            *self._data_trace.get_sample(value),
                            width=ui.Percent(100),
                            height=self.PLOT_STYLE_HEIGHT,
                            style={"color": colors[value], "background_color": 0x00000000, "margin": self.PLOT_STYLE_MARGIN, "padding": 0}
                            ))
                    self._ui_plots = data_plots

                    self._ui_baseline = ui.Plot(
                            ui.Type.LINE,
                            0,
                            1,
                            0.0, 0.0,
                            width=ui.Percent(100),
                            height=self.PLOT_STYLE_HEIGHT,
                            style={"color": 0x44ffffff, "background_color": 0x00000000, "margin": self.PLOT_STYLE_MARGIN, "padding": 0}
                            )

                    self._ui_plot_pre_sim_rect = ui.Rectangle(style= {"background_color": 0x06ffffff})

                    self._ui_label_max = ui.Label("1.0", style={"font_size": 12.0, "margin": 3.0}, width=0, alignment=ui.Alignment.LEFT_TOP)
                    self._ui_label_min = ui.Label("-1.0", style={"font_size": 12.0, "margin": 3.0}, width=0, alignment=ui.Alignment.LEFT_BOTTOM)
                    # This is to block the default plot tooltips.
                    with ui.ZStack(content_clipping=True):
                        self._ui_plot_overlay = ui.Rectangle(mouse_pressed_fn=self._mouse_pressed, name="plot_overlay", mouse_released_fn=self._mouse_released, mouse_moved_fn=self._mouse_moved, mouse_hovered_fn=self._mouse_hovered, width=ui.Percent(100), height=100)
                        self._ui_plot_inspector_hstack = ui.HStack()
                        with self._ui_plot_inspector_hstack:
                            ui.Spacer(width=self.PLOT_STYLE_MARGIN)
                            self._ui_plot_inspector_line = ui.Line(visible=False, width=1, height=self.PLOT_STYLE_HEIGHT - 2 * self.PLOT_STYLE_MARGIN, alignment=ui.Alignment.H_CENTER, style={"color": 0x44ffffff})
                    self._set_inspection_x(None)

                    self.refresh_plot()

        def _refresh_limits(self):
            if (isinstance(self.data_property, SimulationDataVisualizerWindow.DataPropertyEulersFromQuaternion)
                or isinstance(self.data_property, SimulationDataVisualizerWindow.DataPropertyAngleFromQuaternion)):
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
                    self._limit_min_refresh_timer[data_trace_component] = len(component_data) + int(len(component_data) / 5 + 0.5)
                else:
                    # Otherwise, reduce the counter by one.
                    self._limit_min_refresh_timer[data_trace_component] -= 1
                    if self._limit_min_refresh_timer[data_trace_component] <= 0:
                        # Reevaluate the limit by cycling all current data for the component.
                        self._limit_min[data_trace_component] = math.inf
                        for index in range(len(component_data)):
                            if(self._limit_min[data_trace_component] >= component_data[index]):
                                self._limit_min[data_trace_component] = component_data[index]
                                self._limit_min_refresh_timer[data_trace_component] = index + int(len(component_data) / 5 + 0.5)

                # Same process for max limit.
                if component_data[-1] >= self._limit_max[data_trace_component]:
                    self._limit_max_refresh_timer[data_trace_component] = len(component_data) + int(len(component_data) / 5 + 0.5)
                    self._limit_max[data_trace_component] = component_data[-1]
                else:
                    self._limit_max_refresh_timer[data_trace_component] -= 1
                    if self._limit_max_refresh_timer[data_trace_component] <= 0:
                        self._limit_max[data_trace_component] = -math.inf
                        for index in range(len(component_data)):
                            if(self._limit_max[data_trace_component] <= component_data[index]):
                                self._limit_max[data_trace_component] = component_data[index]
                                self._limit_max_refresh_timer[data_trace_component] = index + int(len(component_data) / 5 + 0.5)

                # Only factor it into the visual output if the component plot is actually visible.
                if data_trace_component >= self.data_property.num_components or self.data_property._components_enabled[data_trace_component]:
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

            if isinstance(self.data_property, SimulationDataVisualizerWindow.DataPropertyEulersFromQuaternion):
                self._data_trace.rotation_order = self.data_property.rotation_order
            elif isinstance(self.data_property, SimulationDataVisualizerWindow.DataPropertyAngleFromQuaternion):
                self._data_trace.axis = self.data_property.axis

            self.refresh_plot()

        def refresh_plot(self):
            if self._ui_label_min is None:
                return

            if isinstance(self.data_property, SimulationDataVisualizerWindow.DataPropertyEulersFromQuaternion):
                for axis in range(3):
                    if self._data_trace.rotation_order[axis] != self.data_property.rotation_order[axis]:
                        self.reset_data()
                        return
            elif isinstance(self.data_property, SimulationDataVisualizerWindow.DataPropertyAngleFromQuaternion):
                if self._data_trace.axis != self.data_property.axis:
                    self.reset_data()
                    return

            data_min, data_max = self._refresh_limits()

            self._ui_label_min.text = float_to_string(data_min, precision=4) + self.data_property.unit_name
            self._ui_label_max.text = float_to_string(data_max, precision=4) + self.data_property.unit_name

            for n in range(self._data_trace.num_components):
                self._ui_plots[n].scale_min = data_min
                self._ui_plots[n].scale_max = data_max
                self._ui_plots[n].set_data(*self._data_trace.get_sample(n))
                if n < self.data_property.num_components:
                    self._ui_plots[n].visible = self.data_property._components_enabled[n]

            self._ui_baseline.scale_min = data_min
            self._ui_baseline.scale_max = data_max

            pre_sim_width = 1.0 - self._window._simulation_time_current / (self._data_trace._samples_num * self._data_trace._sample_period)

            if pre_sim_width < 0.0:
                self._ui_plot_pre_sim_rect.visible = False
            else:
                self._ui_plot_pre_sim_rect.visible = True
                self._ui_plot_pre_sim_rect.width = ui.Length(100.0 * pre_sim_width,  ui.UnitType.PERCENT)

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

    class Monitor():
        def __init__(self, window):
            self._window : SimulationDataVisualizerWindow = window

        @property
        def stage(self):
            return self._window._stage

        @property
        def prim(self):
            return self._window._prim

        @property
        def prim_path(self):
            return self._window._prim_path

        @property
        def usdrt_stage(self):
            return self._window._usdrt_stage

        @property
        def usdrt_prim(self):
            return self._window._usdrt_prim

        def refresh_simulation_data(self, delta_time = 0.0):
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
                BUTTON_STYLE = {"background_color": 0x0, "padding": 0, "margin" : 0, "color": 0xFFFFFFFF,  "Tooltip": {"background_color": 0xFF222222, "color": 0xFFDDDDDD},}
                BUTTON_STYLE_ACTIVATED = {"background_color": cl("#1F2123"), "padding": 0, "margin" : 0, "color": 0xFFFFFFFF, "Tooltip": {"background_color": 0xFF222222, "color": 0xFFDDDDDD},}

                def __init__(self, window, name):
                    super().__init__(window, name)
                    icons_folder = Path(__file__).parents[3].joinpath("icons", "simDataVis")
                    self._icon_unlocked = str(icons_folder.joinpath("window_unlocked.svg"))
                    self._icon_locked = str(icons_folder.joinpath("window_locked.svg"))
                    self._icon_new_window = str(icons_folder.joinpath("window_new.svg"))
                    self._window_lock_button = None
                    self._window_new_button = None

                def _refresh_locked_state(self):
                    if window._prim_locked:
                        self._window_lock_button.style = self.BUTTON_STYLE_ACTIVATED
                        self._window_lock_button.image_url = self._icon_locked
                    else:
                        self._window_lock_button.style = self.BUTTON_STYLE
                        self._window_lock_button.image_url = self._icon_unlocked

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

                        with ui.ZStack(width=21):
                            self._window_lock_button = ui.ToolButton(None, clicked_fn=self.toggle_locked, image_url=self._icon_unlocked, fill_policy=ui.FillPolicy.PRESERVE_ASPECT_CROP, image_width=20, image_height=20, style=self.BUTTON_STYLE)
                            self._refresh_locked_state()
                            ui.Rectangle(height=21, width=21, tooltip = "Lock window to this prim", name="hover_highlight", alignment=ui.Alignment.LEFT_TOP)

                        ui.Spacer(width=2)

                        with ui.ZStack(width=21):
                            self._window_new_button = ui.ToolButton(None, clicked_fn=self.new_window, image_url=self._icon_new_window, fill_policy=ui.FillPolicy.PRESERVE_ASPECT_CROP, image_width=20, image_height=20, style=self.BUTTON_STYLE)
                            ui.Rectangle(height=21, width=21, tooltip = "Create new window (locks the current)", name="hover_highlight", alignment=ui.Alignment.LEFT_TOP)

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

        @classmethod
        def get_is_prim_valid_target(cls, prim : Usd.Prim) -> bool:
            return (prim is not None)

        def build_collapsable_section(self, name, build_func):
            def on_collapsed_changed(collapsed):
                if not collapsed:
                    SimulationDataVisualizerWindow.SharedStates.sections_expanded.add(name)
                else:
                    SimulationDataVisualizerWindow.SharedStates.sections_expanded.discard(name)

            with ui.CollapsableFrame(name, height=0, name="section", collapsed=(name not in SimulationDataVisualizerWindow.SharedStates.sections_expanded), collapsed_changed_fn=on_collapsed_changed):
                with ui.HStack():
                    with ui.VStack(spacing=SimulationDataVisualizerWindow.SECTION_SPACING):
                        build_func()


    """
    class MonitorTemplate(MonitorSubsection):
        def __init__(self, window):
            super().__init__(window)

        def refresh_simulation_data(self, delta_time = 0.0):
            super().refresh_simulation_data(delta_time)

        def refresh_data(self):
            super().refresh_data()

        def build_ui(self):
            super().build_ui()

        @classmethod
        def get_is_prim_valid_target(cls, prim : Usd.Prim) -> bool:
            return super().get_is_prim_valid_target(prim)
    """


    class MonitorXformable(MonitorSubsection):
        SETTING_FABRIC_ENABLED = "physics/fabricEnabled"
        SETTING_USE_FABRIC_SCENE_DELEGATE = "/app/useFabricSceneDelegate"

        def __init__(self, window):
            super().__init__(window)

            world_position_prop = SimulationDataVisualizerWindow.DataProperties(window, "Position", 3)
            self._plot_world_position = SimulationDataVisualizerWindow.DataPlot(world_position_prop)

            world_orientation_prop = SimulationDataVisualizerWindow.DataPropertyEulersFromQuaternion(window, "Orientation")
            self._plot_world_orientation = SimulationDataVisualizerWindow.DataPlot(world_orientation_prop)

            self._xformable = None
            self._usdrt_xformable = None
            self._usdrt_xformable_attr_fabric_hierarchy_world_matrix = None

            self._current_prim = None

            self._use_fabric_setting = get_settings().subscribe_to_node_change_events(
                __class__.SETTING_FABRIC_ENABLED, self._on_fabric_setting_changed
            )

        def _on_fabric_setting_changed(self, item, event_type):
            self.refresh_data()

        def refresh_simulation_data(self, delta_time = 0.0):
            super().refresh_simulation_data(delta_time)

            position = None
            quaternion = None

            # Xform for USDRT requires special handling and attributes may not be set until after simulation has started.
            if self._usdrt_xformable is not None:
                usdrt_xformable_attr_fabric_hierarchy_world_matrix = self._usdrt_xformable_attr_fabric_hierarchy_world_matrix
                if usdrt_xformable_attr_fabric_hierarchy_world_matrix is None:
                    usdrt_xformable_attr_fabric_hierarchy_world_matrix = self._usdrt_xformable.GetFabricHierarchyWorldMatrixAttr()
                    if usdrt_xformable_attr_fabric_hierarchy_world_matrix.IsValid():
                        self._usdrt_xformable_attr_fabric_hierarchy_world_matrix = usdrt_xformable_attr_fabric_hierarchy_world_matrix

                if usdrt_xformable_attr_fabric_hierarchy_world_matrix is not None:
                    xform_world = usdrt_xformable_attr_fabric_hierarchy_world_matrix.Get()

                    position = xform_world.ExtractTranslation()
                    quaternion = xform_world.GetOrthonormalized().ExtractRotationQuat()
                    if quaternion.GetReal() < 0.0:
                        quaternion = -quaternion

            if (position is None or quaternion is None):
                if self._xformable is not None:
                    xform_world = matrix4_to_usdrt(self._xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
                    position = xform_world.ExtractTranslation()
                    quaternion = xform_world.GetOrthonormalized().ExtractRotationQuat()
                    if quaternion.GetReal() < 0.0:
                        quaternion = -quaternion
                else:
                    return

            self._plot_world_position.append_data(delta_time, position)
            self._plot_world_orientation.append_data(delta_time, quaternion)

        def reset_plots(self):
            self._plot_world_position.reset_data()
            self._plot_world_orientation.reset_data()

        def refresh_data(self):
            if self._current_prim != self.prim:
                self._current_prim = self.prim
                self.reset_plots()
                self._xformable = UsdGeom.Xformable(self.prim) if self.prim is not None else None
                self._usdrt_xformable = None
                self._usdrt_xformable_attr_fabric_hierarchy_world_matrix = None


            if not (self.usdrt_prim is not None and
                get_settings().get_as_bool(__class__.SETTING_FABRIC_ENABLED) and
                get_settings().get_as_bool(__class__.SETTING_USE_FABRIC_SCENE_DELEGATE)):

                self._usdrt_xformable = None
                self._usdrt_xformable_attr_fabric_hierarchy_world_matrix = None
            else:
                if self._usdrt_xformable is None:
                    self._usdrt_xformable = UsdRtGeom.Xformable(self.usdrt_prim)
                    if self._usdrt_xformable is None:
                        carb.log_error(f"Failed to retrieve USDRT Xformable for prim {self.usdrt_prim}.")
                    # This attribute is only valid after simulation has been started so here we just clear it.
                    self._usdrt_xformable_attr_fabric_hierarchy_world_matrix = None

            super().refresh_data()

        def _build_world_pose_section(self):
            self._plot_world_position.build_ui()
            self._plot_world_orientation.build_ui()

        def build_ui(self):
            super().build_ui()
            self.build_collapsable_section("World Pose", self._build_world_pose_section)

        @classmethod
        def get_is_prim_valid_target(cls, prim : Usd.Prim) -> bool:
            # return super(__class__, __class__).get_is_prim_valid_target(prim) and prim.IsA(UsdGeom.Xformable)
            return super().get_is_prim_valid_target(prim) and prim.IsA(UsdGeom.Xformable)


    class MonitorRigidBody(MonitorSubsection):
        def __init__(self, window):
            super().__init__(window)

            self._rigid_body = None
            self._rigid_body_attr_linear_velocity = None
            self._rigid_body_attr_angular_velocity = None
            self._usdrt_rigid_body = None
            self._usdrt_rigid_body_attr_linear_velocity = None
            self._usdrt_rigid_body_attr_angular_velocity = None

            self._rigid_body_property_query_manager = None
            self._refresh_data_on_query_complete = False

            self._linear_velocity_current = None
            linear_velocity_prop = SimulationDataVisualizerWindow.DataProperties(window, "Velocity", 4, ("X", "Y", "Z", "Magnitude"))
            self._plot_linear_velocity = SimulationDataVisualizerWindow.DataPlot(linear_velocity_prop)
            linear_acceleration_prop = SimulationDataVisualizerWindow.DataProperties(window, "Acceleration", 4, ("X", "Y", "Z", "Magnitude"))
            self._plot_linear_acceleration = SimulationDataVisualizerWindow.DataPlot(linear_acceleration_prop)

            self._angular_velocity_current = None
            angular_velocity_prop = SimulationDataVisualizerWindow.DataProperties(window, "Velocity", 4, ("X", "Y", "Z", "Magnitude"))
            self._plot_angular_velocity = SimulationDataVisualizerWindow.DataPlot(angular_velocity_prop)
            angular_acceleration_prop = SimulationDataVisualizerWindow.DataProperties(window, "Acceleration", 4, ("X", "Y", "Z", "Magnitude"))
            self._plot_angular_acceleration = SimulationDataVisualizerWindow.DataPlot(angular_acceleration_prop)

            self._total_mass_prop = SimulationDataVisualizerWindow.DataProperty(window, "Total mass")
            self._center_of_mass_prop = SimulationDataVisualizerWindow.DataProperties(window, "Center of mass", 3)
            self._principal_axes_prop = SimulationDataVisualizerWindow.DataProperties(window, "Principal Axes", 3)
            self._diagonal_inertia_prop = SimulationDataVisualizerWindow.DataProperties(window, "Diagonal inertia", 3)

            self._current_prim = None

        def refresh_simulation_data(self, delta_time = 0.0):
            super().refresh_simulation_data(delta_time)

            linear_velocity = None
            angular_velocity = None

            if self._usdrt_rigid_body_attr_linear_velocity is not None and self._usdrt_rigid_body_attr_linear_velocity.IsValid():
                linear_velocity = self._usdrt_rigid_body_attr_linear_velocity.Get()
            elif self._rigid_body_attr_linear_velocity is not None:
                linear_velocity = self._rigid_body_attr_linear_velocity.Get()

            if self._usdrt_rigid_body_attr_angular_velocity is not None and self._usdrt_rigid_body_attr_angular_velocity.IsValid():
                angular_velocity = self._usdrt_rigid_body_attr_angular_velocity.Get()
            elif self._rigid_body_attr_angular_velocity is not None:
                angular_velocity = self._rigid_body_attr_angular_velocity.Get()

            if linear_velocity is not None:
                self._plot_linear_velocity.append_data(delta_time, (*linear_velocity, linear_velocity.GetLength()))
                if delta_time > 0.0:
                    linear_velocity_delta = copy.deepcopy(linear_velocity)
                    if self._linear_velocity_current is not None:
                        for n in range(len(linear_velocity)):
                            linear_velocity_delta[n] -= self._linear_velocity_current[n]

                    linear_velocity_delta *= 1.0 / delta_time
                    self._plot_linear_acceleration.append_data(delta_time, (*linear_velocity_delta, linear_velocity_delta.GetLength()))
                    self._linear_velocity_current = linear_velocity

            if angular_velocity is not None:
                self._plot_angular_velocity.append_data(delta_time, (*angular_velocity, angular_velocity.GetLength()))
                if delta_time > 0.0:
                    angular_velocity_delta = copy.deepcopy(angular_velocity)
                    if self._angular_velocity_current is not None:
                        for n in range(len(linear_velocity)):
                            angular_velocity_delta[n] -= self._angular_velocity_current[n]

                    angular_velocity_delta *= 1.0 / delta_time
                    self._plot_angular_acceleration.append_data(delta_time, (*angular_velocity_delta, angular_velocity_delta.GetLength()))
                    self._angular_velocity_current = angular_velocity

        def reset_plots(self):
            self._plot_linear_velocity.reset_data()
            self._plot_angular_velocity.reset_data()
            self._plot_linear_acceleration.reset_data()
            self._plot_angular_acceleration.reset_data()

        def refresh_data(self):
            if self._current_prim != self.prim:
                self._current_prim = self.prim
                self._linear_velocity_current = None
                self._angular_velocity_current = None
                self.reset_plots()
                self._usdrt_rigid_body = None
                self._usdrt_rigid_body_attr_linear_velocity = None
                self._usdrt_rigid_body_attr_angular_velocity = None
                self._refresh_data_on_query_complete = False
                self._rigid_body_property_query_manager = None

            if self.prim is None:
                self._rigid_body = None
                self._rigid_body_attr_linear_velocity = None
                self._rigid_body_attr_angular_velocity = None
                self._usdrt_rigid_body = None
                self._usdrt_rigid_body_attr_linear_velocity = None
                self._usdrt_rigid_body_attr_angular_velocity = None
                super().refresh_data()
                return

            self._rigid_body = UsdPhysics.RigidBodyAPI(self.prim)

            self._rigid_body_attr_linear_velocity = self._rigid_body.GetVelocityAttr()
            if self._rigid_body_attr_linear_velocity is None:
                carb.log_error("Failed to retrieve linear velocity attribute.")

            self._rigid_body_attr_angular_velocity = self._rigid_body.GetAngularVelocityAttr()
            if self._rigid_body_attr_linear_velocity is None:
                carb.log_error("Failed to retrieve angular velocity attribute.")

            if self.usdrt_prim is None:
                self._usdrt_rigid_body = None
                self._usdrt_rigid_body_attr_linear_velocity = None
                self._usdrt_rigid_body_attr_angular_velocity = None
            else:
                if self._usdrt_rigid_body is None:
                    self._usdrt_rigid_body = UsdRtPhysics.RigidBodyAPI(self.usdrt_prim)
                    if self._usdrt_rigid_body is None:
                        carb.log_error("Failed to retrieve USDRT rigid body API.")

                if self._usdrt_rigid_body is not None:
                    if self._usdrt_rigid_body_attr_linear_velocity is None:
                        self._usdrt_rigid_body_attr_linear_velocity = self._usdrt_rigid_body.GetVelocityAttr()
                    if self._usdrt_rigid_body_attr_linear_velocity is None:
                        carb.log_error("Failed to retrieve USDRT rigid body linear velocity attribute.")

                    if self._usdrt_rigid_body_attr_angular_velocity is None:
                        self._usdrt_rigid_body_attr_angular_velocity = self._usdrt_rigid_body.GetAngularVelocityAttr()
                    if self._usdrt_rigid_body_attr_angular_velocity is None:
                        carb.log_error("Failed to retrieve USDRT rigid body angular velocity attribute.")

            class QueryManagerObjectInfo(propertyQueryRigidBody.QueryManager):
                def __init__(self, parent):
                    super().__init__()
                    self._parent = parent

                def on_query_finished(self):
                    if(self.get_query_status() == propertyQueryRigidBody.Query.Status.COMPLETE and
                        self._parent._refresh_data_on_query_complete):
                        self._parent.refresh_data()

            if self._rigid_body_property_query_manager is None:
                self._rigid_body_property_query_manager = QueryManagerObjectInfo(self)

            if self._rigid_body_property_query_manager.get_query_status() == propertyQueryRigidBody.Query.Status.UNSET:
                # Fetch mass properties from PhysX.
                self._rigid_body_property_query_manager.submit_query(self.prim)

            if self._rigid_body_property_query_manager.get_query_status() == propertyQueryRigidBody.Query.Status.IN_PROGRESS:
                self._total_mass_prop.set_value(None)
                self._center_of_mass_prop.set_value(None)
                self._principal_axes_prop.set_value(None)
                self._diagonal_inertia_prop.set_value(None)
                self._refresh_data_on_query_complete = True
            else:
                self._refresh_data_on_query_complete = False
                results = self._rigid_body_property_query_manager.get_query_result()

                world_rotation_eulers = [*Gf.Rotation(results.principal_axes).Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())]

                self._total_mass_prop.set_value(results.mass)
                self._center_of_mass_prop.set_value(results.center_of_mass)
                self._principal_axes_prop.set_value(world_rotation_eulers)
                self._diagonal_inertia_prop.set_value(results.diagonal_inertia)

            super().refresh_data()

        def _build_linear_velocity_section(self):
            self._plot_linear_velocity.build_ui()
            self._plot_linear_acceleration.build_ui()

        def _build_angular_velocity_section(self):
            self._plot_angular_velocity.build_ui()
            self._plot_angular_acceleration.build_ui()

        def _build_mass_section(self):
            self._total_mass_prop.build_ui()
            self._center_of_mass_prop.build_ui()
            self._principal_axes_prop.build_ui()
            self._diagonal_inertia_prop.build_ui()

        def build_ui(self):
            super().build_ui()
            self.build_collapsable_section("Linear Velocity", self._build_linear_velocity_section)
            self.build_collapsable_section("Angular Velocity", self._build_angular_velocity_section)
            self.build_collapsable_section("Mass/inertia", self._build_mass_section)

        @classmethod
        def get_is_prim_valid_target(cls, prim : Usd.Prim) -> bool:
            return super().get_is_prim_valid_target(prim) and prim.HasAPI(UsdPhysics.RigidBodyAPI)


    class MonitorJoint(MonitorSubsection):

        class Attachment():
            SETTING_USE_FABRIC_SCENE_DELEGATE = "/app/useFabricSceneDelegate"
            SETTING_FABRIC_ENABLED = "physics/fabricEnabled"

            def __init__(self, monitor, index):
                self._index = index
                self._monitor : SimulationDataVisualizerWindow.MonitorJoint = monitor
                self.prim = None
                self.prim_path : Sdf.Path = None
                self.xformable = None
                self.rigid_body = None

                self.usdrt_prim = None
                self.usdrt_xformable = None
                self.usdrt_xformable_attr_fabric_hierarchy_world_matrix = None
                self.usdrt_rigid_body = None

            def set_prim_path(self, prim_path : Sdf.Path):
                if self.prim_path == prim_path:
                    return

                self.xformable = None
                self.usdrt_prim = None
                self.usdrt_xformable = None
                self.usdrt_xformable_attr_fabric_hierarchy_world_matrix = None

                self.rigid_body = None

                self.prim_path = prim_path
                self.prim = self._monitor.stage.GetPrimAtPath(self.prim_path) if prim_path is not None else None
                if self.prim is None:
                    if prim_path is not None:
                        carb.log_error(f"{self} failed to retrieve body {self._index} prim ({prim_path}) for {self._monitor.prim_path}")
                    return

                if not self.prim.IsA(UsdGeom.Xformable):
                    carb.log_error(f"{self} body {self._index} prim at path ({self.prim_path}) is not an Xformable.")
                    return
                self.xformable = UsdGeom.Xformable(self.prim)

                if self._monitor.usdrt_prim is not None:
                    self.usdrt_prim = self._monitor.usdrt_stage.GetPrimAtPath(str(self.prim_path))
                    if self.usdrt_prim is None:
                        carb.log_error(f"Failed to retrieve USDRT body {self._index} prim at path {self.prim_path}.")
                        return

                    self.usdrt_xformable = UsdRtGeom.Xformable(self.usdrt_prim)

                self.refresh_data()

            def refresh_data(self):
                self.rigid_body = None
                self.usdrt_xformable_attr_fabric_hierarchy_world_matrix = None

                if self.prim is None:
                    return

                if (self.usdrt_prim is not None and
                    get_settings().get_as_bool(__class__.SETTING_FABRIC_ENABLED) and
                    get_settings().get_as_bool(__class__.SETTING_USE_FABRIC_SCENE_DELEGATE)):
                    if self.usdrt_xformable is None:
                        self.usdrt_xformable = UsdRtGeom.Xformable(self.usdrt_prim)
                        if self.usdrt_xformable is None:
                            carb.log_error(f"Failed to retrieve USDRT Xformable for prim {self.usdrt_prim}.")
                        # These attributes are only valid after simulation has been started so here we just clear them.
                        self.usdrt_xformable_attr_fabric_hierarchy_world_matrix = None

            def fetch_transform(self):
                # Xform for USDRT requires special handling and attributes may not be set until after simulation has started.
                if self.usdrt_xformable is not None:
                    usdrt_xformable_attr_fabric_hierarchy_world_matrix = self.usdrt_xformable_attr_fabric_hierarchy_world_matrix
                    if usdrt_xformable_attr_fabric_hierarchy_world_matrix is None:
                        usdrt_xformable_attr_fabric_hierarchy_world_matrix = self.usdrt_xformable.GetFabricHierarchyWorldMatrixAttr()
                        if usdrt_xformable_attr_fabric_hierarchy_world_matrix.IsValid():
                            self.usdrt_xformable_attr_fabric_hierarchy_world_matrix = usdrt_xformable_attr_fabric_hierarchy_world_matrix


                    if usdrt_xformable_attr_fabric_hierarchy_world_matrix is not None:
                        return usdrt_xformable_attr_fabric_hierarchy_world_matrix.Get()

                if self.xformable is not None:
                    return matrix4_to_usdrt(self.xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
                else:
                    return UsdRtGf.Matrix4d()

        def __init__(self, window):
            super().__init__(window)

            self._current_prim = None

            self._attachments = [__class__.Attachment(self, 0), __class__.Attachment(self, 1)]

            self._joint : UsdPhysics.Joint = None
            self._usdrt_joint = None

            self._body0_rel : Usd.Relationship = None
            self._body0_offset_position_attr = None
            self._body0_offset_rotation_attr = None

            self._body1_rel : Usd.Relationship = None
            self._body1_offset_position_attr = None
            self._body1_offset_rotation_attr = None

            # self._init_joint_type()

        def _get_delta_xform(self) -> UsdRtGf.Matrix4d:
            xform0 = self._attachments[0].fetch_transform()
            # Offsets may be changed during simulation, so we have to update it here.
            offset0_quat = self._body0_offset_rotation_attr.Get()
            offset0 = UsdRtGf.Matrix4d().SetRotate(quaternion_to_usdrt(offset0_quat)) * UsdRtGf.Matrix4d().SetTranslate(vec3_to_usdrt(self._body0_offset_position_attr.Get()))
            xform0 = offset0 * xform0
            xform0 = xform0.RemoveScaleShear()

            xform1 = self._attachments[1].fetch_transform()
            offset1_quat = self._body1_offset_rotation_attr.Get()
            offset1 = UsdRtGf.Matrix4d().SetRotate(quaternion_to_usdrt(offset1_quat)) * UsdRtGf.Matrix4d().SetTranslate(vec3_to_usdrt(self._body1_offset_position_attr.Get()))
            xform1 = offset1 * xform1
            xform1 = xform1.RemoveScaleShear()

            return xform1 * xform0.GetInverse()

        def refresh_data(self):
            if self._current_prim != self.prim:
                self._current_prim = self.prim
                self.reset_plots()
                self._joint = UsdPhysics.Joint(self.prim) if self.prim is not None else None
                self._usdrt_joint = UsdRtPhysics.Joint(self.usdrt_prim) if self.usdrt_prim is not None else None
                if self._joint is None:
                    self._attachments[0].set_prim_path(None)
                    self._attachments[1].set_prim_path(None)
                    self._body0_rel = None
                    self._body0_offset_position_attr = None
                    self._body0_offset_rotation_attr = None
                    self._body1_rel = None
                    self._body1_offset_position_attr = None
                    self._body1_offset_rotation_attr = None
                else:
                    self._body0_rel = self._joint.GetBody0Rel()
                    self._body0_offset_position_attr = self._joint.GetLocalPos0Attr()
                    self._body0_offset_rotation_attr = self._joint.GetLocalRot0Attr()
                    self._body1_rel = self._joint.GetBody1Rel()
                    self._body1_offset_position_attr = self._joint.GetLocalPos1Attr()
                    self._body1_offset_rotation_attr = self._joint.GetLocalRot1Attr()

            if self._joint:
                if self._body0_rel:
                    targets = self._body0_rel.GetTargets()
                    if targets is not None and len(targets) > 0:
                        self._attachments[0].set_prim_path(targets[0])
                    else:
                        self._attachments[0].set_prim_path(None)
                else:
                    self._attachments[0].set_prim_path(None)

                if self._body1_rel:
                    targets = self._body1_rel.GetTargets()
                    if targets is not None and len(targets) > 0:
                        self._body1_prim_path = targets[0]
                        self._attachments[1].set_prim_path(targets[0])
                    else:
                        self._attachments[1].set_prim_path(None)
                else:
                    self._attachments[1].set_prim_path(None)

            super().refresh_data()

        @classmethod
        def get_is_prim_valid_target(cls, prim : Usd.Prim) -> bool:
            return False

    class MonitorD6Joint(MonitorJoint, MonitorSubsection):
        def __init__(self, window):
            self._relative_position_prop = SimulationDataVisualizerWindow.DataProperties(window, "Position", 4, ("X", "Y", "Z", "Magnitude"))
            self._plot_relative_position = SimulationDataVisualizerWindow.DataPlot(self._relative_position_prop)
            self._linear_velocity_prop = SimulationDataVisualizerWindow.DataProperties(window, "Velocity", 4, ("X", "Y", "Z", "Magnitude"))
            self._current_linear_velocity = None
            self._plot_linear_velocity = SimulationDataVisualizerWindow.DataPlot(self._linear_velocity_prop)
            self._linear_acceleration_prop = SimulationDataVisualizerWindow.DataProperties(window, "Acceleration", 4, ("X", "Y", "Z", "Magnitude"))
            self._plot_linear_acceleration = SimulationDataVisualizerWindow.DataPlot(self._linear_acceleration_prop)

            self._relative_rotation_prop = SimulationDataVisualizerWindow.DataPropertyEulersFromQuaternion(window, "Rotation")
            self._plot_relative_rotation = SimulationDataVisualizerWindow.DataPlot(self._relative_rotation_prop)
            self._angular_velocity_prop = SimulationDataVisualizerWindow.DataProperties(window, "Velocity", 4, ("X", "Y", "Z", "Magnitude"))
            self._current_rotation = None
            self._current_angular_velocity = None
            self._plot_angular_velocity = SimulationDataVisualizerWindow.DataPlot(self._angular_velocity_prop)
            self._angular_acceleration_prop = SimulationDataVisualizerWindow.DataProperties(window, "Acceleration", 4, ("X", "Y", "Z", "Magnitude"))
            self._plot_angular_acceleration = SimulationDataVisualizerWindow.DataPlot(self._angular_acceleration_prop)


            super().__init__(window)

        def refresh_simulation_data(self, delta_time = 0.0):
            delta_xform = self._get_delta_xform()

            delta_quaternion = delta_xform.GetOrthonormalized().ExtractRotationQuat()
            if delta_quaternion.GetReal() < 0.0:
                delta_quaternion = -delta_quaternion

            self._plot_relative_rotation.append_data(delta_time, delta_quaternion)
            if self._current_rotation is None:
                self._current_rotation = delta_quaternion

            if delta_time > 0.0:
                velocity = delta_quaternion * self._current_rotation.GetInverse()
                rotation = UsdRtGf.Rotation(velocity)
                decomposed = rotation.Decompose(*self._relative_rotation_prop._rotation_axes)
                velocity = decomposed * (1.0 / delta_time)
                self._current_rotation = delta_quaternion
                acceleration = (velocity - self._current_angular_velocity) * (1.0 / delta_time) if self._current_angular_velocity is not None else UsdRtGf.Vec3d()
                self._current_angular_velocity = velocity
                self._plot_angular_velocity.append_data(delta_time, (*velocity, velocity.GetLength()))
                self._plot_angular_acceleration.append_data(delta_time, (*acceleration, acceleration.GetLength()))

            translation = delta_xform.ExtractTranslation()
            translation = delta_quaternion.GetInverse().Transform(translation)
            if delta_time > 0.0:
                velocity = (translation - UsdRtGf.Vec3d(*self._relative_position_prop.value[0:3])) * (1.0 / delta_time)
                acceleration = (velocity - self._current_linear_velocity) * (1.0 / delta_time) if self._current_linear_velocity is not None else UsdRtGf.Vec3d()
                self._current_linear_velocity = velocity
                self._plot_linear_velocity.append_data(delta_time, (*velocity, velocity.GetLength()))
                self._plot_linear_acceleration.append_data(delta_time, (*acceleration, acceleration.GetLength()))

            self._plot_relative_position.append_data(delta_time, (*translation, translation.GetLength()))

            super().refresh_simulation_data(delta_time)

        def refresh_data(self):
            if self._current_prim != self.prim:
                self._current_rotation = None
                self._current_angular_velocity = None
                self._current_linear_velocity = None
            return super().refresh_data()

        def reset_plots(self):
            self._current_rotation = None
            self._current_angular_velocity = None
            self._current_linear_velocity = None
            self._plot_relative_position.reset_data()
            self._plot_linear_velocity.reset_data()
            self._plot_linear_acceleration.reset_data()
            self._plot_relative_rotation.reset_data()
            self._plot_angular_velocity.reset_data()
            self._plot_angular_acceleration.reset_data()

        def _build_local_offset_section(self):
            self._plot_relative_position.build_ui()
            self._plot_relative_rotation.build_ui()

        def _build_linear_velocity_section(self):
            self._plot_linear_velocity.build_ui()
            self._plot_linear_acceleration.build_ui()

        def _build_angular_velocity_section(self):
            self._plot_angular_velocity.build_ui()
            self._plot_angular_acceleration.build_ui()

        def build_ui(self):
            super().build_ui()
            self.build_collapsable_section("Local Offset", self._build_local_offset_section)
            self.build_collapsable_section("Linear Velocity", self._build_linear_velocity_section)
            self.build_collapsable_section("Angular Velocity", self._build_angular_velocity_section)

        @classmethod
        def get_is_prim_valid_target(cls, prim : Usd.Prim) -> bool:
            return (SimulationDataVisualizerWindow.MonitorSubsection.get_is_prim_valid_target(prim) and prim.IsA(UsdPhysics.Joint)
                    and not prim.IsA(UsdPhysics.SphericalJoint)
                    and not prim.IsA(UsdPhysics.PrismaticJoint)
                    and not prim.IsA(UsdPhysics.RevoluteJoint))

    class DataPlotSphericalJoint(DataPlot):
        def __init__(self, monitor, data_property, data_trace : PhysicsDataTrace | None = None):
            self._monitor = monitor
            super().__init__(data_property, data_trace)

        def _refresh_limits(self):
            return -180.0, 180.0
            # Disabled limiting to attribute limits for now since cone pivot axis will always be free.
            """

            if self._monitor._cone_limit0_attr is not None:
                limit0 = self._monitor._cone_limit0_attr.Get()
                if limit0 == math.inf:
                    return -180.0, 180.0
            else:
                return -180.0, 180.0

            if self._monitor._cone_limit1_attr is not None:
                limit1 = self._monitor._cone_limit1_attr.Get()
                if limit1 == math.inf:
                    return -180.0, 180.0
            else:
                return -180.0, 180.0

            limit = max(limit0, limit1)
            return -limit, limit
            """

    class MonitorSphericalJoint(MonitorJoint, MonitorSubsection):
        def __init__(self, window):
            self._relative_rotation_prop = SimulationDataVisualizerWindow.DataPropertyEulersFromQuaternion(window, "Rotation")
            self._plot_relative_rotation = SimulationDataVisualizerWindow.DataPlotSphericalJoint(self, self._relative_rotation_prop)
            self._angular_velocity_prop = SimulationDataVisualizerWindow.DataProperties(window, "Velocity", 4, ("X", "Y", "Z", "Magnitude"))
            self._current_rotation = None
            self._current_angular_velocity = None
            self._plot_angular_velocity = SimulationDataVisualizerWindow.DataPlot(self._angular_velocity_prop)
            self._angular_acceleration_prop = SimulationDataVisualizerWindow.DataProperties(window, "Acceleration", 4, ("X", "Y", "Z", "Magnitude"))
            self._plot_angular_acceleration = SimulationDataVisualizerWindow.DataPlot(self._angular_acceleration_prop)

            self._cone_limit0_attr = None
            self._cone_limit1_attr = None
            self._axis_attr = None
            self._current_axis = None
            self._current_axis_index = None
            self._spherical_joint = None
            super().__init__(window)

        def refresh_simulation_data(self, delta_time = 0.0):
            delta_xform = self._get_delta_xform()

            delta_quaternion = delta_xform.GetOrthonormalized().ExtractRotationQuat()
            if delta_quaternion.GetReal() < 0.0:
                delta_quaternion = -delta_quaternion

            self._plot_relative_rotation.append_data(delta_time, delta_quaternion)
            if self._current_rotation is None:
                self._current_rotation = delta_quaternion

            if delta_time > 0.0:
                velocity = delta_quaternion * self._current_rotation.GetInverse()
                rotation = UsdRtGf.Rotation(velocity)
                decomposed = rotation.Decompose(*self._relative_rotation_prop._rotation_axes)
                velocity = decomposed * (1.0 / delta_time)
                self._current_rotation = delta_quaternion

                acceleration = (velocity - self._current_angular_velocity) * (1.0 / delta_time) if self._current_angular_velocity is not None else UsdRtGf.Vec3d()
                self._current_angular_velocity = velocity
                self._plot_angular_velocity.append_data(delta_time, (*velocity, velocity.GetLength()))
                self._plot_angular_acceleration.append_data(delta_time, (*acceleration, acceleration.GetLength()))

            super().refresh_simulation_data(delta_time)

        def refresh_data(self):
            if self._current_prim != self.prim:
                self._spherical_joint = UsdPhysics.SphericalJoint(self.prim)
                self._axis_attr = self._spherical_joint.GetAxisAttr()
                self._cone_limit0_attr = self._spherical_joint.GetConeAngle0LimitAttr()
                self._cone_limit1_attr = self._spherical_joint.GetConeAngle1LimitAttr()
                self._current_rotation = None
                self._current_angular_velocity = None

            new_axis = self._axis_attr.Get()
            if self._current_axis is not new_axis:
                self._current_axis = new_axis
                self._current_axis_index = axis_token_to_index(self._current_axis)
                self._relative_rotation_prop.rotation_order = ((self._current_axis_index + 1) % 3, (self._current_axis_index + 2) % 3, self._current_axis_index)
                self.reset_plots()

            super().refresh_data()

        def reset_plots(self):
            self._current_rotation = None
            self._current_angular_velocity = None
            self._plot_relative_rotation.reset_data()
            self._plot_angular_velocity.reset_data()
            self._plot_angular_acceleration.reset_data()

        def _build_local_offset_section(self):
            self._plot_relative_rotation.build_ui()

        def _build_angular_velocity_section(self):
            self._plot_angular_velocity.build_ui()
            self._plot_angular_acceleration.build_ui()

        def build_ui(self):
            super().build_ui()
            self.build_collapsable_section("Local Offset", self._build_local_offset_section)
            self.build_collapsable_section("Angular Velocity", self._build_angular_velocity_section)

        @classmethod
        def get_is_prim_valid_target(cls, prim : Usd.Prim) -> bool:
            return (SimulationDataVisualizerWindow.MonitorSubsection.get_is_prim_valid_target(prim)
                    and prim.IsA(UsdPhysics.SphericalJoint))

    class DataPlotRevoluteJoint(DataPlot):
        def __init__(self, monitor, data_property, data_trace : PhysicsDataTrace | None = None):
            self._monitor = monitor
            super().__init__(data_property, data_trace)

        def _refresh_limits(self):
            if self._monitor._lower_limit_attr is not None:
                lower_limit = self._monitor._lower_limit_attr.Get()
                if lower_limit == -math.inf:
                    return -180.0, 180.0
            else:
                return -180.0, 180.0

            if self._monitor._upper_limit_attr is not None:
                upper_limit = self._monitor._upper_limit_attr.Get()
                if upper_limit == math.inf:
                    return -180.0, 180.0
            else:
                return -180.0, 180.0

            return lower_limit, upper_limit

    class MonitorRevoluteJoint(MonitorJoint, MonitorSubsection):
        def __init__(self, window):
            self._relative_rotation_prop = SimulationDataVisualizerWindow.DataPropertyAngleFromQuaternion(window, "Rotation")
            self._plot_relative_rotation = SimulationDataVisualizerWindow.DataPlotRevoluteJoint(self, self._relative_rotation_prop)
            self._angular_velocity_prop = SimulationDataVisualizerWindow.DataProperty(window, "Velocity")
            self._current_rotation = None
            self._current_angular_velocity = None
            self._plot_angular_velocity = SimulationDataVisualizerWindow.DataPlot(self._angular_velocity_prop)
            self._angular_acceleration_prop = SimulationDataVisualizerWindow.DataProperty(window, "Acceleration")
            self._plot_angular_acceleration = SimulationDataVisualizerWindow.DataPlot(self._angular_acceleration_prop)

            self._lower_limit_attr = None
            self._upper_limit_attr = None
            self._axis_attr = None
            self._current_axis = None
            self._current_axis_index = None
            self._revolute_joint = None
            super().__init__(window)

        def refresh_simulation_data(self, delta_time = 0.0):
            delta_xform = self._get_delta_xform()

            delta_quaternion = delta_xform.GetOrthonormalized().ExtractRotationQuat()
            if delta_quaternion.GetReal() < 0.0:
                delta_quaternion = -delta_quaternion

            self._plot_relative_rotation.append_data(delta_time, delta_quaternion)
            if self._current_rotation is None:
                self._current_rotation = delta_quaternion

            if delta_time > 0.0:
                velocity = delta_quaternion * self._current_rotation.GetInverse()
                rotation = UsdRtGf.Rotation(velocity)
                decomposed = rotation.Decompose(*self._relative_rotation_prop._rotation_axes)
                velocity = decomposed[0] / delta_time
                self._current_rotation = delta_quaternion
                acceleration = (velocity - self._current_angular_velocity) * (1.0 / delta_time) if self._current_angular_velocity is not None else 0.0
                self._current_angular_velocity = velocity
                self._plot_angular_velocity.append_data(delta_time, velocity)
                self._plot_angular_acceleration.append_data(delta_time, acceleration)

            super().refresh_simulation_data(delta_time)

        def refresh_data(self):
            if self._current_prim != self.prim:
                self._revolute_joint = UsdPhysics.RevoluteJoint(self.prim)
                self._axis_attr = self._revolute_joint.GetAxisAttr()
                self._lower_limit_attr = self._revolute_joint.GetLowerLimitAttr()
                self._upper_limit_attr = self._revolute_joint.GetUpperLimitAttr()
                self._current_rotation = None
                self._current_angular_velocity = None

            new_axis = self._axis_attr.Get()
            if self._current_axis is not new_axis:
                self._current_axis = new_axis
                self._current_axis_index = axis_token_to_index(self._current_axis)
                self._relative_rotation_prop.name = new_axis + "-rotation"
                self._relative_rotation_prop.axis = self._current_axis_index
                self.reset_plots()

            super().refresh_data()

        def reset_plots(self):
            self._current_rotation = None
            self._current_angular_velocity = None
            self._plot_relative_rotation.reset_data()
            self._plot_angular_velocity.reset_data()
            self._plot_angular_acceleration.reset_data()

        def _build_local_offset_section(self):
            self._plot_relative_rotation.build_ui()

        def _build_angular_velocity_section(self):
            self._plot_angular_velocity.build_ui()
            self._plot_angular_acceleration.build_ui()

        def build_ui(self):
            super().build_ui()
            self.build_collapsable_section("Local Offset", self._build_local_offset_section)
            self.build_collapsable_section("Angular Velocity", self._build_angular_velocity_section)

        @classmethod
        def get_is_prim_valid_target(cls, prim : Usd.Prim) -> bool:
            return (SimulationDataVisualizerWindow.MonitorSubsection.get_is_prim_valid_target(prim)
                    and prim.IsA(UsdPhysics.RevoluteJoint))


    class DataPlotPrismaticJoint(DataPlot):
        def __init__(self, monitor, data_property, data_trace : PhysicsDataTrace | None = None):
            self._monitor = monitor
            super().__init__(data_property, data_trace)

        def _refresh_limits(self):
            if self._monitor._lower_limit_attr is not None:
                lower_limit = self._monitor._lower_limit_attr.Get()
            else:
                lower_limit = -math.inf

            if self._monitor._upper_limit_attr is not None:
                upper_limit = self._monitor._upper_limit_attr.Get()
            else:
                upper_limit = math.inf

            if lower_limit == -math.inf or upper_limit == math.inf:
                limits = (super()._refresh_limits())
                if lower_limit == -math.inf:
                    lower_limit = limits[0]
                if upper_limit == -math.inf:
                    upper_limit = limits[1]

            return lower_limit, upper_limit

    class MonitorPrismaticJoint(MonitorJoint, MonitorSubsection):
        def __init__(self, window):
            self._relative_position_prop = SimulationDataVisualizerWindow.DataProperty(window, "Offset")
            self._plot_relative_position = SimulationDataVisualizerWindow.DataPlot(self._relative_position_prop)
            self._linear_velocity_prop = SimulationDataVisualizerWindow.DataProperty(window, "Velocity")
            self._current_offset = None
            self._current_linear_velocity = None
            self._plot_linear_velocity = SimulationDataVisualizerWindow.DataPlot(self._linear_velocity_prop)
            self._linear_acceleration_prop = SimulationDataVisualizerWindow.DataProperty(window, "Acceleration")
            self._plot_linear_acceleration = SimulationDataVisualizerWindow.DataPlot(self._linear_acceleration_prop)

            self._lower_limit_attr = None
            self._upper_limit_attr = None
            self._axis_attr = None
            self._current_axis = None
            self._current_axis_index = None
            self._prismatic_joint = None
            super().__init__(window)

        def refresh_simulation_data(self, delta_time = 0.0):
            delta_xform = self._get_delta_xform()
            delta_quaternion = delta_xform.GetOrthonormalized().ExtractRotationQuat()
            if delta_quaternion.GetReal() < 0.0:
                delta_quaternion = -delta_quaternion

            translation = delta_xform.ExtractTranslation()
            translation = delta_quaternion.GetInverse().Transform(translation)

            if self._current_offset is None:
                self._current_offset = translation[self._current_axis_index]

            if delta_time > 0.0:
                velocity = (translation[self._current_axis_index] - self._current_offset) * (1.0 / delta_time)
                self._current_offset = translation[self._current_axis_index]
                acceleration = (velocity - self._current_linear_velocity) * (1.0 / delta_time) if self._current_linear_velocity is not None else 0.0
                self._current_linear_velocity = velocity
                self._plot_linear_velocity.append_data(delta_time, velocity)
                self._plot_linear_acceleration.append_data(delta_time, acceleration)

            self._plot_relative_position.append_data(delta_time, translation[self._current_axis_index])
            super().refresh_simulation_data(delta_time)

        def refresh_data(self):
            if self._current_prim != self.prim:
                self._prismatic_joint = UsdPhysics.PrismaticJoint(self.prim)
                self._axis_attr = self._prismatic_joint.GetAxisAttr()
                self._lower_limit_attr = self._prismatic_joint.GetLowerLimitAttr()
                self._upper_limit_attr = self._prismatic_joint.GetUpperLimitAttr()
                self._current_offset = None
                self._current_linear_velocity = None

            new_axis = self._axis_attr.Get()
            if self._current_axis is not new_axis:
                self._current_axis = new_axis
                self._current_axis_index = axis_token_to_index(self._current_axis)
                self._relative_position_prop.name = new_axis + "-offset"
                # self._plot_relative_position.axis = self._current_axis_index
                self.reset_plots()

            super().refresh_data()

        def reset_plots(self):
            self._current_offset = None
            self._current_linear_velocity = None
            self._plot_relative_position.reset_data()
            self._plot_linear_velocity.reset_data()
            self._plot_linear_acceleration.reset_data()

        def _build_local_offset_section(self):
            self._plot_relative_position.build_ui()

        def _build_linear_velocity_section(self):
            self._plot_linear_velocity.build_ui()
            self._plot_linear_acceleration.build_ui()

        def build_ui(self):
            super().build_ui()
            self.build_collapsable_section("Local Offset", self._build_local_offset_section)
            self.build_collapsable_section("Linear Velocity", self._build_linear_velocity_section)

        @classmethod
        def get_is_prim_valid_target(cls, prim : Usd.Prim) -> bool:
            return (SimulationDataVisualizerWindow.MonitorSubsection.get_is_prim_valid_target(prim)
                    and prim.IsA(UsdPhysics.PrismaticJoint))

    class MonitorResiduals(MonitorSubsection):
        def __init__(self, window):
            super().__init__(window)

            self._residual_api = None
            self._residual_attr_position_rms = None
            self._residual_attr_position_max = None
            self._residual_attr_velocity_rms = None
            self._residual_attr_velocity_max = None
            self._plot_residual_position_rms = SimulationDataVisualizerWindow.DataPlot(
                SimulationDataVisualizerWindow.DataProperty(window, "Position RMS"))
            self._plot_residual_position_max = SimulationDataVisualizerWindow.DataPlot(
                SimulationDataVisualizerWindow.DataProperty(window, "Position Max"))

            self._plot_residual_velocity_rms = SimulationDataVisualizerWindow.DataPlot(
                SimulationDataVisualizerWindow.DataProperty(window, "Velocity RMS"))
            self._plot_residual_velocity_max = SimulationDataVisualizerWindow.DataPlot(
                SimulationDataVisualizerWindow.DataProperty(window, "Velocity Max"))

            self._current_prim = None

        def refresh_simulation_data(self, delta_time = 0.0):
            super().refresh_simulation_data(delta_time)
            if self._residual_api is not None:
                self._plot_residual_position_rms.append_data(delta_time, self._residual_attr_position_rms.Get())
                self._plot_residual_position_max.append_data(delta_time, self._residual_attr_position_max.Get())

                self._plot_residual_velocity_rms.append_data(delta_time, self._residual_attr_velocity_rms.Get())
                self._plot_residual_velocity_max.append_data(delta_time, self._residual_attr_velocity_max.Get())

        def reset_plots(self):
            self._plot_residual_position_rms.reset_data()
            self._plot_residual_position_max.reset_data()
            self._plot_residual_velocity_rms.reset_data()
            self._plot_residual_velocity_max.reset_data()

        def refresh_data(self):
            if self._current_prim != self.prim:
                self._current_prim = self.prim
                self.reset_plots()

            if self.prim is None:
                self._residual_api = None
                self._residual_attr_position_rms = None
                self._residual_attr_position_max = None
                self._residual_attr_velocity_rms = None
                self._residual_attr_velocity_max = None
                super().refresh_data()
                return

            self._residual_api = PhysxSchema.PhysxResidualReportingAPI(self.prim)
            self._residual_attr_position_rms = self._residual_api.GetPhysxResidualReportingRmsResidualPositionIterationAttr()
            self._residual_attr_position_max = self._residual_api.GetPhysxResidualReportingMaxResidualPositionIterationAttr()
            self._residual_attr_velocity_rms = self._residual_api.GetPhysxResidualReportingRmsResidualVelocityIterationAttr()
            self._residual_attr_velocity_max = self._residual_api.GetPhysxResidualReportingMaxResidualVelocityIterationAttr()
            super().refresh_data()

        def _build_residual_section(self):
            self._plot_residual_position_rms.build_ui()
            self._plot_residual_position_max.build_ui()
            self._plot_residual_velocity_rms.build_ui()
            self._plot_residual_velocity_max.build_ui()

        def build_ui(self):
            super().build_ui()
            self.build_collapsable_section("Residuals", self._build_residual_section)

        @classmethod
        def get_is_prim_valid_target(cls, prim : Usd.Prim) -> bool:
            return super().get_is_prim_valid_target(prim) and prim.HasAPI(PhysxSchema.PhysxResidualReportingAPI) and not prim.IsA(UsdPhysics.Joint)

    class MonitorResidualsJoint(MonitorSubsection):
        def __init__(self, window):
            super().__init__(window)

            self._residual_api = None
            self._residual_attr_position = None
            self._residual_attr_velocity = None
            residual_position_prop = SimulationDataVisualizerWindow.DataProperty(window, "Position")
            self._plot_residual_position = SimulationDataVisualizerWindow.DataPlot(residual_position_prop)
            residual_velocity_prop = SimulationDataVisualizerWindow.DataProperty(window, "Velocity")
            self._plot_residual_velocity = SimulationDataVisualizerWindow.DataPlot(residual_velocity_prop)

            self._current_prim = None

        def refresh_simulation_data(self, delta_time = 0.0):
            super().refresh_simulation_data(delta_time)
            if self._residual_api is not None:
                self._plot_residual_position.append_data(delta_time, self._residual_attr_position.Get())
                self._plot_residual_velocity.append_data(delta_time, self._residual_attr_velocity.Get())

        def reset_plots(self):
            self._plot_residual_position.reset_data()
            self._plot_residual_velocity.reset_data()

        def refresh_data(self):
            if self._current_prim != self.prim:
                self._current_prim = self.prim
                self.reset_plots()

            if self.prim is None:
                self._residual_api = None
                self._residual_attr_position = None
                self._residual_attr_velocity = None
                super().refresh_data()
                return

            self._residual_api = PhysxSchema.PhysxResidualReportingAPI(self.prim)
            self._residual_attr_position = self._residual_api.GetPhysxResidualReportingRmsResidualPositionIterationAttr()
            self._residual_attr_velocity = self._residual_api.GetPhysxResidualReportingRmsResidualVelocityIterationAttr()
            super().refresh_data()

        def _build_residual_section(self):
            self._plot_residual_position.build_ui()
            self._plot_residual_velocity.build_ui()

        def build_ui(self):
            super().build_ui()
            self.build_collapsable_section("Residual", self._build_residual_section)

        @classmethod
        def get_is_prim_valid_target(cls, prim : Usd.Prim) -> bool:
            return super().get_is_prim_valid_target(prim) and prim.HasAPI(PhysxSchema.PhysxResidualReportingAPI) and prim.IsA(UsdPhysics.Joint)

    class SharedStates():
        sections_expanded = set()

    def __init__(self, window_manager, title):
        super().__init__(
            title,
            visible=True,
            flags=ui.WINDOW_FLAGS_NO_SCROLLBAR, width=self.WINDOW_WIDTH_MIN, height=500
        )

        self._window_manager: SimulationDataVisualizerWindowManager = window_manager

        self.frame.set_build_fn(self.build)

        self._rigid_body_property_query_manager = None
        self._refresh_text_on_query_complete = False
        self._usd_context = omni.usd.get_context()
        self._stage_event_sub = [
            get_eventdispatcher().observe_event(
                observer_name="omni.physx.ui:SimulationDataVisualizer",
                event_name=self._usd_context.stage_event_name(event),
                on_event=func,
             )
            for event, func in (
                (omni.usd.StageEventType.OPENED, lambda _: self._attach_stage()),
                (omni.usd.StageEventType.CLOSING, lambda _: self._detach_stage()),
                (omni.usd.StageEventType.SELECTION_CHANGED, lambda _: self._refresh_target()),
            )
        ]
        self._stage_update_node = None
        self._stage: Usd.Stage = None

        self._physx_simulation_event_stream = omni.physx.get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(
            self._on_simulation_event)

        self._physx_step_sub = None
        self._simulation_time_current = 0.0
        self._simulation_time_delta = 0.0
        self._simulation_times = [0.0]
        self._usd_object_changed_listener = None

        self._data_monitor_basic: __class__.MonitorPrimBasic = None
        self._data_monitors: list[__class__.MonitorSubsection] = []

        self._prim_path: Sdf.Path | None = None
        self._prim: Usd.Prim | None = None
        self._prim_locked = False

        self._usdrt_stage = None
        self._usdrt_prim = None

        self.set_visibility_changed_fn(self._window_visiblity_changed_fn)

        self._attach_stage()

    def _window_visiblity_changed_fn(self, visible):
        # handle the case when user closes the window by a cross
        if not visible:
            self._window_manager.refresh()

    def _refresh_monitor_data(self):
        prim_monitor_types = self.get_prim_monitor_types(self._prim)
        refresh_data = True
        if len(prim_monitor_types) != len(self._data_monitors):
            self._data_monitors.clear()
            self.frame.rebuild()
            refresh_data = False
        else:
            for monitor, prim_monitor_type in zip(self._data_monitors, prim_monitor_types):
                if type(monitor) is not prim_monitor_type:
                    refresh_data = False
                    self._data_monitors.clear()
                    self.frame.rebuild()
                    break

        if refresh_data:
            self._data_monitor_basic.refresh_data()
            for monitor in self._data_monitors:
                monitor.refresh_data()

    def _refresh_monitor_simulation_data(self, time_delta):
        self._simulation_times.append(self._simulation_times[-1] + time_delta)
        self._simulation_time_current += time_delta
        for monitor in self._data_monitors:
            monitor.refresh_simulation_data(time_delta)

    def _reset_monitor_data(self):
        self._simulation_time_current = 0.0
        self._simulation_time_delta = 0.0
        self._simulation_times.clear()
        self._simulation_times.append(0.0)

        for monitor in self._data_monitors:
            monitor.reset_plots()

        self._refresh_monitor_data()

    def _refresh_target(self):
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

            if self._physx_step_sub is None:
                physx_interface = omni.physx.get_physx_interface()
                if physx_interface is not None:
                    self._physx_step_sub = physx_interface.subscribe_physics_on_step_events(self._on_simulation_step, False, 0 )
                else:
                    carb.log_error("Failed to retrieve Physx interface.")

            if self._stage_update_node is None:
                self.stage_update_node = register_stage_update_node("simDataVisualization", on_update_fn=self._on_stage_update, priority=12)

            if self._usdrt_stage is not None:
                self._usdrt_prim = self._usdrt_stage.GetPrimAtPath(str(prim_path))
                if self._usdrt_prim is None:
                    carb.log_error(f"Failed to retrieve USDRT Prim at path {self._prim_path}.")
            else:
                self._usdrt_prim = None
        else:
            self._prim = None
            self._physx_step_sub = None
            self._stage_update_node = None
            self._usdrt_prim = None

        self._reset_monitor_data()

    def _on_simulation_step(self, delta_time):
        # To support manual stepping and for accuracy, we use simulation time delta rather than just the stage update time.
        self._simulation_time_delta += delta_time

    def _on_stage_update(self, current_time, delta_time):
        if self._simulation_time_delta <= 0.0:
            return

        self._refresh_monitor_simulation_data(self._simulation_time_delta)

        self._simulation_time_delta = 0.0

    def _attach_stage(self):
        self._stage = self._usd_context.get_stage()
        if self._stage is not None:
            self._usdrt_stage = UsdRt.Stage.Attach(self._usd_context.get_stage_id())
            if self._usdrt_stage is None:
                carb.log_error("Failed to attach USDRT to stage.")

            self._attach_usd_change_listener()
        self._refresh_target()

    def _detach_stage(self):
        self._physx_step_sub = None
        self._stage_update_node = None
        self._stage = None
        self._usdrt_stage = None
        self._set_target_path(None)
        self._revoke_usd_change_listener()

    def _attach_usd_change_listener(self):
        if self._usd_object_changed_listener is None and self._stage is not None:
            self._usd_object_changed_listener = Tf.Notice.Register(
                Usd.Notice.ObjectsChanged, self._on_usd_objects_changed,
                self._stage
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
            # UI plots sometimes fail to update so during simulation we redraw each frame, see OMPE-5467
            self.raster_policy = ui.RasterPolicy.NEVER
        elif event.type == int(SimulationEvent.PAUSED) or event.type == int(SimulationEvent.STOPPED):
            self.raster_policy = ui.RasterPolicy.AUTO       # See comment above
            if event.type == int(SimulationEvent.STOPPED):
                self._reset_monitor_data()
            self._attach_usd_change_listener()

    def get_prim_monitor_types(self, prim: Usd.Prim) -> list[MonitorSubsection]:
        monitor_types = []

        for subclass in __class__.MonitorSubsection.__subclasses__():
            if subclass.get_is_prim_valid_target(prim):
                monitor_types.append(subclass)

        return monitor_types

    def build(self):
        if not self.visible:
            return

        self.frame.set_style({"Window": {"background_color": self.WINDOW_COLOR, "border_color": 0x0, "border_width": 0, "border_radius": 5}})

        with self.frame:
            vstack_outer = ui.VStack(spacing=5, style=__class__.active_style)
            with vstack_outer:
                self._data_monitor_basic = __class__.MonitorPrimBasic(self)
                self._data_monitor_basic.build_ui()
                self._data_monitor_basic.refresh_data()
                with ui.ScrollingFrame():
                    with ui.VStack(height=0, spacing=5):
                        self._data_monitors.clear()
                        monitor_types = self.get_prim_monitor_types(self._prim)

                        for monitor_type in monitor_types:
                            self._data_monitors.append(monitor_type(self))

                        for monitor in self._data_monitors:
                            monitor.build_ui()
                            monitor.refresh_data()

    def destroy(self):
        self._detach_stage()
        self._stage_event_sub = None
        self._physx_simulation_event_stream = None
        self.set_visibility_changed_fn(None)
        super().destroy()


class SimulationDataVisualizerWindowManager():
    def __init__(self) -> None:
        self._windows: list[SimulationDataVisualizerWindow] = []

        self._background_opacity_setting = get_settings().subscribe_to_node_change_events(
            SETTINGS_UI_WINDOW_OPACITY, self._on_opacity_setting_changed
        )

        SimulationDataVisualizerWindow.refresh_style()

        self.add_window()

    def add_window(self, creator_window: SimulationDataVisualizerWindow = None):
        window_name = SimulationDataVisualizerWindow.WINDOW_TITLE
        suffix = 1
        window = ui.Workspace.get_window(window_name)
        while window is not None:
            if not isinstance(window, SimulationDataVisualizerWindow):
                # Even if the window was destroyed, it will still show up but only as a handle. In this case we just overwrite it.
                break

            suffix +=1
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

    def destroy(self):
        for window in self._windows:
            window.destroy()
        self._windows.clear()

    def refresh(self):
        windows_visible = []
        for window in self._windows:
            if window.visible:
                windows_visible.append(window)
            else:
                window.destroy()

        self._windows = windows_visible

        if len(self._windows) < 1:
            get_settings().set_bool(SETTING_DISPLAY_SIMULATION_DATA_VISUALIZER, False)

    def _on_opacity_setting_changed(self, item, event_type):
        SimulationDataVisualizerWindow.refresh_style()
        for window in self._windows:
            if window is not None and window.visible:
                window.frame.rebuild()

    def __del__(self):
        self.destroy()
