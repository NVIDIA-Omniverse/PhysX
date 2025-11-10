# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui as ui
import omni.kit.app
import omni.usd
import carb
from carb.eventdispatcher import get_eventdispatcher
from omni.ui_scene import scene as sc
from omni.ui import color as cl
from .physicsViewportOverlayShared import *


class PhysicsSubsectionCollapseClickGesture(PhysicsUIClickGesture):

    def __init__(self, subsection):
        super().__init__(subsection.manipulator, subsection._collapse_toggle)
        self._subsection = subsection

    def on_ended(self):
        super().on_ended()
        self._subsection.toggle_collapsed()


class PhysicsSubsectionCollapseHoverGesture(PhysicsUIHoverGesture):
    def __init__(self, subsection):
        super().__init__(subsection.manipulator, subsection._collapse_toggle)
        self._subsection = subsection


class PhysicsBackgroundHoverGesture(PhysicsUIHoverGesture):
    def __init__(self, manipulator):
        super().__init__(manipulator)

    def on_began(self):
        super().on_began()

    def on_ended(self):
        super().on_ended()


class PhysicsUIFloatingInfobox(PhysicsUIManipulator):
    """
    UI manipulator for a viewport floating infobox.

    The infobox contains a number of subsections (derived from Subsection) that can be changed dynamically.
    """
    # Defaults:
    width = 270.0
    background_color = cl("#1E212360")
    title_background_color = cl("00000020")
    default_font_color = COLOR_TEXT
    default_font_size = 14
    default_font_line_spacing = 1.15  # Kit default
    subsection_background_color = cl("00000030")
    subsection_margin = 4
    subsection_padding = 4

    subsections_collapsed = []

    class Subsection:
        class Style:
            def __init__(self, font_color: int = None, font_size: float = None, font_line_spacing: float = None):
                self.font_color = font_color
                self.font_size = font_size
                self.font_line_spacing = font_line_spacing

        def __init__(self, info_box : 'PhysicsUIFloatingInfobox', title: str = None, entries: list[str] = [""], columns: int = 1, style: Style = None, title_style=None):
            if style is not None:
                self._style: 'PhysicsUIFloatingInfobox.Subsection.Style' = style
            else:
                self._style: 'PhysicsUIFloatingInfobox.Subsection.Style' = info_box._style

            if title_style is not None:
                self._title_style = title_style
            else:
                self._title_style = self._style

            self.manipulator : PhysicsUIFloatingInfobox = info_box

            try:
                self.manipulator.subsections_collapsed.index(__class__)
            except:
                self._collapsed = False
            else:
                self._collapsed = True

            self._collapsed_transform = None
            self._collapse_toggle = PhysicsUIToggleGroup()

            self.text_title = title

            self.columns = columns
            self.text_entries = [None] * columns
            self._ui_text_entries_shape = [None] * columns
            self._text_entries_transform = [None] * columns

            for n in range(columns):
                self.text_entries[n] = (entries if n == 0 else [""] * len(entries))
                self._ui_text_entries_shape[n] = [None] * len(entries)
                self._text_entries_transform[n] = [None] * len(entries)

            self._ui_root_transform = None
            self._ui_subsection_background_transform = None
            self._ui_text_title_transform = None
            self._ui_text_title_shape = None

        def get_visible(self):
            return self._ui_root_transform.visible

        def get_is_active(self) -> bool:
            return True

        def toggle_collapsed(self):
            self._collapsed = not self._collapsed
            try:
                self.manipulator.subsections_collapsed.index(__class__)
            except:
                if self._collapsed:
                    self.manipulator.subsections_collapsed.append(__class__)
            else:
                if not self._collapsed:
                    self.manipulator.subsections_collapsed.remove(__class__)
            self.manipulator.refresh()

        def _get_font_size(self) -> float:
            if self._style.font_size is not None:
                return self._style.font_size
            return self.manipulator._style.font_size

        def _get_title_font_size(self) -> float:
            if self._title_style.font_size is not None:
                return self._title_style.font_size
            return self._get_font_size()

        def _get_font_line_spacing(self) -> float:
            if self._style.font_line_spacing is not None:
                return self._style.font_line_spacing
            return self.manipulator._style.font_line_spacing

        def _get_title_font_line_spacing(self) -> float:
            if self._title_style.font_line_spacing is not None:
                return self._title_style.font_line_spacing
            return self._get_font_line_spacing()

        def _get_font_color(self) -> int:
            if self._style.font_color is not None:
                return self._style.font_color
            return self.manipulator._style.font_color

        def _get_title_font_color(self) -> int:
            if self._title_style.font_color is not None:
                return self._title_style.font_color
            return self._get_font_color()

        def refresh(self):
            if not self.get_is_active():
                self._ui_root_transform.visible = False
                return

            self._ui_root_transform.visible = True
            height = self.get_height()
            inner_width = self.manipulator.width - self.manipulator.subsection_margin * 2

            self._collapsed_transform.visible = not self._collapsed

            self._ui_subsection_background_transform.transform = sc.Matrix44.get_translation_matrix(self.manipulator.width * 0.5, -height * 0.5, -0.01) * sc.Matrix44.get_scale_matrix(inner_width, height, 1.0)

        def _make_ui_shapes(self):
            with self.manipulator._ui_transform_screen:
                self._ui_root_transform = sc.Transform()

            with self._ui_root_transform:
                offset_y = 0
                inner_width = self.manipulator.width - self.manipulator.subsection_margin * 2
                inner_width_padded = inner_width - self.manipulator.subsection_padding * 2
                self._ui_subsection_background_transform = sc.Transform()
                with self._ui_subsection_background_transform:
                    sc.Rectangle(1.0, 1.0, color=self.manipulator.subsection_background_color)

                if self.text_title is not None:
                    line_height = self.get_title_font_height()
                    offset_y += self.manipulator.subsection_padding
                    self._ui_text_title_transform = sc.Transform()
                    self._ui_text_title_transform.transform = sc.Matrix44.get_translation_matrix(self.manipulator.width * 0.5, -offset_y, 0.0)
                    with self._ui_text_title_transform:
                        self._ui_text_title_shape = sc.Label(
                            self.text_title,
                            alignment=ui.Alignment.CENTER_TOP,
                            color=self._get_title_font_color(),
                            size=self._get_title_font_size())
                    with sc.Transform(transform=sc.Matrix44.get_translation_matrix(self.manipulator.width * 0.5, -line_height * 0.5 - offset_y, 0.0)):
                        toggles = self._collapse_toggle.get_manipulator_toggles(self.manipulator)
                        toggles.clear()

                        self._title_background = sc.Rectangle(
                            inner_width,
                            line_height + self.manipulator.subsection_padding * 2,
                            color=self.manipulator.title_background_color,
                            gesture=[PhysicsSubsectionCollapseClickGesture(self), PhysicsSubsectionCollapseHoverGesture(self)])

                        toggles.append(HighlightToggle(self._ui_text_title_shape)) # self._title_background))
                    offset_y += line_height
                    offset_y += self.manipulator.subsection_padding

                height = offset_y

                self._collapsed_transform = sc.Transform()

                with self._collapsed_transform:
                    for column in range(self.columns):
                        self._ui_text_entries_shape[column] = []
                        self._text_entries_transform[column] = []
                        offset_y_column = offset_y + self.manipulator.subsection_padding
                        offset_x = self.manipulator.subsection_margin + self.manipulator.subsection_padding + (0.0 if column == 0 else inner_width_padded - (self.columns - column - 1) * inner_width_padded / (self.columns + 1))
                        for entry in self.text_entries[column]:
                            entry_transform = sc.Transform()
                            entry_transform.transform = sc.Matrix44.get_translation_matrix(offset_x, -offset_y_column, 0.0)
                            offset_y_column += self._get_font_line_spacing() * self._get_font_size()
                            self._text_entries_transform[column].append(entry_transform)
                            with entry_transform:
                                self._ui_text_entries_shape[column].append(sc.Label(
                                    entry,
                                    alignment=(ui.Alignment.LEFT if column == 0 else ui.Alignment.RIGHT),
                                    color=self._get_font_color(),
                                    size=self._get_font_size()))

                        height = max(height, offset_y_column + self.manipulator.subsection_padding)

        def get_title_font_height(self) -> float:
            return round(self._get_title_font_size() * self._get_title_font_line_spacing())

        def get_title_height(self) -> float:
            if self.text_title is not None:
                return self.get_title_font_height() + self.manipulator.subsection_padding * 2
            return 0.0

        def get_font_height(self) -> float:
            return round(self._get_font_size() * self._get_font_line_spacing())

        def get_height(self) -> float:
            height = 0.0
            lines = 0
            if not self._collapsed:
                for text_entries in self.text_entries:
                    lines = max(lines, len(text_entries))
                if lines > 0:
                    height += lines * self.get_font_height() + self.manipulator.subsection_padding * 2
            return height + self.get_title_height()

        def set_title(self, title: str = None):
            self.text_title = title
            if not self.get_visible():
                return
            if ((title is None) != (self._ui_text_title_shape is None)):
                self.manipulator.invalidate()
            else:
                self._ui_text_title_shape.text = title

        def set_text(self, text: list[str], column: int = 0):
            self.text_entries[column] = text
            if not self.get_visible() or self._collapsed is True:
                return
            if (len(self._ui_text_entries_shape[column]) != len(text)):
                self.manipulator.invalidate()
            else:
                for text_entry, shape in zip(text, self._ui_text_entries_shape[column]):
                    if shape is not None:
                        shape.text = text_entry

        # Set to None to use parent manipulator style.
        def set_style(self, style: Style):
            if style is not None:
                self._style = style
            else:
                self._style = self.manipulator._style
            if not self.get_visible():
                return
            self.manipulator.invalidate()

        # Set to None to use text font style.
        def set_title_style(self, style: Style):
            if style is not None:
                self._title_style = style
            else:
                self._title_style = self._style
            self._title_style = style
            if not self.get_visible():
                return
            self.manipulator.invalidate()

        def destroy(self):
            self._collapse_toggle = None
            pass

        def __del__(self):
            self.destroy()

    class Style(Subsection.Style):
        def __init__(self, font_color=None, font_size=None, font_line_spacing=None):
            super().__init__(font_color, font_size, font_line_spacing)

    def __init__(self, viewport_overlay, style: Style = None, **kwargs):
        super().__init__(viewport_overlay, **kwargs)

        if style is not None:
            self._style = style
        else:
            self._style = self.Style(self.default_font_color, self.default_font_size, self.default_font_line_spacing)

        self._ui_root_transform : sc.Transform = None
        self._screen_position_current : list[float, float, float] = None

        self._ui_background_shape : sc.Rectangle = None
        self._ui_background_transform : sc.Transform = None

        self._height = 0

        self._subsection_types: list[type[PhysicsUIFloatingInfobox.Subsection]] = []
        self._subsections: list[PhysicsUIFloatingInfobox.Subsection] = []

        self._subscription_kit_update = get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
            on_event=self._on_kit_update,
            observer_name="omni.usdphysics.ui: infobox",
        )

    def destroy(self):
        self._subscription_kit_update = None
        super().destroy()

    def get_is_active(self) -> bool:
        return True

    def set_subsection_types(self, subsections: list[type[Subsection]]):
        rebuild = False
        if len(subsections) != len(self._subsection_types):
            rebuild = True
        else:
            for a, b in zip(subsections, self._subsection_types):
                if a != b:
                    rebuild = True
                    break
        if rebuild:
            self._subsection_types = subsections
            self.invalidate()

    def refresh(self):
        if self._ui_root_transform is None:
            self.invalidate()
            return

        if not self.get_is_active():
            self.visible = False
            return

        self.visible = True

        self._height = self.subsection_margin

        for subsection in range(len(self._subsections)):
            self._subsections[subsection].refresh()

            if not self._subsections[subsection].get_visible():
                continue

            self._subsections[subsection]._ui_root_transform.transform = sc.Matrix44.get_translation_matrix(0.0, -self._height, 0.0)
            self._height += self._subsections[subsection].get_height() + self.subsection_margin

        self._ui_background_transform.transform = sc.Matrix44.get_translation_matrix(self.width * 0.5, -self._height * 0.5, 0.0) * sc.Matrix44.get_scale_matrix(self.width, self._height, 1.0)

        self._refresh_position()

    # Offset in world units relative to camera view
    def get_view_offset(self) -> tuple[float, float]:
        return (0.0, 0.0)

    # Offset in pixels relative to camera view
    def get_screen_offset(self) -> tuple[float, float]:
        return (0.0, 0.0)

    def get_world_position(self) -> tuple[float, float, float]:
        return None

    def _refresh_position(self, instant: bool = False):
        """
        Handles positioning of the infobox, applying smooth movement and clamping position to fit within the viewport.

        """
        if (self._ui_root_transform is None or not self.visible or not self.get_is_active()):
            self._screen_position_current = None
            return

        world_position_target = self.get_world_position()
        if (world_position_target is None):
            self._ui_root_transform.visible = False
            self._screen_position_current = None
            return

        # Infoboxes may provide a desired view aligned (camera relative) offset.
        view_offset = self.manager.align_to_camera([*self.get_view_offset()] + [0])
        world_position_target = components_sum(world_position_target, view_offset)

        screen_position_new = self.manager.transform_world_to_screen(world_position_target)

        # Infoboxes may also provide a screen space (pixel) offset.
        screen_offset = [*self.get_screen_offset()] + [0]
        screen_position_new = components_sum(screen_position_new, screen_offset)

        # Check if the new position is within the screen boundaries. If exceeding significantly, we simply hide the infobox instead.
        screen_position_new_ndc = self.manager._overlay_transform_screen.transform_space(sc.Space.OBJECT, sc.Space.NDC, screen_position_new)
        if (screen_position_new_ndc[0] < -3.0 or screen_position_new_ndc[0] > 3.0 or
            screen_position_new_ndc[1] < -3.0 or screen_position_new_ndc[1] > 3.0 or
            screen_position_new_ndc[2] < -1.0 or screen_position_new_ndc[2] > 1.0):

            self._ui_root_transform.visible = False
            self._screen_position_current = None
            return

        self._ui_root_transform.visible = True

        # Add some smoothing.
        if self._screen_position_current is not None:
            screen_position_new = components_lerped(self._screen_position_current, screen_position_new, 0.25)

        # Determine the boundaries of the viewport in pixel coords.
        screen_min = self.manager._overlay_transform_screen.transform_space(sc.Space.NDC, sc.Space.OBJECT, [-1.0, -1.0, 0.0])
        screen_max = self.manager._overlay_transform_screen.transform_space(sc.Space.NDC, sc.Space.OBJECT, [1.0, 1.0, 1.0])

        # Shrink to make sure that our own dimensions will also be able to fit.
        screen_max[0] -= self.width
        screen_min[1] += self._height

        # Now clamp the coordinates to fit.
        screen_position_new[0] = max(min(screen_position_new[0], screen_max[0]), screen_min[0])
        screen_position_new[1] = max(min(screen_position_new[1], screen_max[1]), screen_min[1])

        self._screen_position_current = screen_position_new
        world_position_new = self.manager.transform_screen_to_world(screen_position_new)
        self._ui_root_transform.transform = sc.Matrix44.get_translation_matrix(*world_position_new)

    def _on_kit_update(self, event: carb.events.IEvent):
        self._refresh_position()

    def _make_ui_shapes(self):
        self._ui_root_transform = sc.Transform()
        with self._ui_root_transform:
            with ui_create_screen_scale_transform(True):
                self._ui_transform_screen = sc.Transform()

        with self._ui_transform_screen:
            with sc.Transform(sc.Matrix44.get_translation_matrix(0.0, 0.0, -0.1)):
                self._ui_background_transform = sc.Transform()
                with self._ui_background_transform:
                    self._ui_background_shape = sc.Rectangle(1, 1, color=self.background_color, gesture=[PhysicsBackgroundHoverGesture(self)])

        for subsection in self._subsections:
            subsection._make_ui_shapes()

    def on_build(self):
        super().on_build()

        if not self.get_is_active():
            return

        # Verify that we have the correct subsection types.
        while len(self._subsections) > len(self._subsection_types):
            to_remove = self._subsections.pop()
            to_remove.destroy()

        for subsection_type in range(len(self._subsection_types)):
            if subsection_type < len(self._subsections):
                if(type(self._subsections[subsection_type]) != self._subsection_types[subsection_type]):
                    self._subsections[subsection_type].destroy()
                    self._subsections[subsection_type] = self._subsection_types[subsection_type](self)
            else:
                self._subsections.append(self._subsection_types[subsection_type](self))

        self._make_ui_shapes()
        self.refresh()

    def invalidate(self):
        self._ui_root_transform = None
        super().invalidate()


class PhysicsUIFloatingBoxViewportOverlay(PhysicsUIViewportOverlay):
    def __init__(self, main_viewport_overlay):
        super().__init__(main_viewport_overlay)

    def destroy(self):
        super().destroy()

    def __del__(self):
        self.destroy()
