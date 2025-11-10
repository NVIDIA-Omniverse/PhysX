# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui as ui
import omni.kit.app
import omni.usd
import carb
from omni.ui_scene import scene as sc
from omni.ui import color as cl
import collections.abc

UI_RESCALE_FACTOR = 2.0 # This is due to the UI.scene screen scaling error/inconsistency, see OM-76738 

INFO_BOX_MARGIN = 10.0 * UI_RESCALE_FACTOR
INFO_BOX_BACKGROUND_COLOR_DEFAULT = cl(31,31,31,191)
INFO_BOX_TEXT_FONT_SIZE_DEFAULT = 16.0
INFO_BOX_TEXT_LINE_SPACING = 1.5 * UI_RESCALE_FACTOR
INFO_BOX_TEXT_COLOR_DEFAULT = cl(225,225,225,255)
# Since we have no mean to get the exact pixel width of a text string, we have to use this factor to do a loose estimate.
INFO_BOX_TEXT_FONT_WIDTH_FACTOR = 0.5 * UI_RESCALE_FACTOR 
class PhysicsInfoBoxTextManipulator(sc.Manipulator):
    def __init__(self, text = [], text_color=INFO_BOX_TEXT_COLOR_DEFAULT, text_font_size = INFO_BOX_TEXT_FONT_SIZE_DEFAULT, text_alignment = ui.Alignment.CENTER, background_color = INFO_BOX_BACKGROUND_COLOR_DEFAULT, screen_coords = [0, 0], height = -1, width = -1):
        super().__init__()
        if type(text) == str:
            text = [text]
        self._text = text
        self._text_color = text_color
        self._text_alignment = text_alignment
        self._text_font_size = text_font_size
        self._background_color = background_color
        self._screen_coords = screen_coords
        self._height = height
        self._width = width

        self._box_rectangle_shape = None
        self._text_label_shapes = []
        self._translate_screen = None

    def on_build(self):
        if self._text is None:
            return

        self._text_label_shapes = []

        with sc.Transform(scale_to=omni.ui_scene.scene.Space.SCREEN):
            self._translate_screen = sc.Transform(transform=sc.Matrix44.get_translation_matrix(0, 0, 0))
            y_offset = 0.0
            with self._translate_screen:
                lines = len(self._text)
                if isinstance(self._text_font_size, collections.abc.Sized):
                    if len(self._text_font_size) != lines:
                        carb.log_error(f"Text font size array was provided for text, but the lengths of the arrays did not match. Text: {self._text} Sizes: {self._text_font_size}")
                        return
                    y_offset = self._text_font_size[0] * INFO_BOX_TEXT_LINE_SPACING
                else:
                    y_offset = self._text_font_size * INFO_BOX_TEXT_LINE_SPACING

                if isinstance(self._text_color, collections.abc.Sized):
                    if len(self._text_color) != lines:
                        carb.log_error(f"Text color array was provided for text, but the lengths of the arrays did not match. Text: {self._text} Colors: {self._text_color}")
                        return

                text_size = 0
                max_width = 0
                for n in range(len(self._text)):
                    if isinstance(self._text_color, collections.abc.Sized):
                        color = self._text_color[n]
                    else:
                        color = self._text_color
                    if isinstance(self._text_font_size, collections.abc.Sized):
                        font_size = self._text_font_size[n]
                    else:
                        font_size = self._text_font_size
                    y_offset -= font_size * INFO_BOX_TEXT_LINE_SPACING
                    
                    with sc.Transform(transform = sc.Matrix44.get_translation_matrix(0.0, y_offset, 0.0)):
                        # Check against "" since it triggers a crash.
                        self._text_label_shapes.append(sc.Label(self._text[n] if self._text[n] != "" else " ", alignment=self._text_alignment, color = color, size=font_size))
                    text_size += font_size * INFO_BOX_TEXT_LINE_SPACING
                    max_width = max(max_width, len(self._text[n]) * font_size)

                width = self._width * UI_RESCALE_FACTOR if self._width != -1.0 else max_width * INFO_BOX_TEXT_FONT_WIDTH_FACTOR + INFO_BOX_MARGIN * 2.0
                height = self._height * UI_RESCALE_FACTOR if self._height != -1.0 else text_size + INFO_BOX_MARGIN * 2.0
                if self._background_color is not None:
                    with sc.Transform(transform = sc.Matrix44.get_translation_matrix(0.0, y_offset / 2, 0.0)):
                        self._box_rectangle_shape = sc.Rectangle(width, height, color=self._background_color)
            self._translate_screen.transform = sc.Matrix44.get_translation_matrix(self._screen_coords[0] * UI_RESCALE_FACTOR, self._screen_coords[1] * UI_RESCALE_FACTOR - y_offset / 2, 0)

    def _readjust_box_width(self):
        if self._width == -1.0 and self._box_rectangle_shape:
            new_max_width = 0.0
            for n in range(len(self._text_label_shapes)):
                new_max_width  = max(new_max_width, len(self._text_label_shapes[n].text) * self._text_label_shapes[n].size)
            if self._box_rectangle_shape.width != new_max_width * INFO_BOX_TEXT_FONT_WIDTH_FACTOR + INFO_BOX_MARGIN * 2.0:
                self._box_rectangle_shape.width = new_max_width * INFO_BOX_TEXT_FONT_WIDTH_FACTOR + INFO_BOX_MARGIN * 2.0

    def set_text(self, text):
        if type(text) == str:
            text = [text]

        if len(text) != len(self._text_label_shapes):
            # If number of lines change, we must redraw completely to align to new size.
            self._text = text
            self.invalidate()
        else:
            for n in range(len(text)):
                self._text_label_shapes[n].text = text[n]
            self._text = text
            self._readjust_box_width()

    def set_text_color(self, color):
        if isinstance(color, collections.abc.Sized):
            if len(color) != len(self._text):
                carb.log_error(f"An array was provided for text color, but the lengths of the arrays did not match. Text: {self._text} Colors: {color}")
                return

        for n in range(len(self._text_label_shapes)):
            if isinstance(color, collections.abc.Sized):
                self._text_label_shapes[n].color = color[n]
            else:
                self._text_label_shapes[n].color = color
        self._text_color = color

    def set_text_font_size(self, font_size):
        if isinstance(font_size, collections.abc.Sized):
            if len(font_size) != len(self._text):
                carb.log_error(f"An array was provided for text font size, but the lengths of the arrays did not match. Text: {self._text} Colors: {font_size}")
                return
        self._text_font_size = font_size
        self.invalidate()

    # Updates the text at a specific index.
    def set_text_entry_text(self, index, text):
        if index >= len(self._text):
            carb.log_error(f"Set text entry text out of range. Text array length: {len(self._text)} Index: {index} Attempted to insert: {text}")

        self._text[index] = text

        # We need this check since the shapes may not have been created yet.
        if len(self._text_label_shapes) > index:
            self._text_label_shapes[index].text = text

        self._readjust_box_width()

    # Updates the text color at a specific index.
    def set_text_entry_color(self, index, color):
        if index >= len(self._text):
            carb.log_error(f"Set text color entry out of range. Text array length: {len(self._text)} Index: {index} Attempted to insert: {color}")
        if not isinstance(self._text_color, collections.abc.Sized):
            # If our text color was not 
            self._text_color = [self._text_color] * len(self._text)
        
        self._text_color[index] = color

        # We need this check since the shapes may not have been created yet.
        if len(self._text_label_shapes) > index:
            self._text_label_shapes[index].color = color

    def destroy(self):
        self._text = None
        self.clear()

    def __del__(self):
        self.destroy()
