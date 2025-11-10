# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui

from .. helpers import UI

# constants for Label & InputWidget (button, drag, ...) pair
DEFAULT_LABEL_WIDTH = 110
DEFAULT_LABEL_TOOLTIP_WIDTH = 300
DEFAULT_INPUT_WIDGET_WITH = 100
DEFAULT_WIDGET_WIDTH_LIST = [omni.ui.Pixel(DEFAULT_LABEL_WIDTH), omni.ui.Pixel(DEFAULT_INPUT_WIDGET_WITH)]

DEFAULT_CONTROL_BUTTON_WIDTH = 100
DEFAULT_CONTROL_BUTTONS_SPACING_TOP = 2 * UI.DEFAULT_SPACING_V
