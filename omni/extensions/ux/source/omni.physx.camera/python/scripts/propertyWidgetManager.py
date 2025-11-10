# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from .propertyWidgetCamera import CameraInvisibleWidget


class PropertyWidgetManager:
    scheme = "prim"

    def __init__(self, physxCameraInterface):
        self._physxCameraInterface = physxCameraInterface

    def set_up(self):
        self._register_widgets()

    def tear_down(self):
        self._unregister_widgets()
        self._physxCameraInterface = None

    def _register_widgets(self):
        self._invisible_widget = CameraInvisibleWidget()
        
    def _unregister_widgets(self):
        self._invisible_widget.clean()
        self._invisible_widget = None
