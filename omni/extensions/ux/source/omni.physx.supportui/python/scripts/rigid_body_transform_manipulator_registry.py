# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import weakref
from omni.kit.viewport.registry import RegisterScene
from .rigid_body_transform_manipulator import RigidBodyTransformManipulator
from omni.kit.manipulator.prim.core.reference_prim_marker import ReferencePrimMarker


class RigidBodyTransformManipulatorScene:
    def __init__(self, desc: dict):
        usd_context_name = desc.get("usd_context_name")
        self.__transform_manip = RigidBodyTransformManipulator(
            usd_context_name=usd_context_name, viewport_api=desc.get("viewport_api")
        )
        self.__reference_prim_marker = ReferencePrimMarker(
            usd_context_name=usd_context_name, manipulator_model=weakref.proxy(self.__transform_manip.model)
        )

    def destroy(self):
        if self.__transform_manip:
            self.__transform_manip.destroy()
            self.__transform_manip = None

        if self.__reference_prim_marker:
            self.__reference_prim_marker.destroy()
            self.__reference_prim_marker = None

    @property
    def visible(self):
        return True

    @visible.setter
    def visible(self, value):
        pass

    @property
    def categories(self):
        return ("rigid body manipulator")

    @property
    def name(self):
        return "Rigid Body Transform"


class TransformManipulatorRegistry:
    def __init__(self):
        self._scene = RegisterScene(RigidBodyTransformManipulatorScene, "omni.physxsupportui")

    def __del__(self):
        self.destroy()

    def destroy(self):
        self._scene = None
