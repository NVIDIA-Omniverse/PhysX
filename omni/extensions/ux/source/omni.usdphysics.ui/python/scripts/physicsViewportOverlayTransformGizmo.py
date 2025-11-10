# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from typing import List, Union, Sequence
from pxr import Usd, Sdf
from usdrt import Gf
from .physicsViewportOverlayShared import *
import omni.kit.app
import omni.kit.commands
import omni.usd
from omni.ui_scene import scene as sc
from omni.kit.manipulator.prim.core.model import OpFlag, PrimTransformModel
from omni.kit.manipulator.tool.snap import SnapProviderManager
from omni.kit.manipulator.tool.snap import settings_constants as snap_c
from omni.kit.manipulator.transform.settings_constants import c
from omni.kit.manipulator.transform import Constants as transform_c
from omni.kit.manipulator.transform.settings_listener import OpSettingsListener, SnapSettingsListener
from omni.kit.manipulator.transform.types import Operation
from omni.kit.manipulator.prim.core.prim_transform_manipulator import PrimTransformManipulator
from typing import Callable, Any

class PhysicsTransformGizmoModel(PrimTransformModel):
    # Attaches to the gizmo for callbacks and determining the location of the gizmo.
    def attach(self, 
                    link : Any, 
                    get_position : Callable[[], Gf.Vec3d], 
                    get_rotation : Callable[[], Gf.Quatd], 
                    on_began : (Callable[[], None] | None ) = None, 
                    on_changed : (Callable[[], None] | None ) = None, 
                    on_ended  : (Callable[[], None] | None ) = None, 
                    on_apply_transform_delta : (Callable[[Gf.Transform], None] | None ) = None):

        self._attachments.append(PhysicsTransformGizmoModel.Attachment(link, get_position, get_rotation, on_began, on_changed, on_ended, on_apply_transform_delta))
        self.refresh()

    def detach(self, link):
        for attachment in self._attachments:
            if attachment.link is link:
                self._attachments.remove(attachment)
        self.refresh()
        
    class Item:
        # This is actually defined by the parent class and the names should not be changed. They are here only for convenience.
        POSITION                = "translate"
        XFORM                   = "transform"
        GIZMO_XFORM             = "transform_manipulator"
        GIZMO_XFORM_NO_SCALE    = "no_scale_transform_manipulator"
    class Attachment():
        def __init__(self, 
                    link, 
                    get_position : Callable[[], Gf.Vec3d], 
                    get_rotation : Callable[[], Gf.Quatd], 
                    on_began : (Callable[[], None] | None ) = None, 
                    on_changed : (Callable[[], None] | None ) = None, 
                    on_ended  : (Callable[[], None] | None ) = None, 
                    on_apply_transform_delta : (Callable[[Gf.Transform], None] | None ) = None):

            self.link = link
            self._get_position = get_position
            self._get_rotation = get_rotation
            self._on_began = on_began
            self._on_changed = on_changed
            self._on_ended = on_ended
            self._on_apply_transform_delta = on_apply_transform_delta

        def get_position(self) -> Gf.Vec3d:
            return self._get_position()

        def get_rotation(self) -> Gf.Quatd:
            return self._get_rotation()

        def on_began(self):
            if self._on_began is not None:
                self._on_began()

        def on_changed(self):
            if self._on_changed is not None:
                self._on_changed()

        def on_ended(self):
            if self._on_ended is not None:
                self._on_ended()

        def on_apply_transform_delta(self, transform_delta: Gf.Transform):
            if self._on_apply_transform_delta is not None:
                self._on_apply_transform_delta(transform_delta)

    def __init__(self, viewport_overlay_manager, usd_context_name: str = "", viewport_api=None):
        self._viewport_overlay_manager = viewport_overlay_manager

        self._current_manipulation_transform = None
        self._attachments : list[PhysicsTransformGizmoModel.Attachment] = []
        super().__init__(usd_context_name, viewport_api)

    def get_transform_gizmo_matrix(self) -> Gf.Matrix4d:
        if self._current_manipulation_transform is not None:
            return self._current_manipulation_transform.GetMatrix()

        bounds = Gf.Range3d()
        for attachment in self._attachments:
            bounds.UnionWith(attachment.get_position())

        transform = Gf.Matrix4d()
        if self._get_transform_mode_for_current_op() != transform_c.TRANSFORM_MODE_GLOBAL and len(self._attachments) == 1:
            quat = self._attachments[0].get_rotation()
            transform.SetRotate(quat)

        if not bounds.IsEmpty():
            transform = transform.SetTranslateOnly(bounds.GetMidpoint())
        else:
            transform = transform.SetIdentity()

        return transform


    def _calculate_transform_from_prim(self) -> tuple[float]:
        return to_tuple(self.get_transform_gizmo_matrix())

    def on_began(self, payload):
        item = payload.changing_item
        self._current_editing_op = item.operation
        self._current_manipulation_transform = Gf.Transform(self.get_transform_gizmo_matrix())

        for attachment in self._attachments:
            attachment.on_began()

    def on_changed(self, payload):
        for attachment in self._attachments:
            attachment.on_changed()

    def on_ended(self, payload):
        self._current_editing_op = None
        self._current_manipulation_transform = None

        for attachment in self._attachments:
            attachment.on_ended()

    def set_floats(self, item: sc.AbstractManipulatorItem, value: Sequence[float]):
        if isinstance(item, omni.kit.manipulator.transform.model.AbstractTransformManipulatorModel.OperationItem):
            transform_delta = None
            if item.operation == Operation.TRANSLATE_DELTA:
                transform_delta = Gf.Transform()
                transform_delta.SetTranslation(Gf.Vec3d(*value))

                if self._current_manipulation_transform is not None:
                    self._current_manipulation_transform *= transform_delta
                else:
                    self._current_manipulation_transform = transform_delta

            elif item.operation == Operation.ROTATE_DELTA:
                rotate_q = Gf.Quatd(value[3], value[0], value[1], value[2])

                if self._get_transform_mode_for_current_op() == c.TRANSFORM_MODE_LOCAL:
                    current_rotation_matrix = Gf.Matrix4d(self.get_transform_gizmo_matrix())
                    current_rotation_matrix.RemoveScaleShear()
                    current_rotation_matrix.Orthonormalize()
                    current_rotation = current_rotation_matrix.ExtractRotationQuat()
                    rotate_q = current_rotation * rotate_q * current_rotation.GetInverse()

                transform_delta = Gf.Transform()
                transform_delta.SetRotation(Gf.Rotation(rotate_q))

            elif item.operation == Operation.SCALE_DELTA:
                # Currently we do not use scaling.
                pass
            else:
                pass
            if transform_delta is not None:
                for attachment in self._attachments:
                    attachment.on_apply_transform_delta(transform_delta)

                self.refresh()

    def get_as_floats(self, item: sc.AbstractManipulatorItem) -> Sequence[float]:
        if (item == self.get_item(PhysicsTransformGizmoModel.Item.GIZMO_XFORM_NO_SCALE) or item == self.get_item(PhysicsTransformGizmoModel.Item.GIZMO_XFORM) or
            item == self.get_item(PhysicsTransformGizmoModel.Item.XFORM)):
            return to_tuple(self.get_transform_gizmo_matrix())
        elif item == self.get_item(PhysicsTransformGizmoModel.Item.POSITION):
            value = self.get_transform_gizmo_matrix()
            return value.ExtractTranslation()
        else:
            return super().get_as_floats(item)

    def destroy(self):
        self._viewport_overlay_manager = None
        self._current_manipulation_transform = None
        self._attachments = []
        super().destroy()

    def __del__(self):
        if self._viewport_overlay_manager is not None:
            self.destroy()

    def _on_stage_opened(self):
        pass

    def widget_enabled(self):
        pass 

    def widget_disabled(self):
        pass

    def get_operation(self) -> Operation:
        if not self.enabled:
            return Operation.NONE
        else:
            return super().get_operation()

    @property
    def enabled(self):
        return (len(self._attachments) > 0)

    def refresh(self):
        self._item_changed(self.get_item(PhysicsTransformGizmoModel.Item.GIZMO_XFORM_NO_SCALE))


class PhysicsTransformGizmo(PrimTransformManipulator):
    def __init__(self, manager, usd_context_name: str = "", viewport_api=None, name="omni.usdphysicsui", size: float = 1.0):
        self._model = PhysicsTransformGizmoModel(manager, usd_context_name, viewport_api)
        super().__init__(usd_context_name, viewport_api, name, self._model, size)

    # Override to prevent triggering default PrimTransformManipulator behavior.
    def on_selection_changed(self, stage: Usd.Stage, selection: Union[List[Sdf.Path], None], *args, **kwargs) -> bool:  
        return True

    def destroy(self):
        if self._model is not None:
            self._model.destroy()
        self._model = None
        super().destroy()
