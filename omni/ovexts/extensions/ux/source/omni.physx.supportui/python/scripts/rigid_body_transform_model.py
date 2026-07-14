# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from typing import List, Sequence
import carb.profiler
import omni.kit.app
from omni.kit.manipulator.prim.core.model import PrimTransformModel, OpFlag
from pxr import Sdf
from usdrt import Gf
from omni.ui import scene as sc
import omni.physxsupportui.bindings._physxSupportUi as pxsupportui
from omni.physxsupportui.bindings._physxSupportUi import acquire_rigid_body_manipulator, release_rigid_body_manipulator

class RigidBodyTransformModel(PrimTransformModel):
    def __init__(self, usd_context_name: str = ""):
        self._settings = carb.settings.get_settings()
        self._logging_enabled = self._settings.get_as_bool(pxsupportui.SETTINGS_LOGGING_ENABLED)

        self._settings_subs = []
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_LOGGING_ENABLED,
            self._enable_logging_setting_changed
        ))

        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_MANIP_MODE_ALLOW_RIGID_BODY_TRAVERSAL,
            self._manip_mode_allow_rigid_body_traversal_setting_changed
        ))
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_MANIP_MODE_ALLOW_ROT_WHILE_TRANSLATING,
            self._manip_mode_allow_rot_while_translating_setting_changed
        ))
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_MANIP_MODE_ALLOW_TRAN_ON_OTHER_AXES_WHILE_TRANSLATING,
            self._manip_mode_allow_tran_on_other_axes_while_translating_setting_changed
        ))
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_MANIP_MODE_ALLOW_TRAN_WHILE_ROTATING,
            self._manip_mode_allow_tran_while_rotating_setting_changed
        ))
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_MANIP_MODE_ALLOW_ROT_ON_OTHER_AXES_WHILE_ROTATING,
            self._manip_mode_allow_rot_on_other_axes_while_rotating_setting_changed
        ))

        self._manip_mode_allow_rigid_body_traversal = (
            self._settings.get_as_bool(pxsupportui.SETTINGS_MANIP_MODE_ALLOW_RIGID_BODY_TRAVERSAL)
        )
        self._manip_mode_allow_rot_while_translating = (
            self._settings.get_as_bool(pxsupportui.SETTINGS_MANIP_MODE_ALLOW_ROT_WHILE_TRANSLATING)
        )
        self._manip_mode_allow_tran_on_other_axes_while_translating = (
            self._settings.get_as_bool(pxsupportui.SETTINGS_MANIP_MODE_ALLOW_TRAN_ON_OTHER_AXES_WHILE_TRANSLATING)
        )
        self._manip_mode_allow_tran_while_rotating = (
            self._settings.get_as_bool(pxsupportui.SETTINGS_MANIP_MODE_ALLOW_TRAN_WHILE_ROTATING)
        )
        self._manip_mode_allow_rot_on_other_axes_while_rotating = (
            self._settings.get_as_bool(pxsupportui.SETTINGS_MANIP_MODE_ALLOW_ROT_ON_OTHER_AXES_WHILE_ROTATING)
        )

        self._manipulator_helper = None

        self.rigid_body_paths = set[int]()  # Set of rigid body paths as integers (converted from SdfPath with PhysicsSchemaTools.sdfPathToInt)

        super().__init__(usd_context_name)

    def __del__(self):
        self.destroy()

    def destroy(self):
        self._settings_subs = []
        self._settings = None
        release_rigid_body_manipulator(self._manipulator_helper)
        self._manipulator_helper = None
        super().destroy()

    def _enable_logging_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._logging_enabled = self._settings.get_as_bool(pxsupportui.SETTINGS_LOGGING_ENABLED)

    def _manip_mode_allow_rigid_body_traversal_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._manip_mode_allow_rigid_body_traversal = (
                self._settings.get_as_bool(pxsupportui.SETTINGS_MANIP_MODE_ALLOW_RIGID_BODY_TRAVERSAL)
            )

    def _manip_mode_allow_rot_while_translating_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._manip_mode_allow_rot_while_translating = (
                self._settings.get_as_bool(pxsupportui.SETTINGS_MANIP_MODE_ALLOW_ROT_WHILE_TRANSLATING)
            )

    def _manip_mode_allow_tran_on_other_axes_while_translating_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._manip_mode_allow_tran_on_other_axes_while_translating = (
                self._settings.get_as_bool(pxsupportui.SETTINGS_MANIP_MODE_ALLOW_TRAN_ON_OTHER_AXES_WHILE_TRANSLATING)
            )

    def _manip_mode_allow_tran_while_rotating_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._manip_mode_allow_tran_while_rotating = (
                self._settings.get_as_bool(pxsupportui.SETTINGS_MANIP_MODE_ALLOW_TRAN_WHILE_ROTATING)
            )

    def _manip_mode_allow_rot_on_other_axes_while_rotating_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._manip_mode_allow_rot_on_other_axes_while_rotating = (
                self._settings.get_as_bool(pxsupportui.SETTINGS_MANIP_MODE_ALLOW_ROT_ON_OTHER_AXES_WHILE_ROTATING)
            )

    def _get_manipulator_helper(self):
        if self._manipulator_helper is None:
            self._manipulator_helper = acquire_rigid_body_manipulator()
        return self._manipulator_helper

    def on_began(self, payload):
        if self._logging_enabled:
            carb.log_info("RigidBodyTransformModel.on_began")
        for path in self.rigid_body_paths:
            self._get_manipulator_helper().manipulation_began(path)
        super().on_began(payload)

    def on_changed(self, payload):
        if self._logging_enabled:
            carb.log_info("RigidBodyTransformModel.on_changed")

        super().on_changed(payload)

    def on_ended(self, payload):
        if self._logging_enabled:
            carb.log_info("RigidBodyTransformModel.on_ended")

        for path in self.rigid_body_paths:
            self._get_manipulator_helper().manipulation_ended(path)

        TRANSFORM_GIZMO_IS_USING = "/app/transform/gizmoIsUsing"
        if self.custom_manipulator_enabled:
            self._settings.set(TRANSFORM_GIZMO_IS_USING, False)

        # if the manipulator was locked to orientation or translation,
        # refresh it on_ended so the transform is up to date
        mode = self._get_transform_mode_for_current_op()
        if self._should_keep_manipulator_orientation_unchanged(
            mode
        ) or self._should_keep_manipulator_translation_unchanged(mode):
            # set editing op to None AFTER _should_keep_manipulator_*_unchanged but
            # BEFORE self._update_transform_from_prims
            self._current_editing_op = None
            if self._update_transform_from_prims():
                self._item_changed(self._transform_item)

        self._current_editing_op = None

    def on_canceled(self, payload):
        if self._logging_enabled:
            carb.log_info("RigidBodyTransformModel.on_canceled")
        for path in self.rigid_body_paths:
            self._get_manipulator_helper().manipulation_ended(path)
        super().on_canceled(payload)

    def widget_enabled(self):
        if self._logging_enabled:
            carb.log_info("RigidBodyTransformModel.widget_enabled")
        super().widget_enabled()

    def widget_disabled(self):
        if self._logging_enabled:
            carb.log_info("RigidBodyTransformModel.widget_disabled")
        super().widget_disabled()

    def on_selection_changed(self, selection: List[Sdf.Path]):
        if self._logging_enabled:
            carb.log_info("RigidBodyTransformModel._on_selection_changed")
        return super().on_selection_changed(selection)

    def set_floats(self, item: sc.AbstractManipulatorItem, value: Sequence[float]):
        if self._logging_enabled:
            carb.log_info("RigidBodyTransformModel.set_floats")
        super().set_floats(item, value)

    @carb.profiler.profile
    def _transform_selected_prims(
            self,
            new_manipulator_transform: Gf.Matrix4d,
            old_manipulator_transform_no_scale: Gf.Matrix4d,
            old_manipulator_scale: Gf.Vec3d,
            dirty_ops: OpFlag,
    ):
        if self._logging_enabled:
            carb.log_info("RigidBodyTransformModel._transform_selected_prims")

        # NOTE: this intentionally differs from normal gizmo manipulation where rotation can lead to translate.
        should_update_translate = dirty_ops == OpFlag.TRANSLATE
        should_update_rotate = dirty_ops == OpFlag.ROTATE

        old_manipulator_scale_mtx = Gf.Matrix4d(1.0)
        old_manipulator_scale_mtx.SetScale(old_manipulator_scale)
        old_manipulator_transform_inv = (old_manipulator_scale_mtx * old_manipulator_transform_no_scale).GetInverse()

        delta_transform = old_manipulator_transform_inv * new_manipulator_transform
        delta_translate = carb.Float3(*delta_transform.ExtractTranslation())
        delta_rot_q = delta_transform.ExtractRotationQuat()
        delta_rot_q_img = delta_rot_q.GetImaginary()
        delta_rot_q = carb.Float4(
                                delta_rot_q_img[0],
                                delta_rot_q_img[1],
                                delta_rot_q_img[2],
                                delta_rot_q.GetReal())
        pivot_transform = self._calculate_transform_from_prim()

        if pivot_transform:
            pivot_world_position = carb.Float3(*pivot_transform[12:15])
        else:
            pivot_world_position = carb.Float3(*new_manipulator_transform.ExtractTranslation())

        for rigid_body_path in self.rigid_body_paths:

            if should_update_translate:
                self._get_manipulator_helper().move(
                    rigid_body_path,
                    delta_translate,
                    not self._manip_mode_allow_rot_while_translating,
                    not self._manip_mode_allow_tran_on_other_axes_while_translating
                )

            if should_update_rotate:
                self._get_manipulator_helper().rotate(
                    rigid_body_path,
                    pivot_world_position,
                    delta_rot_q,
                    not self._manip_mode_allow_tran_while_rotating,
                    not self._manip_mode_allow_rot_on_other_axes_while_rotating
                )

    def _transform_all_selected_prims_to_manipulator_pivot(
        self,
        new_manipulator_transform: Gf.Matrix4d,
        dirty_ops: OpFlag,
    ):
        if self._logging_enabled:
            carb.log_info("RigidBodyTransformModel._transform_all_selected_prims_to_manipulator_pivot")
        super()._transform_all_selected_prims_to_manipulator_pivot(new_manipulator_transform, dirty_ops)
