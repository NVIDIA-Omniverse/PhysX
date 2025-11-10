# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from typing import List, Sequence
import carb.profiler
import omni.timeline
from omni.kit.manipulator.prim.core.model import PrimTransformModel, OpFlag
from pxr import Sdf, UsdGeom, UsdPhysics, PhysicsSchemaTools
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

    def get_rigid_body(self, prim):
        while (prim and not prim.IsPseudoRoot()):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                return prim
            prim = prim.GetParent() if self._manip_mode_allow_rigid_body_traversal else None
        return None

    def is_rigid_body_xform(self, prim):
        if (prim.IsA(UsdGeom.Xformable)):
            rigid_body = self.get_rigid_body(prim)
            if rigid_body is not None and not UsdPhysics.RigidBodyAPI(rigid_body).GetKinematicEnabledAttr().Get():
                return True
        return False

    def on_began(self, payload):
        if self._logging_enabled:
            carb.log_info("RigidBodyTransformModel.on_began")
        for path in self._xformable_prim_paths:
            prim = self._data_accessor_selector.get_prim_at_path(path)
            rigid_body = self.get_rigid_body(prim)
            if rigid_body:
                rigid_body_path = rigid_body.GetPath()
                self._get_manipulator_helper().manipulation_began(PhysicsSchemaTools.sdfPathToInt(rigid_body_path))
        super().on_began(payload)

    def on_changed(self, payload):
        if self._logging_enabled:
            carb.log_info("RigidBodyTransformModel.on_changed")
        super().on_changed(payload)

    def on_ended(self, payload):
        if self._logging_enabled:
            carb.log_info("RigidBodyTransformModel.on_ended")
        for path in self._xformable_prim_paths:
            prim = self._data_accessor_selector.get_prim_at_path(path)
            rigid_body = self.get_rigid_body(prim)
            rigid_body_path = rigid_body.GetPath()
            self._get_manipulator_helper().manipulation_ended(PhysicsSchemaTools.sdfPathToInt(rigid_body_path))

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
        for path in self._xformable_prim_paths:
            self._get_manipulator_helper().manipulation_ended(PhysicsSchemaTools.sdfPathToInt(path))
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

        self._xformable_prim_paths.clear()
        self._xformable_prim_paths_set.clear()
        self._xformable_prim_paths_prefix_set.clear()
        self._consolidated_xformable_prim_paths.clear()
        self._pivot_prim = None

        for path in selection:
            sdf_path = Sdf.Path(path)
            prim = self._data_accessor_selector.get_prim_at_path(sdf_path)
            if self.is_rigid_body_xform(prim):
                self._xformable_prim_paths.append(sdf_path)

        if self._xformable_prim_paths:
            # Make a sorted list so parents always appears before child
            self._xformable_prim_paths_sorted = self._xformable_prim_paths.copy()
            self._xformable_prim_paths_sorted.sort()

            # Find the most recently selected valid xformable prim as the pivot prim where the transform gizmo is located at.
            self._pivot_prim = self._data_accessor_selector.get_prim_at_path(self._xformable_prim_paths[-1])

            # Get least common prims ancestors.
            # We do this so that if one selected prim is a descendant of other selected prim, the descendant prim won't be
            # transformed twice.
            self._consolidated_xformable_prim_paths = Sdf.Path.RemoveDescendentPaths(self._xformable_prim_paths)

        self._xformable_prim_paths_set.update(self._xformable_prim_paths)
        for path in self._xformable_prim_paths_set:
            self._xformable_prim_paths_prefix_set.update(path.GetPrefixes())

        if self._update_transform_from_prims():
            self._item_changed(self._transform_item)

        # Happens when host widget is already enabled and first selection in a new stage
        if self._enabled_hosting_widget_count > 0 and self._stage_listener is None:
            self._stage_listener = self._data_accessor_selector.setup_update_callback()

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

        paths = []
        new_translations = []
        new_rotation_eulers = []
        new_rotation_orders = []
        new_scales = []

        self._data_accessor_selector.clear_xform_cache()

        # NOTE: this intentionally differs from normal gizmo manipulation where rotation can lead to translate.
        should_update_translate = dirty_ops == OpFlag.TRANSLATE
        should_update_rotate = dirty_ops == OpFlag.ROTATE
        should_update_scale = dirty_ops == OpFlag.SCALE

        old_manipulator_scale_mtx = Gf.Matrix4d(1.0)
        old_manipulator_scale_mtx.SetScale(old_manipulator_scale)
        old_manipulator_transform_inv = (old_manipulator_scale_mtx * old_manipulator_transform_no_scale).GetInverse()

        delta_transform = old_manipulator_transform_inv * new_manipulator_transform
        delta_translate = delta_transform.ExtractTranslation()
        new_rot = delta_transform.ExtractRotation()
        delta_rot_q = new_rot.GetQuat()
        pivot_transform = self._calculate_transform_from_prim()
        if pivot_transform:
            pivot_world_position = Gf.Vec3d(*pivot_transform[12:15])
        else:
            pivot_world_position = new_manipulator_transform.ExtractTranslation()

        # Only apply forces to each rigid body once.
        rigid_bodies_manipulated = set()
        for path in self._consolidated_xformable_prim_paths:
            if self._custom_manipulator_enabled and self._should_skip_custom_manipulator_path(path.pathString):
                continue

            selected_prim = self._data_accessor_selector.get_prim_at_path(path)
            # We check whether path is in consolidated_xformable_prim_data_curr because it may have not made it the dictionary if an error occured
            if not selected_prim or path not in self.consolidated_xformable_prim_data_curr:
                continue

            (s, r, ro, t, selected_pivot) = self.consolidated_xformable_prim_data_curr[path]

            rigid_body = self.get_rigid_body(selected_prim)
            rigid_body_path = rigid_body.GetPath()

            def physx_move(tran_delta: Gf.Vec3d):
                if rigid_body in rigid_bodies_manipulated:
                    return

                return self._get_manipulator_helper().move(
                    PhysicsSchemaTools.sdfPathToInt(rigid_body_path),
                    carb.Float3(*tran_delta),
                    not self._manip_mode_allow_rot_while_translating,
                    not self._manip_mode_allow_tran_on_other_axes_while_translating
                )

            if should_update_translate:
                translation = delta_translate
                physx_move(translation)

            def physx_rotate(target_rot: Gf.Quatd):
                if rigid_body in rigid_bodies_manipulated:
                    return
                imag = target_rot.GetImaginary()
                return self._get_manipulator_helper().rotate(
                    PhysicsSchemaTools.sdfPathToInt(rigid_body_path),
                    carb.Float3(*pivot_world_position),
                    carb.Float4(imag[0], imag[1], imag[2], target_rot.GetReal()),
                    not self._manip_mode_allow_tran_while_rotating,
                    not self._manip_mode_allow_rot_on_other_axes_while_rotating
                )

            if should_update_rotate:
                new_rot = delta_transform.ExtractRotation()
                axes = [Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis()]
                rotation = new_rot.Decompose(axes[ro[0]], axes[ro[1]], axes[ro[2]])
                physx_rotate(delta_rot_q)

            translation = translation if should_update_translate else t
            rotation = rotation if should_update_rotate else r
            scale = s

            paths.append(path.pathString)
            new_translations += [translation[0], translation[1], translation[2]]
            new_rotation_eulers += [rotation[0], rotation[1], rotation[2]]
            new_rotation_orders += [ro[0], ro[1], ro[2]]
            new_scales += [scale[0], scale[1], scale[2]]

            xform_tuple = (scale, rotation, ro, translation, selected_pivot)
            self.consolidated_xformable_prim_data_curr[path] = xform_tuple
            self.all_xformable_prim_data_curr[path] = xform_tuple

            rigid_bodies_manipulated.add(rigid_body)

        # TODO FIXME undo/redo
        # self._ignore_xform_data_change = True
        # omni.kit.commands.create(
        #     "TransformMultiPrimsSRTCpp",
        #     count=len(paths),
        #     no_undo=True,
        #     paths=paths,
        #     new_translations=new_translations,
        #     new_rotation_eulers=new_rotation_eulers,
        #     new_rotation_orders=new_rotation_orders,
        #     new_scales=new_scales,
        #     usd_context_name=self._usd_context_name,
        #     time_code=self._get_current_time_code().GetValue(),
        # ).do()
        # self._ignore_xform_data_change = False

    def _transform_all_selected_prims_to_manipulator_pivot(
        self,
        new_manipulator_transform: Gf.Matrix4d,
        dirty_ops: OpFlag,
    ):
        if self._logging_enabled:
            carb.log_info("RigidBodyTransformModel._transform_all_selected_prims_to_manipulator_pivot")
        super()._transform_all_selected_prims_to_manipulator_pivot(new_manipulator_transform, dirty_ops)
