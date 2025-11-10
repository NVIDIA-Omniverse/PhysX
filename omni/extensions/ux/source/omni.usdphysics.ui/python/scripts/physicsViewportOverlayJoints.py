# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from collections.abc import Iterable
import omni.usd
import omni.ui
from omni.ui import scene as sc
from omni.ui import color as cl
import omni.ui as ui
import omni.timeline
import omni.timeline
import omni.kit.commands
from .physicsViewportOverlayShared import *
from .physicsViewportOverlayPrimInfo import PhysicsUIViewportPrim
from .physicsViewportOverlayInfobox import PhysicsUIFloatingInfobox
from enum import auto, IntEnum
from pxr import Usd, UsdPhysics, UsdGeom, Sdf
from usdrt import Usd as UsdRt, Rt as UsdRtGeom, Gf
from pathlib import Path
from omni.usdphysics.scripts import jointUtils


class JointConnectionAlignClickGesture(PhysicsUIClickGesture):
    def __init__(self, manipulator, target : int):
        self.manipulator : JointEditManipulator = None  # Here just to help the linter with the type.
        self._target = target
        super().__init__(manipulator, manipulator.get_ui_toggle_group(JointEditManipulator.ManipulationType.ALIGN, target))

    def on_ended(self):
        super().on_ended()

        self.manipulator._make_target_offset_valid(self._target)


class JointSwapClickGesture(PhysicsUIClickGesture):
    def __init__(self, manipulator, toggle_group):
        super().__init__(manipulator, toggle_group)

    def on_ended(self):
        super().on_ended()
        target = self.manipulator.get_targets()[0]
        target.attachments_swap()


class JointEditManipulator(PhysicsUIViewportPrim):

    infobox_title = "JOINT PROPERTIES"

    @classmethod
    def get_is_prim_valid_target(cls, prim: Usd.Prim) -> bool:
        if not super().get_is_prim_valid_target(prim):
            return False
        return prim.IsA(UsdPhysics.Joint)

    VISUAL_DELTA_TOLERANCE = 0.01
    ATTACHMENT_ALIGNMENT_POSITION_TOLERANCE = 0.001
    ATTACHMENT_ALIGNMENT_ORIENTATION_TOLERANCE = 1.0

    class ManipulationType(IntEnum):
        ROTATE = 0
        TRANSLATE = auto()
        DISTANCE = auto()
        ALIGN = auto()

    class Limit(IntEnum):
        ROTATION_X = 0
        ROTATION_Y = auto()
        ROTATION_Z = auto()
        TRANSLATION_X = auto()
        TRANSLATION_Y = auto()
        TRANSLATION_Z = auto()
        DISTANCE = auto()

    class Style:
        position_0_color = cl("#66FFFF")
        color_base = cl("#66EE66")
        color_base_shaded = cl("#44BB44")
        error_color = cl("#FF4444")
        error_color_shaded = cl("#AA3333")
        position_overlap_color = cl("#66FF66")
        position_circle_radius = 20.0
        position_line_width = 2.5
        limit_color = cl("#00EFEF")
        limit_color_shaded = cl("#00BFBF")
        limit_line_width = 2.0
        color_axis = [cl("#FF3F3FFF"), cl("#2FDF2FFF"), cl("#4F6FFFFF")]        # Roughly matches luma values between channels.
        color_axis_shaded = [cl("#D93A3AFF"), cl("#1DBC1DFF"), cl("#3A57D9FF")] # Darker variant.

    class Attachment():
        def __init__(self, manipulator, index):
            self._index = index
            self.manipulator = manipulator
            self.prim : Usd.Prim = None
            self.prim_usdrt : UsdRt.Prim = None
            self.xformable : UsdGeom.Xformable = None
            self.xformable_usdrt : UsdRtGeom.Xformable = None
            self.usdrt_xform_query : UsdRtXformQuery = None
            joint = self.manipulator._joint
            self._body_rel_attr : Usd.Relationship = joint.GetBody0Rel() if index == 0 else joint.GetBody1Rel()
            self._body_offset_position_attr : Usd.Attribute = joint.GetLocalPos0Attr() if index == 0 else joint.GetLocalPos1Attr()
            self._body_offset_rotation_attr : Usd.Attribute = joint.GetLocalRot0Attr() if index == 0 else joint.GetLocalRot1Attr()
            self._body_offset_position_manipulation_undo = None
            self._body_offset_rotation_manipulation_undo = None

            self._ui_transform_world = None
            self._ui_marker_arc = None
            self._ui_transform_bbox = None

            if index == 1:
                self._ui_transform_bbox_aligned = None
                self._ui_transform_bbox_edit_gesture = None
                self._ui_transform_bbox_edit = None

            self._ui_align_arrow_transform : sc.Transform = None

            self.prim_xform : Gf.Matrix4d = None
            self.prim_bounds : Gf.Range3d = None

            self.offset_translation_matrix : Gf.Matrix4d = None
            self.offset_rotation_matrix : Gf.Matrix4d = None
            self.joint_xform : Gf.Matrix4d = None
            self.bounds_matrix : Gf.Matrix4d = None

        def _refresh_ui_position(self):
            if self._ui_transform_world is not None:
                self._ui_transform_world.transform = to_ui_scene(self.joint_xform)

        def _refresh_bounds(self):
            # Preliminary approach until ComputeUntransformedBound works better. Only use it if we are ourselves boundable, otherwise derive from children. (OM-100633)
            if self.prim is None:
                body_bounds = None
            elif self.prim.IsA(UsdGeom.Boundable):
                body_bounds_attrib = self.prim.GetAttribute(UsdGeom.Tokens.extent)
                if body_bounds_attrib and body_bounds_attrib.HasAuthoredValue():
                    body_bounds = body_bounds_attrib.Get()
                    body_bounds_vec3d_0 = Gf.Vec3d(body_bounds[0])
                    body_bounds_vec3d_1 = Gf.Vec3d(body_bounds[1])
                    body_bounds = Gf.Range3d(body_bounds_vec3d_0, body_bounds_vec3d_1)
                else:
                    bbox = self.manipulator.bbox_cache.ComputeUntransformedBound(self.prim)
                    body_bounds = bbox.ComputeAlignedRange()
            else:
                body_bounds = None

                def process_children(prim):
                    children = prim.GetChildren()
                    for child in children:
                        if child.IsA(UsdGeom.Boundable):
                            bbox = self.manipulator.bbox_cache.ComputeRelativeBound(child, self.prim)
                            nonlocal body_bounds
                            if body_bounds is not None:
                                body_bounds = body_bounds.UnionWith(bbox.ComputeAlignedRange())
                            else:
                                body_bounds = bbox.ComputeAlignedRange()
                        else:
                            process_children(child)

                process_children(self.prim)

            if body_bounds:
                self.prim_bounds = to_usdrt(body_bounds)
            else:
                self.prim_bounds = None

        def refresh_data(self):
            simulation_active = self.manipulator.simulation_active
            if not simulation_active or self.prim_xform is None:
                targets = self._body_rel_attr.GetTargets()

                if len(targets) == 1:
                    self.prim = self.manipulator.stage.GetPrimAtPath(targets[0])
                    self.prim_usdrt = self.manipulator.stage_usdrt.GetPrimAtPath(str(targets[0])) if self.manipulator.stage_usdrt is not None else None
                else: 
                    self.prim = None
                    self.prim_usdrt = None

                if self.prim is not None and self.prim.IsA(UsdGeom.Xformable):
                    self.xformable = UsdGeom.Xformable(self.prim)
                else:
                    self.xformable = None

                if self.prim_usdrt is not None and self.prim_usdrt.IsValid():
                    self.xformable_usdrt = UsdRtGeom.Xformable(self.prim_usdrt)
                else:
                    self.xformable_usdrt = None

                self.usdrt_xform_query = UsdRtXformQuery(self.xformable_usdrt) if self.xformable_usdrt is not None else None

            if self.usdrt_xform_query is not None:
                self.prim_xform = self.usdrt_xform_query.xform
                if self.prim_xform is None:
                    self.prim_xform = (to_usdrt_d(self.manipulator.xform_cache.GetLocalToWorldTransform(self.prim)) 
                                    if self.prim is not None else Gf.Matrix4d())
            else:
                self.prim_xform = (to_usdrt_d(self.manipulator.xform_cache.GetLocalToWorldTransform(self.prim)) 
                                if self.prim is not None else Gf.Matrix4d())

            if(not simulation_active or
                self.offset_translation_matrix is None):
                self.offset_translation_matrix = Gf.Matrix4d().SetTranslate(to_usdrt_d(self._body_offset_position_attr.Get()))
                self.offset_rotation_matrix = Gf.Matrix4d().SetRotate(to_usdrt_d(self._body_offset_rotation_attr.Get()))

            self.joint_xform = self.offset_rotation_matrix * (self.offset_translation_matrix * self.prim_xform).RemoveScaleShear()

            if(not simulation_active or self.bounds_matrix is None):
                self._refresh_bounds()
                if self.prim_bounds is not None:
                    # Matrix for transforming a point from the attached prims space into the joint space.
                    prim_to_joint_transform = self.prim_xform * self.joint_xform.GetInverse()
                    self.bounds_matrix = (Gf.Matrix4d().SetScale(self.prim_bounds.GetSize()) * Gf.Matrix4d().SetTranslate(self.prim_bounds.GetMin()) * prim_to_joint_transform)
                else:
                    self.bounds_matrix = Gf.Matrix4d()

        def _make_ui_shapes(self):
            self._ui_transform_world = sc.Transform()
            with self._ui_transform_world:
                with sc.Transform(scale_to=sc.Space.SCREEN, look_at=sc.Transform.LookAt.CAMERA):
                    self._ui_marker_arc = sc.Arc(self.manipulator.Style.position_circle_radius, color=self.manipulator.Style.color_base,
                                                       wireframe=True, thickness=self.manipulator.Style.position_line_width)

                self._ui_transform_bbox = sc.Transform()
                with self._ui_transform_bbox:
                    ui_draw_corner_box(color_with_alpha(self.manipulator.Style.color_base if self._index == 1 else self.manipulator.Style.color_base_shaded, 0.75), 2.5, ((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)))

                self._ui_align_arrow_transform = sc.Transform()
                with self._ui_align_arrow_transform:
                    with ui_create_screen_scale_transform():
                        toggle_align_group = self.manipulator.get_ui_toggle_group(JointEditManipulator.ManipulationType.ALIGN, self._index)
                        toggles_align = toggle_align_group.get_manipulator_toggles(self)
                        toggles_align.clear()
                        arrow = ui_draw_arrow(self.manipulator.Style.error_color, Axis.vector_z, 20, 7.5, -20)
                        toggles_align.append(ColorToggle(arrow, COLOR_HIGHLIGHT))
                        arrow = ui_draw_arrow(self.manipulator.Style.error_color_shaded, Axis.vector_z, 0, 7.5, -20)
                        toggles_align.append(ColorToggle(arrow, COLOR_HIGHLIGHT))
                        with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0, 0, -10)):
                            with sc.Transform(look_at=sc.Transform.LookAt.CAMERA):
                                sc.Arc(12.5, color=COLOR_INVISIBLE, gesture=[JointConnectionAlignClickGesture(self.manipulator, self._index), PhysicsUIHoverGesture(self.manipulator, toggle_align_group)])

                                label_transform = sc.Transform(transform=sc.Matrix44.get_translation_matrix(55.0, -25.0, 0.0))
                                toggles_align.append(VisibilityToggle(label_transform))
                                with label_transform:
                                    sc.Label("Align to",
                                                alignment=ui.Alignment.CENTER_TOP,
                                                color=COLOR_TEXT,
                                                size=14
                                            )
                                    with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0.0, -18, 0.0)):
                                        sc.Label("Body "+str(self._index),
                                                    alignment=ui.Alignment.CENTER_TOP,
                                                    color=COLOR_TEXT,
                                                    size=14
                                                )
                                    with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0.0, -17.5, 0.0)):
                                        sc.Rectangle(60, 38, color=cl("#1E212360"))

            if self._index == 1:
                with self.manipulator._attachments[0]._ui_transform_world:
                    self._ui_transform_bbox_aligned = sc.Transform()
                    with self._ui_transform_bbox_aligned:
                        ui_draw_box(color_with_alpha(self.manipulator.Style.limit_color, 0.5), self.manipulator.Style.limit_line_width, ((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)))

                    self._ui_transform_bbox_edit_gesture = sc.Transform()
                    with self._ui_transform_bbox_edit_gesture:
                        self._ui_transform_bbox_edit = sc.Transform(visible=False)
                        with self._ui_transform_bbox_edit:
                            ui_draw_box(color_with_alpha(self.manipulator.Style.limit_color, 0.75), self.manipulator.Style.limit_line_width, ((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)))

    def get_rotation_limits(self, axis) -> list[float]:
        return self._limits[self.Limit.ROTATION_X + axis]

    def get_translation_limits(self, axis) -> list[float]:
        return self._limits[self.Limit.TRANSLATION_X + axis]

    def _joint_validator_make_current(self):
        # Make the joint validator use the current pose (and limits for the respective subtypes). Call this before validating poses.
        for n in range(2):
            self._joint_validator.attachments[n].prim_xform = to_usd(self._attachments[n].prim_xform)
            self._joint_validator.attachments[n].offset_translation = to_usd(self._attachments[n].offset_translation_matrix.ExtractTranslation())
            self._joint_validator.attachments[n].offset_rotation = to_usd(self._attachments[n].offset_rotation_matrix.ExtractRotationQuat())

    def __init__(self, viewport_overlay: PhysicsUIViewportOverlay, prim: Usd.Prim, **kwargs):
        super().__init__(viewport_overlay, prim, **kwargs)

        self._joint_validator = jointUtils.make_joint_validator(self.prim)

        self._joint : UsdPhysics.Joint = UsdPhysics.Joint(self.prim)
        self._attachments : tuple[JointEditManipulator.Attachment, JointEditManipulator.Attachment] = (JointEditManipulator.Attachment(self, 0), JointEditManipulator.Attachment(self, 1))
        self._limits : list[list[float] | None] = [None] * len(JointEditManipulator.Limit)

        self._toggle_groups = []
        for _ in range(len(self.ManipulationType) * AxisDirection.num):
            self._toggle_groups.append(PhysicsUIToggleGroup())

        self._ui_root_transform = None

        self._ui_joint_connector_line_transform = None
        self._ui_joint_connector_line = None

        self._usd_paths_tracked : set[Sdf.Path] = set()

        self.manager.get_transform_gizmo_model().attach(self, self.get_transform_gizmo_position, 
                                                          self.get_transform_gizmo_rotation, 
                                                          self.on_transform_gizmo_manipulation_began, 
                                                          None,
                                                          self.on_transform_gizmo_manipulation_ended, 
                                                          self.apply_transform_gizmo_delta)


    def get_transform_gizmo_position(self) -> Gf.Vec3d:
        if self.prim_xform is not None:
            return self.prim_xform.ExtractTranslation()
        else:
            return Gf.Vec3d()

    def get_transform_gizmo_rotation(self) -> Gf.Quatd:
        if self.prim_xform is not None:
            return self.prim_xform.ExtractRotationQuat()
        else:
            return Gf.Quatd()

    def apply_transform_gizmo_delta(self, transform_delta: Gf.Transform):
        if self.simulation_active:
            return

        transform_rotation = Gf.Matrix3d(transform_delta.GetRotation())
        transform_translation = transform_delta.GetTranslation()

        for attachment in self._attachments:
            if not attachment.xformable:
                continue

            target_joint_xform = Gf.Matrix4d(attachment.joint_xform.GetOrthonormalized().ExtractRotationMatrix() *
                                            transform_rotation,
                                            attachment.joint_xform.ExtractTranslation() + transform_translation)

            # Get this in prim local space.
            prim_local_xform = target_joint_xform * attachment.prim_xform.GetInverse()

            attachment.offset_translation_matrix = Gf.Matrix4d().SetTranslate(prim_local_xform.ExtractTranslation())
            attachment.offset_rotation_matrix = target_joint_xform * (attachment.offset_translation_matrix * attachment.prim_xform).RemoveScaleShear().GetInverse()
            attachment.joint_xform = target_joint_xform

            if attachment.prim_bounds is not None:
                # Matrix for transforming a point from the attached prims space into the joint space.
                prim_to_joint_transform = attachment.prim_xform * attachment.joint_xform.GetInverse()
                attachment.bounds_matrix = (Gf.Matrix4d().SetScale(attachment.prim_bounds.GetSize()) * Gf.Matrix4d().SetTranslate(attachment.prim_bounds.GetMin()) * prim_to_joint_transform)


            translation = to_usd(attachment.offset_translation_matrix.ExtractTranslation())
            rotation = to_usd(Gf.Quatf(attachment.offset_rotation_matrix.ExtractRotationQuat()))
            attachment._body_offset_position_attr.Set(translation)
            attachment._body_offset_rotation_attr.Set(rotation)

    def on_transform_gizmo_manipulation_began(self):
        for attachment in self._attachments:
            if attachment.xformable:
                attachment._body_offset_position_manipulation_undo = attachment._body_offset_position_attr.Get()
                attachment._body_offset_rotation_manipulation_undo = attachment._body_offset_rotation_attr.Get()

    def on_transform_gizmo_manipulation_ended(self):
        self.write_offsets_to_usd()
        for attachment in self._attachments:
            attachment._body_offset_position_manipulation_undo = None
            attachment._body_offset_rotation_manipulation_undo = None

    def _write_values_to_usd(self, properties: Iterable[Usd.Property], values: Iterable):
        if not self._prim:
            return

        omni.kit.undo.begin_group()
        for property, value in zip(properties, values):    # range(min(len(properties), len(values))):
            if type(property) is Usd.Relationship:
                omni.kit.commands.execute('SetRelationshipTargetsCommand', relationship=property, targets=value)
            else:
                if isinstance(value, Sequence):
                    _value = value[0]
                    _prev = value[1]
                else:
                    _value = value
                    _prev = property.Get() if property else None

                if property and type(_value) is not type(property.Get()):
                    # Attempt casting.
                    _value = type(property.Get())(to_usd(_value))
                omni.kit.commands.execute('ChangePropertyCommand', prop_path=property.GetPath(), value=_value, 
                                        prev=_prev)

        omni.kit.undo.end_group()

    def write_offsets_to_usd(self):
        attributes = []
        values = []

        for attachment in self._attachments:
            if attachment.xformable:
                attributes.append(attachment._body_offset_position_attr)
                if attachment._body_offset_position_manipulation_undo is not None:
                    value = (attachment.offset_translation_matrix.ExtractTranslation(), attachment._body_offset_position_manipulation_undo)
                else:
                    value = attachment.offset_translation_matrix.ExtractTranslation()
                values.append(value)

                attributes.append(attachment._body_offset_rotation_attr)
                if attachment._body_offset_rotation_manipulation_undo is not None:
                    value = (attachment.offset_rotation_matrix.ExtractRotationQuat(), attachment._body_offset_rotation_manipulation_undo)
                else:
                    value = attachment.offset_rotation_matrix.ExtractRotationQuat()
                values.append(value)

        if len(attributes) > 0:
            self._write_values_to_usd(attributes, values)

    def write_limits_to_usd(self):
        return

    @property
    def prim_xform(self) -> Gf.Matrix4d:
        return self._attachments[0].joint_xform

    def get_ui_toggle_group(self, manipulation_type, direction=0) -> PhysicsUIToggleGroup:
        return self._toggle_groups[manipulation_type * AxisDirection.num + direction]

    def get_ui_toggles(self, manipulation_type, direction=0) -> list[VisualToggle]:
        return self._toggle_groups[manipulation_type * AxisDirection.num + direction].get_manipulator_toggles(self)

    def _make_ui_shapes(self):
        super()._make_ui_shapes()

        self._ui_root_transform = sc.Transform()
        with self._ui_root_transform:
            self._attachments[0]._make_ui_shapes()
            self._attachments[1]._make_ui_shapes()
            
            with self._attachments[0]._ui_transform_world:
                self._ui_joint_connector_line_transform = sc.Transform()
                with self._ui_joint_connector_line_transform:
                    self._ui_joint_connector_line = sc.Line(start=(0.0, 0.0, 0.0), end=(1.0, 1.0, 1.0), 
                                                         thickness=self.Style.position_line_width, color=self.Style.color_base)

                self._ui_joint_align_connector_line_transform = sc.Transform()
                with self._ui_joint_align_connector_line_transform:
                    ui_draw_stippled_line(start=(0.0, 0.0, 0.0), end=(1.0, 1.0, 1.0), 
                                                            thickness=self.Style.position_line_width, color=self.Style.error_color)

    def _refresh_ui_limits(self):
        pass

    def _refresh_ui_positions(self):
        self._attachments[0]._refresh_ui_position()
        self._attachments[1]._refresh_ui_position()
        self._refresh_ui_joint_connector()
  
    def refresh_data(self):
        self._attachments[0].refresh_data()
        self._attachments[1].refresh_data()

        if self._ui_root_transform is not None:
            self._refresh_ui_positions()
            self._refresh_ui_joint_bounds()
            self._refresh_ui_limits()

        if not self.simulation_active:
            self._usd_paths_tracked.clear()

            def add_ancestors_and_children(prim):
                if prim is not None:
                    # This includes the path itself.
                    for ancestor_path in prim.GetPath().GetAncestorsRange():
                        self._usd_paths_tracked.add(ancestor_path)
                    for child in prim.GetChildren():
                        self._usd_paths_tracked.add(child.GetPrimPath())

            add_ancestors_and_children(self._prim)
            add_ancestors_and_children(self._attachments[0].prim)
            add_ancestors_and_children(self._attachments[1].prim)

        self.manager.get_transform_gizmo_model().refresh()
        super().refresh_data()

    def on_usd_prim_paths_info_changed(self, paths : set[Sdf.Path]):
        if not paths.isdisjoint(self._usd_paths_tracked):
            if not self.get_is_valid():
                self.destroy()
            else:
                self.refresh_data()

    def on_usd_prim_paths_resynced(self, paths : set[Sdf.Path]):
        if not paths.isdisjoint(self._usd_paths_tracked):
            if not self.get_is_valid():
                self.destroy()
            else:
                self.refresh_data()

    def _get_swapped_values(self) -> tuple[list[Usd.Property], list]:
        properties = []
        values = []

        for attachment in range(2):
            other = (attachment + 1) % 2
            properties.append(self._attachments[attachment]._body_rel_attr)
            values.append(self._attachments[other]._body_rel_attr.GetTargets())

            properties.append(self._attachments[attachment]._body_offset_position_attr)
            values.append(self._attachments[other]._body_offset_position_attr.Get())

            properties.append(self._attachments[attachment]._body_offset_rotation_attr)
            values.append(self._attachments[other]._body_offset_rotation_attr.Get())

        # Flip direction of all drives.
        if self.prim.HasAPI(UsdPhysics.DriveAPI):
            drive_apis = UsdPhysics.DriveAPI.GetAll(self.prim)
            for drive_api in drive_apis:
                position_attr = drive_api.GetTargetPositionAttr()
                if position_attr.Get() != 0.0:
                    properties.append(position_attr)
                    values.append(-position_attr.Get())
                velocity_attr = drive_api.GetTargetVelocityAttr()
                if velocity_attr.Get() != 0.0:
                    properties.append(velocity_attr)
                    values.append(-velocity_attr.Get())

        return properties, values

    def attachments_swap(self):
        self._write_values_to_usd(*self._get_swapped_values())

    # Returns true if an update to the UI bounds were detected.
    def _refresh_ui_joint_bounds(self) -> bool:
        attachment_0 = self._attachments[0]
        attachment_1 = self._attachments[1]

        has_updates = False
        
        if attachment_1.prim_bounds is None:
            if attachment_1._ui_transform_bbox.visible:
                attachment_1._ui_transform_bbox.visible = False
                attachment_1._ui_transform_bbox_aligned.visible = False
                attachment_1._ui_transform_bbox_edit.visible = False
                has_updates = True
        elif not attachment_1._ui_transform_bbox.visible:
            attachment_1._ui_transform_bbox.visible = True
            attachment_1._ui_transform_bbox_aligned.visible = True
            has_updates = True

        # The reason for converting to usdrt here is because ui scene flips the sign of negative zero.
        if not components_equal(attachment_1.bounds_matrix, to_usdrt_d(attachment_1._ui_transform_bbox.transform)):
            bbox_transform = to_ui_scene(attachment_1.bounds_matrix)
        
            attachment_1._ui_transform_bbox.transform = bbox_transform
            attachment_1._ui_transform_bbox_aligned.transform = bbox_transform
            attachment_1._ui_transform_bbox_edit.transform = bbox_transform
            has_updates = True

        if attachment_0.prim_bounds is None:
            if attachment_0._ui_transform_bbox.visible:
                attachment_0._ui_transform_bbox.visible = False
                has_updates = True
        elif not attachment_0._ui_transform_bbox.visible:
            attachment_0._ui_transform_bbox.visible = True
            has_updates = True

        if not components_equal(attachment_0.bounds_matrix, to_usdrt_d(attachment_0._ui_transform_bbox.transform)):
            bbox_transform = to_ui_scene(attachment_0.bounds_matrix)
            attachment_0._ui_transform_bbox.transform = bbox_transform
            has_updates = True

        return has_updates

    def _get_target_orientation_offsets(self) -> list[float, float, float]:
        """
        Returns the current orientation offset between joint attachment point 0 and 1 for each axis as degrees.
        NB: this is not the same as decomposing the rotation matrix into Euler angles.
        """
        offsets = [0.0] * 3
        if self._attachments[0].joint_xform is None:
            return offsets

        # Create basis for transforming from joint 1 to joint 0.
        transform_delta = self._attachments[1].joint_xform * self._attachments[0].joint_xform.GetInverse()
        
        for axis in range(Axis.num):
            # For each axis, rotate a vector on the rotation plane to be able to determine the delta.
            zero_angle_direction = transform_delta.GetRow3((axis + 1) % Axis.num)
            offsets[axis] = math.degrees(math.atan2(zero_angle_direction[(axis + 2) % Axis.num], zero_angle_direction[(axis + 1) % Axis.num]))

        return offsets

    def _get_target_position_offset(self) -> Gf.Vec3d:
        if self._attachments[0].joint_xform is None:
            return Gf.Vec3d()

        transform = self._attachments[1].joint_xform * self._attachments[0].joint_xform.GetInverse()
        return transform.Transform(Gf.Vec3d())
        
    def _get_is_target_offset_valid(self) -> bool:
        """
        Determines whether the offset between attachment point 0 and 1 satisfy the constraints.
        """

        if not self._joint_validator:
            return True
        self._joint_validator_make_current()

        return self._joint_validator.validate_pose()

    def _get_valid_target_position_offset(self) -> Gf.Vec3d:
        """
        Returns a valid position offset.
        """
        if not self._joint_validator:
            return Gf.Vec3d()

        return to_usdrt_d(self._joint_validator.get_nearest_valid_position_offset(self._joint_validator.compute_offset_transform()))

    def _get_valid_target_orientation_offset(self) -> Gf.Rotation:
        if not self._joint_validator:
            return Gf.Rotation().SetIdentity()

        return to_usdrt_d(self._joint_validator.get_nearest_valid_orientation_offset(self._joint_validator.compute_offset_transform()))

    def _make_target_offset_valid(self, target : int):
        if not self._joint_validator:
            return
        self._joint_validator_make_current()

        source = (target + 1) % 2
        # Translate the target xform into the source body's reference space.
        xform_target = self._attachments[target].joint_xform
        source_body_xform = self._attachments[source].prim_xform

        valid_position_offset = self._get_valid_target_position_offset()
        valid_orientation_offset = self._get_valid_target_orientation_offset()
        if target == 1:
            valid_position_offset = -valid_position_offset
            valid_orientation_offset = valid_orientation_offset.GetInverse()

        xform_desired = (Gf.Matrix4d().SetRotate(
            valid_orientation_offset) * 
            Gf.Matrix4d().SetTranslate(valid_position_offset) * xform_target ) 

        self._attachments[source].offset_translation_matrix.SetTranslate((xform_desired * source_body_xform.GetInverse()).ExtractTranslation())
        aligned_local_offset_matrix = xform_desired * (source_body_xform * self._attachments[source].offset_translation_matrix).RemoveScaleShear().GetInverse()  # (source_body_xform).RemoveScaleShear().GetInverse()
        self._attachments[source].offset_rotation_matrix.SetRotate(aligned_local_offset_matrix.ExtractRotationMatrix())
        self._attachments[source].joint_xform = Gf.Matrix4d(xform_desired)

        self.write_offsets_to_usd()

    def _refresh_ui_joint_connector(self):
        if self._ui_joint_connector_line_transform is None:
            return

        relative_transform = self._attachments[1].joint_xform * self._attachments[0].joint_xform.GetInverse()
        offset = relative_transform.ExtractTranslation()
        if offset.GetLength() >= self.VISUAL_DELTA_TOLERANCE:
            self._ui_joint_connector_line_transform.transform = sc.Matrix44.get_scale_matrix(offset[0], offset[1], offset[2])
            self._ui_joint_connector_line.visible = True
            self._attachments[1]._ui_marker_arc.visible = True
        else:
            self._ui_joint_connector_line.visible = False
            self._attachments[1]._ui_marker_arc.visible = False

        if self.simulation_active or self._get_is_target_offset_valid():
            if (self._attachments[0]._ui_align_arrow_transform.visible or self._attachments[1]._ui_align_arrow_transform.visible):

                # When making the attachment arrows invisible, make sure that they are not set as active hover.
                active_hover = self.manager.gesture_manager.active_hover

                # As disabling a hover may cause an overlapping suspended hover to be reactivated, we have to repeat the check.
                while (active_hover is not None and active_hover.manipulator == self and
                        (active_hover.toggle_group == self.get_ui_toggle_group(JointEditManipulator.ManipulationType.ALIGN, 0) or
                         active_hover.toggle_group == self.get_ui_toggle_group(JointEditManipulator.ManipulationType.ALIGN, 1))):

                    # Make sure that we trigger the correct setter and getter behavior.
                    self.manager.gesture_manager.active_hover = None
                    active_hover = self.manager.gesture_manager.active_hover

                self._attachments[0]._ui_align_arrow_transform.visible = False
                self._attachments[1]._ui_align_arrow_transform.visible = False

            self._attachments[0]._ui_marker_arc.color = self.Style.color_base
            self._attachments[1]._ui_marker_arc.color = self.Style.color_base
            self._ui_joint_connector_line.color = self.Style.color_base
            self._ui_joint_align_connector_line_transform.visible = False
            return

        self._attachments[0]._ui_align_arrow_transform.visible = True
        self._attachments[1]._ui_align_arrow_transform.visible = True
        self._attachments[0]._ui_marker_arc.color = self.Style.error_color
        self._attachments[1]._ui_marker_arc.color = self.Style.error_color
        self._ui_joint_connector_line.color = self.Style.error_color
            
        valid_position_offset = self._get_valid_target_position_offset()
        offset_delta = valid_position_offset - offset
        if offset_delta.GetLength() >= self.VISUAL_DELTA_TOLERANCE:
            # Draw the align connection between the current and nearest valid position offset.
            self._ui_joint_align_connector_line_transform.visible = True
            self._ui_joint_align_connector_line_transform.transform = sc.Matrix44.get_translation_matrix(*to_tuple(valid_position_offset)) * sc.Matrix44.get_scale_matrix(*to_tuple(-offset_delta))
            self._ui_joint_align_connector_line_transform.visible = True
            
            # Use x as the up vector except in the special case that this is actually our offset, in which case we use y.
            up_vector = Axis.vector_x if offset_delta[1] != 0.0 or offset_delta[2] != 0.0 else Axis.vector_y
           
            arrow_lookat_matrix = Gf.Matrix4d().SetLookAt(valid_position_offset, offset, up_vector)
            arrow_lookat_matrix = to_ui_scene(arrow_lookat_matrix.GetInverse())
            self._attachments[0]._ui_align_arrow_transform.transform = arrow_lookat_matrix

            arrow_lookat_matrix = Gf.Matrix4d().SetLookAt(offset, valid_position_offset, up_vector)
            arrow_lookat_matrix = to_ui_scene(arrow_lookat_matrix.GetInverse() * relative_transform.GetInverse())
            self._attachments[1]._ui_align_arrow_transform.transform = (
                arrow_lookat_matrix)

            self._attachments[0]._ui_align_arrow_transform.scale_to = sc.Space.CURRENT
            self._attachments[1]._ui_align_arrow_transform.scale_to = sc.Space.CURRENT
        else:
            # If the position offset is right, hide the line and just show the alignment arrows.
            self._ui_joint_align_connector_line_transform.visible = False
            valid_orientation_offset = self._get_valid_target_orientation_offset()
            self._attachments[0]._ui_align_arrow_transform.transform = sc.Matrix44.get_rotation_matrix(*to_tuple(valid_orientation_offset.Decompose(*Axis.vectors)), True) * sc.Matrix44.get_translation_matrix(0, 0, -16) # * sc.Matrix44.get_rotation_matrix(*rotation, True)
            self._attachments[0]._ui_align_arrow_transform.scale_to = sc.Space.SCREEN
            self._attachments[1]._ui_align_arrow_transform.transform = sc.Matrix44.get_rotation_matrix(*to_tuple(valid_orientation_offset.GetInverse().Decompose(*Axis.vectors)), True) * sc.Matrix44.get_translation_matrix(0, 0, -16)# * sc.Matrix44.get_rotation_matrix(*rotation, True)
            self._attachments[1]._ui_align_arrow_transform.scale_to = sc.Space.SCREEN

    def destroy(self):
        self._new_frame_sub = None
        self.model = None
        if self.manager is not None:
            self.manager.get_transform_gizmo_model().detach(self)

        super().destroy()

    def __del__(self):
        self.destroy()

    class InfoboxSubsectionAttachments(PhysicsUIFloatingInfobox.Subsection):
        class TextEntries(IntEnum):
            BODY_0 = 0
            BODY_1 = auto()

        def __init__(self, info_box):
            text_entries = [None] * len(self.TextEntries)
            text_entries[self.TextEntries.BODY_0] = "Body 0:"
            text_entries[self.TextEntries.BODY_1] = "Body 1:"
            self._toggle = PhysicsUIToggleGroup()
            self._swap_image_icon = None
            super().__init__(info_box, "Attachments", text_entries, 2)

        def get_is_active(self) -> bool:
            if not super().get_is_active():
                return False

            manipulators: list[PhysicsUIViewportPrim] = self.manipulator.get_targets()

            if (len(manipulators) == 1 and isinstance(manipulators[0], JointEditManipulator) and self.manipulator.get_targets()[0].prim is not None):
                return True
            return False

        def refresh(self):
            if self.get_is_active():
                manipulator: JointEditManipulator = self.manipulator.get_targets()[0]
                if manipulator is None or manipulator._attachments[0] is None or manipulator._attachments[1] is None:
                    return

                self._swap_image_icon.visible = not manipulator.simulation_active

                text_entries = [None] * len(self.TextEntries)

                if manipulator._attachments[0].prim is not None:
                    text_entries[self.TextEntries.BODY_0] = text_truncate(manipulator._attachments[0].prim.GetPrimPath().pathString, 30)
                else: 
                    text_entries[self.TextEntries.BODY_0] = "-"

                if manipulator._attachments[1].prim is not None:
                    text_entries[self.TextEntries.BODY_1] = text_truncate(manipulator._attachments[1].prim.GetPrimPath().pathString, 30)
                else: 
                    text_entries[self.TextEntries.BODY_1] = "-"

                self.set_text(text_entries, 1)

            super().refresh()

        def _make_ui_shapes(self):
            super()._make_ui_shapes()

            self._text_entries_transform[1][0].transform = self._text_entries_transform[1][0].transform * sc.Matrix44().get_translation_matrix(-12, 0, 0)
            self._text_entries_transform[1][1].transform = self._text_entries_transform[1][1].transform * sc.Matrix44().get_translation_matrix(-12, 0, 0)
            with self._text_entries_transform[1][0]:
                with sc.Transform(transform=sc.Matrix44().get_translation_matrix(8, -round(self._style.font_size * (1.0 + self._style.font_line_spacing) * 0.5) - 1, -0.00000001)):
                    filename = str(Path(__file__).parents[3].joinpath("icons", "physicsJoint", "swap.svg"))
                    self._swap_image_icon = sc.Image(filename, color=cl("#FFFFFFFF"), width=16, height=16, gesture=[JointSwapClickGesture(self.manipulator, self._toggle), PhysicsUIHoverGesture(self.manipulator, self._toggle)])
                    toggles = self._toggle.get_manipulator_toggles(self.manipulator)
                    toggles.clear()
                    toggles.append(ColorToggle(self._swap_image_icon, COLOR_TEXT_HIGHLIGHT, self._style.font_color))

    class InfoboxSubsectionLocalOffsets(PhysicsUIFloatingInfobox.Subsection):
        class TextEntries(IntEnum):
            POSITION_OFFSET = 0
            ROTATION_OFFSETS = auto()

        def __init__(self, info_box):
            text_entries = [None] * len(self.TextEntries)
            text_entries[self.TextEntries.POSITION_OFFSET] = "Position (XYZ):"
            text_entries[self.TextEntries.ROTATION_OFFSETS] = "Orientation (XYZ):"
            super().__init__(info_box, "Local Offsets", text_entries, 4)

        def get_is_active(self) -> bool:
            if not super().get_is_active():
                return False

            manipulators: list[PhysicsUIViewportPrim] = self.manipulator.get_targets()

            if (len(manipulators) == 1 and isinstance(manipulators[0], JointEditManipulator) and self.manipulator.get_targets()[0].prim is not None):
                return True
            return False

        def refresh(self):
            if self.get_is_active():
                manipulator: JointEditManipulator = self.manipulator.get_targets()[0]
                if manipulator is None or manipulator._attachments[0] is None or manipulator._attachments[1] is None:
                    return

                text_entries = [None] * len(self.TextEntries)

                orientation_offsets = manipulator._get_target_orientation_offsets()
                position_offset = manipulator._get_target_position_offset()

                for axis in range(Axis.num):
                    text_entries[self.TextEntries.POSITION_OFFSET] = float_to_string(position_offset[axis], 6)
                    text_entries[self.TextEntries.ROTATION_OFFSETS] = float_to_string(orientation_offsets[axis], 6) + "°"
                    self.set_text(text_entries, 1 + axis)

            super().refresh()


    def get_infobox_world_position(self) -> tuple[float, float, float]:
        if self.prim_xform is None or self.stage is None:
            return None 

        # Use the maximum vertical offset from the joint to the prim bounds.
        up_axis = axis_usd_token_to_index(UsdGeom.GetStageUpAxis(self.stage))

        world_position = self.prim_xform.ExtractTranslation() 
        
        for attachment in self._attachments:
            attachment_prim_xform = attachment.prim_xform
            attachment_joint_xform = attachment.joint_xform
            if attachment_prim_xform is not None and attachment_joint_xform is not None:
                attachment_prim_bounds = attachment.prim_bounds
                # If no bounds are provided, use the position.
                if attachment_prim_bounds is None:
                    world_position[up_axis] = max(world_position[up_axis], attachment_prim_xform[3][up_axis])
                else:
                    for corner in range(8):
                        corner_transformed = attachment_prim_xform.Transform(attachment_prim_bounds.GetCorner(corner))
                        world_position[up_axis] = max(world_position[up_axis], corner_transformed[up_axis])

        return to_tuple(world_position)


    def get_infobox_view_offset(self) -> tuple[float, float]:
        if self.prim_xform is None or self.stage is None:
            return None 

        # Use the maximum horizontal distance from the joint to the prim bounds.
        up_axis = axis_usd_token_to_index(UsdGeom.GetStageUpAxis(self.stage))

        max_horizontal_distance = 0.0

        world_position = self.prim_xform.ExtractTranslation() 

        for attachment in self._attachments:
            attachment_prim_xform = attachment.prim_xform
            attachment_joint_xform = attachment.joint_xform
            if attachment_prim_xform is not None and attachment_joint_xform is not None:
                attachment_prim_bounds = attachment.prim_bounds
                # If no bounds are provided, use the position.
                if attachment_prim_bounds is None:
                    attachment_joint_position = attachment_joint_xform.ExtractTranslation()
                    offset = attachment_joint_position - world_position
                    offset[up_axis] = 0.0
                    max_horizontal_distance = max(max_horizontal_distance, offset.GetLength())
                else:
                    for corner in range(8):
                        corner_transformed = attachment_prim_xform.Transform(attachment_prim_bounds.GetCorner(corner))
                        offset = corner_transformed - world_position
                        offset[up_axis] = 0.0
                        max_horizontal_distance = max(max_horizontal_distance, offset.GetLength())

        return (max_horizontal_distance, 0.0)

    def get_infobox_screen_offset(self) -> tuple[float, float]:
        return (20.0, 20.0)

    @classmethod
    def get_infobox_subsections(cls) -> list[PhysicsUIFloatingInfobox.Subsection]:
        return super().get_infobox_subsections() + [cls.InfoboxSubsectionAttachments] + [cls.InfoboxSubsectionLocalOffsets]


class TranslationJointEditManipulator(JointEditManipulator):

    infobox_title = "PRISMATIC JOINT PROPERTIES"

    @classmethod
    def get_is_prim_valid_target(cls, prim: Usd.Prim) -> bool:
        if not super().get_is_prim_valid_target(prim):
            return False
        return prim.IsA(UsdPhysics.PrismaticJoint)

    def _joint_validator_make_current(self):
        super()._joint_validator_make_current()
        self._joint_validator.axis = self._translation_axis
        self._joint_validator.limits = self.get_translation_limits(self._translation_axis)

    def __init__(self, viewport_overlay, prim, **kwargs):
        self._prismatic_joint = UsdPhysics.PrismaticJoint(prim)
        self._axis_attr = self._prismatic_joint.GetAxisAttr()
        self._lower_limit_attr = self._prismatic_joint.GetLowerLimitAttr()
        self._upper_limit_attr = self._prismatic_joint.GetUpperLimitAttr()
        self._translation_axis = None
        self._aligned_bounds_min = None
        self._aligned_bounds_max = None

        super().__init__(viewport_overlay, prim, **kwargs)

        self._ui_translation_joint_bounds_transform = None

    def _get_swapped_values(self):
        properties, values = super()._get_swapped_values()
        properties.append(self._lower_limit_attr)
        values.append(-self._upper_limit_attr.Get())
        properties.append(self._upper_limit_attr)
        values.append(-self._lower_limit_attr.Get())
        return properties, values

    def _make_ui_shapes(self):
        super()._make_ui_shapes()
        self._make_translation_joint_shapes()

    def _make_translation_joint_shapes(self):
        self._target0_translation_transform = [None] * AxisDirection.num
        self._target0_translation_rotation_transform = [None] * Axis.num
        self._target0_translation_connector_transform = [None] * Axis.num

        self._ui_translate_limit_label_shape = [None] * AxisDirection.num

        with self._attachments[0]._ui_transform_world:
            self._ui_translation_joint_bounds_transform = sc.Transform()
            with self._ui_translation_joint_bounds_transform:
                ui_draw_corner_box(color_with_alpha(self.Style.limit_color_shaded, 0.75), 2.5, ((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)), quotient = 0.3333)

            for axis in range(Axis.num):
                self._target0_translation_rotation_transform[axis] = sc.Transform(visible=True)
                with self._target0_translation_rotation_transform[axis]:
                    for n in range(2):
                        direction = axis_to_direction(axis, (n == 0))
                        self._target0_translation_transform[direction] = sc.Transform(visible=False)
                        with self._target0_translation_transform[direction]:
                            # Translate end markers.
                            with ui_create_screen_scale_transform():
                                sc.Arc(22.5, color=self.Style.limit_color, wireframe=True, thickness=self.Style.limit_line_width)
                                sc.Arc(12.5, color=self.Style.limit_color, wireframe=True, thickness=self.Style.limit_line_width)

                                toggles = self.get_ui_toggles(JointEditManipulator.ManipulationType.TRANSLATE, direction)
                                toggles.clear()
                                arrow_direction = Axis.vector_z if direction_is_positive(direction) else -Axis.vector_z
                                arrow_orthogonal_direction = Axis.vector_x * 0.707 + Axis.vector_y * 0.707
                                arrow = ui_draw_arrow(self.Style.color_base, arrow_direction, 20, 10, -20, tesselation=4, direction_orthogonal=arrow_orthogonal_direction)
                                toggles.append(ColorToggle(arrow, COLOR_HIGHLIGHT))
                                arrow = ui_draw_arrow(self.Style.color_base_shaded, arrow_direction, 0, 10, -20, tesselation=4, direction_orthogonal=arrow_orthogonal_direction)
                                toggles.append(ColorToggle(arrow, COLOR_HIGHLIGHT_SHADED))

                                toggles.append(VisibilityToggle(self._attachments[1]._ui_transform_bbox_edit))

                                with sc.Transform(transform = sc.Matrix44().get_translation_matrix(0.0, 0.0, -10.0 if direction_is_positive(direction) else 10.0)):
                                    with sc.Transform(look_at=sc.Transform.LookAt.CAMERA):
                                        with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0.0, 0.0, -0.01)):
                                            shape = sc.Arc(12.5, color=COLOR_INVISIBLE, gesture=[TranslationJointDragGesture(self, direction), TranslationJointHoverGesture(self, direction)])
                                            toggles.append(ColorToggle(shape, COLOR_INVISIBLE))

                                        label_transform = sc.Transform(transform=sc.Matrix44.get_translation_matrix(55.0, -25.0, 0.0))
                                        toggles.append(VisibilityToggle(label_transform))
                                        with label_transform:
                                            sc.Label(axis_index_to_usd_token(axis) + (" upper" if n == 0 else " lower") + ":",
                                                        alignment=ui.Alignment.CENTER_TOP,
                                                        color=COLOR_TEXT,
                                                        size=14
                                                    )
                                            with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0.0, -18, 0.0)):
                                                self._ui_translate_limit_label_shape[direction] = sc.Label("",
                                                            alignment=ui.Alignment.CENTER_TOP,
                                                            color=COLOR_TEXT,
                                                            size=14
                                                        )
                                            with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0.0, -17.5, 0.0)):
                                                sc.Rectangle(60, 38, color=cl("#1E212360"))

                    with self._target0_translation_transform[axis_to_direction(axis, False)]:
                        self._target0_translation_connector_transform[axis] = sc.Transform()
                        with self._target0_translation_connector_transform[axis]:
                            sc.Line(start=(0.0, 0.0, 0.0), end=(1.0, 1.0, 1.0), thickness=self.Style.limit_line_width, color=self.Style.limit_color)

    def _refresh_ui_limits(self):

        display_outer_bounds = True
        outer_bounds_min = Gf.Vec3d(0.0, 0.0, 0.0)
        outer_bounds_max = Gf.Vec3d(0.0, 0.0, 0.0)

        for axis in range(Axis.num):
            rotation_limits = self.get_rotation_limits(axis)
            if rotation_limits is None or rotation_limits[0] != 0.0 or rotation_limits[1] != 0.0:
                display_outer_bounds = False

            translation_limits = self.get_translation_limits(axis)
            if (translation_limits is None or 
                translation_limits[0] == -math.inf or 
                translation_limits[1] == math.inf):
                self._target0_translation_transform[axis_to_direction(axis, True)].visible = False
                self._target0_translation_transform[axis_to_direction(axis, False)].visible = False
                self._target0_translation_connector_transform[axis].visible = False
            else:
                if translation_limits[0] < translation_limits[1]:
                    self._target0_translation_connector_transform[axis].transform = sc.Matrix44.get_scale_matrix(*(Axis.vector_z * (translation_limits[1] - translation_limits[0])))
                    self._target0_translation_connector_transform[axis].visible = True
                else:
                    translation_limits[0] = translation_limits[1]
                    self._target0_translation_connector_transform[axis].visible = False

                outer_bounds_min[axis] += translation_limits[0]
                outer_bounds_max[axis] += translation_limits[1]

                if axis == Axis.z:
                    rotation = sc.Matrix44.get_rotation_matrix(0, 0, 0, True)
                elif axis == Axis.y:
                    rotation= sc.Matrix44.get_rotation_matrix(-90, 0, -90, True)
                else:
                    rotation = sc.Matrix44.get_rotation_matrix(0, 90, 90, True)

                self._target0_translation_rotation_transform[axis].transform = rotation
                self._target0_translation_transform[axis_to_direction(axis, True)].transform = sc.Matrix44.get_translation_matrix(0, 0, translation_limits[1])
                self._target0_translation_transform[axis_to_direction(axis, False)].transform = sc.Matrix44.get_translation_matrix(0, 0, translation_limits[0])
                self._target0_translation_transform[axis_to_direction(axis, True)].visible = True
                self._target0_translation_transform[axis_to_direction(axis, False)].visible = True

                self._ui_translate_limit_label_shape[axis_to_direction(axis, False)].text = float_to_string(translation_limits[0])
                self._ui_translate_limit_label_shape[axis_to_direction(axis, True)].text = float_to_string(translation_limits[1])

        if not display_outer_bounds:
            self._ui_translation_joint_bounds_transform.visible = False
        else:
            if self._attachments[1].prim_bounds is not None:
                bounds = Gf.BBox3d(Gf.Range3d(Gf.Vec3d(0.0, 0.0, 0.0), Gf.Vec3d(1.0, 1.0, 1.0)), self._attachments[1].bounds_matrix)
                aligned = bounds.ComputeAlignedRange()
                outer_bounds_min += aligned.GetMin()
                outer_bounds_max += aligned.GetMax()

            self._ui_translation_joint_bounds_transform.visible = True
            self._ui_translation_joint_bounds_transform.transform = sc.Matrix44.get_translation_matrix(*to_tuple(outer_bounds_min)) * sc.Matrix44.get_scale_matrix(*to_tuple(outer_bounds_max - outer_bounds_min))

    def refresh_data(self):
        if not self.simulation_active or self._translation_axis is None:
            self._translation_axis = axis_usd_token_to_index(self._axis_attr.Get())
            self._limits[self.Limit.TRANSLATION_X + (self._translation_axis + 1) % Axis.num] = None
            self._limits[self.Limit.TRANSLATION_X + (self._translation_axis + 2) % Axis.num] = None

        active_gesture = self.viewport_overlay.manager.gesture_manager.active_gesture

        if active_gesture is None or active_gesture.manipulator is not self:
            lower_limit = self._lower_limit_attr.Get()
            upper_limit = self._upper_limit_attr.Get()
            self._limits[self.Limit.TRANSLATION_X + self._translation_axis] = [lower_limit, upper_limit]

        super().refresh_data()

    def write_limits_to_usd(self):
        attributes = (self._lower_limit_attr, self._upper_limit_attr)
        values = self.get_translation_limits(self._translation_axis)
        self._write_values_to_usd(attributes, values)

    class InfoboxSubsectionTranslateLimits(PhysicsUIFloatingInfobox.Subsection):
        class TextEntries(IntEnum):
            AXIS = 0
            LOWER = auto()
            UPPER = auto()

        def __init__(self, info_box):
            text_entries = [None] * len(self.TextEntries)
            text_entries[self.TextEntries.AXIS] = "Axis:"
            text_entries[self.TextEntries.LOWER] = "Lower Limit:"
            text_entries[self.TextEntries.UPPER] = "Upper Limit:"
            super().__init__(info_box, "Limits", text_entries, 2)

        def get_is_active(self) -> bool:
            if not super().get_is_active():
                return False

            joint_manipulators: list[PhysicsUIViewportPrim] = self.manipulator.get_targets()

            if (len(joint_manipulators) == 1 and isinstance(joint_manipulators[0], TranslationJointEditManipulator)):
                return True
            return False

        def refresh(self):
            if self.get_is_active():
                joint_manipulator: TranslationJointEditManipulator = self.manipulator.get_targets()[0]
                if joint_manipulator.prim is None or joint_manipulator._translation_axis is None:
                    return

                text_entries = ["-"] * len(self.TextEntries)

                text_entries[self.TextEntries.AXIS] = axis_index_to_usd_token(joint_manipulator._translation_axis)

                limits = joint_manipulator.get_translation_limits(joint_manipulator._translation_axis)

                if limits is not None:
                    text_entries[self.TextEntries.LOWER] = float_to_string(limits[0])
                    text_entries[self.TextEntries.UPPER] = float_to_string(limits[1])

                self.set_text(text_entries, 1)

                for n in range(2):
                    direction = axis_to_direction(joint_manipulator._translation_axis, (n == 0))
                    toggle_group = joint_manipulator.get_ui_toggle_group(JointEditManipulator.ManipulationType.TRANSLATE, direction)
                    toggles = toggle_group.get_manipulator_toggles(self.manipulator)
                    if len(toggles) == 0:
                        toggles.append(HighlightToggle(self._ui_text_entries_shape[0][self.TextEntries.UPPER if n == 0 else self.TextEntries.LOWER]))
                        toggles.append(HighlightToggle(self._ui_text_entries_shape[1][self.TextEntries.UPPER if n == 0 else self.TextEntries.LOWER]))

            super().refresh()

    @classmethod
    def get_infobox_subsections(cls) -> list[PhysicsUIFloatingInfobox.Subsection]:
        return JointEditManipulator.get_infobox_subsections() + [cls.InfoboxSubsectionTranslateLimits]


class TranslationJointHoverGesture(PhysicsUIHoverGesture):

    def __init__(self, manipulator : TranslationJointEditManipulator, direction):
        self.manipulator : TranslationJointEditManipulator = None
        self._direction = direction
        self._axis = direction_to_axis(self._direction)
        self._limit_index = JointEditManipulator.Limit.TRANSLATION_X + self._axis
        super().__init__(manipulator, toggle_group=manipulator.get_ui_toggle_group(JointEditManipulator.ManipulationType.TRANSLATE, direction))

    def on_began(self):
        super().on_began()
        limits = self.manipulator._limits[self._limit_index]
        translation = [0.0] * 3
        translation[self._axis] = limits[1 if direction_is_positive(self._direction) else 0]
        self.manipulator._attachments[1]._ui_transform_bbox_edit_gesture.transform = sc.Matrix44().get_translation_matrix(*translation)


class TranslationJointDragGesture(PhysicsUIDragGesture):
    def __init__(self, manipulator : TranslationJointEditManipulator, direction):
        self.manipulator : TranslationJointEditManipulator = None
        super().__init__(manipulator, toggle_group=manipulator.get_ui_toggle_group(JointEditManipulator.ManipulationType.TRANSLATE, direction))
        self._direction = direction
        self._axis = direction_to_axis(self._direction)
        self._limit_index = JointEditManipulator.Limit.TRANSLATION_X + self._axis

    def _offset_to_translate(self, point) -> float:
        transform = self.manipulator._attachments[0]._ui_transform_world.transform * self.manipulator._target0_translation_rotation_transform[direction_to_axis(self._direction)].transform
        # Translate the point into local space.
        point = Gf.Vec4d(point[0], point[1], point[2], 1.0) * to_usdrt(transform).GetInverse()
        return point[2]

    def on_began(self):
        super().on_began()

    def on_changed(self):
        super().on_changed()
        new = self._offset_to_translate(self.sender.gesture_payload.ray_closest_point)

        limits = self.manipulator._limits[self._limit_index]

        if direction_is_positive(self._direction): 
            limits[1] = max(new, limits[0])
        else:
            limits[0] = min(new, limits[1])

        self.manipulator._refresh_ui_limits()

        translation = [0.0] * 3
        translation[self._axis] = limits[1 if direction_is_positive(self._direction) else 0]
        self.manipulator._attachments[1]._ui_transform_bbox_edit_gesture.transform = sc.Matrix44().get_translation_matrix(*translation)
        self.manipulator.viewport_overlay.refresh_info_box()

    def on_ended(self):
        super().on_ended()

        self.manipulator.write_limits_to_usd()


class RevoluteJointEditManipulator(JointEditManipulator):

    infobox_title = "REVOLUTE JOINT PROPERTIES"

    @classmethod
    def get_is_prim_valid_target(cls, prim: Usd.Prim) -> bool:
        if not super().get_is_prim_valid_target(prim):
            return False
        return prim.IsA(UsdPhysics.RevoluteJoint)
        
    def _joint_validator_make_current(self):
        super()._joint_validator_make_current()
        self._joint_validator.axis = self._rotation_axis
        self._joint_validator.limits = self.get_rotation_limits(self._rotation_axis)

    def __init__(self, viewport_overlay, prim, **kwargs):
        self._revolute_joint = UsdPhysics.RevoluteJoint(prim)
        self._axis_attr = self._revolute_joint.GetAxisAttr()
        self._lower_limit_attr = self._revolute_joint.GetLowerLimitAttr()
        self._upper_limit_attr = self._revolute_joint.GetUpperLimitAttr()
        self._rotation_axis = None
        self._bounds_radii = [0.0] * AxisDirection.num
        super().__init__(viewport_overlay, prim, **kwargs)

    def _get_swapped_values(self):
        properties, values = super()._get_swapped_values()
        properties.append(self._lower_limit_attr)
        values.append(-self._upper_limit_attr.Get())
        properties.append(self._upper_limit_attr)
        values.append(-self._lower_limit_attr.Get())
        return properties, values

    def _make_ui_shapes(self):
        super()._make_ui_shapes()
        self._make_rotation_joint_shapes()

    def _make_rotation_joint_shapes(self):
        self._target0_rotation_transform = [None] * Axis.num
        self._target0_rotation_offset_transform = [None] * Axis.num
        self._target0_rotation_baseline_transform = [None] * Axis.num
        self._target0_rotation_arc_shape = [None] * AxisDirection.num
        self._target0_rotation_arc_scale_transform = [None] * AxisDirection.num

        self._target0_rotation_handle_transform = [None] * AxisDirection.num

        self._ui_rotation_limit_label_shape = [[None] * AxisDirection.num, [None] * AxisDirection.num]

        with self._ui_root_transform:
            with self._attachments[0]._ui_transform_world:
                for axis in range(Axis.num):
                    self._target0_rotation_transform[axis] = sc.Transform()
                    if axis == Axis.z:
                        self._target0_rotation_transform[axis].transform = sc.Matrix44().get_rotation_matrix(-90.0, 0.0, -90.0, True)
                    elif axis == Axis.y:
                        self._target0_rotation_transform[axis].transform = sc.Matrix44().get_rotation_matrix(0.0, 90.0, 90.0, True)
                    else:
                        self._target0_rotation_transform[axis].transform = sc.Matrix44().get_rotation_matrix(0.0, 0.0, 0.0, True)
                    with self._target0_rotation_transform[axis]:
                        self._target0_rotation_offset_transform[axis] = sc.Transform()

                    with self._target0_rotation_offset_transform[axis]:
                        self._target0_rotation_baseline_transform[axis] = sc.Transform()
                        with self._target0_rotation_baseline_transform[axis]:
                            ui_draw_stippled_line(start=(0, 0, 0), end=(0, 1, 0), color=self.Style.limit_color, thickness=self.Style.limit_line_width)

                toggles = []
                for direction in range(AxisDirection.num):
                    direction_toggles = self.get_ui_toggles(JointEditManipulator.ManipulationType.ROTATE, direction)
                    direction_toggles.clear()
                    direction_toggles.append(VisibilityToggle(self._attachments[1]._ui_transform_bbox_edit))
                    toggles.append(direction_toggles)

                for direction in range(AxisDirection.num):
                    axis = direction_to_axis(direction)
                    with self._target0_rotation_offset_transform[axis]:
                        self._target0_rotation_arc_scale_transform[direction] = sc.Transform()
                        with self._target0_rotation_arc_scale_transform[direction]:
                            self._target0_rotation_arc_shape[direction] = sc.Arc(1.0, axis=0, sector=True, begin=-1.570796, end=1.570796, color=self.Style.limit_color, wireframe=True, thickness=self.Style.limit_line_width, intersection_thickness=0)

                        self._target0_rotation_handle_transform[direction] = [None, None]
                        for angular_direction in range(2):
                            self._target0_rotation_handle_transform[direction][angular_direction] = sc.Transform()
                            with self._target0_rotation_handle_transform[direction][angular_direction]:
                                with ui_create_screen_scale_transform():
                                    with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0, -10.0, 0)):
                                        with sc.Transform(look_at=sc.Transform.LookAt.CAMERA):
                                            with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0.0, 0.0, -0.01)):
                                                sc.Arc(10.0, color=COLOR_INVISIBLE, gesture=[RevoluteJointDragGesture(self, direction, angular_direction), RevoluteJointHoverGesture(self, direction, angular_direction)])
                                            label_transform = sc.Transform(transform=sc.Matrix44.get_translation_matrix(55.0, -25.0, 0.0))
                                            toggles[axis_to_direction(axis, angular_direction == 1)].append(VisibilityToggle(label_transform))
                                            with label_transform:
                                                sc.Label(axis_index_to_usd_token(axis) + (" upper" if angular_direction == 1 else " lower") + ":",
                                                            alignment=ui.Alignment.CENTER_TOP,
                                                            color=COLOR_TEXT,
                                                            size=14
                                                        )
                                                with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0.0, -18, 0.0)):
                                                    self._ui_rotation_limit_label_shape[0 if direction_is_positive(direction) else 1][axis_to_direction(axis, angular_direction == 1)] = sc.Label("",
                                                                alignment=ui.Alignment.CENTER_TOP,
                                                                color=COLOR_TEXT,
                                                                size=14
                                                            )
                                                with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0.0, -17.5, 0.0)):
                                                    sc.Rectangle(60, 38, color=cl("#1E212360"))

                                        arrow = ui_draw_arrow(self.Style.color_base, Axis.vector_y, 20.0, 8.5, -10.0)
                                        toggles[axis_to_direction(axis, angular_direction == 1)].append(ColorToggle(arrow, COLOR_HIGHLIGHT))
                                        arrow = ui_draw_arrow(self.Style.color_base_shaded, Axis.vector_y, 0.0, 8.5, -10.0)
                                        toggles[axis_to_direction(axis, angular_direction == 1)].append(ColorToggle(arrow, COLOR_HIGHLIGHT_SHADED))

    def _make_rotation_limits_valid(self, rotation_limits):
        if rotation_limits[0] == -math.inf:
            # This means free rotation. -180/180 yields the same but allows us to place the handles.
            rotation_limits[0] = -180.0
        if rotation_limits[1] == math.inf:
            # This means free rotation. -180/180 yields the same but allows us to place the handles.
            rotation_limits[1] = 180.0

        if rotation_limits[0] >= rotation_limits[1]:
            rotation_limits[1] = rotation_limits[0]

    def _refresh_ui_rotation_handles(self, axis):
        rotation_limits = self.get_rotation_limits(axis)
        
        if self._attachments[1].prim_bounds is None:
            scale_to = sc.Space.SCREEN
        else:
            scale_to = sc.Space.CURRENT

        for direction in range(2):
            rotation_handle_transforms = self._target0_rotation_handle_transform[axis_to_direction(axis, (direction == 1))]
            for angular_direction in range(2):
                rotation_handle_transform = rotation_handle_transforms[angular_direction]
                rotation_handle_transform.scale_to = scale_to
                if rotation_limits is None or rotation_limits[0] == -math.inf or rotation_limits[1] == math.inf:
                    rotation_handle_transform.visible = False
                # Only show the handles furthest from center.
                elif direction == 1 and -self._bounds_radii[axis_to_direction(axis, False)] > self._bounds_radii[axis_to_direction(axis, True)]:
                    rotation_handle_transform.visible = False
                elif direction == 0 and -self._bounds_radii[axis_to_direction(axis, False)] <= self._bounds_radii[axis_to_direction(axis, True)]:
                    rotation_handle_transform.visible = False
                else:
                    rotation_handle_transform.visible = True
                    y = math.cos(math.radians(rotation_limits[angular_direction])) * self._bounds_radii[axis_to_direction(axis, (direction == 1))]
                    z = math.sin(math.radians(rotation_limits[angular_direction])) * self._bounds_radii[axis_to_direction(axis, (direction == 1))]
                    rotation_handle_transform.transform = (sc.Matrix44.get_translation_matrix(0.0, y, z) * 
                                                                            sc.Matrix44.get_rotation_matrix((-90.0 if direction == 0 else 90.0) * 
                                                                                                            (-1.0 if angular_direction == 0 else 1.0) + rotation_limits[angular_direction], 0.0, 0.0, True))

    def _refresh_ui_limits(self):
        for axis in range(Axis.num):
            rotation_limits = self.get_rotation_limits(axis)

            if rotation_limits is None or rotation_limits[0] == -math.inf or rotation_limits[1] == math.inf:
                self._target0_rotation_transform[axis].visible = False
            else:
                if rotation_limits[0] == -180.0 and rotation_limits[1] == 180.0:
                    self._target0_rotation_arc_shape[axis_to_direction(axis, False)].sector = False
                    self._target0_rotation_arc_shape[axis_to_direction(axis, True)].sector = False
                else:
                    self._target0_rotation_arc_shape[axis_to_direction(axis, False)].sector = (self._bounds_radii[axis_to_direction(axis, False)] < 0.0)
                    self._target0_rotation_arc_shape[axis_to_direction(axis, True)].sector = (self._bounds_radii[axis_to_direction(axis, True)] > 0.0)

                self._target0_rotation_transform[axis].visible = True

                for n in range(2):
                    clamped_limits = rotation_limits

                    if rotation_limits[1] - rotation_limits[0] >= 180.0:
                        if self._bounds_radii[axis_to_direction(axis, False)] < 0.0 and self._bounds_radii[axis_to_direction(axis, True)] > 0.0:
                            if (n == 1) == (-self._bounds_radii[axis_to_direction(axis, False)] > self._bounds_radii[axis_to_direction(axis, True)]):
                                # If the attachment is extended in both direction and we span more than half a circle, limit the smaller arc to only span the range not covered by the larger arc.
                                clamped_limits = [max(rotation_limits[0], -180.0 + rotation_limits[1]), min(rotation_limits[1], 180.0 + rotation_limits[0])]
                                self._target0_rotation_arc_shape[axis_to_direction(axis, (n == 1))].sector = False

                    self._target0_rotation_arc_shape[axis_to_direction(axis, (n == 1))].begin = math.radians(clamped_limits[0])
                    self._target0_rotation_arc_shape[axis_to_direction(axis, (n == 1))].end = math.radians(clamped_limits[1])
                    self._target0_rotation_arc_shape[axis_to_direction(axis, (n == 1))].tesselation = max(2, int((clamped_limits[1] - clamped_limits[0])))

                    self._ui_rotation_limit_label_shape[n][axis_to_direction(axis, False)].text = float_to_string(rotation_limits[0]) + "°"
                    self._ui_rotation_limit_label_shape[n][axis_to_direction(axis, True)].text = float_to_string(rotation_limits[1]) + "°"

            self._refresh_ui_rotation_handles(axis)

    
    def _refresh_ui_joint_bounds(self) -> bool:
        if not super()._refresh_ui_joint_bounds():
            return False

        attachment_1 = self._attachments[1]
        prim_bounds = attachment_1.prim_bounds

        if prim_bounds is None:
            new_bounds_radii = [200.0, 0.0] * Axis.num
        else:
            # For revolute joints, we scale and reorient the rotation arcs based on the attachment extents.

            # First, transform all corners of bounds to joint space.
            origin = attachment_1.bounds_matrix.Transform(Gf.Vec3d())
            offset_right = Gf.Vec3d(*attachment_1.bounds_matrix[0][0:3])
            offset_up = Gf.Vec3d(*attachment_1.bounds_matrix[1][0:3])
            offset_forward = Gf.Vec3d(*attachment_1.bounds_matrix[2][0:3])

            # Follows the order of USD corner indices which are LDB, RDB, LUB, RUB, LDF, RDF, LUF, RUF. 
            bounds_corners_transformed = (
                origin, 
                origin + offset_right, 
                origin + offset_up, 
                origin + offset_right + offset_up, 
                origin + offset_forward, 
                origin + offset_right + offset_forward, 
                origin + offset_up + offset_forward, 
                origin + offset_right + offset_up + offset_forward
            )

            # Determines the midpoint of a box side. This is the midpoint of the diagonal.
            def get_side_midpoint(_0, _1):
                return (bounds_corners_transformed[_0] + bounds_corners_transformed[_1]) * 0.5

            side_midpoints = (get_side_midpoint(0, 3),  # Backside
                            get_side_midpoint(0, 5),  # Down
                            get_side_midpoint(0, 6),  # Left
                            get_side_midpoint(1, 7),  # Right
                            get_side_midpoint(2, 7),  # Up
                            get_side_midpoint(4, 7)   # Front
                            )

            # We align the zero angle direction for each axis with the furthest midpoint relative to that axial rotation plane.
            zero_angle_directions = (Gf.Vec3d(), Gf.Vec3d(), Gf.Vec3d())

            for axis in range(Axis.num):
                side_center_distance = 0.0
                side_center = Gf.Vec3d()
                for side in range(6):
                    side_center_aligned = Gf.Vec3d(side_midpoints[side][(axis + 1) % Axis.num], side_midpoints[side][(axis + 2) % Axis.num], 0.0)
                    if side_center_aligned.GetLength() > side_center_distance:
                        side_center = side_center_aligned
                        side_center_distance = side_center_aligned.GetLength()
                angle = math.atan2(side_center[1], side_center[0])
                self._target0_rotation_offset_transform[axis].transform = sc.Matrix44.get_rotation_matrix(angle, 0, 0)
                zero_angle_directions[axis][0] = math.cos(angle)
                zero_angle_directions[axis][1] = math.sin(angle)

            new_bounds_radii = [-math.inf, math.inf] * Axis.num

            # Find the furthest distance of each bounding box corner aligned to each axial rotation plane.
            for corner in range(8):
                for axis in range(Axis.num):
                    bounds_corners_axis_transformed = Gf.Vec3d(bounds_corners_transformed[corner][(axis + 1) % Axis.num], bounds_corners_transformed[corner][(axis + 2) % Axis.num], 0.0)
                    
                    # Find the furthest distances of the bounding boxes relative to each rotation plane.
                    distance = bounds_corners_axis_transformed.GetLength()
                    if zero_angle_directions[axis].GetDot(bounds_corners_axis_transformed) > 0.0:
                        new_bounds_radii[axis_to_direction(axis, True)] = max(new_bounds_radii[axis_to_direction(axis, True)], distance)
                        new_bounds_radii[axis_to_direction(axis, False)] = min(new_bounds_radii[axis_to_direction(axis, False)], distance)
                    else:
                        new_bounds_radii[axis_to_direction(axis, True)] = max(new_bounds_radii[axis_to_direction(axis, True)], -distance)
                        new_bounds_radii[axis_to_direction(axis, False)] = min(new_bounds_radii[axis_to_direction(axis, False)], -distance)
                    

            # Find the nearest distance of the edges projected to the axial rotation planes. For this, we have to use line segments projected to each plane.
            bounds_edge_segments = (
                (0, 1), # LDB -> RDB
                (0, 2), # LDB -> LUB
                (0, 4), # LDB -> LDF
                (1, 3), # RDB -> RUB
                (1, 5), # RDB -> RDF
                (2, 3), # LUB -> RUB
                (2, 6), # LUB -> LUF
                (3, 7), # RUB -> RUF
                (4, 5), # LDF -> RDF
                (4, 6), # LDF -> LUF
                (5, 7), # RDF -> RUF
                (6, 7), # LUF -> RUF
                )

            for axis in range(Axis.num):
                for edge in range(12):
                    corner_start = bounds_corners_transformed[bounds_edge_segments[edge][0]]
                    corner_end = bounds_corners_transformed[bounds_edge_segments[edge][1]]
                    edge_start = Gf.Vec3d(corner_start[(axis + 1) % Axis.num], corner_start[(axis + 2) % Axis.num], 0.0)
                    edge_end = Gf.Vec3d(corner_end[(axis + 1) % Axis.num], corner_end[(axis + 2) % Axis.num], 0.0)
                    edge_segment = Gf.LineSeg(edge_start, edge_end)
                    closest_point = edge_segment.FindClosestPoint(Gf.Vec3d(0.0, 0.0, 0.0))

                    # Check what direction the closest point is in.
                    if zero_angle_directions[axis].GetDot(closest_point[0]) > 0.0:
                        new_bounds_radii[axis_to_direction(axis, False)] = min(new_bounds_radii[axis_to_direction(axis, False)], closest_point[0].GetLength())
                    else:
                        new_bounds_radii[axis_to_direction(axis, True)] = max(new_bounds_radii[axis_to_direction(axis, True)], -closest_point[0].GetLength())

        if components_equal(self._bounds_radii, new_bounds_radii):
            return True

        self._bounds_radii = new_bounds_radii
        self._refresh_ui_rotation_arcs()
        self._refresh_ui_limits()

        return True

    def _refresh_ui_rotation_arcs(self):
        if self._attachments[1].prim_bounds is None:
            scale_to = sc.Space.SCREEN
        else:
            scale_to = sc.Space.CURRENT
        
        for axis in range(Axis.num):
            for n in range(2):
                direction = axis_to_direction(axis, (n == 0))
                self._target0_rotation_arc_scale_transform[direction].scale_to = scale_to
                scale = sc.Matrix44.get_scale_matrix(self._bounds_radii[direction], self._bounds_radii[direction], self._bounds_radii[direction])
                self._target0_rotation_arc_scale_transform[direction].transform = scale

                if -self._bounds_radii[axis_to_direction(axis, False)] > self._bounds_radii[axis_to_direction(axis, True)]:
                    self._target0_rotation_arc_shape[direction].color = self.Style.limit_color if n == 1 else color_with_alpha(self.Style.limit_color_shaded, 0.5)
                else:
                    self._target0_rotation_arc_shape[direction].color = self.Style.limit_color if n == 0 else color_with_alpha(self.Style.limit_color_shaded, 0.5)

            self._target0_rotation_baseline_transform[axis].scale_to = scale_to

            axial_radius_min = self._bounds_radii[axis_to_direction(axis, False)]
            axial_radius_max = self._bounds_radii[axis_to_direction(axis, True)]

            if axial_radius_max > -axial_radius_min:
                self._target0_rotation_baseline_transform[axis].transform = sc.Matrix44.get_scale_matrix(1.0, axial_radius_max, 1.0)
            else:
                self._target0_rotation_baseline_transform[axis].transform = sc.Matrix44.get_scale_matrix(1.0, axial_radius_min, 1.0)

    def refresh_data(self):
        if not self.simulation_active or self._rotation_axis is None:
            self._rotation_axis = axis_usd_token_to_index(self._axis_attr.Get())
            self._limits[self.Limit.ROTATION_X + (self._rotation_axis + 1) % Axis.num] = None
            self._limits[self.Limit.ROTATION_X + (self._rotation_axis + 2) % Axis.num] = None

        active_gesture = self.viewport_overlay.manager.gesture_manager.active_gesture
        if active_gesture is None or active_gesture.manipulator is not self:
            limits = [self._lower_limit_attr.Get(), self._upper_limit_attr.Get()]
            self._make_rotation_limits_valid(limits)
            self._limits[self.Limit.ROTATION_X + self._rotation_axis] = limits

        super().refresh_data()

    def write_limits_to_usd(self):
        if self._rotation_axis is None:
            return

        attributes = (self._lower_limit_attr, self._upper_limit_attr)
        values = self.get_rotation_limits(self._rotation_axis)
        if values[0] == -180.0 and values[1] == 180.0:
            values = [-math.inf, math.inf]
        self._write_values_to_usd(attributes, values)

    class InfoboxSubsectionRevoluteLimits(PhysicsUIFloatingInfobox.Subsection):
        class TextEntries(IntEnum):
            AXIS = 0
            LOWER = auto()
            UPPER = auto()

        def __init__(self, info_box):
            text_entries = [None] * len(self.TextEntries)
            text_entries[self.TextEntries.AXIS] = "Axis:"
            text_entries[self.TextEntries.LOWER] = "Lower Limit:"
            text_entries[self.TextEntries.UPPER] = "Upper Limit:"
            super().__init__(info_box, "Limits", text_entries, 2)

        def get_is_active(self) -> bool:
            if not super().get_is_active():
                return False

            joint_manipulators: list[PhysicsUIViewportPrim] = self.manipulator.get_targets()

            if (len(joint_manipulators) == 1 and isinstance(joint_manipulators[0], RevoluteJointEditManipulator)):
                return True
            return False

        def refresh(self):
            if self.get_is_active():
                joint_manipulator: RevoluteJointEditManipulator = self.manipulator.get_targets()[0]
                if joint_manipulator.prim is None or joint_manipulator._rotation_axis is None:
                    return

                text_entries = ["-"] * len(self.TextEntries)

                text_entries[self.TextEntries.AXIS] = axis_index_to_usd_token(joint_manipulator._rotation_axis)

                limits = joint_manipulator.get_rotation_limits(joint_manipulator._rotation_axis)

                if limits is not None and (limits[0] != -180.0 or limits[1] != 180.0):
                    text_entries[self.TextEntries.LOWER] = float_to_string(limits[0]) + "°"
                    text_entries[self.TextEntries.UPPER] = float_to_string(limits[1]) + "°"

                self.set_text(text_entries, 1)

                for n in range(2):
                    direction = axis_to_direction(joint_manipulator._rotation_axis, (n == 0))
                    toggle_group = joint_manipulator.get_ui_toggle_group(JointEditManipulator.ManipulationType.ROTATE, direction)
                    toggles = toggle_group.get_manipulator_toggles(self.manipulator)
                    if len(toggles) == 0:
                        toggles.append(HighlightToggle(self._ui_text_entries_shape[0][self.TextEntries.UPPER if n == 0 else self.TextEntries.LOWER]))
                        toggles.append(HighlightToggle(self._ui_text_entries_shape[1][self.TextEntries.UPPER if n == 0 else self.TextEntries.LOWER]))

            super().refresh()

    @classmethod
    def get_infobox_subsections(cls) -> list[PhysicsUIFloatingInfobox.Subsection]:
        return super().get_infobox_subsections() + [cls.InfoboxSubsectionRevoluteLimits]


    def get_infobox_world_position(self) -> tuple[float, float, float]:
        infobox_position = super().get_infobox_world_position()
        if infobox_position is None:
            return None

        up_axis = axis_usd_token_to_index(UsdGeom.GetStageUpAxis(self.stage))

        # If our rotation plane is not perpendicular to the up axis, make sure that our vertical offset is at least the same height as the arc bounds.
        if self._rotation_axis != up_axis and self._attachments[1].prim_bounds is not None:
            world_position_up = self.prim_xform[3][up_axis]
            world_position_up += max(self._bounds_radii[axis_to_direction(self._rotation_axis, True)],
                                            -self._bounds_radii[axis_to_direction(self._rotation_axis, False)])

            if world_position_up > infobox_position[up_axis]:
                infobox_position = [*infobox_position]
                infobox_position[up_axis] = world_position_up
                infobox_position = (*infobox_position,)

        return infobox_position

    def get_infobox_view_offset(self) -> tuple[float, float]:
        offset = super().get_infobox_view_offset()

        if self._attachments[1].prim_bounds is not None:
            max_radii = max(self._bounds_radii[axis_to_direction(self._rotation_axis, True)],
                                            -self._bounds_radii[axis_to_direction(self._rotation_axis, False)])
            if max_radii > offset[0]:
                offset = (max_radii, offset[1])

        return offset

    def get_infobox_screen_offset(self) -> tuple[float, float]:
        offset = super().get_infobox_view_offset()
        if self._attachments[1].prim_bounds is None:
            offset = (offset[0] + self._bounds_radii[axis_to_direction(self._rotation_axis, True)], offset[1])
        return offset


class RevoluteJointHoverGesture(PhysicsUIHoverGesture):

    def __init__(self, manipulator : RevoluteJointEditManipulator, direction, angular_direction):
        self.manipulator : RevoluteJointEditManipulator = None
        self._axis = direction_to_axis(direction)
        self._angular_direction = angular_direction
        self._direction = direction
        self._limit_index = JointEditManipulator.Limit.ROTATION_X + self._axis
        super().__init__(manipulator, toggle_group=manipulator.get_ui_toggle_group(JointEditManipulator.ManipulationType.ROTATE, axis_to_direction(self._axis, angular_direction)))

    def on_began(self):
        super().on_began()
        limits = self.manipulator._limits[self._limit_index]
        rotation = [0.0] * 3
        rotation[self._axis] = limits[self._angular_direction]
        if rotation[self._axis] == math.inf or rotation[self._axis] == -math.inf:
            rotation[self._axis] = 180.0
        self.manipulator._attachments[1]._ui_transform_bbox_edit_gesture.transform = sc.Matrix44().get_rotation_matrix(*rotation, True)

class RevoluteJointDragGesture(PhysicsUIDragGesture):
    def __init__(self, manipulator : RevoluteJointEditManipulator, direction, angular_direction):
        self.manipulator : RevoluteJointEditManipulator = None
        self._axis = direction_to_axis(direction)
        self._direction = direction
        self._angular_direction = angular_direction
        self._limit_index = JointEditManipulator.Limit.ROTATION_X + self._axis
        super().__init__(manipulator, toggle_group=manipulator.get_ui_toggle_group(JointEditManipulator.ManipulationType.ROTATE, axis_to_direction(self._axis, angular_direction)))

    def _offset_to_plane_angle(self, point) -> float:
        transform = self.manipulator._attachments[0]._ui_transform_world.transform * self.manipulator._target0_rotation_transform[self._axis].transform * self.manipulator._target0_rotation_offset_transform[self._axis].transform
        position = Gf.Vec4d(0.0, 0.0, 0.0, 1.0)
        point = Gf.Vec4d(point[0], point[1], point[2], 1.0) * to_usdrt(transform).GetInverse()

        offset_dir = Gf.Vec3d()
        for n in range(3):
            offset_dir[n] = (point[n] - position[n])
        angle = math.atan2(offset_dir[2], offset_dir[1])
        return math.degrees(angle)

    def on_began(self):
        super().on_began()
        limits = self.manipulator._limits[self._limit_index]
        rotation = [0.0] * 3
        rotation[self._axis] = limits[self._angular_direction]
        self.manipulator._attachments[1]._ui_transform_bbox_edit_gesture.transform = sc.Matrix44().get_rotation_matrix(*rotation, True)


    def on_changed(self):
        super().on_changed()
        new = self._offset_to_plane_angle(self.sender.gesture_payload.ray_closest_point)

        if not direction_is_positive(self._direction):
            if new <= 0.0: 
                new += 180.0
            else:
                new -= 180.0

        limits = self.manipulator._limits[self._limit_index]

        if isinstance(self.manipulator, SphericalJointEditManipulator):
            if self._angular_direction == 0:
                new = -new

            if new < 0.00001:
                if limits[1] > 90.0:
                    new = 179.99999
                else:
                    new = 0.00001
            elif new >= 179.99999:
                new = 179.99999

            limits[0] = -new
            limits[1] = new
        else:
            if self._angular_direction == 0:
                if limits[0] < -175.0 and new > 90.0:
                    new = -180.0
                elif new > limits[1]:
                    new = limits[1]
                limits[0] = new
            else:
                if limits[1] > 175.0 and new < -90.0:
                    new = 180.0
                elif new < limits[0]:
                    new = limits[0]
                limits[1] = new

        self.manipulator._make_rotation_limits_valid(limits)
        self.manipulator._refresh_ui_limits()
        rotation = [0.0] * 3
        rotation[self._axis] = limits[self._angular_direction]
        self.manipulator._attachments[1]._ui_transform_bbox_edit_gesture.transform = sc.Matrix44().get_rotation_matrix(*rotation, True)
        self.manipulator.viewport_overlay.refresh_info_box()

    def on_ended(self):
        super().on_ended()
        self.manipulator.write_limits_to_usd()

class SphericalJointEditManipulator(RevoluteJointEditManipulator):

    infobox_title = "SPHERICAL JOINT PROPERTIES"

    @classmethod
    def get_is_prim_valid_target(cls, prim: Usd.Prim) -> bool:
        if not JointEditManipulator.get_is_prim_valid_target(prim):
            return False
        return prim.IsA(UsdPhysics.SphericalJoint)

    class Style(RevoluteJointEditManipulator.Style):
        spherical_limit_tesselation_max = 512
        spherical_limit_tesselation_min = 64

    def _joint_validator_make_current(self):
        JointEditManipulator._joint_validator_make_current(self)
        self._joint_validator.axis = self._rotation_axis
        self._joint_validator.limits = (self.get_rotation_limits((self._rotation_axis + 1) % Axis.num)[1], self.get_rotation_limits((self._rotation_axis + 2) % Axis.num)[1])

    def __init__(self, viewport_overlay, prim, **kwargs):
        self._spherical_joint = UsdPhysics.SphericalJoint(prim)
        self._axis_attr = self._spherical_joint.GetAxisAttr()
        self._angle0_limit_attr = self._spherical_joint.GetConeAngle0LimitAttr()
        self._angle1_limit_attr = self._spherical_joint.GetConeAngle1LimitAttr()
        self._rotation_axis = None
        self._bounds_radii = [0.0] * AxisDirection.num
        JointEditManipulator.__init__(self, viewport_overlay, prim, **kwargs)

    def _get_swapped_values(self):
        return JointEditManipulator._get_swapped_values(self)

    def _make_ui_shapes(self):
        super()._make_ui_shapes()
        self._make_spherical_joint_shapes()

    def _make_spherical_joint_shapes(self):
        self._target0_spherical_limits_transform = None
        self._target0_spherical_limits_shape = [None, None]
        self._target_marker_rotate_transform = None
        self._target_marker_translate_transforms = [None, None]
        self._target_marker_connector_transform = None
        with self._ui_root_transform:
            with self._attachments[0]._ui_transform_world:
                self._target0_spherical_limits_transform = sc.Transform()
                with self._target0_spherical_limits_transform:
                    for n in range(2):
                        self._target0_spherical_limits_shape[n] = sc.Curve(
                            [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                            thicknesses=[self.Style.limit_line_width], curve_type=sc.Curve.CurveType.LINEAR,
                            colors=[self.Style.limit_color])

            with self._attachments[1]._ui_transform_world:
                self._target_marker_rotate_transform = sc.Transform()
                with self._target_marker_rotate_transform:
                    for n in range(2):
                        self._target_marker_translate_transforms[n] = sc.Transform()
                        with self._target_marker_translate_transforms[n]:
                            with sc.Transform(scale_to=sc.Space.SCREEN, look_at=sc.Transform.LookAt.CAMERA):
                                sc.Arc(10.0, color=self.Style.color_base)
                    self._target_marker_connector_transform = sc.Transform()
                    with self._target_marker_connector_transform:
                        sc.Line(start=(0, 0, 0), end=(1, 0, 0), thickness=2.5, color=self.Style.color_base)

    def _refresh_ui_limits(self):
        super()._refresh_ui_limits()

        rotation_axis = self._rotation_axis
        if rotation_axis == Axis.z:
            self._target0_rotation_transform[rotation_axis].transform = sc.Matrix44().get_rotation_matrix(-90.0, 0.0, -90.0, True)
        elif rotation_axis == Axis.y:
            self._target0_rotation_transform[rotation_axis].transform = sc.Matrix44().get_rotation_matrix(0.0, 90.0, 90.0, True)
        else:
            self._target0_rotation_transform[rotation_axis].transform = sc.Matrix44().get_rotation_matrix(0.0, 0.0, 0.0, True)

        self._target_marker_rotate_transform.transform = self._target0_rotation_transform[rotation_axis].transform
        self._target0_rotation_transform[(rotation_axis + 1) % Axis.num].transform = self._target0_rotation_transform[rotation_axis].transform * sc.Matrix44().get_rotation_matrix(0.0, 180.0, 90.0, True)
        self._target0_rotation_transform[(rotation_axis + 2) % Axis.num].transform = self._target0_rotation_transform[rotation_axis].transform * sc.Matrix44().get_rotation_matrix(-90.0, 0.0, -90.0, True)

        if self._attachments[1].prim_bounds is None:
            self._target0_spherical_limits_transform.scale_to = sc.Space.SCREEN
        else:
            self._target0_spherical_limits_transform.scale_to = sc.Space.CURRENT


        axis0 = (rotation_axis + 1) % Axis.num
        axis1 = (rotation_axis + 2) % Axis.num

        limit1 = self.get_rotation_limits(axis1)[1]
        limit0 = self.get_rotation_limits(axis0)[1]

        positions = [[], []]

        radii = [0, 0]
        
        for n in range(2):
            radii[n] = max(self._bounds_radii[axis_to_direction(axis0, (n == 0))], self._bounds_radii[axis_to_direction(axis1, (n == 0))]) 

        # Approximates the total length of the limit line drawn.
        tesselation = int((self.Style.spherical_limit_tesselation_max - self.Style.spherical_limit_tesselation_min) * (
                        max(max(90 - math.fabs(limit0 - 90), 90 - math.fabs(limit1 - 90)), 
                        math.fabs(limit1 - limit0))
                        ) / 180.0 + self.Style.spherical_limit_tesselation_min + 0.5)


        # Adjust angles to give more tesselation on the wider angle.
        angle_factor0 = math.tan(math.radians(limit0) / 4)
        angle_factor1 = math.tan(math.radians(limit1) / 4)
        
        # In case we are manipulating the limits, we need to make sure the joint validator is up to date.
        self._joint_validator_make_current()

        for n in range(tesselation + 1):
            angle = 2 * math.pi * n / tesselation

            # NB. The order of the factors is intentionally reversed here.
            x = math.cos(angle) * angle_factor1
            y = math.sin(angle) * angle_factor0
            angle = math.atan2(y, x)

            limit_at_angle = self._joint_validator.get_limit_in_direction(math.degrees(angle))
            offset = Gf.Vec3d(math.cos(math.radians(limit_at_angle)), math.cos(angle) * math.sin(math.radians(limit_at_angle)), math.sin(angle) * math.sin(math.radians(limit_at_angle)))
           
            for n2 in range(2):
                position = [0, 0, 0]
                position[rotation_axis] = offset[0] * radii[n2]
                position[axis0] = offset[1] * radii[n2]
                position[axis1] = offset[2] * radii[n2]
                positions[n2].append(position)

        self._target0_spherical_limits_shape[0].positions = positions[0]
        self._target0_spherical_limits_shape[1].positions = positions[1]

        self._target0_spherical_limits_shape[0].visible = True
        self._target0_spherical_limits_shape[1].visible = True

        if limit1 <= -90.0 or limit0 <= -90.0:
            # If the attachment is extended in both direction and we span more than half a circle on either axis, only display the larger spherical limit. 
            if radii[1] < 0.0 and radii[0] > 0.0:
                if -radii[1] > radii[0]:
                    self._target0_spherical_limits_shape[0].visible = False
                else:
                    self._target0_spherical_limits_shape[1].visible = False


    def _refresh_ui_joint_bounds(self) -> bool:
        if not JointEditManipulator._refresh_ui_joint_bounds(self):
            return False

        attachment_1 = self._attachments[1]
        prim_bounds = attachment_1.prim_bounds

        # Spherical joints only scale to the radius at the direction of the rotation axis.
        if prim_bounds is None:
            new_bounds_radii = [200.0, 0.0]
        else:        
            # For spherical joints, we scale the rotation arcs based on the attachment extents.

            # First, transform all corners of bounds to joint space.
            origin = attachment_1.bounds_matrix.Transform(Gf.Vec3d())
            offset_right = attachment_1.bounds_matrix.Transform(Gf.Vec3d(1.0, 0.0, 0.0)) - origin
            offset_up = attachment_1.bounds_matrix.Transform(Gf.Vec3d(0.0, 1.0, 0.0)) - origin
            offset_forward = attachment_1.bounds_matrix.Transform(Gf.Vec3d(0.0, 0.0, 1.0)) - origin

            # Follows the order of USD corner indices which are LDB, RDB, LUB, RUB, LDF, RDF, LUF, RUF. 
            bounds_corners_transformed = (
                origin, 
                origin + offset_right, 
                origin + offset_up, 
                origin + offset_right + offset_up, 
                origin + offset_forward, 
                origin + offset_right + offset_forward, 
                origin + offset_up + offset_forward, 
                origin + offset_right + offset_up + offset_forward
            )

            new_bounds_radii = [-math.inf, math.inf]
            zero_direction_axis = Axis.vectors[self._rotation_axis]
            
            for corner in range(8):
                distance = zero_direction_axis.GetDot(bounds_corners_transformed[corner])
                new_bounds_radii[1] = min(new_bounds_radii[1], distance)
                new_bounds_radii[0] = max(new_bounds_radii[0], distance)

        if components_equal(self._bounds_radii[0:2], new_bounds_radii):
            return True

        self._bounds_radii = new_bounds_radii * 3
        self._refresh_ui_rotation_arcs()
        return True

    def _refresh_ui_rotation_arcs(self):
        super()._refresh_ui_rotation_arcs()

        bounds_radii = self._bounds_radii[0:2]
        self._target_marker_translate_transforms[0].transform = sc.Matrix44.get_translation_matrix(bounds_radii[1], 0, 0)
        self._target_marker_translate_transforms[1].transform = sc.Matrix44.get_translation_matrix(bounds_radii[0], 0, 0)
        self._target_marker_connector_transform.transform = sc.Matrix44.get_translation_matrix(bounds_radii[1], 0, 0) * sc.Matrix44.get_scale_matrix(bounds_radii[0] - bounds_radii[1], 1, 1)

        attachment_1 = self._attachments[1]
        prim_bounds = attachment_1.prim_bounds

        if prim_bounds is None:
            scale_to = sc.Space.SCREEN
        else:
            scale_to = sc.Space.CURRENT

        self._target_marker_translate_transforms[0].scale_to = scale_to
        self._target_marker_translate_transforms[1].scale_to = scale_to
        self._target_marker_connector_transform.scale_to = scale_to

        for n in range(2):
            if bounds_radii[0] >= -bounds_radii[1]:
                self._target0_spherical_limits_shape[n].colors = [self.Style.limit_color if n == 0 else color_with_alpha(self.Style.limit_color_shaded, 0.5)]
            else:
                self._target0_spherical_limits_shape[n].colors = [self.Style.limit_color if n == 1 else color_with_alpha(self.Style.limit_color_shaded, 0.5)]

    def _make_rotation_limits_valid(self, rotation_limits):
        if rotation_limits[1] > 179.99999:
            rotation_limits[1] = 179.99999
        elif rotation_limits[1] < 0.00001:
            rotation_limits[1] = 0.00001

        rotation_limits[0] = -rotation_limits[1]

    def refresh_data(self):
        if not self.simulation_active or self._rotation_axis is None:
            self._rotation_axis = axis_usd_token_to_index(self._axis_attr.Get())

        active_gesture = self.viewport_overlay.manager.gesture_manager.active_gesture
        if active_gesture is None or active_gesture.manipulator is not self:
            angle0_limit = [0.0, self._angle0_limit_attr.Get()]
            angle1_limit = [0.0, self._angle1_limit_attr.Get()]
            self._make_rotation_limits_valid(angle0_limit)
            self._make_rotation_limits_valid(angle1_limit)
            self._limits[self.Limit.ROTATION_X + self._rotation_axis] = None
            self._limits[self.Limit.ROTATION_X + (self._rotation_axis + 1) % Axis.num] = angle0_limit
            self._limits[self.Limit.ROTATION_X + (self._rotation_axis + 2) % Axis.num] = angle1_limit

        JointEditManipulator.refresh_data(self)

    def write_limits_to_usd(self):
        attributes = (self._angle0_limit_attr, self._angle1_limit_attr)
        values = (
            self._limits[self.Limit.ROTATION_X + (self._rotation_axis + 1) % Axis.num][1], 
            self._limits[self.Limit.ROTATION_X + (self._rotation_axis + 2) % Axis.num][1]
            )

        self._write_values_to_usd(attributes, values)


    class InfoboxSubsectionSphericalLimits(PhysicsUIFloatingInfobox.Subsection):
        class TextEntries(IntEnum):
            AXIS = 0
            CONE_0 = auto()
            CONE_1 = auto()

        def __init__(self, info_box):
            text_entries = [None] * len(self.TextEntries)
            text_entries[self.TextEntries.AXIS] = "Axis:"
            text_entries[self.TextEntries.CONE_0] = "Limit 0:"
            text_entries[self.TextEntries.CONE_1] = "Limit 1:"
            super().__init__(info_box, "Limits", text_entries, 2)

        def get_is_active(self) -> bool:
            if not super().get_is_active():
                return False

            joint_manipulators: list[PhysicsUIViewportPrim] = self.manipulator.get_targets()

            if (len(joint_manipulators) == 1 and isinstance(joint_manipulators[0], SphericalJointEditManipulator)):
                return True
            return False

        def refresh(self):
            if self.get_is_active():
                joint_manipulator: SphericalJointEditManipulator = self.manipulator.get_targets()[0]
                if joint_manipulator is None or joint_manipulator.prim is None or joint_manipulator._rotation_axis is None:
                    return

                text_entries = ["-"] * len(self.TextEntries)
                text_entries[self.TextEntries.AXIS] = axis_index_to_usd_token(joint_manipulator._rotation_axis)

                cone_axis_0 = (joint_manipulator._rotation_axis + 1) % Axis.num
                cone_axis_0_limits = joint_manipulator.get_rotation_limits(cone_axis_0)
                if cone_axis_0_limits is not None:
                    text_entries[self.TextEntries.CONE_0] = float_to_string(cone_axis_0_limits[1]) + "°"
                cone_axis_1 = (joint_manipulator._rotation_axis + 2) % Axis.num
                cone_axis_1_limits = joint_manipulator.get_rotation_limits(cone_axis_1)
                if cone_axis_1_limits is not None:
                    text_entries[self.TextEntries.CONE_1] = float_to_string(cone_axis_1_limits[1]) + "°"

                self.set_text(text_entries, 1)

                for axis in range(2):
                    for n in range(2):
                        direction = axis_to_direction((joint_manipulator._rotation_axis + axis + 1) % Axis.num, (n == 0))
                        toggle_group = joint_manipulator.get_ui_toggle_group(JointEditManipulator.ManipulationType.ROTATE, direction)
                        toggles = toggle_group.get_manipulator_toggles(self.manipulator)
                        if len(toggles) == 0:
                            toggles.append(HighlightToggle(self._ui_text_entries_shape[0][self.TextEntries.CONE_0 if axis == 0 else self.TextEntries.CONE_1]))
                            toggles.append(HighlightToggle(self._ui_text_entries_shape[1][self.TextEntries.CONE_0 if axis == 0 else self.TextEntries.CONE_1]))

            super().refresh()

    @classmethod
    def get_infobox_subsections(cls) -> list[PhysicsUIFloatingInfobox.Subsection]:
        return JointEditManipulator.get_infobox_subsections() + [cls.InfoboxSubsectionSphericalLimits]

class DistanceJointDragGesture(PhysicsUIDragGesture):
    def __init__(self, manipulator, direction):
        super().__init__(manipulator, toggle_group=manipulator.get_ui_toggle_group(JointEditManipulator.ManipulationType.DISTANCE, direction))
        self._initial_position_offset : Gf.Vec3d = None
        self._direction = direction

    def _offset_to_distance(self, point) -> float:
        point = Gf.Vec4d(point[0], point[1], point[2], 1.0) * self.manipulator._attachments[0].joint_xform.GetInverse()
        offset = Gf.Vec3d(point[0], point[1], point[2]) + self._initial_position_offset
        return offset.GetLength()

    def on_began(self):
        super().on_began()
        # Compute the initial position offset to add to any other returned position.
        limits = self.manipulator._limits[JointEditManipulator.Limit.DISTANCE]
        point = self.sender.gesture_payload.ray_closest_point
        point = Gf.Vec4d(point[0], point[1], point[2], 1.0) * self.manipulator._attachments[0].joint_xform.GetInverse()
        offset = Gf.Vec3d(point[0], point[1], point[2])
        limit_point = offset.GetNormalized() * limits[self._direction]
        self._initial_position_offset = limit_point - offset

    def on_changed(self):
        super().on_changed()
        new = self._offset_to_distance(self.sender.gesture_payload.ray_closest_point)
        limits = self.manipulator._limits[JointEditManipulator.Limit.DISTANCE]

        if self._direction == 1:
            limits[1] = max(new, limits[0])
        else:
            limits[0] = max(min(new, limits[1]), 0)
       
        self.manipulator._refresh_ui_limits()
        self.manipulator.viewport_overlay.refresh_info_box()

    def on_ended(self):
        super().on_ended()
        self.manipulator.write_limits_to_usd()


class DistanceJointEditManipulator(JointEditManipulator):

    infobox_title = "DISTANCE JOINT PROPERTIES"

    @classmethod
    def get_is_prim_valid_target(cls, prim: Usd.Prim) -> bool:
        if not super().get_is_prim_valid_target(prim):
            return False
        return prim.IsA(UsdPhysics.DistanceJoint)

    def _joint_validator_make_current(self):
        super()._joint_validator_make_current()
        self._joint_validator.limits = self._limits[self.Limit.DISTANCE]

    def __init__(self, viewport_overlay, prim, **kwargs):
        self._distance_joint = UsdPhysics.DistanceJoint(prim)
        self._min_distance_attr = self._distance_joint.GetMinDistanceAttr()
        self._max_distance_attr = self._distance_joint.GetMaxDistanceAttr()
        super().__init__(viewport_overlay, prim, **kwargs)
        self._limits[JointEditManipulator.Limit.ROTATION_X] = [-math.inf, math.inf]
        self._limits[JointEditManipulator.Limit.ROTATION_Y] = [-math.inf, math.inf]
        self._limits[JointEditManipulator.Limit.ROTATION_Z] = [-math.inf, math.inf]
        self._limits[JointEditManipulator.Limit.TRANSLATION_X] = [-math.inf, math.inf]
        self._limits[JointEditManipulator.Limit.TRANSLATION_Y] = [-math.inf, math.inf]
        self._limits[JointEditManipulator.Limit.TRANSLATION_Z] = [-math.inf, math.inf]

    def _make_ui_shapes(self):
        super()._make_ui_shapes()
        self._make_distance_joint_shapes()

    def _make_distance_joint_shapes(self):
        self._target0_distance_limits_transform = [None, None]
        self._target0_distance_transform = None
        self._target0_distance_limits_handle_transform = [None, None]

        self._ui_distance_limit_label_transform = [None, None]
        self._ui_distance_limit_label_shape = [None, None]

        with self._attachments[0]._ui_transform_world:
            self._target0_distance_transform = sc.Transform(look_at=sc.Transform.LookAt.CAMERA, visible=False)
            with self._target0_distance_transform:
                for n in range(2):
                    toggle_group = self.get_ui_toggle_group(JointEditManipulator.ManipulationType.DISTANCE, n)
                    toggles = toggle_group.get_manipulator_toggles(self)
                    toggles.clear()

                    self._target0_distance_limits_transform[n] = sc.Transform()
                    with self._target0_distance_limits_transform[n]:
                        shape = sc.Arc(1.0, axis=2, color=self.Style.limit_color, wireframe=True, thickness=self.Style.limit_line_width, tesselation=360)



                    self._ui_distance_limit_label_transform[n] = sc.Transform()
                    toggles.append(VisibilityToggle(self._ui_distance_limit_label_transform[n]))

                    with self._ui_distance_limit_label_transform[n]:
                        with ui_create_screen_scale_transform():
                            with sc.Transform(transform=sc.Matrix44.get_translation_matrix(30.0 if n == 1 else -30.0, -10.0 if n == 1 else 45.0, 0.0)):
                                sc.Label(("Min" if n == 0 else "Max") + ":",
                                            alignment=ui.Alignment.CENTER_TOP,
                                            color=COLOR_TEXT,
                                            size=14
                                        )
                                with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0.0, -18, 0.0)):
                                    self._ui_distance_limit_label_shape[n] = sc.Label("",
                                                alignment=ui.Alignment.CENTER_TOP,
                                                color=COLOR_TEXT,
                                                size=14
                                            )
                                with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0.0, -17.5, 0.0)):
                                    sc.Rectangle(60, 38, color=cl("#1E212360"))

                    handle_transforms = []

                    for direction in range(4):
                        direction_vector = AxisDirection.vectors[direction]
                        direction_orthogonal_vector = AxisDirection.vectors[3 - direction]
                        transform = sc.Transform()
                        handle_transforms.append(transform)
                        if(n == 0):
                            direction_vector = -direction_vector
                        with transform:
                            screen_scale_transform = ui_create_screen_scale_transform()
                        with screen_scale_transform:
                            translation = -direction_vector * 7.5
                            with sc.Transform(transform=sc.Matrix44().get_translation_matrix(*translation)):
                                arrow = ui_draw_arrow(self.Style.color_base, direction_vector, 15.0, 7.5, -7.5, tesselation=2, direction_orthogonal=direction_orthogonal_vector)
                                toggles.append(ColorToggle(arrow, COLOR_HIGHLIGHT))
                                with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0.0, 0.0, -0.01)):
                                    shape = sc.Arc(20.0, color=COLOR_INVISIBLE, gesture=[DistanceJointDragGesture(self, n), PhysicsUIHoverGesture(self, toggle_group)])
                                    toggles.append(ColorToggle(shape, COLOR_INVISIBLE))


                    self._target0_distance_limits_handle_transform[n] = handle_transforms

    def _refresh_ui_limits(self):
        distance_limits = self._limits[self.Limit.DISTANCE]
        if distance_limits is None:
            self._target0_distance_transform.visible = False
            return
        
        self._target0_distance_transform.visible = True
        for n in range(2):
            self._target0_distance_limits_transform[n].transform = sc.Matrix44.get_scale_matrix(distance_limits[n], distance_limits[n], distance_limits[n])
            for direction in range(4):
                direction_vector = AxisDirection.vectors[direction] * distance_limits[n]
                self._target0_distance_limits_handle_transform[n][direction].transform = sc.Matrix44.get_translation_matrix(*direction_vector)

            self._ui_distance_limit_label_shape[n].text = float_to_string(distance_limits[n])
            self._ui_distance_limit_label_transform[n].transform = sc.Matrix44.get_translation_matrix(distance_limits[n] * 0.707, -distance_limits[n] * 0.707, 0.0)

    # Returns true if an update to the UI bounds were detected.
    def _refresh_ui_joint_bounds(self) -> bool:
        if super()._refresh_ui_joint_bounds():
            # Override the align bbox visibility, since it's not relevant for distance joints.
            self._attachments[1]._ui_transform_bbox_aligned.visible = False
            return True
        else:
            return False

    def refresh_data(self):
        active_gesture = self.viewport_overlay.manager.gesture_manager.active_gesture
        if active_gesture is None or active_gesture.manipulator is not self:
            self._limits[self.Limit.DISTANCE] = [self._min_distance_attr.Get(), self._max_distance_attr.Get()]
        super().refresh_data()

    def write_limits_to_usd(self):
        attributes = (self._min_distance_attr, self._max_distance_attr)
        values = self._limits[self.Limit.DISTANCE]
        self._write_values_to_usd(attributes, values)

    class InfoboxSubsectionDistanceLimits(PhysicsUIFloatingInfobox.Subsection):
        class TextEntries(IntEnum):
            LOWER = 0
            UPPER = auto()

        def __init__(self, info_box):
            text_entries = [None] * len(self.TextEntries)
            text_entries[self.TextEntries.LOWER] = "Min:"
            text_entries[self.TextEntries.UPPER] = "Max:"
            super().__init__(info_box, "Distance Limits", text_entries, 2)

        def get_is_active(self) -> bool:
            if not super().get_is_active():
                return False

            joint_manipulators: list[PhysicsUIViewportPrim] = self.manipulator.get_targets()

            if (len(joint_manipulators) == 1 and isinstance(joint_manipulators[0], DistanceJointEditManipulator)):
                return True
            return False

        def refresh(self):
            if self.get_is_active():
                joint_manipulator: DistanceJointEditManipulator = self.manipulator.get_targets()[0]
                if joint_manipulator.prim is None:
                    return

                text_entries = ["-"] * len(self.TextEntries)

                limits = joint_manipulator._limits[joint_manipulator.Limit.DISTANCE]

                if limits is not None:
                    text_entries[self.TextEntries.LOWER] = float_to_string(limits[0])
                    text_entries[self.TextEntries.UPPER] = float_to_string(limits[1])

                self.set_text(text_entries, 1)

                for n in range(2):
                    toggle_group = joint_manipulator.get_ui_toggle_group(JointEditManipulator.ManipulationType.DISTANCE, n)
                    toggles = toggle_group.get_manipulator_toggles(self.manipulator)
                    if len(toggles) == 0:
                        toggles.append(HighlightToggle(self._ui_text_entries_shape[0][self.TextEntries.LOWER if n == 0 else self.TextEntries.UPPER]))
                        toggles.append(HighlightToggle(self._ui_text_entries_shape[1][self.TextEntries.LOWER if n == 0 else self.TextEntries.UPPER]))

            super().refresh()

    @classmethod
    def get_infobox_subsections(cls) -> list[PhysicsUIFloatingInfobox.Subsection]:
        return JointEditManipulator.get_infobox_subsections() + [cls.InfoboxSubsectionDistanceLimits]

    def get_infobox_world_position(self) -> tuple[float, float, float]:
        infobox_position = super().get_infobox_world_position()
        if infobox_position is None:
            return None

        up_axis = axis_usd_token_to_index(UsdGeom.GetStageUpAxis(self.stage))

        # Make sure that our vertical offset is at least the same height as the far distance of movement.
        if self._limits[self.Limit.DISTANCE] is not None:
            world_position_up = self.prim_xform[3][up_axis]
            world_position_up += self._limits[self.Limit.DISTANCE][1]
            if world_position_up > infobox_position[up_axis]:
                infobox_position = [*infobox_position]
                infobox_position[up_axis] = world_position_up
                infobox_position = (*infobox_position,)

        return infobox_position

    def get_infobox_view_offset(self) -> tuple[float, float]:
        offset = super().get_infobox_view_offset()

        # Make sure that our horizontal offset is at least the same as the far distance of movement.
        if self._limits[self.Limit.DISTANCE] is not None and self._limits[self.Limit.DISTANCE][1] > offset[0]:
            offset = (self._limits[self.Limit.DISTANCE][1], offset[1])
        return offset


class D6JointEditManipulator(TranslationJointEditManipulator, DistanceJointEditManipulator, RevoluteJointEditManipulator):

    infobox_title = "D6 JOINT PROPERTIES"

    def _joint_validator_make_current(self):
        JointEditManipulator._joint_validator_make_current(self)
        self._joint_validator.limits_distance = tuple(self._limits[self.Limit.DISTANCE])
        self._joint_validator.limits_rotation = tuple(self._limits[self.Limit.ROTATION_X:self.Limit.ROTATION_X+3])
        self._joint_validator.limits_translation = tuple(self._limits[self.Limit.TRANSLATION_X:self.Limit.TRANSLATION_X+3])

    @classmethod
    def get_is_prim_valid_target(cls, prim: Usd.Prim) -> bool:
        if not JointEditManipulator.get_is_prim_valid_target(prim):
            return False
        return prim.HasAPI(UsdPhysics.LimitAPI)

    def __init__(self, viewport_overlay, prim, **kwargs):
        JointEditManipulator.__init__(self, viewport_overlay, prim, **kwargs)
        self._limit_api_attr = None  # [None] * len(self.Limit)
        self._bounds_radii = [0.0] * AxisDirection.num
        self._ui_translation_limit_box_transform = None
    
    def _make_ui_shapes(self):
        JointEditManipulator._make_ui_shapes(self)
        DistanceJointEditManipulator._make_distance_joint_shapes(self)
        TranslationJointEditManipulator._make_translation_joint_shapes(self)
        RevoluteJointEditManipulator._make_rotation_joint_shapes(self)
        with self._attachments[0]._ui_transform_world:
            self._ui_translation_limit_box_transform = sc.Transform(visible=False)
            with self._ui_translation_limit_box_transform:
                ui_draw_corner_box(color_with_alpha(self.Style.limit_color, 0.75), 2.5, ((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)), quotient = 0.3333)


    def _refresh_ui_joint_bounds(self) -> bool:
        return RevoluteJointEditManipulator._refresh_ui_joint_bounds(self)

    def _refresh_ui_limits(self):
        DistanceJointEditManipulator._refresh_ui_limits(self)
        TranslationJointEditManipulator._refresh_ui_limits(self)
        RevoluteJointEditManipulator._refresh_ui_limits(self)

        translation_limit_offset = [0.0, 0.0, 0.0]
        translation_limit_scale = [0.000001, 0.000001, 0.000001]
        translation_axes = 0
        for axis in range(Axis.num):
            translation_limits = self.get_translation_limits(axis)
            if translation_limits is not None:
                translation_axes += 1
                translation_limit_offset[axis] = translation_limits[0]
                translation_limit_scale[axis] = translation_limits[1] - translation_limits[0]
        
        if translation_axes < 2:
            self._ui_translation_limit_box_transform.visible = False
        else:
            self._ui_translation_limit_box_transform.visible = True
            self._ui_translation_limit_box_transform.transform = sc.Matrix44.get_translation_matrix(*translation_limit_offset) * sc.Matrix44.get_scale_matrix(*translation_limit_scale)

    def _make_target_offset_valid(self, target : int):
        JointEditManipulator._make_target_offset_valid(self, target)

    _limit_to_token = {
        JointEditManipulator.Limit.ROTATION_X : UsdPhysics.Tokens.rotX, 
        JointEditManipulator.Limit.ROTATION_Y : UsdPhysics.Tokens.rotY, 
        JointEditManipulator.Limit.ROTATION_Z : UsdPhysics.Tokens.rotZ, 
        JointEditManipulator.Limit.TRANSLATION_X : UsdPhysics.Tokens.transX, 
        JointEditManipulator.Limit.TRANSLATION_Y : UsdPhysics.Tokens.transY, 
        JointEditManipulator.Limit.TRANSLATION_Z : UsdPhysics.Tokens.transZ, 
        JointEditManipulator.Limit.DISTANCE : UsdPhysics.Tokens.distance }

    _limit_to_title = {
        JointEditManipulator.Limit.ROTATION_X : "Rotation X", 
        JointEditManipulator.Limit.ROTATION_Y : "Rotation Y", 
        JointEditManipulator.Limit.ROTATION_Z : "Rotation Z", 
        JointEditManipulator.Limit.TRANSLATION_X : "Translation X", 
        JointEditManipulator.Limit.TRANSLATION_Y : "Translation Y", 
        JointEditManipulator.Limit.TRANSLATION_Z : "Translation Z", 
        JointEditManipulator.Limit.DISTANCE : "Distance" }

    def refresh_data(self):
        if not self.simulation_active or self._limit_api_attr is None:
            self._limit_api_attr = [None] * len(self.Limit)
            for limit_type in self.Limit:
                api_limits = UsdPhysics.LimitAPI.Get(self._prim, self._limit_to_token[limit_type])
                if api_limits:
                    self._limit_api_attr[limit_type] = (api_limits.GetLowAttr(), api_limits.GetHighAttr())
                else: 
                    self._limits[limit_type] = [-math.inf, math.inf]

        active_gesture = self.viewport_overlay.manager.gesture_manager.active_gesture
        if active_gesture is None or active_gesture.manipulator is not self:
            for limit_type in self.Limit:
                if self._limit_api_attr[limit_type] is not None:
                    self._limits[limit_type] = [self._limit_api_attr[limit_type][0].Get(), self._limit_api_attr[limit_type][1].Get()]

        JointEditManipulator.refresh_data(self)

    def write_limits_to_usd(self):
        attributes = []
        values = []

        for limit_type in self.Limit:
            if self._limit_api_attr[limit_type] is not None and self._limits[limit_type] is not None:
                api_limits = UsdPhysics.LimitAPI.Get(self._prim, self._limit_to_token[limit_type])
                attributes.append(api_limits.GetLowAttr())
                attributes.append(api_limits.GetHighAttr())
                values.append(self._limits[limit_type][0])
                values.append(self._limits[limit_type][1])

        if len(attributes) > 0:
            self._write_values_to_usd(attributes, values)

    def _get_swapped_values(self):
        properties, values = JointEditManipulator._get_swapped_values(self)

        for limit_type in self.Limit:
            if limit_type is not self.Limit.DISTANCE and self._limit_api_attr[limit_type] is not None and self._limits[limit_type] is not None:
                api_limits = UsdPhysics.LimitAPI.Get(self._prim, self._limit_to_token[limit_type])
                properties.append(api_limits.GetLowAttr())
                properties.append(api_limits.GetHighAttr())
                values.append(-self._limits[limit_type][1])
                values.append(-self._limits[limit_type][0])

        return properties, values

    class InfoboxSubsectionD6Limits(PhysicsUIFloatingInfobox.Subsection):
        class TextEntries(IntEnum):
            TRANSLATE_X = 0
            TRANSLATE_Y = auto()
            TRANSLATE_Z = auto()

        def __init__(self, info_box):
            super().__init__(info_box, "Limits", [""], 3)

        def get_is_active(self) -> bool:
            if not super().get_is_active():
                return False
            
            joint_manipulators: list[PhysicsUIViewportPrim] = self.manipulator.get_targets()

            if (len(joint_manipulators) == 1 and isinstance(joint_manipulators[0], D6JointEditManipulator)):
                return True
            return False

        def refresh(self):
            if self.get_is_active():
                joint_manipulator: D6JointEditManipulator = self.manipulator.get_targets()[0]
                if joint_manipulator.prim is None or joint_manipulator._limit_api_attr is None:
                    return

                text_entries_names = []
                text_entries_limit_lower = []
                text_entries_limit_upper = []

                for limit_type in JointEditManipulator.Limit:
                    if joint_manipulator._limit_api_attr[limit_type] is not None and joint_manipulator._limits[limit_type] is not None:
                        text_entries_names.append(D6JointEditManipulator._limit_to_title[limit_type])
                        unit = "°" if (limit_type == JointEditManipulator.Limit.ROTATION_X or limit_type == JointEditManipulator.Limit.ROTATION_Y or limit_type == JointEditManipulator.Limit.ROTATION_Z) else ""
                        text_entries_limit_lower.append(float_to_string(joint_manipulator._limits[limit_type][0], 8) + unit)
                        text_entries_limit_upper.append(float_to_string(joint_manipulator._limits[limit_type][1], 8) + unit)

                self.set_text(text_entries_names, 0)
                self.set_text(text_entries_limit_lower, 1)
                self.set_text(text_entries_limit_upper, 2)

            super().refresh()

    @classmethod
    def get_infobox_subsections(cls) -> list[PhysicsUIFloatingInfobox.Subsection]:
        return JointEditManipulator.get_infobox_subsections() + [cls.InfoboxSubsectionD6Limits]

    def get_infobox_world_position(self) -> tuple[float, float, float]:
        return JointEditManipulator.get_infobox_world_position(self)

    def get_infobox_view_offset(self) -> tuple[float, float]:
        return JointEditManipulator.get_infobox_view_offset(self)

    def get_infobox_screen_offset(self) -> tuple[float, float]:
        return JointEditManipulator.get_infobox_screen_offset(self)
