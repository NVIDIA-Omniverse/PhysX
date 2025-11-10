# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from operator import mod
import omni.kit.commands
import omni.usd
from omni.usd._impl.utils import get_prim_at_path
from pxr import UsdGeom, UsdUtils, UsdPhysics, Gf, PhysicsSchemaTools, Tf, Usd, Sdf
import carb
from carb.settings import get_settings
from carb.eventdispatcher import get_eventdispatcher
from omni.physx import get_physx_property_query_interface
from omni.timeline import get_timeline_interface
from omni.ui_scene import scene as sc
import omni.ui as ui
from omni.ui import color as cl
from omni.physx.bindings._physx import PhysxPropertyQueryRigidBodyResponse, PhysxPropertyQueryColliderResponse, PhysxPropertyQueryResult, PhysxPropertyQueryMode
from omni.kit.manipulator.tool.snap import settings_constants as snap_c
from omni.kit.manipulator.transform.settings_listener import SnapSettingsListener
import omni.kit.window.property
import math
import collections.abc
from pathlib import Path
from .physicsViewportShared import *
from cmath import inf
from omni.physx.scripts import utils
from omni.physx.bindings._physx import SETTING_MASS_DISTRIBUTION_MANIPULATOR, SETTING_DISPLAY_MASS_PROPERTIES
import weakref

"""
Mass distribution visualization.

Allows conveniently viewing and manipulating mass properties of a body. Computes the diagonal inertia based on a cuboid
approximation shape

"""

DRAG_SMOOTHNESS_FACTOR = 3.0
DRAG_SMOOTHNESS_MODIFIER = 1.0 / (1.0 + DRAG_SMOOTHNESS_FACTOR)

# If true, renders subdivision markers on rulers.
VISUAL_MASS_DISTRIBUTION_RULER_SUBDIVISION_MARKERS = False

# When scaling shapes to screen with ui.scene, it multiplies pixels with this.
UI_SCENE_SCREEN_SCALE_MODIFIER = 2

# Always be applied by kit, doesn't seem like there's any way to truly fetch this value.
UI_FONT_PADDING = 1

GESTURE_SCALE = 0
GESTURE_TRANSLATE = 1
GESTURE_ROTATE = 2
GESTURE_NUM = 3

MASS_DISTRIBUTION_COLOR = cl("#66FFFF00")
MASS_DISTRIBUTION_BOX_COLOR = cl("#000000FF") + MASS_DISTRIBUTION_COLOR
MASS_DISTRIBUTION_BOX_LINE_THICKNESS = 2
MASS_DISTRIBUTION_HANDLE_SIZE = 25
MASS_DISTRIBUTION_HANDLE_RECTANGLE = False
MASS_DISTRIBUTION_HANDLE_RECTANGLE_SIZE = 40
MASS_DISTRIBUTION_HANDLE_RECTANGLE_TRAIL_COUNT = 0
MASS_DISTRIBUTION_HANDLE_RECTANGLE_TRAIL_ALPHA = 0xAA # Separate value since we fade it in.
MASS_DISTRIBUTION_HANDLE_ARROW = False
MASS_DISTRIBUTION_HANDLE_ARROW_SIZE = 20
CENTER_OF_MASS_AXIS_ARROW_LENGTH = 30
CENTER_OF_MASS_AXIS_ARROW_RADIUS = 12
CENTER_OF_MASS_AXIS_HALF_LENGTH = 150.0

PRINCIPAL_AXIS_ROTATION_ARC_TESSELATION = 64

GESTURE_TEXT_ENABLED = False
GESTURE_TEXT_OFFSET = Gf.Vec2f(0.0, -10.0)
GESTURE_TEXT_FONT_SIZE = 15
GESTURE_TEXT_FONT_COLOR = cl("#ADF138")

MASS_INFO_TEXT_ENTRY_PRIM_PATH = 0
MASS_INFO_TEXT_ENTRY_MASS = 1
MASS_INFO_TEXT_ENTRY_CENTER_OF_MASS = 2
MASS_INFO_TEXT_ENTRY_PRINCIPAL_AXIS = 3
MASS_INFO_TEXT_ENTRY_DIAGONAL_INERTIA = 4
MASS_INFO_TEXT_ENTRY_NUM = 5

MASS_INFO_TEXT_BOX_BACKGROUND_COLOR = cl("#1E212399")
MASS_INFO_TEXT_MARGIN = Gf.Vec2f(20.0, 20.0)
MASS_INFO_TEXT_FONT_SIZE = 14
MASS_INFO_TEXT_FONT_LINE_SPAN = MASS_INFO_TEXT_FONT_SIZE + UI_FONT_PADDING * 2
MASS_INFO_TEXT_FONT_COLOR = cl("#ADF138")
MASS_INFO_TEXT_FONT_COLOR_HIGHLIGHT = cl("#FEFFFC")
MASS_INFO_TEXT_FONT_COLOR_WARNING = cl("#FFFF38")
MASS_INFO_TEXT_BOX_WIDTH = 540.0

INFO_BOX_NUM = 2
INFO_BOX_MASS_VIEW = 0
INFO_BOX_MASS_EDIT = 1

DEBUG_USD_UPDATE_FRAME_PROFILING = False

if DEBUG_USD_UPDATE_FRAME_PROFILING:
    import cProfile as profile
    import pstats
    import re
    profiling_instance = profile.Profile()

# This calculates dimensions if the body is a cuboid and matches how the PhysX debug visualization works.
def physx_diagonal_inertia_to_axial_mass_dimensions(mass, diagonal_inertia):
    scale_mod = 0.0 if mass <= 0.0 else 6.0 / mass
    dimensions = Gf.Vec3f(
        math.sqrt(scale_mod * math.fabs(-diagonal_inertia[0] + diagonal_inertia[1] + diagonal_inertia[2])),
        math.sqrt(scale_mod * math.fabs(diagonal_inertia[0] - diagonal_inertia[1] + diagonal_inertia[2])),
        math.sqrt(scale_mod * math.fabs(diagonal_inertia[0] + diagonal_inertia[1] - diagonal_inertia[2]))
    )
    return dimensions

# This calculates the inertia if the body is a cuboid and matches how the PhysX debug visualization works.
def physx_axial_mass_dimensions_to_diagonal_inertia(mass, dimensions):
    # Square the dimensions so we only have to do it once.
    diagonal = Gf.Vec3f(dimensions[0] * dimensions[0], dimensions[1] * dimensions[1], dimensions[2] * dimensions[2])

    # By some simple algebra, we get:
    # diagonal inertia base = Gf.Vec3f((diagonal[1] + diagonal[2]) / 2, (diagonal[0] + diagonal[2]) / 2, (diagonal[0] + diagonal[1]) / 2)

    # and with the modifiers, we get
    diagonal_inertia_modifier = mass / 12.0
    diagonal_inertia = Gf.Vec3f((diagonal[1] + diagonal[2]) * diagonal_inertia_modifier, (diagonal[0] + diagonal[2]) * diagonal_inertia_modifier, (diagonal[0] + diagonal[1]) * diagonal_inertia_modifier)

    return diagonal_inertia

# Handles hover highlighting etc. for manipulation gestures.
class PhysicsMassEditHoverGesture(PhysicsHoverGesture):
    def __init__(self, manipulator, gesture, direction):
        super().__init__(manipulator, gesture * AXIS_DIRECTION_NUM + direction)
        self._gesture = gesture

    def on_began(self) -> bool:
        if not super().on_began():
            return False
        self._manipulator.update_info_text()
        if GESTURE_TEXT_ENABLED:
            self._manipulator.update_gesture_text(self.sender.gesture_payload.ray_closest_point)
        return True

    def on_changed(self):
        super().on_changed()
        if GESTURE_TEXT_ENABLED:
            self._manipulator.update_gesture_text(self.sender.gesture_payload.ray_closest_point)

    def on_ended(self):
        super().on_ended()
        self._manipulator.update_info_text()
        if GESTURE_TEXT_ENABLED:
            self._manipulator.update_gesture_text(self.sender.gesture_payload.ray_closest_point)

# Scales the axial mass.
class PhysicsMassEditGestureScale(PhysicsDragGesture):
    def __init__(self, manipulator, direction):
        super().__init__(manipulator, PhysicsMassDistributionEditManipulator.get_toggle_group_by_gesture_direction(GESTURE_SCALE, direction))
        self._direction = direction
        self._origin = None
        self._gesture = GESTURE_SCALE

    def on_began(self) -> bool:
        if not super().on_began():
            return False
        self._manipulator.update_info_text()
        return True

    def on_changed(self):
        if get_active_gesture() is not self:
            return
        super().on_changed()
        # The current transform of our scaling handle.
        transform = self._manipulator._transform_pose.transform * self._manipulator._transform_center_of_mass.transform * self._manipulator._transform_principal_rotate.transform * self._manipulator._shape_transform_scale_inertia_handle[self._direction].transform
        axis = direction_to_axis(self._direction)
        position = Gf.Vec4f(0.0, 0.0, 0.0, 1.0) * Gf.Matrix4f(*transform)
        offset_dir = Gf.Vec4f()
        for n in range(3):
            offset_dir[n] = (self.sender.gesture_payload.ray_closest_point[n] - position[n])
        offset_dir[3] = 0.0

        normal_dir = Gf.Vec4f(0.0)
        normal_dir[axis] = 1.0 if mod(self._direction, 2) == 0.0 else -1.0
        normal_dir = normal_dir * Gf.Matrix4f(*transform)

        # Finds the offset and normals in view space.
        offset_view = offset_dir * Gf.Matrix4f(*self.sender.scene_view.view)
        normal_view = normal_dir * Gf.Matrix4f(*self.sender.scene_view.view)

        # Make offset parallel to the normal on screen.
        offset_view_screen = Gf.Vec2f(offset_view[0], offset_view[1])
        normal_view_screen = Gf.Vec2f(normal_view[0], normal_view[1])
        normal_view_screen = normal_view_screen.GetNormalized()
        n_dot_o_screen = normal_view_screen.GetDot(offset_view_screen)
        offset_view_screen = normal_view_screen * n_dot_o_screen
        offset_view[0] = offset_view_screen[0]
        offset_view[1] = offset_view_screen[1]

        # Pull offset towards the screen until it becomes fully parallel with normal but with the proper dimensions.
        # This is essentially |offset| / cos(offset to normal).
        offset_mag_sq = (offset_view[0]*offset_view[0] + offset_view[1]*offset_view[1] + offset_view[2]*offset_view[2])
        dot_on_view = offset_view.GetDot(normal_view)

        # Prevent divide by zero.
        scale_inertia_modifier = 0.0 if dot_on_view == 0.0 else 2.0 * offset_mag_sq / dot_on_view

        # Apply snap if relevant.
        if self._manipulator._snap_settings_listener.snap_enabled and self._manipulator._snap_settings_listener.snap_scale > 0.0:
            scale_inertia_modifier = round(scale_inertia_modifier / self._manipulator._snap_settings_listener.snap_scale) * self._manipulator._snap_settings_listener.snap_scale
        else:
            scale_inertia_modifier *= DRAG_SMOOTHNESS_MODIFIER

        self._manipulator._scale_inertia[axis] += scale_inertia_modifier

        # Prevent from going into negative scale.
        self._manipulator._scale_inertia[axis] = max(FLOAT_EPSILON, self._manipulator._scale_inertia[axis])
        self._manipulator.recompute_diagonal_intertia()

        # Update transforms based on our new values.
        self._manipulator.rebuild_inertia_scale()
        self._manipulator.rebuild_shape_transforms()

        if GESTURE_TEXT_ENABLED:
            self._manipulator.update_gesture_text(self.sender.gesture_payload.ray_closest_point)

        self._manipulator.update_info_text()

    def on_ended(self):
        super().on_ended()

        self._manipulator.recompute_diagonal_intertia()

        self._manipulator.write_to_usd([UsdPhysics.Tokens.physicsDiagonalInertia], [self._manipulator._body_diagonal_inertia])

        self._manipulator.rebuild_inertia_scale()
        self._manipulator.rebuild_shape_transforms()

        if GESTURE_TEXT_ENABLED:
            self._manipulator.update_gesture_text()

        self._manipulator.update_info_text()

# Moves around the center of mass.
class PhysicsMassEditGestureTranslate(PhysicsDragGesture):
    def __init__(self, manipulator, axis):
        super().__init__(manipulator, toggle_group=PhysicsMassDistributionEditManipulator.get_toggle_group_by_gesture_axis(GESTURE_TRANSLATE, axis))
        self._axis = axis
        self._translate_offset = None
        self._initial_position = None
        self._gesture = GESTURE_TRANSLATE

    def _set_translation(self):
        transform = self._manipulator._transform_pose.transform * self._manipulator._transform_center_of_mass.transform * self._manipulator._transform_principal_rotate.transform * self._manipulator._shape_transform_axis[self._axis].transform
        pose_rotate = Gf.Matrix4f(*self._manipulator._transform_pose.transform).GetOrthonormalized().ExtractRotationMatrix()
        pose_rotate = Gf.Matrix4f(pose_rotate, Gf.Vec3f(0.0))
        translate_direction = Gf.Vec4f(*AXIS_VECTOR[self._axis],0.0) * Gf.Matrix4f(*self._manipulator._transform_principal_rotate.transform)
        position = Gf.Vec4f(0.0, 0.0, 0.0, 1.0) * Gf.Matrix4f(*transform)
        offset_dir = Gf.Vec4f()
        for n in range(3):
            offset_dir[n] = (self.sender.gesture_payload.ray_closest_point[n]) - position[n]
        offset_dir[3] = 0.0
        offset_dir = offset_dir * pose_rotate.GetInverse()
        if self._translate_offset is None:
            self._translate_offset = offset_dir.GetDot(translate_direction)
        else:
            translate_modifier = offset_dir.GetDot(translate_direction) - self._translate_offset

            if self._manipulator._snap_settings_listener.snap_enabled:
                if self._axis == AXIS_X and self._manipulator._snap_settings_listener.snap_move_x > 0.0:
                    translate_modifier = round(translate_modifier /  self._manipulator._snap_settings_listener.snap_move_x) * self._manipulator._snap_settings_listener.snap_move_x
                elif self._axis == AXIS_Y and self._manipulator._snap_settings_listener.snap_move_y > 0.0:
                    translate_modifier = round(translate_modifier /  self._manipulator._snap_settings_listener.snap_move_y) * self._manipulator._snap_settings_listener.snap_move_y
                elif self._axis == AXIS_Z and self._manipulator._snap_settings_listener.snap_move_z > 0.0:
                    translate_modifier = round(translate_modifier /  self._manipulator._snap_settings_listener.snap_move_z) * self._manipulator._snap_settings_listener.snap_move_z
            else:
                translate_modifier *= DRAG_SMOOTHNESS_MODIFIER

            self._manipulator._body_center_of_mass += Gf.Vec3f(translate_direction[0], translate_direction[1], translate_direction[2]) * translate_modifier

        if GESTURE_TEXT_ENABLED:
            self._manipulator.update_gesture_text(self.sender.gesture_payload.ray_closest_point)

        self._manipulator.recompute_diagonal_intertia()

    def on_began(self):
        if not super().on_began():
            return False
        self._manipulator.update_info_text()
        self._set_translation()
        return True

    def on_changed(self):
        if get_active_gesture() is not self:
            return
        super().on_changed()
        self._set_translation()
        self._manipulator.rebuild_center_of_mass_translate()
        self._manipulator.rebuild_shape_transforms()
        self._manipulator.update_info_text()

    def on_ended(self):
        super().on_ended()
        self._manipulator.update_info_text()

        if GESTURE_TEXT_ENABLED:
            self._manipulator.update_gesture_text()

        self._translate_offset = None

        self._manipulator.recompute_diagonal_intertia()
        self._manipulator.write_to_usd([UsdPhysics.Tokens.physicsCenterOfMass, UsdPhysics.Tokens.physicsDiagonalInertia, UsdPhysics.Tokens.physicsPrincipalAxes], [components_divide(self._manipulator._body_center_of_mass, self._manipulator._transform_scale), self._manipulator._body_diagonal_inertia, self._manipulator._body_principal_axes])

# Rotates the principal axes.
class PhysicsMassEditGestureRotate(PhysicsDragGesture):
    def __init__(self, manipulator, axis):
        super().__init__(manipulator, toggle_group=PhysicsMassDistributionEditManipulator.get_toggle_group_by_gesture_axis(GESTURE_ROTATE, axis))
        self._axis = axis
        self._angle_origin = None
        self._angle_moved = None
        self._gesture = GESTURE_ROTATE

    def _set_rotation(self):
        transform = self._manipulator._transform_pose.transform * self._manipulator._transform_center_of_mass.transform * self._manipulator._transform_principal_rotate.transform * self._manipulator._shape_transform_circle[self._axis].transform
        axis = AXIS_VECTOR[self._axis]
        position = Gf.Vec4f(0.0, 0.0, 0.0, 1.0) * Gf.Matrix4f(*transform)
        offset_dir = Gf.Vec4f()
        for n in range(3):
            offset_dir[n] = (self.sender.gesture_payload.ray_closest_point[n] - position[n])
        offset_dir[3] = 0.0
        offset_dir = offset_dir.GetNormalized()
        offset_dir_local = offset_dir * Gf.Matrix4f(*transform).GetInverse()
        offset_dir_local = offset_dir_local.GetNormalized()

        angle = math.acos(offset_dir_local[0])
        if offset_dir_local[1] < 0:
            angle = - angle
        angle = Gf.RadiansToDegrees(angle)

        if self._angle_origin is None:
            self._angle_origin = angle
            self._angle_moved = 0.0
        else:
            if self._manipulator._snap_settings_listener.snap_enabled:
                self._angle_moved = angle - self._angle_origin + self._angle_moved
                self._angle_moved += self._manipulator._snap_settings_listener.snap_rotate / 2
                self._angle_moved -= math.fmod(self._angle_moved, self._manipulator._snap_settings_listener.snap_rotate)
            else:
                self._angle_moved += DRAG_SMOOTHNESS_MODIFIER * (angle - self._angle_origin)

            self._manipulator._rotation_modifier = axis_angle_to_quaternion(axis, math.radians(self._angle_moved))

        if GESTURE_TEXT_ENABLED:
            self._manipulator.update_gesture_text(self.sender.gesture_payload.ray_closest_point)

    def on_began(self):
        if not super().on_began():
            return False
        self._manipulator.update_info_text()
        self._set_rotation()
        return True

    def on_changed(self):
        if get_active_gesture() is not self:
            return
        super().on_changed()
        self._set_rotation()
        self._manipulator.rebuild_principal_rotate()
        self._manipulator.recompute_diagonal_intertia()
        self._manipulator.rebuild_shape_transforms()
        self._manipulator.update_info_text()

    def on_ended(self):
        super().on_ended()
        self._manipulator._body_principal_axes = self._manipulator._body_principal_axes*Gf.Quatf(self._manipulator._rotation_modifier)
        self._manipulator._rotation_modifier = Gf.Quatf.GetIdentity()
        self._angle_origin = None
        self._manipulator.update_info_text()

        if GESTURE_TEXT_ENABLED:
            self._manipulator.update_gesture_text()

        self._manipulator.write_to_usd([UsdPhysics.Tokens.physicsPrincipalAxes, UsdPhysics.Tokens.physicsDiagonalInertia], [self._manipulator._body_principal_axes, self._manipulator._body_diagonal_inertia])


class PhysxPropertyQueryObject():
    class Status:
        IN_PROGRESS = 0
        COMPLETE = 1
        FAIL = 2

    def __init__(self, manipulator):
        self.status = PhysxPropertyQueryObject.Status.IN_PROGRESS
        self.body_mass = -1.0
        self.body_diagonal_inertia = Gf.Vec3f(-1.0)
        self.body_center_of_mass = Gf.Vec3f(0.0)
        self.body_principal_axes = Gf.Quatf().GetIdentity()
        self.manipulator = manipulator
        self.dirty = False

class PhysicsMassManipulator(PhysicsManipulator):
    def __init__(self, viewport_overlay, prim):
        super().__init__(viewport_overlay)
        self._prim = prim
        self._stage = None
        self._active = False

        self._physx_property_query_object = None
        self._physx_property_query_populate_on_finish = False

        self._prim_xform = None

        self._body_mass = 0.0
        self._body_diagonal_inertia = Gf.Vec3f(0.0)
        self._body_principal_axes = Gf.Quatf.GetIdentity()
        # Note: this is scaled to world space, so when reading from and writing to USD, it should be adjusted by xform scale.
        self._body_center_of_mass = Gf.Vec3f(0.0)

        self._scale_inertia = None
        self._rotation_modifier = Gf.Quatf.GetIdentity()

        self._transform_scale = Gf.Vec3f(1.0)
        self._transform_pose = None
        self._transform_center_of_mass=None
        self._transform_principal_rotate = None

        self._shape_transform_center_of_mass_visualization=None
        self._shape_transform_inertia_distribution_box=None

    def get_physx_property_query_status(self):
        if self._physx_property_query_object is not None:
            return self._physx_property_query_object.status
        return PhysxPropertyQueryObject.Status.IN_PROGRESS

    def recompute_diagonal_intertia(self):
        self._body_diagonal_inertia = physx_axial_mass_dimensions_to_diagonal_inertia(self._body_mass, self._scale_inertia)

    def rebuild_principal_rotate(self):
        rotate_matrix = Gf.Matrix4f()
        rotate_matrix.SetRotate(self._body_principal_axes * self._rotation_modifier)
        self._transform_principal_rotate.transform = sc.Matrix44(*rotate_matrix[0], *rotate_matrix[1], *rotate_matrix[2], *rotate_matrix[3])

    def rebuild_center_of_mass_translate(self):
        self._transform_center_of_mass.transform = sc.Matrix44.get_translation_matrix(*self._body_center_of_mass)

    def rebuild_inertia_scale(self):
        self._scale_inertia = physx_diagonal_inertia_to_axial_mass_dimensions(self._body_mass, self._body_diagonal_inertia)

    def rebuild_shape_transforms(self):
        self._shape_transform_inertia_distribution_box.transform = sc.Matrix44.get_scale_matrix(*self._scale_inertia)

    def update_info_text(self):
        return

    def refresh(self):
        super().refresh()

    def populate(self, force_update_all = False):
        usd_context = omni.usd.get_context()
        if not usd_context or not self._stage or usd_context.get_stage() != self._stage:
            if self._active:
                self.invalidate()
            return

        if not self._prim or not self._prim.IsValid() or not self._prim.HasAPI(UsdPhysics.RigidBodyAPI):
            if self._active:
                self.invalidate()
            return

        if not self._active:
            self.invalidate()
            return

        # Helper functions for sanity checks against USD and PhysX property returns.
        def validate_output_array(output, desired_length):
            if output is None:
                return False
            if not isinstance(output, collections.abc.Sized) or len(output) != desired_length:
                return False
            return True

        def validate_output_quaternion(output):
            if output is None:
                return False
            if (not isinstance(output, Gf.Quatd) and not isinstance(output, Gf.Quatf) and not isinstance(output, Gf.Quaternion)):
                return False
            return True

        def validate_output_float(output):
            if output is None:
                return False
            if not isinstance(output, float):
                return False
            return True

        xform_cache = self.get_viewport_overlay().get_xform_cache()
        self._prim_xform = xform_cache.GetLocalToWorldTransform(self._prim)

        self._transform_scale = Gf.Vec3f(Gf.Transform(self._prim_xform).GetScale())
        xform_mat_no_scale = self._prim_xform.RemoveScaleShear()

        self._transform_pose.transform = sc.Matrix44(*xform_mat_no_scale[0], *xform_mat_no_scale[1], *xform_mat_no_scale[2], *xform_mat_no_scale[3])

        body_mass_new = None
        body_diagonal_inertia_new = None
        body_center_of_mass_new = None
        body_principal_axes_new = None

        if self._physx_property_query_object is None or force_update_all:
            # Fetch mass properties from PhysX.
            self._physx_property_query_object = PhysxPropertyQueryObject(self)

            # Weak reference that we pass to the query functions to allow them to track if they are still relevant when finished.
            # If a new query is made or the manipulator deactivated or removed in the meantime, the reference will point to None.
            physx_property_query_object_ref = weakref.ref(self._physx_property_query_object)

            # Only run populate at the end of the qyery in case the results are not returned immediately.
            self._physx_property_query_populate_on_finish = False

            def rigid_info_received(rigid_info : PhysxPropertyQueryRigidBodyResponse):
                nonlocal physx_property_query_object_ref
                physx_property_query_object = physx_property_query_object_ref()
                if (physx_property_query_object is None):
                    return

                physx_property_query_object.dirty = True

                if rigid_info.result == PhysxPropertyQueryResult.VALID:
                    if validate_output_float(rigid_info.mass):
                        physx_property_query_object.body_mass = rigid_info.mass
                    if validate_output_array(rigid_info.inertia, 3):
                        physx_property_query_object.body_diagonal_inertia = Gf.Vec3f(*rigid_info.inertia)
                    if validate_output_array(rigid_info.center_of_mass, 3):
                        physx_property_query_object.body_center_of_mass = Gf.Vec3f(*rigid_info.center_of_mass)
                    if validate_output_array(rigid_info.principal_axes, 4):
                        physx_property_query_object.body_principal_axes = Gf.Quatf(rigid_info.principal_axes[3], rigid_info.principal_axes[0], rigid_info.principal_axes[1], rigid_info.principal_axes[2])
                else:
                    physx_property_query_object.status = PhysxPropertyQueryObject.Status.FAIL
                    carb.log_error(f"PhysX rigid body property query for {physx_property_query_object.manipulator._prim.GetPrimPath()} returned with error: {rigid_info.result.name}")



            def property_query_finished():
                nonlocal physx_property_query_object_ref
                physx_property_query_object = physx_property_query_object_ref()

                if physx_property_query_object is None:
                    return

                assert physx_property_query_object.dirty, "Physx property query error - finished without having received info from a rigid body."
                assert physx_property_query_object == self._physx_property_query_object, "Physx property query error - a new query was made without the current being deleted."

                if physx_property_query_object.status == PhysxPropertyQueryObject.Status.FAIL:
                    return

                physx_property_query_object.status = PhysxPropertyQueryObject.Status.COMPLETE

                if self._physx_property_query_populate_on_finish:
                    self.populate()

            body_path = PhysicsSchemaTools.sdfPathToInt(self._prim.GetPrimPath())
            stage_id = UsdUtils.StageCache.Get().Insert(self._stage).ToLongInt()

            get_physx_property_query_interface().query_prim(stage_id = stage_id,
                                                            prim_id = body_path,
                                                            query_mode = PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS,
                                                            finished_fn = property_query_finished,
                                                            rigid_body_fn = rigid_info_received,
                                                            collider_fn = None,
                                                            timeout_ms = 60000) # Timeout after 1 minute.

            if self._physx_property_query_object.status == PhysxPropertyQueryObject.Status.IN_PROGRESS:
                self._physx_property_query_populate_on_finish = True

        # Flags whether we need to update the visual output.
        has_updates = force_update_all

        if self._physx_property_query_object.dirty:
            self._physx_property_query_object.dirty = False

            if self._physx_property_query_object.status == PhysxPropertyQueryObject.Status.COMPLETE:
                body_mass_new = self._physx_property_query_object.body_mass
                body_diagonal_inertia_new = self._physx_property_query_object.body_diagonal_inertia
                body_center_of_mass_new = self._physx_property_query_object.body_center_of_mass
                body_principal_axes_new = self._physx_property_query_object.body_principal_axes

        # Next read any USD attributes if present. These have higher priority than what we get from PhysX since they may be more current.
        active_gesture = get_active_gesture()
        current_gesture = -1
        if active_gesture is not None and active_gesture._manipulator == self:
            current_gesture = active_gesture._gesture

        mass_api = UsdPhysics.MassAPI(self._prim)
        if (force_update_all or current_gesture != GESTURE_ROTATE):
            attribute = mass_api.GetPrincipalAxesAttr()
            if attribute.HasAuthoredValue():
                value = attribute.Get()
                if validate_output_quaternion(value):
                    if (value.GetReal() != 0.0 or value.GetImaginary() != Gf.Vec3f(0.0)):
                        body_principal_axes_new = value
                else:
                    carb.log_error(f"Invalid USD value for principal axis: {value}")

            if body_principal_axes_new is not None and body_principal_axes_new != self._body_principal_axes:
                self._body_principal_axes = body_principal_axes_new
                self.rebuild_principal_rotate()
                has_updates = True
            elif force_update_all:
                self.rebuild_principal_rotate()

        if (force_update_all or current_gesture != GESTURE_TRANSLATE):
            attribute = mass_api.GetCenterOfMassAttr()
            if attribute.HasAuthoredValue():
                value = attribute.Get()
                if validate_output_array(value, 3):
                    if (value[0] != -inf and value[1] != -inf and value[2] != -inf):
                        body_center_of_mass_new = components_multiply(self._transform_scale, value)
                else:
                    carb.log_error(f"Invalid USD value for center of mass: {value}")

            if body_center_of_mass_new is not None and body_center_of_mass_new != self._body_center_of_mass:
                self._body_center_of_mass = body_center_of_mass_new
                self.rebuild_center_of_mass_translate()
                has_updates = True
            elif force_update_all:
                self.rebuild_center_of_mass_translate()

        if (force_update_all or current_gesture != GESTURE_SCALE):
            attribute = mass_api.GetMassAttr()
            if attribute.HasAuthoredValue():
                value = attribute.Get()
                if validate_output_float(value):
                    if value != 0.0:
                        body_mass_new = value
                else:
                    carb.log_error(f"Invalid USD value for mass: {value}")

            attribute = mass_api.GetDiagonalInertiaAttr()
            if attribute.HasAuthoredValue():
                value = attribute.Get()
                if validate_output_array(value, 3):
                    if (value[0] != 0.0 or value[0] != 0.0 or value[0] != 0.0):
                        body_diagonal_inertia_new = Gf.Vec3f(*value)
                else:
                    carb.log_error(f"Invalid USD value for diagonal inertia: {value}")

            if (body_diagonal_inertia_new is not None and body_mass_new is not None and
                (body_diagonal_inertia_new != self._body_diagonal_inertia or body_mass_new != self._body_mass)):
                self._body_diagonal_inertia = body_diagonal_inertia_new
                self._body_mass = body_mass_new
                self.rebuild_inertia_scale()
                has_updates = True
            elif force_update_all:
                self.rebuild_inertia_scale()

        if has_updates:
            self.rebuild_shape_transforms()
            self.update_info_text()

    def _make_shapes(self):

        def draw_box(color, thickness):
            """
            sc.Curve([[0.5, -0.5, -0.5],
                    [-0.5, -0.5, -0.5],
                    [-0.5, -0.5, 0.5],
                    [0.5, -0.5, 0.5],
                    [0.5, 0.5, 0.5],
                    [0.5, 0.5, -0.5],
                    [0.5, -0.5, -0.5],
                    [0.5, -0.5, 0.5],
                    [-0.5, -0.5, 0.5],
                    [-0.5, 0.5, 0.5],
                    [0.5, 0.5, 0.5],
                    [0.5, 0.5, -0.5],
                    [-0.5, 0.5, -0.5],
                    [-0.5, -0.5, -0.5],
                    [-0.5, 0.5, -0.5],
                    [-0.5, 0.5, 0.5]],
                    thicknesses=[thickness],
                    colors=[color],
                    curve_type=sc.Curve.CurveType.LINEAR)
            """
            sc.Line([-0.5, -0.5, -0.5], [0.5, -0.5, -0.5], color=color, thickness=thickness)
            sc.Line([-0.5, 0.5, -0.5], [0.5, 0.5, -0.5], color=color, thickness=thickness)
            sc.Line([-0.5, -0.5, -0.5], [-0.5, 0.5, -0.5], color=color, thickness=thickness)
            sc.Line([0.5, -0.5, -0.5], [0.5, 0.5, -0.5], color=color, thickness=thickness)
            sc.Line([-0.5, -0.5, 0.5], [0.5, -0.5, 0.5], color=color, thickness=thickness)
            sc.Line([-0.5, 0.5, 0.5], [0.5, 0.5, 0.5], color=color, thickness=thickness)
            sc.Line([-0.5, -0.5, 0.5], [-0.5, 0.5, 0.5], color=color, thickness=thickness)
            sc.Line([0.5, -0.5, 0.5], [0.5, 0.5, 0.5], color=color, thickness=thickness)
            sc.Line([-0.5, -0.5, 0.5], [-0.5, -0.5, -0.5], color=color, thickness=thickness)
            sc.Line([0.5, -0.5, 0.5], [0.5, -0.5, -0.5], color=color, thickness=thickness)
            sc.Line([-0.5, 0.5, 0.5], [-0.5, 0.5, -0.5], color=color, thickness=thickness)
            sc.Line([0.5, 0.5, 0.5], [0.5, 0.5, -0.5], color=color, thickness=thickness)

        self._transform_pose = sc.Transform(transform=sc.Matrix44.get_scale_matrix(0.0, 0.0, 0.0))
        with self._transform_pose:
            self._transform_center_of_mass = sc.Transform()
            with self._transform_center_of_mass:
                self._transform_principal_rotate = sc.Transform()
                with self._transform_principal_rotate:
                    self._shape_transform_center_of_mass_visualization = sc.Transform(scale_to=sc.Space.SCREEN, look_at=sc.Transform.LookAt.CAMERA)
                    with self._shape_transform_center_of_mass_visualization:
                        size = 45.0
                        subdivisions = 4
                        steps = 3
                        arc_length = 0.75
                        for step in range(steps):
                            for n in range(subdivisions):
                                length = (2 + step) / (steps + 1)
                                angle = math.radians((float(n) + step / 2 - arc_length * 0.5 + 0.5) * 360.0 / subdivisions)
                                angle_end = angle + arc_length * math.radians(360.0 / subdivisions)
                                sc.Arc(size * length, sector=0, begin=angle, end=angle_end, tesselation=(4 + step), color=MASS_DISTRIBUTION_BOX_COLOR, wireframe=True, thickness=1)

                        sc.Line([-size / (steps + 1), 0.0, 0.0], [size / (steps + 1), 0.0, 0.0], color=MASS_DISTRIBUTION_BOX_COLOR, thickness=MASS_DISTRIBUTION_BOX_LINE_THICKNESS)
                        sc.Line([0.0, -size / (steps + 1), 0.0], [0.0, size / (steps + 1), 0.0], color=MASS_DISTRIBUTION_BOX_COLOR, thickness=MASS_DISTRIBUTION_BOX_LINE_THICKNESS)

                    self._shape_transform_inertia_distribution_box = sc.Transform()
                    with self._shape_transform_inertia_distribution_box:
                        draw_box(MASS_DISTRIBUTION_BOX_COLOR, MASS_DISTRIBUTION_BOX_LINE_THICKNESS)

    def on_build(self):
        super().on_build()

        if self._viewport_overlay is None:
            self._active = False
            return

        if not self._prim or not self._prim.IsValid():
            self._prim = None
            self._active = False
            return

        usd_context = omni.usd.get_context()
        if not usd_context:
            self._active = False
            return

        self._stage=usd_context.get_stage()
        if not self._stage:
            self._active = False
            return

        if not self._prim.HasAPI(UsdPhysics.RigidBodyAPI):
            self._active = False
            return

        self._active = True

        self._rotation_modifier = Gf.Quatf.GetIdentity()

        self._scale_inertia = Gf.Vec3f(0.0)
        self._translate = Gf.Vec3f(0.0)

        self._make_shapes()
        self.populate(True)
        self.refresh()

    def destroy(self):
        self._prim = None
        self._physx_property_query_object = None
        super().destroy()

class PhysicsMassDistributionEditManipulator(PhysicsMassManipulator):
    def __init__(self, viewport_overlay, prim):
        super().__init__(viewport_overlay, prim)

        self._snap_settings_listener = None
        self._edit_enabled = True
        self._writing_to_usd = False

        if GESTURE_TEXT_ENABLED:
            self._gesture_text = None
            self._gesture_text_transform = None

        self._shape_transform_scale_inertia_handle = []

        self._shape_transform_axis = []
        self._shape_transform_circle = []
        if VISUAL_MASS_DISTRIBUTION_RULER_SUBDIVISION_MARKERS:
            self._shape_transform_circle_ruler = []

    @staticmethod
    def get_toggle_group_by_gesture_direction(gesture, direction):
        return gesture * AXIS_DIRECTION_NUM + direction

    @staticmethod
    def get_toggle_group_by_gesture_axis(gesture, axis):
        return gesture * AXIS_DIRECTION_NUM + axis * 2

    class ApplyMassAPICommand(omni.kit.commands.Command):
        def __init__(self, prim, mass, center_of_mass, diagonal_inertia, principal_axes):
            super().__init__()
            self._prim = prim
            self._mass = mass
            self._center_of_mass = center_of_mass
            self._diagonal_inertia = diagonal_inertia
            self._principal_axes = principal_axes

        def do(self):
            massAPI = UsdPhysics.MassAPI.Apply(self._prim)
            massAPI.GetMassAttr().Set(self._mass)
            massAPI.GetCenterOfMassAttr().Set(self._center_of_mass)
            massAPI.GetDiagonalInertiaAttr().Set(self._diagonal_inertia)
            massAPI.GetPrincipalAxesAttr().Set(self._principal_axes)
            property_window = omni.kit.window.property.get_window()
            if property_window:
                property_window._window.frame.rebuild()

        def undo(self):
            self._prim.RemoveAPI(UsdPhysics.MassAPI)
            utils.removeAPISchemaProperties(UsdPhysics.MassAPI, self._prim)
            property_window = omni.kit.window.property.get_window()
            if property_window:
                property_window._window.frame.rebuild()

    def write_to_usd(self, attribute_names, values):
        if not self._prim or not self._prim.IsValid():
            return

        self._writing_to_usd = True
        omni.kit.undo.begin_group()
        if not self._prim.HasAPI(UsdPhysics.MassAPI):
            omni.kit.commands.execute('ApplyMassAPICommand', prim=self._prim, mass=self._body_mass,
                center_of_mass=components_divide(self._body_center_of_mass, self._transform_scale), diagonal_inertia=self._body_diagonal_inertia,
                principal_axes=self._body_principal_axes)
        else:
            for n in range(min(len(attribute_names), len(values))):
                attribute = self._prim.GetAttribute(attribute_names[n])
                omni.kit.commands.execute('ChangePropertyCommand', prop_path=attribute.GetPath(), value=values[n],
                    prev=attribute.Get() if attribute else None)
        omni.kit.undo.end_group()
        self._writing_to_usd = False

    def update_gesture_text(self, gesture_coords = None):
        if not GESTURE_TEXT_ENABLED:
            return

        if not self._edit_enabled:
            return

        if self._gesture_text_transform is not None:
            if gesture_coords is None:
                self._gesture_text_transform.visible = False
                return

            active_hover = get_active_hover()
            active_gesture = get_active_gesture()
            gesture = -1

            # Active gesture has precedence over hovering.
            if active_gesture is not None:
                gesture = active_gesture._gesture
            elif active_hover is not None and isinstance(active_hover, PhysicsMassEditHoverGesture):
                gesture = active_hover._gesture

            if gesture == -1:
                self._gesture_text_transform.visible = False
                return

            self._gesture_text_transform.visible = True
            self._gesture_text_transform.transform = sc.Matrix44.get_translation_matrix(*gesture_coords)

            if gesture == GESTURE_SCALE:
                self._gesture_text.text = ""
            elif gesture == GESTURE_ROTATE:
                matrix = Gf.Matrix4d()
                matrix.SetRotate(self._body_principal_axes*self._rotation_modifier)
                decomposed = Gf.Rotation.DecomposeRotation3(matrix, Gf.Vec3d(*AXIS_VECTOR_X), Gf.Vec3d(*AXIS_VECTOR_Y), Gf.Vec3d(*AXIS_VECTOR_Z), 1.0)
                self._gesture_text.text = axis_to_string(active_hover._axis)+": "+ float_to_string(math.degrees(decomposed[active_hover._axis]))+"°"
            elif gesture == GESTURE_TRANSLATE:
                self._gesture_text.text = float_to_string(self._body_center_of_mass[0]) + "; " + float_to_string(self._body_center_of_mass[1]) + "; " + float_to_string(self._body_center_of_mass[2])

    def update_info_text(self):
        super().update_info_text()
        self._viewport_overlay.refresh_info_box(INFO_BOX_MASS_EDIT)

    def refresh(self):
        super().refresh()

    def rebuild_principal_rotate(self):
        super().rebuild_principal_rotate()

        if not self._edit_enabled:
            return

        if VISUAL_MASS_DISTRIBUTION_RULER_SUBDIVISION_MARKERS:
            decomposed = Gf.Rotation.DecomposeRotation3(Gf.Matrix4d(*self._transform_principal_rotate.transform), Gf.Vec3d(*AXIS_VECTOR_X), Gf.Vec3d(*AXIS_VECTOR_Y), Gf.Vec3d(*AXIS_VECTOR_Z), 1.0)
            self._shape_transform_circle_ruler[AXIS_X].transform =  sc.Matrix44.get_rotation_matrix(-decomposed[0], 0.0, 0.0, False) * sc.Matrix44.get_rotation_matrix(0.0, 90.0, 0.0, True)
            decomposed = Gf.Rotation.DecomposeRotation3(Gf.Matrix4d(*self._transform_principal_rotate.transform), Gf.Vec3d(*AXIS_VECTOR_Y), Gf.Vec3d(*AXIS_VECTOR_Z), Gf.Vec3d(*AXIS_VECTOR_X), 1.0)
            self._shape_transform_circle_ruler[AXIS_Y].transform =  sc.Matrix44.get_rotation_matrix(0.0, -decomposed[0], 0.0, False) * sc.Matrix44.get_rotation_matrix(-90.0, 0.0, 0.0, True)
            decomposed = Gf.Rotation.DecomposeRotation3(Gf.Matrix4d(*self._transform_principal_rotate.transform), Gf.Vec3d(*AXIS_VECTOR_Z), Gf.Vec3d(*AXIS_VECTOR_Y), Gf.Vec3d(*AXIS_VECTOR_X), 1.0)
            self._shape_transform_circle_ruler[AXIS_Z].transform =  sc.Matrix44.get_rotation_matrix(0.0, 0.0, -decomposed[0], False)

    def rebuild_shape_transforms(self):
        super().rebuild_shape_transforms()

        if not self._edit_enabled:
            return

        # Scaling gesture
        for direction in range(AXIS_DIRECTION_NUM):
            axis = direction_to_axis(direction)
            translate_dir = Gf.Vec3f(0.0)
            translate_dir[axis] = 0.5 * self._scale_inertia[axis] if mod(direction, 2) == 0 else -0.5 * self._scale_inertia[axis]

            # Scale handles.
            self._shape_transform_scale_inertia_handle[direction].transform = sc.Matrix44.get_translation_matrix(*translate_dir)

    def set_edit_enabled(self, enabled):
        if enabled != self._edit_enabled:
            self._edit_enabled = enabled
            self.invalidate()

    def _make_shapes(self):
        super()._make_shapes()

        if not self._edit_enabled:
            return

        with self._transform_pose:
            if GESTURE_TEXT_ENABLED:
                self._gesture_text_transform = sc.Transform()
                with self._gesture_text_transform:
                    with sc.Transform(look_at=sc.Transform.LookAt.CAMERA):
                        with sc.Transform(transform=sc.Matrix44.get_translation_matrix(*GESTURE_TEXT_OFFSET, 0.01)):
                            self._gesture_text = sc.Label(
                            " ",
                                alignment=ui.Alignment.CENTER,
                                color=GESTURE_TEXT_FONT_COLOR,
                                size=GESTURE_TEXT_FONT_SIZE
                            )

            self._snap_settings_listener = SnapSettingsListener(
                enabled_setting_path=None,
                move_x_setting_path=snap_c.SNAP_TRANSLATE_SETTING_PATH,
                move_y_setting_path=snap_c.SNAP_TRANSLATE_SETTING_PATH,
                move_z_setting_path=snap_c.SNAP_TRANSLATE_SETTING_PATH,
                rotate_setting_path=snap_c.SNAP_ROTATE_SETTING_PATH,
                scale_setting_path=snap_c.SNAP_SCALE_SETTING_PATH,
                provider_setting_path=snap_c.SNAP_PROVIDER_NAME_SETTING_PATH,
            )

            # Generate data for arrowheads.
            translate_arrow_face_count = 24
            translate_arrow_vertex_indices = []
            translate_arrow_points_front = [[0, 0, CENTER_OF_MASS_AXIS_HALF_LENGTH]]
            translate_arrow_faces_vertex_count = []
            angle = 0.0
            angle_cos = 1.0
            angle_sin = 0.0
            translate_arrow_points_front.append(
                [
                    angle_cos * CENTER_OF_MASS_AXIS_ARROW_RADIUS,
                    angle_sin * CENTER_OF_MASS_AXIS_ARROW_RADIUS,
                    CENTER_OF_MASS_AXIS_HALF_LENGTH - CENTER_OF_MASS_AXIS_ARROW_LENGTH
                ])
            for i in range(translate_arrow_face_count):
                angle += 2.0 * math.pi / translate_arrow_face_count
                angle_cos = math.cos(angle)
                angle_sin = math.sin(angle)
                translate_arrow_points_front.append(
                    [
                        angle_cos * CENTER_OF_MASS_AXIS_ARROW_RADIUS,
                        angle_sin * CENTER_OF_MASS_AXIS_ARROW_RADIUS,
                        CENTER_OF_MASS_AXIS_HALF_LENGTH - CENTER_OF_MASS_AXIS_ARROW_LENGTH
                    ])
                translate_arrow_vertex_indices.append(0)
                translate_arrow_vertex_indices.append(len(translate_arrow_points_front) - 2)
                translate_arrow_vertex_indices.append(len(translate_arrow_points_front) - 1)

                translate_arrow_faces_vertex_count.append(3)

            translate_arrow_points_back = translate_arrow_points_front.copy()
            translate_arrow_points_back[0] = [0, 0, CENTER_OF_MASS_AXIS_HALF_LENGTH - CENTER_OF_MASS_AXIS_ARROW_LENGTH]

            if MASS_DISTRIBUTION_HANDLE_ARROW:
                scale_arrow_face_count = 4
                scale_arrow_vertex_indices = []
                scale_arrow_faces_vertex_count = []
                scale_arrow_points_front = [[0, 0, MASS_DISTRIBUTION_HANDLE_ARROW_SIZE]]
                angle = math.radians(45.0)
                angle_cos = math.cos(angle)
                angle_sin = math.sin(angle)
                scale_arrow_points_front.append(
                    [
                        angle_cos * MASS_DISTRIBUTION_HANDLE_ARROW_SIZE * 0.707,
                        angle_sin * MASS_DISTRIBUTION_HANDLE_ARROW_SIZE * 0.707,
                        0.0
                    ])
                for i in range(scale_arrow_face_count):
                    angle += 2.0 * math.pi / scale_arrow_face_count
                    angle_cos = math.cos(angle)
                    angle_sin = math.sin(angle)
                    scale_arrow_points_front.append(
                        [
                            angle_cos * MASS_DISTRIBUTION_HANDLE_ARROW_SIZE * 0.707,
                            angle_sin * MASS_DISTRIBUTION_HANDLE_ARROW_SIZE * 0.707,
                            0.0
                        ])
                    scale_arrow_vertex_indices.append(0)
                    scale_arrow_vertex_indices.append(len(scale_arrow_points_front) - 2)
                    scale_arrow_vertex_indices.append(len(scale_arrow_points_front) - 1)

                    scale_arrow_faces_vertex_count.append(3)

            self._shape_transform_scale_inertia_handle.clear()

            for direction in range(AXIS_DIRECTION_NUM):

                if direction == AXIS_DIRECTION_X_POS:
                    rotation = Gf.Vec3f(0.0, 90.0, 0.0)
                elif direction == AXIS_DIRECTION_X_NEG:
                    rotation = Gf.Vec3f(0.0, -90.0, 0.0)
                elif direction == AXIS_DIRECTION_Y_POS:
                    rotation = Gf.Vec3f(-90.0, 0.0, 0.0)
                elif direction == AXIS_DIRECTION_Y_NEG:
                    rotation = Gf.Vec3f(90.0, 0.0, 0.0)
                elif direction == AXIS_DIRECTION_Z_POS:
                    rotation = Gf.Vec3f(0.0, 0.0, -90.0)
                elif direction == AXIS_DIRECTION_Z_NEG:
                    rotation = Gf.Vec3f(180.0, 0.0, 90.0)
                mat44_direction_rotate = sc.Matrix44.get_rotation_matrix(*rotation, True)

                with self._transform_center_of_mass:
                    with self._transform_principal_rotate:
                        self._shape_transform_scale_inertia_handle.append(sc.Transform())
                        with self._shape_transform_scale_inertia_handle[direction]:
                            visibility = sc.Transform(visible=False)
                            self.add_to_toggles(VisibilityToggle(visibility), PhysicsMassDistributionEditManipulator.get_toggle_group_by_gesture_direction(GESTURE_SCALE, direction))
                            with visibility:
                                with sc.Transform(scale_to=sc.Space.SCREEN, transform=mat44_direction_rotate):
                                    cross_size = MASS_DISTRIBUTION_HANDLE_SIZE * 4.0
                                    sc.Line([-cross_size * 0.5, 0.0, 0.0], [cross_size * 0.5, 0.0, 0.0], color=MASS_DISTRIBUTION_BOX_COLOR, thickness=MAIN_LINE_THICKNESS)
                                    sc.Line([0.0, -cross_size * 0.5, 0.0], [0.0, cross_size * 0.5, 0.0], color=MASS_DISTRIBUTION_BOX_COLOR, thickness=MAIN_LINE_THICKNESS)

                            if MASS_DISTRIBUTION_HANDLE_RECTANGLE:
                                with sc.Transform(scale_to=sc.Space.SCREEN):
                                    for n in range(MASS_DISTRIBUTION_HANDLE_RECTANGLE_TRAIL_COUNT):
                                        color = MASS_DISTRIBUTION_COLOR + 256 * 256 * 256 * int(MASS_DISTRIBUTION_HANDLE_RECTANGLE_TRAIL_ALPHA * (1.0 - n / (MASS_DISTRIBUTION_HANDLE_RECTANGLE_TRAIL_COUNT)))
                                        z = - 0.65 * MASS_DISTRIBUTION_HANDLE_RECTANGLE_SIZE * (n + 1) / (MASS_DISTRIBUTION_HANDLE_RECTANGLE_TRAIL_COUNT)
                                        size = MASS_DISTRIBUTION_HANDLE_RECTANGLE_SIZE * ( 1.0  - 0.35 * (n + 1 )/ (MASS_DISTRIBUTION_HANDLE_RECTANGLE_TRAIL_COUNT))
                                        with sc.Transform(transform = mat44_direction_rotate):
                                            sc.Line([-size * 0.5, -size * 0.5, z], [size * 0.5, -size * 0.5, z], color=color, thickness=1.0)
                                            sc.Line([size * 0.5, -size * 0.5, z], [size * 0.5, size * 0.5, z], color=color, thickness=1.0)
                                            sc.Line([size * 0.5, size * 0.5, z], [-size * 0.5, size * 0.5, z], color=color, thickness=1.0)
                                            sc.Line([-size * 0.5, size * 0.5, z], [-size * 0.5, -size * 0.5, z], color=color, thickness=1.0)
                                    with sc.Transform(transform = mat44_direction_rotate):
                                        shape = sc.Rectangle(wireframe=True, thickness=HOVER_LINE_THICKNESS, width=MASS_DISTRIBUTION_HANDLE_RECTANGLE_SIZE + HOVER_LINE_THICKNESS * 2, height=MASS_DISTRIBUTION_HANDLE_RECTANGLE_SIZE + HOVER_LINE_THICKNESS, color=COLOR_INVISIBLE)
                                        self.add_to_toggles(HighlightToggle(shape), PhysicsMassDistributionEditManipulator.get_toggle_group_by_gesture_direction(GESTURE_SCALE, direction))
                                        sc.Rectangle(width=MASS_DISTRIBUTION_HANDLE_RECTANGLE_SIZE, height=MASS_DISTRIBUTION_HANDLE_RECTANGLE_SIZE, color=COLOR_AXIS_SHADED[direction_to_axis(direction)])
                                        with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0.0, 0.0, 0.01)):
                                            sc.Rectangle(width=MASS_DISTRIBUTION_HANDLE_RECTANGLE_SIZE, height=MASS_DISTRIBUTION_HANDLE_RECTANGLE_SIZE, color=COLOR_AXIS[direction_to_axis(direction)])

                                # Invisible handle for the actual interaction.
                                with sc.Transform(look_at=sc.Transform.LookAt.CAMERA, scale_to=sc.Space.SCREEN):
                                    sc.Arc(MASS_DISTRIBUTION_HANDLE_RECTANGLE_SIZE / 2 + HOVER_LINE_THICKNESS, color=COLOR_INVISIBLE, gesture=[PhysicsMassEditGestureScale(self, direction), PhysicsMassEditHoverGesture(self, GESTURE_SCALE, direction)])
                            else:
                                with sc.Transform(look_at=sc.Transform.LookAt.CAMERA, scale_to=sc.Space.SCREEN):
                                    # Always render this behind:
                                    with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0.0, 0.0, -0.01)):
                                        shape = sc.Arc(MASS_DISTRIBUTION_HANDLE_SIZE / 2 + HOVER_LINE_THICKNESS, color=COLOR_INVISIBLE, gesture=[PhysicsMassEditGestureScale(self, direction), PhysicsMassEditHoverGesture(self, GESTURE_SCALE, direction)])
                                        self.add_to_toggles(HighlightToggle(shape), PhysicsMassDistributionEditManipulator.get_toggle_group_by_gesture_direction(GESTURE_SCALE, direction))
                                    sc.Arc(MASS_DISTRIBUTION_HANDLE_SIZE / 2, color=COLOR_AXIS[direction_to_axis(direction)])

                            if MASS_DISTRIBUTION_HANDLE_ARROW:
                                with sc.Transform(scale_to=sc.Space.SCREEN):
                                    with sc.Transform(transform = mat44_direction_rotate):
                                        with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0.0, 0.0, (MASS_DISTRIBUTION_HANDLE_RECTANGLE_SIZE / 2 if MASS_DISTRIBUTION_HANDLE_RECTANGLE else MASS_DISTRIBUTION_HANDLE_SIZE))):
                                            sc.Rectangle(width=MASS_DISTRIBUTION_HANDLE_ARROW_SIZE, height=MASS_DISTRIBUTION_HANDLE_ARROW_SIZE, color=COLOR_AXIS_SHADED[direction_to_axis(direction)])
                                            sc.PolygonMesh(scale_arrow_points_front, ([COLOR_AXIS[direction_to_axis(direction)]] * 3 + [COLOR_AXIS_SHADED[direction_to_axis(direction)]] * 3) * int(scale_arrow_face_count / 2), scale_arrow_faces_vertex_count, scale_arrow_vertex_indices)
                                        with sc.Transform(transform = sc.Matrix44.get_rotation_matrix(0.0, 180.0, 0.0, True)):
                                            with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0.0, 0.0, (MASS_DISTRIBUTION_HANDLE_RECTANGLE_SIZE / 2 if MASS_DISTRIBUTION_HANDLE_RECTANGLE else MASS_DISTRIBUTION_HANDLE_SIZE))):
                                                sc.Rectangle(width=MASS_DISTRIBUTION_HANDLE_ARROW_SIZE, height=MASS_DISTRIBUTION_HANDLE_ARROW_SIZE, color=COLOR_AXIS[direction_to_axis(direction)])
                                                sc.PolygonMesh(scale_arrow_points_front, ([COLOR_AXIS[direction_to_axis(direction)]] * 3 + [COLOR_AXIS_SHADED[direction_to_axis(direction)]] * 3) * int(scale_arrow_face_count / 2), scale_arrow_faces_vertex_count, scale_arrow_vertex_indices)

                        with sc.Transform(transform = mat44_direction_rotate):
                            with sc.Transform(scale_to=sc.Space.SCREEN):
                                sc.PolygonMesh(translate_arrow_points_front, [COLOR_AXIS[direction_to_axis(direction)]] * 3 * translate_arrow_face_count, translate_arrow_faces_vertex_count, translate_arrow_vertex_indices)
                                sc.PolygonMesh(translate_arrow_points_back, [COLOR_AXIS_SHADED[direction_to_axis(direction)]] * 3 * translate_arrow_face_count, translate_arrow_faces_vertex_count, translate_arrow_vertex_indices)

            self._shape_transform_axis.clear()
            self._shape_transform_circle.clear()
            if VISUAL_MASS_DISTRIBUTION_RULER_SUBDIVISION_MARKERS:
                self._shape_transform_circle_ruler.clear()

            for axis in range(AXIS_NUM):
                if axis == AXIS_X:
                    rotation_circle = Gf.Vec3f(0.0, 90.0, 0.0)
                    rotation_axis = Gf.Vec3f(90.0, 0.0, 0.0)
                elif axis == AXIS_Y:
                    rotation_circle = Gf.Vec3f(-90.0, 0.0, 0.0)
                    rotation_axis = Gf.Vec3f(0.0, 0.0, -90.0)
                elif axis == AXIS_Z:
                    rotation_circle = Gf.Vec3f(0.0, 0.0, 0.0)
                    rotation_axis = Gf.Vec3f(0.0, 90.0, 0.0)

                with self._transform_center_of_mass:
                    with self._transform_principal_rotate:
                        self._shape_transform_axis.append(sc.Transform(transform=sc.Matrix44.get_rotation_matrix(*rotation_axis, True)))
                        with self._shape_transform_axis[axis]:
                            with sc.Transform(scale_to=sc.Space.SCREEN):
                                shape = sc.Line([-CENTER_OF_MASS_AXIS_HALF_LENGTH+CENTER_OF_MASS_AXIS_ARROW_LENGTH, 0.0, 0.0], [CENTER_OF_MASS_AXIS_HALF_LENGTH-CENTER_OF_MASS_AXIS_ARROW_LENGTH, 0.0, 0.0], color=COLOR_INVISIBLE, thickness=MAIN_LINE_THICKNESS+HOVER_LINE_THICKNESS, gesture=[PhysicsMassEditGestureTranslate(self, axis), PhysicsMassEditHoverGesture(self, GESTURE_TRANSLATE, axis * 2)])
                                self.add_to_toggles(HighlightToggle(shape), PhysicsMassDistributionEditManipulator.get_toggle_group_by_gesture_axis(GESTURE_TRANSLATE, axis))
                                sc.Line([-CENTER_OF_MASS_AXIS_HALF_LENGTH+CENTER_OF_MASS_AXIS_ARROW_LENGTH, 0.0, 0.0], [CENTER_OF_MASS_AXIS_HALF_LENGTH-CENTER_OF_MASS_AXIS_ARROW_LENGTH, 0.0, 0.0], color=COLOR_AXIS[axis], thickness=MAIN_LINE_THICKNESS)

                        self._shape_transform_circle.append(sc.Transform(transform=sc.Matrix44.get_rotation_matrix(*rotation_circle, True)))
                        with self._shape_transform_circle[axis]:
                            with sc.Transform(scale_to=sc.Space.SCREEN):
                                shape = sc.Arc(CENTER_OF_MASS_AXIS_HALF_LENGTH, tesselation=PRINCIPAL_AXIS_ROTATION_ARC_TESSELATION, color=COLOR_INVISIBLE, wireframe=True, thickness=MAIN_LINE_THICKNESS+HOVER_LINE_THICKNESS, gesture=[PhysicsMassEditGestureRotate(self, axis), PhysicsMassEditHoverGesture(self, GESTURE_ROTATE, axis * 2)])
                                self.add_to_toggles(HighlightToggle(shape), PhysicsMassDistributionEditManipulator.get_toggle_group_by_gesture_axis(GESTURE_ROTATE, axis))
                                sc.Arc(CENTER_OF_MASS_AXIS_HALF_LENGTH, tesselation=PRINCIPAL_AXIS_ROTATION_ARC_TESSELATION, color=COLOR_AXIS[axis], wireframe=True, thickness=MAIN_LINE_THICKNESS)

                        if VISUAL_MASS_DISTRIBUTION_RULER_SUBDIVISION_MARKERS:
                            self._shape_transform_circle_ruler.append(sc.Transform())
                            with self._shape_transform_circle_ruler[axis]:
                                with sc.Transform(scale_to=sc.Space.SCREEN):
                                    for n in range(16):
                                        length = 0.1
                                        if mod(n, 4) == 0:
                                            length = 0.3
                                        elif mod(n, 2) == 0:
                                            length = 0.2
                                        angle = math.radians(float(n) * 22.5)
                                        x = CENTER_OF_MASS_AXIS_HALF_LENGTH * math.cos( angle)
                                        y = CENTER_OF_MASS_AXIS_HALF_LENGTH * math.sin( angle)
                                        sc.Line([x, y, 0.0], [x * (1.0 - length), y * (1.0 - length), 0.0], color=COLOR_AXIS_SHADED[axis], thickness=1)

        with self._shape_transform_inertia_distribution_box:
            for direction in range(AXIS_DIRECTION_NUM):
                if direction == AXIS_DIRECTION_X_POS:
                    rotation = Gf.Vec3f(0.0, 90.0, 0.0)
                elif direction == AXIS_DIRECTION_X_NEG:
                    rotation = Gf.Vec3f(0.0, -90.0, 0.0)
                elif direction == AXIS_DIRECTION_Y_POS:
                    rotation = Gf.Vec3f(-90.0, 0.0, 0.0)
                elif direction == AXIS_DIRECTION_Y_NEG:
                    rotation = Gf.Vec3f(90.0, 0.0, 0.0)
                elif direction == AXIS_DIRECTION_Z_POS:
                    rotation = Gf.Vec3f(0.0, 0.0, -90.0)
                elif direction == AXIS_DIRECTION_Z_NEG:
                    rotation = Gf.Vec3f(180.0, 0.0, 90.0)
                mat44_direction_rotate = sc.Matrix44.get_rotation_matrix(*rotation, True)
                visibility = sc.Transform(visible=False)
                self.add_to_toggles(VisibilityToggle(visibility), PhysicsMassDistributionEditManipulator.get_toggle_group_by_gesture_direction(GESTURE_SCALE, direction))
                with visibility:
                    with sc.Transform(transform=mat44_direction_rotate):
                        sc.Line([0.0, 0.0, -0.5], [0.0, 0.0, 0.5], color=COLOR_AXIS[direction_to_axis(direction)], thickness=1)

    def destroy(self):
        if self._snap_settings_listener is not None:
            self._snap_settings_listener = None
        super().destroy()


VISUAL_MASS_DISTRIBUTION_VIEW_ICON_SIZE = Gf.Vec2f(40.0, 50.0)

# Handles hover highlighting etc. for manipulation gestures.
class PhysicsMassViewInfoHoverGesture(PhysicsHoverGesture):
    def __init__(self, manipulator):
        super().__init__(manipulator)

    def on_began(self) -> bool:
        if not super().on_began():
            return False
        self._manipulator.populate(True)
        self._manipulator.get_viewport_overlay().set_info_box_target(INFO_BOX_MASS_VIEW, self._manipulator)
        # Icon hover info have precedence over the edit info box.
        self._manipulator.get_viewport_overlay().refresh_info_box(INFO_BOX_MASS_EDIT)
        return True

    def on_ended(self):
        super().on_ended()
        if self._manipulator.get_viewport_overlay().get_info_box_target(INFO_BOX_MASS_VIEW) == self._manipulator:
            self._manipulator.get_viewport_overlay().set_info_box_target(INFO_BOX_MASS_VIEW, None)
            # Refresh any mass edit info box in case it was blocked by this.
            self._manipulator.get_viewport_overlay().refresh_info_box(INFO_BOX_MASS_EDIT)

class GestureAlwaysPassClick(sc.GestureManager):
    def can_be_prevented(self, gesture):
        return False

    def should_prevent(self, gesture, preventer):
        return False


class PhysicsMassViewInfoClickGesture(PhysicsClickGesture):

    def __init__(self, manipulator):
        super().__init__(manipulator)

    def on_ended(self) -> bool:
        if not super().on_ended():
            return False
        path = self._manipulator._prim.GetPrimPath().pathString
        input = self._manipulator.get_viewport_overlay()._main_viewport_overlay._input
        keyboard = self._manipulator.get_viewport_overlay()._main_viewport_overlay._keyboard
        if (input.get_keyboard_value(keyboard, carb.input.KeyboardInput.LEFT_CONTROL)
            or input.get_keyboard_value(keyboard, carb.input.KeyboardInput.RIGHT_CONTROL)):
            self._manipulator.get_viewport_overlay().toggle_in_selection(path)
        elif (input.get_keyboard_value(keyboard, carb.input.KeyboardInput.LEFT_SHIFT)
            or input.get_keyboard_value(keyboard, carb.input.KeyboardInput.RIGHT_SHIFT)):
            self._manipulator.get_viewport_overlay().add_to_selection(path)
        else:
            self._manipulator.get_viewport_overlay().set_selection(path)
        return True


class PhysicsMassPropertiesViewManipulator(PhysicsMassManipulator):

    def __init__(self, viewport_overlay, prim):
        super().__init__(viewport_overlay, prim)

    def update_info_text(self):
        super().update_info_text()
        self._viewport_overlay.refresh_info_box(INFO_BOX_MASS_VIEW)

    def _make_shapes(self):
        super()._make_shapes()
        with self._transform_pose:
            with sc.Transform(scale_to=sc.Space.SCREEN, look_at=sc.Transform.LookAt.CAMERA):
                icons_folder = Path(__file__).parents[3].joinpath("icons", "physicsMass")

                filename = str(icons_folder.joinpath("mass_info_weight_on.png"))
                image_shape = sc.Image(filename, width=VISUAL_MASS_DISTRIBUTION_VIEW_ICON_SIZE[0], height=VISUAL_MASS_DISTRIBUTION_VIEW_ICON_SIZE[1], gesture=[PhysicsMassViewInfoClickGesture(self), PhysicsMassViewInfoHoverGesture(self)])

                filename_off = str(icons_folder.joinpath("mass_info_weight_off.png"))
                self.add_to_toggles(ImageToggle(image_shape, filename, filename_off))

            self.add_to_toggles(VisibilityToggle(self._shape_transform_center_of_mass_visualization))
            self.add_to_toggles(VisibilityToggle(self._shape_transform_inertia_distribution_box))

    def populate(self, force_update_all=False):
        if not self._active:
            self.invalidate()
            return

        if not self._prim or not self._prim.IsValid() or not self._prim.HasAPI(UsdPhysics.RigidBodyAPI):
            self.invalidate()
            return

        if not self._shape_transform_center_of_mass_visualization.visible:
            # Only update Xform:
            xform_cache = self.get_viewport_overlay().get_xform_cache()
            xform_mat = xform_cache.GetLocalToWorldTransform(self._prim).RemoveScaleShear()
            self._transform_pose.transform = sc.Matrix44(*xform_mat[0], *xform_mat[1], *xform_mat[2], *xform_mat[3])
            return

        super().populate(force_update_all)


VISUAL_MASS_DISTRIBUTION_EDIT_INFO_BOX_OFFSET = Gf.Vec3f(150.0, 150.0, 0.0)
VISUAL_MASS_DISTRIBUTION_VIEW_INFO_BOX_OFFSET = Gf.Vec3f(VISUAL_MASS_DISTRIBUTION_VIEW_ICON_SIZE[0] * 0.5 + 50.0, (VISUAL_MASS_DISTRIBUTION_VIEW_ICON_SIZE[0] * 0.5) + 50.0, 0.0)

class PhysicsMassInfoBoxManipulator(sc.Manipulator):

    def __init__(self, viewport_overlay):
        super().__init__()
        self._viewport_overlay = viewport_overlay
        self._stage = None
        self._mass_info_text_header = None
        self._mass_info_text_title = [None] * MASS_INFO_TEXT_ENTRY_NUM
        self._mass_info_text_value = [None] * MASS_INFO_TEXT_ENTRY_NUM
        self._mass_info_text_footer = None
        self._mass_info_text_transform_outer = None
        self._mass_info_text_transform_inner_offset_screen = None
        self._mass_info_text_transform_header = None
        self._mass_info_text_transform_titles = None
        self._mass_info_text_transform_values = None
        self._mass_info_text_transform_footer = None
        self._mass_info_background = None
        self._mass_info_background_transform = None
        self._accumulated_center_of_mass = None
        self._mass_manipulators = None

    def refresh(self):
        if self._mass_info_text_transform_outer is None:
            self.invalidate()
            return
        if self._mass_manipulators is None:
            self._mass_info_text_transform_outer.visible = False
            return

        mass_manipulators = self._mass_manipulators
        # Even if only being tied to a single manipulator, put it into an array for code simplicity below.
        if not isinstance(self._mass_manipulators, collections.abc.Sized):
            mass_manipulators = [self._mass_manipulators]
        elif len(self._mass_manipulators) < 1:
            self._mass_info_text_transform_outer.visible = False
            return

        self._mass_info_text_transform_outer.visible = True
        multiple_prims = True if len(mass_manipulators) > 1 else False

        txt_total_mass = "-"
        txt_center_of_mass = "-"
        txt_principal_axis = "-"
        txt_diagonal_inertia = "-"

        info_box_position = None

        txt_footer = []
        query_status = PhysxPropertyQueryObject.Status.COMPLETE
        if multiple_prims:
            prim_path = "Multiple selected"
            self._accumulated_center_of_mass = Gf.Vec3f(0.0)
            total_mass = 0.0

            for manipulator in mass_manipulators:
                if manipulator._prim_xform is not None:
                    manipulator_query_status = manipulator.get_physx_property_query_status()
                    if manipulator_query_status == PhysxPropertyQueryObject.Status.COMPLETE:
                            total_mass += manipulator._body_mass
                            prim_center_of_mass = components_divide(manipulator._body_center_of_mass, manipulator._transform_scale)
                            prim_center_of_mass = manipulator._prim_xform.Transform(prim_center_of_mass)
                            self._accumulated_center_of_mass += prim_center_of_mass * manipulator._body_mass
                    elif query_status != PhysxPropertyQueryObject.Status.FAIL:
                        query_status = manipulator_query_status

            if total_mass > 0.0:
                self._accumulated_center_of_mass = self._accumulated_center_of_mass * (1.0 / total_mass)
                txt_total_mass = float_to_string(total_mass)

            info_box_position = self._accumulated_center_of_mass

        else:  # Single prim
            if mass_manipulators[0]._prim is not None and mass_manipulators[0]._prim.IsValid() and mass_manipulators[0]._prim_xform is not None:
                prim_path = str(mass_manipulators[0]._prim.GetPrimPath())
                # Truncate if exceeding 20 characters. Note: since the font is not monotype, the actual width may vary.
                if len(prim_path) > 20:
                    prim_path = "..." + prim_path[-17:]

                if isinstance(self, PhysicsMassEditInfoBoxManipulator):
                    prim_center_of_mass = components_divide(mass_manipulators[0]._body_center_of_mass, mass_manipulators[0]._transform_scale)
                    prim_center_of_mass = mass_manipulators[0]._prim_xform.Transform(prim_center_of_mass)
                    info_box_position = prim_center_of_mass
                else:
                    info_box_position = mass_manipulators[0]._prim_xform.ExtractTranslation()

                query_status = mass_manipulators[0].get_physx_property_query_status()
                if query_status == PhysxPropertyQueryObject.Status.COMPLETE:
                    txt_center_of_mass = (float_to_string(mass_manipulators[0]._body_center_of_mass[0], 1) + "; " +
                                        float_to_string(mass_manipulators[0]._body_center_of_mass[1], 1) + "; " +
                                        float_to_string(mass_manipulators[0]._body_center_of_mass[2], 1))
                    matrix = Gf.Matrix4d()
                    matrix.SetRotate(mass_manipulators[0]._body_principal_axes*mass_manipulators[0]._rotation_modifier)
                    decomposed = Gf.Rotation.DecomposeRotation3(matrix, Gf.Vec3d(*AXIS_VECTOR_X), Gf.Vec3d(*AXIS_VECTOR_Y), Gf.Vec3d(*AXIS_VECTOR_Z), 1.0)
                    txt_principal_axis = float_to_string(math.degrees(decomposed[0]), 1)+"°; "+float_to_string(math.degrees(decomposed[1]), 1)+"°; " + float_to_string(math.degrees(decomposed[2]), 1)+"°"
                    txt_diagonal_inertia = "X: "+float_to_string(mass_manipulators[0]._body_diagonal_inertia[0])+"\nY: "+float_to_string(mass_manipulators[0]._body_diagonal_inertia[1])+"\nZ: " + float_to_string(mass_manipulators[0]._body_diagonal_inertia[2])
                    txt_total_mass = float_to_string(mass_manipulators[0]._body_mass)

                    if mass_manipulators[0]._prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                        collision_API = UsdPhysics.MeshCollisionAPI(mass_manipulators[0]._prim)
                        approximation = collision_API.GetApproximationAttr().Get()

                        if approximation == UsdPhysics.Tokens.none:
                            txt_footer.append("WARNING:")
                            txt_footer.append("Triangle Mesh collider approximation is")
                            txt_footer.append("unsupported for rigid bodies. Convex Hull")
                            txt_footer.append("will be used instead.")
                            self._mass_info_text_footer.color = MASS_INFO_TEXT_FONT_COLOR_WARNING
                        elif approximation == UsdPhysics.Tokens.meshSimplification:
                            txt_footer.append("WARNING:")
                            txt_footer.append("Mesh Simplification collider approximation")
                            txt_footer.append("is unsupported for rigid bodies. Convex Hull")
                            txt_footer.append("will be used instead.")
                            self._mass_info_text_footer.color = MASS_INFO_TEXT_FONT_COLOR_WARNING

        if info_box_position is None:
            self._mass_info_text_transform_outer.visible = False
            return

        if query_status != PhysxPropertyQueryObject.Status.COMPLETE:
            txt_total_mass = "..."
            txt_center_of_mass = "..."
            txt_principal_axis = "..."
            txt_diagonal_inertia = "..."

            if query_status == PhysxPropertyQueryObject.Status.FAIL:
                txt_footer.append("ERROR:")
                txt_footer.append("Failed to retrieve mass properties.")

        self._mass_info_text_transform_outer.transform = sc.Matrix44.get_translation_matrix(*info_box_position)

        line_offset = 2
        self._mass_info_text_value[MASS_INFO_TEXT_ENTRY_PRIM_PATH].text="\n" * line_offset + prim_path
        line_offset += 1
        self._mass_info_text_value[MASS_INFO_TEXT_ENTRY_MASS].text="\n" * line_offset + txt_total_mass
        line_offset += 1
        self._mass_info_text_value[MASS_INFO_TEXT_ENTRY_CENTER_OF_MASS].text="\n" * line_offset + txt_center_of_mass
        line_offset += 1
        self._mass_info_text_value[MASS_INFO_TEXT_ENTRY_PRINCIPAL_AXIS].text="\n" * line_offset + txt_principal_axis
        line_offset += 1
        self._mass_info_text_value[MASS_INFO_TEXT_ENTRY_DIAGONAL_INERTIA].text="\n" * line_offset + txt_diagonal_inertia
        line_offset += 3

        if len(txt_footer) > 0:
            self._mass_info_text_transform_footer.visible = True
            self._mass_info_text_footer.text = "\n" * line_offset
            line_offset+=1
            for line in range(len(txt_footer)):
                self._mass_info_text_footer.text += "\n" + txt_footer[line]
                line_offset+=1
        else:
            self._mass_info_text_transform_footer.visible = False

        width = MASS_INFO_TEXT_BOX_WIDTH
        height = (UI_SCENE_SCREEN_SCALE_MODIFIER *
                      MASS_INFO_TEXT_FONT_LINE_SPAN * line_offset +
                      MASS_INFO_TEXT_MARGIN[1] * 2)  # Translates are not scaled


        if isinstance(self, PhysicsMassEditInfoBoxManipulator):
            self._mass_info_text_transform_inner_offset_screen.transform = sc.Matrix44.get_translation_matrix(VISUAL_MASS_DISTRIBUTION_EDIT_INFO_BOX_OFFSET[0], VISUAL_MASS_DISTRIBUTION_EDIT_INFO_BOX_OFFSET[1] + height, VISUAL_MASS_DISTRIBUTION_EDIT_INFO_BOX_OFFSET[2])
        else:
            self._mass_info_text_transform_inner_offset_screen.transform = sc.Matrix44.get_translation_matrix(VISUAL_MASS_DISTRIBUTION_VIEW_INFO_BOX_OFFSET[0], VISUAL_MASS_DISTRIBUTION_VIEW_INFO_BOX_OFFSET[1] + height, VISUAL_MASS_DISTRIBUTION_VIEW_INFO_BOX_OFFSET[2])

        self._mass_info_background_transform.transform=sc.Matrix44.get_translation_matrix(width * 0.5, -height * 0.5, 0.0)*sc.Matrix44.get_scale_matrix(width, height, 0.0)
        self._mass_info_text_transform_header.transform=sc.Matrix44.get_translation_matrix(width * 0.5, -MASS_INFO_TEXT_MARGIN[1], 0.0)
        self._mass_info_text_transform_titles.transform=sc.Matrix44.get_translation_matrix(MASS_INFO_TEXT_MARGIN[0], -MASS_INFO_TEXT_MARGIN[1], 0.0)
        self._mass_info_text_transform_values.transform=sc.Matrix44.get_translation_matrix(width - MASS_INFO_TEXT_MARGIN[0], -MASS_INFO_TEXT_MARGIN[1], 0.0)
        self._mass_info_text_transform_footer.transform=sc.Matrix44.get_translation_matrix(MASS_INFO_TEXT_MARGIN[0], -MASS_INFO_TEXT_MARGIN[1], 0.0)

    def _make_shapes(self):
        # Draw mass info box.
        titles = ["Prim:", "Total mass:", "Center of mass:", "Principal axis:", "Diagonal inertia:"]

        self._mass_info_text_transform_outer = sc.Transform(transform=sc.Matrix44.get_translation_matrix(0, 0, 0))
        self._mass_info_text_transform_outer.visible = False
        with self._mass_info_text_transform_outer:
            with sc.Transform(look_at=sc.Transform.LookAt.CAMERA):
                self._mass_info_text_transform_inner_offset_screen = sc.Transform(scale_to=sc.Space.SCREEN)
                with self._mass_info_text_transform_inner_offset_screen:
                    self._mass_info_text_transform_header = sc.Transform()
                    with self._mass_info_text_transform_header:
                        self._mass_info_text_header = sc.Label(
                        "MASS PROPERTIES INFO",
                            alignment=ui.Alignment.CENTER_TOP,
                            color=MASS_INFO_TEXT_FONT_COLOR,
                            size=MASS_INFO_TEXT_FONT_SIZE
                            )
                    self._mass_info_text_transform_titles = sc.Transform()
                    with self._mass_info_text_transform_titles:
                        for entry in range(MASS_INFO_TEXT_ENTRY_NUM):
                            self._mass_info_text_title[entry] = sc.Label(
                            "\n" * (entry + 2) + titles[entry],
                                alignment=ui.Alignment.LEFT,
                                color=MASS_INFO_TEXT_FONT_COLOR,
                                size=MASS_INFO_TEXT_FONT_SIZE
                                )
                    self._mass_info_text_transform_values = sc.Transform()
                    with self._mass_info_text_transform_values:
                        for entry in range(MASS_INFO_TEXT_ENTRY_NUM):
                            self._mass_info_text_value[entry] = sc.Label(
                            " ",
                                alignment=ui.Alignment.RIGHT,
                                color=MASS_INFO_TEXT_FONT_COLOR,
                                size=MASS_INFO_TEXT_FONT_SIZE
                                )
                    self._mass_info_text_transform_footer = sc.Transform(visible=False)
                    with self._mass_info_text_transform_footer:
                        self._mass_info_text_footer = sc.Label("",
                            alignment=ui.Alignment.LEFT,
                            color=MASS_INFO_TEXT_FONT_COLOR,
                            size=MASS_INFO_TEXT_FONT_SIZE
                            )
                    self._mass_info_background_transform =  sc.Transform()
                    with self._mass_info_background_transform:
                        self._mass_info_background = sc.Rectangle(1, 1, color=MASS_INFO_TEXT_BOX_BACKGROUND_COLOR)

    def on_build(self):
        super().on_build()

        if self._viewport_overlay is None:
            return

        self._make_shapes()
        self.refresh()

    def set_target(self, manipulators):
        self._mass_manipulators = manipulators
        self.refresh()

    def get_target(self):
        return self._mass_manipulators

    def destroy(self):
        self._mass_manipulators = None
        self._viewport_overlay = None
        self.invalidate()

    def __del__(self):
        self.destroy()

class PhysicsMassEditInfoBoxManipulator(PhysicsMassInfoBoxManipulator):
    def __init__(self, viewport_overlay):
        super().__init__(viewport_overlay)
        self._toggle_groups = []
        self._accumulated_center_of_mass_transform = None

    def add_to_toggles(self, item, group = 0):
        if group < 0 or not isinstance(group, int):
            carb.log_error(f"Fatal error - invalid visual toggle group: {group}")
        while len(self._toggle_groups) <= group:
            self._toggle_groups.append([])
        self._toggle_groups[group].append(item)

    def refresh(self):
        # Icon hover info have precedence over the edit info box.
        if self._viewport_overlay.get_info_box_target(INFO_BOX_MASS_VIEW) is not None:
            self._mass_info_text_transform_outer.visible = False
            return

        super().refresh()

        if self._mass_info_text_transform_outer is None:
            self.invalidate()
            return
        if not self._mass_info_text_transform_outer.visible:
            self._accumulated_center_of_mass_transform.visible = False
            return

        if len(self._mass_manipulators) > 1:
            self._accumulated_center_of_mass_transform.visible = True
            self._accumulated_center_of_mass_transform.transform = sc.Matrix44.get_translation_matrix(*self._accumulated_center_of_mass)
        else:
            self._accumulated_center_of_mass_transform.visible = False

        active_hover = get_active_hover()
        active_gesture = get_active_gesture()
        enable = False
        toggle_group = -1
        if active_gesture is not None:
            if active_gesture._manipulator in self._mass_manipulators:  # self._mass_manipulators.count(active_gesture._manipulator) > 0:
                toggle_group = active_gesture._toggle_group
                enable = True
        elif active_hover is not None and active_hover._manipulator in self._mass_manipulators: # self._mass_manipulators.count(active_hover._manipulator) > 0:
            toggle_group = active_hover._toggle_group
            enable = True
        if len(self._toggle_groups) == 1:
            for object in self._toggle_groups[0]:
                object.toggle(enable)
        elif len(self._toggle_groups) > 1:
            # If having multiple, first set all to invisible, then switch on the relevant.
            # This approach is necessary as some objects may be present in several groups.
            for group in range(len(self._toggle_groups)):
                for object in self._toggle_groups[group]:
                    object.toggle(False)
            if toggle_group >= 0:
                if toggle_group >= len(self._toggle_groups):
                    carb.log_error(f"Fatal error: visibility group exceeding available: {toggle_group} (+1) vs {len(self._toggle_groups)}")
                for object in self._toggle_groups[toggle_group]:
                    object.toggle(True)

    def _make_shapes(self):
        super()._make_shapes()

        self._accumulated_center_of_mass_transform = sc.Transform(visible = False)

        # Highlights text entries when gestures are active.
        for axis in range(AXIS_NUM):
            self.add_to_toggles(ColorToggle(self._mass_info_text_title[MASS_INFO_TEXT_ENTRY_CENTER_OF_MASS], MASS_INFO_TEXT_FONT_COLOR_HIGHLIGHT, MASS_INFO_TEXT_FONT_COLOR), PhysicsMassDistributionEditManipulator.get_toggle_group_by_gesture_axis(GESTURE_TRANSLATE, axis))
            self.add_to_toggles(ColorToggle(self._mass_info_text_value[MASS_INFO_TEXT_ENTRY_CENTER_OF_MASS], MASS_INFO_TEXT_FONT_COLOR_HIGHLIGHT, MASS_INFO_TEXT_FONT_COLOR), PhysicsMassDistributionEditManipulator.get_toggle_group_by_gesture_axis(GESTURE_TRANSLATE, axis))
            self.add_to_toggles(ColorToggle(self._mass_info_text_title[MASS_INFO_TEXT_ENTRY_PRINCIPAL_AXIS], MASS_INFO_TEXT_FONT_COLOR_HIGHLIGHT, MASS_INFO_TEXT_FONT_COLOR), PhysicsMassDistributionEditManipulator.get_toggle_group_by_gesture_axis(GESTURE_ROTATE, axis))
            self.add_to_toggles(ColorToggle(self._mass_info_text_value[MASS_INFO_TEXT_ENTRY_PRINCIPAL_AXIS], MASS_INFO_TEXT_FONT_COLOR_HIGHLIGHT, MASS_INFO_TEXT_FONT_COLOR), PhysicsMassDistributionEditManipulator.get_toggle_group_by_gesture_axis(GESTURE_ROTATE, axis))

        for direction in range(AXIS_DIRECTION_NUM):
            self.add_to_toggles(ColorToggle(self._mass_info_text_title[MASS_INFO_TEXT_ENTRY_DIAGONAL_INERTIA], MASS_INFO_TEXT_FONT_COLOR_HIGHLIGHT, MASS_INFO_TEXT_FONT_COLOR), PhysicsMassDistributionEditManipulator.get_toggle_group_by_gesture_direction(GESTURE_SCALE, direction))
            self.add_to_toggles(ColorToggle(self._mass_info_text_value[MASS_INFO_TEXT_ENTRY_DIAGONAL_INERTIA], MASS_INFO_TEXT_FONT_COLOR_HIGHLIGHT, MASS_INFO_TEXT_FONT_COLOR), PhysicsMassDistributionEditManipulator.get_toggle_group_by_gesture_direction(GESTURE_SCALE, direction))

        with self._accumulated_center_of_mass_transform:
            with sc.Transform(look_at=sc.Transform.LookAt.CAMERA, scale_to=sc.Space.SCREEN):
                size = 100.0
                subdivisions = 4
                steps = 4
                arc_length = 0.75
                for step in range(steps):
                    for n in range(subdivisions):
                        length = (2 + step) / (steps + 1)
                        angle = math.radians((float(n) + step / 2 - arc_length * 0.5 + 0.5) * 360.0 / subdivisions)
                        angle_end = angle + arc_length * math.radians(360.0 / subdivisions)
                        sc.Arc(size * length, sector=0, begin=angle, end=angle_end, tesselation=(4 + step), color=MASS_DISTRIBUTION_BOX_COLOR, wireframe=True, thickness=MAIN_LINE_THICKNESS)

                sc.Line([-size / (steps + 1), 0.0, 0.0], [size / (steps + 1), 0.0, 0.0], color=MASS_DISTRIBUTION_BOX_COLOR, thickness=MAIN_LINE_THICKNESS)
                sc.Line([0.0, -size / (steps + 1), 0.0], [0.0, size / (steps + 1), 0.0], color=MASS_DISTRIBUTION_BOX_COLOR, thickness=MAIN_LINE_THICKNESS)


SETTING_DISPLAY_MASS_PROPERTIES_NONE = 0
SETTING_DISPLAY_MASS_PROPERTIES_SELECTED = 1
SETTING_DISPLAY_MASS_PROPERTIES_ALL = 2

MASS_PROPERTIES_VIEW_MAX_NUM = 1000
MASS_DISTRIBUTION_EDIT_MAX_NUM = 100

class PhysicsMassViewportOverlay(PhysicsViewportOverlay):
    def __init__(self, main_viewport_overlay):
        super().__init__(main_viewport_overlay)
        self._info_boxes = []

        self._mass_properties_view_manipulators = []
        self._mass_properties_view_manipulators_setting = None

        self._mass_distribution_edit_manipulators = []
        self._mass_distribution_edit_manipulators_setting = None

        self._usd_object_changed_listener = None
        self._usd_object_changed_prim_paths = set()
        self._usd_object_changed_kit_update_sub = None
        self._usd_context = None

        self._usd_geom_xform_cache = None

        self._selection = None
        self._stage_event_sub = None
        self._usd_context = omni.usd.get_context()
        settings = get_settings()
        self._mass_properties_view_manipulators_setting = settings.subscribe_to_node_change_events(
            SETTING_DISPLAY_MASS_PROPERTIES, self._on_mass_distribution_view_setting_changed
        )
        self._mass_distribution_edit_manipulators_setting = settings.subscribe_to_node_change_events(
            SETTING_MASS_DISTRIBUTION_MANIPULATOR, self._on_mass_distribution_edit_setting_changed
        )

        if self._usd_context is not None:
            self._selection = self._usd_context.get_selection()
            self._stage_event_sub = [
                get_eventdispatcher().observe_event(
                    observer_name="omni.physx.ui:PhysicsMassViewportOverlay",
                    event_name=self._usd_context.stage_event_name(event),
                    on_event=func
                )
                for event, func in (
                    (omni.usd.StageEventType.OPENED, lambda _: self._on_stage_opened()),
                    (omni.usd.StageEventType.CLOSING, lambda _: self._on_stage_closed()),
                    (omni.usd.StageEventType.SELECTION_CHANGED, lambda _: self._on_stage_selection_changed_event()),
                )
            ]

        self._gesture_manager = PhysicsGestureManager(self)
        self._on_stage_opened()

    def set_selection(self, prim_paths, update_stage_window = True):
        if self._selection:
            if not isinstance(prim_paths, list):
                prim_paths = [prim_paths]
            self._selection.set_selected_prim_paths(prim_paths, update_stage_window)

    def toggle_in_selection(self, prim_paths):
        if self._selection:
            selected_prim_paths = self._selection.get_selected_prim_paths()
            selected_prim_paths_new = selected_prim_paths.copy()
            added = False
            changed = False
            if not isinstance(prim_paths, list):
                prim_paths = [prim_paths]
            for prim_path in prim_paths:
                changed = True
                if not prim_path in selected_prim_paths:
                    selected_prim_paths_new.append(prim_path)
                    added = True
                else:
                    selected_prim_paths_new.remove(prim_path)

            if changed:
                self.set_selection(selected_prim_paths_new, added)

    def add_to_selection(self, prim_paths):
        if self._selection:
            selected_prim_paths = self._selection.get_selected_prim_paths()
            changed = False
            if not isinstance(prim_paths, list):
                prim_paths = [prim_paths]
            for prim_path in prim_paths:
                if not prim_path in selected_prim_paths:
                    selected_prim_paths.append(prim_path)
                    changed = True
            if changed:
                self.set_selection(selected_prim_paths)

    def remove_from_selection(self, prim_paths):
        if self._selection:
            selected_prim_paths = self._selection.get_selected_prim_paths()
            changed = False
            if not isinstance(prim_paths, list):
                prim_paths = [prim_paths]
            for prim_path in prim_paths:
                if prim_path in selected_prim_paths:
                    selected_prim_paths.remove(prim_path)
                    changed = True
            if changed:
                self.set_selection(selected_prim_paths, False)

    def _create_info_boxes(self):
        for n in range(INFO_BOX_NUM):
            with self._main_viewport_overlay.get_overlay_scene_view().scene:
                if n == INFO_BOX_MASS_VIEW:
                    info_box = PhysicsMassInfoBoxManipulator(self)
                elif n == INFO_BOX_MASS_EDIT:
                    info_box = PhysicsMassEditInfoBoxManipulator(self)
            self._info_boxes.append(info_box)

    def get_xform_cache(self) -> UsdGeom.XformCache:
        return self._usd_geom_xform_cache

    def _destroy_info_boxes(self):
        for info_box in self._info_boxes:
            info_box.destroy()
        self._info_boxes.clear()

    def set_info_box_target(self, index, target):
        if index < len(self._info_boxes):
            self._info_boxes[index].set_target(target)

    def get_info_box_target(self, index):
        if index < len(self._info_boxes):
            return self._info_boxes[index].get_target()
        else:
            return None

    def refresh_info_box(self, index):
        if index < len(self._info_boxes):
            self._info_boxes[index].refresh()

    def _clear_mass_properties_view_manipulators(self):
        for manipulator in self._mass_properties_view_manipulators:
            manipulator.destroy()
        self._mass_properties_view_manipulators.clear()
        self.set_info_box_target(INFO_BOX_MASS_VIEW, None)

    def _clear_mass_distribution_edit_manipulators(self):
        for manipulator in self._mass_distribution_edit_manipulators:
            manipulator.destroy()
        self._mass_distribution_edit_manipulators.clear()
        self.set_info_box_target(INFO_BOX_MASS_EDIT, None)

    def _revoke_usd_change_listener(self):
        if self._usd_object_changed_listener is not None:
            self._usd_object_changed_listener.Revoke()
            self._usd_object_changed_listener = None
            self._usd_object_changed_kit_update_sub = None

    def _manage_usd_change_listener(self):
        if(self._usd_context and self._usd_context.get_stage() and
            (get_settings().get_as_bool(SETTING_MASS_DISTRIBUTION_MANIPULATOR) or
            get_settings().get_as_int(SETTING_DISPLAY_MASS_PROPERTIES) != SETTING_DISPLAY_MASS_PROPERTIES_NONE)):
            if self._usd_object_changed_listener is None:
                self._usd_object_changed_listener = Tf.Notice.Register(
                        Usd.Notice.ObjectsChanged, self._on_usd_objects_changed,
                        self._usd_context.get_stage())

                self._usd_object_changed_kit_update_sub = get_eventdispatcher().observe_event(
                    event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
                    on_event=self._on_kit_update,
                    observer_name="omni.physx.ui:PhysicsMassViewportOverlay"
                )                
        else:
            self._revoke_usd_change_listener()

    # NB: Using Usd.Notice is not ideal for performance so we should consider changing the following to something
    # that does not rely on it.
    def _on_usd_objects_changed(self, notice, stage):
        for changed_path in (notice.GetChangedInfoOnlyPaths() + notice.GetResyncedPaths()):
            self._usd_object_changed_prim_paths.add(changed_path.GetPrimPath())

    def _on_kit_update(self, event: carb.events.IEvent):
        if len(self._usd_object_changed_prim_paths) == 0:
            return

        stage = self._usd_context.get_stage()

        if not stage:
            return

        if self._usd_geom_xform_cache is not None:
            self._usd_geom_xform_cache.Clear()

        # Use set for quicker lookup.
        changed_rigid_body_paths = set()

        for changed_path in self._usd_object_changed_prim_paths:
            changed_prim = stage.GetPrimAtPath(changed_path)
            if (changed_prim is not None and changed_prim.IsValid()
                and changed_prim.HasAPI(UsdPhysics.RigidBodyAPI)):
                changed_rigid_body_paths.add(changed_path)

        self._usd_object_changed_prim_paths.clear()

        if (len(changed_rigid_body_paths) == 0 and
            len(self._mass_distribution_edit_manipulators) == 0 and
            len(self._mass_properties_view_manipulators) == 0):
            return

        if DEBUG_USD_UPDATE_FRAME_PROFILING:
            global profiling_instance
            profiling_instance.enable()

        # If not simulating, we always do a new physx property query on USD change. As they are cached, the performance impact is minimal.
        timeline_playing = get_timeline_interface().is_playing()

        settings = get_settings()
        if settings.get_as_bool(SETTING_MASS_DISTRIBUTION_MANIPULATOR):
            # Always add to the list of prims to add. We will filter out as relevant below.
            edit_prims_to_add = changed_rigid_body_paths.copy()
            manipulators_to_remove = []
            refresh_info_box = False
            for manipulator in self._mass_distribution_edit_manipulators:
                # Check if still active.
                if manipulator._prim is not None:
                    prim_path = manipulator._prim.GetPrimPath()
                    if not manipulator._prim.IsValid():
                        # If the manipulator has an invalid prim, make sure that it's removed.
                        # This can be caused by moving around prims in the scene graph. At this point, the path will still be retrievable.
                        manipulators_to_remove.append(manipulator)
                        if prim_path in changed_rigid_body_paths:
                            edit_prims_to_add.remove(prim_path)
                        refresh_info_box = True
                    elif prim_path in changed_rigid_body_paths:
                        # Only update if we are not the ones updating it.
                        if not manipulator._writing_to_usd:
                            refresh_info_box = True
                            manipulator.populate(not timeline_playing)

                        if prim_path in edit_prims_to_add:
                            # A manipulator for this prim already exists so we don't need to add it.
                            edit_prims_to_add.remove(prim_path)
                    elif prim_path in self._usd_object_changed_prim_paths:
                        # This means that the associated prim no longer has the rigid body API.
                        refresh_info_box = True
                        manipulators_to_remove.append(manipulator)

            for manipulator in manipulators_to_remove:
                self._mass_distribution_edit_manipulators.remove(manipulator)
                manipulator.destroy()
                del manipulator

            if len(edit_prims_to_add) > 0:
                with self._main_viewport_overlay.get_overlay_scene_view().scene:
                    # Prim must also be part of the selection.
                    selection = self._selection.get_selected_prim_paths()
                    for prim in edit_prims_to_add:
                        if not prim in selection:
                            continue
                        if len(self._mass_distribution_edit_manipulators) >= MASS_DISTRIBUTION_EDIT_MAX_NUM:
                            break
                        manipulator = PhysicsMassDistributionEditManipulator(self, stage.GetPrimAtPath(prim))
                        self._mass_distribution_edit_manipulators.append(manipulator)
                        refresh_info_box = True

            if refresh_info_box:
                self.refresh_info_box(INFO_BOX_MASS_EDIT)

        if settings.get_as_int(SETTING_DISPLAY_MASS_PROPERTIES) != SETTING_DISPLAY_MASS_PROPERTIES_NONE:
            view_prims_to_add = changed_rigid_body_paths.copy()

            # Repeat the above process for views as relevant.
            manipulators_to_remove = []
            refresh_info_box = False
            info_box_target = self.get_info_box_target(INFO_BOX_MASS_VIEW)
            for manipulator in self._mass_properties_view_manipulators:
                if manipulator._prim:
                    prim_path = manipulator._prim.GetPrimPath()
                    if not manipulator._prim.IsValid():
                        # If the manipulator has an invalid prim, make sure that it's removed.
                        # This can be caused by moving around prims in the scene graph. At this point, the path will still be retrievable.
                        manipulators_to_remove.append(manipulator)
                        if prim_path in changed_rigid_body_paths:
                            view_prims_to_add.remove(prim_path)
                        if manipulator == info_box_target:
                            refresh_info_box = True
                    elif prim_path in changed_rigid_body_paths:
                        manipulator.populate(not timeline_playing)
                        if manipulator == info_box_target:
                            refresh_info_box = True
                        if prim_path in view_prims_to_add:
                            view_prims_to_add.remove(prim_path)
                    elif prim_path in self._usd_object_changed_prim_paths:
                        # This means that the associated prim no longer has the rigid body API.
                        manipulators_to_remove.append(manipulator)
                        if manipulator == info_box_target:
                            refresh_info_box = True

            for manipulator in manipulators_to_remove:
                self._mass_properties_view_manipulators.remove(manipulator)
                manipulator.destroy()
                del manipulator

            if len(view_prims_to_add) > 0:
                selection = None
                if settings.get_as_int(SETTING_DISPLAY_MASS_PROPERTIES) == SETTING_DISPLAY_MASS_PROPERTIES_SELECTED:
                    selection = self._selection.get_selected_prim_paths()
                with self._main_viewport_overlay.get_overlay_scene_view().scene:
                    for prim in view_prims_to_add:
                        if selection:
                            # Prim must also be part of the selection.
                            if not prim in selection:
                                continue
                        if len(self._mass_properties_view_manipulators) >= MASS_PROPERTIES_VIEW_MAX_NUM:
                            break
                        manipulator = PhysicsMassPropertiesViewManipulator(self, stage.GetPrimAtPath(prim))
                        self._mass_properties_view_manipulators.append(manipulator)
                        refresh_info_box = True

            if refresh_info_box:
                self.refresh_info_box(INFO_BOX_MASS_VIEW)

        if DEBUG_USD_UPDATE_FRAME_PROFILING:
            profiling_instance.disable()
            stats = pstats.Stats(profiling_instance).strip_dirs().sort_stats("cumtime")
            stats.print_stats(20)

    def _on_mass_distribution_view_setting_changed(self, item, event_type):
        self._update_mass_properties_view_manipulators()
        self._manage_usd_change_listener()

    def _on_mass_distribution_edit_setting_changed(self, item, event_type):
        self._update_mass_distribution_edit_manipulators()
        self._manage_usd_change_listener()

    def _update_mass_properties_view_manipulators(self):
        settings = get_settings()
        if (settings.get_as_int(SETTING_DISPLAY_MASS_PROPERTIES) == SETTING_DISPLAY_MASS_PROPERTIES_NONE or
            not self._usd_context):
            self._clear_mass_properties_view_manipulators()
            return
        stage = self._usd_context.get_stage()
        if not stage:
            self._clear_mass_properties_view_manipulators()
            return

        prims_to_add = []
        if settings.get_as_int(SETTING_DISPLAY_MASS_PROPERTIES) == SETTING_DISPLAY_MASS_PROPERTIES_SELECTED:
            manipulators_to_remove = self._mass_properties_view_manipulators.copy()

            if self._selection:
                selected_prim_paths = self._selection.get_selected_prim_paths()
                for selected_prim_path in selected_prim_paths:
                    prim = stage.GetPrimAtPath(selected_prim_path)
                    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                        exists = False
                        for manipulator in self._mass_properties_view_manipulators:
                            if manipulator._prim == prim:
                                manipulators_to_remove.remove(manipulator)
                                manipulator.populate()
                                exists = True
                                break
                        if not exists:
                            prims_to_add.append(prim)

            for manipulator in manipulators_to_remove:
                self._mass_properties_view_manipulators.remove(manipulator)
                manipulator.destroy()
                del manipulator
        else:
            self._clear_mass_properties_view_manipulators()
            prims_to_add = []
            for prim in stage.Traverse():
                if (prim.HasAPI(UsdPhysics.RigidBodyAPI)):
                    prims_to_add.append(prim)

        if len(prims_to_add) > 0:
            if (len(self._mass_properties_view_manipulators) + len( prims_to_add)
                    > MASS_PROPERTIES_VIEW_MAX_NUM):
                carb.log_warn(f"Exceeding maximum prims for mass view - " +
                    f"{(len(self._mass_properties_view_manipulators) + len( prims_to_add))}" +
                    f" vs {MASS_PROPERTIES_VIEW_MAX_NUM}")
                prims_to_add = prims_to_add[0:max(0, (MASS_PROPERTIES_VIEW_MAX_NUM-len(self._mass_properties_view_manipulators)))]
            with self._main_viewport_overlay.get_overlay_scene_view().scene:
                for prim in prims_to_add:
                    manipulator = PhysicsMassPropertiesViewManipulator(self, prim)
                    self._mass_properties_view_manipulators.append(manipulator)

        self.refresh_info_box(INFO_BOX_MASS_VIEW)

    def _update_mass_distribution_edit_manipulators(self):
        if (not get_settings().get_as_bool(SETTING_MASS_DISTRIBUTION_MANIPULATOR) or
            not self._usd_context):
            self._clear_mass_distribution_edit_manipulators()
            return
        stage = self._usd_context.get_stage()
        if not stage:
            self._clear_mass_distribution_edit_manipulators()
            return

        manipulators_to_remove = self._mass_distribution_edit_manipulators.copy()

        prims_to_add = []
        if self._selection:
            selected_prim_paths = self._selection.get_selected_prim_paths()
            for selected_prim_path in selected_prim_paths:
                prim = stage.GetPrimAtPath(selected_prim_path)
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    exists = False
                    for manipulator in self._mass_distribution_edit_manipulators:
                        if manipulator._prim == prim:
                            manipulators_to_remove.remove(manipulator)
                            manipulator.populate()
                            exists = True
                            break
                    if not exists:
                        prims_to_add.append(prim)

        for manipulator in manipulators_to_remove:
            self._mass_distribution_edit_manipulators.remove(manipulator)
            manipulator.destroy()
            del manipulator

        if len(prims_to_add) > 0:
            if (len(self._mass_distribution_edit_manipulators) + len( prims_to_add)
                    > MASS_DISTRIBUTION_EDIT_MAX_NUM):
                carb.log_warn(f"Exceeding maximum prims for mass distribution edit - " +
                    f"{(len(self._mass_distribution_edit_manipulators) + len(prims_to_add))}" +
                    f" vs {MASS_DISTRIBUTION_EDIT_MAX_NUM}")
                prims_to_add = prims_to_add[0:max(0, (MASS_DISTRIBUTION_EDIT_MAX_NUM-len(self._mass_distribution_edit_manipulators)))]
            with self._main_viewport_overlay.get_overlay_scene_view().scene:
                for prim in prims_to_add:
                    manipulator = PhysicsMassDistributionEditManipulator(self, prim)
                    self._mass_distribution_edit_manipulators.append(manipulator)

        for manipulator in self._mass_distribution_edit_manipulators:
            manipulator.set_edit_enabled(False if len(self._mass_distribution_edit_manipulators) > 1 else True)
        self.set_info_box_target(INFO_BOX_MASS_EDIT, self._mass_distribution_edit_manipulators)

    def _on_stage_opened(self):
        self._stage = self._usd_context.get_stage()
        if self._stage is None:
            return

        self._usd_geom_xform_cache = UsdGeom.XformCache()

        self._create_info_boxes()
        self._update_mass_properties_view_manipulators()
        self._update_mass_distribution_edit_manipulators()
        self._manage_usd_change_listener()

    def _on_stage_closed(self):
        self._usd_geom_xform_cache = None
        self._destroy_info_boxes()
        self._clear_mass_distribution_edit_manipulators()
        self._clear_mass_properties_view_manipulators()
        self._revoke_usd_change_listener()

    def _on_stage_selection_changed_event(self):
        settings = get_settings()
        if settings.get_as_bool(SETTING_MASS_DISTRIBUTION_MANIPULATOR):
            self._update_mass_distribution_edit_manipulators()
        if settings.get_as_int(SETTING_DISPLAY_MASS_PROPERTIES) == SETTING_DISPLAY_MASS_PROPERTIES_SELECTED:
            self._update_mass_properties_view_manipulators()

    def destroy(self):
        self._on_stage_closed()
        self._usd_geom_xform_cache = None
        self._selection = None
        self._stage_event_sub = None
        self._usd_context = None
        self._stage = None
        self._mass_distribution_edit_manipulators = []
        self._mass_properties_view_manipulators = []
        self._usd_object_changed_listener = None
        self._usd_object_changed_kit_update_sub = None
        self._usd_object_changed_prim_paths = set()
        super().destroy()
