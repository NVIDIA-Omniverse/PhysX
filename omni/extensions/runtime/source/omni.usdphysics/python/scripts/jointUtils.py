# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Usd, UsdPhysics, UsdGeom, Gf
import math
import carb

__all__ = ["JointValidator", "RevoluteJointValidator", "PrismaticJointValidator", "SphericalJointValidator", "DistanceJointValidator", "D6JointValidator", "make_joint_validator"]

axis_usd_token_to_index = (UsdGeom.Tokens.x, UsdGeom.Tokens.y, UsdGeom.Tokens.z)

def within_range(value, lower, upper, tolerance):
    return value >= lower - tolerance and value <= upper + tolerance

def clamp(value, lower, upper):
    return max(lower, min(value, upper))

def compute_transform_axial_rotation_offsets(offset_transform: Gf.Matrix4d):
    # Calculate the orientation offsets for each axis between the two attachment points. 
    # NB: this is not the same as decomposing the rotation matrix into Euler angles.
    orientation_offset = [0.0, 0.0, 0.0]

    for axis in range(3):
        zero_angle_direction = offset_transform.GetRow3((axis + 1) % 3)
        orientation_offset[axis] = math.degrees(math.atan2(zero_angle_direction[(axis + 2) % 3], zero_angle_direction[(axis + 1) % 3]))

    return orientation_offset


class JointValidator:
    TOLERANCE_POSITION = 0.1
    TOLERANCE_ORIENTATION = 1.0

    class Attachment:
        def __init__(self):
            self.offset_translation = Gf.Vec3d()
            self.offset_rotation = Gf.Quatd()
            self.prim_xform = Gf.Matrix4d()

        def compute_transform(self) -> Gf.Matrix4d:
            return Gf.Matrix4d().SetRotate(self.offset_rotation) * (Gf.Matrix4d().SetTranslate(self.offset_translation) * self.prim_xform).RemoveScaleShear()

    def __init__(self, prim: Usd.Prim = None):
        self.attachments = [JointValidator.Attachment(), JointValidator.Attachment()]
        if not prim:
            return

        joint = UsdPhysics.Joint(prim)
        if not joint:
            carb.log_error(f"Joint is not a valid joint: {prim.GetPath()}")
            return
        
        stage = prim.GetStage()
        if not stage:
            carb.log_error(f"Failed to retrieve stage for joint: {prim.GetPath()}")
            return

        xform_cache = UsdGeom.XformCache()
        
        offset_position_attr = (joint.GetLocalPos0Attr(), joint.GetLocalPos1Attr())
        offset_rotation_attr = (joint.GetLocalRot0Attr(), joint.GetLocalRot1Attr())
        body_rel_attr = (joint.GetBody0Rel(), joint.GetBody1Rel())
        for index in range(2):
            self.attachments[index].offset_translation = Gf.Vec3d(offset_position_attr[index].Get())
            self.attachments[index].offset_rotation = Gf.Quatd(offset_rotation_attr[index].Get())

            body_targets = body_rel_attr[index].GetTargets()

            attachment_prim = None
            if len(body_targets) == 1:
                attachment_prim = stage.GetPrimAtPath(body_targets[0])
            else:
                attachment_prim = None

            if attachment_prim is not None:
                self.attachments[index].prim_xform = xform_cache.GetLocalToWorldTransform(attachment_prim)
            else: 
                self.attachments[index].prim_xform = Gf.Matrix4d()

    def compute_offset_transform(self):
        return self.attachments[1].compute_transform() * self.attachments[0].compute_transform().GetInverse()

    def validate_orientation_offset(self, offset_transform: Gf.Matrix4d) -> bool:
        rotation = offset_transform.ExtractRotation()

        if math.fabs(rotation.GetAngle()) > JointValidator.TOLERANCE_ORIENTATION:
            return False

        return True

    def validate_position_offset(self, offset_transform: Gf.Matrix4d) -> bool:
        position_offset = offset_transform.ExtractTranslation()

        if position_offset.GetLength() > JointValidator.TOLERANCE_POSITION:
            return False

        return True

    def get_nearest_valid_position_offset(self, current_offset_transform: Gf.Matrix4d) -> Gf.Vec3d:
        return Gf.Vec3d()

    def get_nearest_valid_orientation_offset(self, current_offset_transform: Gf.Matrix4d) -> Gf.Rotation:
        return Gf.Rotation().SetIdentity()

    def make_pose_valid(self, prim: Usd.Prim):
        current_offset_transform = self.compute_offset_transform()

        valid_orientation_offset = self.get_nearest_valid_orientation_offset(current_offset_transform)
        valid_orientation_transform = Gf.Matrix4d().SetRotate(valid_orientation_offset)
        valid_position_offset = self.get_nearest_valid_position_offset(current_offset_transform)

        # Compute the desired world transform for the attachment 1, i.e. where we want the attachment point to be.
        desired_world_transform = (
            valid_orientation_transform *
            Gf.Matrix4d().SetTranslate(valid_position_offset) * 
            self.attachments[0].compute_transform()
        )

        # Compute the offsets in the attachment body 1's local space.
        desired_attachment1_transform = desired_world_transform * self.attachments[1].prim_xform.GetInverse()
        desired_attachment1_translate = desired_attachment1_transform.ExtractTranslation()
        desired_attachment1_rotation = desired_attachment1_transform.RemoveScaleShear().ExtractRotationQuat()

        joint = UsdPhysics.Joint(prim)
        offset_position_attr = joint.GetLocalPos1Attr()
        offset_rotation_attr = joint.GetLocalRot1Attr()

        offset_position_attr.Set(desired_attachment1_translate)
        offset_rotation_attr.Set(Gf.Quatf(desired_attachment1_rotation))

    def validate_pose(self) -> bool:
        offset_transform = self.compute_offset_transform()
        return self.validate_orientation_offset(offset_transform) and self.validate_position_offset(offset_transform)


class RevoluteJointValidator(JointValidator):
    def __init__(self, prim: Usd.Prim):
        super().__init__(prim)
        if not prim:
            return
        joint = UsdPhysics.RevoluteJoint(prim)
        if not joint:
            carb.log_error(f"Failed to retrieve revolute joint: {prim.GetPath()}")
            return

        self.axis = axis_usd_token_to_index.index(joint.GetAxisAttr().Get())
        lower_limit = joint.GetLowerLimitAttr().Get()
        upper_limit = joint.GetUpperLimitAttr().Get()
        self.limits = (lower_limit if lower_limit is not None else -math.inf, upper_limit if upper_limit is not None else math.inf)

    def validate_orientation_offset(self, offset_transform: Gf.Matrix4d) -> bool:
        orientation_offset = compute_transform_axial_rotation_offsets(offset_transform)

        if orientation_offset[self.axis] > 90.0 or orientation_offset[self.axis] < -90.0:
            # If we've rotated more than 90 degrees on the rotation axis, assume that the next orientation will be flipped.
            orientation_offset[(self.axis + 1) % 3] += 180.0 if orientation_offset[(self.axis + 1) % 3] < 0.0 else -180.0            

        # Check that the axial angular offsets are within tolerance.
        for axis in range(3):
            if axis == self.axis:
                if not within_range(orientation_offset[axis], self.limits[0], self.limits[1], JointValidator.TOLERANCE_ORIENTATION):
                    return False
            elif math.fabs(orientation_offset[axis]) > JointValidator.TOLERANCE_ORIENTATION:
                return False
        return True


class PrismaticJointValidator(JointValidator):
    def __init__(self, prim: Usd.Prim):
        super().__init__(prim)
        if not prim:
            return

        joint = UsdPhysics.PrismaticJoint(prim)
        if not joint:
            carb.log_error(f"Failed to retrieve prismatic joint: {prim.GetPath()}")
            return

        self.axis = axis_usd_token_to_index.index(joint.GetAxisAttr().Get())
        lower_limit = joint.GetLowerLimitAttr().Get()
        upper_limit = joint.GetUpperLimitAttr().Get()
        self.limits = (lower_limit if lower_limit is not None else -math.inf, upper_limit if upper_limit is not None else math.inf)

    def get_nearest_valid_position_offset(self, current_offset_transform: Gf.Matrix4d) -> Gf.Vec3d:
        position_offset = current_offset_transform.ExtractTranslation()
        
        for axis in range(3):
            if axis == self.axis:
                position_offset[axis] = clamp(position_offset[axis], self.limits[0], self.limits[1])
            else:
                position_offset[axis] = 0.0

        return position_offset

    def validate_position_offset(self, offset_transform: Gf.Matrix4d) -> bool:
        position_offset = offset_transform.ExtractTranslation()

        if not within_range(position_offset[self.axis], self.limits[0], self.limits[1], JointValidator.TOLERANCE_POSITION):
            return False

        for axis in range(3):
            if axis != self.axis and math.fabs(position_offset[axis]) > JointValidator.TOLERANCE_POSITION:
                return False

        return True        

class SphericalJointValidator(JointValidator):
    def __init__(self, prim: Usd.Prim):
        super().__init__(prim)
        if not prim:
            return

        joint = UsdPhysics.SphericalJoint(prim)
        if not joint:
            carb.log_error(f"Failed to retrieve spherical joint: {prim.GetPath()}")
            return

        self.axis = axis_usd_token_to_index.index(joint.GetAxisAttr().Get())
        cone_0_limit = joint.GetConeAngle0LimitAttr().Get()
        cone_1_limit = joint.GetConeAngle1LimitAttr().Get()
        self.limits = (cone_0_limit if cone_0_limit is not None else math.inf, cone_1_limit if cone_1_limit is not None else math.inf)


    def get_limit_in_direction(self, direction: float) -> float:
        # Project the limits to an elliptical cone. Use quarter angles to avoid negative angles and limit issues at 90 degrees.
        cone_quarter_angles = (math.radians(self.limits[0]) / 4.0, math.radians(self.limits[1]) / 4.0)
        cone_extents = (math.tan(cone_quarter_angles[0]), math.tan(cone_quarter_angles[1]))

        # Calculate the radius of the elliptical cone at the given axis angle on polar coordinates.
        cone_radius_at_angle = (cone_extents[0] * cone_extents[1] 
            / math.sqrt((cone_extents[0] * math.cos(math.radians(direction))) ** 2 + (cone_extents[1] * math.sin(math.radians(direction))) ** 2))

        # Derive the angle from the radius at the given direction and multiply by 4 to get the full limit.
        return math.degrees(math.atan(cone_radius_at_angle)) * 4

    def _get_swing(self, offset_transform: Gf.Matrix4d) -> tuple[float, float]:
        axis_orientation = offset_transform.GetRow3(self.axis)
        swing_direction = math.degrees(math.atan2(axis_orientation[(self.axis + 2) % 3], axis_orientation[(self.axis + 1) % 3]))
        swing_angle = math.degrees(math.acos(clamp(axis_orientation[self.axis], -1.0, 1.0)))
        return (swing_direction, swing_angle)

    def get_nearest_valid_orientation_offset(self, current_offset_transform: Gf.Matrix4d) -> Gf.Rotation:
        swing_direction, swing_angle = self._get_swing(current_offset_transform)
        limit_at_direction = self.get_limit_in_direction(swing_direction)

        if swing_angle - JointValidator.TOLERANCE_ORIENTATION > limit_at_direction:
            swing_axis = Gf.Rotation(Gf.Vec3d.Axis(self.axis), swing_direction + 90.0).TransformDir(Gf.Vec3d.Axis((self.axis + 1) % 3))
            nearest_valid_rotation = Gf.Rotation(swing_axis, limit_at_direction)
            return nearest_valid_rotation
        else:
            return current_offset_transform.ExtractRotation()

    def validate_orientation_offset(self, offset_transform: Gf.Matrix4d) -> bool:
        swing_direction, swing_angle = self._get_swing(offset_transform)
        return (swing_angle - JointValidator.TOLERANCE_ORIENTATION < self.get_limit_in_direction(swing_direction))

class DistanceJointValidator(JointValidator): 
    def __init__(self, prim: Usd.Prim):
        super().__init__(prim)
        if not prim:
            return

        joint = UsdPhysics.DistanceJoint(prim)
        if not joint:
            carb.log_error(f"Failed to retrieve distance joint: {prim.GetPath()}")
            return

        self.limits = (joint.GetMinDistanceAttr().Get(), joint.GetMaxDistanceAttr().Get())

    def get_nearest_valid_position_offset(self, current_offset_transform: Gf.Matrix4d):
        position_offset = current_offset_transform.ExtractTranslation()
        if position_offset.GetLength() > 0.0:
            offset_normal = position_offset.GetNormalized()
        else:
            offset_normal = Gf.Vec3d.ZAxis()

        distance = position_offset.GetLength()
        if distance < self.limits[0]:
            position_offset = offset_normal * self.limits[0]
        elif distance > self.limits[1]:
            position_offset = offset_normal * self.limits[1]

        return position_offset

    def get_nearest_valid_orientation_offset(self, current_offset_transform: Gf.Matrix4d):
        # Always valid for distance joint so just return the current orientation.
        return current_offset_transform.ExtractRotation()

    def validate_position_offset(self, offset_transform: Gf.Matrix4d) -> bool:
        position_offset = offset_transform.ExtractTranslation()
        distance = position_offset.GetLength()
        if not within_range(distance, self.limits[0], self.limits[1], JointValidator.TOLERANCE_POSITION):
            return False

        return True

    def validate_orientation_offset(self, offset_transform: Gf.Matrix4d) -> bool:
        return True

class D6JointValidator(JointValidator):
    def __init__(self, prim: Usd.Prim):
        super().__init__(prim)
        if not prim:
            return

        limits_translation_attr = (
            UsdPhysics.LimitAPI.Get(prim, UsdPhysics.Tokens.transX),
            UsdPhysics.LimitAPI.Get(prim, UsdPhysics.Tokens.transY),
            UsdPhysics.LimitAPI.Get(prim, UsdPhysics.Tokens.transZ)
        )

        if None in limits_translation_attr:
            carb.log_error(f"Failed to retrieve D6 limits for translation: {prim.GetPath()}")
            return

        self.limits_translation = tuple((limit_attr.GetLowAttr().Get(), limit_attr.GetHighAttr().Get()) for limit_attr in limits_translation_attr)

        limits_rotation_attr = (
            UsdPhysics.LimitAPI.Get(prim, UsdPhysics.Tokens.rotX),
            UsdPhysics.LimitAPI.Get(prim, UsdPhysics.Tokens.rotY),
            UsdPhysics.LimitAPI.Get(prim, UsdPhysics.Tokens.rotZ)
        )

        if None in limits_translation_attr:
            carb.log_error(f"Failed to retrieve D6 limits for rotation: {prim.GetPath()}")
            return

        self.limits_rotation = tuple((limit_attr.GetLowAttr().Get(), limit_attr.GetHighAttr().Get()) for limit_attr in limits_rotation_attr)

        limits_distance_attr = UsdPhysics.LimitAPI.Get(prim, UsdPhysics.Tokens.distance)
        if not limits_distance_attr:
            # For the distance limit attribute, it will return None if not applied.
            self.limits_distance = (None, None)
        else:
            self.limits_distance = (limits_distance_attr.GetLowAttr().Get(), limits_distance_attr.GetHighAttr().Get())

    def get_nearest_valid_position_offset(self, current_offset_transform: Gf.Matrix4d) -> Gf.Vec3d:
        translation = current_offset_transform.ExtractTranslation()
        if self.limits_distance[0] is not None and self.limits_distance[1] is not None:
            distance = translation.GetLength()
            if distance < self.limits_distance[0]:
                translation = translation.GetNormalized() * self.limits_distance[0]
            elif distance > self.limits_distance[1]:
                translation = translation.GetNormalized() * self.limits_distance[1]

        for axis in range(3):
            if self.limits_translation[axis][0] is not None and self.limits_translation[axis][1] is not None:
                translation[axis] = clamp(translation[axis], self.limits_translation[axis][0], self.limits_translation[axis][1])

        return translation

    def get_nearest_valid_orientation_offset(self, current_offset_transform: Gf.Matrix4d) -> Gf.Rotation:
        # Attempt to decompose the orientation offset into the nearest valid orientation to the current orientation.
        # Try all relevant Euler angle rotation orders for decomposing the rotation 
        # NB: D6 joints does not support proper Euler angles (e.g. XYX), only Tait-Bryan (e.g. XYZ).
        rotation_orders = ((0, 1, 2), (0, 2, 1), (1, 0, 2), (1, 2, 0), (2, 0, 1), (2, 1, 0))
        rotation = current_offset_transform.ExtractRotation()
        rotation_inverse = rotation.GetInverse()
        nearest_valid_rotation = None
        nearest_valid_rotation_delta_angle = math.inf

        for rotation_order in rotation_orders:
            rotation_decomposed = rotation.Decompose(Gf.Vec3d.Axis(rotation_order[0]), Gf.Vec3d.Axis(rotation_order[1]), Gf.Vec3d.Axis(rotation_order[2]))
            for axis in range(3):
                axis_index = rotation_order[axis]
                if self.limits_rotation[axis_index][0] is not None and self.limits_rotation[axis_index][1] is not None:
                    rotation_decomposed[axis] = clamp(rotation_decomposed[axis], self.limits_rotation[axis_index][0], self.limits_rotation[axis_index][1])

            rotation_clamped = Gf.Rotation(Gf.Vec3d.Axis(rotation_order[2]), rotation_decomposed[2]) * Gf.Rotation(Gf.Vec3d.Axis(rotation_order[1]), rotation_decomposed[1]) * Gf.Rotation(Gf.Vec3d.Axis(rotation_order[0]), rotation_decomposed[0])

            delta_angle = (rotation_inverse * rotation_clamped).GetAngle()
            # Check if this rotation is closer to the original rotation than the current nearest valid rotation. If so, prefer this instead.
            if math.fabs(delta_angle) < math.fabs(nearest_valid_rotation_delta_angle):
                nearest_valid_rotation = rotation_clamped
                nearest_valid_rotation_delta_angle = delta_angle

            if nearest_valid_rotation_delta_angle == 0.0:
                break

        if nearest_valid_rotation is None:
            # This should only be possible if the input transform is somehow invalid.
            carb.log_error("Unable to determine orientation offsets for D6 joint.")

        return nearest_valid_rotation

    def validate_position_offset(self, offset_transform: Gf.Matrix4d) -> bool:
        position_offset = offset_transform.ExtractTranslation()
        for axis in range(3):
            if (self.limits_translation[axis][0] is not None and self.limits_translation[axis][1] is not None and 
            not within_range(position_offset[axis], self.limits_translation[axis][0], self.limits_translation[axis][1], JointValidator.TOLERANCE_POSITION)):
                return False
        
        if self.limits_distance[0] is not None and self.limits_distance[1] is not None:
            distance = position_offset.GetLength()
            if not within_range(distance, self.limits_distance[0], self.limits_distance[1], JointValidator.TOLERANCE_POSITION):
                return False

        return True

    def validate_orientation_offset(self, offset_transform: Gf.Matrix4d) -> bool:
        nearest_valid_orientation = self.get_nearest_valid_orientation_offset(offset_transform)
        orientation_delta = offset_transform.ExtractRotation().GetInverse() * nearest_valid_orientation
        return (math.fabs(orientation_delta.GetAngle()) < JointValidator.TOLERANCE_ORIENTATION)


def make_joint_validator(prim: Usd.Prim) -> JointValidator:
    if prim.IsA(UsdPhysics.FixedJoint):
        return JointValidator(prim)
    elif prim.IsA(UsdPhysics.RevoluteJoint):
        return RevoluteJointValidator(prim)
    elif prim.IsA(UsdPhysics.PrismaticJoint):
        return PrismaticJointValidator(prim)
    elif prim.IsA(UsdPhysics.SphericalJoint):
        return SphericalJointValidator(prim)
    elif prim.IsA(UsdPhysics.DistanceJoint):
        return DistanceJointValidator(prim)
    elif prim.HasAPI(UsdPhysics.LimitAPI):
        return D6JointValidator(prim)
    else:
        return None
