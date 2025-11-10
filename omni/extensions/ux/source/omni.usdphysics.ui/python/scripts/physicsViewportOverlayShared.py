# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Usd, UsdPhysics, UsdGeom, Gf as UsdGf, Sdf, Tf
from usdrt import Usd as UsdRt, Rt as UsdRtGeom, UsdPhysics as UsdRtPhysics, Gf
from omni.ui_scene import scene as sc
from omni.ui import color as cl
import math
from math import isinf, isnan
import copy
import weakref
import carb
from enum import auto, IntEnum
from omni.usd import UsdContext
from collections.abc import Sequence, MutableSequence

# This color is useful for when wanting to have shapes invisible yet still react to gestures, like for hovering.
COLOR_INVISIBLE = cl("#00000000")

COLOR_HIGHLIGHT = cl("#FFFF66FF")
COLOR_HIGHLIGHT_SHADED = cl("#AAAA44FF")

COLOR_TEXT_HIGHLIGHT = cl("#FEFFFC")
COLOR_TEXT = cl("#ADF138")

# Helper functions for objects that does not inherently support unpacking fully.
def to_tuple(value) -> tuple[float]:
    match type(value):
        case Gf.Matrix4d | Gf.Matrix4f | UsdGf.Matrix4d | UsdGf.Matrix4f:
            return (
            value[0][0], value[0][1], value[0][2], value[0][3],
            value[1][0], value[1][1], value[1][2], value[1][3],
            value[2][0], value[2][1], value[2][2], value[2][3],
            value[3][0], value[3][1], value[3][2], value[3][3]
            )
        case Gf.Matrix3d | Gf.Matrix3f | UsdGf.Matrix3d | UsdGf.Matrix3f:
            return (
            value[0][0], value[0][1], value[0][2],
            value[1][0], value[1][1], value[1][2],
            value[2][0], value[2][1], value[2][2]
            )
        case Gf.Quatd | Gf.Quatf | UsdGf.Quatd | UsdGf.Quatf:
            return (value.GetReal(),) + to_tuple(value.GetImaginary())
        case Gf.Vec4d | Gf.Vec4f | UsdGf.Vec4d | UsdGf.Vec4f:
            return (value[0], value[1], value[2], value[3])
        case Gf.Vec3d | Gf.Vec3f | UsdGf.Vec3d | UsdGf.Vec3f:
            return (value[0], value[1], value[2])
        case Gf.Vec2d | Gf.Vec2f | UsdGf.Vec2d | UsdGf.Vec2f:
            return (value[0], value[1])
        case sc.Matrix44:
            return (
            value[0], value[1], value[2], value[3],
            value[4], value[5], value[6], value[7],
            value[8], value[9], value[10], value[11],
            value[12], value[13], value[14], value[15]
            )
        case _:
            return (*value,)

def to_list(value) -> list[float]:
    return list(to_tuple(value))

def to_usdrt(value):
    match type(value):
        case UsdGf.Vec2d:
            return Gf.Vec2d(value[0], value[1])
        case UsdGf.Vec3d:
            return Gf.Vec3d(value[0], value[1], value[2])
        case UsdGf.Vec4d:
            return Gf.Vec4d(value[0], value[1], value[2], value[3])
        case UsdGf.Quatd:
            return Gf.Quatd(value.GetReal(), to_usdrt(value.GetImaginary()))
        case UsdGf.Matrix3d:
            return Gf.Matrix3d().Set(*to_tuple(value))
        case UsdGf.Matrix4d:
            return Gf.Matrix4d().Set(*to_tuple(value))
        case UsdGf.Vec2f:
            return Gf.Vec2f(value[0], value[1])
        case UsdGf.Vec3f:
            return Gf.Vec3f(value[0], value[1], value[2])
        case UsdGf.Vec4f:
            return Gf.Vec4f(value[0], value[1], value[2], value[3])
        case UsdGf.Quatf:
            return Gf.Quatf(value.GetReal(), to_usdrt(value.GetImaginary()))
        case UsdGf.Matrix3f:
            return Gf.Matrix3f().Set(*to_tuple(value))
        case UsdGf.Matrix4f:
            return Gf.Matrix4f().Set(*to_tuple(value))
        case UsdGf.Rotation:
            return Gf.Rotation(to_usdrt_d(value.GetQuat()))
        case UsdGf.Range3d:
            return Gf.Range3d(to_usdrt_d(value.GetMin()), to_usdrt_d(value.GetMax()))
        case sc.Vector2:
            return Gf.Vec2d(value[0], value[1])
        case sc.Vector3:
            return Gf.Vec3d(value[0], value[1], value[2])
        case sc.Vector4:
            return Gf.Vec4d(value[0], value[1], value[2], value[3])
        case sc.Matrix44:
            return Gf.Matrix4d().Set(*to_tuple(value))
        case _:
            if isinstance(value, Sequence):
                length = len(value)
                if length == 2:
                    return Gf.Vec2d(value[0], value[1])
                elif length == 3:
                    return Gf.Vec3d(value[0], value[1], value[2])
                elif length == 4:
                    return Gf.Vec4d(value[0], value[1], value[2], value[3])
            return value

def to_usdrt_d(value):
    match type(value):
        case UsdGf.Vec2d | UsdGf.Vec2f:
            return Gf.Vec2d(value[0], value[1])
        case UsdGf.Vec3d | UsdGf.Vec3f:
            return Gf.Vec3d(value[0], value[1], value[2])
        case UsdGf.Vec4d | UsdGf.Vec4f:
            return Gf.Vec4d(value[0], value[1], value[2], value[3])
        case UsdGf.Quatd | UsdGf.Quatf:
            return Gf.Quatd(value.GetReal(), to_usdrt_d(value.GetImaginary()))
        case UsdGf.Matrix3d | UsdGf.Matrix3f:
            return Gf.Matrix3d().Set(*to_tuple(value))
        case UsdGf.Matrix4d | UsdGf.Matrix4f:
            return Gf.Matrix4d().Set(*to_tuple(value))
        case UsdGf.Rotation:
            return Gf.Rotation(to_usdrt_d(value.GetQuat()))
        case sc.Vector2:
            return Gf.Vec2d(value[0], value[1])
        case sc.Vector3:
            return Gf.Vec3d(value[0], value[1], value[2])
        case sc.Vector4:
            return Gf.Vec4d(value[0], value[1], value[2], value[3])
        case sc.Matrix44:
            return Gf.Matrix4d().Set(*to_tuple(value))
        case _:
            if isinstance(value, Sequence):
                length = len(value)
                if length == 2:
                    return Gf.Vec2d(value[0], value[1])
                elif length == 3:
                    return Gf.Vec3d(value[0], value[1], value[2])
                elif length == 4:
                    return Gf.Vec4d(value[0], value[1], value[2], value[3])
            return value

def to_usd(value):
    match type(value):
        case Gf.Vec2d:
            return UsdGf.Vec2d(value[0], value[1])
        case Gf.Vec3d:
            return UsdGf.Vec3d(value[0], value[1], value[2])
        case Gf.Vec4d:
            return UsdGf.Vec4d(value[0], value[1], value[2], value[3])
        case Gf.Quatd:
            return UsdGf.Quatd(value.GetReal(), to_usd(value.GetImaginary()))
        case Gf.Matrix3d:
            return UsdGf.Matrix3d(*to_tuple(value))
        case Gf.Matrix4d:
            return UsdGf.Matrix4d(*to_tuple(value))
        case Gf.Vec2f:
            return UsdGf.Vec2f(value[0], value[1])
        case Gf.Vec3f:
            return UsdGf.Vec3f(value[0], value[1], value[2])
        case Gf.Vec4f:
            return UsdGf.Vec4f(value[0], value[1], value[2], value[3])
        case Gf.Quatf:
            return UsdGf.Quatf(value.GetReal(), to_usd(value.GetImaginary()))
        case Gf.Matrix3f:
            return UsdGf.Matrix3f(*to_tuple(value))
        case Gf.Matrix4f:
            return UsdGf.Matrix4f(*to_tuple(value))
        case Gf.Rotation:
            return UsdGf.Rotation(to_usd(value.GetQuat()))
        case _:
            return value

def to_ui_scene(value):
    match type(value):
        case Gf.Matrix4d | Gf.Matrix4f | UsdGf.Matrix4d | UsdGf.Matrix4f:
            return sc.Matrix44(*to_tuple(value))
        case Gf.Vec2d | Gf.Vec2f | UsdGf.Vec2d | UsdGf.Vec2f:
            return sc.Vector2(*to_tuple(value))
        case Gf.Vec3d | Gf.Vec3f | UsdGf.Vec3d | UsdGf.Vec3f:
            return sc.Vector3(*to_tuple(value))
        case Gf.Vec4d | Gf.Vec4f | UsdGf.Vec4d | UsdGf.Vec4f:
            return sc.Vector4(*to_tuple(value))
        case _:
            return value

def axis_usd_token_to_index(axis : UsdGeom.Tokens) -> int:
    if axis == UsdGeom.Tokens.x:
        return 0
    elif axis == UsdGeom.Tokens.y:
        return 1
    return 2

def axis_index_to_usd_token(axis : int) -> UsdGeom.Tokens:
    if axis == 0:
        return UsdGeom.Tokens.x
    elif axis == 1:
        return UsdGeom.Tokens.y
    return UsdGeom.Tokens.z

def sqr(value):
    return value * value

def float_to_string(value, max_length = 12, precision=2) -> str:
    if value is None or not isinstance(value, float):
        return "-"
    elif isnan(value):
        return "NaN"
    elif value == 0.0:
        # Will often be the case so return early. Also prevents -0.0 output.
        return "0.0"
    elif isinf(value):
        if value > 0.0:
            return "Infinity"
        else:
            return "-Infinity"

    max_decimals = max_length - 2  # assume one is used for the decimal point, and there always being at least one number on each side.
    if value < 0.0:
        max_decimals -= 1  # negative sign

    highest_decimal = math.floor(math.log10(math.fabs(value)))
    if highest_decimal < 0:
        # Precision is relative to the first decimal if within -1 to 1.
        precision -= highest_decimal

    value = round(value, min(precision, max_decimals))

    if value == 0.0:
        # Prevents -0.0 output.
        return "0.0"

    if abs(highest_decimal) + 1 > max_decimals:
        return ("{value:>{max_length}.{decimals}e}").format(value=value, decimals=max(max_decimals - (4 if highest_decimal < 0 else 4), 1), max_length=max_length)

    if highest_decimal > 3:
        # Reserve space for 1k separators.
        max_decimals -= ((highest_decimal - 1 )// 3)

    value_string = ("{value:,.{decimals}f}").format(value=value, decimals=min(precision, max_decimals if max_decimals > 0 else 1))

    while (value_string[-1] == "0" and value_string[-2:] != ".0"):
        value_string = value_string[:-1]

    return value_string

def floats_to_string(values, max_length = 12, precision=2) -> str:
    first = True
    strReturn = ""
    for value in values:
        if not first:
            strReturn += "; "
        first = False
        strReturn += float_to_string(value, max_length, precision)
    return strReturn

def text_truncate(text : str, max_chars = 20):
    if len(text) > max_chars:
        text = "..." + text[-(max_chars-3):]
    return text

def direction_to_axis(direction) -> int:
    return int(direction / 2)

def direction_is_positive(direction) -> bool:
    return (direction == AxisDirection.x_pos or
            direction == AxisDirection.y_pos or
            direction == AxisDirection.z_pos)

def axis_to_direction(axis, positive: bool) -> int:
    return int(axis * 2 + (0 if positive else 1))

def axis_angle_to_quaternion(axis, angle):
    return Gf.Quatf(math.cos(angle / 2), axis * math.sin(angle / 2))

class Axis:
    x = 0
    y = 1
    z = 2
    num = 3

    vectors = (Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())

    vector_x = vectors[x]
    vector_y = vectors[y]
    vector_z = vectors[z]

class AxisDirection:
    x_pos = 0
    x_neg = 1
    y_pos = 2
    y_neg = 3
    z_pos = 4
    z_neg = 5
    num = 6

    vectors = (Gf.Vec3d.XAxis(), -Gf.Vec3d.XAxis(), 
              Gf.Vec3d.YAxis(), -Gf.Vec3d.YAxis(), 
              Gf.Vec3d.ZAxis(), -Gf.Vec3d.ZAxis())

    vector_x_pos = vectors[x_pos]
    vector_x_neg = vectors[x_neg]
    vector_y_pos = vectors[y_pos]
    vector_y_neg = vectors[y_neg]
    vector_z_pos = vectors[z_pos]
    vector_z_neg = vectors[z_neg]

# Multiplies each component of a with b.
def components_multiply_by(a : MutableSequence, b):
    if isinstance(b, Sequence):
        for n in range(min(len(a), len(b))):
            a[n] *= b[n]
    else:
        for n in range(len(a)):
            a[n] *= b
       
# Multiplies by component into a copy of a.
def components_product(a : Sequence, b) -> MutableSequence:
    # if type(a) is tuple:
    if not isinstance(a, MutableSequence): 
        c = [*a]
    else:
        c = copy.copy(a)

    components_multiply_by(c, b)
    return c

# Divides each component of a with b.
def components_divide_by(a : MutableSequence, b):
    if isinstance(b, Sequence):
        for n in range(min(len(a), len(b))):
            a[n] /= b[n]
    else:
        for n in range(len(a)):
            a[n] /= b

# Divides per component into a copy of a
def components_quotients(a : Sequence, b) -> MutableSequence:
    if not isinstance(a, MutableSequence):
        c = [*a]
    else:
        c = copy.copy(a)

    components_divide_by(c, b)
    return c

# Add each component of b to a.
def components_add_to(a : MutableSequence, b):
    if isinstance(b, Sequence):
        for n in range(min(len(a), len(b))):
            a[n] += b[n]
    else:
        for n in range(len(a)):
            a[n] += b

# Add per component into a copy of a
def components_sum(a : Sequence, b) -> MutableSequence:
    if not isinstance(a, MutableSequence):
        c = [*a]
    else:
        c = copy.copy(a)

    components_add_to(c, b)
    return c

# Subtracts each component of b from a.
def components_subtract_by(a : MutableSequence, b):
    if isinstance(b, Sequence):
        for n in range(min(len(a), len(b))):
            a[n] -= b[n]
    else:
        for n in range(min(len(a), len(b))):
            a[n] -= b

# Subracts per component into a copy of a
def components_difference(a : Sequence, b) -> MutableSequence:
    if not isinstance(a, MutableSequence):
        c = [*a]
    else:
        c = copy.copy(a)

    components_subtract_by(c, b)
    return c

def components_equal(a : Sequence, b) -> bool:
    if isinstance(b, Sequence):
        if len(a) != len(b):
            return False
        
        for n in range(len(a)):
            if a[n] != b[n]:
                return False
    else:
        for n in range(len(a)):
            if a[n] != b:
                return False

    return True

def clamp(value, lower, upper):
    return max(lower, min(value, upper))

def components_clamp(a : Sequence, lower, upper) -> MutableSequence:
    if not isinstance(a, MutableSequence):
        c = [*a]
    else:
        c = copy.copy(a)
    
    lower_seq = isinstance(lower, Sequence)
    upper_seq = isinstance(upper, Sequence)
    for n in range(len(c)):
        c[n] = clamp(c[n], lower if not lower_seq else lower[n], upper if not upper_seq else upper[n])

    return c

def components_max(a : Sequence, b) -> MutableSequence:
    if not isinstance(a, MutableSequence):
        c = [*a]
    else:
        c = copy.copy(a)
    
    if isinstance(b, Sequence):
        for n in range(len(c)):
            c[n] = max(c[n], b[n])
    else:
        for n in range(len(c)):
            c[n] = max(c[n], b)

    return c

def components_min(a : Sequence, b) -> MutableSequence:
    if not isinstance(a, MutableSequence):
        c = [*a]
    else:
        c = copy.copy(a)
    
    if isinstance(b, Sequence):
        for n in range(len(c)):
            c[n] = min(c[n], b[n])
    else:
        for n in range(len(c)):
            c[n] = min(c[n], b)

    return c

def lerp(a: float, b: float, factor: float):
    return a * (1.0 - factor) + b * factor

def components_lerp_with(a : MutableSequence, b : Sequence, factor: float):
    for n in range(min(len(a), len(b))):
        a[n] = lerp(a[n], b[n], factor)

def components_lerped(a : Sequence, b : Sequence, factor: float) -> MutableSequence:
    if not isinstance(a, MutableSequence):
        c = [*a]
    else:
        c = copy.copy(a)
    components_lerp_with(c, b, factor)
    return c

def color_with_alpha(color_in : int, alpha : float):
    color = color_in & ~0xff000000
    return color | (min(math.floor(alpha * 256.0), 255) << 24)


UI_RESCALE_FACTOR = 2.0  # This is due to the UI.scene screen scaling error/inconsistency, see OM-76738 

def ui_create_screen_scale_transform(aligned_to_camera = False):
    return sc.Transform(look_at=(sc.Transform.LookAt.CAMERA if aligned_to_camera else sc.Transform.LookAt.NONE), scale_to=sc.Space.SCREEN, transform=sc.Matrix44.get_scale_matrix(UI_RESCALE_FACTOR, UI_RESCALE_FACTOR, (1.0 if aligned_to_camera else UI_RESCALE_FACTOR)))


def ui_draw_arrow(color, direction, length, radius, offset=0.0, tesselation=24, direction_orthogonal=None):
    base_plane_vector = direction_orthogonal
    if not direction_orthogonal:
        # Find a vector on the plane where direction is normal. Any will do.
        base_plane_vector = Gf.Vec3d(direction[1], direction[2], direction[0])
        base_plane_vector = Gf.Vec3d.GetCross(direction, base_plane_vector)
        base_plane_vector.Normalize()
    ortho_base_plane_vector = Gf.Vec3d.GetCross(direction, base_plane_vector)
    base_plane_vector = base_plane_vector * radius
    ortho_base_plane_vector = ortho_base_plane_vector * radius
    arrow_vertex_indices = []
    coords = direction * (length + offset)
    arrow_points = [(coords[0], coords[1], coords[2])]
    arrow_faces_vertex_count = []
    angle = 0.0
    angle_cos = 1.0
    angle_sin = 0.0
    coords = base_plane_vector + direction * offset
    arrow_points.append((coords[0], coords[1], coords[2]))
    for i in range(tesselation):
        angle += 2.0 * math.pi / tesselation
        angle_cos = math.cos(angle)
        angle_sin = math.sin(angle)

        # Rotate using Rodrigues' rotation formula.
        coords = base_plane_vector * angle_cos + ortho_base_plane_vector * angle_sin + direction * offset
        arrow_points.append((coords[0], coords[1], coords[2]))
        arrow_vertex_indices.append(0)
        arrow_vertex_indices.append(len(arrow_points) - 2)
        arrow_vertex_indices.append(len(arrow_points) - 1)
        arrow_faces_vertex_count.append(3)

    return sc.PolygonMesh(arrow_points, [color] * 3 * len(arrow_faces_vertex_count), arrow_faces_vertex_count, arrow_vertex_indices)

def ui_draw_box(color, thickness, extents : tuple[tuple[float, float, float], tuple[float, float, float]] = ((-0.5, -0.5, -0.5), (0.5, 0.5, 0.5))):
    cube_corners = (
                        (extents[0][0], extents[0][1], extents[0][2]), 
                        (extents[1][0], extents[0][1], extents[0][2]), 
                        (extents[0][0], extents[1][1], extents[0][2]), 
                        (extents[1][0], extents[1][1], extents[0][2]), 
                        (extents[0][0], extents[0][1], extents[1][2]), 
                        (extents[1][0], extents[0][1], extents[1][2]), 
                        (extents[0][0], extents[1][1], extents[1][2]),
                        (extents[1][0], extents[1][1], extents[1][2])
                    )

    sc.Line(cube_corners[0], cube_corners[1], color=color, thickness=thickness)
    sc.Line(cube_corners[0], cube_corners[2], color=color, thickness=thickness)
    sc.Line(cube_corners[0], cube_corners[4], color=color, thickness=thickness)
    sc.Line(cube_corners[1], cube_corners[3], color=color, thickness=thickness)
    sc.Line(cube_corners[1], cube_corners[5], color=color, thickness=thickness)
    sc.Line(cube_corners[2], cube_corners[3], color=color, thickness=thickness)
    sc.Line(cube_corners[2], cube_corners[6], color=color, thickness=thickness)
    sc.Line(cube_corners[3], cube_corners[7], color=color, thickness=thickness)
    sc.Line(cube_corners[4], cube_corners[5], color=color, thickness=thickness)
    sc.Line(cube_corners[4], cube_corners[6], color=color, thickness=thickness)
    sc.Line(cube_corners[5], cube_corners[7], color=color, thickness=thickness)
    sc.Line(cube_corners[6], cube_corners[7], color=color, thickness=thickness)


def ui_draw_stippled_line(start, end, color, thickness, stipples = 7):
    offset = components_product(components_difference(end, start), [1 / (stipples * 2 - 1)] * 3)
    coord = to_list(start)
    for _ in range(stipples):
        vector_begin = coord.copy()
        components_add_to(coord, offset)
        sc.Line(start=vector_begin, end=coord, color=color, thickness=thickness)
        components_add_to(coord, offset)


def ui_draw_corner_box(color, thickness, extents : tuple[tuple[float, float, float], tuple[float, float, float]] = ((-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)), quotient = 0.5):
    cube_corners = (
                        (extents[0][0], extents[0][1], extents[0][2]), 
                        (extents[1][0], extents[0][1], extents[0][2]), 
                        (extents[0][0], extents[1][1], extents[0][2]), 
                        (extents[1][0], extents[1][1], extents[0][2]), 
                        (extents[0][0], extents[0][1], extents[1][2]), 
                        (extents[1][0], extents[0][1], extents[1][2]), 
                        (extents[0][0], extents[1][1], extents[1][2]),
                        (extents[1][0], extents[1][1], extents[1][2])
                    )

    def draw_line(start, end):
        offset = components_difference(end, start)
        sc.Line(start=start, end=components_sum(start, components_product(offset, [quotient * 0.5] * 3)), color=color, thickness=thickness)
        sc.Line(start=end, end=components_sum(start, components_product(offset, [1.0 - quotient * 0.5] * 3)), color=color, thickness=thickness)

    draw_line(cube_corners[0], cube_corners[1])
    draw_line(cube_corners[0], cube_corners[2])
    draw_line(cube_corners[0], cube_corners[4])
    draw_line(cube_corners[1], cube_corners[3])
    draw_line(cube_corners[1], cube_corners[5])
    draw_line(cube_corners[2], cube_corners[3])
    draw_line(cube_corners[2], cube_corners[6])
    draw_line(cube_corners[3], cube_corners[7])
    draw_line(cube_corners[4], cube_corners[5])
    draw_line(cube_corners[4], cube_corners[6])
    draw_line(cube_corners[5], cube_corners[7])
    draw_line(cube_corners[6], cube_corners[7])

class UsdRtXformQuery():
    def __init__(self, xformable : UsdRtGeom.Xformable):
        self._xformable = xformable
        self._xformable_attr_fabric_hierarchy_world_matrix = None

    @property
    def xform(self) -> Gf.Matrix4d | None:
        xformable = self._xformable
        if xformable is None:
            return None

        xformable_attr_fabric_hierarchy_world_matrix = self._xformable_attr_fabric_hierarchy_world_matrix

        if xformable_attr_fabric_hierarchy_world_matrix is None:
            xformable_attr_fabric_hierarchy_world_matrix = xformable.GetFabricHierarchyWorldMatrixAttr()
            if not xformable_attr_fabric_hierarchy_world_matrix.IsValid():
                return None
            self._xformable_attr_fabric_hierarchy_world_matrix = xformable_attr_fabric_hierarchy_world_matrix

        return xformable_attr_fabric_hierarchy_world_matrix.Get()


class PhysicsUIGestureManager(sc.GestureManager):
    def __init__(self, viewport_overlay):
        super().__init__()
        self.viewport_overlay = viewport_overlay
        self._active_gesture : weakref.ref = None
        self._active_hover : weakref.ref = None
        self._active_toggle : weakref.ref = None

        # Track a hover stack for whenever hovering objects overlap.
        self._suspended_hovers = []

    @property
    def active_gesture(self):
        if self._active_gesture is not None:
            return self._active_gesture()
        return None
    
    @active_gesture.setter
    def active_gesture(self, gesture):
        self._active_gesture = weakref.ref(gesture) if gesture is not None else None
        self._manage_active_toggle()

    @property
    def active_hover(self):
        if self._active_hover is not None:
            return self._active_hover()
        return None
    
    @active_hover.setter
    def active_hover(self, gesture):
        if gesture is None:
            self._active_hover = None
            # Check if we have a suspended hover that we should reactivate.
            while len(self._suspended_hovers) > 0:
                suspended = self._suspended_hovers.pop()()
                if suspended is not None and suspended.suspended:
                    if suspended.sender is None:
                        carb.log_warn(f"Detected an unattached suspended hover gesture: {suspended} Manipulator: {suspended.manipulator} State: {suspended.state}")
                    elif not suspended.sender.visible or not suspended.sender.compute_visibility():
                        carb.log_warn(f"Detected an invisible suspended hover gesture: {suspended} Manipulator: {suspended.manipulator} State: {suspended.state}")
                    elif suspended.state != sc.GestureState.CHANGED and suspended.state != sc.GestureState.BEGAN:
                        carb.log_warn(f"Detected a suspended hover gesture that is no longer active: {suspended} Manipulator: {suspended.manipulator} State: {suspended.state}")
                    else:
                        suspended.call_on_began_fn(suspended.sender)
                        break
        else:
            active_hover = self.active_hover
            if active_hover == gesture:
                return
            self._active_hover = weakref.ref(gesture)
            if active_hover is not None and not active_hover.suspended:
                # Suspend the current hover for reactivation whenever the new hover becomes inactive (provided the old is still relevant).
                active_hover.on_ended()
                self._suspended_hovers.append(weakref.ref(active_hover))
                active_hover.suspended = True

        self._manage_active_toggle()

    def _manage_active_toggle(self):
        if self.active_gesture is not None:
            new_gesture_toggle_group = self.active_gesture.toggle_group
        else:
            new_gesture_toggle_group = None
        
        if (new_gesture_toggle_group is None and 
            self.active_hover is not None and
            self.active_hover.toggle_group is not None
            ):
            new_gesture_toggle_group = self.active_hover.toggle_group

        active_toggle = self._active_toggle() if self._active_toggle is not None else None
        if new_gesture_toggle_group is not active_toggle:
            if active_toggle is not None:
                active_toggle.toggle(False)
            if new_gesture_toggle_group is not None:
                new_gesture_toggle_group.toggle(True)
                self._active_toggle = weakref.ref(new_gesture_toggle_group)
            else:
                self._active_toggle = None

        self.viewport_overlay.refresh_clipping_state()

    def should_prevent(self, gesture, preventer):
        if (gesture.state == sc.GestureState.POSSIBLE or 
            gesture.state == sc.GestureState.BEGAN or 
            gesture.state == sc.GestureState.CHANGED):
            active_hover = self.active_hover
            active_gesture = self.active_gesture

            if not isinstance(gesture, PhysicsUIGesture):
                if active_gesture or active_hover:
                    return True
                else:
                    return False

            if active_gesture:
                if active_gesture == gesture:
                    return False
                else:
                    return True

            # If we are hovering a gesture and the manipulator or toggle group differs, prevent.
            if active_hover:
                if (active_hover.manipulator != gesture.manipulator or 
                    active_hover.toggle_group != gesture.toggle_group):
                    # Compare which is closer to the screen.
                    active_hover_ndc = active_hover.sender.transform_space(sc.Space.OBJECT, sc.Space.NDC, [0.0, 0.0, 0.0])
                    sender_ndc = gesture.sender.transform_space(sc.Space.OBJECT, sc.Space.NDC, [0.0, 0.0, 0.0])
                    if active_hover_ndc[2] < sender_ndc[2]:
                        return True
                    else:
                        return False
                else:
                    return False

        # This seem to block all gestures (?)
        # return super().should_prevent(gesture, preventer)
        return False 



class PhysicsUIViewportOverlay:

    @property
    def stage(self) -> Usd.Stage:
        return self.manager.stage

    @property
    def stage_usdrt(self) -> UsdRt.Stage:
        return self.manager.stage_usdrt

    @property
    def simulation_active(self) -> bool:
        return self.manager.simulation_active

    def __init__(self, viewport_overlay_manager):
        # These are all shallow copies of persisting objects kept for fast access.
        self.manager = viewport_overlay_manager
        self.xform_cache : UsdGeom.XformCache = viewport_overlay_manager.xform_cache
        self.bbox_cache : UsdGeom.BBoxCache = viewport_overlay_manager.bbox_cache
        self.usd_context : UsdContext = viewport_overlay_manager.usd_context

    def destroy(self):
        self.manager = None
        self.xform_cache = None
        self.bbox_cache = None
        self.usd_context = None

    def __del__(self):
        if self.manager is not None:
            self.destroy()

    def subscribes_to_usd_prim_paths_info_changed(self) -> bool:
        return False

    def on_usd_prim_paths_info_changed(self, changed_paths: set[Sdf.Path]):
        pass

    def subscribes_to_usd_prim_paths_resynced(self) -> bool:
        return False

    def on_usd_prim_paths_resynced(self, resynced_paths: set[Sdf.Path]):
        pass

    def on_kit_update(self):
        pass
    
    def refresh_clipping_state(self):
        self.manager.refresh_clipping_state()


class PhysicsUIManipulator(sc.Manipulator):

    @property
    def stage(self) -> Usd.Stage:
        return self.manager.stage

    @property
    def stage_usdrt(self) -> UsdRt.Stage:
        return self.manager.stage_usdrt

    @property
    def simulation_active(self) -> bool:
        return self.manager.simulation_active

    def __init__(self, viewport_overlay: PhysicsUIViewportOverlay, **kwargs):
        super().__init__(**kwargs)
        # These are all shallow copies of persisting objects kept for fast access.
        self.viewport_overlay : PhysicsUIViewportOverlay = viewport_overlay
        self.manager = viewport_overlay.manager
        self.gesture_manager : PhysicsUIGestureManager = viewport_overlay.manager.gesture_manager
        self.xform_cache : UsdGeom.XformCache = viewport_overlay.xform_cache
        self.bbox_cache : UsdGeom.BBoxCache = viewport_overlay.bbox_cache
        self.usd_context : UsdContext = viewport_overlay.usd_context

    def _reset(self):
        if self.gesture_manager is None:
            return

        if self.gesture_manager.active_gesture is not None and self.gesture_manager.active_gesture.manipulator == self:
            self.gesture_manager.active_gesture = None


        active_hover = self.manager.gesture_manager.active_hover

        # As disabling a hover may cause an overlapping suspended hover to be reactivated, we have to repeat the check.
        while active_hover is not None and active_hover.manipulator == self:
            # Make sure that we trigger the correct setter and getter behavior.
            self.manager.gesture_manager.active_hover = None
            active_hover = self.manager.gesture_manager.active_hover

    def destroy(self):
        if self.viewport_overlay is not None:
            self._reset()
            self.viewport_overlay = None
            self.manager = None
            self.gesture_manager = None
            self.xform_cache = None
            self.bbox_cache = None
            self.usd_context = None
            self.invalidate()

    def __del__(self):
        self.destroy()

    def get_is_valid(self) -> bool:
        if self.viewport_overlay is not None:
            return True
        return False

    def get_is_active(self) -> bool:
        return self.get_is_valid()

    def on_build(self):
        self._reset()


class VisualToggle():
    def __init__(self, ui_toggle_object: sc.AbstractItem):
        self._ui_object: sc.AbstractItem = ui_toggle_object

    def toggle(self, enabled):
        pass


class VisibilityToggle(VisualToggle):
    def __init__(self, ui_toggle_object: sc.AbstractItem):
        super().__init__(ui_toggle_object)
        self._ui_object.visible = False

    def toggle(self, enabled):
        super().toggle(enabled)
        self._ui_object.visible = enabled

class ImageToggle(VisualToggle):
    def __init__(self, ui_toggle_object: sc.Image, enabled_image: str, disabled_image: str):
        super().__init__(ui_toggle_object)
        self._enabled_image = enabled_image
        self._disabled_image = disabled_image
        self._ui_object.source_url = disabled_image

    def toggle(self, enabled):
        super().toggle(enabled)
        self._ui_object.source_url = self._enabled_image if enabled else self._disabled_image


class ColorToggle(VisualToggle):
    def __init__(self, ui_toggle_object: sc.AbstractItem, enabled_color, disabled_color = None):
        super().__init__(ui_toggle_object)
        self._enabled_color = enabled_color
        if disabled_color:
            self._disabled_color = disabled_color
            if isinstance(self._ui_object, sc.PolygonMesh):
                self._ui_object.colors = [disabled_color] * len(self._ui_object.colors)
            else:
                self._ui_object.color = disabled_color
        else:
            if isinstance(self._ui_object, sc.PolygonMesh):
                self._disabled_color = self._ui_object.colors[0]
            else:
                self._disabled_color = self._ui_object.color

    def toggle(self, enabled):
        super().toggle(enabled)
        if isinstance(self._ui_object, sc.PolygonMesh):
            self._ui_object.colors = ([self._enabled_color] * len(self._ui_object.colors)) if enabled else ([self._disabled_color] * len(self._ui_object.colors))
        else:
            self._ui_object.color = self._enabled_color if enabled else self._disabled_color


# Just a shorthand for ColorToggle with the shared highlight color.
class HighlightToggle(ColorToggle):
    def __init__(self, ui_toggle_object: sc.AbstractItem):
        super().__init__(ui_toggle_object, COLOR_TEXT_HIGHLIGHT if isinstance(ui_toggle_object, sc.Label) else COLOR_HIGHLIGHT)


class PhysicsUIToggleGroup():
    def __init__(self) -> None:
        self._toggles : dict[PhysicsUIManipulator, list[VisualToggle]] = {}

    def toggle(self, enabled):
        for manipulator_toggles in self._toggles.values():
            for toggle in manipulator_toggles:
                toggle.toggle(enabled)

    def get_manipulator_toggles(self, manipulator: PhysicsUIManipulator) -> list[VisualToggle]:
        toggles = self._toggles.get(manipulator)
        if toggles is not None:
            return toggles
        else:
            toggles = []
            self._toggles[manipulator] = toggles
            return toggles

    def __del__(self):
        self._toggles.clear()


class PhysicsUIGesture():
    def __init__(self, manipulator: PhysicsUIManipulator, toggle_group=None):
        self.manipulator = manipulator
        self.toggle_group = toggle_group


class PhysicsUIHoverGesture(sc.HoverGesture, PhysicsUIGesture):
    def __init__(self, manipulator: PhysicsUIManipulator, toggle_group : PhysicsUIToggleGroup=None, *args, **kwargs):
        PhysicsUIGesture.__init__(self, manipulator, toggle_group)

        if not kwargs:
            kwargs = dict(manager = manipulator.viewport_overlay.manager.gesture_manager)
        elif not "manager" in kwargs:
            kwargs["manager"] = manipulator.viewport_overlay.manager.gesture_manager

        self.suspended = False
        super().__init__(*args, **kwargs)

    def on_began(self):
        self.suspended = False
        self.manager.active_hover = self

    def on_changed(self):
        pass

    def on_ended(self):
        active_hover = self.manager.active_hover
        if active_hover == self:
            self.manager.active_hover = None

        self.suspended = False


class PhysicsUIDragGesture(sc.DragGesture, PhysicsUIGesture):
    def __init__(self, manipulator: PhysicsUIManipulator, toggle_group : PhysicsUIToggleGroup=None, *args, **kwargs):
        PhysicsUIGesture.__init__(self, manipulator, toggle_group)
        if not kwargs:
            kwargs = dict(manager = manipulator.viewport_overlay.manager.gesture_manager)
        elif not "manager" in kwargs:
            kwargs["manager"] = manipulator.viewport_overlay.manager.gesture_manager

        super().__init__(*args, **kwargs)

    def on_began(self):
        self.manager.active_gesture = self

    def on_changed(self):
        pass

    def on_ended(self):
        if self.manager.active_gesture == self:
            self.manager.active_gesture = None


class PhysicsUIClickGesture(sc.ClickGesture, PhysicsUIGesture):
    def __init__(self, manipulator: PhysicsUIManipulator, toggle_group : PhysicsUIToggleGroup=None, *args, **kwargs):
        PhysicsUIGesture.__init__(self, manipulator, toggle_group)
        if not kwargs:
            kwargs = dict(manager = manipulator.viewport_overlay.manager.gesture_manager)
        elif not "manager" in kwargs:
            kwargs["manager"] = manipulator.viewport_overlay.manager.gesture_manager

        super().__init__(*args, **kwargs)

    def on_ended(self):
        pass
