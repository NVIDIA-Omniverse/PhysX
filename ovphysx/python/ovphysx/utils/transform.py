# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-UTILS-001
# @covers AC-1 AC-2 AC-4 AC-12

"""Transform and xform-op helpers.

The physics runtime expects a translate-orient-scale xform op stack, and
authoring one correctly by hand is fiddly: an op may already exist at a
different precision, and ``resetXformStack`` has to survive a rewrite. These
helpers encapsulate that.
"""

import logging
import typing

from pxr import Gf, Usd, UsdGeom

from .constants import AXES_INDICES

logger = logging.getLogger(__name__)

__all__ = [
    "get_axis_aligned_vector",
    "get_forward_vector",
    "get_basis",
    "get_unit_scale_factor",
    "get_translation",
    "get_world_position",
    "get_aligned_body_transform",
    "set_or_add_scale_op",
    "set_or_add_translate_op",
    "set_or_add_orient_op",
    "set_or_add_scale_orient_translate",
    "setup_transform_as_scale_orient_translate",
    "copy_transform_as_scale_orient_translate",
]


def get_axis_aligned_vector(axis, len):
    """Build an axis-aligned vector of the given length.

    Args:
        axis: The axis name - "X", "Y" or "Z".
        len:  The signed length along that axis.
    """
    vec = Gf.Vec3f(0.0)
    vec[AXES_INDICES[axis]] = len
    return vec


def get_forward_vector(up_axis):
    """Get the forward vector for a stage's up axis.

    Args:
        up_axis: The stage up axis - "Y" or "Z".
    """
    if up_axis == "Y":
        return get_axis_aligned_vector("Z", -1)
    elif up_axis == "Z":
        return get_axis_aligned_vector("X", -1)


def get_basis(up_axis):
    """Get the (up, forward, right) basis vectors for a stage's up axis.

    Args:
        up_axis: The stage up axis - "Y" selects a Y-up basis, anything else
                 selects a Z-up basis.
    """
    if up_axis == "Y":
        up = Gf.Vec3d(0, 1, 0)
        forward = Gf.Vec3d(0, 0, -1)
        right = Gf.Vec3d(1, 0, 0)
    else:
        up = Gf.Vec3d(0, 0, 1)
        forward = Gf.Vec3d(-1, 0, 0)
        right = Gf.Vec3d(0, 1, 0)
    return up, forward, right


def get_unit_scale_factor(stage):
    """Get the stage's units-per-meter scale factor.

    Args:
        stage: The Usd.Stage to query.
    """
    metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
    scaleFactor = 1.0 / metersPerUnit
    return scaleFactor


def get_translation(prim: Usd.Prim) -> typing.Union[Gf.Vec3f, Gf.Vec3d]:
    """Return the translate xform op value from the given prim.

    Args:
        prim:      The Usd.Prim to check.
    """
    return prim.GetAttribute("xformOp:translate").Get()


def get_world_position(stage, path):
    """Get the world-space center of a prim's axis-aligned world bound.

    Args:
        stage: The Usd.Stage to query.
        path:  The path of the imageable prim.
    """
    imageable = UsdGeom.Imageable.Get(stage, path)
    obb = imageable.ComputeWorldBound(Usd.TimeCode.Default(), purpose1=imageable.GetPurposeAttr().Get())
    alignedBox = obb.ComputeAlignedBox()
    minCorner = alignedBox.GetMin()
    maxCorner = alignedBox.GetMax()
    return (minCorner + maxCorner) / 2.0


def get_aligned_body_transform(stage, cache, joint, body0base):
    """Compute the transform that aligns one joint body onto the other.

    Args:
        stage:     The Usd.Stage holding the bodies.
        cache:     A UsdGeom.XformCache used for the world transforms.
        joint:     The UsdPhysics.Joint to read.
        body0base: True to treat body0 as the fixed base, False for body1.
    """
    b0paths = joint.GetBody0Rel().GetTargets()
    b1paths = joint.GetBody1Rel().GetTargets()

    b0prim = None
    b1prim = None

    if len(b0paths):
        b0prim = stage.GetPrimAtPath(b0paths[0])
        if not b0prim.IsValid():
            b0prim = None

    if len(b1paths):
        b1prim = stage.GetPrimAtPath(b1paths[0])
        if not b1prim.IsValid():
            b1prim = None

    b0locpos = joint.GetLocalPos0Attr().Get()
    b1locpos = joint.GetLocalPos1Attr().Get()
    b0locrot = joint.GetLocalRot0Attr().Get()
    b1locrot = joint.GetLocalRot1Attr().Get()

    if body0base:
        t0prim = b0prim
        t0locpos = b0locpos
        t0locrot = b0locrot
        t1prim = b1prim
    else:
        t0prim = b1prim
        t0locpos = b1locpos
        t0locrot = b1locrot
        t1prim = b0prim

    if t0prim:
        t0world = cache.GetLocalToWorldTransform(t0prim)
    else:
        t0world = Gf.Matrix4d()
        t0world.SetIdentity()

    if t1prim:
        t1world = cache.GetLocalToWorldTransform(t1prim)
    else:
        t1world = Gf.Matrix4d()
        t1world.SetIdentity()

    t0local = Gf.Transform()
    t0local.SetRotation(Gf.Rotation(Gf.Quatd(t0locrot)))
    t0local.SetTranslation(Gf.Vec3d(t0locpos))
    t0mult = t0local * Gf.Transform(t0world)

    t1world = Gf.Transform(t1world.GetInverse())
    rel_tm = t0mult * t1world
    return rel_tm


def _get_or_create_xform_op(
    xformable: UsdGeom.Xformable, op_name: str, op_type: str, op_precision_if_create=UsdGeom.XformOp.PrecisionFloat
) -> UsdGeom.XformOp:
    """Get or create an XformOp of an Xformable.

    Args:
        xformable:  The Xformable to modify.
        op_name:    The XformOp attribute name, e.g. "xformOp:translate"
        op_type:    The XformOp type, e.g. UsdGeom.XformOp.TypeScale
    """
    dstOp = UsdGeom.XformOp(xformable.GetPrim().GetAttribute(op_name))
    if not dstOp:
        dstOp = xformable.AddXformOp(op_type, op_precision_if_create)
    return dstOp


def set_or_add_scale_op(
    xformable: UsdGeom.Xformable, scale: typing.Union[Gf.Vec3f, Gf.Vec3d, Gf.Vec3h]
) -> typing.Union[UsdGeom.XformOp, typing.Literal[False]]:
    """
    Sets or adds the scale XformOp on the input Xformable to provided scale value.

    Note that:
        - The precision of an added attribute is UsdGeom.XformOp.PrecisionFloat.

    Args:
        xformable:  The Xformable to modify.
        scale:      The scale vector
    Returns:
        The set or added XformOp, or ``False`` if the prim is not an Xformable.
    """
    prim = xformable.GetPrim()
    if not (prim.IsA(UsdGeom.Xformable)):
        logger.warning(f"{__name__}.set_or_add_scale_op: Provided prim is not an Xformable")
        return False
    xformOp = _get_or_create_xform_op(xformable, "xformOp:scale", UsdGeom.XformOp.TypeScale)
    if xformOp.Get() is None:
        xformOp.Set(Gf.Vec3f(scale))
    else:
        typeName = type(xformOp.Get())
        xformOp.Set(typeName(scale))
    return xformOp


def set_or_add_translate_op(
    xformable: UsdGeom.Xformable, translate: typing.Union[Gf.Vec3f, Gf.Vec3d, Gf.Vec3h]
) -> typing.Union[UsdGeom.XformOp, typing.Literal[False]]:
    """
    Sets or adds the translate XformOp on the input Xformable to provided translate value.

    Note that:
        - The precision of an added attribute is UsdGeom.XformOp.PrecisionFloat.

    Args:
        xformable:  The Xformable to modify.
        translate:      The translate vector
    Returns:
        The set or added XformOp, or ``False`` if the prim is not an Xformable.
    """
    prim = xformable.GetPrim()
    if not (prim.IsA(UsdGeom.Xformable)):
        logger.warning(f"{__name__}.set_or_add_translate_op: Provided prim is not an Xformable")
        return False
    xformOp = _get_or_create_xform_op(xformable, "xformOp:translate", UsdGeom.XformOp.TypeTranslate)
    if xformOp.Get() is None:
        xformOp.Set(Gf.Vec3f(translate))
    else:
        typeName = type(xformOp.Get())
        xformOp.Set(typeName(translate))
    return xformOp


def set_or_add_orient_op(
    xformable: UsdGeom.Xformable, orient: typing.Union[Gf.Quatf, Gf.Quatd, Gf.Quath]
) -> typing.Optional[UsdGeom.XformOp]:
    """
    Sets or adds the orient XformOp on the input Xformable to provided orient value.

    Note that:
        - The precision of an added attribute is UsdGeom.XformOp.PrecisionFloat.

    Args:
        xformable:  The Xformable to modify.
        orient:      The orient quaternion
    Returns:
        The set or added XformOp, or ``None`` if the prim is not an Xformable.
        Its scale and translate siblings answer ``False`` for that case rather
        than ``None``.
    """
    prim = xformable.GetPrim()
    if not (prim.IsA(UsdGeom.Xformable)):
        logger.warning(f"{__name__}.set_or_add_orient_op: Provided prim is not an Xformable")
        return None
    xformOp = _get_or_create_xform_op(xformable, "xformOp:orient", UsdGeom.XformOp.TypeOrient)
    if xformOp.Get() is None:
        xformOp.Set(Gf.Quatf(orient))
    else:
        typeName = type(xformOp.Get())
        xformOp.Set(typeName(orient))
    return xformOp


def set_or_add_scale_orient_translate(
    xformable: UsdGeom.Xformable,
    scale: typing.Union[Gf.Vec3f, Gf.Vec3d, Gf.Vec3h],
    orient: typing.Union[Gf.Quatf, Gf.Quatd, Gf.Quath],
    translate: typing.Union[Gf.Vec3f, Gf.Vec3d, Gf.Vec3h],
) -> typing.Union[typing.List[UsdGeom.XformOp], typing.Literal[False]]:
    """
    Sets or adds scale, orient, and translate XformOps of xformable.

    Note that:
        - The precision of created attributes is UsdGeom.XformOp.PrecisionFloat.

    Args:
        xformable:  The Xformable to modify.
        scale:      The scale vector
        orient:     The orientation quaternion
        translate:  The translation vector
    Returns:
        List of set and created xform ops that will be [translate, orient, scale],
        or ``False`` if the prim is not an Xformable.
    """
    prim = xformable.GetPrim()
    if not (prim.IsA(UsdGeom.Xformable)):
        logger.warning(f"{__name__}.set_or_add_scale_orient_translate: Provided prim is not an Xformable")
        return False
    tosOps = []
    tosOps.append(set_or_add_translate_op(xformable, translate))
    tosOps.append(set_or_add_orient_op(xformable, orient))
    tosOps.append(set_or_add_scale_op(xformable, scale))
    return tosOps


def setup_transform_as_scale_orient_translate(
    xformable: typing.Union[Usd.Prim, UsdGeom.Xformable],
):
    """
    Changes the local transform (ops) to the physics default scale->orient->translate stack.

    Note that:
        - Any skew in the transform will be lost.
        - A resetXformStack is preserved, but not the XformOps that are ignored due to the reset.
        - The transform attribute precision is set to UsdGeom.XformOp.PrecisionFloat.
        - Obsolete xformOp: namespace attributes are not removed (and cannot be for layers)

    Args:
        xformable: The prim or Xformable to modify.
    """
    prim = xformable.GetPrim()
    if not (prim.IsA(UsdGeom.Xformable)):
        logger.warning(f"{__name__}.setup_transform_as_scale_orient_translate: Provided prim is not an Xformable")
        return
    # Callers pass a bare prim as often as an Xformable, so re-wrap either way.
    xformable = UsdGeom.Xformable(xformable)
    hasReset = xformable.GetResetXformStack()
    tf = Gf.Transform(xformable.GetLocalTransformation())
    scale = Gf.Vec3d(tf.GetScale())
    translation = Gf.Vec3d(tf.GetTranslation())
    quat = Gf.Quatd(tf.GetRotation().GetQuat())
    newOps = set_or_add_scale_orient_translate(xformable, scale, quat, translation)
    xformable.SetXformOpOrder(newOps, hasReset)


def copy_transform_as_scale_orient_translate(
    src: typing.Union[Usd.Prim, UsdGeom.Xformable], dst: typing.Union[Usd.Prim, UsdGeom.Xformable]
):
    """
    Copies the local transforms from one Xformable to another as a default scale->orient->translate stack.

    Note that:
        - Any skew in the src transform will be lost.
        - A resetXformStack is preserved, but not the XformOps that are ignored due to the reset.
        - The transform attribute precision of added XformOps is set to UsdGeom.XformOp.PrecisionFloat.
        - Obsolete xformOp: namespace attributes in dst are not removed (and cannot be for layers)

    Args:
        src: The source prim or Xformable.
        dst: The destination prim or Xformable.
    """
    srcPrim = src.GetPrim()
    dstPrim = dst.GetPrim()
    if not (srcPrim.IsA(UsdGeom.Xformable) and dstPrim.IsA(UsdGeom.Xformable)):
        logger.warning(
            f"{__name__}.copy_transform_as_scale_orient_translate: "
            "Either the src or dst Xformable parameter is not an Xformable"
        )
        return
    srcXformable = UsdGeom.Xformable(src)
    dstXformable = UsdGeom.Xformable(dst)
    hasReset = srcXformable.GetResetXformStack()
    tf = Gf.Transform(srcXformable.GetLocalTransformation())
    scale = Gf.Vec3d(tf.GetScale())
    translation = Gf.Vec3d(tf.GetTranslation())
    quat = Gf.Quatd(tf.GetRotation().GetQuat())
    newOps = set_or_add_scale_orient_translate(dstXformable, scale, quat, translation)
    dstXformable.SetXformOpOrder(newOps, hasReset)
