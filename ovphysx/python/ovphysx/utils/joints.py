# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-UTILS-001
# @covers AC-1 AC-2 AC-5 AC-10 AC-15

"""Joint creation between two prims, or between a prim and the world.

:func:`create_joint` computes the two local frames from the bodies' current world
poses, so the joint holds them where they already are; :func:`add_joint_fixed`
takes those frames from the caller instead. Passing one body anchors the joint
to the world.

The PhysX joint types are codeless, so they are defined by type name. They
derive from ``UsdPhysicsJoint``, which stock ``usd-core`` does bind, so they are
then reached through that typed base. Refer to :mod:`ovphysx.utils.codeless`.
"""

import typing

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from . import codeless
from ._deprecation import deprecated_alias
from .constants import MAX_FLOAT
from .paths import _create_unused_path, get_stage_next_free_path

__all__ = [
    "add_joint_fixed",
    "create_joint",
    "create_joints",
]


def add_joint_fixed(
    stage: Usd.Stage,
    joint_path: typing.Union["str", Sdf.Path],
    actor0: typing.Union["str", Sdf.Path],
    actor1: typing.Union["str", Sdf.Path],
    local_pos0: Gf.Vec3f,
    local_rot0: Gf.Quatf,
    local_pos1: Gf.Vec3f,
    local_rot1: Gf.Quatf,
    break_force: float,
    break_torque: float,
) -> UsdPhysics.FixedJoint:
    """
    Add fixed joint to the stage.

    Args:
        stage:      The Usd.Stage to add the joint.
        joint_path: The desired joint path.
        actor0:     The actor0 for the joint.
        actor1:     The actor1 for the joint.
        local_pos0: The joint local position offset from the actor0
        local_rot0: The joint local rotation offset from the actor0
        local_pos1: The joint local position offset from the actor1
        local_rot1: The joint local rotation offset from the actor1
        break_force: The joint break force.
        break_torque: The joint break torque.
    """
    joint_path = get_stage_next_free_path(stage, joint_path, True)
    d6FixedJoint = UsdPhysics.FixedJoint.Define(stage, joint_path)

    if actor0:
        d6FixedJoint.CreateBody0Rel().SetTargets([actor0])
    if actor1:
        d6FixedJoint.CreateBody1Rel().SetTargets([actor1])

    d6FixedJoint.CreateLocalPos0Attr().Set(local_pos0)
    d6FixedJoint.CreateLocalRot0Attr().Set(local_rot0)

    d6FixedJoint.CreateLocalPos1Attr().Set(local_pos1)
    d6FixedJoint.CreateLocalRot1Attr().Set(local_rot1)

    d6FixedJoint.CreateBreakForceAttr().Set(break_force)
    d6FixedJoint.CreateBreakTorqueAttr().Set(break_torque)
    return d6FixedJoint


def _define_codeless_joint(stage: Usd.Stage, joint_path, type_name: str) -> UsdPhysics.Joint:
    """Define a codeless PhysX joint prim and wrap it in its typed base schema.

    The PhysX joint types are codeless, so the prim is defined by type name.
    They derive from UsdPhysicsJoint, which stock usd-core has a binding for, so
    the shared joint properties (bodies, local frames, break force) stay
    reachable through the typed base. The concrete type is checked before the
    stage is changed.
    """
    codeless._require_concrete_prim_definition(type_name)
    prim = stage.DefinePrim(joint_path, type_name)
    if not prim:
        raise codeless.CodelessSchemaError(f"Could not define a {type_name} at {joint_path}.")
    joint = UsdPhysics.Joint(prim)
    if not joint:
        raise codeless.CodelessSchemaError(
            f"{joint_path} was defined as {type_name} but does not present as a "
            "UsdPhysics.Joint, so the PhysX schemas are most likely not "
            "registered. See ovphysx.utils.codeless.register_schemas()."
        )
    return joint


def create_joint(stage: Usd.Stage, joint_type: str, from_prim: Usd.Prim, to_prim: Usd.Prim) -> Usd.Prim:
    """
    Create a joint between two prims, with local frames computed from their world poses.

    The joint prim is placed under the first writable ancestor of to_prim, so
    that instanced and prototype prims do not break authoring. Passing only one
    prim anchors the joint to the world.

    Args:
        stage:      The Usd.Stage to add the joint.
        joint_type: One of "Fixed", "Revolute", "Prismatic", "Spherical",
                    "Distance", "Gear", "RackAndPinion"; anything else creates a
                    D6 joint with all axes locked.
        from_prim:  The body0 prim, or None.
        to_prim:    The body1 prim.
    """
    # A single prim is normalized into to_prim, so it becomes body1 either way.
    if to_prim is None:
        to_prim = from_prim
        from_prim = None

    from_path = from_prim.GetPath().pathString if from_prim is not None and from_prim.IsValid() else ""
    to_path = to_prim.GetPath().pathString if to_prim is not None and to_prim.IsValid() else ""
    single_selection = from_path == "" or to_path == ""

    # An instanced or prototype to_path is not writable, so walk up to one that is.
    joint_base_path = to_path
    base_prim = stage.GetPrimAtPath(joint_base_path)
    while base_prim != stage.GetPseudoRoot():
        if base_prim.IsInPrototype():
            base_prim = base_prim.GetParent()
        elif base_prim.IsInstanceProxy():
            base_prim = base_prim.GetParent()
        elif base_prim.IsInstanceable():
            base_prim = base_prim.GetParent()
        else:
            break
    joint_base_path = str(base_prim.GetPrimPath())
    if joint_base_path == "/":
        joint_base_path = ""

    joint_name = "/" + _create_unused_path(stage, joint_base_path, joint_type + "Joint")
    joint_path = joint_base_path + joint_name

    if joint_type == "Fixed":
        component = UsdPhysics.FixedJoint.Define(stage, joint_path)
    elif joint_type == "Revolute":
        component = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
        component.CreateAxisAttr("X")
    elif joint_type == "Prismatic":
        component = UsdPhysics.PrismaticJoint.Define(stage, joint_path)
        component.CreateAxisAttr("X")
    elif joint_type == "Spherical":
        component = UsdPhysics.SphericalJoint.Define(stage, joint_path)
        component.CreateAxisAttr("X")
    elif joint_type == "Distance":
        component = UsdPhysics.DistanceJoint.Define(stage, joint_path)
        component.CreateMinDistanceAttr(0.0)
        component.CreateMaxDistanceAttr(0.0)
    elif joint_type == "Gear":
        component = _define_codeless_joint(stage, joint_path, "PhysxPhysicsGearJoint")
    elif joint_type == "RackAndPinion":
        component = _define_codeless_joint(stage, joint_path, "PhysxPhysicsRackAndPinionJoint")
    else:
        component = UsdPhysics.Joint.Define(stage, joint_path)
        prim = component.GetPrim()
        for limit_name in ["transX", "transY", "transZ", "rotX", "rotY", "rotZ"]:
            limit_api = UsdPhysics.LimitAPI.Apply(prim, limit_name)
            limit_api.CreateLowAttr(1.0)
            limit_api.CreateHighAttr(-1.0)

    xfCache = UsdGeom.XformCache()

    if not single_selection:
        to_pose = xfCache.GetLocalToWorldTransform(to_prim)
        from_pose = xfCache.GetLocalToWorldTransform(from_prim)
        rel_pose = to_pose * from_pose.GetInverse()
        rel_pose = rel_pose.RemoveScaleShear()
        pos1 = Gf.Vec3f(rel_pose.ExtractTranslation())
        rot1 = Gf.Quatf(rel_pose.ExtractRotationQuat())

        component.CreateBody0Rel().SetTargets([Sdf.Path(from_path)])
        component.CreateBody1Rel().SetTargets([Sdf.Path(to_path)])
        component.CreateLocalPos0Attr().Set(pos1)
        component.CreateLocalRot0Attr().Set(rot1)
        component.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        component.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    else:
        to_pose = xfCache.GetLocalToWorldTransform(to_prim)
        to_pose = to_pose.RemoveScaleShear()
        pos1 = Gf.Vec3f(to_pose.ExtractTranslation())
        rot1 = Gf.Quatf(to_pose.ExtractRotationQuat())

        component.CreateBody1Rel().SetTargets([Sdf.Path(to_path)])
        component.CreateLocalPos0Attr().Set(pos1)
        component.CreateLocalRot0Attr().Set(rot1)
        component.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        component.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

    component.CreateBreakForceAttr().Set(MAX_FLOAT)
    component.CreateBreakTorqueAttr().Set(MAX_FLOAT)

    return stage.GetPrimAtPath(joint_base_path + joint_name)


def create_joints(
    stage: Usd.Stage, joint_type: str, paths: typing.List[typing.Union[str, Sdf.Path]], join_to_parent: bool = False
) -> typing.List[Usd.Prim]:
    """
    Create one joint per path, optionally anchoring each to its parent prim.

    Args:
        stage:      The Usd.Stage to add the joints.
        joint_type: The joint type, see create_joint.
        paths:      The body1 prim paths.
        join_to_parent: Whether to use each prim's parent as body0.
    """
    new_joints = []

    for path in paths:
        to_prim = stage.GetPrimAtPath(path)
        from_prim = None

        if join_to_parent and to_prim.IsValid():
            from_prim = to_prim.GetParent()
            if from_prim == from_prim.GetStage().GetPseudoRoot():
                from_prim = None

        new_joint = create_joint(stage, joint_type, from_prim, to_prim)

        if new_joint is not None and new_joint.IsValid():
            new_joints.append(new_joint)

    return new_joints


# Deprecated alias; see the note beside authoring.py's for why it is not in
# __all__ (AC-15).
createJoint = deprecated_alias(create_joint, "createJoint")
