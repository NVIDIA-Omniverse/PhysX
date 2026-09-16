# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-UTILS-001
# @covers AC-1 AC-2 AC-5 AC-12 AC-15

"""Collision filtering: collision groups and filtered pairs.

Two independent mechanisms decide which colliders are allowed to interact. A
``UsdPhysics.CollisionGroup`` holds a collection of colliders and a relationship
naming the groups it does not collide with, which is the scalable one. A
``UsdPhysics.FilteredPairsAPI`` names individual prims on the prim itself, which
is the one to reach for when two specific bodies must ignore each other.
"""

import typing

from pxr import Sdf, Usd, UsdPhysics

from .schema import remove_api_schema_properties

__all__ = [
    "add_collision_to_collision_group",
    "remove_collision_from_collision_group",
    "is_in_collision_group",
    "add_collision_group",
    "add_pair_filter",
    "remove_pair_filter",
]


def _get_collision_group_includes(stage: Usd.Stage, collision_group_path: typing.Union["str", Sdf.Path]):
    collisionGroup = stage.GetPrimAtPath(collision_group_path)
    if collisionGroup:
        collisionToken = "colliders"
        collectionAPI = Usd.CollectionAPI.Get(collisionGroup, collisionToken)
        if collectionAPI:
            return collectionAPI.GetIncludesRel()

    return None


def add_collision_to_collision_group(
    stage: Usd.Stage, collision_path: typing.Union["str", Sdf.Path], collision_group_path: typing.Union["str", Sdf.Path]
):
    """
    Add collision path to a collision group include rel.

    Args:
        stage:      The Usd.Stage to add path.
        collision_path:  Collision path to add.
        collision_group_path: Collision group prim path.
    """
    includesRel = _get_collision_group_includes(stage, collision_group_path)
    if includesRel:
        includesRel.AddTarget(collision_path)


def remove_collision_from_collision_group(
    stage: Usd.Stage, collision_path: typing.Union["str", Sdf.Path], collision_group_path: typing.Union["str", Sdf.Path]
):
    """
    Remove collision path to a collision group include rel.

    Args:
        stage:      The Usd.Stage to add path.
        collision_path:  Collision path to add.
        collision_group_path: Collision group prim path.
    """
    includesRel = _get_collision_group_includes(stage, collision_group_path)
    if includesRel:
        includesRel.RemoveTarget(collision_path)


def is_in_collision_group(
    stage: Usd.Stage, collision_path: typing.Union["str", Sdf.Path], collision_group_path: typing.Union["str", Sdf.Path]
) -> bool:
    """
    Checks if a collision path belongs to a collision group include rel.

    Args:
        stage:      The Usd.Stage to add path.
        collision_path:  Collision path to add.
        collision_group_path: Collision group prim path.
    """
    includesRel = _get_collision_group_includes(stage, collision_group_path)
    if includesRel:
        return collision_path in includesRel.GetTargets()

    return False


def add_collision_group(stage: Usd.Stage, path: typing.Union[str, Sdf.Path]):
    """
    Define a collision group with an empty filtered-groups relationship.

    Args:
        stage:      The Usd.Stage to add the group.
        path:       The desired collision group path.
    """
    collisionGroup = UsdPhysics.CollisionGroup.Define(stage, Sdf.Path(path))
    collisionGroup.CreateFilteredGroupsRel()


def add_pair_filter(stage: Usd.Stage, paths: typing.List[typing.Union[str, Sdf.Path]]):
    """
    Make every prim in paths filter collisions against every other one.

    Duplicate entries and a mix of ``str`` and ``Sdf.Path`` spellings of the same
    prim are tolerated. Paths are compared by value, so no prim is given a
    filter against itself.

    Args:
        stage:      The Usd.Stage holding the prims.
        paths:      The prim paths to filter against each other.
    """
    sdf_paths = [Sdf.Path(path) for path in paths]

    for path in sdf_paths:
        prim = stage.GetPrimAtPath(path)

        filteringPairsAPI = UsdPhysics.FilteredPairsAPI.Apply(prim)
        rel = filteringPairsAPI.CreateFilteredPairsRel()

        for otherPath in sdf_paths:
            if otherPath != path:
                rel.AddTarget(otherPath)


def remove_pair_filter(stage: Usd.Stage, paths: typing.List[typing.Union[str, Sdf.Path]]):
    """
    Undo add_pair_filter for the given paths.

    The UsdPhysics.FilteredPairsAPI is removed, and the physics:filteredPairs
    relationship with it, so a later add_pair_filter starts from no targets.

    Args:
        stage:      The Usd.Stage holding the prims.
        paths:      The prim paths to stop filtering.
    """
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        prim.RemoveAPI(UsdPhysics.FilteredPairsAPI)
        remove_api_schema_properties(UsdPhysics.FilteredPairsAPI, prim)
