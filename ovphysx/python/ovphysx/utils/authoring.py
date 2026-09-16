# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-UTILS-001
# @covers AC-1 AC-2 AC-5 AC-10 AC-11 AC-12 AC-15

"""Applying physics to prims that already exist.

Where :mod:`~ovphysx.utils.shapes` builds a body and its collider together, the
helpers here take a prim a stage already has and give it -- or take from it --
mass, a force, a physics scene, or a whole API set. The ``set*`` and ``remove*``
family is the core of that: it applies and strips the collider and rigid-body
API sets, and an Xformable is given colliders across its whole subtree and
stripped of them the same way.

The two halves are inverses over the API schemas, and only partly over the
properties authored under them. ``Usd.Prim.RemoveAPI`` and
:func:`~ovphysx.utils.codeless.remove_api` drop the ``apiSchemas`` entry alone,
so :func:`remove_physics` and the rigid-body helpers built on it leave every
property in place: a prim that has to come out clean needs
:func:`~ovphysx.utils.schema.remove_api_schema_properties` as well -- and
:func:`~ovphysx.utils.schema.create_api_schema_property_cache` first, if those
values have to survive the round trip. :func:`remove_collider` is the deliberate
exception, and the helpers that call it inherit what it does: it takes the mesh
approximation and cooked-data APIs' properties with the APIs, since those APIs
are mutually exclusive and their properties would otherwise orphan onto a prim
re-approximated another way. The collision APIs :func:`set_collider` re-applies
keep theirs, so a re-approximated collider keeps its contact tuning.

How schemas are reached here
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
PhysX APIs are applied by identifier and their properties authored by name,
because ovphysx ships the PhysX schemas codeless. Refer to
:mod:`ovphysx.utils.codeless`. Core ``UsdPhysics`` schemas still use their typed
bindings, which are part of stock ``usd-core``.
"""

import logging
import typing

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from . import codeless
from ._deprecation import deprecated_alias, deprecated_parameter
from .constants import (
    COOKED_DATA_TOKENS,
    MESH_APPROXIMATIONS,
    METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES,
    SCENE_UPDATE_TYPE_ASYNCHRONOUS,
    SCENE_UPDATE_TYPE_DISABLED,
    SCENE_UPDATE_TYPE_SYNCHRONOUS,
)
from .schema import (
    has_schema,
    remove_api_schema_properties,
    remove_multiple_api_schema_properties,
)

logger = logging.getLogger(__name__)

# Every approximation API set_collider can apply, read off the same table it
# applies them from so the two cannot drift. Some tokens map to no API at all,
# and two tokens can share one, hence the filter and the deduplication.
_APPROXIMATION_APIS = tuple(
    dict.fromkeys(api for api in MESH_APPROXIMATIONS.values() if api is not None)
)

# The property namespace PhysxCookedDataAPI's instances author under.
_COOKED_DATA_PREFIX = "physxCookedData"

__all__ = [
    "add_density",
    "add_mass",
    "add_force_torque",
    "set_physics",
    "set_collider",
    "set_collider_subtree",
    "set_rigid_body",
    "set_static_collider",
    "remove_physics",
    "remove_collider",
    "remove_collider_subtree",
    "remove_rigid_body",
    "remove_rigid_body_subtree",
    "remove_static_collider",
    "add_physics_scene",
    "set_physics_scene_asyncsimrender",
    "set_local_space_velocities",
    "clear_local_space_velocities",
]


def add_density(stage: Usd.Stage, path: typing.Union[str, Sdf.Path], value: float) -> UsdPhysics.MassAPI:
    """
    Add density to given prim. Note that his will apply MassAPI on the prim.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        value:      The desired density.
    """
    rbPrim = stage.GetPrimAtPath(Sdf.Path(path))
    densityAPI = UsdPhysics.MassAPI.Apply(rbPrim)
    densityAPI.CreateDensityAttr().Set(value)
    return densityAPI


def add_mass(stage: Usd.Stage, path: typing.Union["str", Sdf.Path], mass: float = 1.0) -> UsdPhysics.MassAPI:
    """
    Add mass to given prim. Note that his will apply MassAPI on the prim.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        mass:       The desired mass.
    """
    rbPrim = stage.GetPrimAtPath(Sdf.Path(path))
    massAPI = UsdPhysics.MassAPI.Apply(rbPrim)
    massAPI.CreateMassAttr().Set(mass)
    return massAPI


def add_force_torque(
    stage: Usd.Stage,
    path: typing.Union["str", Sdf.Path],
    force: Gf.Vec3f = Gf.Vec3f(0.0),
    torque: Gf.Vec3f = Gf.Vec3f(0.0),
    mode: str = "acceleration",
    is_enabled: bool = True,
    is_world_space: bool = False,
) -> Usd.Prim:
    """
    Add force/torque to given prim. Note that his will apply PhysxForceAPI on the prim.

    Args:
        stage:      The Usd.Stage to add path.
        path:       The desired path to create.
        force:      The desired force.
        torque:     The desired torque.
        mode:       The force/torque mode, "force" or "acceleration".
        is_enabled: Bool defining whether force is enabled or not.
        is_world_space: Bool defining whether force is applied in world space or body local space.

    Returns:
        The prim the API was applied to.
    """
    rbPrim = stage.GetPrimAtPath(Sdf.Path(path))
    codeless.apply_api(rbPrim, "PhysxForceAPI")
    codeless.set_attrs(
        rbPrim,
        {
            "physxForce:force": force,
            "physxForce:torque": torque,
            "physxForce:mode": mode,
            "physxForce:forceEnabled": is_enabled,
            "physxForce:worldFrameEnabled": is_world_space,
        },
    )
    return rbPrim


def set_physics(prim: Usd.Prim, kinematic: bool):
    """
    Apply the rigid body API set to a prim.

    If the core rigid-body API is already present, this function completes the
    API set, enables the body, and updates its kinematic state.

    Args:
        prim:       The prim to make a rigid body.
        kinematic:  Whether the body is kinematic.
    """
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        physics_api = UsdPhysics.RigidBodyAPI(prim)
    else:
        physics_api = UsdPhysics.RigidBodyAPI.Apply(prim)
    codeless.apply_api(prim, "PhysxRigidBodyAPI")
    physics_api.CreateRigidBodyEnabledAttr(True)
    physics_api.CreateKinematicEnabledAttr(kinematic)


@deprecated_parameter("approximationShape", "approximation_shape")
def set_collider(prim: Usd.Prim, approximation_shape=UsdPhysics.Tokens.none):
    """
    Apply the collision API set to a prim, choosing the mesh approximation APIs to match.

    A mesh that is part of a rigid body cannot use the triangle-mesh
    approximation, so a `none` request on such a mesh is upgraded to
    `convexHull` with a warning.

    A prim that is already a collider is left untouched with a warning.
    A prim whose ``omni:no_collision`` attribute is true is left untouched. An
    authored false value does not opt out of collision.

    Args:
        prim:       The prim to make a collider.
        approximation_shape: The mesh approximation token, one of
                    ``"none"``, ``"convexHull"``, ``"convexDecomposition"``,
                    ``"meshSimplification"``, ``"boundingCube"``,
                    ``"boundingSphere"``, ``"sphereFill"`` or ``"sdf"``.
                    The first six are the ``UsdPhysics.Tokens`` values; the
                    last two are PhysX-only and are declared here as
                    :data:`ovphysx.utils.TOKEN_SPHERE_FILL` and
                    :data:`ovphysx.utils.TOKEN_SDF`.
    """
    # Opt out by attribute rather than purpose=guide, so the volume stays renderable.
    no_collision = prim.GetAttribute("omni:no_collision")
    if no_collision and no_collision.Get():
        return

    if has_schema(prim, "PhysicsCollisionAPI"):
        logger.warning("PhysicsCollisionAPI is already defined")
        return

    def is_part_of_rigid_body(curr_prim):
        if curr_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return True

        curr_prim = curr_prim.GetParent()

        if not curr_prim.IsValid():
            return False

        return is_part_of_rigid_body(curr_prim)

    isMesh = prim.IsA(UsdGeom.Mesh)
    if isMesh and approximation_shape == UsdPhysics.Tokens.none and is_part_of_rigid_body(prim):
        logger.warning(
            f"set_collider: {prim.GetPath()} is a part of a rigid body. "
            "Resetting approximation shape from none (trimesh) to convexHull"
        )
        approximation_shape = UsdPhysics.Tokens.convexHull

    collisionAPI = UsdPhysics.CollisionAPI.Apply(prim)
    codeless.apply_api(prim, "PhysxCollisionAPI")
    collisionAPI.CreateCollisionEnabledAttr().Set(True)

    if isMesh or prim.IsInstanceable():
        api = MESH_APPROXIMATIONS.get(approximation_shape, 0)  # None is a valid value
        if api == 0:
            logger.warning(
                f"set_collider: invalid approximation type {approximation_shape} provided for "
                f"{prim.GetPath()}. Falling back to convexHull."
            )
            approximation_shape = UsdPhysics.Tokens.convexHull
            api = MESH_APPROXIMATIONS[approximation_shape]
        if api is not None:
            codeless.apply_api(prim, api)
        meshcollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(prim)
        meshcollisionAPI.CreateApproximationAttr().Set(approximation_shape)


def set_collider_subtree(prim: Usd.Prim, approximation_shape=UsdPhysics.Tokens.none):
    """
    Apply the collision API set to every gprim and instanceable prim in a subtree.

    Args:
        prim:       The subtree root.
        approximation_shape: The mesh approximation token, see set_collider.
    """
    pit = iter(Usd.PrimRange(prim))
    for p in pit:
        if p.GetMetadata("hide_in_stage_window"):
            pit.PruneChildren()
            continue
        if p.IsA(UsdGeom.Gprim) or p.IsInstanceable():
            set_collider(p, approximation_shape)


@deprecated_parameter("approximationShape", "approximation_shape")
def set_rigid_body(prim: Usd.Prim, approximation_shape, kinematic: bool):
    """
    Make a prim a rigid body and give it colliders.

    An Xformable gets colliders across its whole subtree, matching how a
    rigid-body hierarchy is normally authored; anything else is treated as a
    single collider.

    Args:
        prim:       The prim to make a rigid body.
        approximation_shape: The mesh approximation token, see set_collider.
        kinematic:  Whether the body is kinematic.
    """
    set_physics(prim, kinematic)

    if prim.IsA(UsdGeom.Xformable):
        set_collider_subtree(prim, approximation_shape)
    else:
        set_collider(prim, approximation_shape)


def set_static_collider(prim: Usd.Prim, approximation_shape=UsdPhysics.Tokens.none):
    """
    Give a subtree colliders without making it a rigid body.

    Args:
        prim:       The subtree root.
        approximation_shape: The mesh approximation token, see set_collider.
    """
    set_collider_subtree(prim, approximation_shape)


def remove_physics(prim: Usd.Prim):
    """
    Remove the rigid body API set from a prim.

    Args:
        prim:       The prim to strip.
    """
    ret = prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
    codeless.remove_api(prim, "PhysxRigidBodyAPI")

    if not ret:
        logger.error("Failed to remove a UsdPhysics.RigidBodyAPI from prim {}".format(prim.GetPrimPath().pathString))


def remove_collider(prim: Usd.Prim):
    """
    Remove the collision API set from a prim, including mesh approximation and cooked data.

    The mesh approximation APIs are stripped from instanceable prims as well as
    meshes, mirroring set_collider. Every approximation API set_collider can
    apply is removed, the PhysX-only ``sdf`` and ``sphereFill`` ones included.

    The approximation and cooked-data APIs' authored properties are removed with
    the APIs, so nothing is left orphaned.

    ``PhysicsCollisionAPI``, ``PhysxCollisionAPI`` and ``PhysicsMeshCollisionAPI``
    keep their properties -- ``physxCollision:contactOffset`` and
    ``physxCollision:restOffset`` above all -- because every
    :func:`set_collider` call re-applies those APIs, so a re-approximated
    collider keeps its contact tuning. A caller who wants those gone too calls
    :func:`~ovphysx.utils.schema.remove_api_schema_properties` for them, as the
    example in :func:`~ovphysx.utils.schema.create_api_schema_property_cache` does.

    Args:
        prim:       The prim to strip.
    """
    ret = prim.RemoveAPI(UsdPhysics.CollisionAPI)
    codeless.remove_api(prim, "PhysxCollisionAPI")
    if prim.IsA(UsdGeom.Mesh) or prim.IsInstanceable():
        prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
        for api in _APPROXIMATION_APIS:
            codeless.remove_api(prim, api)
            remove_api_schema_properties(api, prim)
    for token in COOKED_DATA_TOKENS:
        codeless.remove_api(prim, "PhysxCookedDataAPI", token)
        remove_multiple_api_schema_properties(
            "PhysxCookedDataAPI", prim, _COOKED_DATA_PREFIX, token
        )

    if not ret:
        logger.error("Failed to remove a UsdPhysics.CollisionAPI from prim {}".format(prim.GetPrimPath().pathString))


def remove_collider_subtree(prim: Usd.Prim):
    """
    Remove the collision API set from every gprim and instanceable prim in a subtree.

    Instanceable prims are visited as well as gprims, mirroring
    set_collider_subtree; an instanceable prim is not a gprim.

    Args:
        prim:       The subtree root.
    """
    primRange = Usd.PrimRange(prim)
    for p in primRange:
        if p.IsA(UsdGeom.Gprim) or p.IsInstanceable():
            remove_collider(p)


def remove_rigid_body(prim: Usd.Prim):
    """
    Undo set_rigid_body, following the same Xformable subtree rule.

    Args:
        prim:       The prim to strip.
    """
    if prim.IsA(UsdGeom.Xformable):
        remove_rigid_body_subtree(prim)
    else:
        remove_physics(prim)
        remove_collider(prim)


def remove_rigid_body_subtree(prim: Usd.Prim):
    """
    Remove the rigid body API from a prim and the collision APIs from its subtree.

    Args:
        prim:       The subtree root.
    """
    remove_physics(prim)
    remove_collider_subtree(prim)


def remove_static_collider(prim: Usd.Prim):
    """
    Undo set_static_collider.

    Args:
        prim:       The subtree root.
    """
    remove_collider_subtree(prim)


def add_physics_scene(stage: Usd.Stage, path: typing.Union[str, Sdf.Path]) -> typing.List:
    """
    Define a UsdPhysics.Scene, unless a prim already exists at path.

    Args:
        stage:      The Usd.Stage to add the scene.
        path:       The desired scene path.

    Returns:
        A list holding the created path, or an empty list if nothing was created.
    """
    if stage.GetPrimAtPath(path).IsValid():
        logger.warning("Prim at path %s is already defined" % path)
        return []
    UsdPhysics.Scene.Define(stage, path)
    return [path]


def set_physics_scene_asyncsimrender(scene_prim: Usd.Prim, val: bool = True):
    """
    Switch a physics scene between asynchronous and synchronous update, unless it is disabled.

    Args:
        scene_prim: The UsdPhysics.Scene prim.
        val:        True for asynchronous, False for synchronous.
    """
    codeless.apply_api(scene_prim, "PhysxSceneAPI")
    sceneUpdateType = codeless.get_attr(scene_prim, "physxScene:updateType").Get()
    if sceneUpdateType != SCENE_UPDATE_TYPE_DISABLED:
        codeless.set_attr(
            scene_prim,
            "physxScene:updateType",
            SCENE_UPDATE_TYPE_ASYNCHRONOUS if val else SCENE_UPDATE_TYPE_SYNCHRONOUS,
        )


def set_local_space_velocities(prim: Usd.Prim, value: bool):
    """
    Choose whether a rigid body's authored velocities are read in its own frame.

    With this enabled the velocity and angularVelocity attributes are
    interpreted in the body's local frame instead of world space.

    Args:
        prim:       The rigid body prim.
        value:      True to interpret velocities in body local space.
    """
    prim.SetCustomDataByKey(METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES, value)


def clear_local_space_velocities(prim: Usd.Prim):
    """
    Undo set_local_space_velocities, returning the prim to world-space velocities.

    Args:
        prim:       The rigid body prim.
    """
    prim.ClearCustomDataByKey(METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES)


# Deprecated camelCase aliases for five names in this module (AC-15). Absent
# from __all__, so they stay out of `import *` and out of the rendered docs;
# ovphysx.utils resolves them through its own alias table.
setCollider = deprecated_alias(set_collider, "setCollider")
setRigidBody = deprecated_alias(set_rigid_body, "setRigidBody")
removeCollider = deprecated_alias(remove_collider, "removeCollider")
removePhysics = deprecated_alias(remove_physics, "removePhysics")
removeRigidBodySubtree = deprecated_alias(remove_rigid_body_subtree, "removeRigidBodySubtree")
