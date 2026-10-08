# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-UTILS-001
# @covers AC-1 AC-2 AC-10

"""Constants shared by the authoring helpers.

Collected here rather than duplicated per module.

ovphysx ships the PhysX schemas *codeless*, so PhysX schema tokens are spelled
out here as string literals; refer to :mod:`ovphysx.utils.codeless`.
"""

from pxr import UsdPhysics

__all__ = [
    "MAX_FLOAT",
    "AXES_INDICES",
    "TOKEN_TRIANGLE_MESH",
    "TOKEN_SDF",
    "TOKEN_SPHERE_FILL",
    "SCENE_UPDATE_TYPE_SYNCHRONOUS",
    "SCENE_UPDATE_TYPE_ASYNCHRONOUS",
    "SCENE_UPDATE_TYPE_DISABLED",
    "COOKED_DATA_TOKENS",
    "MESH_APPROXIMATIONS",
    "METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES",
]

#: The largest finite value a USD ``float`` attribute can hold.
MAX_FLOAT = 3.40282347e38

#: Maps an axis name to its index in a 3-vector.
AXES_INDICES = {"X": 0, "Y": 1, "Z": 2}

# The three tokens below are PhysX-only, i.e. the ones stock UsdPhysics does not
# define. They must stay in sync with the libraryTokens block in
# schemas/physx/source/physxSchema/schema.usda. The remaining approximation
# tokens this module needs (none, convexHull, convexDecomposition,
# meshSimplification, boundingCube, boundingSphere) are all part of stock
# UsdPhysics and are read from UsdPhysics.Tokens below.

#: Mesh approximation token for an exact triangle mesh collider.
TOKEN_TRIANGLE_MESH = "triangleMesh"
#: Mesh approximation token for a signed-distance-field collider.
TOKEN_SDF = "sdf"
#: Mesh approximation token for a sphere-fill collider.
TOKEN_SPHERE_FILL = "sphereFill"

#: ``physxScene:updateType`` value: the scene steps in the update thread.
SCENE_UPDATE_TYPE_SYNCHRONOUS = "Synchronous"
#: ``physxScene:updateType`` value: the scene steps asynchronously.
SCENE_UPDATE_TYPE_ASYNCHRONOUS = "Asynchronous"
#: ``physxScene:updateType`` value: the scene does not step.
SCENE_UPDATE_TYPE_DISABLED = "Disabled"

#: The ``PhysxCookedDataAPI`` instance names, one per cooked representation.
#: Each is the instance name of a multiple-apply API, so its buffer lives at
#: ``physxCookedData:<token>:buffer``.
#:
#: The three values are ``"convexHull"``, ``"convexDecomposition"`` and
#: ``"triangleMesh"``. They are spelled out here because the first two are read
#: from ``UsdPhysics.Tokens``, which the documentation build mocks along with
#: the rest of ``pxr``: the rendered value of this list therefore shows a mock
#: placeholder in place of each of the two, beside ``triangleMesh`` as a real
#: literal. Do not read the rendered list as the values.
COOKED_DATA_TOKENS = [
    UsdPhysics.Tokens.convexHull,
    UsdPhysics.Tokens.convexDecomposition,
    TOKEN_TRIANGLE_MESH,
]

#: Maps a mesh approximation token to the *identifier* of the PhysX collision
#: API schema carrying that approximation's tuning parameters. A ``None`` value
#: means the approximation is fully described by the token and needs no extra
#: API. The values are schema identifier strings rather than typed classes
#: because the PhysX schemas are codeless; apply them with
#: :func:`ovphysx.utils.codeless.apply_api`.
#:
#: The eight entries are spelled out below, and the keys are the whole of what
#: :func:`ovphysx.utils.set_collider` accepts as its ``approximation_shape``:
#:
#: * ``"none"`` -- ``"PhysxTriangleMeshCollisionAPI"``
#: * ``"convexHull"`` -- ``"PhysxConvexHullCollisionAPI"``
#: * ``"convexDecomposition"`` -- ``"PhysxConvexDecompositionCollisionAPI"``
#: * ``"meshSimplification"`` -- ``"PhysxTriangleMeshSimplificationCollisionAPI"``
#: * ``"boundingCube"`` -- ``None``
#: * ``"boundingSphere"`` -- ``None``
#: * ``"sphereFill"`` -- ``"PhysxSphereFillCollisionAPI"``
#: * ``"sdf"`` -- ``"PhysxSDFMeshCollisionAPI"``
#:
#: Spelled out because six of the eight keys are read from
#: ``UsdPhysics.Tokens``, which the documentation build mocks along with the
#: rest of ``pxr``, so the rendered value of this dict shows a mock placeholder
#: in place of each of those six keys. Do not read the rendered mapping as the
#: mapping.
MESH_APPROXIMATIONS = {
    UsdPhysics.Tokens.none: "PhysxTriangleMeshCollisionAPI",
    UsdPhysics.Tokens.convexHull: "PhysxConvexHullCollisionAPI",
    UsdPhysics.Tokens.convexDecomposition: "PhysxConvexDecompositionCollisionAPI",
    UsdPhysics.Tokens.meshSimplification: "PhysxTriangleMeshSimplificationCollisionAPI",
    UsdPhysics.Tokens.boundingCube: None,
    UsdPhysics.Tokens.boundingSphere: None,
    TOKEN_SPHERE_FILL: "PhysxSphereFillCollisionAPI",
    TOKEN_SDF: "PhysxSDFMeshCollisionAPI",
}

# Must stay in sync with kLocalSpaceVelocitiesMetadataAttributeName in
# ovruntime/include/omni/physx/IPhysxSettings.h.

#: The ``customData`` dict key that makes the runtime read a rigid body's
#: authored velocities in the body frame instead of world space.
METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES = "physics:localSpaceVelocities"
