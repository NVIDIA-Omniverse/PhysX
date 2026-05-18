# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from .builders import LocalSpaceVelocitiesWidgetBuilder, QuatEulerRotationBuilder, RelationshipWidgetBuilder, PrettyPrintTokenComboBuilder
from .widgets import MassAPIWidget, JointWidget, RevoluteJointWidget, PrismaticJointWidget, DistanceJointWidget, SphericalJointWidget
from .widgets import LimitWidget, DriveWidget, FixedJointWidget 
from. widgets import ExtendedColliderWidget
from .widgets import LocalSpaceVelocitiesWidget
from pxr import Usd, UsdGeom, UsdPhysics
import omni.physx.bindings._physx as pxb

property_builders = {
    "physics:body0": [RelationshipWidgetBuilder, [UsdGeom.Xformable], 1],
    "physics:body1": [RelationshipWidgetBuilder, [UsdGeom.Xformable], 1],
    "physics:localRot0": [QuatEulerRotationBuilder],
    "physics:localRot1": [QuatEulerRotationBuilder],
    "physics:principalAxes": [QuatEulerRotationBuilder],
    "physics:simulationOwner": [RelationshipWidgetBuilder, [UsdPhysics.Scene]],
    "physics:filteredGroups": [RelationshipWidgetBuilder, [UsdPhysics.CollisionGroup]],
    "physxContactReport:reportPairs": [RelationshipWidgetBuilder, [], 0, lambda prim: prim.HasAPI(UsdPhysics.CollisionAPI)],
    "physics:hinge0": [RelationshipWidgetBuilder, [UsdPhysics.RevoluteJoint], 1],
    "physics:hinge1": [RelationshipWidgetBuilder, [UsdPhysics.RevoluteJoint], 1],
    "physics:hinge": [RelationshipWidgetBuilder, [UsdPhysics.RevoluteJoint], 1],
    "physics:prismatic": [RelationshipWidgetBuilder, [UsdPhysics.PrismaticJoint], 1],
    "rigid": [RelationshipWidgetBuilder, [], 1, lambda prim: prim.HasAPI(UsdPhysics.RigidBodyAPI)],
    "physics:approximation": [PrettyPrintTokenComboBuilder, ["Triangle Mesh", "Convex Decomposition", "Convex Hull", "Bounding Sphere", "Bounding Cube", "Mesh Simplification"],
                              [('sdf', 'SDF Mesh'),('sphereFill', 'Sphere Approximation')]],
    "simulationOwner": [RelationshipWidgetBuilder, [UsdPhysics.Scene], 1],
    pxb.METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES: [LocalSpaceVelocitiesWidgetBuilder, pxb.METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES],
}

widgets = {
    UsdPhysics.MassAPI: MassAPIWidget,
    UsdPhysics.Joint: JointWidget,
    UsdPhysics.FixedJoint: FixedJointWidget,
    UsdPhysics.RevoluteJoint: RevoluteJointWidget,
    UsdPhysics.PrismaticJoint: PrismaticJointWidget,
    UsdPhysics.DistanceJoint: DistanceJointWidget,
    UsdPhysics.SphericalJoint: SphericalJointWidget,
    UsdPhysics.LimitAPI: LimitWidget,
    UsdPhysics.DriveAPI: DriveWidget,
    UsdPhysics.CollisionAPI: ExtendedColliderWidget,
    UsdPhysics.RigidBodyAPI: LocalSpaceVelocitiesWidget,
}

property_order = {
    UsdPhysics.Scene: ['physics:gravityDirection', 'physics:gravityMagnitude'],
    UsdPhysics.RigidBodyAPI: ['physics:rigidBodyEnabled', 'physics:kinematicEnabled', 'physics:simulationOwner', 'physics:startsAsleep', pxb.METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES, 'physics:velocity', 'physics:angularVelocity'],
    UsdPhysics.MassAPI: ['physics:mass', 'physics:density', 'physics:centerOfMass', 'physics:diagonalInertia', 'physics:principalAxes'],
    UsdPhysics.CollisionAPI: ['physics:collisionEnabled', 'physics:simulationOwner'],
    UsdPhysics.MeshCollisionAPI: ['physics:approximation'],
    UsdPhysics.MaterialAPI: ['physics:dynamicFriction', 'physics:staticFriction', 'physics:restitution', 'physics:density'],
    UsdPhysics.CollisionGroup: ['physics:filteredGroups'],
    UsdPhysics.FilteredPairsAPI: ['physics:filteredPairs'],
    UsdPhysics.Joint: ['physics:jointEnabled', 'physics:body0', 'physics:body1', 'physics:localPos0', 'physics:localRot0', 'physics:localPos1', 'physics:localRot1', 'physics:collisionEnabled', 'physics:excludeFromArticulation', 'physics:breakForce', 'physics:breakTorque'],
    UsdPhysics.RevoluteJoint: ['physics:jointEnabled', 'physics:axis', 'physics:lowerLimit', 'physics:upperLimit'],
    UsdPhysics.PrismaticJoint: ['physics:jointEnabled', 'physics:axis', 'physics:lowerLimit', 'physics:upperLimit'],
    UsdPhysics.SphericalJoint: ['physics:jointEnabled', 'physics:axis', 'physics:coneAngle0Limit', 'physics:coneAngle1Limit'],
    UsdPhysics.DistanceJoint: ['physics:jointEnabled', 'physics:minDistance', 'physics:maxDistance'],
    UsdPhysics.FixedJoint: ['physics:jointEnabled'],
    UsdPhysics.LimitAPI: ['physics:low', 'physics:high'],
    UsdPhysics.DriveAPI: ['physics:type', 'physics:maxForce', 'physics:targetPosition', 'physics:targetVelocity', 'physics:damping', 'physics:stiffness'],
    UsdGeom.Plane: ['axis'],
    Usd.CollectionAPI: ["includes", "excludes", "expansionRule", "includeRoot"]
}

extensions = {
}

internal_extensions = {
    UsdPhysics.CollisionAPI: [UsdPhysics.MeshCollisionAPI],
}

extras = {
    UsdPhysics.Scene: [Usd.CollectionAPI],
    UsdPhysics.CollisionGroup: [Usd.CollectionAPI],
}

ignore = {
}
