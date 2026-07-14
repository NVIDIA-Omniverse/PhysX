# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from pxr import UsdPhysics

ignore = {}

extensions = {
    UsdPhysics.DriveAPI: ["PhysxDrivePerformanceEnvelopeAPI"],
}

internal_extensions = {
    "OmniPhysicsDeformableBodyAPI": ["PhysxBaseDeformableBodyAPI", "PhysxSurfaceDeformableBodyAPI"],
    "OmniPhysicsDeformableMaterialAPI": ["PhysxDeformableMaterialAPI"],
    "OmniPhysicsSurfaceDeformableMaterialAPI": ["PhysxSurfaceDeformableMaterialAPI"],
}

extras = {}

from ..widgets import JointAxisWidget, ExtendedAutoDeformableAttachmentWidget

widgets = {
    "PhysxJointAxisAPI": JointAxisWidget,
    "PhysxAutoDeformableAttachmentAPI": ExtendedAutoDeformableAttachmentWidget,
}

property_order = {
    'PhysxDrivePerformanceEnvelopeAPI': ["maxActuatorVelocity", "speedEffortGradient", "velocityDependentResistance"], 
    'PhysxJointAxisAPI': ["maxJointVelocity", "armature", "staticFrictionEffort", "dynamicFrictionEffort", "viscousFrictionCoefficient"],
    #~DEPRECATED
    "PhysxBaseDeformableBodyAPI": ['physxDeformableBody:linearDamping', 'physxDeformableBody:maxLinearVelocity', 'physxDeformableBody:sleepThreshold', 'physxDeformableBody:selfCollision', 'physxDeformableBody:enableSpeculativeCCD', 'physxDeformableBody:disableGravity', 'physxDeformableBody:settlingThreshold', 'physxDeformableBody:settlingDamping', 'physxDeformableBody:maxDepenetrationVelocity', 'physxDeformableBody:solverPositionIterationCount', 'physxDeformableBody:selfCollisionFilterDistance'],
    "PhysxAutoDeformableAttachmentAPI": ['physxAutoDeformableAttachment:attachable0','physxAutoDeformableAttachment:attachable1','physxAutoDeformableAttachment:enableDeformableVertexAttachments', 'physxAutoDeformableAttachment:deformableVertexOverlapOffset', 'physxAutoDeformableAttachment:enableRigidSurfaceAttachments', 'physxAutoDeformableAttachment:rigidSurfaceSamplingDistance', 'physxAutoDeformableAttachment:enableCollisionFiltering', 'physxAutoDeformableAttachment:collisionFilteringOffset', 'physxAutoDeformableAttachment:maskShapes'],
    "PhysxDeformableMaterialAPI" : ['physxDeformableMaterial:elasticityDamping'],
    "PhysxSurfaceDeformableMaterialAPI" : ['physxDeformableMaterial:bendDamping'],
}

property_builders = {}
