# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

ignore = {
    "OmniPhysicsBaseMaterialAPI",
    "OmniPhysicsBodyAPI",
    "OmniPhysicsCurvesDeformableMaterialAPI",
    "OmniPhysicsCurvesDeformableSimAPI",
    "OmniPhysicsAttachment",
    "OmniPhysicsTetXformAttachment",
    "OmniPhysicsVtxCrvAttachment",
    "OmniPhysicsElementCollisionFilter",
}

extensions = {}
internal_extensions = {}
extras = {}

from ..widgets import (
    ExtendedSurfaceDeformableMaterialWidget, ExtendedAttachmentElementFilterWidget, ExtendedDeformableBodyWidget
)

widgets = {
    "OmniPhysicsDeformableBodyAPI": ExtendedDeformableBodyWidget,
    "OmniPhysicsSurfaceDeformableMaterialAPI": ExtendedSurfaceDeformableMaterialWidget,
    "OmniPhysicsVtxVtxAttachment": ExtendedAttachmentElementFilterWidget,
    "OmniPhysicsVtxTriAttachment": ExtendedAttachmentElementFilterWidget,
    "OmniPhysicsVtxTetAttachment": ExtendedAttachmentElementFilterWidget,
    "OmniPhysicsVtxXformAttachment": ExtendedAttachmentElementFilterWidget,
    "OmniPhysicsTriTriAttachment": ExtendedAttachmentElementFilterWidget,
    "OmniPhysicsElementCollisionFilter": ExtendedAttachmentElementFilterWidget,
}

property_order = {
    "OmniPhysicsDeformableBodyAPI": ['omniphysics:deformableBodyEnabled', 'omniphysics:kinematicEnabled', 'omniphysics:simulationOwner', 'omniphysics:startsAsleep', 'omniphysics:mass'],
    "OmniPhysicsDeformableMaterialAPI": ['omniphysics:density', 'omniphysics:dynamicFriction', 'omniphysics:staticFriction', 'omniphysics:youngsModulus', 'omniphysics:poissonsRatio'],
    "OmniPhysicsSurfaceDeformableMaterialAPI": ['omniphysics:surfaceThickness', 'omniphysics:surfaceStretchStiffness', 'omniphysics:surfaceShearStiffness', 'omniphysics:surfaceBendStiffness'],
}

property_builders = {}
