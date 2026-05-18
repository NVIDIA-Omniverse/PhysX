# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema

ignore = {
    PhysxSchema.PhysxCameraAPI,
    PhysxSchema.PhysxCameraFollowAPI,
    PhysxSchema.PhysxCookedDataAPI,
    PhysxSchema.PhysxDeformableAPI,
    PhysxSchema.PhysxParticleAPI,
    PhysxSchema.PhysxPhysicsInstancer,
    PhysxSchema.PhysxPhysicsJointInstancer,
    PhysxSchema.PhysxVehicleTireFrictionTable,
}

extensions = {
    UsdPhysics.Scene: [PhysxSchema.PhysxSceneAPI],
    UsdPhysics.RigidBodyAPI: [PhysxSchema.PhysxRigidBodyAPI],
    UsdPhysics.MaterialAPI: [PhysxSchema.PhysxMaterialAPI],
    UsdPhysics.Joint: [PhysxSchema.PhysxJointAPI],
    UsdPhysics.DistanceJoint: [PhysxSchema.PhysxPhysicsDistanceJointAPI],
    UsdPhysics.LimitAPI: [PhysxSchema.PhysxLimitAPI],
    UsdPhysics.ArticulationRootAPI: [PhysxSchema.PhysxArticulationAPI],
}

internal_extensions = {
    # DEPRECATED
    PhysxSchema.PhysxDeformableBodyAPI: [PhysxSchema.PhysxCollisionAPI], 
    PhysxSchema.PhysxDeformableSurfaceAPI: [PhysxSchema.PhysxCollisionAPI],
    # FIXME: this is still integrated into a collision API widget
    # the active/inactive logic of the widget would otherwise fail
    UsdPhysics.CollisionAPI: [
        PhysxSchema.PhysxCollisionAPI,
        PhysxSchema.PhysxConvexHullCollisionAPI,
        PhysxSchema.PhysxConvexDecompositionCollisionAPI,
        PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI,
        PhysxSchema.PhysxTriangleMeshCollisionAPI,
        PhysxSchema.PhysxSDFMeshCollisionAPI,
        PhysxSchema.PhysxSphereFillCollisionAPI,
    ],
}

extras = {
    PhysxSchema.PhysxMeshMergeCollisionAPI: [Usd.CollectionAPI],
    PhysxSchema.PhysxSceneQuasistaticAPI: [Usd.CollectionAPI],    
}


from ..widgets import (
    ExtendedDeformableBodyWidgetDeprecated, ExtendedDeformableSurfaceWidgetDeprecated, ExtendedTetrahedralMeshWidgetDeprecated,
    ExtendedDeformalbeSurfaceMaterialWidgetDeprecated, ExtendedAttachmentWidgetDeprecated, ExtendedComputeAutoAttachmentWidgetDeprecated,
    ExtendedParticleClothWidgetDeprecated, ExtendedAutoParticleClothWidgetDeprecated,
    ExtendedVehicleContextWidget, ExtendedVehicleWidget, ExtendedVehicleWheelWidget, ExtendedVehicleSuspensionWidget,
    ExtendedVehicleTireWidget, ExtendedVehicleWheelAttachmentWidget, ExtendedVehicleDriveStandardWidget, ExtendedVehicleMultiWheelDifferentialWidget,
    ExtendedVehicleControllerBaseWidget, ExtendedVehicleControllerWidget, ExtendedMimicJointWidget, ExtendedParticleSystemWidget,
    JointStateWidget, FixedTendonWidget, SpatialTendonWidget, GearJointWidget, RackAndPinionJoint,
    ExtendedPhysxSceneWidget, ExtendedPhysxMaterialWidget,
)

widgets = {
    PhysxSchema.PhysxSceneAPI: ExtendedPhysxSceneWidget,
    PhysxSchema.PhysxMaterialAPI: ExtendedPhysxMaterialWidget,
    PhysxSchema.PhysxPhysicsGearJoint: GearJointWidget,
    PhysxSchema.PhysxPhysicsRackAndPinionJoint: RackAndPinionJoint,
    PhysxSchema.JointStateAPI: JointStateWidget,
    PhysxSchema.PhysxTendonAxisAPI: FixedTendonWidget,
    PhysxSchema.PhysxTendonAxisRootAPI: FixedTendonWidget,
    PhysxSchema.PhysxTendonAttachmentRootAPI: SpatialTendonWidget,
    PhysxSchema.PhysxTendonAttachmentAPI: SpatialTendonWidget,
    PhysxSchema.PhysxTendonAttachmentLeafAPI: SpatialTendonWidget,
    # DEPRECATED
    PhysxSchema.PhysxDeformableBodyAPI: ExtendedDeformableBodyWidgetDeprecated,
    PhysxSchema.PhysxDeformableSurfaceAPI: ExtendedDeformableSurfaceWidgetDeprecated,
    PhysxSchema.PhysxDeformableSurfaceMaterialAPI: ExtendedDeformalbeSurfaceMaterialWidgetDeprecated,
    PhysxSchema.PhysxPhysicsAttachment: ExtendedAttachmentWidgetDeprecated,
    PhysxSchema.PhysxAutoAttachmentAPI: ExtendedComputeAutoAttachmentWidgetDeprecated,
    PhysxSchema.TetrahedralMesh: ExtendedTetrahedralMeshWidgetDeprecated,
    #~DEPRECATED
    PhysxSchema.PhysxVehicleContextAPI: ExtendedVehicleContextWidget,
    PhysxSchema.PhysxVehicleAPI: ExtendedVehicleWidget,
    PhysxSchema.PhysxVehicleWheelAPI: ExtendedVehicleWheelWidget,
    PhysxSchema.PhysxVehicleSuspensionAPI: ExtendedVehicleSuspensionWidget,
    PhysxSchema.PhysxVehicleTireAPI: ExtendedVehicleTireWidget,
    PhysxSchema.PhysxVehicleWheelAttachmentAPI: ExtendedVehicleWheelAttachmentWidget,
    PhysxSchema.PhysxVehicleDriveStandardAPI: ExtendedVehicleDriveStandardWidget,
    PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI: ExtendedVehicleMultiWheelDifferentialWidget,
    PhysxSchema.PhysxVehicleControllerAPI: ExtendedVehicleControllerWidget,
    PhysxSchema.PhysxVehicleTankControllerAPI: ExtendedVehicleControllerBaseWidget,
    # DEPRECATED
    PhysxSchema.PhysxParticleClothAPI: ExtendedParticleClothWidgetDeprecated,
    PhysxSchema.PhysxAutoParticleClothAPI: ExtendedAutoParticleClothWidgetDeprecated,
    #~DEPRECATED
    PhysxSchema.PhysxParticleSystem: ExtendedParticleSystemWidget,
    PhysxSchema.PhysxMimicJointAPI: ExtendedMimicJointWidget,
}

from omni.kit.property.physics.builders import (
   BitfieldWidgetBuilder,
   QuatEulerRotationBuilder,
   RelationshipWidgetBuilder,
   ModelWithWidgetBuilder,
   ReferenceFrameIsCenterOfMassWidgetBuilder,
   HideWidgetBuilder,
   PrettyPrintTokenComboBuilder,
)

property_builders = {
    "physxRigidBody:lockedPosAxis": [BitfieldWidgetBuilder, ["X", "Y", "Z"]],
    "physxRigidBody:lockedRotAxis": [BitfieldWidgetBuilder, ["X", "Y", "Z"]],
    "groundMaterials": [RelationshipWidgetBuilder, [], 0, lambda prim: prim.HasAPI(PhysxSchema.PhysxMaterialAPI)],
    "physxVehicleContext:verticalAxis": [PrettyPrintTokenComboBuilder, ["X+", "X-", "Y+", "Y-", "Z+", "Z-", "undefined"]],
    "physxVehicleContext:longitudinalAxis": [PrettyPrintTokenComboBuilder, ["X+", "X-", "Y+", "Y-", "Z+", "Z-", "undefined"]],
    "physxVehicleTire:frictionTable": [RelationshipWidgetBuilder, [], 1, lambda prim: prim.IsA(PhysxSchema.PhysxVehicleTireFrictionTable)],
    "physxVehicleWheelAttachment:wheel": [RelationshipWidgetBuilder, [], 1, lambda prim: prim.HasAPI(PhysxSchema.PhysxVehicleWheelAPI)],
    "physxVehicleWheelAttachment:tire": [RelationshipWidgetBuilder, [], 1, lambda prim: prim.HasAPI(PhysxSchema.PhysxVehicleTireAPI)],
    "physxVehicleWheelAttachment:suspension": [RelationshipWidgetBuilder, [], 1, lambda prim: prim.HasAPI(PhysxSchema.PhysxVehicleSuspensionAPI)],
    "physxVehicleWheelAttachment:suspensionFrameOrientation": [QuatEulerRotationBuilder],
    "physxVehicleWheelAttachment:wheelFrameOrientation": [QuatEulerRotationBuilder],
    "physxVehicleWheelAttachment:collisionGroup": [RelationshipWidgetBuilder, [UsdPhysics.CollisionGroup], 1],
    "physxVehicleDriveStandard:engine": [RelationshipWidgetBuilder, [], 1, lambda prim: prim.HasAPI(PhysxSchema.PhysxVehicleEngineAPI)],
    "physxVehicleDriveStandard:gears": [RelationshipWidgetBuilder, [], 1, lambda prim: prim.HasAPI(PhysxSchema.PhysxVehicleGearsAPI)],
    "physxVehicleDriveStandard:autoGearBox": [RelationshipWidgetBuilder, [], 1, lambda prim: prim.HasAPI(PhysxSchema.PhysxVehicleAutoGearBoxAPI)],
    "physxVehicleDriveStandard:clutch": [RelationshipWidgetBuilder, [], 1, lambda prim: prim.HasAPI(PhysxSchema.PhysxVehicleClutchAPI)],
    "physxVehicle:drive": [RelationshipWidgetBuilder, [], 1, lambda prim: prim.HasAPI(PhysxSchema.PhysxVehicleDriveBasicAPI) or prim.HasAPI(PhysxSchema.PhysxVehicleDriveStandardAPI)],
    PhysxSchema.Tokens.referenceFrameIsCenterOfMass: [ReferenceFrameIsCenterOfMassWidgetBuilder, PhysxSchema.Tokens.referenceFrameIsCenterOfMass],
    "physxPhysics:simulationOwner": [RelationshipWidgetBuilder, [UsdPhysics.Scene, PhysxSchema.PhysxParticleSystem], 1],
    # DEPRECATED
    "deformable": [RelationshipWidgetBuilder, [], 1, lambda prim: prim.HasAPI(PhysxSchema.PhysxParticleClothAPI)],
    #~DEPRECATED
    "physxScene:broadphaseType": [ModelWithWidgetBuilder],
    "physxScene:collisionSystem": [ModelWithWidgetBuilder],
    # DEPRECATED
    "physxDeformable:kinematicEnabled": [ModelWithWidgetBuilder],
    "physxDeformable:simulationHexahedralResolution": [ModelWithWidgetBuilder],
    "physxDeformable:collisionSimplificationRemeshing": [ModelWithWidgetBuilder],
    "physxDeformable:collisionSimplificationRemeshingResolution": [ModelWithWidgetBuilder],
    "physxDeformable:collisionSimplificationTargetTriangleCount": [ModelWithWidgetBuilder],
    "physxDeformable:collisionSimplificationForceConforming": [ModelWithWidgetBuilder],
    "physxAutoAttachment:enableDeformableVertexAttachments": [ModelWithWidgetBuilder],
    "physxAutoAttachment:deformableVertexOverlapOffset": [ModelWithWidgetBuilder],
    "physxAutoAttachment:enableRigidSurfaceAttachments": [ModelWithWidgetBuilder],
    "physxAutoAttachment:rigidSurfaceSamplingDistance": [ModelWithWidgetBuilder],
    "physxAutoAttachment:enableCollisionFiltering": [ModelWithWidgetBuilder],
    "physxAutoAttachment:collisionFilteringOffset": [ModelWithWidgetBuilder],
    "physxAutoAttachment:maskShapes": [RelationshipWidgetBuilder, [], 0, lambda prim: prim.IsA(UsdGeom.Sphere) or prim.IsA(UsdGeom.Cube) or prim.IsA(UsdGeom.Capsule)],
    #~DEPRECATED
    "physxAutoDeformableAttachment:enableDeformableVertexAttachments": [ModelWithWidgetBuilder],
    "physxAutoDeformableAttachment:deformableVertexOverlapOffset": [ModelWithWidgetBuilder],
    "physxAutoDeformableAttachment:enableRigidSurfaceAttachments": [ModelWithWidgetBuilder],
    "physxAutoDeformableAttachment:rigidSurfaceSamplingDistance": [ModelWithWidgetBuilder],
    "physxAutoDeformableAttachment:enableCollisionFiltering": [ModelWithWidgetBuilder],
    "physxAutoDeformableAttachment:collisionFilteringOffset": [ModelWithWidgetBuilder],
    "physxAutoDeformableAttachment:maskShapes": [RelationshipWidgetBuilder, [], 0, lambda prim: prim.IsA(UsdGeom.Sphere) or prim.IsA(UsdGeom.Cube) or prim.IsA(UsdGeom.Capsule)],
    "physxMaterial:compliantContactDamping": [ModelWithWidgetBuilder],
    "physxMimicJoint:rotX:referenceJoint": [RelationshipWidgetBuilder, [], 1, lambda prim: prim.IsA(UsdPhysics.Joint)],
    "physxMimicJoint:rotY:referenceJoint": [RelationshipWidgetBuilder, [], 1, lambda prim: prim.IsA(UsdPhysics.Joint)],
    "physxMimicJoint:rotZ:referenceJoint": [RelationshipWidgetBuilder, [], 1, lambda prim: prim.IsA(UsdPhysics.Joint)],
    "physxMimicJoint:rotX:referenceJointAxis": [ModelWithWidgetBuilder],
    "physxMimicJoint:rotY:referenceJointAxis": [ModelWithWidgetBuilder],
    "physxMimicJoint:rotZ:referenceJointAxis": [ModelWithWidgetBuilder],
    "actor0": [RelationshipWidgetBuilder, [UsdGeom.Xformable], 1],
    "actor1": [RelationshipWidgetBuilder, [UsdGeom.Xformable], 1],
    "parentLink": [RelationshipWidgetBuilder, [], 1, lambda prim: prim.HasAPI(PhysxSchema.PhysxTendonAttachmentAPI) or prim.HasAPI(PhysxSchema.PhysxTendonAttachmentRootAPI)],
    "state:angular:physics:position": [ModelWithWidgetBuilder],
    "state:angular:physics:velocity": [ModelWithWidgetBuilder],
    "state:linear:physics:position": [ModelWithWidgetBuilder],
    "state:linear:physics:velocity": [ModelWithWidgetBuilder],
    "state:rotX:physics:position": [ModelWithWidgetBuilder],
    "state:rotX:physics:velocity": [ModelWithWidgetBuilder],
    "state:rotY:physics:position": [ModelWithWidgetBuilder],
    "state:rotY:physics:velocity": [ModelWithWidgetBuilder],
    "state:rotZ:physics:position": [ModelWithWidgetBuilder],
    "state:rotZ:physics:velocity": [ModelWithWidgetBuilder],
    "state:transX:physics:position": [ModelWithWidgetBuilder],
    "state:transX:physics:velocity": [ModelWithWidgetBuilder],
    "state:transY:physics:position": [ModelWithWidgetBuilder],
    "state:transY:physics:velocity": [ModelWithWidgetBuilder],
    "state:transZ:physics:position": [ModelWithWidgetBuilder],
    "state:transZ:physics:velocity": [ModelWithWidgetBuilder],
    "jointAxis": [HideWidgetBuilder],
}

vehicleControllerAPIPropOrder = ['physxVehicleController:accelerator', 'physxVehicleController:brake0', 'physxVehicleController:brake1', 'physxVehicleController:steer', 'physxVehicleController:targetGear']
vehicleMultiWheelDifferentialAPIPropOrder = ['physxVehicleMultiWheelDifferential:wheels', 'physxVehicleMultiWheelDifferential:torqueRatios', 'physxVehicleMultiWheelDifferential:averageWheelSpeedRatios']

property_order = {
    PhysxSchema.TetrahedralMesh: ['indices'], # DEPRECATED
    PhysxSchema.PhysxSceneAPI: ['physxScene:bounceThreshold', 'physxScene:frictionOffsetThreshold', 'physxScene:collisionSystem', 'physxScene:solverType', 'physxScene:broadphaseType', 'physxScene:frictionType', 'physxScene:enableCCD', 'physxScene:enableStabilization', 'physxScene:enableGPUDynamics', 'physxScene:enableEnhancedDeterminism', 'physxScene:enableExternalForcesEveryIteration', 'physxScene:solveArticulationContactLast','physxScene:gpuTempBufferCapacity', 'physxScene:gpuMaxRigidContactCount', 'physxScene:gpuMaxRigidPatchCount', 'physxScene:gpuHeapCapacity', 'physxScene:gpuFoundLostPairsCapacity', 'physxScene:gpuFoundLostAggregatePairsCapacity', 'physxScene:gpuTotalAggregatePairsCapacity', 'physxScene:gpuMaxSoftBodyContacts', 'physxScene:gpuMaxFEMClothContacts', 'physxScene:gpuMaxParticleContacts', 'physxScene:gpuMaxNumPartitions', 'physxScene:invertCollisionGroupFilter'],
    PhysxSchema.PhysxRigidBodyAPI: ['physxRigidBody:linearDamping', 'physxRigidBody:angularDamping', 'physxRigidBody:maxLinearVelocity', 'physxRigidBody:maxAngularVelocity', 'physxRigidBody:sleepThreshold', 'physxRigidBody:stabilizationThreshold', 'physxRigidBody:maxDepenetrationVelocity', 'physxRigidBody:maxContactImpulse', 'physxRigidBody:solverPositionIterationCount', 'physxRigidBody:solverVelocityIterationCount', 'physxRigidBody:enableCCD', 'physxRigidBody:enableSpeculativeCCD', 'physxRigidBody:disableGravity', 'physxRigidBody:lockedPosAxis', 'physxRigidBody:lockedRotAxis'],
    PhysxSchema.PhysxContactReportAPI: ['physxContactReport:threshold', 'physxContactReport:reportPairs'],
    PhysxSchema.PhysxCollisionAPI: ['physxCollision:contactOffset', 'physxCollision:restOffset', 'physxCollision:torsionalPatchRadius', 'physxCollision:minTorsionalPatchRadius'],
    PhysxSchema.PhysxMaterialAPI: ['physxMaterial:frictionCombineMode', 'physxMaterial:restitutionCombineMode', 'physxMaterial:dampingCombineMode', 'physxMaterial:compliantContactAccelerationSpring', 'physxMaterial:compliantContactStiffness', 'physxMaterial:compliantContactDamping'],
    PhysxSchema.PhysxPhysicsGearJoint: ['physics:jointEnabled', 'physics:hinge0', 'physics:hinge1', 'physics:gearRatio'],
    PhysxSchema.PhysxPhysicsRackAndPinionJoint: ['physics:jointEnabled', 'physics:hinge', 'physics:prismatic', 'physics:ratio'],
    PhysxSchema.PhysxJointAPI: ['physxJoint:jointFriction', 'physxJoint:maxJointVelocity'],
    PhysxSchema.PhysxPhysicsDistanceJointAPI: ['physxPhysicsDistanceJoint:springEnabled', 'physxPhysicsDistanceJoint:springStiffness', 'physxPhysicsDistanceJoint:springDamping'],
    PhysxSchema.PhysxLimitAPI: ['restitution', 'bounceThreshold', 'stiffness', 'damping'],
    PhysxSchema.PhysxArticulationAPI: ['physxArticulation:articulationEnabled', 'physxArticulation:solverPositionIterationCount', 'physxArticulation:solverVelocityIterationCount', 'physxArticulation:sleepThreshold', 'physxArticulation:stabilizationThreshold', 'physxArticulation:enabledSelfCollisions'],
    PhysxSchema.PhysxCharacterControllerAPI: ['physxCharacterController:moveTarget', 'physxCharacterController:slopeLimit', 'physxCharacterController:upAxis', 'physxCharacterController:nonWalkableMode', 'physxCharacterController:climbingMode', 'physxCharacterController:invisibleWallHeight', 'physxCharacterController:maxJumpHeight', 'physxCharacterController:contactOffset', 'physxCharacterController:stepOffset', 'physxCharacterController:scaleCoeff', 'physxCharacterController:volumeGrowth', 'physxCharacterController:simulationOwner'],
    PhysxSchema.PhysxTriggerAPI: ['physxTrigger:enterScriptType', 'physxTrigger:leaveScriptType', 'physxTrigger:onEnterScript', 'physxTrigger:onLeaveScript'],
    PhysxSchema.PhysxCookedDataAPI: ['physxCookedData:type', 'physxCookedData:buffer'],
    PhysxSchema.PhysxVehicleContextAPI: ['physxVehicleContext:updateMode', 'physxVehicleContext:verticalAxis', 'physxVehicleContext:longitudinalAxis'],
    PhysxSchema.PhysxVehicleTireFrictionTable: ['groundMaterials', 'frictionValues', 'defaultFrictionValue'],
    PhysxSchema.PhysxVehicleWheelAPI: ['physxVehicleWheel:radius', 'physxVehicleWheel:width', 'physxVehicleWheel:mass', 'physxVehicleWheel:moi', 'physxVehicleWheel:dampingRate'],
    PhysxSchema.PhysxVehicleTireAPI: ['physxVehicleTire:lateralStiffnessGraph', 'physxVehicleTire:longitudinalStiffness', 'physxVehicleTire:camberStiffness', 'physxVehicleTire:frictionVsSlipGraph', 'physxVehicleTire:frictionTable', 'physxVehicleTire:restLoad'],
    PhysxSchema.PhysxVehicleSuspensionAPI: ['physxVehicleSuspension:springStrength', 'physxVehicleSuspension:springDamperRate', 'physxVehicleSuspension:travelDistance', 'physxVehicleSuspension:sprungMass'],
    PhysxSchema.PhysxVehicleWheelAttachmentAPI: ['physxVehicleWheelAttachment:index', 'physxVehicleWheelAttachment:wheel', 'physxVehicleWheelAttachment:tire', 'physxVehicleWheelAttachment:suspension', 'physxVehicleWheelAttachment:suspensionTravelDirection', 'physxVehicleWheelAttachment:suspensionFramePosition', 'physxVehicleWheelAttachment:suspensionFrameOrientation', 'physxVehicleWheelAttachment:wheelFramePosition', 'physxVehicleWheelAttachment:wheelFrameOrientation', 'physxVehicleWheelAttachment:collisionGroup'],
    PhysxSchema.PhysxVehicleEngineAPI: ['physxVehicleEngine:moi', 'physxVehicleEngine:peakTorque', 'physxVehicleEngine:maxRotationSpeed', 'physxVehicleEngine:idleRotationSpeed', 'physxVehicleEngine:torqueCurve', 'physxVehicleEngine:dampingRateFullThrottle', 'physxVehicleEngine:dampingRateZeroThrottleClutchEngaged', 'physxVehicleEngine:dampingRateZeroThrottleClutchDisengaged'],
    PhysxSchema.PhysxVehicleGearsAPI: ['physxVehicleGears:ratios', 'physxVehicleGears:ratioScale', 'physxVehicleGears:switchTime'],
    PhysxSchema.PhysxVehicleAutoGearBoxAPI: ['physxVehicleAutoGearBox:upRatios', 'physxVehicleAutoGearBox:downRatios', 'physxVehicleAutoGearBox:latency'],
    PhysxSchema.PhysxVehicleClutchAPI: ['physxVehicleClutch:strength'],
    PhysxSchema.PhysxVehicleNonlinearCommandResponseAPI: ['commandValues', 'speedResponsesPerCommandValue', 'speedResponses'],
    PhysxSchema.PhysxVehicleDriveBasicAPI: ['physxVehicleDriveBasic:peakTorque'],
    PhysxSchema.PhysxVehicleDriveStandardAPI: ['physxVehicleDriveStandard:engine', 'physxVehicleDriveStandard:gears', 'physxVehicleDriveStandard:autoGearBox', 'physxVehicleDriveStandard:clutch'],
    PhysxSchema.PhysxVehicleAPI: ['physxVehicle:vehicleEnabled', PhysxSchema.Tokens.referenceFrameIsCenterOfMass, 'physxVehicle:limitSuspensionExpansionVelocity', 'physxVehicle:drive', 'physxVehicle:suspensionLineQueryType', 'physxVehicle:subStepThresholdLongitudinalSpeed', 'physxVehicle:lowForwardSpeedSubStepCount', 'physxVehicle:highForwardSpeedSubStepCount', 'physxVehicle:minPassiveLongitudinalSlipDenominator', 'physxVehicle:minActiveLongitudinalSlipDenominator', 'physxVehicle:minLateralSlipDenominator', 'physxVehicle:longitudinalStickyTireThresholdSpeed', 'physxVehicle:longitudinalStickyTireThresholdTime', 'physxVehicle:longitudinalStickyTireDamping', 'physxVehicle:lateralStickyTireThresholdSpeed', 'physxVehicle:lateralStickyTireThresholdTime', 'physxVehicle:lateralStickyTireDamping'],
    PhysxSchema.PhysxVehicleControllerAPI: vehicleControllerAPIPropOrder,
    PhysxSchema.PhysxVehicleTankControllerAPI: ['physxVehicleTankController:thrust0', 'physxVehicleTankController:thrust1'] + vehicleControllerAPIPropOrder,
    PhysxSchema.PhysxVehicleWheelControllerAPI: ['physxVehicleWheelController:driveTorque', 'physxVehicleWheelController:brakeTorque', 'physxVehicleWheelController:steerAngle'],
    PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI: vehicleMultiWheelDifferentialAPIPropOrder,
    PhysxSchema.PhysxVehicleTankDifferentialAPI: vehicleMultiWheelDifferentialAPIPropOrder + ['physxVehicleTankDifferential:numberOfWheelsPerTrack', 'physxVehicleTankDifferential:thrustIndexPerTrack', 'physxVehicleTankDifferential:trackToWheelIndices', 'physxVehicleTankDifferential:wheelIndicesInTrackOrder'],
    PhysxSchema.PhysxVehicleBrakesAPI: ['maxBrakeTorque', 'wheels', 'torqueMultipliers'],
    PhysxSchema.PhysxVehicleSteeringAPI: ['physxVehicleSteering:wheels', 'physxVehicleSteering:maxSteerAngle', 'physxVehicleSteering:angleMultipliers'],
    PhysxSchema.PhysxVehicleAckermannSteeringAPI: ['physxVehicleAckermannSteering:wheel0', 'physxVehicleAckermannSteering:wheel1', 'physxVehicleAckermannSteering:maxSteerAngle', 'physxVehicleAckermannSteering:wheelBase', 'physxVehicleAckermannSteering:trackWidth', 'physxVehicleAckermannSteering:strength'],
    PhysxSchema.PhysxVehicleSuspensionComplianceAPI: ['physxVehicleSuspensionCompliance:wheelToeAngle', 'physxVehicleSuspensionCompliance:wheelCamberAngle', 'physxVehicleSuspensionCompliance:suspensionForceAppPoint', 'physxVehicleSuspensionCompliance:tireForceAppPoint'],
    PhysxSchema.PhysxCameraAPI: ['physxCamera:alwaysUpdateEnabled', 'physxCamera:subject'],
    PhysxSchema.PhysxCameraFollowLookAPI: ['physxFollowCamera:yawAngle', 'physxFollowCamera:pitchAngle', 'physxFollowCamera:pitchAngleTimeConstant', 'physxFollowCamera:slowSpeedPitchAngleScale', 'physxFollowCamera:slowPitchAngleSpeed', 'physxFollowLookCamera:downHillGroundAngle', 'physxFollowLookCamera:downHillGroundPitch', 'physxFollowLookCamera:upHillGroundAngle', 'physxFollowLookCamera:upHillGroundPitch', 'physxFollowCamera:velocityNormalMinSpeed', 'physxFollowLookCamera:velocityBlendTimeConstant', 'physxFollowCamera:followMinSpeed', 'physxFollowCamera:followMinDistance', 'physxFollowCamera:followMaxSpeed', 'physxFollowCamera:followMaxDistance', 'physxFollowLookCamera:followReverseSpeed', 'physxFollowLookCamera:followReverseDistance', 'physxFollowCamera:yawRateTimeConstant', 'physxFollowCamera:followTurnRateGain', 'physxFollowCamera:cameraPositionTimeConstant', 'physxFollowCamera:vehiclePositionOffset', 'physxFollowCamera:lookAheadMinSpeed', 'physxFollowCamera:lookAheadMinDistance', 'physxFollowCamera:lookAheadMaxSpeed', 'physxFollowCamera:lookAheadMaxDistance', 'physxFollowCamera:lookAheadTurnRateGain', 'physxFollowCamera:lookPositionHeight', 'physxFollowCamera:lookPositionTimeConstant'],
    PhysxSchema.PhysxCameraFollowVelocityAPI: ['physxVelocityCamera:yawAngle', 'physxVelocityCamera:pitchAngle', 'physxVelocityCamera:pitchAngleTimeConstant', 'physxVelocityCamera:slowSpeedPitchAngleScale', 'physxVelocityCamera:slowPitchAngleSpeed', 'physxVelocityCamera:velocityNormalMinSpeed', 'physxVelocityCamera:followMinSpeed', 'physxVelocityCamera:followMinDistance', 'physxVelocityCamera:followMaxSpeed', 'physxVelocityCamera:followMaxDistance', 'physxVelocityCamera:cameraPositionTimeConstant', 'physxVelocityCamera:PositionOffset', 'physxVelocityCamera:lookAheadMinSpeed', 'physxVelocityCamera:lookAheadMinDistance', 'physxVelocityCamera:lookAheadMaxSpeed', 'physxFollowVelocityCamera:lookAheadMaxDistance', 'physxVelocityCamera:lookPositionHeight', 'physxVelocityCamera:lookPositionTimeConstant'],
    PhysxSchema.PhysxCameraDroneAPI: ['physxVehicleDroneCamera:followHeight', 'physxVehicleDroneCamera:followDistance', 'physxVehicleDroneCamera:maxDistance', 'physxVehicleDroneCamera:maxSpeed', 'physxVehicleDroneCamera:horizontalVelocityGain', 'physxVehicleDroneCamera:verticalVelocityGain', 'physxVehicleDroneCamera:feedForwardVelocityGain', 'physxVehicleDroneCamera:velocityFilterTimeConstant', 'physxVehicleDroneCamera:rotationFilterTimeConstant', 'physxVehicleDroneCamera:vehiclePositionOffset'],
    PhysxSchema.PhysxParticleSystem: ['particleSystemEnabled', 'enableCCD', 'contactOffset', 'restOffset', 'particleContactOffset', 'solidRestOffset', 'fluidRestOffset', 'solverPositionIterationCount', 'solverVelocityIterationCount', 'maxVelocity', 'maxDepenetrationVelocity', 'enableCCD', 'wind', 'maxNeighborhood', 'neighborhoodScale'],
    PhysxSchema.PhysxTendonAxisAPI: ['gearing', 'jointAxis'],
    PhysxSchema.PhysxTendonAxisRootAPI: ['tendonEnabled', 'stiffness', 'limitStiffness', 'damping', 'offset', 'restLength', 'lowerLimit', 'upperLimit'],
    PhysxSchema.PhysxTendonAttachmentRootAPI: ['tendonEnabled', 'stiffness', 'limitStiffness', 'damping', 'offset'],
    PhysxSchema.PhysxTendonAttachmentLeafAPI: ['restLength', 'upperLimit', 'lowerLimit'],
    PhysxSchema.PhysxParticleSamplingAPI: ['physxParticleSampling:samplingDistance', 'physxParticleSampling:volume', 'physxParticleSampling:particles', 'physxParticleSampling:maxSamples'],
    PhysxSchema.PhysxParticleSetAPI: ['physxParticle:particleEnabled', 'physxParticle:selfCollision', 'physxParticle:fluid', 'physxParticle:particleGroup'],
    # DEPRECATED
    PhysxSchema.PhysxParticleClothAPI: ['physxParticle:particleEnabled', 'physxParticleCloth:selfCollisionFilter', 'physxParticle:pressure'],
    PhysxSchema.PhysxAutoParticleClothAPI: ['physxAutoParticleCloth:springStretchStiffness', 'physxAutoParticleCloth:springBendStiffness', 'physxAutoParticleCloth:springShearStiffness', 'physxAutoParticleCloth:springDamping'],
    #~DEPRECATED
    PhysxSchema.PhysxParticleAnisotropyAPI: ['physxParticleAnisotropy:particleAnisotropyEnabled', 'physxParticleAnisotropy:scale', 'physxParticleAnisotropy:min', 'physxParticleAnisotropy:max'],
    PhysxSchema.PhysxParticleSmoothingAPI: ['physxParticleSmoothing:particleSmoothingEnabled', 'physxParticleSmoothing:smoothing'],
    PhysxSchema.PhysxParticleIsosurfaceAPI: ['physxParticleIsosurface:isosurfaceEnabled', 'physxParticleIsosurface:maxVertices', 'physxParticleIsosurface:maxTriangles', 'physxParticleIsosurface:maxSubgrids', 'physxParticleIsosurface:gridSpacing', 'physxParticleIsosurface:surfaceDistance', 'physxParticleIsosurface:gridFilteringFlags', 'physxParticleIsosurface:gridSmoothingRadiusRelativeToCellSize', 'physxParticleIsosurface:enableAnisotropy', 'physxParticleIsosurface:anisotropyMin', 'physxParticleIsosurface:anisotropyMax', 'physxParticleIsosurface:anisotropyRadius', 'physxParticleIsosurface:numMeshSmoothingPasses', 'physxParticleIsosurface:numMeshNormalSmoothingPasses'],
    PhysxSchema.PhysxDiffuseParticlesAPI: ['physxDiffuseParticles:diffuseParticlesEnabled', 'physxDiffuseParticles:maxDiffuseParticles', 'physxDiffuseParticles:threshold', 'physxDiffuseParticles:lifetime', 'physxDiffuseParticles:airDrag', 'physxDiffuseParticles:bubbleDrag', 'physxDiffuseParticles:buoyancy', 'physxDiffuseParticles:kineticEnergyWeight', 'physxDiffuseParticles:pressureWeight', 'physxDiffuseParticles:divergenceWeight', 'physxDiffuseParticles:collisionDecay'],
    PhysxSchema.PhysxPBDMaterialAPI: ['physxPBDMaterial:density', 'physxPBDMaterial:friction', 'physxPBDMaterial:damping', 'physxPBDMaterial:adhesion', 'physxPBDMaterial:viscosity', 'physxPBDMaterial:cohesion', 'physxPBDMaterial:surfaceTension', 'physxPBDMaterial:particleAdhesionScale', 'physxPBDMaterial:adhesionOffsetScale', 'physxPBDMaterial:drag', 'physxPBDMaterial:lift', 'physxPBDMaterial:particleFrictionScale', 'physxPBDMaterial:vorticityConfinement'],
    PhysxSchema.PhysxMimicJointAPI: ['physxMimicJoint:referenceJoint', 'physxMimicJoint:referenceJointAxis', 'physxMimicJoint:gearing', 'physxMimicJoint:offset', 'physxMimicJoint:naturalFrequency', 'physxMimicJoint:dampingRatio'],
	# DEPRECATED
    PhysxSchema.PhysxDeformableBodyMaterialAPI: ['physxDeformableBodyMaterial:density', 'physxDeformableBodyMaterial:dynamicFriction', 'physxDeformableBodyMaterial:youngsModulus', 'physxDeformableBodyMaterial:poissonsRatio', 'physxDeformableBodyMaterial:elasticityDamping', 'physxDeformableBodyMaterial:dampingScale'],
    PhysxSchema.PhysxDeformableSurfaceMaterialAPI: ['physxDeformableSurfaceMaterial:density', 'physxDeformableSurfaceMaterial:thickness', 'physxDeformableSurfaceMaterial:dynamicFriction', 'physxDeformableSurfaceMaterial:youngsModulus', 'physxDeformableSurfaceMaterial:poissonsRatio', 'physxDeformableSurfaceMaterial:elasticityDamping', 'physxDeformableSurfaceMaterial:bendStiffness', 'physxDeformableSurfaceMaterial:bendDamping'],
    PhysxSchema.PhysxDeformableBodyAPI: ['physxDeformable:deformableEnabled', 'physxDeformable:kinematicEnabled', 'physxDeformable:collisionSimplification', 'physxDeformable:selfCollision', 'physxDeformable:enableCCD', 'physxDeformable:simulationHexahedralResolution', 'physxDeformable:vertexVelocityDamping', 'physxDeformable:solverPositionIterationCount', 'physxDeformable:sleepThreshold', 'physxDeformable:settlingThreshold', 'physxDeformable:sleepDamping', 'physxDeformable:selfCollisionFilterDistance'],
    PhysxSchema.PhysxDeformableSurfaceAPI: ['physxDeformable:deformableEnabled', 'physxDeformable:selfCollision', 'physxDeformableSurface:flatteningEnabled', 'physxDeformable:solverPositionIterationCount', 'physxDeformableSurface:maxVelocity', 'physxDeformable:vertexVelocityDamping', 'physxDeformable:sleepThreshold', 'physxDeformable:settlingThreshold', 'physxDeformable:sleepDamping', 'physxCollision:contactOffset', 'physxCollision:restOffset', 'physxDeformableSurface:collisionPairUpdateFrequency', 'physxDeformableSurface:collisionIterationMultiplier', 'physxDeformable:maxDepenetrationVelocity', 'physxDeformable:selfCollisionFilterDistance'],
    PhysxSchema.PhysxPhysicsAttachment: ['attachmentEnabled'],
    PhysxSchema.PhysxAutoAttachmentAPI: ['physxAutoAttachment:enableDeformableVertexAttachments', 'physxAutoAttachment:deformableVertexOverlapOffset', 'physxAutoAttachment:enableRigidSurfaceAttachments', 'physxAutoAttachment:rigidSurfaceSamplingDistance', 'physxAutoAttachment:enableCollisionFiltering', 'physxAutoAttachment:collisionFilteringOffset', 'physxAutoAttachment:maskShapes'],
}