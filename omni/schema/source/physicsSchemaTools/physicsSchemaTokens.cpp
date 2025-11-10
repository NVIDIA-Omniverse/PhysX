//
// Copyright 2016 Pixar
//
// Licensed under the Apache License, Version 2.0 (the "Apache License")
// with the following modification; you may not use this file except in
// compliance with the Apache License and the following modification to it:
// Section 6. Trademarks. is deleted and replaced with:
//
// 6. Trademarks. This License does not grant permission to use the trade
//    names, trademarks, service marks, or product names of the Licensor
//    and its affiliates, except as required to comply with Section 4(c) of
//    the License and to reproduce the content of the NOTICE file.
//
// You may obtain a copy of the Apache License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the Apache License with the above modification is
// distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied. See the Apache License for the specific
// language governing permissions and limitations under the Apache License.
//
#include "physicsSchemaTokens.h"

PXR_NAMESPACE_OPEN_SCOPE

///////////////////////////////////////////////////////////
// UsdPhysics token tables                               //
///////////////////////////////////////////////////////////

PhysicsAPITokensType::PhysicsAPITokensType() :
    RigidBodyAPI("PhysicsRigidBodyAPI", TfToken::Immortal),
    MassAPI("PhysicsMassAPI", TfToken::Immortal),
    CollisionAPI("PhysicsCollisionAPI", TfToken::Immortal),
    MeshCollisionAPI("PhysicsMeshCollisionAPI", TfToken::Immortal),
    MaterialAPI("PhysicsMaterialAPI", TfToken::Immortal),
    FilteredPairsAPI("PhysicsFilteredPairsAPI", TfToken::Immortal),
    LimitAPI("PhysicsLimitAPI", TfToken::Immortal),
    DriveAPI("PhysicsDriveAPI", TfToken::Immortal),
    ArticulationRootAPI("PhysicsArticulationRootAPI", TfToken::Immortal),

    allTokens({
        RigidBodyAPI,
        MassAPI,
        CollisionAPI,
        MeshCollisionAPI,
        MaterialAPI,
        FilteredPairsAPI,
        LimitAPI,
        DriveAPI,
        ArticulationRootAPI
    })
{
}

TfStaticData<PhysicsAPITokensType> PhysicsAPITokens;

PhysicsTypeTokensType::PhysicsTypeTokensType() :
    Scene("PhysicsScene", TfToken::Immortal),
    CollisionGroup("PhysicsCollisionGroup", TfToken::Immortal),
    Joint("PhysicsJoint", TfToken::Immortal),
    RevoluteJoint("PhysicsRevoluteJoint", TfToken::Immortal),
    PrismaticJoint("PhysicsPrismaticJoint", TfToken::Immortal),
    SphericalJoint("PhysicsSphericalJoint", TfToken::Immortal),
    DistanceJoint("PhysicsDistanceJoint", TfToken::Immortal),
    FixedJoint("PhysicsFixedJoint", TfToken::Immortal),

    allTokens({
        Scene,
        CollisionGroup,
        Joint,
        RevoluteJoint,
        PrismaticJoint,
        SphericalJoint,
        DistanceJoint,
        FixedJoint
    })
{
}

TfStaticData<PhysicsTypeTokensType> PhysicsTypeTokens;


///////////////////////////////////////////////////////////
// PhysxAddition token tables                            //
///////////////////////////////////////////////////////////

PhysxAdditionAPITokensType::PhysxAdditionAPITokensType() :
    DrivePerformanceEnvelopeAPI("PhysxDrivePerformanceEnvelopeAPI", TfToken::Immortal),
    PhysxJointAxisAPI("PhysxJointAxisAPI", TfToken::Immortal),
    PhysxSplinesSurfaceVelocityAPI("PhysxSplinesSurfaceVelocityAPI", TfToken::Immortal),
    BaseDeformableBodyAPI("PhysxBaseDeformableBodyAPI", TfToken::Immortal),
    SurfaceDeformableBodyAPI("PhysxSurfaceDeformableBodyAPI", TfToken::Immortal),
    AutoDeformableBodyAPI("PhysxAutoDeformableBodyAPI", TfToken::Immortal),
    AutoDeformableHexahedralMeshAPI("PhysxAutoDeformableHexahedralMeshAPI", TfToken::Immortal),
    AutoDeformableMeshSimplificationAPI("PhysxAutoDeformableMeshSimplificationAPI", TfToken::Immortal),
    DeformableMaterialAPI("PhysxDeformableMaterialAPI", TfToken::Immortal),
    SurfaceDeformableMaterialAPI("PhysxSurfaceDeformableMaterialAPI", TfToken::Immortal),
    AutoDeformableAttachmentAPI("PhysxAutoDeformableAttachmentAPI", TfToken::Immortal),

    allTokens({
        DrivePerformanceEnvelopeAPI,
        PhysxJointAxisAPI,
        PhysxSplinesSurfaceVelocityAPI,
        BaseDeformableBodyAPI,
        SurfaceDeformableBodyAPI,
        AutoDeformableBodyAPI,
        AutoDeformableHexahedralMeshAPI,
        AutoDeformableMeshSimplificationAPI,
        DeformableMaterialAPI,
        SurfaceDeformableMaterialAPI,
        AutoDeformableAttachmentAPI
    })
{
}

TfStaticData<PhysxAdditionAPITokensType> PhysxAdditionAPITokens;

PhysxAdditionAttrTokensType::PhysxAdditionAttrTokensType() :
    maxActuatorVelocityAngular("physxDrivePerformanceEnvelope:angular:maxActuatorVelocity", TfToken::Immortal),
    maxActuatorVelocityLinear("physxDrivePerformanceEnvelope:linear:maxActuatorVelocity", TfToken::Immortal),
    maxActuatorVelocityRotX("physxDrivePerformanceEnvelope:rotX:maxActuatorVelocity", TfToken::Immortal),
    maxActuatorVelocityRotY("physxDrivePerformanceEnvelope:rotY:maxActuatorVelocity", TfToken::Immortal),
    maxActuatorVelocityRotZ("physxDrivePerformanceEnvelope:rotZ:maxActuatorVelocity", TfToken::Immortal),
    velocityDependentResistanceAngular("physxDrivePerformanceEnvelope:angular:velocityDependentResistance", TfToken::Immortal),
    velocityDependentResistanceLinear("physxDrivePerformanceEnvelope:linear:velocityDependentResistance", TfToken::Immortal),
    velocityDependentResistanceRotX("physxDrivePerformanceEnvelope:rotX:velocityDependentResistance", TfToken::Immortal),
    velocityDependentResistanceRotY("physxDrivePerformanceEnvelope:rotY:velocityDependentResistance", TfToken::Immortal),
    velocityDependentResistanceRotZ("physxDrivePerformanceEnvelope:rotZ:velocityDependentResistance", TfToken::Immortal),
    speedEffortGradientAngular("physxDrivePerformanceEnvelope:angular:speedEffortGradient", TfToken::Immortal),
    speedEffortGradientLinear("physxDrivePerformanceEnvelope:linear:speedEffortGradient", TfToken::Immortal),
    speedEffortGradientRotX("physxDrivePerformanceEnvelope:rotX:speedEffortGradient", TfToken::Immortal),
    speedEffortGradientRotY("physxDrivePerformanceEnvelope:rotY:speedEffortGradient", TfToken::Immortal),
    speedEffortGradientRotZ("physxDrivePerformanceEnvelope:rotZ:speedEffortGradient", TfToken::Immortal),
    armatureAngular("physxJointAxis:angular:armature", TfToken::Immortal),
    armatureLinear("physxJointAxis:linear:armature", TfToken::Immortal),
    armatureRotX("physxJointAxis:rotX:armature", TfToken::Immortal),
    armatureRotY("physxJointAxis:rotY:armature", TfToken::Immortal),
    armatureRotZ("physxJointAxis:rotZ:armature", TfToken::Immortal),
    maxJointVelocityAngular("physxJointAxis:angular:maxJointVelocity", TfToken::Immortal),
    maxJointVelocityLinear("physxJointAxis:linear:maxJointVelocity", TfToken::Immortal),
    maxJointVelocityRotX("physxJointAxis:rotX:maxJointVelocity", TfToken::Immortal),
    maxJointVelocityRotY("physxJointAxis:rotY:maxJointVelocity", TfToken::Immortal),
    maxJointVelocityRotZ("physxJointAxis:rotZ:maxJointVelocity", TfToken::Immortal),
    staticFrictionEffortAngular("physxJointAxis:angular:staticFrictionEffort", TfToken::Immortal),
    staticFrictionEffortLinear("physxJointAxis:linear:staticFrictionEffort", TfToken::Immortal),
    staticFrictionEffortRotX("physxJointAxis:rotX:staticFrictionEffort", TfToken::Immortal),
    staticFrictionEffortRotY("physxJointAxis:rotY:staticFrictionEffort", TfToken::Immortal),
    staticFrictionEffortRotZ("physxJointAxis:rotZ:staticFrictionEffort", TfToken::Immortal),
    dynamicFrictionEffortAngular("physxJointAxis:angular:dynamicFrictionEffort", TfToken::Immortal),
    dynamicFrictionEffortLinear("physxJointAxis:linear:dynamicFrictionEffort", TfToken::Immortal),
    dynamicFrictionEffortRotX("physxJointAxis:rotX:dynamicFrictionEffort", TfToken::Immortal),
    dynamicFrictionEffortRotY("physxJointAxis:rotY:dynamicFrictionEffort", TfToken::Immortal),
    dynamicFrictionEffortRotZ("physxJointAxis:rotZ:dynamicFrictionEffort", TfToken::Immortal),
    viscousFrictionCoefficientAngular("physxJointAxis:angular:viscousFrictionCoefficient", TfToken::Immortal),
    viscousFrictionCoefficientLinear("physxJointAxis:linear:viscousFrictionCoefficient", TfToken::Immortal),
    viscousFrictionCoefficientRotX("physxJointAxis:rotX:viscousFrictionCoefficient", TfToken::Immortal),
    viscousFrictionCoefficientRotY("physxJointAxis:rotY:viscousFrictionCoefficient", TfToken::Immortal),
    viscousFrictionCoefficientRotZ("physxJointAxis:rotZ:viscousFrictionCoefficient", TfToken::Immortal),
    surfaceVelocityEnabled("physxSplinesSurfaceVelocity:surfaceVelocityEnabled", TfToken::Immortal),
    surfaceVelocityMagnitude("physxSplinesSurfaceVelocity:surfaceVelocityMagnitude", TfToken::Immortal),
    surfaceVelocityCurve("physxSplinesSurfaceVelocity:surfaceVelocityCurve", TfToken::Immortal),
    solverPositionIterationCount("physxDeformableBody:solverPositionIterationCount", TfToken::Immortal), // rel name
    linearDamping("physxDeformableBody:linearDamping", TfToken::Immortal),
    maxLinearVelocity("physxDeformableBody:maxLinearVelocity", TfToken::Immortal),
    settlingDamping("physxDeformableBody:settlingDamping", TfToken::Immortal),
    settlingThreshold("physxDeformableBody:settlingThreshold", TfToken::Immortal),
    sleepThreshold("physxDeformableBody:sleepThreshold", TfToken::Immortal),
    maxDepenetrationVelocity("physxDeformableBody:maxDepenetrationVelocity", TfToken::Immortal),
    selfCollision("physxDeformableBody:selfCollision", TfToken::Immortal),
    selfCollisionFilterDistance("physxDeformableBody:selfCollisionFilterDistance", TfToken::Immortal),
    enableSpeculativeCCD("physxDeformableBody:enableSpeculativeCCD", TfToken::Immortal),
    disableGravity("physxDeformableBody:disableGravity", TfToken::Immortal),
    collisionPairUpdateFrequency("physxDeformableBody:collisionPairUpdateFrequency", TfToken::Immortal),
    collisionIterationMultiplier("physxDeformableBody:collisionIterationMultiplier", TfToken::Immortal),
    autoDeformableBodyEnabled("physxDeformableBody:autoDeformableBodyEnabled", TfToken::Immortal),
    cookingSourceMesh("physxDeformableBody:cookingSourceMesh", TfToken::Immortal), // rel name
    resolution("physxDeformableBody:resolution", TfToken::Immortal),
    autoDeformableMeshSimplificationEnabled("physxDeformableBody:autoDeformableMeshSimplificationEnabled", TfToken::Immortal),
    remeshingEnabled("physxDeformableBody:remeshingEnabled", TfToken::Immortal),
    remeshingResolution("physxDeformableBody:remeshingResolution", TfToken::Immortal),
    targetTriangleCount("physxDeformableBody:targetTriangleCount", TfToken::Immortal),
    forceConforming("physxDeformableBody:forceConforming", TfToken::Immortal),
    elasticityDamping("physxDeformableMaterial:elasticityDamping", TfToken::Immortal),
    bendDamping("physxDeformableMaterial:bendDamping", TfToken::Immortal),
    attachable0("physxAutoDeformableAttachment:attachable0", TfToken::Immortal),
    attachable1("physxAutoDeformableAttachment:attachable1", TfToken::Immortal),
    enableDeformableVertexAttachments("physxAutoDeformableAttachment:enableDeformableVertexAttachments", TfToken::Immortal),
    deformableVertexOverlapOffset("physxAutoDeformableAttachment:deformableVertexOverlapOffset", TfToken::Immortal), // rel name
    enableRigidSurfaceAttachments("physxAutoDeformableAttachment:enableRigidSurfaceAttachments", TfToken::Immortal), // rel name
    rigidSurfaceSamplingDistance("physxAutoDeformableAttachment:rigidSurfaceSamplingDistance", TfToken::Immortal),
    enableCollisionFiltering("physxAutoDeformableAttachment:enableCollisionFiltering", TfToken::Immortal),
    collisionFilteringOffset("physxAutoDeformableAttachment:collisionFilteringOffset", TfToken::Immortal),
    maskShapes("physxAutoDeformableAttachment:maskShapes", TfToken::Immortal),
    enableDeformableFilteringPairs("physxAutoDeformableAttachment:enableDeformableFilteringPairs", TfToken::Immortal),

    allTokens({
        maxActuatorVelocityAngular,
        maxActuatorVelocityLinear,
        maxActuatorVelocityRotX,
        maxActuatorVelocityRotY,
        maxActuatorVelocityRotZ,
        velocityDependentResistanceAngular,
        velocityDependentResistanceLinear,
        velocityDependentResistanceRotX,
        velocityDependentResistanceRotY,
        velocityDependentResistanceRotZ,
        speedEffortGradientAngular,
        speedEffortGradientLinear,
        speedEffortGradientRotX,
        speedEffortGradientRotY,
        speedEffortGradientRotZ,
        armatureAngular,
        armatureLinear,
        armatureRotX,
        armatureRotY,
        armatureRotZ,
        maxJointVelocityAngular,
        maxJointVelocityLinear,
        maxJointVelocityRotX,
        maxJointVelocityRotY,
        maxJointVelocityRotZ,
        staticFrictionEffortAngular,
        staticFrictionEffortLinear,
        staticFrictionEffortRotX,
        staticFrictionEffortRotY,
        staticFrictionEffortRotZ,
        dynamicFrictionEffortAngular,
        dynamicFrictionEffortLinear,
        dynamicFrictionEffortRotX,
        dynamicFrictionEffortRotY,
        dynamicFrictionEffortRotZ,
        viscousFrictionCoefficientAngular,
        viscousFrictionCoefficientLinear,
        viscousFrictionCoefficientRotX,
        viscousFrictionCoefficientRotY,
        viscousFrictionCoefficientRotZ,
        surfaceVelocityEnabled,
        surfaceVelocityMagnitude,
        surfaceVelocityCurve,
        solverPositionIterationCount,
        linearDamping,
        maxLinearVelocity,
        settlingDamping,
        settlingThreshold,
        sleepThreshold,
        maxDepenetrationVelocity,
        selfCollision,
        selfCollisionFilterDistance,
        enableSpeculativeCCD,
        disableGravity,
        collisionPairUpdateFrequency,
        collisionIterationMultiplier,
        autoDeformableBodyEnabled,
        cookingSourceMesh,
        resolution,
        autoDeformableMeshSimplificationEnabled,
        remeshingEnabled,
        remeshingResolution,
        targetTriangleCount,
        forceConforming,
        elasticityDamping,
        bendDamping,
        attachable0,
        attachable1,
        enableDeformableVertexAttachments,
        deformableVertexOverlapOffset,
        enableRigidSurfaceAttachments,
        rigidSurfaceSamplingDistance,
        enableCollisionFiltering,
        collisionFilteringOffset,
        maskShapes,
        enableDeformableFilteringPairs
    })
{
}

TfStaticData<PhysxAdditionAttrTokensType> PhysxAdditionAttrTokens;

///////////////////////////////////////////////////////////
// OmniPhysicsDeformable token tables                    //
///////////////////////////////////////////////////////////

OmniPhysicsDeformableAPITokensType::OmniPhysicsDeformableAPITokensType() :
    BodyAPI("OmniPhysicsBodyAPI", TfToken::Immortal),
    DeformableBodyAPI("OmniPhysicsDeformableBodyAPI", TfToken::Immortal),
    VolumeDeformableSimAPI("OmniPhysicsVolumeDeformableSimAPI", TfToken::Immortal),
    SurfaceDeformableSimAPI("OmniPhysicsSurfaceDeformableSimAPI", TfToken::Immortal),
    CurvesDeformableSimAPI("OmniPhysicsCurvesDeformableSimAPI", TfToken::Immortal),
    DeformablePoseAPI("OmniPhysicsDeformablePoseAPI", TfToken::Immortal),
    BaseMaterialAPI("OmniPhysicsBaseMaterialAPI", TfToken::Immortal),
    DeformableMaterialAPI("OmniPhysicsDeformableMaterialAPI", TfToken::Immortal),
    SurfaceDeformableMaterialAPI("OmniPhysicsSurfaceDeformableMaterialAPI", TfToken::Immortal),
    CurvesDeformableMaterialAPI("OmniPhysicsCurvesDeformableMaterialAPI", TfToken::Immortal),

    allTokens({
        BodyAPI,
        DeformableBodyAPI,
        VolumeDeformableSimAPI,
        SurfaceDeformableSimAPI,
        CurvesDeformableSimAPI,
        DeformablePoseAPI,
        BaseMaterialAPI,
        DeformableMaterialAPI,
        SurfaceDeformableMaterialAPI,
        CurvesDeformableMaterialAPI
    })
{
}

TfStaticData<OmniPhysicsDeformableAPITokensType> OmniPhysicsDeformableAPITokens;

OmniPhysicsDeformableTypeTokensType::OmniPhysicsDeformableTypeTokensType() :
    Attachment("OmniPhysicsAttachment", TfToken::Immortal),
    VtxVtxAttachment("OmniPhysicsVtxVtxAttachment", TfToken::Immortal),
    VtxTriAttachment("OmniPhysicsVtxTriAttachment", TfToken::Immortal),
    VtxTetAttachment("OmniPhysicsVtxTetAttachment", TfToken::Immortal),
    VtxCrvAttachment("OmniPhysicsVtxCrvAttachment", TfToken::Immortal),
    VtxXformAttachment("OmniPhysicsVtxXformAttachment", TfToken::Immortal),
    TetXformAttachment("OmniPhysicsTetXformAttachment", TfToken::Immortal),
    TriTriAttachment("OmniPhysicsTriTriAttachment", TfToken::Immortal),
    ElementCollisionFilter("OmniPhysicsElementCollisionFilter", TfToken::Immortal),

    allTokens({
        Attachment,
        VtxVtxAttachment,
        VtxTriAttachment,
        VtxTetAttachment,
        VtxCrvAttachment,
        VtxXformAttachment,
        TetXformAttachment,
        TriTriAttachment,
        ElementCollisionFilter
    })
{
}

TfStaticData<OmniPhysicsDeformableTypeTokensType> OmniPhysicsDeformableTypeTokens;

OmniPhysicsDeformableAttrTokensType::OmniPhysicsDeformableAttrTokensType() :
    simulationOwner("omniphysics:simulationOwner", TfToken::Immortal), // rel name
    kinematicEnabled("omniphysics:kinematicEnabled", TfToken::Immortal),
    startsAsleep("omniphysics:startsAsleep", TfToken::Immortal),
    deformableBodyEnabled("omniphysics:deformableBodyEnabled", TfToken::Immortal),
    mass("omniphysics:mass", TfToken::Immortal),
    restShapePoints("omniphysics:restShapePoints", TfToken::Immortal),
    restTetVtxIndices("omniphysics:restTetVtxIndices", TfToken::Immortal),
    restTriVtxIndices("omniphysics:restTriVtxIndices", TfToken::Immortal),
    restBendAnglesDefault("omniphysics:restBendAnglesDefault", TfToken::Immortal),
    restAdjTriPairs("omniphysics:restAdjTriPairs", TfToken::Immortal),
    restBendAngles("omniphysics:restBendAngles", TfToken::Immortal),
    restCrvVtxIndices("omniphysics:restCrvVtxIndices", TfToken::Immortal),
    restNormals("omniphysics:restNormals", TfToken::Immortal),
    multipleApplyTemplate_purposes("deformablePose:__INSTANCE_NAME__:omniphysics:purposes", TfToken::Immortal),
    multipleApplyTemplate_points("deformablePose:__INSTANCE_NAME__:omniphysics:points", TfToken::Immortal),
    dynamicFriction("omniphysics:dynamicFriction", TfToken::Immortal),
    staticFriction("omniphysics:staticFriction", TfToken::Immortal),
    density("omniphysics:density", TfToken::Immortal),
    youngsModulus("omniphysics:youngsModulus", TfToken::Immortal),
    poissonsRatio("omniphysics:poissonsRatio", TfToken::Immortal),
    surfaceThickness("omniphysics:surfaceThickness", TfToken::Immortal),
    surfaceStretchStiffness("omniphysics:surfaceStretchStiffness", TfToken::Immortal),
    surfaceShearStiffness("omniphysics:surfaceShearStiffness", TfToken::Immortal),
    surfaceBendStiffness("omniphysics:surfaceBendStiffness", TfToken::Immortal),
    curveThickness("omniphysics:curveThickness", TfToken::Immortal),
    curveStretchStiffness("omniphysics:curveStretchStiffness", TfToken::Immortal),
    curveBendStiffness("omniphysics:curveBendStiffness", TfToken::Immortal),
    curveTwistStiffness("omniphysics:curveTwistStiffness", TfToken::Immortal),
    attachmentEnabled("omniphysics:attachmentEnabled", TfToken::Immortal),
    damping("omniphysics:damping", TfToken::Immortal),
    stiffness("omniphysics:stiffness", TfToken::Immortal),
    src0("omniphysics:src0", TfToken::Immortal), // rel name
    src1("omniphysics:src1", TfToken::Immortal), // rel name
    vtxIndicesSrc0("omniphysics:vtxIndicesSrc0", TfToken::Immortal),
    vtxIndicesSrc1("omniphysics:vtxIndicesSrc1", TfToken::Immortal),
    triIndicesSrc0("omniphysics:triIndicesSrc0", TfToken::Immortal),
    triIndicesSrc1("omniphysics:triIndicesSrc1", TfToken::Immortal),
    triCoordsSrc0("omniphysics:triCoordsSrc0", TfToken::Immortal),
    triCoordsSrc1("omniphysics:triCoordsSrc1", TfToken::Immortal),
    tetIndicesSrc0("omniphysics:tetIndicesSrc0", TfToken::Immortal),
    tetIndicesSrc1("omniphysics:tetIndicesSrc1", TfToken::Immortal),
    tetCoordsSrc0("omniphysics:tetCoordsSrc0", TfToken::Immortal),
    tetCoordsSrc1("omniphysics:tetCoordsSrc1", TfToken::Immortal),
    crvSegIndicesSrc1("omniphysics:crvSegIndicesSrc1", TfToken::Immortal),
    crvSegCoordsSrc1("omniphysics:crvSegCoordsSrc1", TfToken::Immortal),
    localPositionsSrc1("omniphysics:localPositionsSrc1", TfToken::Immortal),
    filterEnabled("omniphysics:filterEnabled", TfToken::Immortal),
    groupElemCounts0("omniphysics:groupElemCounts0", TfToken::Immortal),
    groupElemIndices0("omniphysics:groupElemIndices0", TfToken::Immortal),
    groupElemCounts1("omniphysics:groupElemCounts1", TfToken::Immortal),
    groupElemIndices1("omniphysics:groupElemIndices1", TfToken::Immortal),

    bindPose("bindPose", TfToken::Immortal), // purpose token
    selfCollisionFilterPose("selfCollisionFilterPose", TfToken::Immortal), // purpose token
    flatDefault("flatDefault", TfToken::Immortal), // restBendAnglesDefault token
    restShapeDefault("restShapeDefault", TfToken::Immortal), // restBendAnglesDefault token

    allTokens({
        simulationOwner,
        kinematicEnabled,
        startsAsleep,
        deformableBodyEnabled,
        mass,
        restShapePoints,
        restTetVtxIndices,
        restTriVtxIndices,
        restBendAnglesDefault,
        restAdjTriPairs,
        restBendAngles,
        restCrvVtxIndices,
        restNormals,
        multipleApplyTemplate_purposes,
        multipleApplyTemplate_points,
        dynamicFriction,
        staticFriction,
        density,
        youngsModulus,
        poissonsRatio,
        surfaceThickness,
        surfaceStretchStiffness,
        surfaceShearStiffness,
        surfaceBendStiffness,
        curveThickness,
        curveStretchStiffness,
        curveBendStiffness,
        curveTwistStiffness,
        attachmentEnabled,
        damping,
        stiffness,
        src0,
        src1,
        vtxIndicesSrc0,
        vtxIndicesSrc1,
        triIndicesSrc0,
        triIndicesSrc1,
        triCoordsSrc0,
        triCoordsSrc1,
        tetIndicesSrc0,
        tetIndicesSrc1,
        tetCoordsSrc0,
        tetCoordsSrc1,
        crvSegIndicesSrc1,
        crvSegCoordsSrc1,
        localPositionsSrc1,
        filterEnabled,
        groupElemCounts0,
        groupElemIndices0,
        groupElemCounts1,
        groupElemIndices1,
        bindPose,
        selfCollisionFilterPose,
        flatDefault,
        restShapeDefault
    })
{
}

TfStaticData<OmniPhysicsDeformableAttrTokensType> OmniPhysicsDeformableAttrTokens;


PXR_NAMESPACE_CLOSE_SCOPE
