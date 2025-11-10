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
#ifndef PHYSICS_SCHEMA_TOKENS_H
#define PHYSICS_SCHEMA_TOKENS_H

#include ".//api.h"
#include "pxr/pxr.h"
#include "pxr/base/tf/staticData.h"
#include "pxr/base/tf/token.h"
#include <vector>

PXR_NAMESPACE_OPEN_SCOPE

///////////////////////////////////////////////////////////
// UsdPhysics token tables                               //
///////////////////////////////////////////////////////////

struct PhysicsAPITokensType  {
    PHYSICSSCHEMATOOLS_API PhysicsAPITokensType();
    // applied API's
    const TfToken RigidBodyAPI;
    const TfToken MassAPI;
    const TfToken CollisionAPI;
    const TfToken MeshCollisionAPI;
    const TfToken MaterialAPI;
    const TfToken FilteredPairsAPI;
    const TfToken LimitAPI;
    const TfToken DriveAPI;
    const TfToken ArticulationRootAPI;

    /// A vector of all of the tokens listed above.
    const std::vector<TfToken> allTokens;
};

struct PhysicsTypeTokensType {
    PHYSICSSCHEMATOOLS_API PhysicsTypeTokensType();

    // derived types
    const TfToken Scene;
    const TfToken CollisionGroup;
    const TfToken Joint;
    const TfToken RevoluteJoint;
    const TfToken PrismaticJoint;
    const TfToken SphericalJoint;
    const TfToken DistanceJoint;
    const TfToken FixedJoint;

    /// A vector of all of the tokens listed above.
    const std::vector<TfToken> allTokens;
};

/// \var PhysicsAPITokens
///
/// A global variable with static, efficient \link TfToken TfTokens\endlink
/// for use in all public USD API.  \sa PhysicsTokensType
extern PHYSICSSCHEMATOOLS_API TfStaticData<PhysicsAPITokensType> PhysicsAPITokens;

/// \var PhysicsTypeTokens
///
/// A global variable with static, efficient \link TfToken TfTokens\endlink
/// for use in all public USD API.  \sa PhysicsTypeTokensType
extern PHYSICSSCHEMATOOLS_API TfStaticData<PhysicsTypeTokensType> PhysicsTypeTokens;


///////////////////////////////////////////////////////////
// PhysxAddition token tables                            //
///////////////////////////////////////////////////////////

struct PhysxAdditionAPITokensType{
    PHYSICSSCHEMATOOLS_API PhysxAdditionAPITokensType();
    const TfToken DrivePerformanceEnvelopeAPI;
    const TfToken PhysxJointAxisAPI;
    const TfToken PhysxSplinesSurfaceVelocityAPI;
    const TfToken BaseDeformableBodyAPI;
    const TfToken SurfaceDeformableBodyAPI;
    const TfToken AutoDeformableBodyAPI;
    const TfToken AutoDeformableHexahedralMeshAPI;
    const TfToken AutoDeformableMeshSimplificationAPI;
    const TfToken DeformableMaterialAPI;
    const TfToken SurfaceDeformableMaterialAPI;
    const TfToken AutoDeformableAttachmentAPI;

     /// A vector of all of the tokens listed above.
    const std::vector<TfToken> allTokens;
};

struct PhysxAdditionAttrTokensType {
    PHYSICSSCHEMATOOLS_API PhysxAdditionAttrTokensType();

    // attribute tokens
    const TfToken maxActuatorVelocityAngular;
    const TfToken maxActuatorVelocityLinear;
    const TfToken maxActuatorVelocityRotX;
    const TfToken maxActuatorVelocityRotY;
    const TfToken maxActuatorVelocityRotZ;
    const TfToken velocityDependentResistanceAngular;
    const TfToken velocityDependentResistanceLinear;
    const TfToken velocityDependentResistanceRotX;
    const TfToken velocityDependentResistanceRotY;
    const TfToken velocityDependentResistanceRotZ;
    const TfToken speedEffortGradientAngular;
    const TfToken speedEffortGradientLinear;
    const TfToken speedEffortGradientRotX;
    const TfToken speedEffortGradientRotY;
    const TfToken speedEffortGradientRotZ;
    const TfToken armatureAngular;
    const TfToken armatureLinear;
    const TfToken armatureRotX;
    const TfToken armatureRotY;
    const TfToken armatureRotZ;
    const TfToken maxJointVelocityAngular;
    const TfToken maxJointVelocityLinear;
    const TfToken maxJointVelocityRotX;
    const TfToken maxJointVelocityRotY;
    const TfToken maxJointVelocityRotZ;
    const TfToken staticFrictionEffortAngular;
    const TfToken staticFrictionEffortLinear;
    const TfToken staticFrictionEffortRotX;
    const TfToken staticFrictionEffortRotY;
    const TfToken staticFrictionEffortRotZ;
    const TfToken dynamicFrictionEffortAngular;
    const TfToken dynamicFrictionEffortLinear;
    const TfToken dynamicFrictionEffortRotX;
    const TfToken dynamicFrictionEffortRotY;
    const TfToken dynamicFrictionEffortRotZ;
    const TfToken viscousFrictionCoefficientAngular;
    const TfToken viscousFrictionCoefficientLinear;
    const TfToken viscousFrictionCoefficientRotX;
    const TfToken viscousFrictionCoefficientRotY;
    const TfToken viscousFrictionCoefficientRotZ;
    const TfToken surfaceVelocityEnabled;
    const TfToken surfaceVelocityMagnitude;
    const TfToken surfaceVelocityCurve;
    const TfToken solverPositionIterationCount;
    const TfToken linearDamping;
    const TfToken maxLinearVelocity;
    const TfToken settlingDamping;
    const TfToken settlingThreshold;
    const TfToken sleepThreshold;
    const TfToken maxDepenetrationVelocity;
    const TfToken selfCollision;
    const TfToken selfCollisionFilterDistance;
    const TfToken enableSpeculativeCCD;
    const TfToken disableGravity;
    const TfToken collisionPairUpdateFrequency;
    const TfToken collisionIterationMultiplier;
    const TfToken autoDeformableBodyEnabled;
    const TfToken cookingSourceMesh; // rel name
    const TfToken resolution;
    const TfToken autoDeformableMeshSimplificationEnabled;
    const TfToken remeshingEnabled;
    const TfToken remeshingResolution;
    const TfToken targetTriangleCount;
    const TfToken forceConforming;
    const TfToken elasticityDamping;
    const TfToken bendDamping;
    const TfToken attachable0; // rel name
    const TfToken attachable1; // rel name
    const TfToken enableDeformableVertexAttachments;
    const TfToken deformableVertexOverlapOffset;
    const TfToken enableRigidSurfaceAttachments;
    const TfToken rigidSurfaceSamplingDistance;
    const TfToken enableCollisionFiltering;
    const TfToken collisionFilteringOffset;
    const TfToken maskShapes; // rel name
    const TfToken enableDeformableFilteringPairs;

    /// A vector of all of the tokens listed above.
    const std::vector<TfToken> allTokens;
};

/// \var PhysxAdditionAPITokens
///
/// A global variable with static, efficient \link TfToken TfTokens\endlink
/// for use in all public USD API.  \sa PhysxAdditionAPITokensType
extern PHYSICSSCHEMATOOLS_API TfStaticData<PhysxAdditionAPITokensType> PhysxAdditionAPITokens;

/// \var PhysxAdditionAttrTokens
///
/// A global variable with static, efficient \link TfToken TfTokens\endlink
/// for use in all public USD API.  \sa PhysxAdditionAttrTokensType
extern PHYSICSSCHEMATOOLS_API TfStaticData<PhysxAdditionAttrTokensType> PhysxAdditionAttrTokens;

///////////////////////////////////////////////////////////
// OmniPhysicsDeformable token tables                    //
///////////////////////////////////////////////////////////

struct OmniPhysicsDeformableAPITokensType {
    PHYSICSSCHEMATOOLS_API OmniPhysicsDeformableAPITokensType();
    // applied API's
    const TfToken BodyAPI;
    const TfToken DeformableBodyAPI;
    const TfToken VolumeDeformableSimAPI;
    const TfToken SurfaceDeformableSimAPI;
    const TfToken CurvesDeformableSimAPI;
    const TfToken DeformablePoseAPI;
    const TfToken BaseMaterialAPI;
    const TfToken DeformableMaterialAPI;
    const TfToken SurfaceDeformableMaterialAPI;
    const TfToken CurvesDeformableMaterialAPI;

    /// A vector of all of the tokens listed above.
    const std::vector<TfToken> allTokens;
};

struct OmniPhysicsDeformableTypeTokensType {
    PHYSICSSCHEMATOOLS_API OmniPhysicsDeformableTypeTokensType();

    // derived types
    const TfToken Attachment;
    const TfToken VtxVtxAttachment;
    const TfToken VtxTriAttachment;
    const TfToken VtxTetAttachment;
    const TfToken VtxCrvAttachment;
    const TfToken VtxXformAttachment;
    const TfToken TetXformAttachment;
    const TfToken TriTriAttachment;
    const TfToken ElementCollisionFilter;

    /// A vector of all of the tokens listed above.
    const std::vector<TfToken> allTokens;
};

struct OmniPhysicsDeformableAttrTokensType {
    PHYSICSSCHEMATOOLS_API OmniPhysicsDeformableAttrTokensType();

    // attribute tokens
    const TfToken simulationOwner; // rel name
    const TfToken kinematicEnabled;
    const TfToken startsAsleep;
    const TfToken deformableBodyEnabled;
    const TfToken mass;
    const TfToken restShapePoints;
    const TfToken restTetVtxIndices;
    const TfToken restTriVtxIndices;
    const TfToken restBendAnglesDefault;
    const TfToken restAdjTriPairs;
    const TfToken restBendAngles;
    const TfToken restCrvVtxIndices;
    const TfToken restNormals;
    const TfToken multipleApplyTemplate_purposes;
    const TfToken multipleApplyTemplate_points;
    const TfToken dynamicFriction;
    const TfToken staticFriction;
    const TfToken density;
    const TfToken youngsModulus;
    const TfToken poissonsRatio;
    const TfToken surfaceThickness;
    const TfToken surfaceStretchStiffness;
    const TfToken surfaceShearStiffness;
    const TfToken surfaceBendStiffness;
    const TfToken curveThickness;
    const TfToken curveStretchStiffness;
    const TfToken curveBendStiffness;
    const TfToken curveTwistStiffness;
    const TfToken attachmentEnabled;
    const TfToken damping;
    const TfToken stiffness;
    const TfToken src0; // rel name
    const TfToken src1; // rel name
    const TfToken vtxIndicesSrc0;
    const TfToken vtxIndicesSrc1;
    const TfToken triIndicesSrc0;
    const TfToken triIndicesSrc1;
    const TfToken triCoordsSrc0;
    const TfToken triCoordsSrc1;
    const TfToken tetIndicesSrc0;
    const TfToken tetIndicesSrc1;
    const TfToken tetCoordsSrc0;
    const TfToken tetCoordsSrc1;
    const TfToken crvSegIndicesSrc1;
    const TfToken crvSegCoordsSrc1;
    const TfToken localPositionsSrc1;
    const TfToken filterEnabled;
    const TfToken groupElemCounts0;
    const TfToken groupElemIndices0;
    const TfToken groupElemCounts1;
    const TfToken groupElemIndices1;

    // standalone tokens
    const TfToken bindPose; // purposes token
    const TfToken selfCollisionFilterPose; // purposes token
    const TfToken flatDefault; // restBendAnglesDefault token
    const TfToken restShapeDefault; // restBendAnglesDefault token

    /// A vector of all of the tokens listed above.
    const std::vector<TfToken> allTokens;
};

/// \var OmniPhysicsDeformableAPITokens
///
/// A global variable with static, efficient \link TfToken TfTokens\endlink
/// for use in all public USD API.  \sa OmniPhysicsDeformableAPITokensType
extern PHYSICSSCHEMATOOLS_API TfStaticData<OmniPhysicsDeformableAPITokensType> OmniPhysicsDeformableAPITokens;

/// \var OmniPhysicsDeformableTypeTokens
///
/// A global variable with static, efficient \link TfToken TfTokens\endlink
/// for use in all public USD API.  \sa OmniPhysicsDeformableTypeTokensType
extern PHYSICSSCHEMATOOLS_API TfStaticData<OmniPhysicsDeformableTypeTokensType> OmniPhysicsDeformableTypeTokens;

/// \var OmniPhysicsDeformableAttrTokens
///
/// A global variable with static, efficient \link TfToken TfTokens\endlink
/// for use in all public USD API.  \sa OmniPhysicsDeformableAttrTokensType
extern PHYSICSSCHEMATOOLS_API TfStaticData<OmniPhysicsDeformableAttrTokensType> OmniPhysicsDeformableAttrTokens;

PXR_NAMESPACE_CLOSE_SCOPE

#endif
