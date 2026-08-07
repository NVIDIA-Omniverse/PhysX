// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// GENERATED (header-only) by tools/gen_tokens.py from schema.usda via usdGenSchema's
// GatherTokens. DO NOT EDIT. Codeless tokens: OmniUsdPhysicsDeformableSchemaTokens is defined inline (no lib).
/// \file omniUsdPhysicsDeformableSchema/tokens.h
#ifndef OMNIUSDPHYSICSDEFORMABLESCHEMA_TOKENS_H
#define OMNIUSDPHYSICSDEFORMABLESCHEMA_TOKENS_H

#include <pxr/pxr.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/token.h>
#include <vector>

PXR_NAMESPACE_OPEN_SCOPE

/// \class OmniUsdPhysicsDeformableSchemaTokensType
///
/// \link OmniUsdPhysicsDeformableSchemaTokens \endlink provides static, efficient
/// \link TfToken TfTokens\endlink for use in all public USD API.
///
/// These tokens are auto-generated from the module's schema, representing
/// property names, for when you need to fetch an attribute or relationship
/// directly by name, e.g. UsdPrim::GetAttribute(), in the most efficient
/// manner, and allow the compiler to verify that you spelled the name
/// correctly.
///
/// OmniUsdPhysicsDeformableSchemaTokens also contains all of the \em allowedTokens values
/// declared for schema builtin attributes of 'token' scene description type.
/// Use OmniUsdPhysicsDeformableSchemaTokens like so:
///
/// \code
///     gprim.GetMyTokenValuedAttr().Set(OmniUsdPhysicsDeformableSchemaTokens->bindPose);
/// \endcode
struct OmniUsdPhysicsDeformableSchemaTokensType {
    OmniUsdPhysicsDeformableSchemaTokensType();
    /// \brief "bindPose"
    ///
    /// Possible value for OmniUsdPhysicsDeformableSchemaDeformablePoseAPI::GetPurposesAttr()
    const TfToken bindPose;
    /// \brief "deformablePose"
    ///
    /// Property namespace prefix for the OmniUsdPhysicsDeformableSchemaDeformablePoseAPI schema.
    const TfToken deformablePose;
    /// \brief "deformablePose:__INSTANCE_NAME__:omniphysics:points"
    ///
    /// OmniUsdPhysicsDeformableSchemaDeformablePoseAPI
    const TfToken deformablePose_MultipleApplyTemplate_OmniphysicsPoints;
    /// \brief "deformablePose:__INSTANCE_NAME__:omniphysics:purposes"
    ///
    /// OmniUsdPhysicsDeformableSchemaDeformablePoseAPI
    const TfToken deformablePose_MultipleApplyTemplate_OmniphysicsPurposes;
    /// \brief "flatDefault"
    ///
    /// Fallback value for OmniUsdPhysicsDeformableSchemaSurfaceDeformableSimAPI::GetRestBendAnglesDefaultAttr()
    const TfToken flatDefault;
    /// \brief "omniphysics:attachmentEnabled"
    ///
    /// OmniUsdPhysicsDeformableSchemaAttachment
    const TfToken omniphysicsAttachmentEnabled;
    /// \brief "omniphysics:crvSegCoordsSrc1"
    ///
    /// OmniUsdPhysicsDeformableSchemaVtxCrvAttachment
    const TfToken omniphysicsCrvSegCoordsSrc1;
    /// \brief "omniphysics:crvSegIndicesSrc1"
    ///
    /// OmniUsdPhysicsDeformableSchemaVtxCrvAttachment
    const TfToken omniphysicsCrvSegIndicesSrc1;
    /// \brief "omniphysics:curveBendStiffness"
    ///
    /// OmniUsdPhysicsDeformableSchemaCurvesDeformableMaterialAPI
    const TfToken omniphysicsCurveBendStiffness;
    /// \brief "omniphysics:curveStretchStiffness"
    ///
    /// OmniUsdPhysicsDeformableSchemaCurvesDeformableMaterialAPI
    const TfToken omniphysicsCurveStretchStiffness;
    /// \brief "omniphysics:curveThickness"
    ///
    /// OmniUsdPhysicsDeformableSchemaCurvesDeformableMaterialAPI
    const TfToken omniphysicsCurveThickness;
    /// \brief "omniphysics:curveTwistStiffness"
    ///
    /// OmniUsdPhysicsDeformableSchemaCurvesDeformableMaterialAPI
    const TfToken omniphysicsCurveTwistStiffness;
    /// \brief "omniphysics:damping"
    ///
    /// OmniUsdPhysicsDeformableSchemaAttachment
    const TfToken omniphysicsDamping;
    /// \brief "omniphysics:deformableBodyEnabled"
    ///
    /// OmniUsdPhysicsDeformableSchemaDeformableBodyAPI
    const TfToken omniphysicsDeformableBodyEnabled;
    /// \brief "omniphysics:density"
    ///
    /// OmniUsdPhysicsDeformableSchemaBaseMaterialAPI
    const TfToken omniphysicsDensity;
    /// \brief "omniphysics:dynamicFriction"
    ///
    /// OmniUsdPhysicsDeformableSchemaBaseMaterialAPI
    const TfToken omniphysicsDynamicFriction;
    /// \brief "omniphysics:filterEnabled"
    ///
    /// OmniUsdPhysicsDeformableSchemaElementCollisionFilter
    const TfToken omniphysicsFilterEnabled;
    /// \brief "omniphysics:groupElemCounts0"
    ///
    /// OmniUsdPhysicsDeformableSchemaElementCollisionFilter
    const TfToken omniphysicsGroupElemCounts0;
    /// \brief "omniphysics:groupElemCounts1"
    ///
    /// OmniUsdPhysicsDeformableSchemaElementCollisionFilter
    const TfToken omniphysicsGroupElemCounts1;
    /// \brief "omniphysics:groupElemIndices0"
    ///
    /// OmniUsdPhysicsDeformableSchemaElementCollisionFilter
    const TfToken omniphysicsGroupElemIndices0;
    /// \brief "omniphysics:groupElemIndices1"
    ///
    /// OmniUsdPhysicsDeformableSchemaElementCollisionFilter
    const TfToken omniphysicsGroupElemIndices1;
    /// \brief "omniphysics:kinematicEnabled"
    ///
    /// OmniUsdPhysicsDeformableSchemaBodyAPI
    const TfToken omniphysicsKinematicEnabled;
    /// \brief "omniphysics:localPositionsSrc1"
    ///
    /// OmniUsdPhysicsDeformableSchemaVtxXformAttachment, OmniUsdPhysicsDeformableSchemaTetXformAttachment
    const TfToken omniphysicsLocalPositionsSrc1;
    /// \brief "omniphysics:mass"
    ///
    /// OmniUsdPhysicsDeformableSchemaDeformableBodyAPI
    const TfToken omniphysicsMass;
    /// \brief "omniphysics:poissonsRatio"
    ///
    /// OmniUsdPhysicsDeformableSchemaDeformableMaterialAPI
    const TfToken omniphysicsPoissonsRatio;
    /// \brief "omniphysics:restAdjTriPairs"
    ///
    /// OmniUsdPhysicsDeformableSchemaSurfaceDeformableSimAPI
    const TfToken omniphysicsRestAdjTriPairs;
    /// \brief "omniphysics:restBendAngles"
    ///
    /// OmniUsdPhysicsDeformableSchemaSurfaceDeformableSimAPI
    const TfToken omniphysicsRestBendAngles;
    /// \brief "omniphysics:restBendAnglesDefault"
    ///
    /// OmniUsdPhysicsDeformableSchemaSurfaceDeformableSimAPI
    const TfToken omniphysicsRestBendAnglesDefault;
    /// \brief "omniphysics:restCrvVtxIndices"
    ///
    /// OmniUsdPhysicsDeformableSchemaCurvesDeformableSimAPI
    const TfToken omniphysicsRestCrvVtxIndices;
    /// \brief "omniphysics:restNormals"
    ///
    /// OmniUsdPhysicsDeformableSchemaCurvesDeformableSimAPI
    const TfToken omniphysicsRestNormals;
    /// \brief "omniphysics:restShapePoints"
    ///
    /// OmniUsdPhysicsDeformableSchemaVolumeDeformableSimAPI, OmniUsdPhysicsDeformableSchemaSurfaceDeformableSimAPI, OmniUsdPhysicsDeformableSchemaCurvesDeformableSimAPI
    const TfToken omniphysicsRestShapePoints;
    /// \brief "omniphysics:restTetVtxIndices"
    ///
    /// OmniUsdPhysicsDeformableSchemaVolumeDeformableSimAPI
    const TfToken omniphysicsRestTetVtxIndices;
    /// \brief "omniphysics:restTriVtxIndices"
    ///
    /// OmniUsdPhysicsDeformableSchemaSurfaceDeformableSimAPI
    const TfToken omniphysicsRestTriVtxIndices;
    /// \brief "omniphysics:simulationOwner"
    ///
    /// OmniUsdPhysicsDeformableSchemaBodyAPI
    const TfToken omniphysicsSimulationOwner;
    /// \brief "omniphysics:src0"
    ///
    /// OmniUsdPhysicsDeformableSchemaAttachment, OmniUsdPhysicsDeformableSchemaElementCollisionFilter
    const TfToken omniphysicsSrc0;
    /// \brief "omniphysics:src1"
    ///
    /// OmniUsdPhysicsDeformableSchemaAttachment, OmniUsdPhysicsDeformableSchemaElementCollisionFilter
    const TfToken omniphysicsSrc1;
    /// \brief "omniphysics:startsAsleep"
    ///
    /// OmniUsdPhysicsDeformableSchemaBodyAPI
    const TfToken omniphysicsStartsAsleep;
    /// \brief "omniphysics:staticFriction"
    ///
    /// OmniUsdPhysicsDeformableSchemaBaseMaterialAPI
    const TfToken omniphysicsStaticFriction;
    /// \brief "omniphysics:stiffness"
    ///
    /// OmniUsdPhysicsDeformableSchemaAttachment
    const TfToken omniphysicsStiffness;
    /// \brief "omniphysics:surfaceBendStiffness"
    ///
    /// OmniUsdPhysicsDeformableSchemaSurfaceDeformableMaterialAPI
    const TfToken omniphysicsSurfaceBendStiffness;
    /// \brief "omniphysics:surfaceShearStiffness"
    ///
    /// OmniUsdPhysicsDeformableSchemaSurfaceDeformableMaterialAPI
    const TfToken omniphysicsSurfaceShearStiffness;
    /// \brief "omniphysics:surfaceStretchStiffness"
    ///
    /// OmniUsdPhysicsDeformableSchemaSurfaceDeformableMaterialAPI
    const TfToken omniphysicsSurfaceStretchStiffness;
    /// \brief "omniphysics:surfaceThickness"
    ///
    /// OmniUsdPhysicsDeformableSchemaSurfaceDeformableMaterialAPI
    const TfToken omniphysicsSurfaceThickness;
    /// \brief "omniphysics:tetCoordsSrc0"
    ///
    /// OmniUsdPhysicsDeformableSchemaTetXformAttachment
    const TfToken omniphysicsTetCoordsSrc0;
    /// \brief "omniphysics:tetCoordsSrc1"
    ///
    /// OmniUsdPhysicsDeformableSchemaVtxTetAttachment
    const TfToken omniphysicsTetCoordsSrc1;
    /// \brief "omniphysics:tetIndicesSrc0"
    ///
    /// OmniUsdPhysicsDeformableSchemaTetXformAttachment
    const TfToken omniphysicsTetIndicesSrc0;
    /// \brief "omniphysics:tetIndicesSrc1"
    ///
    /// OmniUsdPhysicsDeformableSchemaVtxTetAttachment
    const TfToken omniphysicsTetIndicesSrc1;
    /// \brief "omniphysics:triCoordsSrc0"
    ///
    /// OmniUsdPhysicsDeformableSchemaTriTriAttachment
    const TfToken omniphysicsTriCoordsSrc0;
    /// \brief "omniphysics:triCoordsSrc1"
    ///
    /// OmniUsdPhysicsDeformableSchemaVtxTriAttachment, OmniUsdPhysicsDeformableSchemaTriTriAttachment
    const TfToken omniphysicsTriCoordsSrc1;
    /// \brief "omniphysics:triIndicesSrc0"
    ///
    /// OmniUsdPhysicsDeformableSchemaTriTriAttachment
    const TfToken omniphysicsTriIndicesSrc0;
    /// \brief "omniphysics:triIndicesSrc1"
    ///
    /// OmniUsdPhysicsDeformableSchemaVtxTriAttachment, OmniUsdPhysicsDeformableSchemaTriTriAttachment
    const TfToken omniphysicsTriIndicesSrc1;
    /// \brief "omniphysics:vtxIndicesSrc0"
    ///
    /// OmniUsdPhysicsDeformableSchemaVtxVtxAttachment, OmniUsdPhysicsDeformableSchemaVtxTriAttachment, OmniUsdPhysicsDeformableSchemaVtxTetAttachment, OmniUsdPhysicsDeformableSchemaVtxCrvAttachment, OmniUsdPhysicsDeformableSchemaVtxXformAttachment
    const TfToken omniphysicsVtxIndicesSrc0;
    /// \brief "omniphysics:vtxIndicesSrc1"
    ///
    /// OmniUsdPhysicsDeformableSchemaVtxVtxAttachment
    const TfToken omniphysicsVtxIndicesSrc1;
    /// \brief "omniphysics:youngsModulus"
    ///
    /// OmniUsdPhysicsDeformableSchemaDeformableMaterialAPI
    const TfToken omniphysicsYoungsModulus;
    /// \brief "restShapeDefault"
    ///
    /// Possible value for OmniUsdPhysicsDeformableSchemaSurfaceDeformableSimAPI::GetRestBendAnglesDefaultAttr()
    const TfToken restShapeDefault;
    /// \brief "selfCollisionFilterPose"
    ///
    /// Possible value for OmniUsdPhysicsDeformableSchemaDeformablePoseAPI::GetPurposesAttr()
    const TfToken selfCollisionFilterPose;
    /// \brief "OmniPhysicsAttachment"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaAttachment
    const TfToken OmniPhysicsAttachment;
    /// \brief "OmniPhysicsBaseMaterialAPI"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaBaseMaterialAPI
    const TfToken OmniPhysicsBaseMaterialAPI;
    /// \brief "OmniPhysicsBodyAPI"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaBodyAPI
    const TfToken OmniPhysicsBodyAPI;
    /// \brief "OmniPhysicsCurvesDeformableMaterialAPI"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaCurvesDeformableMaterialAPI
    const TfToken OmniPhysicsCurvesDeformableMaterialAPI;
    /// \brief "OmniPhysicsCurvesDeformableSimAPI"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaCurvesDeformableSimAPI
    const TfToken OmniPhysicsCurvesDeformableSimAPI;
    /// \brief "OmniPhysicsDeformableBodyAPI"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaDeformableBodyAPI
    const TfToken OmniPhysicsDeformableBodyAPI;
    /// \brief "OmniPhysicsDeformableMaterialAPI"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaDeformableMaterialAPI
    const TfToken OmniPhysicsDeformableMaterialAPI;
    /// \brief "OmniPhysicsDeformablePoseAPI"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaDeformablePoseAPI
    const TfToken OmniPhysicsDeformablePoseAPI;
    /// \brief "OmniPhysicsElementCollisionFilter"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaElementCollisionFilter
    const TfToken OmniPhysicsElementCollisionFilter;
    /// \brief "OmniPhysicsSurfaceDeformableMaterialAPI"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaSurfaceDeformableMaterialAPI
    const TfToken OmniPhysicsSurfaceDeformableMaterialAPI;
    /// \brief "OmniPhysicsSurfaceDeformableSimAPI"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaSurfaceDeformableSimAPI
    const TfToken OmniPhysicsSurfaceDeformableSimAPI;
    /// \brief "OmniPhysicsTetXformAttachment"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaTetXformAttachment
    const TfToken OmniPhysicsTetXformAttachment;
    /// \brief "OmniPhysicsTriTriAttachment"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaTriTriAttachment
    const TfToken OmniPhysicsTriTriAttachment;
    /// \brief "OmniPhysicsVolumeDeformableSimAPI"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaVolumeDeformableSimAPI
    const TfToken OmniPhysicsVolumeDeformableSimAPI;
    /// \brief "OmniPhysicsVtxCrvAttachment"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaVtxCrvAttachment
    const TfToken OmniPhysicsVtxCrvAttachment;
    /// \brief "OmniPhysicsVtxTetAttachment"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaVtxTetAttachment
    const TfToken OmniPhysicsVtxTetAttachment;
    /// \brief "OmniPhysicsVtxTriAttachment"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaVtxTriAttachment
    const TfToken OmniPhysicsVtxTriAttachment;
    /// \brief "OmniPhysicsVtxVtxAttachment"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaVtxVtxAttachment
    const TfToken OmniPhysicsVtxVtxAttachment;
    /// \brief "OmniPhysicsVtxXformAttachment"
    ///
    /// Schema identifer and family for OmniUsdPhysicsDeformableSchemaVtxXformAttachment
    const TfToken OmniPhysicsVtxXformAttachment;
    /// A vector of all of the tokens listed above.
    const std::vector<TfToken> allTokens;
};

inline OmniUsdPhysicsDeformableSchemaTokensType::OmniUsdPhysicsDeformableSchemaTokensType() :
    bindPose("bindPose", TfToken::Immortal),
    deformablePose("deformablePose", TfToken::Immortal),
    deformablePose_MultipleApplyTemplate_OmniphysicsPoints("deformablePose:__INSTANCE_NAME__:omniphysics:points", TfToken::Immortal),
    deformablePose_MultipleApplyTemplate_OmniphysicsPurposes("deformablePose:__INSTANCE_NAME__:omniphysics:purposes", TfToken::Immortal),
    flatDefault("flatDefault", TfToken::Immortal),
    omniphysicsAttachmentEnabled("omniphysics:attachmentEnabled", TfToken::Immortal),
    omniphysicsCrvSegCoordsSrc1("omniphysics:crvSegCoordsSrc1", TfToken::Immortal),
    omniphysicsCrvSegIndicesSrc1("omniphysics:crvSegIndicesSrc1", TfToken::Immortal),
    omniphysicsCurveBendStiffness("omniphysics:curveBendStiffness", TfToken::Immortal),
    omniphysicsCurveStretchStiffness("omniphysics:curveStretchStiffness", TfToken::Immortal),
    omniphysicsCurveThickness("omniphysics:curveThickness", TfToken::Immortal),
    omniphysicsCurveTwistStiffness("omniphysics:curveTwistStiffness", TfToken::Immortal),
    omniphysicsDamping("omniphysics:damping", TfToken::Immortal),
    omniphysicsDeformableBodyEnabled("omniphysics:deformableBodyEnabled", TfToken::Immortal),
    omniphysicsDensity("omniphysics:density", TfToken::Immortal),
    omniphysicsDynamicFriction("omniphysics:dynamicFriction", TfToken::Immortal),
    omniphysicsFilterEnabled("omniphysics:filterEnabled", TfToken::Immortal),
    omniphysicsGroupElemCounts0("omniphysics:groupElemCounts0", TfToken::Immortal),
    omniphysicsGroupElemCounts1("omniphysics:groupElemCounts1", TfToken::Immortal),
    omniphysicsGroupElemIndices0("omniphysics:groupElemIndices0", TfToken::Immortal),
    omniphysicsGroupElemIndices1("omniphysics:groupElemIndices1", TfToken::Immortal),
    omniphysicsKinematicEnabled("omniphysics:kinematicEnabled", TfToken::Immortal),
    omniphysicsLocalPositionsSrc1("omniphysics:localPositionsSrc1", TfToken::Immortal),
    omniphysicsMass("omniphysics:mass", TfToken::Immortal),
    omniphysicsPoissonsRatio("omniphysics:poissonsRatio", TfToken::Immortal),
    omniphysicsRestAdjTriPairs("omniphysics:restAdjTriPairs", TfToken::Immortal),
    omniphysicsRestBendAngles("omniphysics:restBendAngles", TfToken::Immortal),
    omniphysicsRestBendAnglesDefault("omniphysics:restBendAnglesDefault", TfToken::Immortal),
    omniphysicsRestCrvVtxIndices("omniphysics:restCrvVtxIndices", TfToken::Immortal),
    omniphysicsRestNormals("omniphysics:restNormals", TfToken::Immortal),
    omniphysicsRestShapePoints("omniphysics:restShapePoints", TfToken::Immortal),
    omniphysicsRestTetVtxIndices("omniphysics:restTetVtxIndices", TfToken::Immortal),
    omniphysicsRestTriVtxIndices("omniphysics:restTriVtxIndices", TfToken::Immortal),
    omniphysicsSimulationOwner("omniphysics:simulationOwner", TfToken::Immortal),
    omniphysicsSrc0("omniphysics:src0", TfToken::Immortal),
    omniphysicsSrc1("omniphysics:src1", TfToken::Immortal),
    omniphysicsStartsAsleep("omniphysics:startsAsleep", TfToken::Immortal),
    omniphysicsStaticFriction("omniphysics:staticFriction", TfToken::Immortal),
    omniphysicsStiffness("omniphysics:stiffness", TfToken::Immortal),
    omniphysicsSurfaceBendStiffness("omniphysics:surfaceBendStiffness", TfToken::Immortal),
    omniphysicsSurfaceShearStiffness("omniphysics:surfaceShearStiffness", TfToken::Immortal),
    omniphysicsSurfaceStretchStiffness("omniphysics:surfaceStretchStiffness", TfToken::Immortal),
    omniphysicsSurfaceThickness("omniphysics:surfaceThickness", TfToken::Immortal),
    omniphysicsTetCoordsSrc0("omniphysics:tetCoordsSrc0", TfToken::Immortal),
    omniphysicsTetCoordsSrc1("omniphysics:tetCoordsSrc1", TfToken::Immortal),
    omniphysicsTetIndicesSrc0("omniphysics:tetIndicesSrc0", TfToken::Immortal),
    omniphysicsTetIndicesSrc1("omniphysics:tetIndicesSrc1", TfToken::Immortal),
    omniphysicsTriCoordsSrc0("omniphysics:triCoordsSrc0", TfToken::Immortal),
    omniphysicsTriCoordsSrc1("omniphysics:triCoordsSrc1", TfToken::Immortal),
    omniphysicsTriIndicesSrc0("omniphysics:triIndicesSrc0", TfToken::Immortal),
    omniphysicsTriIndicesSrc1("omniphysics:triIndicesSrc1", TfToken::Immortal),
    omniphysicsVtxIndicesSrc0("omniphysics:vtxIndicesSrc0", TfToken::Immortal),
    omniphysicsVtxIndicesSrc1("omniphysics:vtxIndicesSrc1", TfToken::Immortal),
    omniphysicsYoungsModulus("omniphysics:youngsModulus", TfToken::Immortal),
    restShapeDefault("restShapeDefault", TfToken::Immortal),
    selfCollisionFilterPose("selfCollisionFilterPose", TfToken::Immortal),
    OmniPhysicsAttachment("OmniPhysicsAttachment", TfToken::Immortal),
    OmniPhysicsBaseMaterialAPI("OmniPhysicsBaseMaterialAPI", TfToken::Immortal),
    OmniPhysicsBodyAPI("OmniPhysicsBodyAPI", TfToken::Immortal),
    OmniPhysicsCurvesDeformableMaterialAPI("OmniPhysicsCurvesDeformableMaterialAPI", TfToken::Immortal),
    OmniPhysicsCurvesDeformableSimAPI("OmniPhysicsCurvesDeformableSimAPI", TfToken::Immortal),
    OmniPhysicsDeformableBodyAPI("OmniPhysicsDeformableBodyAPI", TfToken::Immortal),
    OmniPhysicsDeformableMaterialAPI("OmniPhysicsDeformableMaterialAPI", TfToken::Immortal),
    OmniPhysicsDeformablePoseAPI("OmniPhysicsDeformablePoseAPI", TfToken::Immortal),
    OmniPhysicsElementCollisionFilter("OmniPhysicsElementCollisionFilter", TfToken::Immortal),
    OmniPhysicsSurfaceDeformableMaterialAPI("OmniPhysicsSurfaceDeformableMaterialAPI", TfToken::Immortal),
    OmniPhysicsSurfaceDeformableSimAPI("OmniPhysicsSurfaceDeformableSimAPI", TfToken::Immortal),
    OmniPhysicsTetXformAttachment("OmniPhysicsTetXformAttachment", TfToken::Immortal),
    OmniPhysicsTriTriAttachment("OmniPhysicsTriTriAttachment", TfToken::Immortal),
    OmniPhysicsVolumeDeformableSimAPI("OmniPhysicsVolumeDeformableSimAPI", TfToken::Immortal),
    OmniPhysicsVtxCrvAttachment("OmniPhysicsVtxCrvAttachment", TfToken::Immortal),
    OmniPhysicsVtxTetAttachment("OmniPhysicsVtxTetAttachment", TfToken::Immortal),
    OmniPhysicsVtxTriAttachment("OmniPhysicsVtxTriAttachment", TfToken::Immortal),
    OmniPhysicsVtxVtxAttachment("OmniPhysicsVtxVtxAttachment", TfToken::Immortal),
    OmniPhysicsVtxXformAttachment("OmniPhysicsVtxXformAttachment", TfToken::Immortal),
    allTokens({
        bindPose,
        deformablePose,
        deformablePose_MultipleApplyTemplate_OmniphysicsPoints,
        deformablePose_MultipleApplyTemplate_OmniphysicsPurposes,
        flatDefault,
        omniphysicsAttachmentEnabled,
        omniphysicsCrvSegCoordsSrc1,
        omniphysicsCrvSegIndicesSrc1,
        omniphysicsCurveBendStiffness,
        omniphysicsCurveStretchStiffness,
        omniphysicsCurveThickness,
        omniphysicsCurveTwistStiffness,
        omniphysicsDamping,
        omniphysicsDeformableBodyEnabled,
        omniphysicsDensity,
        omniphysicsDynamicFriction,
        omniphysicsFilterEnabled,
        omniphysicsGroupElemCounts0,
        omniphysicsGroupElemCounts1,
        omniphysicsGroupElemIndices0,
        omniphysicsGroupElemIndices1,
        omniphysicsKinematicEnabled,
        omniphysicsLocalPositionsSrc1,
        omniphysicsMass,
        omniphysicsPoissonsRatio,
        omniphysicsRestAdjTriPairs,
        omniphysicsRestBendAngles,
        omniphysicsRestBendAnglesDefault,
        omniphysicsRestCrvVtxIndices,
        omniphysicsRestNormals,
        omniphysicsRestShapePoints,
        omniphysicsRestTetVtxIndices,
        omniphysicsRestTriVtxIndices,
        omniphysicsSimulationOwner,
        omniphysicsSrc0,
        omniphysicsSrc1,
        omniphysicsStartsAsleep,
        omniphysicsStaticFriction,
        omniphysicsStiffness,
        omniphysicsSurfaceBendStiffness,
        omniphysicsSurfaceShearStiffness,
        omniphysicsSurfaceStretchStiffness,
        omniphysicsSurfaceThickness,
        omniphysicsTetCoordsSrc0,
        omniphysicsTetCoordsSrc1,
        omniphysicsTetIndicesSrc0,
        omniphysicsTetIndicesSrc1,
        omniphysicsTriCoordsSrc0,
        omniphysicsTriCoordsSrc1,
        omniphysicsTriIndicesSrc0,
        omniphysicsTriIndicesSrc1,
        omniphysicsVtxIndicesSrc0,
        omniphysicsVtxIndicesSrc1,
        omniphysicsYoungsModulus,
        restShapeDefault,
        selfCollisionFilterPose,
        OmniPhysicsAttachment,
        OmniPhysicsBaseMaterialAPI,
        OmniPhysicsBodyAPI,
        OmniPhysicsCurvesDeformableMaterialAPI,
        OmniPhysicsCurvesDeformableSimAPI,
        OmniPhysicsDeformableBodyAPI,
        OmniPhysicsDeformableMaterialAPI,
        OmniPhysicsDeformablePoseAPI,
        OmniPhysicsElementCollisionFilter,
        OmniPhysicsSurfaceDeformableMaterialAPI,
        OmniPhysicsSurfaceDeformableSimAPI,
        OmniPhysicsTetXformAttachment,
        OmniPhysicsTriTriAttachment,
        OmniPhysicsVolumeDeformableSimAPI,
        OmniPhysicsVtxCrvAttachment,
        OmniPhysicsVtxTetAttachment,
        OmniPhysicsVtxTriAttachment,
        OmniPhysicsVtxVtxAttachment,
        OmniPhysicsVtxXformAttachment,
    })
{
}

/// \var OmniUsdPhysicsDeformableSchemaTokens
///
/// A global variable with static, efficient \link TfToken TfTokens\endlink
/// for use in all public USD API.  \sa OmniUsdPhysicsDeformableSchemaTokensType
///
/// Codeless: defined inline (header-only) -- no compiled tokens.cpp / library.
inline TfStaticData<OmniUsdPhysicsDeformableSchemaTokensType> OmniUsdPhysicsDeformableSchemaTokens;

PXR_NAMESPACE_CLOSE_SCOPE

#endif
