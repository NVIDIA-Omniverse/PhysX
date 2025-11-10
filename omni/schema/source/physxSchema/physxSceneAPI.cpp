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
#include ".//physxSceneAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxSceneAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxSceneAPI::~PhysxSchemaPhysxSceneAPI()
{
}

/* static */
PhysxSchemaPhysxSceneAPI
PhysxSchemaPhysxSceneAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxSceneAPI();
    }
    return PhysxSchemaPhysxSceneAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxSceneAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxSceneAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxSceneAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxSceneAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxSceneAPI
PhysxSchemaPhysxSceneAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxSceneAPI>()) {
        return PhysxSchemaPhysxSceneAPI(prim);
    }
    return PhysxSchemaPhysxSceneAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxSceneAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxSceneAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxSceneAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxSceneAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetBounceThresholdAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneBounceThreshold);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateBounceThresholdAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneBounceThreshold,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetFrictionOffsetThresholdAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneFrictionOffsetThreshold);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateFrictionOffsetThresholdAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneFrictionOffsetThreshold,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetFrictionCorrelationDistanceAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneFrictionCorrelationDistance);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateFrictionCorrelationDistanceAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneFrictionCorrelationDistance,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetMaxBiasCoefficientAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneMaxBiasCoefficient);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateMaxBiasCoefficientAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneMaxBiasCoefficient,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetCollisionSystemAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneCollisionSystem);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateCollisionSystemAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneCollisionSystem,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetSolverTypeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneSolverType);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateSolverTypeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneSolverType,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetBroadphaseTypeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneBroadphaseType);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateBroadphaseTypeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneBroadphaseType,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetFrictionTypeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneFrictionType);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateFrictionTypeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneFrictionType,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetEnableCCDAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneEnableCCD);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateEnableCCDAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneEnableCCD,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetEnableStabilizationAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneEnableStabilization);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateEnableStabilizationAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneEnableStabilization,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetUpdateTypeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneUpdateType);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateUpdateTypeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneUpdateType,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetEnableGPUDynamicsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneEnableGPUDynamics);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateEnableGPUDynamicsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneEnableGPUDynamics,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetEnableEnhancedDeterminismAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneEnableEnhancedDeterminism);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateEnableEnhancedDeterminismAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneEnableEnhancedDeterminism,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetEnableResidualReportingAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneEnableResidualReporting);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateEnableResidualReportingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneEnableResidualReporting,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetEnableExternalForcesEveryIterationAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneEnableExternalForcesEveryIteration);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateEnableExternalForcesEveryIterationAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneEnableExternalForcesEveryIteration,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetEnableSceneQuerySupportAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneEnableSceneQuerySupport);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateEnableSceneQuerySupportAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneEnableSceneQuerySupport,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetTimeStepsPerSecondAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneTimeStepsPerSecond);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateTimeStepsPerSecondAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneTimeStepsPerSecond,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetGpuTempBufferCapacityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneGpuTempBufferCapacity);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateGpuTempBufferCapacityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneGpuTempBufferCapacity,
                       SdfValueTypeNames->UInt64,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetGpuMaxRigidContactCountAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneGpuMaxRigidContactCount);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateGpuMaxRigidContactCountAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneGpuMaxRigidContactCount,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetGpuMaxRigidPatchCountAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneGpuMaxRigidPatchCount);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateGpuMaxRigidPatchCountAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneGpuMaxRigidPatchCount,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetGpuHeapCapacityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneGpuHeapCapacity);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateGpuHeapCapacityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneGpuHeapCapacity,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetGpuFoundLostPairsCapacityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneGpuFoundLostPairsCapacity);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateGpuFoundLostPairsCapacityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneGpuFoundLostPairsCapacity,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetGpuFoundLostAggregatePairsCapacityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneGpuFoundLostAggregatePairsCapacity);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateGpuFoundLostAggregatePairsCapacityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneGpuFoundLostAggregatePairsCapacity,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetGpuTotalAggregatePairsCapacityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneGpuTotalAggregatePairsCapacity);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateGpuTotalAggregatePairsCapacityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneGpuTotalAggregatePairsCapacity,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetGpuMaxSoftBodyContactsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneGpuMaxSoftBodyContacts);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateGpuMaxSoftBodyContactsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneGpuMaxSoftBodyContacts,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetGpuMaxDeformableSurfaceContactsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneGpuMaxDeformableSurfaceContacts);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateGpuMaxDeformableSurfaceContactsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneGpuMaxDeformableSurfaceContacts,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetGpuMaxParticleContactsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneGpuMaxParticleContacts);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateGpuMaxParticleContactsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneGpuMaxParticleContacts,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetGpuMaxNumPartitionsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneGpuMaxNumPartitions);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateGpuMaxNumPartitionsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneGpuMaxNumPartitions,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetGpuCollisionStackSizeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneGpuCollisionStackSize);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateGpuCollisionStackSizeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneGpuCollisionStackSize,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetInvertCollisionGroupFilterAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneInvertCollisionGroupFilter);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateInvertCollisionGroupFilterAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneInvertCollisionGroupFilter,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetReportKinematicKinematicPairsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneReportKinematicKinematicPairs);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateReportKinematicKinematicPairsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneReportKinematicKinematicPairs,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetReportKinematicStaticPairsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneReportKinematicStaticPairs);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateReportKinematicStaticPairsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneReportKinematicStaticPairs,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetMinPositionIterationCountAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneMinPositionIterationCount);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateMinPositionIterationCountAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneMinPositionIterationCount,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetMaxPositionIterationCountAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneMaxPositionIterationCount);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateMaxPositionIterationCountAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneMaxPositionIterationCount,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetMinVelocityIterationCountAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneMinVelocityIterationCount);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateMinVelocityIterationCountAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneMinVelocityIterationCount,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::GetMaxVelocityIterationCountAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSceneMaxVelocityIterationCount);
}

UsdAttribute
PhysxSchemaPhysxSceneAPI::CreateMaxVelocityIterationCountAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSceneMaxVelocityIterationCount,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

namespace {
static inline TfTokenVector
_ConcatenateAttributeNames(const TfTokenVector& left,const TfTokenVector& right)
{
    TfTokenVector result;
    result.reserve(left.size() + right.size());
    result.insert(result.end(), left.begin(), left.end());
    result.insert(result.end(), right.begin(), right.end());
    return result;
}
}

/*static*/
const TfTokenVector&
PhysxSchemaPhysxSceneAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxSceneBounceThreshold,
        PhysxSchemaTokens->physxSceneFrictionOffsetThreshold,
        PhysxSchemaTokens->physxSceneFrictionCorrelationDistance,
        PhysxSchemaTokens->physxSceneMaxBiasCoefficient,
        PhysxSchemaTokens->physxSceneCollisionSystem,
        PhysxSchemaTokens->physxSceneSolverType,
        PhysxSchemaTokens->physxSceneBroadphaseType,
        PhysxSchemaTokens->physxSceneFrictionType,
        PhysxSchemaTokens->physxSceneEnableCCD,
        PhysxSchemaTokens->physxSceneEnableStabilization,
        PhysxSchemaTokens->physxSceneUpdateType,
        PhysxSchemaTokens->physxSceneEnableGPUDynamics,
        PhysxSchemaTokens->physxSceneEnableEnhancedDeterminism,
        PhysxSchemaTokens->physxSceneEnableResidualReporting,
        PhysxSchemaTokens->physxSceneEnableExternalForcesEveryIteration,
        PhysxSchemaTokens->physxSceneEnableSceneQuerySupport,
        PhysxSchemaTokens->physxSceneTimeStepsPerSecond,
        PhysxSchemaTokens->physxSceneGpuTempBufferCapacity,
        PhysxSchemaTokens->physxSceneGpuMaxRigidContactCount,
        PhysxSchemaTokens->physxSceneGpuMaxRigidPatchCount,
        PhysxSchemaTokens->physxSceneGpuHeapCapacity,
        PhysxSchemaTokens->physxSceneGpuFoundLostPairsCapacity,
        PhysxSchemaTokens->physxSceneGpuFoundLostAggregatePairsCapacity,
        PhysxSchemaTokens->physxSceneGpuTotalAggregatePairsCapacity,
        PhysxSchemaTokens->physxSceneGpuMaxSoftBodyContacts,
        PhysxSchemaTokens->physxSceneGpuMaxDeformableSurfaceContacts,
        PhysxSchemaTokens->physxSceneGpuMaxParticleContacts,
        PhysxSchemaTokens->physxSceneGpuMaxNumPartitions,
        PhysxSchemaTokens->physxSceneGpuCollisionStackSize,
        PhysxSchemaTokens->physxSceneInvertCollisionGroupFilter,
        PhysxSchemaTokens->physxSceneReportKinematicKinematicPairs,
        PhysxSchemaTokens->physxSceneReportKinematicStaticPairs,
        PhysxSchemaTokens->physxSceneMinPositionIterationCount,
        PhysxSchemaTokens->physxSceneMaxPositionIterationCount,
        PhysxSchemaTokens->physxSceneMinVelocityIterationCount,
        PhysxSchemaTokens->physxSceneMaxVelocityIterationCount,
    };
    static TfTokenVector allNames =
        _ConcatenateAttributeNames(
            UsdAPISchemaBase::GetSchemaAttributeNames(true),
            localNames);

    if (includeInherited)
        return allNames;
    else
        return localNames;
}

PXR_NAMESPACE_CLOSE_SCOPE

// ===================================================================== //
// Feel free to add custom code below this line. It will be preserved by
// the code generator.
//
// Just remember to wrap code in the appropriate delimiters:
// 'PXR_NAMESPACE_OPEN_SCOPE', 'PXR_NAMESPACE_CLOSE_SCOPE'.
// ===================================================================== //
// --(BEGIN CUSTOM CODE)--
