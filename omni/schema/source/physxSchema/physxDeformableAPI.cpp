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
#include ".//physxDeformableAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxDeformableAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxDeformableAPI::~PhysxSchemaPhysxDeformableAPI()
{
}

/* static */
PhysxSchemaPhysxDeformableAPI
PhysxSchemaPhysxDeformableAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxDeformableAPI();
    }
    return PhysxSchemaPhysxDeformableAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxDeformableAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxDeformableAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxDeformableAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxDeformableAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxDeformableAPI
PhysxSchemaPhysxDeformableAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxDeformableAPI>()) {
        return PhysxSchemaPhysxDeformableAPI(prim);
    }
    return PhysxSchemaPhysxDeformableAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxDeformableAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxDeformableAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxDeformableAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxDeformableAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::GetDeformableEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableDeformableEnabled);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::CreateDeformableEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableDeformableEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::GetSolverPositionIterationCountAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSolverPositionIterationCount);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::CreateSolverPositionIterationCountAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSolverPositionIterationCount,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::GetVertexVelocityDampingAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableVertexVelocityDamping);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::CreateVertexVelocityDampingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableVertexVelocityDamping,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::GetSleepDampingAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSleepDamping);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::CreateSleepDampingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSleepDamping,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::GetSleepThresholdAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSleepThreshold);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::CreateSleepThresholdAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSleepThreshold,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::GetSettlingThresholdAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSettlingThreshold);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::CreateSettlingThresholdAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSettlingThreshold,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::GetMaxDepenetrationVelocityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableMaxDepenetrationVelocity);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::CreateMaxDepenetrationVelocityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableMaxDepenetrationVelocity,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::GetSelfCollisionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSelfCollision);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::CreateSelfCollisionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSelfCollision,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::GetSelfCollisionFilterDistanceAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSelfCollisionFilterDistance);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::CreateSelfCollisionFilterDistanceAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSelfCollisionFilterDistance,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::GetEnableCCDAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableEnableCCD);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::CreateEnableCCDAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableEnableCCD,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::GetRestPointsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableRestPoints);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::CreateRestPointsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableRestPoints,
                       SdfValueTypeNames->Point3fArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::GetSimulationVelocitiesAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSimulationVelocities);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::CreateSimulationVelocitiesAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSimulationVelocities,
                       SdfValueTypeNames->Point3fArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::GetSimulationIndicesAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSimulationIndices);
}

UsdAttribute
PhysxSchemaPhysxDeformableAPI::CreateSimulationIndicesAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSimulationIndices,
                       SdfValueTypeNames->IntArray,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdRelationship
PhysxSchemaPhysxDeformableAPI::GetSimulationOwnerRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physxDeformableSimulationOwner);
}

UsdRelationship
PhysxSchemaPhysxDeformableAPI::CreateSimulationOwnerRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physxDeformableSimulationOwner,
                       /* custom = */ false);
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
PhysxSchemaPhysxDeformableAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxDeformableDeformableEnabled,
        PhysxSchemaTokens->physxDeformableSolverPositionIterationCount,
        PhysxSchemaTokens->physxDeformableVertexVelocityDamping,
        PhysxSchemaTokens->physxDeformableSleepDamping,
        PhysxSchemaTokens->physxDeformableSleepThreshold,
        PhysxSchemaTokens->physxDeformableSettlingThreshold,
        PhysxSchemaTokens->physxDeformableMaxDepenetrationVelocity,
        PhysxSchemaTokens->physxDeformableSelfCollision,
        PhysxSchemaTokens->physxDeformableSelfCollisionFilterDistance,
        PhysxSchemaTokens->physxDeformableEnableCCD,
        PhysxSchemaTokens->physxDeformableRestPoints,
        PhysxSchemaTokens->physxDeformableSimulationVelocities,
        PhysxSchemaTokens->physxDeformableSimulationIndices,
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
