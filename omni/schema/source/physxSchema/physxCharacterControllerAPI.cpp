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
#include ".//physxCharacterControllerAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxCharacterControllerAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxCharacterControllerAPI::~PhysxSchemaPhysxCharacterControllerAPI()
{
}

/* static */
PhysxSchemaPhysxCharacterControllerAPI
PhysxSchemaPhysxCharacterControllerAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxCharacterControllerAPI();
    }
    return PhysxSchemaPhysxCharacterControllerAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxCharacterControllerAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxCharacterControllerAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxCharacterControllerAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxCharacterControllerAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxCharacterControllerAPI
PhysxSchemaPhysxCharacterControllerAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxCharacterControllerAPI>()) {
        return PhysxSchemaPhysxCharacterControllerAPI(prim);
    }
    return PhysxSchemaPhysxCharacterControllerAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxCharacterControllerAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxCharacterControllerAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxCharacterControllerAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxCharacterControllerAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::GetSlopeLimitAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxCharacterControllerSlopeLimit);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::CreateSlopeLimitAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxCharacterControllerSlopeLimit,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::GetMoveTargetAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxCharacterControllerMoveTarget);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::CreateMoveTargetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxCharacterControllerMoveTarget,
                       SdfValueTypeNames->Vector3f,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::GetUpAxisAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxCharacterControllerUpAxis);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::CreateUpAxisAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxCharacterControllerUpAxis,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::GetNonWalkableModeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxCharacterControllerNonWalkableMode);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::CreateNonWalkableModeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxCharacterControllerNonWalkableMode,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::GetClimbingModeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxCharacterControllerClimbingMode);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::CreateClimbingModeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxCharacterControllerClimbingMode,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::GetInvisibleWallHeightAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxCharacterControllerInvisibleWallHeight);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::CreateInvisibleWallHeightAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxCharacterControllerInvisibleWallHeight,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::GetMaxJumpHeightAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxCharacterControllerMaxJumpHeight);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::CreateMaxJumpHeightAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxCharacterControllerMaxJumpHeight,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::GetContactOffsetAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxCharacterControllerContactOffset);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::CreateContactOffsetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxCharacterControllerContactOffset,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::GetStepOffsetAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxCharacterControllerStepOffset);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::CreateStepOffsetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxCharacterControllerStepOffset,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::GetScaleCoeffAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxCharacterControllerScaleCoeff);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::CreateScaleCoeffAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxCharacterControllerScaleCoeff,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::GetVolumeGrowthAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxCharacterControllerVolumeGrowth);
}

UsdAttribute
PhysxSchemaPhysxCharacterControllerAPI::CreateVolumeGrowthAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxCharacterControllerVolumeGrowth,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdRelationship
PhysxSchemaPhysxCharacterControllerAPI::GetSimulationOwnerRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physxCharacterControllerSimulationOwner);
}

UsdRelationship
PhysxSchemaPhysxCharacterControllerAPI::CreateSimulationOwnerRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physxCharacterControllerSimulationOwner,
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
PhysxSchemaPhysxCharacterControllerAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxCharacterControllerSlopeLimit,
        PhysxSchemaTokens->physxCharacterControllerMoveTarget,
        PhysxSchemaTokens->physxCharacterControllerUpAxis,
        PhysxSchemaTokens->physxCharacterControllerNonWalkableMode,
        PhysxSchemaTokens->physxCharacterControllerClimbingMode,
        PhysxSchemaTokens->physxCharacterControllerInvisibleWallHeight,
        PhysxSchemaTokens->physxCharacterControllerMaxJumpHeight,
        PhysxSchemaTokens->physxCharacterControllerContactOffset,
        PhysxSchemaTokens->physxCharacterControllerStepOffset,
        PhysxSchemaTokens->physxCharacterControllerScaleCoeff,
        PhysxSchemaTokens->physxCharacterControllerVolumeGrowth,
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
