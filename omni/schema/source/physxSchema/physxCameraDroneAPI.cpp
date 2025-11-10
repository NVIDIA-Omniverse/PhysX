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
#include ".//physxCameraDroneAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxCameraDroneAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxCameraDroneAPI::~PhysxSchemaPhysxCameraDroneAPI()
{
}

/* static */
PhysxSchemaPhysxCameraDroneAPI
PhysxSchemaPhysxCameraDroneAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxCameraDroneAPI();
    }
    return PhysxSchemaPhysxCameraDroneAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxCameraDroneAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxCameraDroneAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxCameraDroneAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxCameraDroneAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxCameraDroneAPI
PhysxSchemaPhysxCameraDroneAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxCameraDroneAPI>()) {
        return PhysxSchemaPhysxCameraDroneAPI(prim);
    }
    return PhysxSchemaPhysxCameraDroneAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxCameraDroneAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxCameraDroneAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxCameraDroneAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxCameraDroneAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::GetFollowHeightAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDroneCameraFollowHeight);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::CreateFollowHeightAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDroneCameraFollowHeight,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::GetFollowDistanceAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDroneCameraFollowDistance);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::CreateFollowDistanceAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDroneCameraFollowDistance,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::GetMaxDistanceAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDroneCameraMaxDistance);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::CreateMaxDistanceAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDroneCameraMaxDistance,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::GetMaxSpeedAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDroneCameraMaxSpeed);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::CreateMaxSpeedAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDroneCameraMaxSpeed,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::GetHorizontalVelocityGainAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDroneCameraHorizontalVelocityGain);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::CreateHorizontalVelocityGainAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDroneCameraHorizontalVelocityGain,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::GetVerticalVelocityGainAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDroneCameraVerticalVelocityGain);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::CreateVerticalVelocityGainAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDroneCameraVerticalVelocityGain,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::GetFeedForwardVelocityGainAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDroneCameraFeedForwardVelocityGain);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::CreateFeedForwardVelocityGainAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDroneCameraFeedForwardVelocityGain,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::GetVelocityFilterTimeConstantAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDroneCameraVelocityFilterTimeConstant);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::CreateVelocityFilterTimeConstantAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDroneCameraVelocityFilterTimeConstant,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::GetRotationFilterTimeConstantAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDroneCameraRotationFilterTimeConstant);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::CreateRotationFilterTimeConstantAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDroneCameraRotationFilterTimeConstant,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::GetPositionOffsetAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDroneCameraPositionOffset);
}

UsdAttribute
PhysxSchemaPhysxCameraDroneAPI::CreatePositionOffsetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDroneCameraPositionOffset,
                       SdfValueTypeNames->Float3,
                       /* custom = */ false,
                       SdfVariabilityVarying,
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
PhysxSchemaPhysxCameraDroneAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxDroneCameraFollowHeight,
        PhysxSchemaTokens->physxDroneCameraFollowDistance,
        PhysxSchemaTokens->physxDroneCameraMaxDistance,
        PhysxSchemaTokens->physxDroneCameraMaxSpeed,
        PhysxSchemaTokens->physxDroneCameraHorizontalVelocityGain,
        PhysxSchemaTokens->physxDroneCameraVerticalVelocityGain,
        PhysxSchemaTokens->physxDroneCameraFeedForwardVelocityGain,
        PhysxSchemaTokens->physxDroneCameraVelocityFilterTimeConstant,
        PhysxSchemaTokens->physxDroneCameraRotationFilterTimeConstant,
        PhysxSchemaTokens->physxDroneCameraPositionOffset,
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
