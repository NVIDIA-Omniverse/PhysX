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
#include ".//physxCameraFollowAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxCameraFollowAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxCameraFollowAPI::~PhysxSchemaPhysxCameraFollowAPI()
{
}

/* static */
PhysxSchemaPhysxCameraFollowAPI
PhysxSchemaPhysxCameraFollowAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxCameraFollowAPI();
    }
    return PhysxSchemaPhysxCameraFollowAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxCameraFollowAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxCameraFollowAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxCameraFollowAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxCameraFollowAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxCameraFollowAPI
PhysxSchemaPhysxCameraFollowAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxCameraFollowAPI>()) {
        return PhysxSchemaPhysxCameraFollowAPI(prim);
    }
    return PhysxSchemaPhysxCameraFollowAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxCameraFollowAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxCameraFollowAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxCameraFollowAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxCameraFollowAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetYawAngleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraYawAngle);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateYawAngleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraYawAngle,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetPitchAngleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraPitchAngle);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreatePitchAngleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraPitchAngle,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetPitchAngleTimeConstantAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraPitchAngleTimeConstant);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreatePitchAngleTimeConstantAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraPitchAngleTimeConstant,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetSlowSpeedPitchAngleScaleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraSlowSpeedPitchAngleScale);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateSlowSpeedPitchAngleScaleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraSlowSpeedPitchAngleScale,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetSlowPitchAngleSpeedAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraSlowPitchAngleSpeed);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateSlowPitchAngleSpeedAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraSlowPitchAngleSpeed,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetVelocityNormalMinSpeedAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraVelocityNormalMinSpeed);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateVelocityNormalMinSpeedAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraVelocityNormalMinSpeed,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetFollowMinSpeedAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraFollowMinSpeed);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateFollowMinSpeedAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraFollowMinSpeed,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetFollowMinDistanceAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraFollowMinDistance);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateFollowMinDistanceAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraFollowMinDistance,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetFollowMaxSpeedAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraFollowMaxSpeed);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateFollowMaxSpeedAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraFollowMaxSpeed,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetFollowMaxDistanceAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraFollowMaxDistance);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateFollowMaxDistanceAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraFollowMaxDistance,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetYawRateTimeConstantAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraYawRateTimeConstant);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateYawRateTimeConstantAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraYawRateTimeConstant,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetFollowTurnRateGainAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraFollowTurnRateGain);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateFollowTurnRateGainAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraFollowTurnRateGain,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetCameraPositionTimeConstantAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraCameraPositionTimeConstant);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateCameraPositionTimeConstantAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraCameraPositionTimeConstant,
                       SdfValueTypeNames->Float3,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetPositionOffsetAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraPositionOffset);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreatePositionOffsetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraPositionOffset,
                       SdfValueTypeNames->Float3,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetLookAheadMinSpeedAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraLookAheadMinSpeed);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateLookAheadMinSpeedAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraLookAheadMinSpeed,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetLookAheadMinDistanceAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraLookAheadMinDistance);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateLookAheadMinDistanceAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraLookAheadMinDistance,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetLookAheadMaxSpeedAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraLookAheadMaxSpeed);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateLookAheadMaxSpeedAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraLookAheadMaxSpeed,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetLookAheadMaxDistanceAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowFollowCameraLookAheadMaxDistance);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateLookAheadMaxDistanceAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowFollowCameraLookAheadMaxDistance,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetLookAheadTurnRateGainAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraLookAheadTurnRateGain);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateLookAheadTurnRateGainAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraLookAheadTurnRateGain,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetLookPositionHeightAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraLookPositionHeight);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateLookPositionHeightAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraLookPositionHeight,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::GetLookPositionTimeConstantAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowCameraLookPositionTimeConstant);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowAPI::CreateLookPositionTimeConstantAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowCameraLookPositionTimeConstant,
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
PhysxSchemaPhysxCameraFollowAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxFollowCameraYawAngle,
        PhysxSchemaTokens->physxFollowCameraPitchAngle,
        PhysxSchemaTokens->physxFollowCameraPitchAngleTimeConstant,
        PhysxSchemaTokens->physxFollowCameraSlowSpeedPitchAngleScale,
        PhysxSchemaTokens->physxFollowCameraSlowPitchAngleSpeed,
        PhysxSchemaTokens->physxFollowCameraVelocityNormalMinSpeed,
        PhysxSchemaTokens->physxFollowCameraFollowMinSpeed,
        PhysxSchemaTokens->physxFollowCameraFollowMinDistance,
        PhysxSchemaTokens->physxFollowCameraFollowMaxSpeed,
        PhysxSchemaTokens->physxFollowCameraFollowMaxDistance,
        PhysxSchemaTokens->physxFollowCameraYawRateTimeConstant,
        PhysxSchemaTokens->physxFollowCameraFollowTurnRateGain,
        PhysxSchemaTokens->physxFollowCameraCameraPositionTimeConstant,
        PhysxSchemaTokens->physxFollowCameraPositionOffset,
        PhysxSchemaTokens->physxFollowCameraLookAheadMinSpeed,
        PhysxSchemaTokens->physxFollowCameraLookAheadMinDistance,
        PhysxSchemaTokens->physxFollowCameraLookAheadMaxSpeed,
        PhysxSchemaTokens->physxFollowFollowCameraLookAheadMaxDistance,
        PhysxSchemaTokens->physxFollowCameraLookAheadTurnRateGain,
        PhysxSchemaTokens->physxFollowCameraLookPositionHeight,
        PhysxSchemaTokens->physxFollowCameraLookPositionTimeConstant,
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
