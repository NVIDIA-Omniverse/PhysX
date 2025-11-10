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
#include ".//physxCameraFollowLookAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxCameraFollowLookAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxCameraFollowLookAPI::~PhysxSchemaPhysxCameraFollowLookAPI()
{
}

/* static */
PhysxSchemaPhysxCameraFollowLookAPI
PhysxSchemaPhysxCameraFollowLookAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxCameraFollowLookAPI();
    }
    return PhysxSchemaPhysxCameraFollowLookAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxCameraFollowLookAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxCameraFollowLookAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxCameraFollowLookAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxCameraFollowLookAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxCameraFollowLookAPI
PhysxSchemaPhysxCameraFollowLookAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxCameraFollowLookAPI>()) {
        return PhysxSchemaPhysxCameraFollowLookAPI(prim);
    }
    return PhysxSchemaPhysxCameraFollowLookAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxCameraFollowLookAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxCameraFollowLookAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxCameraFollowLookAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxCameraFollowLookAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxCameraFollowLookAPI::GetDownHillGroundAngleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowLookCameraDownHillGroundAngle);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowLookAPI::CreateDownHillGroundAngleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowLookCameraDownHillGroundAngle,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowLookAPI::GetDownHillGroundPitchAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowLookCameraDownHillGroundPitch);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowLookAPI::CreateDownHillGroundPitchAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowLookCameraDownHillGroundPitch,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowLookAPI::GetUpHillGroundAngleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowLookCameraUpHillGroundAngle);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowLookAPI::CreateUpHillGroundAngleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowLookCameraUpHillGroundAngle,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowLookAPI::GetUpHillGroundPitchAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowLookCameraUpHillGroundPitch);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowLookAPI::CreateUpHillGroundPitchAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowLookCameraUpHillGroundPitch,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowLookAPI::GetVelocityBlendTimeConstantAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowLookCameraVelocityBlendTimeConstant);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowLookAPI::CreateVelocityBlendTimeConstantAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowLookCameraVelocityBlendTimeConstant,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowLookAPI::GetFollowReverseSpeedAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowLookCameraFollowReverseSpeed);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowLookAPI::CreateFollowReverseSpeedAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowLookCameraFollowReverseSpeed,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowLookAPI::GetFollowReverseDistanceAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxFollowLookCameraFollowReverseDistance);
}

UsdAttribute
PhysxSchemaPhysxCameraFollowLookAPI::CreateFollowReverseDistanceAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxFollowLookCameraFollowReverseDistance,
                       SdfValueTypeNames->Float,
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
PhysxSchemaPhysxCameraFollowLookAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxFollowLookCameraDownHillGroundAngle,
        PhysxSchemaTokens->physxFollowLookCameraDownHillGroundPitch,
        PhysxSchemaTokens->physxFollowLookCameraUpHillGroundAngle,
        PhysxSchemaTokens->physxFollowLookCameraUpHillGroundPitch,
        PhysxSchemaTokens->physxFollowLookCameraVelocityBlendTimeConstant,
        PhysxSchemaTokens->physxFollowLookCameraFollowReverseSpeed,
        PhysxSchemaTokens->physxFollowLookCameraFollowReverseDistance,
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
