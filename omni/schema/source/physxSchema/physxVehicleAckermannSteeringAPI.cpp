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
#include ".//physxVehicleAckermannSteeringAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleAckermannSteeringAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxVehicleAckermannSteeringAPI::~PhysxSchemaPhysxVehicleAckermannSteeringAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleAckermannSteeringAPI
PhysxSchemaPhysxVehicleAckermannSteeringAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleAckermannSteeringAPI();
    }
    return PhysxSchemaPhysxVehicleAckermannSteeringAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleAckermannSteeringAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleAckermannSteeringAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleAckermannSteeringAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleAckermannSteeringAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxVehicleAckermannSteeringAPI
PhysxSchemaPhysxVehicleAckermannSteeringAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleAckermannSteeringAPI>()) {
        return PhysxSchemaPhysxVehicleAckermannSteeringAPI(prim);
    }
    return PhysxSchemaPhysxVehicleAckermannSteeringAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleAckermannSteeringAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleAckermannSteeringAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleAckermannSteeringAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleAckermannSteeringAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxVehicleAckermannSteeringAPI::GetWheel0Attr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleAckermannSteeringWheel0);
}

UsdAttribute
PhysxSchemaPhysxVehicleAckermannSteeringAPI::CreateWheel0Attr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleAckermannSteeringWheel0,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAckermannSteeringAPI::GetWheel1Attr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleAckermannSteeringWheel1);
}

UsdAttribute
PhysxSchemaPhysxVehicleAckermannSteeringAPI::CreateWheel1Attr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleAckermannSteeringWheel1,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAckermannSteeringAPI::GetMaxSteerAngleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleAckermannSteeringMaxSteerAngle);
}

UsdAttribute
PhysxSchemaPhysxVehicleAckermannSteeringAPI::CreateMaxSteerAngleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleAckermannSteeringMaxSteerAngle,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAckermannSteeringAPI::GetWheelBaseAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleAckermannSteeringWheelBase);
}

UsdAttribute
PhysxSchemaPhysxVehicleAckermannSteeringAPI::CreateWheelBaseAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleAckermannSteeringWheelBase,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAckermannSteeringAPI::GetTrackWidthAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleAckermannSteeringTrackWidth);
}

UsdAttribute
PhysxSchemaPhysxVehicleAckermannSteeringAPI::CreateTrackWidthAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleAckermannSteeringTrackWidth,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAckermannSteeringAPI::GetStrengthAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleAckermannSteeringStrength);
}

UsdAttribute
PhysxSchemaPhysxVehicleAckermannSteeringAPI::CreateStrengthAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleAckermannSteeringStrength,
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
PhysxSchemaPhysxVehicleAckermannSteeringAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxVehicleAckermannSteeringWheel0,
        PhysxSchemaTokens->physxVehicleAckermannSteeringWheel1,
        PhysxSchemaTokens->physxVehicleAckermannSteeringMaxSteerAngle,
        PhysxSchemaTokens->physxVehicleAckermannSteeringWheelBase,
        PhysxSchemaTokens->physxVehicleAckermannSteeringTrackWidth,
        PhysxSchemaTokens->physxVehicleAckermannSteeringStrength,
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
