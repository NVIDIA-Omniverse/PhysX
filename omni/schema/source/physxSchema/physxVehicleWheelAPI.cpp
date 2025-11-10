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
#include ".//physxVehicleWheelAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleWheelAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxVehicleWheelAPI::~PhysxSchemaPhysxVehicleWheelAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleWheelAPI
PhysxSchemaPhysxVehicleWheelAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleWheelAPI();
    }
    return PhysxSchemaPhysxVehicleWheelAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleWheelAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleWheelAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleWheelAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleWheelAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxVehicleWheelAPI
PhysxSchemaPhysxVehicleWheelAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleWheelAPI>()) {
        return PhysxSchemaPhysxVehicleWheelAPI(prim);
    }
    return PhysxSchemaPhysxVehicleWheelAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleWheelAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleWheelAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleWheelAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleWheelAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::GetRadiusAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelRadius);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::CreateRadiusAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelRadius,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::GetWidthAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelWidth);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::CreateWidthAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelWidth,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::GetMassAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelMass);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::CreateMassAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelMass,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::GetMoiAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelMoi);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::CreateMoiAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelMoi,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::GetDampingRateAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelDampingRate);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::CreateDampingRateAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelDampingRate,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::GetMaxBrakeTorqueAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelMaxBrakeTorque);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::CreateMaxBrakeTorqueAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelMaxBrakeTorque,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::GetMaxHandBrakeTorqueAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelMaxHandBrakeTorque);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::CreateMaxHandBrakeTorqueAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelMaxHandBrakeTorque,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::GetMaxSteerAngleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelMaxSteerAngle);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::CreateMaxSteerAngleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelMaxSteerAngle,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::GetToeAngleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelToeAngle);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAPI::CreateToeAngleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelToeAngle,
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
PhysxSchemaPhysxVehicleWheelAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxVehicleWheelRadius,
        PhysxSchemaTokens->physxVehicleWheelWidth,
        PhysxSchemaTokens->physxVehicleWheelMass,
        PhysxSchemaTokens->physxVehicleWheelMoi,
        PhysxSchemaTokens->physxVehicleWheelDampingRate,
        PhysxSchemaTokens->physxVehicleWheelMaxBrakeTorque,
        PhysxSchemaTokens->physxVehicleWheelMaxHandBrakeTorque,
        PhysxSchemaTokens->physxVehicleWheelMaxSteerAngle,
        PhysxSchemaTokens->physxVehicleWheelToeAngle,
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
