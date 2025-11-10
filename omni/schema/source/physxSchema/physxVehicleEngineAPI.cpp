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
#include ".//physxVehicleEngineAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleEngineAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxVehicleEngineAPI::~PhysxSchemaPhysxVehicleEngineAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleEngineAPI
PhysxSchemaPhysxVehicleEngineAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleEngineAPI();
    }
    return PhysxSchemaPhysxVehicleEngineAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleEngineAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleEngineAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleEngineAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleEngineAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxVehicleEngineAPI
PhysxSchemaPhysxVehicleEngineAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleEngineAPI>()) {
        return PhysxSchemaPhysxVehicleEngineAPI(prim);
    }
    return PhysxSchemaPhysxVehicleEngineAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleEngineAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleEngineAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleEngineAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleEngineAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxVehicleEngineAPI::GetMoiAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleEngineMoi);
}

UsdAttribute
PhysxSchemaPhysxVehicleEngineAPI::CreateMoiAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleEngineMoi,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleEngineAPI::GetPeakTorqueAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleEnginePeakTorque);
}

UsdAttribute
PhysxSchemaPhysxVehicleEngineAPI::CreatePeakTorqueAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleEnginePeakTorque,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleEngineAPI::GetMaxRotationSpeedAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleEngineMaxRotationSpeed);
}

UsdAttribute
PhysxSchemaPhysxVehicleEngineAPI::CreateMaxRotationSpeedAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleEngineMaxRotationSpeed,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleEngineAPI::GetIdleRotationSpeedAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleEngineIdleRotationSpeed);
}

UsdAttribute
PhysxSchemaPhysxVehicleEngineAPI::CreateIdleRotationSpeedAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleEngineIdleRotationSpeed,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleEngineAPI::GetTorqueCurveAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleEngineTorqueCurve);
}

UsdAttribute
PhysxSchemaPhysxVehicleEngineAPI::CreateTorqueCurveAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleEngineTorqueCurve,
                       SdfValueTypeNames->Float2Array,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleEngineAPI::GetDampingRateFullThrottleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleEngineDampingRateFullThrottle);
}

UsdAttribute
PhysxSchemaPhysxVehicleEngineAPI::CreateDampingRateFullThrottleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleEngineDampingRateFullThrottle,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleEngineAPI::GetDampingRateZeroThrottleClutchEngagedAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleEngineDampingRateZeroThrottleClutchEngaged);
}

UsdAttribute
PhysxSchemaPhysxVehicleEngineAPI::CreateDampingRateZeroThrottleClutchEngagedAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleEngineDampingRateZeroThrottleClutchEngaged,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleEngineAPI::GetDampingRateZeroThrottleClutchDisengagedAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleEngineDampingRateZeroThrottleClutchDisengaged);
}

UsdAttribute
PhysxSchemaPhysxVehicleEngineAPI::CreateDampingRateZeroThrottleClutchDisengagedAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleEngineDampingRateZeroThrottleClutchDisengaged,
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
PhysxSchemaPhysxVehicleEngineAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxVehicleEngineMoi,
        PhysxSchemaTokens->physxVehicleEnginePeakTorque,
        PhysxSchemaTokens->physxVehicleEngineMaxRotationSpeed,
        PhysxSchemaTokens->physxVehicleEngineIdleRotationSpeed,
        PhysxSchemaTokens->physxVehicleEngineTorqueCurve,
        PhysxSchemaTokens->physxVehicleEngineDampingRateFullThrottle,
        PhysxSchemaTokens->physxVehicleEngineDampingRateZeroThrottleClutchEngaged,
        PhysxSchemaTokens->physxVehicleEngineDampingRateZeroThrottleClutchDisengaged,
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
