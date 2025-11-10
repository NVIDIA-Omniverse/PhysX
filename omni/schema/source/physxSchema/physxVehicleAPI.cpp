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
#include ".//physxVehicleAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxVehicleAPI::~PhysxSchemaPhysxVehicleAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleAPI
PhysxSchemaPhysxVehicleAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleAPI();
    }
    return PhysxSchemaPhysxVehicleAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxVehicleAPI
PhysxSchemaPhysxVehicleAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleAPI>()) {
        return PhysxSchemaPhysxVehicleAPI(prim);
    }
    return PhysxSchemaPhysxVehicleAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::GetVehicleEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleVehicleEnabled);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::CreateVehicleEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleVehicleEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::GetSuspensionLineQueryTypeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleSuspensionLineQueryType);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::CreateSuspensionLineQueryTypeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleSuspensionLineQueryType,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::GetLimitSuspensionExpansionVelocityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleLimitSuspensionExpansionVelocity);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::CreateLimitSuspensionExpansionVelocityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleLimitSuspensionExpansionVelocity,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::GetSubStepThresholdLongitudinalSpeedAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleSubStepThresholdLongitudinalSpeed);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::CreateSubStepThresholdLongitudinalSpeedAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleSubStepThresholdLongitudinalSpeed,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::GetLowForwardSpeedSubStepCountAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleLowForwardSpeedSubStepCount);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::CreateLowForwardSpeedSubStepCountAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleLowForwardSpeedSubStepCount,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::GetHighForwardSpeedSubStepCountAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleHighForwardSpeedSubStepCount);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::CreateHighForwardSpeedSubStepCountAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleHighForwardSpeedSubStepCount,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::GetMinLongitudinalSlipDenominatorAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleMinLongitudinalSlipDenominator);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::CreateMinLongitudinalSlipDenominatorAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleMinLongitudinalSlipDenominator,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::GetMinPassiveLongitudinalSlipDenominatorAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleMinPassiveLongitudinalSlipDenominator);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::CreateMinPassiveLongitudinalSlipDenominatorAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleMinPassiveLongitudinalSlipDenominator,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::GetMinActiveLongitudinalSlipDenominatorAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleMinActiveLongitudinalSlipDenominator);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::CreateMinActiveLongitudinalSlipDenominatorAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleMinActiveLongitudinalSlipDenominator,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::GetMinLateralSlipDenominatorAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleMinLateralSlipDenominator);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::CreateMinLateralSlipDenominatorAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleMinLateralSlipDenominator,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::GetLongitudinalStickyTireThresholdSpeedAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleLongitudinalStickyTireThresholdSpeed);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::CreateLongitudinalStickyTireThresholdSpeedAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleLongitudinalStickyTireThresholdSpeed,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::GetLongitudinalStickyTireThresholdTimeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleLongitudinalStickyTireThresholdTime);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::CreateLongitudinalStickyTireThresholdTimeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleLongitudinalStickyTireThresholdTime,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::GetLongitudinalStickyTireDampingAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleLongitudinalStickyTireDamping);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::CreateLongitudinalStickyTireDampingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleLongitudinalStickyTireDamping,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::GetLateralStickyTireThresholdSpeedAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleLateralStickyTireThresholdSpeed);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::CreateLateralStickyTireThresholdSpeedAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleLateralStickyTireThresholdSpeed,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::GetLateralStickyTireThresholdTimeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleLateralStickyTireThresholdTime);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::CreateLateralStickyTireThresholdTimeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleLateralStickyTireThresholdTime,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::GetLateralStickyTireDampingAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleLateralStickyTireDamping);
}

UsdAttribute
PhysxSchemaPhysxVehicleAPI::CreateLateralStickyTireDampingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleLateralStickyTireDamping,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdRelationship
PhysxSchemaPhysxVehicleAPI::GetDriveRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physxVehicleDrive);
}

UsdRelationship
PhysxSchemaPhysxVehicleAPI::CreateDriveRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physxVehicleDrive,
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
PhysxSchemaPhysxVehicleAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxVehicleVehicleEnabled,
        PhysxSchemaTokens->physxVehicleSuspensionLineQueryType,
        PhysxSchemaTokens->physxVehicleLimitSuspensionExpansionVelocity,
        PhysxSchemaTokens->physxVehicleSubStepThresholdLongitudinalSpeed,
        PhysxSchemaTokens->physxVehicleLowForwardSpeedSubStepCount,
        PhysxSchemaTokens->physxVehicleHighForwardSpeedSubStepCount,
        PhysxSchemaTokens->physxVehicleMinLongitudinalSlipDenominator,
        PhysxSchemaTokens->physxVehicleMinPassiveLongitudinalSlipDenominator,
        PhysxSchemaTokens->physxVehicleMinActiveLongitudinalSlipDenominator,
        PhysxSchemaTokens->physxVehicleMinLateralSlipDenominator,
        PhysxSchemaTokens->physxVehicleLongitudinalStickyTireThresholdSpeed,
        PhysxSchemaTokens->physxVehicleLongitudinalStickyTireThresholdTime,
        PhysxSchemaTokens->physxVehicleLongitudinalStickyTireDamping,
        PhysxSchemaTokens->physxVehicleLateralStickyTireThresholdSpeed,
        PhysxSchemaTokens->physxVehicleLateralStickyTireThresholdTime,
        PhysxSchemaTokens->physxVehicleLateralStickyTireDamping,
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
