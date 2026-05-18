//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxVehicleSuspensionAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleSuspensionAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxVehicleSuspensionAPI::~PhysxSchemaPhysxVehicleSuspensionAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleSuspensionAPI
PhysxSchemaPhysxVehicleSuspensionAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleSuspensionAPI();
    }
    return PhysxSchemaPhysxVehicleSuspensionAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleSuspensionAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleSuspensionAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleSuspensionAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleSuspensionAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxVehicleSuspensionAPI
PhysxSchemaPhysxVehicleSuspensionAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleSuspensionAPI>()) {
        return PhysxSchemaPhysxVehicleSuspensionAPI(prim);
    }
    return PhysxSchemaPhysxVehicleSuspensionAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleSuspensionAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleSuspensionAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleSuspensionAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleSuspensionAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::GetSpringStrengthAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleSuspensionSpringStrength);
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::CreateSpringStrengthAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleSuspensionSpringStrength,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::GetSpringDamperRateAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleSuspensionSpringDamperRate);
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::CreateSpringDamperRateAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleSuspensionSpringDamperRate,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::GetTravelDistanceAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleSuspensionTravelDistance);
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::CreateTravelDistanceAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleSuspensionTravelDistance,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::GetMaxCompressionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleSuspensionMaxCompression);
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::CreateMaxCompressionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleSuspensionMaxCompression,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::GetMaxDroopAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleSuspensionMaxDroop);
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::CreateMaxDroopAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleSuspensionMaxDroop,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::GetSprungMassAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleSuspensionSprungMass);
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::CreateSprungMassAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleSuspensionSprungMass,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::GetCamberAtRestAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleSuspensionCamberAtRest);
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::CreateCamberAtRestAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleSuspensionCamberAtRest,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::GetCamberAtMaxCompressionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleSuspensionCamberAtMaxCompression);
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::CreateCamberAtMaxCompressionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleSuspensionCamberAtMaxCompression,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::GetCamberAtMaxDroopAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleSuspensionCamberAtMaxDroop);
}

UsdAttribute
PhysxSchemaPhysxVehicleSuspensionAPI::CreateCamberAtMaxDroopAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleSuspensionCamberAtMaxDroop,
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
PhysxSchemaPhysxVehicleSuspensionAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxVehicleSuspensionSpringStrength,
        PhysxSchemaTokens->physxVehicleSuspensionSpringDamperRate,
        PhysxSchemaTokens->physxVehicleSuspensionTravelDistance,
        PhysxSchemaTokens->physxVehicleSuspensionMaxCompression,
        PhysxSchemaTokens->physxVehicleSuspensionMaxDroop,
        PhysxSchemaTokens->physxVehicleSuspensionSprungMass,
        PhysxSchemaTokens->physxVehicleSuspensionCamberAtRest,
        PhysxSchemaTokens->physxVehicleSuspensionCamberAtMaxCompression,
        PhysxSchemaTokens->physxVehicleSuspensionCamberAtMaxDroop,
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
