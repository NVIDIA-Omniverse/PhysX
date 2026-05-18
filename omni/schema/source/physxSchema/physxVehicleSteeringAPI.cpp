//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxVehicleSteeringAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleSteeringAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxVehicleSteeringAPI::~PhysxSchemaPhysxVehicleSteeringAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleSteeringAPI
PhysxSchemaPhysxVehicleSteeringAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleSteeringAPI();
    }
    return PhysxSchemaPhysxVehicleSteeringAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleSteeringAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleSteeringAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleSteeringAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleSteeringAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxVehicleSteeringAPI
PhysxSchemaPhysxVehicleSteeringAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleSteeringAPI>()) {
        return PhysxSchemaPhysxVehicleSteeringAPI(prim);
    }
    return PhysxSchemaPhysxVehicleSteeringAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleSteeringAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleSteeringAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleSteeringAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleSteeringAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxVehicleSteeringAPI::GetWheelsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleSteeringWheels);
}

UsdAttribute
PhysxSchemaPhysxVehicleSteeringAPI::CreateWheelsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleSteeringWheels,
                       SdfValueTypeNames->IntArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleSteeringAPI::GetMaxSteerAngleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleSteeringMaxSteerAngle);
}

UsdAttribute
PhysxSchemaPhysxVehicleSteeringAPI::CreateMaxSteerAngleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleSteeringMaxSteerAngle,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleSteeringAPI::GetAngleMultipliersAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleSteeringAngleMultipliers);
}

UsdAttribute
PhysxSchemaPhysxVehicleSteeringAPI::CreateAngleMultipliersAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleSteeringAngleMultipliers,
                       SdfValueTypeNames->FloatArray,
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
PhysxSchemaPhysxVehicleSteeringAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxVehicleSteeringWheels,
        PhysxSchemaTokens->physxVehicleSteeringMaxSteerAngle,
        PhysxSchemaTokens->physxVehicleSteeringAngleMultipliers,
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
