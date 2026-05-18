//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxVehicleAutoGearBoxAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleAutoGearBoxAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxVehicleAutoGearBoxAPI::~PhysxSchemaPhysxVehicleAutoGearBoxAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleAutoGearBoxAPI
PhysxSchemaPhysxVehicleAutoGearBoxAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleAutoGearBoxAPI();
    }
    return PhysxSchemaPhysxVehicleAutoGearBoxAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleAutoGearBoxAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleAutoGearBoxAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleAutoGearBoxAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleAutoGearBoxAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxVehicleAutoGearBoxAPI
PhysxSchemaPhysxVehicleAutoGearBoxAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleAutoGearBoxAPI>()) {
        return PhysxSchemaPhysxVehicleAutoGearBoxAPI(prim);
    }
    return PhysxSchemaPhysxVehicleAutoGearBoxAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleAutoGearBoxAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleAutoGearBoxAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleAutoGearBoxAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleAutoGearBoxAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxVehicleAutoGearBoxAPI::GetUpRatiosAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleAutoGearBoxUpRatios);
}

UsdAttribute
PhysxSchemaPhysxVehicleAutoGearBoxAPI::CreateUpRatiosAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleAutoGearBoxUpRatios,
                       SdfValueTypeNames->FloatArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAutoGearBoxAPI::GetDownRatiosAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleAutoGearBoxDownRatios);
}

UsdAttribute
PhysxSchemaPhysxVehicleAutoGearBoxAPI::CreateDownRatiosAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleAutoGearBoxDownRatios,
                       SdfValueTypeNames->FloatArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleAutoGearBoxAPI::GetLatencyAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleAutoGearBoxLatency);
}

UsdAttribute
PhysxSchemaPhysxVehicleAutoGearBoxAPI::CreateLatencyAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleAutoGearBoxLatency,
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
PhysxSchemaPhysxVehicleAutoGearBoxAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxVehicleAutoGearBoxUpRatios,
        PhysxSchemaTokens->physxVehicleAutoGearBoxDownRatios,
        PhysxSchemaTokens->physxVehicleAutoGearBoxLatency,
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
