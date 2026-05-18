//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxVehicleDriveBasicAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleDriveBasicAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxVehicleDriveBasicAPI::~PhysxSchemaPhysxVehicleDriveBasicAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleDriveBasicAPI
PhysxSchemaPhysxVehicleDriveBasicAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleDriveBasicAPI();
    }
    return PhysxSchemaPhysxVehicleDriveBasicAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleDriveBasicAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleDriveBasicAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleDriveBasicAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleDriveBasicAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxVehicleDriveBasicAPI
PhysxSchemaPhysxVehicleDriveBasicAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleDriveBasicAPI>()) {
        return PhysxSchemaPhysxVehicleDriveBasicAPI(prim);
    }
    return PhysxSchemaPhysxVehicleDriveBasicAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleDriveBasicAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleDriveBasicAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleDriveBasicAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleDriveBasicAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxVehicleDriveBasicAPI::GetPeakTorqueAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleDriveBasicPeakTorque);
}

UsdAttribute
PhysxSchemaPhysxVehicleDriveBasicAPI::CreatePeakTorqueAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleDriveBasicPeakTorque,
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
PhysxSchemaPhysxVehicleDriveBasicAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxVehicleDriveBasicPeakTorque,
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
