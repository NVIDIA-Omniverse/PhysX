//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxVehicleTankControllerAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleTankControllerAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxVehicleTankControllerAPI::~PhysxSchemaPhysxVehicleTankControllerAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleTankControllerAPI
PhysxSchemaPhysxVehicleTankControllerAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleTankControllerAPI();
    }
    return PhysxSchemaPhysxVehicleTankControllerAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleTankControllerAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleTankControllerAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleTankControllerAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleTankControllerAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxVehicleTankControllerAPI
PhysxSchemaPhysxVehicleTankControllerAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleTankControllerAPI>()) {
        return PhysxSchemaPhysxVehicleTankControllerAPI(prim);
    }
    return PhysxSchemaPhysxVehicleTankControllerAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleTankControllerAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleTankControllerAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleTankControllerAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleTankControllerAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxVehicleTankControllerAPI::GetThrust0Attr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleTankControllerThrust0);
}

UsdAttribute
PhysxSchemaPhysxVehicleTankControllerAPI::CreateThrust0Attr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleTankControllerThrust0,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleTankControllerAPI::GetThrust1Attr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleTankControllerThrust1);
}

UsdAttribute
PhysxSchemaPhysxVehicleTankControllerAPI::CreateThrust1Attr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleTankControllerThrust1,
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
PhysxSchemaPhysxVehicleTankControllerAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxVehicleTankControllerThrust0,
        PhysxSchemaTokens->physxVehicleTankControllerThrust1,
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
