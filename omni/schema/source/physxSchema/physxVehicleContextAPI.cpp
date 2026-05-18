//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxVehicleContextAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleContextAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxVehicleContextAPI::~PhysxSchemaPhysxVehicleContextAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleContextAPI
PhysxSchemaPhysxVehicleContextAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleContextAPI();
    }
    return PhysxSchemaPhysxVehicleContextAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleContextAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleContextAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleContextAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleContextAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxVehicleContextAPI
PhysxSchemaPhysxVehicleContextAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleContextAPI>()) {
        return PhysxSchemaPhysxVehicleContextAPI(prim);
    }
    return PhysxSchemaPhysxVehicleContextAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleContextAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleContextAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleContextAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleContextAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::GetUpdateModeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleContextUpdateMode);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::CreateUpdateModeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleContextUpdateMode,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::GetUpAxisAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleContextUpAxis);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::CreateUpAxisAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleContextUpAxis,
                       SdfValueTypeNames->Float3,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::GetForwardAxisAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleContextForwardAxis);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::CreateForwardAxisAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleContextForwardAxis,
                       SdfValueTypeNames->Float3,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::GetVerticalAxisAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleContextVerticalAxis);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::CreateVerticalAxisAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleContextVerticalAxis,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::GetLongitudinalAxisAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleContextLongitudinalAxis);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::CreateLongitudinalAxisAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleContextLongitudinalAxis,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
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
PhysxSchemaPhysxVehicleContextAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxVehicleContextUpdateMode,
        PhysxSchemaTokens->physxVehicleContextUpAxis,
        PhysxSchemaTokens->physxVehicleContextForwardAxis,
        PhysxSchemaTokens->physxVehicleContextVerticalAxis,
        PhysxSchemaTokens->physxVehicleContextLongitudinalAxis,
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
