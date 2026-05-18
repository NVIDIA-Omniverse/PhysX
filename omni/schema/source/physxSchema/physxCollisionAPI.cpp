//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxCollisionAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxCollisionAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxCollisionAPI::~PhysxSchemaPhysxCollisionAPI()
{
}

/* static */
PhysxSchemaPhysxCollisionAPI
PhysxSchemaPhysxCollisionAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxCollisionAPI();
    }
    return PhysxSchemaPhysxCollisionAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxCollisionAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxCollisionAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxCollisionAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxCollisionAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxCollisionAPI
PhysxSchemaPhysxCollisionAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxCollisionAPI>()) {
        return PhysxSchemaPhysxCollisionAPI(prim);
    }
    return PhysxSchemaPhysxCollisionAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxCollisionAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxCollisionAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxCollisionAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxCollisionAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxCollisionAPI::GetContactOffsetAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxCollisionContactOffset);
}

UsdAttribute
PhysxSchemaPhysxCollisionAPI::CreateContactOffsetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxCollisionContactOffset,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCollisionAPI::GetRestOffsetAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxCollisionRestOffset);
}

UsdAttribute
PhysxSchemaPhysxCollisionAPI::CreateRestOffsetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxCollisionRestOffset,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCollisionAPI::GetTorsionalPatchRadiusAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxCollisionTorsionalPatchRadius);
}

UsdAttribute
PhysxSchemaPhysxCollisionAPI::CreateTorsionalPatchRadiusAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxCollisionTorsionalPatchRadius,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxCollisionAPI::GetMinTorsionalPatchRadiusAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxCollisionMinTorsionalPatchRadius);
}

UsdAttribute
PhysxSchemaPhysxCollisionAPI::CreateMinTorsionalPatchRadiusAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxCollisionMinTorsionalPatchRadius,
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
PhysxSchemaPhysxCollisionAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxCollisionContactOffset,
        PhysxSchemaTokens->physxCollisionRestOffset,
        PhysxSchemaTokens->physxCollisionTorsionalPatchRadius,
        PhysxSchemaTokens->physxCollisionMinTorsionalPatchRadius,
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
