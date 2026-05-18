//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxMaterialAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxMaterialAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxMaterialAPI::~PhysxSchemaPhysxMaterialAPI()
{
}

/* static */
PhysxSchemaPhysxMaterialAPI
PhysxSchemaPhysxMaterialAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxMaterialAPI();
    }
    return PhysxSchemaPhysxMaterialAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxMaterialAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxMaterialAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxMaterialAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxMaterialAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxMaterialAPI
PhysxSchemaPhysxMaterialAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxMaterialAPI>()) {
        return PhysxSchemaPhysxMaterialAPI(prim);
    }
    return PhysxSchemaPhysxMaterialAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxMaterialAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxMaterialAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxMaterialAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxMaterialAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxMaterialAPI::GetFrictionCombineModeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxMaterialFrictionCombineMode);
}

UsdAttribute
PhysxSchemaPhysxMaterialAPI::CreateFrictionCombineModeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxMaterialFrictionCombineMode,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxMaterialAPI::GetRestitutionCombineModeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxMaterialRestitutionCombineMode);
}

UsdAttribute
PhysxSchemaPhysxMaterialAPI::CreateRestitutionCombineModeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxMaterialRestitutionCombineMode,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxMaterialAPI::GetDampingCombineModeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxMaterialDampingCombineMode);
}

UsdAttribute
PhysxSchemaPhysxMaterialAPI::CreateDampingCombineModeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxMaterialDampingCombineMode,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxMaterialAPI::GetCompliantContactAccelerationSpringAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxMaterialCompliantContactAccelerationSpring);
}

UsdAttribute
PhysxSchemaPhysxMaterialAPI::CreateCompliantContactAccelerationSpringAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxMaterialCompliantContactAccelerationSpring,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxMaterialAPI::GetCompliantContactStiffnessAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxMaterialCompliantContactStiffness);
}

UsdAttribute
PhysxSchemaPhysxMaterialAPI::CreateCompliantContactStiffnessAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxMaterialCompliantContactStiffness,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxMaterialAPI::GetCompliantContactDampingAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxMaterialCompliantContactDamping);
}

UsdAttribute
PhysxSchemaPhysxMaterialAPI::CreateCompliantContactDampingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxMaterialCompliantContactDamping,
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
PhysxSchemaPhysxMaterialAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxMaterialFrictionCombineMode,
        PhysxSchemaTokens->physxMaterialRestitutionCombineMode,
        PhysxSchemaTokens->physxMaterialDampingCombineMode,
        PhysxSchemaTokens->physxMaterialCompliantContactAccelerationSpring,
        PhysxSchemaTokens->physxMaterialCompliantContactStiffness,
        PhysxSchemaTokens->physxMaterialCompliantContactDamping,
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
