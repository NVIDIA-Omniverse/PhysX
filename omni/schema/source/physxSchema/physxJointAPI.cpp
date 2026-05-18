//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxJointAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxJointAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxJointAPI::~PhysxSchemaPhysxJointAPI()
{
}

/* static */
PhysxSchemaPhysxJointAPI
PhysxSchemaPhysxJointAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxJointAPI();
    }
    return PhysxSchemaPhysxJointAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxJointAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxJointAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxJointAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxJointAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxJointAPI
PhysxSchemaPhysxJointAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxJointAPI>()) {
        return PhysxSchemaPhysxJointAPI(prim);
    }
    return PhysxSchemaPhysxJointAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxJointAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxJointAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxJointAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxJointAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxJointAPI::GetJointFrictionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxJointJointFriction);
}

UsdAttribute
PhysxSchemaPhysxJointAPI::CreateJointFrictionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxJointJointFriction,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxJointAPI::GetMaxJointVelocityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxJointMaxJointVelocity);
}

UsdAttribute
PhysxSchemaPhysxJointAPI::CreateMaxJointVelocityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxJointMaxJointVelocity,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxJointAPI::GetArmatureAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxJointArmature);
}

UsdAttribute
PhysxSchemaPhysxJointAPI::CreateArmatureAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxJointArmature,
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
PhysxSchemaPhysxJointAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxJointJointFriction,
        PhysxSchemaTokens->physxJointMaxJointVelocity,
        PhysxSchemaTokens->physxJointArmature,
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
