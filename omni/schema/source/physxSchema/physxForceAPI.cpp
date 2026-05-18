//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxForceAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxForceAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxForceAPI::~PhysxSchemaPhysxForceAPI()
{
}

/* static */
PhysxSchemaPhysxForceAPI
PhysxSchemaPhysxForceAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxForceAPI();
    }
    return PhysxSchemaPhysxForceAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxForceAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxForceAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxForceAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxForceAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxForceAPI
PhysxSchemaPhysxForceAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxForceAPI>()) {
        return PhysxSchemaPhysxForceAPI(prim);
    }
    return PhysxSchemaPhysxForceAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxForceAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxForceAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxForceAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxForceAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxForceAPI::GetForceEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxForceForceEnabled);
}

UsdAttribute
PhysxSchemaPhysxForceAPI::CreateForceEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxForceForceEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxForceAPI::GetWorldFrameEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxForceWorldFrameEnabled);
}

UsdAttribute
PhysxSchemaPhysxForceAPI::CreateWorldFrameEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxForceWorldFrameEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxForceAPI::GetModeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxForceMode);
}

UsdAttribute
PhysxSchemaPhysxForceAPI::CreateModeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxForceMode,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxForceAPI::GetForceAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxForceForce);
}

UsdAttribute
PhysxSchemaPhysxForceAPI::CreateForceAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxForceForce,
                       SdfValueTypeNames->Vector3f,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxForceAPI::GetTorqueAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxForceTorque);
}

UsdAttribute
PhysxSchemaPhysxForceAPI::CreateTorqueAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxForceTorque,
                       SdfValueTypeNames->Vector3f,
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
PhysxSchemaPhysxForceAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxForceForceEnabled,
        PhysxSchemaTokens->physxForceWorldFrameEnabled,
        PhysxSchemaTokens->physxForceMode,
        PhysxSchemaTokens->physxForceForce,
        PhysxSchemaTokens->physxForceTorque,
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
