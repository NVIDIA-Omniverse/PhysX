//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxParticleAnisotropyAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxParticleAnisotropyAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxParticleAnisotropyAPI::~PhysxSchemaPhysxParticleAnisotropyAPI()
{
}

/* static */
PhysxSchemaPhysxParticleAnisotropyAPI
PhysxSchemaPhysxParticleAnisotropyAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxParticleAnisotropyAPI();
    }
    return PhysxSchemaPhysxParticleAnisotropyAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxParticleAnisotropyAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxParticleAnisotropyAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxParticleAnisotropyAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxParticleAnisotropyAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxParticleAnisotropyAPI
PhysxSchemaPhysxParticleAnisotropyAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxParticleAnisotropyAPI>()) {
        return PhysxSchemaPhysxParticleAnisotropyAPI(prim);
    }
    return PhysxSchemaPhysxParticleAnisotropyAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxParticleAnisotropyAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxParticleAnisotropyAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxParticleAnisotropyAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxParticleAnisotropyAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxParticleAnisotropyAPI::GetParticleAnisotropyEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleAnisotropyParticleAnisotropyEnabled);
}

UsdAttribute
PhysxSchemaPhysxParticleAnisotropyAPI::CreateParticleAnisotropyEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleAnisotropyParticleAnisotropyEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleAnisotropyAPI::GetScaleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleAnisotropyScale);
}

UsdAttribute
PhysxSchemaPhysxParticleAnisotropyAPI::CreateScaleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleAnisotropyScale,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleAnisotropyAPI::GetMinAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleAnisotropyMin);
}

UsdAttribute
PhysxSchemaPhysxParticleAnisotropyAPI::CreateMinAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleAnisotropyMin,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleAnisotropyAPI::GetMaxAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleAnisotropyMax);
}

UsdAttribute
PhysxSchemaPhysxParticleAnisotropyAPI::CreateMaxAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleAnisotropyMax,
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
PhysxSchemaPhysxParticleAnisotropyAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxParticleAnisotropyParticleAnisotropyEnabled,
        PhysxSchemaTokens->physxParticleAnisotropyScale,
        PhysxSchemaTokens->physxParticleAnisotropyMin,
        PhysxSchemaTokens->physxParticleAnisotropyMax,
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
