//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxParticleSmoothingAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxParticleSmoothingAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxParticleSmoothingAPI::~PhysxSchemaPhysxParticleSmoothingAPI()
{
}

/* static */
PhysxSchemaPhysxParticleSmoothingAPI
PhysxSchemaPhysxParticleSmoothingAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxParticleSmoothingAPI();
    }
    return PhysxSchemaPhysxParticleSmoothingAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxParticleSmoothingAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxParticleSmoothingAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxParticleSmoothingAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxParticleSmoothingAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxParticleSmoothingAPI
PhysxSchemaPhysxParticleSmoothingAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxParticleSmoothingAPI>()) {
        return PhysxSchemaPhysxParticleSmoothingAPI(prim);
    }
    return PhysxSchemaPhysxParticleSmoothingAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxParticleSmoothingAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxParticleSmoothingAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxParticleSmoothingAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxParticleSmoothingAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxParticleSmoothingAPI::GetParticleSmoothingEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleSmoothingParticleSmoothingEnabled);
}

UsdAttribute
PhysxSchemaPhysxParticleSmoothingAPI::CreateParticleSmoothingEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleSmoothingParticleSmoothingEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleSmoothingAPI::GetStrengthAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleSmoothingStrength);
}

UsdAttribute
PhysxSchemaPhysxParticleSmoothingAPI::CreateStrengthAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleSmoothingStrength,
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
PhysxSchemaPhysxParticleSmoothingAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxParticleSmoothingParticleSmoothingEnabled,
        PhysxSchemaTokens->physxParticleSmoothingStrength,
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
