//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxParticleAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxParticleAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxParticleAPI::~PhysxSchemaPhysxParticleAPI()
{
}

/* static */
PhysxSchemaPhysxParticleAPI
PhysxSchemaPhysxParticleAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxParticleAPI();
    }
    return PhysxSchemaPhysxParticleAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxParticleAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxParticleAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxParticleAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxParticleAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxParticleAPI
PhysxSchemaPhysxParticleAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxParticleAPI>()) {
        return PhysxSchemaPhysxParticleAPI(prim);
    }
    return PhysxSchemaPhysxParticleAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxParticleAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxParticleAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxParticleAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxParticleAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxParticleAPI::GetParticleEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleParticleEnabled);
}

UsdAttribute
PhysxSchemaPhysxParticleAPI::CreateParticleEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleParticleEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleAPI::GetSelfCollisionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleSelfCollision);
}

UsdAttribute
PhysxSchemaPhysxParticleAPI::CreateSelfCollisionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleSelfCollision,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleAPI::GetParticleGroupAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleParticleGroup);
}

UsdAttribute
PhysxSchemaPhysxParticleAPI::CreateParticleGroupAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleParticleGroup,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdRelationship
PhysxSchemaPhysxParticleAPI::GetParticleSystemRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physxParticleParticleSystem);
}

UsdRelationship
PhysxSchemaPhysxParticleAPI::CreateParticleSystemRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physxParticleParticleSystem,
                       /* custom = */ false);
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
PhysxSchemaPhysxParticleAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxParticleParticleEnabled,
        PhysxSchemaTokens->physxParticleSelfCollision,
        PhysxSchemaTokens->physxParticleParticleGroup,
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
