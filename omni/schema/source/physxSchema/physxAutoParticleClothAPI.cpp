//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxAutoParticleClothAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxAutoParticleClothAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxAutoParticleClothAPI::~PhysxSchemaPhysxAutoParticleClothAPI()
{
}

/* static */
PhysxSchemaPhysxAutoParticleClothAPI
PhysxSchemaPhysxAutoParticleClothAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxAutoParticleClothAPI();
    }
    return PhysxSchemaPhysxAutoParticleClothAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxAutoParticleClothAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxAutoParticleClothAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxAutoParticleClothAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxAutoParticleClothAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxAutoParticleClothAPI
PhysxSchemaPhysxAutoParticleClothAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxAutoParticleClothAPI>()) {
        return PhysxSchemaPhysxAutoParticleClothAPI(prim);
    }
    return PhysxSchemaPhysxAutoParticleClothAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxAutoParticleClothAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxAutoParticleClothAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxAutoParticleClothAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxAutoParticleClothAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxAutoParticleClothAPI::GetSpringStretchStiffnessAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxAutoParticleClothSpringStretchStiffness);
}

UsdAttribute
PhysxSchemaPhysxAutoParticleClothAPI::CreateSpringStretchStiffnessAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxAutoParticleClothSpringStretchStiffness,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxAutoParticleClothAPI::GetSpringBendStiffnessAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxAutoParticleClothSpringBendStiffness);
}

UsdAttribute
PhysxSchemaPhysxAutoParticleClothAPI::CreateSpringBendStiffnessAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxAutoParticleClothSpringBendStiffness,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxAutoParticleClothAPI::GetSpringShearStiffnessAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxAutoParticleClothSpringShearStiffness);
}

UsdAttribute
PhysxSchemaPhysxAutoParticleClothAPI::CreateSpringShearStiffnessAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxAutoParticleClothSpringShearStiffness,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxAutoParticleClothAPI::GetSpringDampingAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxAutoParticleClothSpringDamping);
}

UsdAttribute
PhysxSchemaPhysxAutoParticleClothAPI::CreateSpringDampingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxAutoParticleClothSpringDamping,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxAutoParticleClothAPI::GetDisableMeshWeldingAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxAutoParticleClothDisableMeshWelding);
}

UsdAttribute
PhysxSchemaPhysxAutoParticleClothAPI::CreateDisableMeshWeldingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxAutoParticleClothDisableMeshWelding,
                       SdfValueTypeNames->Bool,
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
PhysxSchemaPhysxAutoParticleClothAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxAutoParticleClothSpringStretchStiffness,
        PhysxSchemaTokens->physxAutoParticleClothSpringBendStiffness,
        PhysxSchemaTokens->physxAutoParticleClothSpringShearStiffness,
        PhysxSchemaTokens->physxAutoParticleClothSpringDamping,
        PhysxSchemaTokens->physxAutoParticleClothDisableMeshWelding,
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
