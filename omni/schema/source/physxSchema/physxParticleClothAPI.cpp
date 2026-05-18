//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxParticleClothAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxParticleClothAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxParticleClothAPI::~PhysxSchemaPhysxParticleClothAPI()
{
}

/* static */
PhysxSchemaPhysxParticleClothAPI
PhysxSchemaPhysxParticleClothAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxParticleClothAPI();
    }
    return PhysxSchemaPhysxParticleClothAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxParticleClothAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxParticleClothAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxParticleClothAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxParticleClothAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxParticleClothAPI
PhysxSchemaPhysxParticleClothAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxParticleClothAPI>()) {
        return PhysxSchemaPhysxParticleClothAPI(prim);
    }
    return PhysxSchemaPhysxParticleClothAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxParticleClothAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxParticleClothAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxParticleClothAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxParticleClothAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxParticleClothAPI::GetSelfCollisionFilterAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleSelfCollisionFilter);
}

UsdAttribute
PhysxSchemaPhysxParticleClothAPI::CreateSelfCollisionFilterAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleSelfCollisionFilter,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleClothAPI::GetRestPointsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleRestPoints);
}

UsdAttribute
PhysxSchemaPhysxParticleClothAPI::CreateRestPointsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleRestPoints,
                       SdfValueTypeNames->Point3fArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleClothAPI::GetSpringIndicesAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleSpringIndices);
}

UsdAttribute
PhysxSchemaPhysxParticleClothAPI::CreateSpringIndicesAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleSpringIndices,
                       SdfValueTypeNames->Int2Array,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleClothAPI::GetSpringStiffnessesAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleSpringStiffnesses);
}

UsdAttribute
PhysxSchemaPhysxParticleClothAPI::CreateSpringStiffnessesAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleSpringStiffnesses,
                       SdfValueTypeNames->FloatArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleClothAPI::GetSpringDampingsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleSpringDampings);
}

UsdAttribute
PhysxSchemaPhysxParticleClothAPI::CreateSpringDampingsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleSpringDampings,
                       SdfValueTypeNames->FloatArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleClothAPI::GetSpringRestLengthsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleSpringRestLengths);
}

UsdAttribute
PhysxSchemaPhysxParticleClothAPI::CreateSpringRestLengthsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleSpringRestLengths,
                       SdfValueTypeNames->FloatArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleClothAPI::GetPressureAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticlePressure);
}

UsdAttribute
PhysxSchemaPhysxParticleClothAPI::CreatePressureAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticlePressure,
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
PhysxSchemaPhysxParticleClothAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxParticleSelfCollisionFilter,
        PhysxSchemaTokens->physxParticleRestPoints,
        PhysxSchemaTokens->physxParticleSpringIndices,
        PhysxSchemaTokens->physxParticleSpringStiffnesses,
        PhysxSchemaTokens->physxParticleSpringDampings,
        PhysxSchemaTokens->physxParticleSpringRestLengths,
        PhysxSchemaTokens->physxParticlePressure,
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
