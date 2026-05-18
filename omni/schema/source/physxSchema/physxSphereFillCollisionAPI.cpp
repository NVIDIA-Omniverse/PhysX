//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxSphereFillCollisionAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxSphereFillCollisionAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxSphereFillCollisionAPI::~PhysxSchemaPhysxSphereFillCollisionAPI()
{
}

/* static */
PhysxSchemaPhysxSphereFillCollisionAPI
PhysxSchemaPhysxSphereFillCollisionAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxSphereFillCollisionAPI();
    }
    return PhysxSchemaPhysxSphereFillCollisionAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxSphereFillCollisionAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxSphereFillCollisionAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxSphereFillCollisionAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxSphereFillCollisionAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxSphereFillCollisionAPI
PhysxSchemaPhysxSphereFillCollisionAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxSphereFillCollisionAPI>()) {
        return PhysxSchemaPhysxSphereFillCollisionAPI(prim);
    }
    return PhysxSchemaPhysxSphereFillCollisionAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxSphereFillCollisionAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxSphereFillCollisionAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxSphereFillCollisionAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxSphereFillCollisionAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxSphereFillCollisionAPI::GetMaxSpheresAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSphereFillCollisionMaxSpheres);
}

UsdAttribute
PhysxSchemaPhysxSphereFillCollisionAPI::CreateMaxSpheresAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSphereFillCollisionMaxSpheres,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSphereFillCollisionAPI::GetVoxelResolutionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSphereFillCollisionVoxelResolution);
}

UsdAttribute
PhysxSchemaPhysxSphereFillCollisionAPI::CreateVoxelResolutionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSphereFillCollisionVoxelResolution,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSphereFillCollisionAPI::GetSeedCountAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSphereFillCollisionSeedCount);
}

UsdAttribute
PhysxSchemaPhysxSphereFillCollisionAPI::CreateSeedCountAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSphereFillCollisionSeedCount,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSphereFillCollisionAPI::GetFillModeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSphereFillCollisionFillMode);
}

UsdAttribute
PhysxSchemaPhysxSphereFillCollisionAPI::CreateFillModeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSphereFillCollisionFillMode,
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
PhysxSchemaPhysxSphereFillCollisionAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxSphereFillCollisionMaxSpheres,
        PhysxSchemaTokens->physxSphereFillCollisionVoxelResolution,
        PhysxSchemaTokens->physxSphereFillCollisionSeedCount,
        PhysxSchemaTokens->physxSphereFillCollisionFillMode,
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
