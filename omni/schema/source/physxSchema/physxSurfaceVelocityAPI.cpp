//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxSurfaceVelocityAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxSurfaceVelocityAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxSurfaceVelocityAPI::~PhysxSchemaPhysxSurfaceVelocityAPI()
{
}

/* static */
PhysxSchemaPhysxSurfaceVelocityAPI
PhysxSchemaPhysxSurfaceVelocityAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxSurfaceVelocityAPI();
    }
    return PhysxSchemaPhysxSurfaceVelocityAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxSurfaceVelocityAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxSurfaceVelocityAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxSurfaceVelocityAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxSurfaceVelocityAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxSurfaceVelocityAPI
PhysxSchemaPhysxSurfaceVelocityAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxSurfaceVelocityAPI>()) {
        return PhysxSchemaPhysxSurfaceVelocityAPI(prim);
    }
    return PhysxSchemaPhysxSurfaceVelocityAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxSurfaceVelocityAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxSurfaceVelocityAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxSurfaceVelocityAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxSurfaceVelocityAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxSurfaceVelocityAPI::GetSurfaceVelocityEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocityEnabled);
}

UsdAttribute
PhysxSchemaPhysxSurfaceVelocityAPI::CreateSurfaceVelocityEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocityEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSurfaceVelocityAPI::GetSurfaceVelocityLocalSpaceAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocityLocalSpace);
}

UsdAttribute
PhysxSchemaPhysxSurfaceVelocityAPI::CreateSurfaceVelocityLocalSpaceAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocityLocalSpace,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSurfaceVelocityAPI::GetSurfaceVelocityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocity);
}

UsdAttribute
PhysxSchemaPhysxSurfaceVelocityAPI::CreateSurfaceVelocityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocity,
                       SdfValueTypeNames->Vector3f,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSurfaceVelocityAPI::GetSurfaceAngularVelocityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSurfaceVelocitySurfaceAngularVelocity);
}

UsdAttribute
PhysxSchemaPhysxSurfaceVelocityAPI::CreateSurfaceAngularVelocityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSurfaceVelocitySurfaceAngularVelocity,
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
PhysxSchemaPhysxSurfaceVelocityAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocityEnabled,
        PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocityLocalSpace,
        PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocity,
        PhysxSchemaTokens->physxSurfaceVelocitySurfaceAngularVelocity,
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
