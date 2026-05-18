//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxConvexHullCollisionAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxConvexHullCollisionAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxConvexHullCollisionAPI::~PhysxSchemaPhysxConvexHullCollisionAPI()
{
}

/* static */
PhysxSchemaPhysxConvexHullCollisionAPI
PhysxSchemaPhysxConvexHullCollisionAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxConvexHullCollisionAPI();
    }
    return PhysxSchemaPhysxConvexHullCollisionAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxConvexHullCollisionAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxConvexHullCollisionAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxConvexHullCollisionAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxConvexHullCollisionAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxConvexHullCollisionAPI
PhysxSchemaPhysxConvexHullCollisionAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxConvexHullCollisionAPI>()) {
        return PhysxSchemaPhysxConvexHullCollisionAPI(prim);
    }
    return PhysxSchemaPhysxConvexHullCollisionAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxConvexHullCollisionAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxConvexHullCollisionAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxConvexHullCollisionAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxConvexHullCollisionAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxConvexHullCollisionAPI::GetHullVertexLimitAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxConvexHullCollisionHullVertexLimit);
}

UsdAttribute
PhysxSchemaPhysxConvexHullCollisionAPI::CreateHullVertexLimitAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxConvexHullCollisionHullVertexLimit,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxConvexHullCollisionAPI::GetMinThicknessAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxConvexHullCollisionMinThickness);
}

UsdAttribute
PhysxSchemaPhysxConvexHullCollisionAPI::CreateMinThicknessAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxConvexHullCollisionMinThickness,
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
PhysxSchemaPhysxConvexHullCollisionAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxConvexHullCollisionHullVertexLimit,
        PhysxSchemaTokens->physxConvexHullCollisionMinThickness,
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
