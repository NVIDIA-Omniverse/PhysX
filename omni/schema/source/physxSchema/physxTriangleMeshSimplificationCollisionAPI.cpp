//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxTriangleMeshSimplificationCollisionAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI::~PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI()
{
}

/* static */
PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI
PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI();
    }
    return PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI
PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI>()) {
        return PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI(prim);
    }
    return PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI::GetSimplificationMetricAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxTriangleMeshSimplificationCollisionMetric);
}

UsdAttribute
PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI::CreateSimplificationMetricAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxTriangleMeshSimplificationCollisionMetric,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI::GetWeldToleranceAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxTriangleMeshSimplificationCollisionWeldTolerance);
}

UsdAttribute
PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI::CreateWeldToleranceAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxTriangleMeshSimplificationCollisionWeldTolerance,
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
PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxTriangleMeshSimplificationCollisionMetric,
        PhysxSchemaTokens->physxTriangleMeshSimplificationCollisionWeldTolerance,
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
