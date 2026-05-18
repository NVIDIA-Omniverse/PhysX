//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//tetrahedralMesh.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaTetrahedralMesh,
        TfType::Bases< UsdGeomPointBased > >();
    
    // Register the usd prim typename as an alias under UsdSchemaBase. This
    // enables one to call
    // TfType::Find<UsdSchemaBase>().FindDerivedByName("TetrahedralMesh")
    // to find TfType<PhysxSchemaTetrahedralMesh>, which is how IsA queries are
    // answered.
    TfType::AddAlias<UsdSchemaBase, PhysxSchemaTetrahedralMesh>("TetrahedralMesh");
}

/* virtual */
PhysxSchemaTetrahedralMesh::~PhysxSchemaTetrahedralMesh()
{
}

/* static */
PhysxSchemaTetrahedralMesh
PhysxSchemaTetrahedralMesh::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaTetrahedralMesh();
    }
    return PhysxSchemaTetrahedralMesh(stage->GetPrimAtPath(path));
}

/* static */
PhysxSchemaTetrahedralMesh
PhysxSchemaTetrahedralMesh::Define(
    const UsdStagePtr &stage, const SdfPath &path)
{
    static TfToken usdPrimTypeName("TetrahedralMesh");
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaTetrahedralMesh();
    }
    return PhysxSchemaTetrahedralMesh(
        stage->DefinePrim(path, usdPrimTypeName));
}

/* virtual */
UsdSchemaKind PhysxSchemaTetrahedralMesh::_GetSchemaKind() const
{
    return PhysxSchemaTetrahedralMesh::schemaKind;
}

/* static */
const TfType &
PhysxSchemaTetrahedralMesh::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaTetrahedralMesh>();
    return tfType;
}

/* static */
bool 
PhysxSchemaTetrahedralMesh::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaTetrahedralMesh::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaTetrahedralMesh::GetIndicesAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->indices);
}

UsdAttribute
PhysxSchemaTetrahedralMesh::CreateIndicesAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->indices,
                       SdfValueTypeNames->IntArray,
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
PhysxSchemaTetrahedralMesh::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->indices,
    };
    static TfTokenVector allNames =
        _ConcatenateAttributeNames(
            UsdGeomPointBased::GetSchemaAttributeNames(true),
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
