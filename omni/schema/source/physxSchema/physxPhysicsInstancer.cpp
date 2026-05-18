//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxPhysicsInstancer.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxPhysicsInstancer,
        TfType::Bases< UsdGeomImageable > >();
    
    // Register the usd prim typename as an alias under UsdSchemaBase. This
    // enables one to call
    // TfType::Find<UsdSchemaBase>().FindDerivedByName("PhysxPhysicsInstancer")
    // to find TfType<PhysxSchemaPhysxPhysicsInstancer>, which is how IsA queries are
    // answered.
    TfType::AddAlias<UsdSchemaBase, PhysxSchemaPhysxPhysicsInstancer>("PhysxPhysicsInstancer");
}

/* virtual */
PhysxSchemaPhysxPhysicsInstancer::~PhysxSchemaPhysxPhysicsInstancer()
{
}

/* static */
PhysxSchemaPhysxPhysicsInstancer
PhysxSchemaPhysxPhysicsInstancer::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxPhysicsInstancer();
    }
    return PhysxSchemaPhysxPhysicsInstancer(stage->GetPrimAtPath(path));
}

/* static */
PhysxSchemaPhysxPhysicsInstancer
PhysxSchemaPhysxPhysicsInstancer::Define(
    const UsdStagePtr &stage, const SdfPath &path)
{
    static TfToken usdPrimTypeName("PhysxPhysicsInstancer");
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxPhysicsInstancer();
    }
    return PhysxSchemaPhysxPhysicsInstancer(
        stage->DefinePrim(path, usdPrimTypeName));
}

/* virtual */
UsdSchemaKind PhysxSchemaPhysxPhysicsInstancer::_GetSchemaKind() const
{
    return PhysxSchemaPhysxPhysicsInstancer::schemaKind;
}

/* static */
const TfType &
PhysxSchemaPhysxPhysicsInstancer::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxPhysicsInstancer>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxPhysicsInstancer::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxPhysicsInstancer::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxPhysicsInstancer::GetPhysicsProtoIndicesAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physicsProtoIndices);
}

UsdAttribute
PhysxSchemaPhysxPhysicsInstancer::CreatePhysicsProtoIndicesAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physicsProtoIndices,
                       SdfValueTypeNames->IntArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdRelationship
PhysxSchemaPhysxPhysicsInstancer::GetPhysicsPrototypesRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physicsPrototypes);
}

UsdRelationship
PhysxSchemaPhysxPhysicsInstancer::CreatePhysicsPrototypesRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physicsPrototypes,
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
PhysxSchemaPhysxPhysicsInstancer::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physicsProtoIndices,
    };
    static TfTokenVector allNames =
        _ConcatenateAttributeNames(
            UsdGeomImageable::GetSchemaAttributeNames(true),
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
