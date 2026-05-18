//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxVehicleTireFrictionTable.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleTireFrictionTable,
        TfType::Bases< UsdTyped > >();
    
    // Register the usd prim typename as an alias under UsdSchemaBase. This
    // enables one to call
    // TfType::Find<UsdSchemaBase>().FindDerivedByName("PhysxVehicleTireFrictionTable")
    // to find TfType<PhysxSchemaPhysxVehicleTireFrictionTable>, which is how IsA queries are
    // answered.
    TfType::AddAlias<UsdSchemaBase, PhysxSchemaPhysxVehicleTireFrictionTable>("PhysxVehicleTireFrictionTable");
}

/* virtual */
PhysxSchemaPhysxVehicleTireFrictionTable::~PhysxSchemaPhysxVehicleTireFrictionTable()
{
}

/* static */
PhysxSchemaPhysxVehicleTireFrictionTable
PhysxSchemaPhysxVehicleTireFrictionTable::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleTireFrictionTable();
    }
    return PhysxSchemaPhysxVehicleTireFrictionTable(stage->GetPrimAtPath(path));
}

/* static */
PhysxSchemaPhysxVehicleTireFrictionTable
PhysxSchemaPhysxVehicleTireFrictionTable::Define(
    const UsdStagePtr &stage, const SdfPath &path)
{
    static TfToken usdPrimTypeName("PhysxVehicleTireFrictionTable");
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleTireFrictionTable();
    }
    return PhysxSchemaPhysxVehicleTireFrictionTable(
        stage->DefinePrim(path, usdPrimTypeName));
}

/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleTireFrictionTable::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleTireFrictionTable::schemaKind;
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleTireFrictionTable::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleTireFrictionTable>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleTireFrictionTable::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleTireFrictionTable::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxVehicleTireFrictionTable::GetFrictionValuesAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->frictionValues);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireFrictionTable::CreateFrictionValuesAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->frictionValues,
                       SdfValueTypeNames->FloatArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireFrictionTable::GetDefaultFrictionValueAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->defaultFrictionValue);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireFrictionTable::CreateDefaultFrictionValueAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->defaultFrictionValue,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdRelationship
PhysxSchemaPhysxVehicleTireFrictionTable::GetGroundMaterialsRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->groundMaterials);
}

UsdRelationship
PhysxSchemaPhysxVehicleTireFrictionTable::CreateGroundMaterialsRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->groundMaterials,
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
PhysxSchemaPhysxVehicleTireFrictionTable::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->frictionValues,
        PhysxSchemaTokens->defaultFrictionValue,
    };
    static TfTokenVector allNames =
        _ConcatenateAttributeNames(
            UsdTyped::GetSchemaAttributeNames(true),
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
