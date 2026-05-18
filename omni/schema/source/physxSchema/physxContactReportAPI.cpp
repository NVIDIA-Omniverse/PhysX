//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxContactReportAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxContactReportAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxContactReportAPI::~PhysxSchemaPhysxContactReportAPI()
{
}

/* static */
PhysxSchemaPhysxContactReportAPI
PhysxSchemaPhysxContactReportAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxContactReportAPI();
    }
    return PhysxSchemaPhysxContactReportAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxContactReportAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxContactReportAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxContactReportAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxContactReportAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxContactReportAPI
PhysxSchemaPhysxContactReportAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxContactReportAPI>()) {
        return PhysxSchemaPhysxContactReportAPI(prim);
    }
    return PhysxSchemaPhysxContactReportAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxContactReportAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxContactReportAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxContactReportAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxContactReportAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxContactReportAPI::GetThresholdAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxContactReportThreshold);
}

UsdAttribute
PhysxSchemaPhysxContactReportAPI::CreateThresholdAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxContactReportThreshold,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdRelationship
PhysxSchemaPhysxContactReportAPI::GetReportPairsRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physxContactReportReportPairs);
}

UsdRelationship
PhysxSchemaPhysxContactReportAPI::CreateReportPairsRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physxContactReportReportPairs,
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
PhysxSchemaPhysxContactReportAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxContactReportThreshold,
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
