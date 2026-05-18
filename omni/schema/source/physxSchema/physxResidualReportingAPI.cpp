//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxResidualReportingAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxResidualReportingAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxResidualReportingAPI::~PhysxSchemaPhysxResidualReportingAPI()
{
}

/* static */
PhysxSchemaPhysxResidualReportingAPI
PhysxSchemaPhysxResidualReportingAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxResidualReportingAPI();
    }
    return PhysxSchemaPhysxResidualReportingAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxResidualReportingAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxResidualReportingAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxResidualReportingAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxResidualReportingAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxResidualReportingAPI
PhysxSchemaPhysxResidualReportingAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxResidualReportingAPI>()) {
        return PhysxSchemaPhysxResidualReportingAPI(prim);
    }
    return PhysxSchemaPhysxResidualReportingAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxResidualReportingAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxResidualReportingAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxResidualReportingAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxResidualReportingAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxResidualReportingAPI::GetPhysxResidualReportingRmsResidualPositionIterationAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxResidualReportingRmsResidualPositionIteration);
}

UsdAttribute
PhysxSchemaPhysxResidualReportingAPI::CreatePhysxResidualReportingRmsResidualPositionIterationAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxResidualReportingRmsResidualPositionIteration,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxResidualReportingAPI::GetPhysxResidualReportingMaxResidualPositionIterationAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxResidualReportingMaxResidualPositionIteration);
}

UsdAttribute
PhysxSchemaPhysxResidualReportingAPI::CreatePhysxResidualReportingMaxResidualPositionIterationAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxResidualReportingMaxResidualPositionIteration,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxResidualReportingAPI::GetPhysxResidualReportingRmsResidualVelocityIterationAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxResidualReportingRmsResidualVelocityIteration);
}

UsdAttribute
PhysxSchemaPhysxResidualReportingAPI::CreatePhysxResidualReportingRmsResidualVelocityIterationAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxResidualReportingRmsResidualVelocityIteration,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxResidualReportingAPI::GetPhysxResidualReportingMaxResidualVelocityIterationAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxResidualReportingMaxResidualVelocityIteration);
}

UsdAttribute
PhysxSchemaPhysxResidualReportingAPI::CreatePhysxResidualReportingMaxResidualVelocityIterationAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxResidualReportingMaxResidualVelocityIteration,
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
PhysxSchemaPhysxResidualReportingAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxResidualReportingRmsResidualPositionIteration,
        PhysxSchemaTokens->physxResidualReportingMaxResidualPositionIteration,
        PhysxSchemaTokens->physxResidualReportingRmsResidualVelocityIteration,
        PhysxSchemaTokens->physxResidualReportingMaxResidualVelocityIteration,
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
