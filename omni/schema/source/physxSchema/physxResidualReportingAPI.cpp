//
// Copyright 2016 Pixar
//
// Licensed under the Apache License, Version 2.0 (the "Apache License")
// with the following modification; you may not use this file except in
// compliance with the Apache License and the following modification to it:
// Section 6. Trademarks. is deleted and replaced with:
//
// 6. Trademarks. This License does not grant permission to use the trade
//    names, trademarks, service marks, or product names of the Licensor
//    and its affiliates, except as required to comply with Section 4(c) of
//    the License and to reproduce the content of the NOTICE file.
//
// You may obtain a copy of the Apache License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the Apache License with the above modification is
// distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied. See the Apache License for the specific
// language governing permissions and limitations under the Apache License.
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
