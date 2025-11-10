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
