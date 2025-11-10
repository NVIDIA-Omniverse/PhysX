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
#include ".//physxDeformableBodyAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxDeformableBodyAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxDeformableBodyAPI::~PhysxSchemaPhysxDeformableBodyAPI()
{
}

/* static */
PhysxSchemaPhysxDeformableBodyAPI
PhysxSchemaPhysxDeformableBodyAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxDeformableBodyAPI();
    }
    return PhysxSchemaPhysxDeformableBodyAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxDeformableBodyAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxDeformableBodyAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxDeformableBodyAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxDeformableBodyAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxDeformableBodyAPI
PhysxSchemaPhysxDeformableBodyAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxDeformableBodyAPI>()) {
        return PhysxSchemaPhysxDeformableBodyAPI(prim);
    }
    return PhysxSchemaPhysxDeformableBodyAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxDeformableBodyAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxDeformableBodyAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxDeformableBodyAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxDeformableBodyAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyAPI::GetDisableGravityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableDisableGravity);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyAPI::CreateDisableGravityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableDisableGravity,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyAPI::GetCollisionIndicesAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableCollisionIndices);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyAPI::CreateCollisionIndicesAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableCollisionIndices,
                       SdfValueTypeNames->IntArray,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyAPI::GetCollisionPointsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableCollisionPoints);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyAPI::CreateCollisionPointsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableCollisionPoints,
                       SdfValueTypeNames->Point3fArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyAPI::GetCollisionRestPointsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableCollisionRestPoints);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyAPI::CreateCollisionRestPointsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableCollisionRestPoints,
                       SdfValueTypeNames->Point3fArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyAPI::GetSimulationPointsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSimulationPoints);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyAPI::CreateSimulationPointsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSimulationPoints,
                       SdfValueTypeNames->Point3fArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyAPI::GetSimulationRestPointsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSimulationRestPoints);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyAPI::CreateSimulationRestPointsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSimulationRestPoints,
                       SdfValueTypeNames->Point3fArray,
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
PhysxSchemaPhysxDeformableBodyAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxDeformableDisableGravity,
        PhysxSchemaTokens->physxDeformableCollisionIndices,
        PhysxSchemaTokens->physxDeformableCollisionPoints,
        PhysxSchemaTokens->physxDeformableCollisionRestPoints,
        PhysxSchemaTokens->physxDeformableSimulationPoints,
        PhysxSchemaTokens->physxDeformableSimulationRestPoints,
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
