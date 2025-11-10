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
#include ".//physxConvexDecompositionCollisionAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxConvexDecompositionCollisionAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxConvexDecompositionCollisionAPI::~PhysxSchemaPhysxConvexDecompositionCollisionAPI()
{
}

/* static */
PhysxSchemaPhysxConvexDecompositionCollisionAPI
PhysxSchemaPhysxConvexDecompositionCollisionAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxConvexDecompositionCollisionAPI();
    }
    return PhysxSchemaPhysxConvexDecompositionCollisionAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxConvexDecompositionCollisionAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxConvexDecompositionCollisionAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxConvexDecompositionCollisionAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxConvexDecompositionCollisionAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxConvexDecompositionCollisionAPI
PhysxSchemaPhysxConvexDecompositionCollisionAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxConvexDecompositionCollisionAPI>()) {
        return PhysxSchemaPhysxConvexDecompositionCollisionAPI(prim);
    }
    return PhysxSchemaPhysxConvexDecompositionCollisionAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxConvexDecompositionCollisionAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxConvexDecompositionCollisionAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxConvexDecompositionCollisionAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxConvexDecompositionCollisionAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxConvexDecompositionCollisionAPI::GetHullVertexLimitAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxConvexDecompositionCollisionHullVertexLimit);
}

UsdAttribute
PhysxSchemaPhysxConvexDecompositionCollisionAPI::CreateHullVertexLimitAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxConvexDecompositionCollisionHullVertexLimit,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxConvexDecompositionCollisionAPI::GetMaxConvexHullsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxConvexDecompositionCollisionMaxConvexHulls);
}

UsdAttribute
PhysxSchemaPhysxConvexDecompositionCollisionAPI::CreateMaxConvexHullsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxConvexDecompositionCollisionMaxConvexHulls,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxConvexDecompositionCollisionAPI::GetMinThicknessAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxConvexDecompositionCollisionMinThickness);
}

UsdAttribute
PhysxSchemaPhysxConvexDecompositionCollisionAPI::CreateMinThicknessAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxConvexDecompositionCollisionMinThickness,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxConvexDecompositionCollisionAPI::GetVoxelResolutionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxConvexDecompositionCollisionVoxelResolution);
}

UsdAttribute
PhysxSchemaPhysxConvexDecompositionCollisionAPI::CreateVoxelResolutionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxConvexDecompositionCollisionVoxelResolution,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxConvexDecompositionCollisionAPI::GetErrorPercentageAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxConvexDecompositionCollisionErrorPercentage);
}

UsdAttribute
PhysxSchemaPhysxConvexDecompositionCollisionAPI::CreateErrorPercentageAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxConvexDecompositionCollisionErrorPercentage,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxConvexDecompositionCollisionAPI::GetShrinkWrapAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxConvexDecompositionCollisionShrinkWrap);
}

UsdAttribute
PhysxSchemaPhysxConvexDecompositionCollisionAPI::CreateShrinkWrapAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxConvexDecompositionCollisionShrinkWrap,
                       SdfValueTypeNames->Bool,
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
PhysxSchemaPhysxConvexDecompositionCollisionAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxConvexDecompositionCollisionHullVertexLimit,
        PhysxSchemaTokens->physxConvexDecompositionCollisionMaxConvexHulls,
        PhysxSchemaTokens->physxConvexDecompositionCollisionMinThickness,
        PhysxSchemaTokens->physxConvexDecompositionCollisionVoxelResolution,
        PhysxSchemaTokens->physxConvexDecompositionCollisionErrorPercentage,
        PhysxSchemaTokens->physxConvexDecompositionCollisionShrinkWrap,
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
