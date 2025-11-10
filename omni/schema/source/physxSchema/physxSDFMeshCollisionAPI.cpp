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
#include ".//physxSDFMeshCollisionAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxSDFMeshCollisionAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxSDFMeshCollisionAPI::~PhysxSchemaPhysxSDFMeshCollisionAPI()
{
}

/* static */
PhysxSchemaPhysxSDFMeshCollisionAPI
PhysxSchemaPhysxSDFMeshCollisionAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxSDFMeshCollisionAPI();
    }
    return PhysxSchemaPhysxSDFMeshCollisionAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxSDFMeshCollisionAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxSDFMeshCollisionAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxSDFMeshCollisionAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxSDFMeshCollisionAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxSDFMeshCollisionAPI
PhysxSchemaPhysxSDFMeshCollisionAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxSDFMeshCollisionAPI>()) {
        return PhysxSchemaPhysxSDFMeshCollisionAPI(prim);
    }
    return PhysxSchemaPhysxSDFMeshCollisionAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxSDFMeshCollisionAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxSDFMeshCollisionAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxSDFMeshCollisionAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxSDFMeshCollisionAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxSDFMeshCollisionAPI::GetSdfResolutionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSDFMeshCollisionSdfResolution);
}

UsdAttribute
PhysxSchemaPhysxSDFMeshCollisionAPI::CreateSdfResolutionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSDFMeshCollisionSdfResolution,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSDFMeshCollisionAPI::GetSdfSubgridResolutionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSDFMeshCollisionSdfSubgridResolution);
}

UsdAttribute
PhysxSchemaPhysxSDFMeshCollisionAPI::CreateSdfSubgridResolutionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSDFMeshCollisionSdfSubgridResolution,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSDFMeshCollisionAPI::GetSdfBitsPerSubgridPixelAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSDFMeshCollisionSdfBitsPerSubgridPixel);
}

UsdAttribute
PhysxSchemaPhysxSDFMeshCollisionAPI::CreateSdfBitsPerSubgridPixelAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSDFMeshCollisionSdfBitsPerSubgridPixel,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSDFMeshCollisionAPI::GetSdfNarrowBandThicknessAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSDFMeshCollisionSdfNarrowBandThickness);
}

UsdAttribute
PhysxSchemaPhysxSDFMeshCollisionAPI::CreateSdfNarrowBandThicknessAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSDFMeshCollisionSdfNarrowBandThickness,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSDFMeshCollisionAPI::GetSdfMarginAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSDFMeshCollisionSdfMargin);
}

UsdAttribute
PhysxSchemaPhysxSDFMeshCollisionAPI::CreateSdfMarginAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSDFMeshCollisionSdfMargin,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSDFMeshCollisionAPI::GetSdfEnableRemeshingAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSDFMeshCollisionSdfEnableRemeshing);
}

UsdAttribute
PhysxSchemaPhysxSDFMeshCollisionAPI::CreateSdfEnableRemeshingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSDFMeshCollisionSdfEnableRemeshing,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSDFMeshCollisionAPI::GetSdfTriangleCountReductionFactorAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSDFMeshCollisionSdfTriangleCountReductionFactor);
}

UsdAttribute
PhysxSchemaPhysxSDFMeshCollisionAPI::CreateSdfTriangleCountReductionFactorAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSDFMeshCollisionSdfTriangleCountReductionFactor,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityUniform,
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
PhysxSchemaPhysxSDFMeshCollisionAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxSDFMeshCollisionSdfResolution,
        PhysxSchemaTokens->physxSDFMeshCollisionSdfSubgridResolution,
        PhysxSchemaTokens->physxSDFMeshCollisionSdfBitsPerSubgridPixel,
        PhysxSchemaTokens->physxSDFMeshCollisionSdfNarrowBandThickness,
        PhysxSchemaTokens->physxSDFMeshCollisionSdfMargin,
        PhysxSchemaTokens->physxSDFMeshCollisionSdfEnableRemeshing,
        PhysxSchemaTokens->physxSDFMeshCollisionSdfTriangleCountReductionFactor,
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
