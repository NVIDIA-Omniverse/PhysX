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
#include ".//physxParticleIsosurfaceAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxParticleIsosurfaceAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxParticleIsosurfaceAPI::~PhysxSchemaPhysxParticleIsosurfaceAPI()
{
}

/* static */
PhysxSchemaPhysxParticleIsosurfaceAPI
PhysxSchemaPhysxParticleIsosurfaceAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxParticleIsosurfaceAPI();
    }
    return PhysxSchemaPhysxParticleIsosurfaceAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxParticleIsosurfaceAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxParticleIsosurfaceAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxParticleIsosurfaceAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxParticleIsosurfaceAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxParticleIsosurfaceAPI
PhysxSchemaPhysxParticleIsosurfaceAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxParticleIsosurfaceAPI>()) {
        return PhysxSchemaPhysxParticleIsosurfaceAPI(prim);
    }
    return PhysxSchemaPhysxParticleIsosurfaceAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxParticleIsosurfaceAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxParticleIsosurfaceAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxParticleIsosurfaceAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxParticleIsosurfaceAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::GetIsosurfaceEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleIsosurfaceIsosurfaceEnabled);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::CreateIsosurfaceEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleIsosurfaceIsosurfaceEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::GetMaxVerticesAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleIsosurfaceMaxVertices);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::CreateMaxVerticesAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleIsosurfaceMaxVertices,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::GetMaxTrianglesAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleIsosurfaceMaxTriangles);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::CreateMaxTrianglesAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleIsosurfaceMaxTriangles,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::GetMaxSubgridsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleIsosurfaceMaxSubgrids);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::CreateMaxSubgridsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleIsosurfaceMaxSubgrids,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::GetGridSpacingAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleIsosurfaceGridSpacing);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::CreateGridSpacingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleIsosurfaceGridSpacing,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::GetSurfaceDistanceAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleIsosurfaceSurfaceDistance);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::CreateSurfaceDistanceAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleIsosurfaceSurfaceDistance,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::GetGridFilteringPassesAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleIsosurfaceGridFilteringPasses);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::CreateGridFilteringPassesAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleIsosurfaceGridFilteringPasses,
                       SdfValueTypeNames->String,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::GetGridSmoothingRadiusAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleIsosurfaceGridSmoothingRadius);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::CreateGridSmoothingRadiusAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleIsosurfaceGridSmoothingRadius,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::GetNumMeshSmoothingPassesAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleIsosurfaceNumMeshSmoothingPasses);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::CreateNumMeshSmoothingPassesAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleIsosurfaceNumMeshSmoothingPasses,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::GetNumMeshNormalSmoothingPassesAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxParticleIsosurfaceNumMeshNormalSmoothingPasses);
}

UsdAttribute
PhysxSchemaPhysxParticleIsosurfaceAPI::CreateNumMeshNormalSmoothingPassesAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxParticleIsosurfaceNumMeshNormalSmoothingPasses,
                       SdfValueTypeNames->Int,
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
PhysxSchemaPhysxParticleIsosurfaceAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxParticleIsosurfaceIsosurfaceEnabled,
        PhysxSchemaTokens->physxParticleIsosurfaceMaxVertices,
        PhysxSchemaTokens->physxParticleIsosurfaceMaxTriangles,
        PhysxSchemaTokens->physxParticleIsosurfaceMaxSubgrids,
        PhysxSchemaTokens->physxParticleIsosurfaceGridSpacing,
        PhysxSchemaTokens->physxParticleIsosurfaceSurfaceDistance,
        PhysxSchemaTokens->physxParticleIsosurfaceGridFilteringPasses,
        PhysxSchemaTokens->physxParticleIsosurfaceGridSmoothingRadius,
        PhysxSchemaTokens->physxParticleIsosurfaceNumMeshSmoothingPasses,
        PhysxSchemaTokens->physxParticleIsosurfaceNumMeshNormalSmoothingPasses,
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
