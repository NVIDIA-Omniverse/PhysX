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
#include ".//physxDeformableSurfaceMaterialAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxDeformableSurfaceMaterialAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::~PhysxSchemaPhysxDeformableSurfaceMaterialAPI()
{
}

/* static */
PhysxSchemaPhysxDeformableSurfaceMaterialAPI
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxDeformableSurfaceMaterialAPI();
    }
    return PhysxSchemaPhysxDeformableSurfaceMaterialAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxDeformableSurfaceMaterialAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxDeformableSurfaceMaterialAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxDeformableSurfaceMaterialAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxDeformableSurfaceMaterialAPI
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxDeformableSurfaceMaterialAPI>()) {
        return PhysxSchemaPhysxDeformableSurfaceMaterialAPI(prim);
    }
    return PhysxSchemaPhysxDeformableSurfaceMaterialAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxDeformableSurfaceMaterialAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::GetDensityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSurfaceMaterialDensity);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::CreateDensityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSurfaceMaterialDensity,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::GetDynamicFrictionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSurfaceMaterialDynamicFriction);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::CreateDynamicFrictionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSurfaceMaterialDynamicFriction,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::GetYoungsModulusAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSurfaceMaterialYoungsModulus);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::CreateYoungsModulusAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSurfaceMaterialYoungsModulus,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::GetPoissonsRatioAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSurfaceMaterialPoissonsRatio);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::CreatePoissonsRatioAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSurfaceMaterialPoissonsRatio,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::GetThicknessAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSurfaceMaterialThickness);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::CreateThicknessAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSurfaceMaterialThickness,
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
PhysxSchemaPhysxDeformableSurfaceMaterialAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxDeformableSurfaceMaterialDensity,
        PhysxSchemaTokens->physxDeformableSurfaceMaterialDynamicFriction,
        PhysxSchemaTokens->physxDeformableSurfaceMaterialYoungsModulus,
        PhysxSchemaTokens->physxDeformableSurfaceMaterialPoissonsRatio,
        PhysxSchemaTokens->physxDeformableSurfaceMaterialThickness,
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
