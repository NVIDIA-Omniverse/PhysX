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
#include ".//physxDeformableBodyMaterialAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxDeformableBodyMaterialAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxDeformableBodyMaterialAPI::~PhysxSchemaPhysxDeformableBodyMaterialAPI()
{
}

/* static */
PhysxSchemaPhysxDeformableBodyMaterialAPI
PhysxSchemaPhysxDeformableBodyMaterialAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxDeformableBodyMaterialAPI();
    }
    return PhysxSchemaPhysxDeformableBodyMaterialAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxDeformableBodyMaterialAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxDeformableBodyMaterialAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxDeformableBodyMaterialAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxDeformableBodyMaterialAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxDeformableBodyMaterialAPI
PhysxSchemaPhysxDeformableBodyMaterialAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxDeformableBodyMaterialAPI>()) {
        return PhysxSchemaPhysxDeformableBodyMaterialAPI(prim);
    }
    return PhysxSchemaPhysxDeformableBodyMaterialAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxDeformableBodyMaterialAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxDeformableBodyMaterialAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxDeformableBodyMaterialAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxDeformableBodyMaterialAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyMaterialAPI::GetDensityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableBodyMaterialDensity);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyMaterialAPI::CreateDensityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableBodyMaterialDensity,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyMaterialAPI::GetElasticityDampingAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableBodyMaterialElasticityDamping);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyMaterialAPI::CreateElasticityDampingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableBodyMaterialElasticityDamping,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyMaterialAPI::GetDynamicFrictionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableBodyMaterialDynamicFriction);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyMaterialAPI::CreateDynamicFrictionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableBodyMaterialDynamicFriction,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyMaterialAPI::GetYoungsModulusAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableBodyMaterialYoungsModulus);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyMaterialAPI::CreateYoungsModulusAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableBodyMaterialYoungsModulus,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyMaterialAPI::GetPoissonsRatioAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableBodyMaterialPoissonsRatio);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyMaterialAPI::CreatePoissonsRatioAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableBodyMaterialPoissonsRatio,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyMaterialAPI::GetDampingScaleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableBodyMaterialDampingScale);
}

UsdAttribute
PhysxSchemaPhysxDeformableBodyMaterialAPI::CreateDampingScaleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableBodyMaterialDampingScale,
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
PhysxSchemaPhysxDeformableBodyMaterialAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxDeformableBodyMaterialDensity,
        PhysxSchemaTokens->physxDeformableBodyMaterialElasticityDamping,
        PhysxSchemaTokens->physxDeformableBodyMaterialDynamicFriction,
        PhysxSchemaTokens->physxDeformableBodyMaterialYoungsModulus,
        PhysxSchemaTokens->physxDeformableBodyMaterialPoissonsRatio,
        PhysxSchemaTokens->physxDeformableBodyMaterialDampingScale,
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
