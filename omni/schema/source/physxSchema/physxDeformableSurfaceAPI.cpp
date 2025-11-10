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
#include ".//physxDeformableSurfaceAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxDeformableSurfaceAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxDeformableSurfaceAPI::~PhysxSchemaPhysxDeformableSurfaceAPI()
{
}

/* static */
PhysxSchemaPhysxDeformableSurfaceAPI
PhysxSchemaPhysxDeformableSurfaceAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxDeformableSurfaceAPI();
    }
    return PhysxSchemaPhysxDeformableSurfaceAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxDeformableSurfaceAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxDeformableSurfaceAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxDeformableSurfaceAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxDeformableSurfaceAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxDeformableSurfaceAPI
PhysxSchemaPhysxDeformableSurfaceAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxDeformableSurfaceAPI>()) {
        return PhysxSchemaPhysxDeformableSurfaceAPI(prim);
    }
    return PhysxSchemaPhysxDeformableSurfaceAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxDeformableSurfaceAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxDeformableSurfaceAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxDeformableSurfaceAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxDeformableSurfaceAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::GetFlatteningEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSurfaceFlatteningEnabled);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::CreateFlatteningEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSurfaceFlatteningEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::GetBendingStiffnessScaleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSurfaceBendingStiffnessScale);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::CreateBendingStiffnessScaleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSurfaceBendingStiffnessScale,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::GetCollisionPairUpdateFrequencyAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSurfaceCollisionPairUpdateFrequency);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::CreateCollisionPairUpdateFrequencyAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSurfaceCollisionPairUpdateFrequency,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::GetCollisionIterationMultiplierAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSurfaceCollisionIterationMultiplier);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::CreateCollisionIterationMultiplierAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSurfaceCollisionIterationMultiplier,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::GetMaxVelocityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSurfaceMaxVelocity);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::CreateMaxVelocityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSurfaceMaxVelocity,
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
PhysxSchemaPhysxDeformableSurfaceAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxDeformableSurfaceFlatteningEnabled,
        PhysxSchemaTokens->physxDeformableSurfaceBendingStiffnessScale,
        PhysxSchemaTokens->physxDeformableSurfaceCollisionPairUpdateFrequency,
        PhysxSchemaTokens->physxDeformableSurfaceCollisionIterationMultiplier,
        PhysxSchemaTokens->physxDeformableSurfaceMaxVelocity,
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
