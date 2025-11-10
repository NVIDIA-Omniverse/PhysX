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
#include ".//physxSurfaceVelocityAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxSurfaceVelocityAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxSurfaceVelocityAPI::~PhysxSchemaPhysxSurfaceVelocityAPI()
{
}

/* static */
PhysxSchemaPhysxSurfaceVelocityAPI
PhysxSchemaPhysxSurfaceVelocityAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxSurfaceVelocityAPI();
    }
    return PhysxSchemaPhysxSurfaceVelocityAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxSurfaceVelocityAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxSurfaceVelocityAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxSurfaceVelocityAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxSurfaceVelocityAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxSurfaceVelocityAPI
PhysxSchemaPhysxSurfaceVelocityAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxSurfaceVelocityAPI>()) {
        return PhysxSchemaPhysxSurfaceVelocityAPI(prim);
    }
    return PhysxSchemaPhysxSurfaceVelocityAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxSurfaceVelocityAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxSurfaceVelocityAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxSurfaceVelocityAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxSurfaceVelocityAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxSurfaceVelocityAPI::GetSurfaceVelocityEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocityEnabled);
}

UsdAttribute
PhysxSchemaPhysxSurfaceVelocityAPI::CreateSurfaceVelocityEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocityEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSurfaceVelocityAPI::GetSurfaceVelocityLocalSpaceAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocityLocalSpace);
}

UsdAttribute
PhysxSchemaPhysxSurfaceVelocityAPI::CreateSurfaceVelocityLocalSpaceAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocityLocalSpace,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSurfaceVelocityAPI::GetSurfaceVelocityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocity);
}

UsdAttribute
PhysxSchemaPhysxSurfaceVelocityAPI::CreateSurfaceVelocityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocity,
                       SdfValueTypeNames->Vector3f,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSurfaceVelocityAPI::GetSurfaceAngularVelocityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSurfaceVelocitySurfaceAngularVelocity);
}

UsdAttribute
PhysxSchemaPhysxSurfaceVelocityAPI::CreateSurfaceAngularVelocityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSurfaceVelocitySurfaceAngularVelocity,
                       SdfValueTypeNames->Vector3f,
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
PhysxSchemaPhysxSurfaceVelocityAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocityEnabled,
        PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocityLocalSpace,
        PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocity,
        PhysxSchemaTokens->physxSurfaceVelocitySurfaceAngularVelocity,
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
