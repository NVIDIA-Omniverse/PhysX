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
#include ".//physxSphereFillCollisionAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxSphereFillCollisionAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxSphereFillCollisionAPI::~PhysxSchemaPhysxSphereFillCollisionAPI()
{
}

/* static */
PhysxSchemaPhysxSphereFillCollisionAPI
PhysxSchemaPhysxSphereFillCollisionAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxSphereFillCollisionAPI();
    }
    return PhysxSchemaPhysxSphereFillCollisionAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxSphereFillCollisionAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxSphereFillCollisionAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxSphereFillCollisionAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxSphereFillCollisionAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxSphereFillCollisionAPI
PhysxSchemaPhysxSphereFillCollisionAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxSphereFillCollisionAPI>()) {
        return PhysxSchemaPhysxSphereFillCollisionAPI(prim);
    }
    return PhysxSchemaPhysxSphereFillCollisionAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxSphereFillCollisionAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxSphereFillCollisionAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxSphereFillCollisionAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxSphereFillCollisionAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxSphereFillCollisionAPI::GetMaxSpheresAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSphereFillCollisionMaxSpheres);
}

UsdAttribute
PhysxSchemaPhysxSphereFillCollisionAPI::CreateMaxSpheresAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSphereFillCollisionMaxSpheres,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSphereFillCollisionAPI::GetVoxelResolutionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSphereFillCollisionVoxelResolution);
}

UsdAttribute
PhysxSchemaPhysxSphereFillCollisionAPI::CreateVoxelResolutionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSphereFillCollisionVoxelResolution,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSphereFillCollisionAPI::GetSeedCountAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSphereFillCollisionSeedCount);
}

UsdAttribute
PhysxSchemaPhysxSphereFillCollisionAPI::CreateSeedCountAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSphereFillCollisionSeedCount,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxSphereFillCollisionAPI::GetFillModeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxSphereFillCollisionFillMode);
}

UsdAttribute
PhysxSchemaPhysxSphereFillCollisionAPI::CreateFillModeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxSphereFillCollisionFillMode,
                       SdfValueTypeNames->Token,
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
PhysxSchemaPhysxSphereFillCollisionAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxSphereFillCollisionMaxSpheres,
        PhysxSchemaTokens->physxSphereFillCollisionVoxelResolution,
        PhysxSchemaTokens->physxSphereFillCollisionSeedCount,
        PhysxSchemaTokens->physxSphereFillCollisionFillMode,
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
