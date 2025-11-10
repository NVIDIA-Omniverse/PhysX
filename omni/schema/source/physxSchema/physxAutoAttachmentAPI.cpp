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
#include ".//physxAutoAttachmentAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxAutoAttachmentAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxAutoAttachmentAPI::~PhysxSchemaPhysxAutoAttachmentAPI()
{
}

/* static */
PhysxSchemaPhysxAutoAttachmentAPI
PhysxSchemaPhysxAutoAttachmentAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxAutoAttachmentAPI();
    }
    return PhysxSchemaPhysxAutoAttachmentAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxAutoAttachmentAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxAutoAttachmentAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxAutoAttachmentAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxAutoAttachmentAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxAutoAttachmentAPI
PhysxSchemaPhysxAutoAttachmentAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxAutoAttachmentAPI>()) {
        return PhysxSchemaPhysxAutoAttachmentAPI(prim);
    }
    return PhysxSchemaPhysxAutoAttachmentAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxAutoAttachmentAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxAutoAttachmentAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxAutoAttachmentAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxAutoAttachmentAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxAutoAttachmentAPI::GetEnableDeformableVertexAttachmentsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxAutoAttachmentEnableDeformableVertexAttachments);
}

UsdAttribute
PhysxSchemaPhysxAutoAttachmentAPI::CreateEnableDeformableVertexAttachmentsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxAutoAttachmentEnableDeformableVertexAttachments,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxAutoAttachmentAPI::GetDeformableVertexOverlapOffsetAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxAutoAttachmentDeformableVertexOverlapOffset);
}

UsdAttribute
PhysxSchemaPhysxAutoAttachmentAPI::CreateDeformableVertexOverlapOffsetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxAutoAttachmentDeformableVertexOverlapOffset,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxAutoAttachmentAPI::GetEnableRigidSurfaceAttachmentsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxAutoAttachmentEnableRigidSurfaceAttachments);
}

UsdAttribute
PhysxSchemaPhysxAutoAttachmentAPI::CreateEnableRigidSurfaceAttachmentsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxAutoAttachmentEnableRigidSurfaceAttachments,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxAutoAttachmentAPI::GetRigidSurfaceSamplingDistanceAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxAutoAttachmentRigidSurfaceSamplingDistance);
}

UsdAttribute
PhysxSchemaPhysxAutoAttachmentAPI::CreateRigidSurfaceSamplingDistanceAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxAutoAttachmentRigidSurfaceSamplingDistance,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxAutoAttachmentAPI::GetEnableCollisionFilteringAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxAutoAttachmentEnableCollisionFiltering);
}

UsdAttribute
PhysxSchemaPhysxAutoAttachmentAPI::CreateEnableCollisionFilteringAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxAutoAttachmentEnableCollisionFiltering,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxAutoAttachmentAPI::GetCollisionFilteringOffsetAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxAutoAttachmentCollisionFilteringOffset);
}

UsdAttribute
PhysxSchemaPhysxAutoAttachmentAPI::CreateCollisionFilteringOffsetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxAutoAttachmentCollisionFilteringOffset,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxAutoAttachmentAPI::GetEnableDeformableFilteringPairsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxAutoAttachmentEnableDeformableFilteringPairs);
}

UsdAttribute
PhysxSchemaPhysxAutoAttachmentAPI::CreateEnableDeformableFilteringPairsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxAutoAttachmentEnableDeformableFilteringPairs,
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
PhysxSchemaPhysxAutoAttachmentAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxAutoAttachmentEnableDeformableVertexAttachments,
        PhysxSchemaTokens->physxAutoAttachmentDeformableVertexOverlapOffset,
        PhysxSchemaTokens->physxAutoAttachmentEnableRigidSurfaceAttachments,
        PhysxSchemaTokens->physxAutoAttachmentRigidSurfaceSamplingDistance,
        PhysxSchemaTokens->physxAutoAttachmentEnableCollisionFiltering,
        PhysxSchemaTokens->physxAutoAttachmentCollisionFilteringOffset,
        PhysxSchemaTokens->physxAutoAttachmentEnableDeformableFilteringPairs,
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
