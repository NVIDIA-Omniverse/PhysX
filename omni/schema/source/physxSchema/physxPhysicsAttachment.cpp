//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxPhysicsAttachment.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxPhysicsAttachment,
        TfType::Bases< UsdTyped > >();
    
    // Register the usd prim typename as an alias under UsdSchemaBase. This
    // enables one to call
    // TfType::Find<UsdSchemaBase>().FindDerivedByName("PhysxPhysicsAttachment")
    // to find TfType<PhysxSchemaPhysxPhysicsAttachment>, which is how IsA queries are
    // answered.
    TfType::AddAlias<UsdSchemaBase, PhysxSchemaPhysxPhysicsAttachment>("PhysxPhysicsAttachment");
}

/* virtual */
PhysxSchemaPhysxPhysicsAttachment::~PhysxSchemaPhysxPhysicsAttachment()
{
}

/* static */
PhysxSchemaPhysxPhysicsAttachment
PhysxSchemaPhysxPhysicsAttachment::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxPhysicsAttachment();
    }
    return PhysxSchemaPhysxPhysicsAttachment(stage->GetPrimAtPath(path));
}

/* static */
PhysxSchemaPhysxPhysicsAttachment
PhysxSchemaPhysxPhysicsAttachment::Define(
    const UsdStagePtr &stage, const SdfPath &path)
{
    static TfToken usdPrimTypeName("PhysxPhysicsAttachment");
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxPhysicsAttachment();
    }
    return PhysxSchemaPhysxPhysicsAttachment(
        stage->DefinePrim(path, usdPrimTypeName));
}

/* virtual */
UsdSchemaKind PhysxSchemaPhysxPhysicsAttachment::_GetSchemaKind() const
{
    return PhysxSchemaPhysxPhysicsAttachment::schemaKind;
}

/* static */
const TfType &
PhysxSchemaPhysxPhysicsAttachment::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxPhysicsAttachment>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxPhysicsAttachment::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxPhysicsAttachment::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxPhysicsAttachment::GetAttachmentEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->attachmentEnabled);
}

UsdAttribute
PhysxSchemaPhysxPhysicsAttachment::CreateAttachmentEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->attachmentEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPhysicsAttachment::GetPoints0Attr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->points0);
}

UsdAttribute
PhysxSchemaPhysxPhysicsAttachment::CreatePoints0Attr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->points0,
                       SdfValueTypeNames->Point3fArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPhysicsAttachment::GetPoints1Attr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->points1);
}

UsdAttribute
PhysxSchemaPhysxPhysicsAttachment::CreatePoints1Attr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->points1,
                       SdfValueTypeNames->Point3fArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPhysicsAttachment::GetCollisionFilterIndices0Attr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->collisionFilterIndices0);
}

UsdAttribute
PhysxSchemaPhysxPhysicsAttachment::CreateCollisionFilterIndices0Attr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->collisionFilterIndices0,
                       SdfValueTypeNames->UIntArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPhysicsAttachment::GetFilterType0Attr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->filterType0);
}

UsdAttribute
PhysxSchemaPhysxPhysicsAttachment::CreateFilterType0Attr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->filterType0,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPhysicsAttachment::GetCollisionFilterIndices1Attr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->collisionFilterIndices1);
}

UsdAttribute
PhysxSchemaPhysxPhysicsAttachment::CreateCollisionFilterIndices1Attr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->collisionFilterIndices1,
                       SdfValueTypeNames->UIntArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPhysicsAttachment::GetFilterType1Attr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->filterType1);
}

UsdAttribute
PhysxSchemaPhysxPhysicsAttachment::CreateFilterType1Attr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->filterType1,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdRelationship
PhysxSchemaPhysxPhysicsAttachment::GetActor0Rel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->actor0);
}

UsdRelationship
PhysxSchemaPhysxPhysicsAttachment::CreateActor0Rel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->actor0,
                       /* custom = */ false);
}

UsdRelationship
PhysxSchemaPhysxPhysicsAttachment::GetActor1Rel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->actor1);
}

UsdRelationship
PhysxSchemaPhysxPhysicsAttachment::CreateActor1Rel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->actor1,
                       /* custom = */ false);
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
PhysxSchemaPhysxPhysicsAttachment::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->attachmentEnabled,
        PhysxSchemaTokens->points0,
        PhysxSchemaTokens->points1,
        PhysxSchemaTokens->collisionFilterIndices0,
        PhysxSchemaTokens->filterType0,
        PhysxSchemaTokens->collisionFilterIndices1,
        PhysxSchemaTokens->filterType1,
    };
    static TfTokenVector allNames =
        _ConcatenateAttributeNames(
            UsdTyped::GetSchemaAttributeNames(true),
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

PXR_NAMESPACE_OPEN_SCOPE

UsdAttribute
PhysxSchemaPhysxPhysicsAttachment::GetPointsAttr(int index) const
{
    if (index < 0 || index > 1)
        return UsdAttribute();
    return GetPrim().GetAttribute(index == 0 ? PhysxSchemaTokens->points0 : PhysxSchemaTokens->points1);
}

UsdAttribute
PhysxSchemaPhysxPhysicsAttachment::GetCollisionFilterIndicesAttr(int index) const
{
    if (index < 0 || index > 1)
        return UsdAttribute();
    return GetPrim().GetAttribute(index == 0 ? PhysxSchemaTokens->collisionFilterIndices0 : PhysxSchemaTokens->collisionFilterIndices1);
}

UsdAttribute
PhysxSchemaPhysxPhysicsAttachment::GetFilterTypeAttr(int index) const
{
    if (index < 0 || index > 1)
        return UsdAttribute();
    return GetPrim().GetAttribute(index == 0 ? PhysxSchemaTokens->filterType0 : PhysxSchemaTokens->filterType1);
}

UsdRelationship
PhysxSchemaPhysxPhysicsAttachment::GetActorRel(int index) const
{
    if (index < 0 || index > 1)
        return UsdRelationship();
    return GetPrim().GetRelationship(index == 0 ? PhysxSchemaTokens->actor0 : PhysxSchemaTokens->actor1);
}

PXR_NAMESPACE_CLOSE_SCOPE
