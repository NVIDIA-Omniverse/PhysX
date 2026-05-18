//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxTendonAttachmentAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxTendonAttachmentAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxTendonAttachmentAPI::~PhysxSchemaPhysxTendonAttachmentAPI()
{
}

/* static */
PhysxSchemaPhysxTendonAttachmentAPI
PhysxSchemaPhysxTendonAttachmentAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxTendonAttachmentAPI();
    }
    TfToken name;
    if (!IsPhysxTendonAttachmentAPIPath(path, &name)) {
        TF_CODING_ERROR("Invalid physxTendon path <%s>.", path.GetText());
        return PhysxSchemaPhysxTendonAttachmentAPI();
    }
    return PhysxSchemaPhysxTendonAttachmentAPI(stage->GetPrimAtPath(path.GetPrimPath()), name);
}

PhysxSchemaPhysxTendonAttachmentAPI
PhysxSchemaPhysxTendonAttachmentAPI::Get(const UsdPrim &prim, const TfToken &name)
{
    return PhysxSchemaPhysxTendonAttachmentAPI(prim, name);
}

/* static */
std::vector<PhysxSchemaPhysxTendonAttachmentAPI>
PhysxSchemaPhysxTendonAttachmentAPI::GetAll(const UsdPrim &prim)
{
    std::vector<PhysxSchemaPhysxTendonAttachmentAPI> schemas;
    
    for (const auto &schemaName :
         UsdAPISchemaBase::_GetMultipleApplyInstanceNames(prim, _GetStaticTfType())) {
        schemas.emplace_back(prim, schemaName);
    }

    return schemas;
}


/* static */
bool 
PhysxSchemaPhysxTendonAttachmentAPI::IsSchemaPropertyBaseName(const TfToken &baseName)
{
    static TfTokenVector attrsAndRels = {
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Gearing),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_LocalPos),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_ParentAttachment),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_ParentLink),
    };

    return find(attrsAndRels.begin(), attrsAndRels.end(), baseName)
            != attrsAndRels.end();
}

/* static */
bool
PhysxSchemaPhysxTendonAttachmentAPI::IsPhysxTendonAttachmentAPIPath(
    const SdfPath &path, TfToken *name)
{
    if (!path.IsPropertyPath()) {
        return false;
    }

    std::string propertyName = path.GetName();
    TfTokenVector tokens = SdfPath::TokenizeIdentifierAsTokens(propertyName);

    // The baseName of the  path can't be one of the 
    // schema properties. We should validate this in the creation (or apply)
    // API.
    TfToken baseName = *tokens.rbegin();
    if (IsSchemaPropertyBaseName(baseName)) {
        return false;
    }

    if (tokens.size() >= 2
        && tokens[0] == PhysxSchemaTokens->physxTendon) {
        *name = TfToken(propertyName.substr(
           PhysxSchemaTokens->physxTendon.GetString().size() + 1));
        return true;
    }

    return false;
}

/* virtual */
UsdSchemaKind PhysxSchemaPhysxTendonAttachmentAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxTendonAttachmentAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxTendonAttachmentAPI::CanApply(
    const UsdPrim &prim, const TfToken &name, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxTendonAttachmentAPI>(name, whyNot);
}

/* static */
PhysxSchemaPhysxTendonAttachmentAPI
PhysxSchemaPhysxTendonAttachmentAPI::Apply(const UsdPrim &prim, const TfToken &name)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxTendonAttachmentAPI>(name)) {
        return PhysxSchemaPhysxTendonAttachmentAPI(prim, name);
    }
    return PhysxSchemaPhysxTendonAttachmentAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxTendonAttachmentAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxTendonAttachmentAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxTendonAttachmentAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxTendonAttachmentAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

/// Returns the property name prefixed with the correct namespace prefix, which
/// is composed of the the API's propertyNamespacePrefix metadata and the
/// instance name of the API.
static inline
TfToken
_GetNamespacedPropertyName(const TfToken instanceName, const TfToken propName)
{
    return UsdSchemaRegistry::MakeMultipleApplyNameInstance(propName, instanceName);
}

UsdAttribute
PhysxSchemaPhysxTendonAttachmentAPI::GetGearingAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Gearing));
}

UsdAttribute
PhysxSchemaPhysxTendonAttachmentAPI::CreateGearingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Gearing),
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxTendonAttachmentAPI::GetLocalPosAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_LocalPos));
}

UsdAttribute
PhysxSchemaPhysxTendonAttachmentAPI::CreateLocalPosAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_LocalPos),
                       SdfValueTypeNames->Point3f,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxTendonAttachmentAPI::GetParentAttachmentAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_ParentAttachment));
}

UsdAttribute
PhysxSchemaPhysxTendonAttachmentAPI::CreateParentAttachmentAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_ParentAttachment),
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdRelationship
PhysxSchemaPhysxTendonAttachmentAPI::GetParentLinkRel() const
{
    return GetPrim().GetRelationship(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_ParentLink));
}

UsdRelationship
PhysxSchemaPhysxTendonAttachmentAPI::CreateParentLinkRel() const
{
    return GetPrim().CreateRelationship(
                       _GetNamespacedPropertyName(
                           GetName(),
                           PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_ParentLink),
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
PhysxSchemaPhysxTendonAttachmentAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Gearing,
        PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_LocalPos,
        PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_ParentAttachment,
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

/*static*/
TfTokenVector
PhysxSchemaPhysxTendonAttachmentAPI::GetSchemaAttributeNames(
    bool includeInherited, const TfToken &instanceName)
{
    const TfTokenVector &attrNames = GetSchemaAttributeNames(includeInherited);
    if (instanceName.IsEmpty()) {
        return attrNames;
    }
    TfTokenVector result;
    result.reserve(attrNames.size());
    for (const TfToken &attrName : attrNames) {
        result.push_back(
            UsdSchemaRegistry::MakeMultipleApplyNameInstance(attrName, instanceName));
    }
    return result;
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
