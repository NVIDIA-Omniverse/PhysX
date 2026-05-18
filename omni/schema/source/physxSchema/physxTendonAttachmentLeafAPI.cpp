//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxTendonAttachmentLeafAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxTendonAttachmentLeafAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxTendonAttachmentLeafAPI::~PhysxSchemaPhysxTendonAttachmentLeafAPI()
{
}

/* static */
PhysxSchemaPhysxTendonAttachmentLeafAPI
PhysxSchemaPhysxTendonAttachmentLeafAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxTendonAttachmentLeafAPI();
    }
    TfToken name;
    if (!IsPhysxTendonAttachmentLeafAPIPath(path, &name)) {
        TF_CODING_ERROR("Invalid physxTendon path <%s>.", path.GetText());
        return PhysxSchemaPhysxTendonAttachmentLeafAPI();
    }
    return PhysxSchemaPhysxTendonAttachmentLeafAPI(stage->GetPrimAtPath(path.GetPrimPath()), name);
}

PhysxSchemaPhysxTendonAttachmentLeafAPI
PhysxSchemaPhysxTendonAttachmentLeafAPI::Get(const UsdPrim &prim, const TfToken &name)
{
    return PhysxSchemaPhysxTendonAttachmentLeafAPI(prim, name);
}

/* static */
std::vector<PhysxSchemaPhysxTendonAttachmentLeafAPI>
PhysxSchemaPhysxTendonAttachmentLeafAPI::GetAll(const UsdPrim &prim)
{
    std::vector<PhysxSchemaPhysxTendonAttachmentLeafAPI> schemas;
    
    for (const auto &schemaName :
         UsdAPISchemaBase::_GetMultipleApplyInstanceNames(prim, _GetStaticTfType())) {
        schemas.emplace_back(prim, schemaName);
    }

    return schemas;
}


/* static */
bool 
PhysxSchemaPhysxTendonAttachmentLeafAPI::IsSchemaPropertyBaseName(const TfToken &baseName)
{
    static TfTokenVector attrsAndRels = {
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_RestLength),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_LowerLimit),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_UpperLimit),
    };

    return find(attrsAndRels.begin(), attrsAndRels.end(), baseName)
            != attrsAndRels.end();
}

/* static */
bool
PhysxSchemaPhysxTendonAttachmentLeafAPI::IsPhysxTendonAttachmentLeafAPIPath(
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
UsdSchemaKind PhysxSchemaPhysxTendonAttachmentLeafAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxTendonAttachmentLeafAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxTendonAttachmentLeafAPI::CanApply(
    const UsdPrim &prim, const TfToken &name, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxTendonAttachmentLeafAPI>(name, whyNot);
}

/* static */
PhysxSchemaPhysxTendonAttachmentLeafAPI
PhysxSchemaPhysxTendonAttachmentLeafAPI::Apply(const UsdPrim &prim, const TfToken &name)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxTendonAttachmentLeafAPI>(name)) {
        return PhysxSchemaPhysxTendonAttachmentLeafAPI(prim, name);
    }
    return PhysxSchemaPhysxTendonAttachmentLeafAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxTendonAttachmentLeafAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxTendonAttachmentLeafAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxTendonAttachmentLeafAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxTendonAttachmentLeafAPI::_GetTfType() const
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
PhysxSchemaPhysxTendonAttachmentLeafAPI::GetRestLengthAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_RestLength));
}

UsdAttribute
PhysxSchemaPhysxTendonAttachmentLeafAPI::CreateRestLengthAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_RestLength),
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxTendonAttachmentLeafAPI::GetLowerLimitAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_LowerLimit));
}

UsdAttribute
PhysxSchemaPhysxTendonAttachmentLeafAPI::CreateLowerLimitAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_LowerLimit),
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxTendonAttachmentLeafAPI::GetUpperLimitAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_UpperLimit));
}

UsdAttribute
PhysxSchemaPhysxTendonAttachmentLeafAPI::CreateUpperLimitAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_UpperLimit),
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
PhysxSchemaPhysxTendonAttachmentLeafAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_RestLength,
        PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_LowerLimit,
        PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_UpperLimit,
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
PhysxSchemaPhysxTendonAttachmentLeafAPI::GetSchemaAttributeNames(
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
