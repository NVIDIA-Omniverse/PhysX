//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxLimitAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxLimitAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxLimitAPI::~PhysxSchemaPhysxLimitAPI()
{
}

/* static */
PhysxSchemaPhysxLimitAPI
PhysxSchemaPhysxLimitAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxLimitAPI();
    }
    TfToken name;
    if (!IsPhysxLimitAPIPath(path, &name)) {
        TF_CODING_ERROR("Invalid physxLimit path <%s>.", path.GetText());
        return PhysxSchemaPhysxLimitAPI();
    }
    return PhysxSchemaPhysxLimitAPI(stage->GetPrimAtPath(path.GetPrimPath()), name);
}

PhysxSchemaPhysxLimitAPI
PhysxSchemaPhysxLimitAPI::Get(const UsdPrim &prim, const TfToken &name)
{
    return PhysxSchemaPhysxLimitAPI(prim, name);
}

/* static */
std::vector<PhysxSchemaPhysxLimitAPI>
PhysxSchemaPhysxLimitAPI::GetAll(const UsdPrim &prim)
{
    std::vector<PhysxSchemaPhysxLimitAPI> schemas;
    
    for (const auto &schemaName :
         UsdAPISchemaBase::_GetMultipleApplyInstanceNames(prim, _GetStaticTfType())) {
        schemas.emplace_back(prim, schemaName);
    }

    return schemas;
}


/* static */
bool 
PhysxSchemaPhysxLimitAPI::IsSchemaPropertyBaseName(const TfToken &baseName)
{
    static TfTokenVector attrsAndRels = {
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxLimit_MultipleApplyTemplate_Restitution),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxLimit_MultipleApplyTemplate_BounceThreshold),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxLimit_MultipleApplyTemplate_Stiffness),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxLimit_MultipleApplyTemplate_Damping),
    };

    return find(attrsAndRels.begin(), attrsAndRels.end(), baseName)
            != attrsAndRels.end();
}

/* static */
bool
PhysxSchemaPhysxLimitAPI::IsPhysxLimitAPIPath(
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
        && tokens[0] == PhysxSchemaTokens->physxLimit) {
        *name = TfToken(propertyName.substr(
           PhysxSchemaTokens->physxLimit.GetString().size() + 1));
        return true;
    }

    return false;
}

/* virtual */
UsdSchemaKind PhysxSchemaPhysxLimitAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxLimitAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxLimitAPI::CanApply(
    const UsdPrim &prim, const TfToken &name, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxLimitAPI>(name, whyNot);
}

/* static */
PhysxSchemaPhysxLimitAPI
PhysxSchemaPhysxLimitAPI::Apply(const UsdPrim &prim, const TfToken &name)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxLimitAPI>(name)) {
        return PhysxSchemaPhysxLimitAPI(prim, name);
    }
    return PhysxSchemaPhysxLimitAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxLimitAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxLimitAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxLimitAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxLimitAPI::_GetTfType() const
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
PhysxSchemaPhysxLimitAPI::GetRestitutionAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxLimit_MultipleApplyTemplate_Restitution));
}

UsdAttribute
PhysxSchemaPhysxLimitAPI::CreateRestitutionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxLimit_MultipleApplyTemplate_Restitution),
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxLimitAPI::GetBounceThresholdAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxLimit_MultipleApplyTemplate_BounceThreshold));
}

UsdAttribute
PhysxSchemaPhysxLimitAPI::CreateBounceThresholdAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxLimit_MultipleApplyTemplate_BounceThreshold),
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxLimitAPI::GetStiffnessAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxLimit_MultipleApplyTemplate_Stiffness));
}

UsdAttribute
PhysxSchemaPhysxLimitAPI::CreateStiffnessAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxLimit_MultipleApplyTemplate_Stiffness),
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxLimitAPI::GetDampingAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxLimit_MultipleApplyTemplate_Damping));
}

UsdAttribute
PhysxSchemaPhysxLimitAPI::CreateDampingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxLimit_MultipleApplyTemplate_Damping),
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
PhysxSchemaPhysxLimitAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxLimit_MultipleApplyTemplate_Restitution,
        PhysxSchemaTokens->physxLimit_MultipleApplyTemplate_BounceThreshold,
        PhysxSchemaTokens->physxLimit_MultipleApplyTemplate_Stiffness,
        PhysxSchemaTokens->physxLimit_MultipleApplyTemplate_Damping,
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
PhysxSchemaPhysxLimitAPI::GetSchemaAttributeNames(
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
