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
#include ".//physxTendonAxisRootAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

#include "pxr/base/tf/staticTokens.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxTendonAxisRootAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

TF_DEFINE_PRIVATE_TOKENS(
    _schemaTokens,
    (physxTendon)
);

/* virtual */
PhysxSchemaPhysxTendonAxisRootAPI::~PhysxSchemaPhysxTendonAxisRootAPI()
{
}

/* static */
PhysxSchemaPhysxTendonAxisRootAPI
PhysxSchemaPhysxTendonAxisRootAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxTendonAxisRootAPI();
    }
    TfToken name;
    if (!IsPhysxTendonAxisRootAPIPath(path, &name)) {
        TF_CODING_ERROR("Invalid physxTendon path <%s>.", path.GetText());
        return PhysxSchemaPhysxTendonAxisRootAPI();
    }
    return PhysxSchemaPhysxTendonAxisRootAPI(stage->GetPrimAtPath(path.GetPrimPath()), name);
}

PhysxSchemaPhysxTendonAxisRootAPI
PhysxSchemaPhysxTendonAxisRootAPI::Get(const UsdPrim &prim, const TfToken &name)
{
    return PhysxSchemaPhysxTendonAxisRootAPI(prim, name);
}

/* static */
std::vector<PhysxSchemaPhysxTendonAxisRootAPI>
PhysxSchemaPhysxTendonAxisRootAPI::GetAll(const UsdPrim &prim)
{
    std::vector<PhysxSchemaPhysxTendonAxisRootAPI> schemas;
    
    for (const auto &schemaName :
         UsdAPISchemaBase::_GetMultipleApplyInstanceNames(prim, _GetStaticTfType())) {
        schemas.emplace_back(prim, schemaName);
    }

    return schemas;
}


/* static */
bool 
PhysxSchemaPhysxTendonAxisRootAPI::IsSchemaPropertyBaseName(const TfToken &baseName)
{
    static TfTokenVector attrsAndRels = {
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Stiffness),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Damping),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_LimitStiffness),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Offset),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_RestLength),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_LowerLimit),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_UpperLimit),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_TendonEnabled),
    };

    return find(attrsAndRels.begin(), attrsAndRels.end(), baseName)
            != attrsAndRels.end();
}

/* static */
bool
PhysxSchemaPhysxTendonAxisRootAPI::IsPhysxTendonAxisRootAPIPath(
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
        && tokens[0] == _schemaTokens->physxTendon) {
        *name = TfToken(propertyName.substr(
            _schemaTokens->physxTendon.GetString().size() + 1));
        return true;
    }

    return false;
}

/* virtual */
UsdSchemaKind PhysxSchemaPhysxTendonAxisRootAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxTendonAxisRootAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxTendonAxisRootAPI::CanApply(
    const UsdPrim &prim, const TfToken &name, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxTendonAxisRootAPI>(name, whyNot);
}

/* static */
PhysxSchemaPhysxTendonAxisRootAPI
PhysxSchemaPhysxTendonAxisRootAPI::Apply(const UsdPrim &prim, const TfToken &name)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxTendonAxisRootAPI>(name)) {
        return PhysxSchemaPhysxTendonAxisRootAPI(prim, name);
    }
    return PhysxSchemaPhysxTendonAxisRootAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxTendonAxisRootAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxTendonAxisRootAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxTendonAxisRootAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxTendonAxisRootAPI::_GetTfType() const
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
PhysxSchemaPhysxTendonAxisRootAPI::GetStiffnessAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Stiffness));
}

UsdAttribute
PhysxSchemaPhysxTendonAxisRootAPI::CreateStiffnessAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Stiffness),
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxTendonAxisRootAPI::GetDampingAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Damping));
}

UsdAttribute
PhysxSchemaPhysxTendonAxisRootAPI::CreateDampingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Damping),
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxTendonAxisRootAPI::GetLimitStiffnessAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_LimitStiffness));
}

UsdAttribute
PhysxSchemaPhysxTendonAxisRootAPI::CreateLimitStiffnessAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_LimitStiffness),
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxTendonAxisRootAPI::GetOffsetAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Offset));
}

UsdAttribute
PhysxSchemaPhysxTendonAxisRootAPI::CreateOffsetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Offset),
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxTendonAxisRootAPI::GetRestLengthAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_RestLength));
}

UsdAttribute
PhysxSchemaPhysxTendonAxisRootAPI::CreateRestLengthAttr(VtValue const &defaultValue, bool writeSparsely) const
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
PhysxSchemaPhysxTendonAxisRootAPI::GetLowerLimitAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_LowerLimit));
}

UsdAttribute
PhysxSchemaPhysxTendonAxisRootAPI::CreateLowerLimitAttr(VtValue const &defaultValue, bool writeSparsely) const
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
PhysxSchemaPhysxTendonAxisRootAPI::GetUpperLimitAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_UpperLimit));
}

UsdAttribute
PhysxSchemaPhysxTendonAxisRootAPI::CreateUpperLimitAttr(VtValue const &defaultValue, bool writeSparsely) const
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

UsdAttribute
PhysxSchemaPhysxTendonAxisRootAPI::GetTendonEnabledAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_TendonEnabled));
}

UsdAttribute
PhysxSchemaPhysxTendonAxisRootAPI::CreateTendonEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_TendonEnabled),
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
PhysxSchemaPhysxTendonAxisRootAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Stiffness,
        PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Damping,
        PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_LimitStiffness,
        PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Offset,
        PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_RestLength,
        PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_LowerLimit,
        PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_UpperLimit,
        PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_TendonEnabled,
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
PhysxSchemaPhysxTendonAxisRootAPI::GetSchemaAttributeNames(
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
