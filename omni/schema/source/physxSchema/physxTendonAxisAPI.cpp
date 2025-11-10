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
#include ".//physxTendonAxisAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

#include "pxr/base/tf/staticTokens.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxTendonAxisAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

TF_DEFINE_PRIVATE_TOKENS(
    _schemaTokens,
    (physxTendon)
);

/* virtual */
PhysxSchemaPhysxTendonAxisAPI::~PhysxSchemaPhysxTendonAxisAPI()
{
}

/* static */
PhysxSchemaPhysxTendonAxisAPI
PhysxSchemaPhysxTendonAxisAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxTendonAxisAPI();
    }
    TfToken name;
    if (!IsPhysxTendonAxisAPIPath(path, &name)) {
        TF_CODING_ERROR("Invalid physxTendon path <%s>.", path.GetText());
        return PhysxSchemaPhysxTendonAxisAPI();
    }
    return PhysxSchemaPhysxTendonAxisAPI(stage->GetPrimAtPath(path.GetPrimPath()), name);
}

PhysxSchemaPhysxTendonAxisAPI
PhysxSchemaPhysxTendonAxisAPI::Get(const UsdPrim &prim, const TfToken &name)
{
    return PhysxSchemaPhysxTendonAxisAPI(prim, name);
}

/* static */
std::vector<PhysxSchemaPhysxTendonAxisAPI>
PhysxSchemaPhysxTendonAxisAPI::GetAll(const UsdPrim &prim)
{
    std::vector<PhysxSchemaPhysxTendonAxisAPI> schemas;
    
    for (const auto &schemaName :
         UsdAPISchemaBase::_GetMultipleApplyInstanceNames(prim, _GetStaticTfType())) {
        schemas.emplace_back(prim, schemaName);
    }

    return schemas;
}


/* static */
bool 
PhysxSchemaPhysxTendonAxisAPI::IsSchemaPropertyBaseName(const TfToken &baseName)
{
    static TfTokenVector attrsAndRels = {
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Gearing),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_ForceCoefficient),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_JointAxis),
    };

    return find(attrsAndRels.begin(), attrsAndRels.end(), baseName)
            != attrsAndRels.end();
}

/* static */
bool
PhysxSchemaPhysxTendonAxisAPI::IsPhysxTendonAxisAPIPath(
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
UsdSchemaKind PhysxSchemaPhysxTendonAxisAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxTendonAxisAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxTendonAxisAPI::CanApply(
    const UsdPrim &prim, const TfToken &name, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxTendonAxisAPI>(name, whyNot);
}

/* static */
PhysxSchemaPhysxTendonAxisAPI
PhysxSchemaPhysxTendonAxisAPI::Apply(const UsdPrim &prim, const TfToken &name)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxTendonAxisAPI>(name)) {
        return PhysxSchemaPhysxTendonAxisAPI(prim, name);
    }
    return PhysxSchemaPhysxTendonAxisAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxTendonAxisAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxTendonAxisAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxTendonAxisAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxTendonAxisAPI::_GetTfType() const
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
PhysxSchemaPhysxTendonAxisAPI::GetGearingAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Gearing));
}

UsdAttribute
PhysxSchemaPhysxTendonAxisAPI::CreateGearingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Gearing),
                       SdfValueTypeNames->FloatArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxTendonAxisAPI::GetForceCoefficientAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_ForceCoefficient));
}

UsdAttribute
PhysxSchemaPhysxTendonAxisAPI::CreateForceCoefficientAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_ForceCoefficient),
                       SdfValueTypeNames->FloatArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxTendonAxisAPI::GetJointAxisAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_JointAxis));
}

UsdAttribute
PhysxSchemaPhysxTendonAxisAPI::CreateJointAxisAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_JointAxis),
                       SdfValueTypeNames->TokenArray,
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
PhysxSchemaPhysxTendonAxisAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_Gearing,
        PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_ForceCoefficient,
        PhysxSchemaTokens->physxTendon_MultipleApplyTemplate_JointAxis,
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
PhysxSchemaPhysxTendonAxisAPI::GetSchemaAttributeNames(
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
