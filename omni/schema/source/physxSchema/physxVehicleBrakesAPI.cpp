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
#include ".//physxVehicleBrakesAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

#include "pxr/base/tf/staticTokens.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleBrakesAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

TF_DEFINE_PRIVATE_TOKENS(
    _schemaTokens,
    (physxVehicleBrakes)
);

/* virtual */
PhysxSchemaPhysxVehicleBrakesAPI::~PhysxSchemaPhysxVehicleBrakesAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleBrakesAPI
PhysxSchemaPhysxVehicleBrakesAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleBrakesAPI();
    }
    TfToken name;
    if (!IsPhysxVehicleBrakesAPIPath(path, &name)) {
        TF_CODING_ERROR("Invalid physxVehicleBrakes path <%s>.", path.GetText());
        return PhysxSchemaPhysxVehicleBrakesAPI();
    }
    return PhysxSchemaPhysxVehicleBrakesAPI(stage->GetPrimAtPath(path.GetPrimPath()), name);
}

PhysxSchemaPhysxVehicleBrakesAPI
PhysxSchemaPhysxVehicleBrakesAPI::Get(const UsdPrim &prim, const TfToken &name)
{
    return PhysxSchemaPhysxVehicleBrakesAPI(prim, name);
}

/* static */
std::vector<PhysxSchemaPhysxVehicleBrakesAPI>
PhysxSchemaPhysxVehicleBrakesAPI::GetAll(const UsdPrim &prim)
{
    std::vector<PhysxSchemaPhysxVehicleBrakesAPI> schemas;
    
    for (const auto &schemaName :
         UsdAPISchemaBase::_GetMultipleApplyInstanceNames(prim, _GetStaticTfType())) {
        schemas.emplace_back(prim, schemaName);
    }

    return schemas;
}


/* static */
bool 
PhysxSchemaPhysxVehicleBrakesAPI::IsSchemaPropertyBaseName(const TfToken &baseName)
{
    static TfTokenVector attrsAndRels = {
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxVehicleBrakes_MultipleApplyTemplate_Wheels),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxVehicleBrakes_MultipleApplyTemplate_MaxBrakeTorque),
        UsdSchemaRegistry::GetMultipleApplyNameTemplateBaseName(
            PhysxSchemaTokens->physxVehicleBrakes_MultipleApplyTemplate_TorqueMultipliers),
    };

    return find(attrsAndRels.begin(), attrsAndRels.end(), baseName)
            != attrsAndRels.end();
}

/* static */
bool
PhysxSchemaPhysxVehicleBrakesAPI::IsPhysxVehicleBrakesAPIPath(
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
        && tokens[0] == _schemaTokens->physxVehicleBrakes) {
        *name = TfToken(propertyName.substr(
            _schemaTokens->physxVehicleBrakes.GetString().size() + 1));
        return true;
    }

    return false;
}

/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleBrakesAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleBrakesAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleBrakesAPI::CanApply(
    const UsdPrim &prim, const TfToken &name, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleBrakesAPI>(name, whyNot);
}

/* static */
PhysxSchemaPhysxVehicleBrakesAPI
PhysxSchemaPhysxVehicleBrakesAPI::Apply(const UsdPrim &prim, const TfToken &name)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleBrakesAPI>(name)) {
        return PhysxSchemaPhysxVehicleBrakesAPI(prim, name);
    }
    return PhysxSchemaPhysxVehicleBrakesAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleBrakesAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleBrakesAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleBrakesAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleBrakesAPI::_GetTfType() const
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
PhysxSchemaPhysxVehicleBrakesAPI::GetWheelsAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxVehicleBrakes_MultipleApplyTemplate_Wheels));
}

UsdAttribute
PhysxSchemaPhysxVehicleBrakesAPI::CreateWheelsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxVehicleBrakes_MultipleApplyTemplate_Wheels),
                       SdfValueTypeNames->IntArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleBrakesAPI::GetMaxBrakeTorqueAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxVehicleBrakes_MultipleApplyTemplate_MaxBrakeTorque));
}

UsdAttribute
PhysxSchemaPhysxVehicleBrakesAPI::CreateMaxBrakeTorqueAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxVehicleBrakes_MultipleApplyTemplate_MaxBrakeTorque),
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleBrakesAPI::GetTorqueMultipliersAttr() const
{
    return GetPrim().GetAttribute(
        _GetNamespacedPropertyName(
            GetName(),
            PhysxSchemaTokens->physxVehicleBrakes_MultipleApplyTemplate_TorqueMultipliers));
}

UsdAttribute
PhysxSchemaPhysxVehicleBrakesAPI::CreateTorqueMultipliersAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(
                       _GetNamespacedPropertyName(
                            GetName(),
                           PhysxSchemaTokens->physxVehicleBrakes_MultipleApplyTemplate_TorqueMultipliers),
                       SdfValueTypeNames->FloatArray,
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
PhysxSchemaPhysxVehicleBrakesAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxVehicleBrakes_MultipleApplyTemplate_Wheels,
        PhysxSchemaTokens->physxVehicleBrakes_MultipleApplyTemplate_MaxBrakeTorque,
        PhysxSchemaTokens->physxVehicleBrakes_MultipleApplyTemplate_TorqueMultipliers,
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
PhysxSchemaPhysxVehicleBrakesAPI::GetSchemaAttributeNames(
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
