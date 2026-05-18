//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxVehicleBrakesAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleBrakesAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

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
        && tokens[0] == PhysxSchemaTokens->physxVehicleBrakes) {
        *name = TfToken(propertyName.substr(
           PhysxSchemaTokens->physxVehicleBrakes.GetString().size() + 1));
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
