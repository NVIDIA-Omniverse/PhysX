//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxCameraAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxCameraAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxCameraAPI::~PhysxSchemaPhysxCameraAPI()
{
}

/* static */
PhysxSchemaPhysxCameraAPI
PhysxSchemaPhysxCameraAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxCameraAPI();
    }
    return PhysxSchemaPhysxCameraAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxCameraAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxCameraAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxCameraAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxCameraAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxCameraAPI
PhysxSchemaPhysxCameraAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxCameraAPI>()) {
        return PhysxSchemaPhysxCameraAPI(prim);
    }
    return PhysxSchemaPhysxCameraAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxCameraAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxCameraAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxCameraAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxCameraAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxCameraAPI::GetAlwaysUpdateEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->alwaysUpdateEnabled);
}

UsdAttribute
PhysxSchemaPhysxCameraAPI::CreateAlwaysUpdateEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->alwaysUpdateEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdRelationship
PhysxSchemaPhysxCameraAPI::GetPhysxCameraSubjectRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physxCameraSubject);
}

UsdRelationship
PhysxSchemaPhysxCameraAPI::CreatePhysxCameraSubjectRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physxCameraSubject,
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
PhysxSchemaPhysxCameraAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->alwaysUpdateEnabled,
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
