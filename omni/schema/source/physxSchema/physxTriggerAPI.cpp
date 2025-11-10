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
#include ".//physxTriggerAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxTriggerAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxTriggerAPI::~PhysxSchemaPhysxTriggerAPI()
{
}

/* static */
PhysxSchemaPhysxTriggerAPI
PhysxSchemaPhysxTriggerAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxTriggerAPI();
    }
    return PhysxSchemaPhysxTriggerAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxTriggerAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxTriggerAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxTriggerAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxTriggerAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxTriggerAPI
PhysxSchemaPhysxTriggerAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxTriggerAPI>()) {
        return PhysxSchemaPhysxTriggerAPI(prim);
    }
    return PhysxSchemaPhysxTriggerAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxTriggerAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxTriggerAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxTriggerAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxTriggerAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxTriggerAPI::GetEnterScriptTypeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxTriggerEnterScriptType);
}

UsdAttribute
PhysxSchemaPhysxTriggerAPI::CreateEnterScriptTypeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxTriggerEnterScriptType,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxTriggerAPI::GetLeaveScriptTypeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxTriggerLeaveScriptType);
}

UsdAttribute
PhysxSchemaPhysxTriggerAPI::CreateLeaveScriptTypeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxTriggerLeaveScriptType,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxTriggerAPI::GetOnEnterScriptAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxTriggerOnEnterScript);
}

UsdAttribute
PhysxSchemaPhysxTriggerAPI::CreateOnEnterScriptAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxTriggerOnEnterScript,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxTriggerAPI::GetOnLeaveScriptAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxTriggerOnLeaveScript);
}

UsdAttribute
PhysxSchemaPhysxTriggerAPI::CreateOnLeaveScriptAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxTriggerOnLeaveScript,
                       SdfValueTypeNames->Token,
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
PhysxSchemaPhysxTriggerAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxTriggerEnterScriptType,
        PhysxSchemaTokens->physxTriggerLeaveScriptType,
        PhysxSchemaTokens->physxTriggerOnEnterScript,
        PhysxSchemaTokens->physxTriggerOnLeaveScript,
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
