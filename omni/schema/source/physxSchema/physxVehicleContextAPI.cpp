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
#include ".//physxVehicleContextAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleContextAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxVehicleContextAPI::~PhysxSchemaPhysxVehicleContextAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleContextAPI
PhysxSchemaPhysxVehicleContextAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleContextAPI();
    }
    return PhysxSchemaPhysxVehicleContextAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleContextAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleContextAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleContextAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleContextAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxVehicleContextAPI
PhysxSchemaPhysxVehicleContextAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleContextAPI>()) {
        return PhysxSchemaPhysxVehicleContextAPI(prim);
    }
    return PhysxSchemaPhysxVehicleContextAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleContextAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleContextAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleContextAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleContextAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::GetUpdateModeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleContextUpdateMode);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::CreateUpdateModeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleContextUpdateMode,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::GetUpAxisAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleContextUpAxis);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::CreateUpAxisAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleContextUpAxis,
                       SdfValueTypeNames->Float3,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::GetForwardAxisAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleContextForwardAxis);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::CreateForwardAxisAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleContextForwardAxis,
                       SdfValueTypeNames->Float3,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::GetVerticalAxisAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleContextVerticalAxis);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::CreateVerticalAxisAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleContextVerticalAxis,
                       SdfValueTypeNames->Token,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::GetLongitudinalAxisAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleContextLongitudinalAxis);
}

UsdAttribute
PhysxSchemaPhysxVehicleContextAPI::CreateLongitudinalAxisAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleContextLongitudinalAxis,
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
PhysxSchemaPhysxVehicleContextAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxVehicleContextUpdateMode,
        PhysxSchemaTokens->physxVehicleContextUpAxis,
        PhysxSchemaTokens->physxVehicleContextForwardAxis,
        PhysxSchemaTokens->physxVehicleContextVerticalAxis,
        PhysxSchemaTokens->physxVehicleContextLongitudinalAxis,
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
