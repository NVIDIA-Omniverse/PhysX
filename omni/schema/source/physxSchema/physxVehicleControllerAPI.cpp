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
#include ".//physxVehicleControllerAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleControllerAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxVehicleControllerAPI::~PhysxSchemaPhysxVehicleControllerAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleControllerAPI
PhysxSchemaPhysxVehicleControllerAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleControllerAPI();
    }
    return PhysxSchemaPhysxVehicleControllerAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleControllerAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleControllerAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleControllerAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleControllerAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxVehicleControllerAPI
PhysxSchemaPhysxVehicleControllerAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleControllerAPI>()) {
        return PhysxSchemaPhysxVehicleControllerAPI(prim);
    }
    return PhysxSchemaPhysxVehicleControllerAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleControllerAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleControllerAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleControllerAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleControllerAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::GetAcceleratorAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleControllerAccelerator);
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::CreateAcceleratorAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleControllerAccelerator,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::GetBrake0Attr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleControllerBrake0);
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::CreateBrake0Attr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleControllerBrake0,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::GetBrake1Attr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleControllerBrake1);
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::CreateBrake1Attr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleControllerBrake1,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::GetBrakeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleControllerBrake);
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::CreateBrakeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleControllerBrake,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::GetHandbrakeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleControllerHandbrake);
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::CreateHandbrakeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleControllerHandbrake,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::GetSteerAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleControllerSteer);
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::CreateSteerAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleControllerSteer,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::GetSteerLeftAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleControllerSteerLeft);
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::CreateSteerLeftAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleControllerSteerLeft,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::GetSteerRightAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleControllerSteerRight);
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::CreateSteerRightAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleControllerSteerRight,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::GetTargetGearAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleControllerTargetGear);
}

UsdAttribute
PhysxSchemaPhysxVehicleControllerAPI::CreateTargetGearAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleControllerTargetGear,
                       SdfValueTypeNames->Int,
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
PhysxSchemaPhysxVehicleControllerAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxVehicleControllerAccelerator,
        PhysxSchemaTokens->physxVehicleControllerBrake0,
        PhysxSchemaTokens->physxVehicleControllerBrake1,
        PhysxSchemaTokens->physxVehicleControllerBrake,
        PhysxSchemaTokens->physxVehicleControllerHandbrake,
        PhysxSchemaTokens->physxVehicleControllerSteer,
        PhysxSchemaTokens->physxVehicleControllerSteerLeft,
        PhysxSchemaTokens->physxVehicleControllerSteerRight,
        PhysxSchemaTokens->physxVehicleControllerTargetGear,
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
