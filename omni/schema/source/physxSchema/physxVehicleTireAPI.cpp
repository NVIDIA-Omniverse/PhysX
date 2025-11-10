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
#include ".//physxVehicleTireAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleTireAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxVehicleTireAPI::~PhysxSchemaPhysxVehicleTireAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleTireAPI
PhysxSchemaPhysxVehicleTireAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleTireAPI();
    }
    return PhysxSchemaPhysxVehicleTireAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleTireAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleTireAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleTireAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleTireAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxVehicleTireAPI
PhysxSchemaPhysxVehicleTireAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleTireAPI>()) {
        return PhysxSchemaPhysxVehicleTireAPI(prim);
    }
    return PhysxSchemaPhysxVehicleTireAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleTireAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleTireAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleTireAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleTireAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::GetLatStiffXAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleTireLatStiffX);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::CreateLatStiffXAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleTireLatStiffX,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::GetLatStiffYAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleTireLatStiffY);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::CreateLatStiffYAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleTireLatStiffY,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::GetLateralStiffnessGraphAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleTireLateralStiffnessGraph);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::CreateLateralStiffnessGraphAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleTireLateralStiffnessGraph,
                       SdfValueTypeNames->Float2,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::GetLongitudinalStiffnessPerUnitGravityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleTireLongitudinalStiffnessPerUnitGravity);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::CreateLongitudinalStiffnessPerUnitGravityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleTireLongitudinalStiffnessPerUnitGravity,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::GetLongitudinalStiffnessAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleTireLongitudinalStiffness);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::CreateLongitudinalStiffnessAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleTireLongitudinalStiffness,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::GetCamberStiffnessPerUnitGravityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleTireCamberStiffnessPerUnitGravity);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::CreateCamberStiffnessPerUnitGravityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleTireCamberStiffnessPerUnitGravity,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::GetCamberStiffnessAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleTireCamberStiffness);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::CreateCamberStiffnessAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleTireCamberStiffness,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::GetFrictionVsSlipGraphAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleTireFrictionVsSlipGraph);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::CreateFrictionVsSlipGraphAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleTireFrictionVsSlipGraph,
                       SdfValueTypeNames->Float2Array,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::GetRestLoadAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleTireRestLoad);
}

UsdAttribute
PhysxSchemaPhysxVehicleTireAPI::CreateRestLoadAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleTireRestLoad,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdRelationship
PhysxSchemaPhysxVehicleTireAPI::GetFrictionTableRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physxVehicleTireFrictionTable);
}

UsdRelationship
PhysxSchemaPhysxVehicleTireAPI::CreateFrictionTableRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physxVehicleTireFrictionTable,
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
PhysxSchemaPhysxVehicleTireAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxVehicleTireLatStiffX,
        PhysxSchemaTokens->physxVehicleTireLatStiffY,
        PhysxSchemaTokens->physxVehicleTireLateralStiffnessGraph,
        PhysxSchemaTokens->physxVehicleTireLongitudinalStiffnessPerUnitGravity,
        PhysxSchemaTokens->physxVehicleTireLongitudinalStiffness,
        PhysxSchemaTokens->physxVehicleTireCamberStiffnessPerUnitGravity,
        PhysxSchemaTokens->physxVehicleTireCamberStiffness,
        PhysxSchemaTokens->physxVehicleTireFrictionVsSlipGraph,
        PhysxSchemaTokens->physxVehicleTireRestLoad,
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
