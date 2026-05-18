//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxVehicleGearsAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleGearsAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxVehicleGearsAPI::~PhysxSchemaPhysxVehicleGearsAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleGearsAPI
PhysxSchemaPhysxVehicleGearsAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleGearsAPI();
    }
    return PhysxSchemaPhysxVehicleGearsAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleGearsAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleGearsAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleGearsAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleGearsAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxVehicleGearsAPI
PhysxSchemaPhysxVehicleGearsAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleGearsAPI>()) {
        return PhysxSchemaPhysxVehicleGearsAPI(prim);
    }
    return PhysxSchemaPhysxVehicleGearsAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleGearsAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleGearsAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleGearsAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleGearsAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxVehicleGearsAPI::GetRatiosAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleGearsRatios);
}

UsdAttribute
PhysxSchemaPhysxVehicleGearsAPI::CreateRatiosAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleGearsRatios,
                       SdfValueTypeNames->FloatArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleGearsAPI::GetRatioScaleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleGearsRatioScale);
}

UsdAttribute
PhysxSchemaPhysxVehicleGearsAPI::CreateRatioScaleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleGearsRatioScale,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleGearsAPI::GetSwitchTimeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleGearsSwitchTime);
}

UsdAttribute
PhysxSchemaPhysxVehicleGearsAPI::CreateSwitchTimeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleGearsSwitchTime,
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
PhysxSchemaPhysxVehicleGearsAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxVehicleGearsRatios,
        PhysxSchemaTokens->physxVehicleGearsRatioScale,
        PhysxSchemaTokens->physxVehicleGearsSwitchTime,
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
