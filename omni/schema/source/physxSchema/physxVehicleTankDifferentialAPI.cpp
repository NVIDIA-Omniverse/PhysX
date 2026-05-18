//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxVehicleTankDifferentialAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleTankDifferentialAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxVehicleTankDifferentialAPI::~PhysxSchemaPhysxVehicleTankDifferentialAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleTankDifferentialAPI
PhysxSchemaPhysxVehicleTankDifferentialAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleTankDifferentialAPI();
    }
    return PhysxSchemaPhysxVehicleTankDifferentialAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleTankDifferentialAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleTankDifferentialAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleTankDifferentialAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleTankDifferentialAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxVehicleTankDifferentialAPI
PhysxSchemaPhysxVehicleTankDifferentialAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleTankDifferentialAPI>()) {
        return PhysxSchemaPhysxVehicleTankDifferentialAPI(prim);
    }
    return PhysxSchemaPhysxVehicleTankDifferentialAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleTankDifferentialAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleTankDifferentialAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleTankDifferentialAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleTankDifferentialAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxVehicleTankDifferentialAPI::GetNumberOfWheelsPerTrackAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleTankDifferentialNumberOfWheelsPerTrack);
}

UsdAttribute
PhysxSchemaPhysxVehicleTankDifferentialAPI::CreateNumberOfWheelsPerTrackAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleTankDifferentialNumberOfWheelsPerTrack,
                       SdfValueTypeNames->IntArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleTankDifferentialAPI::GetThrustIndexPerTrackAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleTankDifferentialThrustIndexPerTrack);
}

UsdAttribute
PhysxSchemaPhysxVehicleTankDifferentialAPI::CreateThrustIndexPerTrackAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleTankDifferentialThrustIndexPerTrack,
                       SdfValueTypeNames->IntArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleTankDifferentialAPI::GetTrackToWheelIndicesAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleTankDifferentialTrackToWheelIndices);
}

UsdAttribute
PhysxSchemaPhysxVehicleTankDifferentialAPI::CreateTrackToWheelIndicesAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleTankDifferentialTrackToWheelIndices,
                       SdfValueTypeNames->IntArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleTankDifferentialAPI::GetWheelIndicesInTrackOrderAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleTankDifferentialWheelIndicesInTrackOrder);
}

UsdAttribute
PhysxSchemaPhysxVehicleTankDifferentialAPI::CreateWheelIndicesInTrackOrderAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleTankDifferentialWheelIndicesInTrackOrder,
                       SdfValueTypeNames->IntArray,
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
PhysxSchemaPhysxVehicleTankDifferentialAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxVehicleTankDifferentialNumberOfWheelsPerTrack,
        PhysxSchemaTokens->physxVehicleTankDifferentialThrustIndexPerTrack,
        PhysxSchemaTokens->physxVehicleTankDifferentialTrackToWheelIndices,
        PhysxSchemaTokens->physxVehicleTankDifferentialWheelIndicesInTrackOrder,
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
