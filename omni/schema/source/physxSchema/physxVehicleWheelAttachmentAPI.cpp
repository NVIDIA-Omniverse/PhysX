//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxVehicleWheelAttachmentAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleWheelAttachmentAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxVehicleWheelAttachmentAPI::~PhysxSchemaPhysxVehicleWheelAttachmentAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleWheelAttachmentAPI
PhysxSchemaPhysxVehicleWheelAttachmentAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleWheelAttachmentAPI();
    }
    return PhysxSchemaPhysxVehicleWheelAttachmentAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleWheelAttachmentAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleWheelAttachmentAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleWheelAttachmentAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleWheelAttachmentAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxVehicleWheelAttachmentAPI
PhysxSchemaPhysxVehicleWheelAttachmentAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleWheelAttachmentAPI>()) {
        return PhysxSchemaPhysxVehicleWheelAttachmentAPI(prim);
    }
    return PhysxSchemaPhysxVehicleWheelAttachmentAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleWheelAttachmentAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleWheelAttachmentAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleWheelAttachmentAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleWheelAttachmentAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::GetIndexAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelAttachmentIndex);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::CreateIndexAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelAttachmentIndex,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::GetSuspensionTravelDirectionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelAttachmentSuspensionTravelDirection);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::CreateSuspensionTravelDirectionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelAttachmentSuspensionTravelDirection,
                       SdfValueTypeNames->Vector3f,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::GetSuspensionFramePositionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelAttachmentSuspensionFramePosition);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::CreateSuspensionFramePositionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelAttachmentSuspensionFramePosition,
                       SdfValueTypeNames->Point3f,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::GetSuspensionFrameOrientationAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelAttachmentSuspensionFrameOrientation);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::CreateSuspensionFrameOrientationAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelAttachmentSuspensionFrameOrientation,
                       SdfValueTypeNames->Quatf,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::GetSuspensionForceAppPointOffsetAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelAttachmentSuspensionForceAppPointOffset);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::CreateSuspensionForceAppPointOffsetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelAttachmentSuspensionForceAppPointOffset,
                       SdfValueTypeNames->Float3,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::GetWheelCenterOfMassOffsetAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelAttachmentWheelCenterOfMassOffset);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::CreateWheelCenterOfMassOffsetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelAttachmentWheelCenterOfMassOffset,
                       SdfValueTypeNames->Float3,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::GetTireForceAppPointOffsetAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelAttachmentTireForceAppPointOffset);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::CreateTireForceAppPointOffsetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelAttachmentTireForceAppPointOffset,
                       SdfValueTypeNames->Float3,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::GetWheelFramePositionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelAttachmentWheelFramePosition);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::CreateWheelFramePositionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelAttachmentWheelFramePosition,
                       SdfValueTypeNames->Point3f,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::GetWheelFrameOrientationAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelAttachmentWheelFrameOrientation);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::CreateWheelFrameOrientationAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelAttachmentWheelFrameOrientation,
                       SdfValueTypeNames->Quatf,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::GetDrivenAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxVehicleWheelAttachmentDriven);
}

UsdAttribute
PhysxSchemaPhysxVehicleWheelAttachmentAPI::CreateDrivenAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxVehicleWheelAttachmentDriven,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdRelationship
PhysxSchemaPhysxVehicleWheelAttachmentAPI::GetWheelRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physxVehicleWheelAttachmentWheel);
}

UsdRelationship
PhysxSchemaPhysxVehicleWheelAttachmentAPI::CreateWheelRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physxVehicleWheelAttachmentWheel,
                       /* custom = */ false);
}

UsdRelationship
PhysxSchemaPhysxVehicleWheelAttachmentAPI::GetTireRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physxVehicleWheelAttachmentTire);
}

UsdRelationship
PhysxSchemaPhysxVehicleWheelAttachmentAPI::CreateTireRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physxVehicleWheelAttachmentTire,
                       /* custom = */ false);
}

UsdRelationship
PhysxSchemaPhysxVehicleWheelAttachmentAPI::GetSuspensionRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physxVehicleWheelAttachmentSuspension);
}

UsdRelationship
PhysxSchemaPhysxVehicleWheelAttachmentAPI::CreateSuspensionRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physxVehicleWheelAttachmentSuspension,
                       /* custom = */ false);
}

UsdRelationship
PhysxSchemaPhysxVehicleWheelAttachmentAPI::GetCollisionGroupRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physxVehicleWheelAttachmentCollisionGroup);
}

UsdRelationship
PhysxSchemaPhysxVehicleWheelAttachmentAPI::CreateCollisionGroupRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physxVehicleWheelAttachmentCollisionGroup,
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
PhysxSchemaPhysxVehicleWheelAttachmentAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxVehicleWheelAttachmentIndex,
        PhysxSchemaTokens->physxVehicleWheelAttachmentSuspensionTravelDirection,
        PhysxSchemaTokens->physxVehicleWheelAttachmentSuspensionFramePosition,
        PhysxSchemaTokens->physxVehicleWheelAttachmentSuspensionFrameOrientation,
        PhysxSchemaTokens->physxVehicleWheelAttachmentSuspensionForceAppPointOffset,
        PhysxSchemaTokens->physxVehicleWheelAttachmentWheelCenterOfMassOffset,
        PhysxSchemaTokens->physxVehicleWheelAttachmentTireForceAppPointOffset,
        PhysxSchemaTokens->physxVehicleWheelAttachmentWheelFramePosition,
        PhysxSchemaTokens->physxVehicleWheelAttachmentWheelFrameOrientation,
        PhysxSchemaTokens->physxVehicleWheelAttachmentDriven,
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
