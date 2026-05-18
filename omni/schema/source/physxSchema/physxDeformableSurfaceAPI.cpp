//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxDeformableSurfaceAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxDeformableSurfaceAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxDeformableSurfaceAPI::~PhysxSchemaPhysxDeformableSurfaceAPI()
{
}

/* static */
PhysxSchemaPhysxDeformableSurfaceAPI
PhysxSchemaPhysxDeformableSurfaceAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxDeformableSurfaceAPI();
    }
    return PhysxSchemaPhysxDeformableSurfaceAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxDeformableSurfaceAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxDeformableSurfaceAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxDeformableSurfaceAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxDeformableSurfaceAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxDeformableSurfaceAPI
PhysxSchemaPhysxDeformableSurfaceAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxDeformableSurfaceAPI>()) {
        return PhysxSchemaPhysxDeformableSurfaceAPI(prim);
    }
    return PhysxSchemaPhysxDeformableSurfaceAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxDeformableSurfaceAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxDeformableSurfaceAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxDeformableSurfaceAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxDeformableSurfaceAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::GetFlatteningEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSurfaceFlatteningEnabled);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::CreateFlatteningEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSurfaceFlatteningEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::GetBendingStiffnessScaleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSurfaceBendingStiffnessScale);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::CreateBendingStiffnessScaleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSurfaceBendingStiffnessScale,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::GetCollisionPairUpdateFrequencyAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSurfaceCollisionPairUpdateFrequency);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::CreateCollisionPairUpdateFrequencyAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSurfaceCollisionPairUpdateFrequency,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::GetCollisionIterationMultiplierAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSurfaceCollisionIterationMultiplier);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::CreateCollisionIterationMultiplierAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSurfaceCollisionIterationMultiplier,
                       SdfValueTypeNames->UInt,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::GetMaxVelocityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDeformableSurfaceMaxVelocity);
}

UsdAttribute
PhysxSchemaPhysxDeformableSurfaceAPI::CreateMaxVelocityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDeformableSurfaceMaxVelocity,
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
PhysxSchemaPhysxDeformableSurfaceAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxDeformableSurfaceFlatteningEnabled,
        PhysxSchemaTokens->physxDeformableSurfaceBendingStiffnessScale,
        PhysxSchemaTokens->physxDeformableSurfaceCollisionPairUpdateFrequency,
        PhysxSchemaTokens->physxDeformableSurfaceCollisionIterationMultiplier,
        PhysxSchemaTokens->physxDeformableSurfaceMaxVelocity,
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
