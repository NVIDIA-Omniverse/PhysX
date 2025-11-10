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
#include ".//physxDiffuseParticlesAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxDiffuseParticlesAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxDiffuseParticlesAPI::~PhysxSchemaPhysxDiffuseParticlesAPI()
{
}

/* static */
PhysxSchemaPhysxDiffuseParticlesAPI
PhysxSchemaPhysxDiffuseParticlesAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxDiffuseParticlesAPI();
    }
    return PhysxSchemaPhysxDiffuseParticlesAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxDiffuseParticlesAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxDiffuseParticlesAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxDiffuseParticlesAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxDiffuseParticlesAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxDiffuseParticlesAPI
PhysxSchemaPhysxDiffuseParticlesAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxDiffuseParticlesAPI>()) {
        return PhysxSchemaPhysxDiffuseParticlesAPI(prim);
    }
    return PhysxSchemaPhysxDiffuseParticlesAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxDiffuseParticlesAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxDiffuseParticlesAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxDiffuseParticlesAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxDiffuseParticlesAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::GetDiffuseParticlesEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDiffuseParticlesDiffuseParticlesEnabled);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::CreateDiffuseParticlesEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDiffuseParticlesDiffuseParticlesEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::GetMaxDiffuseParticleMultiplierAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDiffuseParticlesMaxDiffuseParticleMultiplier);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::CreateMaxDiffuseParticleMultiplierAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDiffuseParticlesMaxDiffuseParticleMultiplier,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityUniform,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::GetThresholdAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDiffuseParticlesThreshold);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::CreateThresholdAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDiffuseParticlesThreshold,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::GetLifetimeAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDiffuseParticlesLifetime);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::CreateLifetimeAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDiffuseParticlesLifetime,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::GetAirDragAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDiffuseParticlesAirDrag);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::CreateAirDragAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDiffuseParticlesAirDrag,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::GetBubbleDragAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDiffuseParticlesBubbleDrag);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::CreateBubbleDragAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDiffuseParticlesBubbleDrag,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::GetBuoyancyAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDiffuseParticlesBuoyancy);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::CreateBuoyancyAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDiffuseParticlesBuoyancy,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::GetKineticEnergyWeightAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDiffuseParticlesKineticEnergyWeight);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::CreateKineticEnergyWeightAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDiffuseParticlesKineticEnergyWeight,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::GetPressureWeightAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDiffuseParticlesPressureWeight);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::CreatePressureWeightAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDiffuseParticlesPressureWeight,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::GetDivergenceWeightAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDiffuseParticlesDivergenceWeight);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::CreateDivergenceWeightAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDiffuseParticlesDivergenceWeight,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::GetCollisionDecayAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDiffuseParticlesCollisionDecay);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::CreateCollisionDecayAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDiffuseParticlesCollisionDecay,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::GetUseAccurateVelocityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxDiffuseParticlesUseAccurateVelocity);
}

UsdAttribute
PhysxSchemaPhysxDiffuseParticlesAPI::CreateUseAccurateVelocityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxDiffuseParticlesUseAccurateVelocity,
                       SdfValueTypeNames->Bool,
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
PhysxSchemaPhysxDiffuseParticlesAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxDiffuseParticlesDiffuseParticlesEnabled,
        PhysxSchemaTokens->physxDiffuseParticlesMaxDiffuseParticleMultiplier,
        PhysxSchemaTokens->physxDiffuseParticlesThreshold,
        PhysxSchemaTokens->physxDiffuseParticlesLifetime,
        PhysxSchemaTokens->physxDiffuseParticlesAirDrag,
        PhysxSchemaTokens->physxDiffuseParticlesBubbleDrag,
        PhysxSchemaTokens->physxDiffuseParticlesBuoyancy,
        PhysxSchemaTokens->physxDiffuseParticlesKineticEnergyWeight,
        PhysxSchemaTokens->physxDiffuseParticlesPressureWeight,
        PhysxSchemaTokens->physxDiffuseParticlesDivergenceWeight,
        PhysxSchemaTokens->physxDiffuseParticlesCollisionDecay,
        PhysxSchemaTokens->physxDiffuseParticlesUseAccurateVelocity,
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
