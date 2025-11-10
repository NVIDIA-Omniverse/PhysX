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
#include ".//physxPBDMaterialAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxPBDMaterialAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxPBDMaterialAPI::~PhysxSchemaPhysxPBDMaterialAPI()
{
}

/* static */
PhysxSchemaPhysxPBDMaterialAPI
PhysxSchemaPhysxPBDMaterialAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxPBDMaterialAPI();
    }
    return PhysxSchemaPhysxPBDMaterialAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxPBDMaterialAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxPBDMaterialAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxPBDMaterialAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxPBDMaterialAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxPBDMaterialAPI
PhysxSchemaPhysxPBDMaterialAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxPBDMaterialAPI>()) {
        return PhysxSchemaPhysxPBDMaterialAPI(prim);
    }
    return PhysxSchemaPhysxPBDMaterialAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxPBDMaterialAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxPBDMaterialAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxPBDMaterialAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxPBDMaterialAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::GetFrictionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxPBDMaterialFriction);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::CreateFrictionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxPBDMaterialFriction,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::GetParticleFrictionScaleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxPBDMaterialParticleFrictionScale);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::CreateParticleFrictionScaleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxPBDMaterialParticleFrictionScale,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::GetDampingAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxPBDMaterialDamping);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::CreateDampingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxPBDMaterialDamping,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::GetViscosityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxPBDMaterialViscosity);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::CreateViscosityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxPBDMaterialViscosity,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::GetVorticityConfinementAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxPBDMaterialVorticityConfinement);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::CreateVorticityConfinementAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxPBDMaterialVorticityConfinement,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::GetSurfaceTensionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxPBDMaterialSurfaceTension);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::CreateSurfaceTensionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxPBDMaterialSurfaceTension,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::GetCohesionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxPBDMaterialCohesion);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::CreateCohesionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxPBDMaterialCohesion,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::GetAdhesionAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxPBDMaterialAdhesion);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::CreateAdhesionAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxPBDMaterialAdhesion,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::GetParticleAdhesionScaleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxPBDMaterialParticleAdhesionScale);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::CreateParticleAdhesionScaleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxPBDMaterialParticleAdhesionScale,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::GetAdhesionOffsetScaleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxPBDMaterialAdhesionOffsetScale);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::CreateAdhesionOffsetScaleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxPBDMaterialAdhesionOffsetScale,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::GetGravityScaleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxPBDMaterialGravityScale);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::CreateGravityScaleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxPBDMaterialGravityScale,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::GetLiftAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxPBDMaterialLift);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::CreateLiftAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxPBDMaterialLift,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::GetDragAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxPBDMaterialDrag);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::CreateDragAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxPBDMaterialDrag,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::GetDensityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxPBDMaterialDensity);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::CreateDensityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxPBDMaterialDensity,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::GetCflCoefficientAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxPBDMaterialCflCoefficient);
}

UsdAttribute
PhysxSchemaPhysxPBDMaterialAPI::CreateCflCoefficientAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxPBDMaterialCflCoefficient,
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
PhysxSchemaPhysxPBDMaterialAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxPBDMaterialFriction,
        PhysxSchemaTokens->physxPBDMaterialParticleFrictionScale,
        PhysxSchemaTokens->physxPBDMaterialDamping,
        PhysxSchemaTokens->physxPBDMaterialViscosity,
        PhysxSchemaTokens->physxPBDMaterialVorticityConfinement,
        PhysxSchemaTokens->physxPBDMaterialSurfaceTension,
        PhysxSchemaTokens->physxPBDMaterialCohesion,
        PhysxSchemaTokens->physxPBDMaterialAdhesion,
        PhysxSchemaTokens->physxPBDMaterialParticleAdhesionScale,
        PhysxSchemaTokens->physxPBDMaterialAdhesionOffsetScale,
        PhysxSchemaTokens->physxPBDMaterialGravityScale,
        PhysxSchemaTokens->physxPBDMaterialLift,
        PhysxSchemaTokens->physxPBDMaterialDrag,
        PhysxSchemaTokens->physxPBDMaterialDensity,
        PhysxSchemaTokens->physxPBDMaterialCflCoefficient,
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
