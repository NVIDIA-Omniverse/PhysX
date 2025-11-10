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
#include ".//physxParticleSystem.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxParticleSystem,
        TfType::Bases< UsdGeomGprim > >();
    
    // Register the usd prim typename as an alias under UsdSchemaBase. This
    // enables one to call
    // TfType::Find<UsdSchemaBase>().FindDerivedByName("PhysxParticleSystem")
    // to find TfType<PhysxSchemaPhysxParticleSystem>, which is how IsA queries are
    // answered.
    TfType::AddAlias<UsdSchemaBase, PhysxSchemaPhysxParticleSystem>("PhysxParticleSystem");
}

/* virtual */
PhysxSchemaPhysxParticleSystem::~PhysxSchemaPhysxParticleSystem()
{
}

/* static */
PhysxSchemaPhysxParticleSystem
PhysxSchemaPhysxParticleSystem::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxParticleSystem();
    }
    return PhysxSchemaPhysxParticleSystem(stage->GetPrimAtPath(path));
}

/* static */
PhysxSchemaPhysxParticleSystem
PhysxSchemaPhysxParticleSystem::Define(
    const UsdStagePtr &stage, const SdfPath &path)
{
    static TfToken usdPrimTypeName("PhysxParticleSystem");
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxParticleSystem();
    }
    return PhysxSchemaPhysxParticleSystem(
        stage->DefinePrim(path, usdPrimTypeName));
}

/* virtual */
UsdSchemaKind PhysxSchemaPhysxParticleSystem::_GetSchemaKind() const
{
    return PhysxSchemaPhysxParticleSystem::schemaKind;
}

/* static */
const TfType &
PhysxSchemaPhysxParticleSystem::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxParticleSystem>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxParticleSystem::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxParticleSystem::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::GetParticleSystemEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->particleSystemEnabled);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::CreateParticleSystemEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->particleSystemEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::GetContactOffsetAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->contactOffset);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::CreateContactOffsetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->contactOffset,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::GetRestOffsetAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->restOffset);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::CreateRestOffsetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->restOffset,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::GetParticleContactOffsetAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->particleContactOffset);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::CreateParticleContactOffsetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->particleContactOffset,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::GetSolidRestOffsetAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->solidRestOffset);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::CreateSolidRestOffsetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->solidRestOffset,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::GetFluidRestOffsetAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->fluidRestOffset);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::CreateFluidRestOffsetAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->fluidRestOffset,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::GetEnableCCDAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->enableCCD);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::CreateEnableCCDAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->enableCCD,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::GetSolverPositionIterationCountAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->solverPositionIterationCount);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::CreateSolverPositionIterationCountAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->solverPositionIterationCount,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::GetMaxDepenetrationVelocityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->maxDepenetrationVelocity);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::CreateMaxDepenetrationVelocityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->maxDepenetrationVelocity,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::GetWindAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->wind);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::CreateWindAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->wind,
                       SdfValueTypeNames->Float3,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::GetMaxNeighborhoodAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->maxNeighborhood);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::CreateMaxNeighborhoodAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->maxNeighborhood,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::GetNeighborhoodScaleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->neighborhoodScale);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::CreateNeighborhoodScaleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->neighborhoodScale,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::GetMaxVelocityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->maxVelocity);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::CreateMaxVelocityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->maxVelocity,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::GetGlobalSelfCollisionEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->globalSelfCollisionEnabled);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::CreateGlobalSelfCollisionEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->globalSelfCollisionEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::GetNonParticleCollisionEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->nonParticleCollisionEnabled);
}

UsdAttribute
PhysxSchemaPhysxParticleSystem::CreateNonParticleCollisionEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->nonParticleCollisionEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdRelationship
PhysxSchemaPhysxParticleSystem::GetSimulationOwnerRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->simulationOwner);
}

UsdRelationship
PhysxSchemaPhysxParticleSystem::CreateSimulationOwnerRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->simulationOwner,
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
PhysxSchemaPhysxParticleSystem::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->particleSystemEnabled,
        PhysxSchemaTokens->contactOffset,
        PhysxSchemaTokens->restOffset,
        PhysxSchemaTokens->particleContactOffset,
        PhysxSchemaTokens->solidRestOffset,
        PhysxSchemaTokens->fluidRestOffset,
        PhysxSchemaTokens->enableCCD,
        PhysxSchemaTokens->solverPositionIterationCount,
        PhysxSchemaTokens->maxDepenetrationVelocity,
        PhysxSchemaTokens->wind,
        PhysxSchemaTokens->maxNeighborhood,
        PhysxSchemaTokens->neighborhoodScale,
        PhysxSchemaTokens->maxVelocity,
        PhysxSchemaTokens->globalSelfCollisionEnabled,
        PhysxSchemaTokens->nonParticleCollisionEnabled,
    };
    static TfTokenVector allNames =
        _ConcatenateAttributeNames(
            UsdGeomGprim::GetSchemaAttributeNames(true),
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
