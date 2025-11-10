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
#include ".//physxArticulationAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxArticulationAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxArticulationAPI::~PhysxSchemaPhysxArticulationAPI()
{
}

/* static */
PhysxSchemaPhysxArticulationAPI
PhysxSchemaPhysxArticulationAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxArticulationAPI();
    }
    return PhysxSchemaPhysxArticulationAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxArticulationAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxArticulationAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxArticulationAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxArticulationAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxArticulationAPI
PhysxSchemaPhysxArticulationAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxArticulationAPI>()) {
        return PhysxSchemaPhysxArticulationAPI(prim);
    }
    return PhysxSchemaPhysxArticulationAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxArticulationAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxArticulationAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxArticulationAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxArticulationAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxArticulationAPI::GetArticulationEnabledAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxArticulationArticulationEnabled);
}

UsdAttribute
PhysxSchemaPhysxArticulationAPI::CreateArticulationEnabledAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxArticulationArticulationEnabled,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxArticulationAPI::GetSolverPositionIterationCountAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxArticulationSolverPositionIterationCount);
}

UsdAttribute
PhysxSchemaPhysxArticulationAPI::CreateSolverPositionIterationCountAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxArticulationSolverPositionIterationCount,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxArticulationAPI::GetSolverVelocityIterationCountAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxArticulationSolverVelocityIterationCount);
}

UsdAttribute
PhysxSchemaPhysxArticulationAPI::CreateSolverVelocityIterationCountAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxArticulationSolverVelocityIterationCount,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxArticulationAPI::GetSleepThresholdAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxArticulationSleepThreshold);
}

UsdAttribute
PhysxSchemaPhysxArticulationAPI::CreateSleepThresholdAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxArticulationSleepThreshold,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxArticulationAPI::GetStabilizationThresholdAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxArticulationStabilizationThreshold);
}

UsdAttribute
PhysxSchemaPhysxArticulationAPI::CreateStabilizationThresholdAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxArticulationStabilizationThreshold,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxArticulationAPI::GetEnabledSelfCollisionsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxArticulationEnabledSelfCollisions);
}

UsdAttribute
PhysxSchemaPhysxArticulationAPI::CreateEnabledSelfCollisionsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxArticulationEnabledSelfCollisions,
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
PhysxSchemaPhysxArticulationAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxArticulationArticulationEnabled,
        PhysxSchemaTokens->physxArticulationSolverPositionIterationCount,
        PhysxSchemaTokens->physxArticulationSolverVelocityIterationCount,
        PhysxSchemaTokens->physxArticulationSleepThreshold,
        PhysxSchemaTokens->physxArticulationStabilizationThreshold,
        PhysxSchemaTokens->physxArticulationEnabledSelfCollisions,
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
