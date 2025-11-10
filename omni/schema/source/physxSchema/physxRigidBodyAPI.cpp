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
#include ".//physxRigidBodyAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxRigidBodyAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxRigidBodyAPI::~PhysxSchemaPhysxRigidBodyAPI()
{
}

/* static */
PhysxSchemaPhysxRigidBodyAPI
PhysxSchemaPhysxRigidBodyAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxRigidBodyAPI();
    }
    return PhysxSchemaPhysxRigidBodyAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxRigidBodyAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxRigidBodyAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxRigidBodyAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxRigidBodyAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxRigidBodyAPI
PhysxSchemaPhysxRigidBodyAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxRigidBodyAPI>()) {
        return PhysxSchemaPhysxRigidBodyAPI(prim);
    }
    return PhysxSchemaPhysxRigidBodyAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxRigidBodyAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxRigidBodyAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxRigidBodyAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxRigidBodyAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetLinearDampingAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodyLinearDamping);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateLinearDampingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodyLinearDamping,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetAngularDampingAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodyAngularDamping);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateAngularDampingAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodyAngularDamping,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetMaxLinearVelocityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodyMaxLinearVelocity);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateMaxLinearVelocityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodyMaxLinearVelocity,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetMaxAngularVelocityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodyMaxAngularVelocity);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateMaxAngularVelocityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodyMaxAngularVelocity,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetSleepThresholdAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodySleepThreshold);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateSleepThresholdAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodySleepThreshold,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetStabilizationThresholdAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodyStabilizationThreshold);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateStabilizationThresholdAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodyStabilizationThreshold,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetMaxDepenetrationVelocityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodyMaxDepenetrationVelocity);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateMaxDepenetrationVelocityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodyMaxDepenetrationVelocity,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetMaxContactImpulseAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodyMaxContactImpulse);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateMaxContactImpulseAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodyMaxContactImpulse,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetSolverPositionIterationCountAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodySolverPositionIterationCount);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateSolverPositionIterationCountAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodySolverPositionIterationCount,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetSolverVelocityIterationCountAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodySolverVelocityIterationCount);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateSolverVelocityIterationCountAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodySolverVelocityIterationCount,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetEnableCCDAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodyEnableCCD);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateEnableCCDAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodyEnableCCD,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetEnableSpeculativeCCDAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodyEnableSpeculativeCCD);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateEnableSpeculativeCCDAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodyEnableSpeculativeCCD,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetRetainAccelerationsAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodyRetainAccelerations);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateRetainAccelerationsAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodyRetainAccelerations,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetEnableGyroscopicForcesAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodyEnableGyroscopicForces);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateEnableGyroscopicForcesAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodyEnableGyroscopicForces,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetDisableGravityAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodyDisableGravity);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateDisableGravityAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodyDisableGravity,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetSolveContactAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodySolveContact);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateSolveContactAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodySolveContact,
                       SdfValueTypeNames->Bool,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetLockedPosAxisAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodyLockedPosAxis);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateLockedPosAxisAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodyLockedPosAxis,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetLockedRotAxisAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodyLockedRotAxis);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateLockedRotAxisAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodyLockedRotAxis,
                       SdfValueTypeNames->Int,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetContactSlopCoefficientAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodyContactSlopCoefficient);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateContactSlopCoefficientAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodyContactSlopCoefficient,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::GetCfmScaleAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physxRigidBodyCfmScale);
}

UsdAttribute
PhysxSchemaPhysxRigidBodyAPI::CreateCfmScaleAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physxRigidBodyCfmScale,
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
PhysxSchemaPhysxRigidBodyAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physxRigidBodyLinearDamping,
        PhysxSchemaTokens->physxRigidBodyAngularDamping,
        PhysxSchemaTokens->physxRigidBodyMaxLinearVelocity,
        PhysxSchemaTokens->physxRigidBodyMaxAngularVelocity,
        PhysxSchemaTokens->physxRigidBodySleepThreshold,
        PhysxSchemaTokens->physxRigidBodyStabilizationThreshold,
        PhysxSchemaTokens->physxRigidBodyMaxDepenetrationVelocity,
        PhysxSchemaTokens->physxRigidBodyMaxContactImpulse,
        PhysxSchemaTokens->physxRigidBodySolverPositionIterationCount,
        PhysxSchemaTokens->physxRigidBodySolverVelocityIterationCount,
        PhysxSchemaTokens->physxRigidBodyEnableCCD,
        PhysxSchemaTokens->physxRigidBodyEnableSpeculativeCCD,
        PhysxSchemaTokens->physxRigidBodyRetainAccelerations,
        PhysxSchemaTokens->physxRigidBodyEnableGyroscopicForces,
        PhysxSchemaTokens->physxRigidBodyDisableGravity,
        PhysxSchemaTokens->physxRigidBodySolveContact,
        PhysxSchemaTokens->physxRigidBodyLockedPosAxis,
        PhysxSchemaTokens->physxRigidBodyLockedRotAxis,
        PhysxSchemaTokens->physxRigidBodyContactSlopCoefficient,
        PhysxSchemaTokens->physxRigidBodyCfmScale,
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
