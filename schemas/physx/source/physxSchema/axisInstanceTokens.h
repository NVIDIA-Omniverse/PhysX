// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// GENERATED (header-only) by tools/gen_axis_instance_tokens.py. DO NOT EDIT.
// Concrete per-axis instances of the physxJointAxis / physxDrivePerformanceEnvelope
// multiple-apply schemas, derived from PhysxSchemaTokens templates (no hand-authored
// strings).
/// \file physxSchema/axisInstanceTokens.h
#ifndef PHYSXSCHEMA_AXIS_INSTANCE_TOKENS_H
#define PHYSXSCHEMA_AXIS_INSTANCE_TOKENS_H

#include "physxSchema/tokens.h"
#include <pxr/pxr.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <vector>

PXR_NAMESPACE_OPEN_SCOPE

/// \class PhysxAxisInstanceTokensType
///
/// Per-axis (angular/linear/rotX/rotY/rotZ) instance tokens for the physxJointAxis and
/// physxDrivePerformanceEnvelope multiple-apply schemas, e.g.
/// PhysxAxisInstanceTokens->maxJointVelocityAngular == "physxJointAxis:angular:maxJointVelocity".
struct PhysxAxisInstanceTokensType {
    PhysxAxisInstanceTokensType();
    const TfToken maxActuatorVelocityAngular;
    const TfToken maxActuatorVelocityLinear;
    const TfToken maxActuatorVelocityRotX;
    const TfToken maxActuatorVelocityRotY;
    const TfToken maxActuatorVelocityRotZ;
    const TfToken speedEffortGradientAngular;
    const TfToken speedEffortGradientLinear;
    const TfToken speedEffortGradientRotX;
    const TfToken speedEffortGradientRotY;
    const TfToken speedEffortGradientRotZ;
    const TfToken velocityDependentResistanceAngular;
    const TfToken velocityDependentResistanceLinear;
    const TfToken velocityDependentResistanceRotX;
    const TfToken velocityDependentResistanceRotY;
    const TfToken velocityDependentResistanceRotZ;
    const TfToken armatureAngular;
    const TfToken armatureLinear;
    const TfToken armatureRotX;
    const TfToken armatureRotY;
    const TfToken armatureRotZ;
    const TfToken dynamicFrictionEffortAngular;
    const TfToken dynamicFrictionEffortLinear;
    const TfToken dynamicFrictionEffortRotX;
    const TfToken dynamicFrictionEffortRotY;
    const TfToken dynamicFrictionEffortRotZ;
    const TfToken maxJointVelocityAngular;
    const TfToken maxJointVelocityLinear;
    const TfToken maxJointVelocityRotX;
    const TfToken maxJointVelocityRotY;
    const TfToken maxJointVelocityRotZ;
    const TfToken staticFrictionEffortAngular;
    const TfToken staticFrictionEffortLinear;
    const TfToken staticFrictionEffortRotX;
    const TfToken staticFrictionEffortRotY;
    const TfToken staticFrictionEffortRotZ;
    const TfToken viscousFrictionCoefficientAngular;
    const TfToken viscousFrictionCoefficientLinear;
    const TfToken viscousFrictionCoefficientRotX;
    const TfToken viscousFrictionCoefficientRotY;
    const TfToken viscousFrictionCoefficientRotZ;
    /// A vector of all of the tokens listed above.
    const std::vector<TfToken> allTokens;
};

inline PhysxAxisInstanceTokensType::PhysxAxisInstanceTokensType() :
    maxActuatorVelocityAngular(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxDrivePerformanceEnvelope_MultipleApplyTemplate_MaxActuatorVelocity, TfToken("angular"))),
    maxActuatorVelocityLinear(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxDrivePerformanceEnvelope_MultipleApplyTemplate_MaxActuatorVelocity, TfToken("linear"))),
    maxActuatorVelocityRotX(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxDrivePerformanceEnvelope_MultipleApplyTemplate_MaxActuatorVelocity, TfToken("rotX"))),
    maxActuatorVelocityRotY(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxDrivePerformanceEnvelope_MultipleApplyTemplate_MaxActuatorVelocity, TfToken("rotY"))),
    maxActuatorVelocityRotZ(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxDrivePerformanceEnvelope_MultipleApplyTemplate_MaxActuatorVelocity, TfToken("rotZ"))),
    speedEffortGradientAngular(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxDrivePerformanceEnvelope_MultipleApplyTemplate_SpeedEffortGradient, TfToken("angular"))),
    speedEffortGradientLinear(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxDrivePerformanceEnvelope_MultipleApplyTemplate_SpeedEffortGradient, TfToken("linear"))),
    speedEffortGradientRotX(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxDrivePerformanceEnvelope_MultipleApplyTemplate_SpeedEffortGradient, TfToken("rotX"))),
    speedEffortGradientRotY(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxDrivePerformanceEnvelope_MultipleApplyTemplate_SpeedEffortGradient, TfToken("rotY"))),
    speedEffortGradientRotZ(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxDrivePerformanceEnvelope_MultipleApplyTemplate_SpeedEffortGradient, TfToken("rotZ"))),
    velocityDependentResistanceAngular(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxDrivePerformanceEnvelope_MultipleApplyTemplate_VelocityDependentResistance, TfToken("angular"))),
    velocityDependentResistanceLinear(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxDrivePerformanceEnvelope_MultipleApplyTemplate_VelocityDependentResistance, TfToken("linear"))),
    velocityDependentResistanceRotX(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxDrivePerformanceEnvelope_MultipleApplyTemplate_VelocityDependentResistance, TfToken("rotX"))),
    velocityDependentResistanceRotY(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxDrivePerformanceEnvelope_MultipleApplyTemplate_VelocityDependentResistance, TfToken("rotY"))),
    velocityDependentResistanceRotZ(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxDrivePerformanceEnvelope_MultipleApplyTemplate_VelocityDependentResistance, TfToken("rotZ"))),
    armatureAngular(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_Armature, TfToken("angular"))),
    armatureLinear(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_Armature, TfToken("linear"))),
    armatureRotX(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_Armature, TfToken("rotX"))),
    armatureRotY(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_Armature, TfToken("rotY"))),
    armatureRotZ(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_Armature, TfToken("rotZ"))),
    dynamicFrictionEffortAngular(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_DynamicFrictionEffort, TfToken("angular"))),
    dynamicFrictionEffortLinear(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_DynamicFrictionEffort, TfToken("linear"))),
    dynamicFrictionEffortRotX(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_DynamicFrictionEffort, TfToken("rotX"))),
    dynamicFrictionEffortRotY(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_DynamicFrictionEffort, TfToken("rotY"))),
    dynamicFrictionEffortRotZ(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_DynamicFrictionEffort, TfToken("rotZ"))),
    maxJointVelocityAngular(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_MaxJointVelocity, TfToken("angular"))),
    maxJointVelocityLinear(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_MaxJointVelocity, TfToken("linear"))),
    maxJointVelocityRotX(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_MaxJointVelocity, TfToken("rotX"))),
    maxJointVelocityRotY(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_MaxJointVelocity, TfToken("rotY"))),
    maxJointVelocityRotZ(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_MaxJointVelocity, TfToken("rotZ"))),
    staticFrictionEffortAngular(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_StaticFrictionEffort, TfToken("angular"))),
    staticFrictionEffortLinear(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_StaticFrictionEffort, TfToken("linear"))),
    staticFrictionEffortRotX(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_StaticFrictionEffort, TfToken("rotX"))),
    staticFrictionEffortRotY(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_StaticFrictionEffort, TfToken("rotY"))),
    staticFrictionEffortRotZ(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_StaticFrictionEffort, TfToken("rotZ"))),
    viscousFrictionCoefficientAngular(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_ViscousFrictionCoefficient, TfToken("angular"))),
    viscousFrictionCoefficientLinear(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_ViscousFrictionCoefficient, TfToken("linear"))),
    viscousFrictionCoefficientRotX(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_ViscousFrictionCoefficient, TfToken("rotX"))),
    viscousFrictionCoefficientRotY(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_ViscousFrictionCoefficient, TfToken("rotY"))),
    viscousFrictionCoefficientRotZ(UsdSchemaRegistry::MakeMultipleApplyNameInstance(PhysxSchemaTokens->physxJointAxis_MultipleApplyTemplate_ViscousFrictionCoefficient, TfToken("rotZ"))),
    allTokens({
        maxActuatorVelocityAngular,
        maxActuatorVelocityLinear,
        maxActuatorVelocityRotX,
        maxActuatorVelocityRotY,
        maxActuatorVelocityRotZ,
        speedEffortGradientAngular,
        speedEffortGradientLinear,
        speedEffortGradientRotX,
        speedEffortGradientRotY,
        speedEffortGradientRotZ,
        velocityDependentResistanceAngular,
        velocityDependentResistanceLinear,
        velocityDependentResistanceRotX,
        velocityDependentResistanceRotY,
        velocityDependentResistanceRotZ,
        armatureAngular,
        armatureLinear,
        armatureRotX,
        armatureRotY,
        armatureRotZ,
        dynamicFrictionEffortAngular,
        dynamicFrictionEffortLinear,
        dynamicFrictionEffortRotX,
        dynamicFrictionEffortRotY,
        dynamicFrictionEffortRotZ,
        maxJointVelocityAngular,
        maxJointVelocityLinear,
        maxJointVelocityRotX,
        maxJointVelocityRotY,
        maxJointVelocityRotZ,
        staticFrictionEffortAngular,
        staticFrictionEffortLinear,
        staticFrictionEffortRotX,
        staticFrictionEffortRotY,
        staticFrictionEffortRotZ,
        viscousFrictionCoefficientAngular,
        viscousFrictionCoefficientLinear,
        viscousFrictionCoefficientRotX,
        viscousFrictionCoefficientRotY,
        viscousFrictionCoefficientRotZ,
    })
{
}

/// \var PhysxAxisInstanceTokens
///
/// Codeless: defined inline (header-only) -- no compiled tokens.cpp / library.
inline TfStaticData<PhysxAxisInstanceTokensType> PhysxAxisInstanceTokens;

PXR_NAMESPACE_CLOSE_SCOPE

#endif
