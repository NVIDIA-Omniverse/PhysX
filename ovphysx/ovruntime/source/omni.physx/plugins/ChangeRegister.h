// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <vector>
#include <physxSchema/axisInstanceTokens.h>
#include <omniUsdPhysicsDeformableSchema/tokens.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
struct ChangeParams;
class AttachedStage;
} // namespace usdparser

void registerChangeParams(std::vector<usdparser::ChangeParams>& changeParams);
void registerDriveTimeSampledChanges(usdparser::AttachedStage&, const PXR_NS::SdfPath& jointPrimKey, std::string driveAxis);
void registerSceneTimeSampledChanges(usdparser::AttachedStage&, const PXR_NS::SdfPath& scenePrimKey);
void registerJointTimeSampledChanges(usdparser::AttachedStage&, const PXR_NS::SdfPath& jointPrimKey);

void registerSpatialTendonChangeParams(usdparser::AttachedStage&, const std::string& instanceName);
void registerTendonAttachmentChangeParams(usdparser::AttachedStage&, const std::string& instanceName);
void registerTendonAttachmentLeafChangeParams(usdparser::AttachedStage&, const std::string& instanceName);

void registerFixedTendonChangeParams(usdparser::AttachedStage&, const std::string& instanceName);
void registerTendonAxisChangeParam(usdparser::AttachedStage&, const std::string& instanceName);

void registerDeformablePoseChangeParams(usdparser::AttachedStage&, const std::string& instanceName);

static const PXR_NS::TfToken gDrivePerformanceEnvelopeMaxActuatorVelocityAttributeNameToken[3] = {
    PXR_NS::PhysxAxisInstanceTokens->maxActuatorVelocityRotX,
    PXR_NS::PhysxAxisInstanceTokens->maxActuatorVelocityRotY,
    PXR_NS::PhysxAxisInstanceTokens->maxActuatorVelocityRotZ
};

static const PXR_NS::TfToken gDrivePerformanceEnvelopeVelocityDependentResistanceAttributeNameToken[3] = {
    PXR_NS::PhysxAxisInstanceTokens->velocityDependentResistanceRotX,
    PXR_NS::PhysxAxisInstanceTokens->velocityDependentResistanceRotY,
    PXR_NS::PhysxAxisInstanceTokens->velocityDependentResistanceRotZ,
};

static const PXR_NS::TfToken gDrivePerformanceEnvelopeSpeedEffortGradientAttributeNameToken[3] = {
    PXR_NS::PhysxAxisInstanceTokens->speedEffortGradientRotX,
    PXR_NS::PhysxAxisInstanceTokens->speedEffortGradientRotY,
    PXR_NS::PhysxAxisInstanceTokens->speedEffortGradientRotZ,
};

static const PXR_NS::TfToken gPhysxJointAxisMaxJointVelocityAttributeNameToken[3] = {
    PXR_NS::PhysxAxisInstanceTokens->maxJointVelocityRotX,
    PXR_NS::PhysxAxisInstanceTokens->maxJointVelocityRotY,
    PXR_NS::PhysxAxisInstanceTokens->maxJointVelocityRotZ,
};
static const PXR_NS::TfToken gPhysxJointAxisArmatureAttributeNameToken[3] = {
    PXR_NS::PhysxAxisInstanceTokens->armatureRotX,
    PXR_NS::PhysxAxisInstanceTokens->armatureRotY,
    PXR_NS::PhysxAxisInstanceTokens->armatureRotZ,
};
static const PXR_NS::TfToken gPhysxJointAxisStaticFrictionEffortAttributeNameToken[3] = {
    PXR_NS::PhysxAxisInstanceTokens->staticFrictionEffortRotX,
    PXR_NS::PhysxAxisInstanceTokens->staticFrictionEffortRotY,
    PXR_NS::PhysxAxisInstanceTokens->staticFrictionEffortRotZ,
};
static const PXR_NS::TfToken gPhysxJointAxisDynamicFrictionEffortAttributeNameToken[3] = {
    PXR_NS::PhysxAxisInstanceTokens->dynamicFrictionEffortRotX,
    PXR_NS::PhysxAxisInstanceTokens->dynamicFrictionEffortRotY,
    PXR_NS::PhysxAxisInstanceTokens->dynamicFrictionEffortRotZ,
};
static const PXR_NS::TfToken gPhysxJointAxisViscousFrictionCoefficientAttributeNameToken[3] = {
    PXR_NS::PhysxAxisInstanceTokens->viscousFrictionCoefficientRotX,
    PXR_NS::PhysxAxisInstanceTokens->viscousFrictionCoefficientRotY,
    PXR_NS::PhysxAxisInstanceTokens->viscousFrictionCoefficientRotZ,
};
} // namespace physx
} // namespace omni
