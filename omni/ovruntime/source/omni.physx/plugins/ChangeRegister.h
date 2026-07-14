// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <vector>
#include <physicsSchemaTools/physicsSchemaTokens.h>

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
void registerDriveTimeSampledChanges(usdparser::AttachedStage&, PXR_NS::UsdPrim& prim, std::string driveAxis);
void registerSceneTimeSampledChanges(usdparser::AttachedStage&, PXR_NS::UsdPrim& prim);
void registerJointTimeSampledChanges(usdparser::AttachedStage&, PXR_NS::UsdPrim& prim);

void registerSpatialTendonChangeParams(usdparser::AttachedStage&, const std::string& instanceName);
void registerTendonAttachmentChangeParams(usdparser::AttachedStage&, const std::string& instanceName);
void registerTendonAttachmentLeafChangeParams(usdparser::AttachedStage&, const std::string& instanceName);

void registerFixedTendonChangeParams(usdparser::AttachedStage&, const std::string& instanceName);
void registerTendonAxisChangeParam(usdparser::AttachedStage&, const std::string& instanceName);

void registerDeformablePoseChangeParams(usdparser::AttachedStage&, const std::string& instanceName);

static const PXR_NS::TfToken gDrivePerformanceEnvelopeMaxActuatorVelocityAttributeNameToken[3] = {
    PXR_NS::PhysxAdditionAttrTokens->maxActuatorVelocityRotX,
    PXR_NS::PhysxAdditionAttrTokens->maxActuatorVelocityRotY,
    PXR_NS::PhysxAdditionAttrTokens->maxActuatorVelocityRotZ
};

static const PXR_NS::TfToken gDrivePerformanceEnvelopeVelocityDependentResistanceAttributeNameToken[3] = {
    PXR_NS::PhysxAdditionAttrTokens->velocityDependentResistanceRotX,
    PXR_NS::PhysxAdditionAttrTokens->velocityDependentResistanceRotY,
    PXR_NS::PhysxAdditionAttrTokens->velocityDependentResistanceRotZ,
};

static const PXR_NS::TfToken gDrivePerformanceEnvelopeSpeedEffortGradientAttributeNameToken[3] = {
    PXR_NS::PhysxAdditionAttrTokens->speedEffortGradientRotX,
    PXR_NS::PhysxAdditionAttrTokens->speedEffortGradientRotY,
    PXR_NS::PhysxAdditionAttrTokens->speedEffortGradientRotZ,
};

static const PXR_NS::TfToken gPhysxJointAxisMaxJointVelocityAttributeNameToken[3] = {
    PXR_NS::PhysxAdditionAttrTokens->maxJointVelocityRotX,
    PXR_NS::PhysxAdditionAttrTokens->maxJointVelocityRotY,
    PXR_NS::PhysxAdditionAttrTokens->maxJointVelocityRotZ,
};
static const PXR_NS::TfToken gPhysxJointAxisArmatureAttributeNameToken[3] = {
    PXR_NS::PhysxAdditionAttrTokens->armatureRotX,
    PXR_NS::PhysxAdditionAttrTokens->armatureRotY,
    PXR_NS::PhysxAdditionAttrTokens->armatureRotZ,
};
static const PXR_NS::TfToken gPhysxJointAxisStaticFrictionEffortAttributeNameToken[3] = {
    PXR_NS::PhysxAdditionAttrTokens->staticFrictionEffortRotX,
    PXR_NS::PhysxAdditionAttrTokens->staticFrictionEffortRotY,
    PXR_NS::PhysxAdditionAttrTokens->staticFrictionEffortRotZ,
};
static const PXR_NS::TfToken gPhysxJointAxisDynamicFrictionEffortAttributeNameToken[3] = {
    PXR_NS::PhysxAdditionAttrTokens->dynamicFrictionEffortRotX,
    PXR_NS::PhysxAdditionAttrTokens->dynamicFrictionEffortRotY,
    PXR_NS::PhysxAdditionAttrTokens->dynamicFrictionEffortRotZ,
};
static const PXR_NS::TfToken gPhysxJointAxisViscousFrictionCoefficientAttributeNameToken[3] = {
    PXR_NS::PhysxAdditionAttrTokens->viscousFrictionCoefficientRotX,
    PXR_NS::PhysxAdditionAttrTokens->viscousFrictionCoefficientRotY,
    PXR_NS::PhysxAdditionAttrTokens->viscousFrictionCoefficientRotZ,
};
} // namespace physx
} // namespace omni
