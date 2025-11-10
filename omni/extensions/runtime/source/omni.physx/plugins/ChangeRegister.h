// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
void registerDriveTimeSampledChanges(usdparser::AttachedStage&, pxr::UsdPrim& prim, std::string driveAxis);
void registerSceneTimeSampledChanges(usdparser::AttachedStage&, pxr::UsdPrim& prim);
void registerJointTimeSampledChanges(usdparser::AttachedStage&, pxr::UsdPrim& prim);

void registerSpatialTendonChangeParams(usdparser::AttachedStage&, const std::string& instanceName);
void registerTendonAttachmentChangeParams(usdparser::AttachedStage&, const std::string& instanceName);
void registerTendonAttachmentLeafChangeParams(usdparser::AttachedStage&, const std::string& instanceName);

void registerFixedTendonChangeParams(usdparser::AttachedStage&, const std::string& instanceName);
void registerTendonAxisChangeParam(usdparser::AttachedStage&, const std::string& instanceName);

void registerDeformablePoseChangeParams(usdparser::AttachedStage&, const std::string& instanceName);

static const pxr::TfToken gDrivePerformanceEnvelopeMaxActuatorVelocityAttributeNameToken[3] = {
    pxr::PhysxAdditionAttrTokens->maxActuatorVelocityRotX,
    pxr::PhysxAdditionAttrTokens->maxActuatorVelocityRotY,
    pxr::PhysxAdditionAttrTokens->maxActuatorVelocityRotZ
};

static const pxr::TfToken gDrivePerformanceEnvelopeVelocityDependentResistanceAttributeNameToken[3] = {
    pxr::PhysxAdditionAttrTokens->velocityDependentResistanceRotX,
    pxr::PhysxAdditionAttrTokens->velocityDependentResistanceRotY,
    pxr::PhysxAdditionAttrTokens->velocityDependentResistanceRotZ,
};

static const pxr::TfToken gDrivePerformanceEnvelopeSpeedEffortGradientAttributeNameToken[3] = {
    pxr::PhysxAdditionAttrTokens->speedEffortGradientRotX,
    pxr::PhysxAdditionAttrTokens->speedEffortGradientRotY,
    pxr::PhysxAdditionAttrTokens->speedEffortGradientRotZ,
};

static const pxr::TfToken gPhysxJointAxisMaxJointVelocityAttributeNameToken[3] = {
    pxr::PhysxAdditionAttrTokens->maxJointVelocityRotX,
    pxr::PhysxAdditionAttrTokens->maxJointVelocityRotY,
    pxr::PhysxAdditionAttrTokens->maxJointVelocityRotZ,
};
static const pxr::TfToken gPhysxJointAxisArmatureAttributeNameToken[3] = {
    pxr::PhysxAdditionAttrTokens->armatureRotX,
    pxr::PhysxAdditionAttrTokens->armatureRotY,
    pxr::PhysxAdditionAttrTokens->armatureRotZ,
};
static const pxr::TfToken gPhysxJointAxisStaticFrictionEffortAttributeNameToken[3] = {
    pxr::PhysxAdditionAttrTokens->staticFrictionEffortRotX,
    pxr::PhysxAdditionAttrTokens->staticFrictionEffortRotY,
    pxr::PhysxAdditionAttrTokens->staticFrictionEffortRotZ,
};
static const pxr::TfToken gPhysxJointAxisDynamicFrictionEffortAttributeNameToken[3] = {
    pxr::PhysxAdditionAttrTokens->dynamicFrictionEffortRotX,
    pxr::PhysxAdditionAttrTokens->dynamicFrictionEffortRotY,
    pxr::PhysxAdditionAttrTokens->dynamicFrictionEffortRotZ,
};
static const pxr::TfToken gPhysxJointAxisViscousFrictionCoefficientAttributeNameToken[3] = {
    pxr::PhysxAdditionAttrTokens->viscousFrictionCoefficientRotX,
    pxr::PhysxAdditionAttrTokens->viscousFrictionCoefficientRotY,
    pxr::PhysxAdditionAttrTokens->viscousFrictionCoefficientRotZ,
};
} // namespace physx
} // namespace omni
