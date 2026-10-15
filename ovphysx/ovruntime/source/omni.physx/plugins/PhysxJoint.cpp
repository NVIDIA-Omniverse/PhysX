// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-2

#include <omni/physx/IPhysxJoint.h>
#include <PxPhysicsAPI.h>
#include <common/foundation/CarbPhysXCast.h>
#include "OmniPhysX.h"
#include "usdLoad/LoadUsd.h"
#include "RuntimeExecution.h"
#include "AttachmentJointLookup.h"

namespace omni
{
namespace physx
{

static carb::Float4 physxJointGetJointFramePhysxToUsdQuat(AttachHandle attach, usdparser::ObjectId id)
{
    RuntimeExecutionScope operation(attach);
    const usdparser::AttachedStage* owner = operation ? usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach) : nullptr;
    return attachmentJoint::frameCorrection(owner, id);
}

static void physxJointGetJointStateData(AttachHandle attach, usdparser::ObjectId id, JointStateData* data)
{
    RuntimeExecutionScope operation(attach);
    const usdparser::AttachedStage* owner = operation ? usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach) : nullptr;
    if (owner)
        attachmentJoint::stateData(owner, id, data);
}

} // namespace physx
} // namespace omni

void fillInterface(omni::physx::IPhysxJoint& iface)
{
    iface.getJointStateData = omni::physx::physxJointGetJointStateData;
    iface.getJointFramePhysxToUsdQuat = omni::physx::physxJointGetJointFramePhysxToUsdQuat;
}
