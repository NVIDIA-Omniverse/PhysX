// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CONFIG-SNAPSHOT-001
// @covers AC-1 AC-2 AC-3
#include <omni/physx/NativeConfiguration.h>
#include <omni/physx/PhysXRuntime.h>
#include "RuntimeExecution.h"
#include "usdLoad/LoadUsd.h"
#include "usdLoad/AttachedStage.h"
namespace omni::physx::native
{
ConfigurationResult configureAttachment(uint64_t attachment,uint32_t action,bool disabled,
                                        ConfigurationSnapshot& output) noexcept
try
{
    if (action>1) return ConfigurationResult::Invalid;
    if (!runtime::tryGetPhysxInterface()) return ConfigurationResult::Stale;
    std::shared_ptr<omni::physx::usdparser::AttachedStage> owner=usdparser::UsdLoad::getUsdLoad()->acquireAttachment(attachment);
    if (!owner) return ConfigurationResult::Stale;
    RuntimeExecutionScope admission(attachment);
    if (!admission) return ConfigurationResult::Busy;
    if (usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachment) != owner.get())
        return ConfigurationResult::Stale;
    if (action==1) owner->setContactProcessingDisabled(disabled);
    ConfigurationSnapshot result;
    result.attachment=attachment;result.step=owner->simulationTimestamp();
    result.settings=owner->getCachedSettings();result.resources=owner->getSharedSettings();
    output=result;
    return ConfigurationResult::Success;
}
catch (...) { return ConfigurationResult::Error; }
}
