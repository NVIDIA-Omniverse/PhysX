// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-CONFIG-001
// @covers AC-1 AC-2 AC-3 AC-4
// @implements REQ-CAPI-COMPANION-001
// @covers AC-1
#include "ovphysx_companion/configuration.h"
#include "internal/sdk/CompanionRuntime.hpp"
#include <omni/physx/NativeConfiguration.h>
namespace
{
ovphysx_companion_status_t execute(uint64_t attachment,uint32_t action,uint32_t disabled,
    uint32_t size,ovphysx_companion_configuration_snapshot_t* output) noexcept
{
    if (!output || size!=sizeof(*output) || disabled>1) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    CompanionOperationGuard guard(companionRuntime());
    if (!guard) return OVPHYSX_COMPANION_BUSY;
    using namespace omni::physx::native;
    ConfigurationSnapshot result;
    switch(configureAttachment(attachment,action,disabled!=0,result))
    {
    case ConfigurationResult::Invalid:return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    case ConfigurationResult::Stale:return OVPHYSX_COMPANION_STALE;
    case ConfigurationResult::Busy:return OVPHYSX_COMPANION_BUSY;
    case ConfigurationResult::Error:return OVPHYSX_COMPANION_ERROR;
    case ConfigurationResult::Success:break;
    }
    const omni::physx::PhysxAttachmentSettings & s=result.settings;
    const omni::physx::PhysxSharedSettings & r=result.resources;
    uint32_t flags=(s.disableContactProcessing?1u:0u)|(s.updateToUsd?2u:0u)|(s.updateVelocitiesToUsd?4u:0u)|
        (s.updateParticlesToUsd?8u:0u)|(r.useActiveCudaContext?16u:0u)|(r.synchronousCudaLaunches?32u:0u)|
        (r.usePhysxDispatcher?64u:0u)|(s.suppressReadback?128u:0u)|(s.outputVelocitiesLocalSpace?256u:0u);
    *output={sizeof(*output),1,result.attachment,result.step,r.threadCount,flags,r.cudaDevice,r.sceneMultiGpuMode,
        s.gpuPipelineOverride,s.solverTypeOverride,s.viewportGizmoScale,0};
    return OVPHYSX_COMPANION_OK;
}
ovphysx_companion_status_t getConfiguration(uint64_t attachment,uint32_t size,
    ovphysx_companion_configuration_snapshot_t* output) noexcept
{
    return execute(attachment,0,0,size,output);
}
ovphysx_companion_status_t setContactProcessingDisabled(uint64_t attachment,uint32_t disabled,uint32_t size,
    ovphysx_companion_configuration_snapshot_t* output) noexcept
{
    return execute(attachment,1,disabled,size,output);
}

}
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_configuration_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_configuration_api_t* output)
{
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if (version!=1 || size!=sizeof(*output)) return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output={sizeof(*output),1,3,getConfiguration,setContactProcessingDisabled};return OVPHYSX_COMPANION_OK;
}
