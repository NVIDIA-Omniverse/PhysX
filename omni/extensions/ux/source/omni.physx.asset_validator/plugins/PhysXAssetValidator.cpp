// SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#define CARB_EXPORTS
#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <private/omni/physx/IPhysxAssetValidator.h>
#include <omni/physx/IPhysx.h>

const struct carb::PluginImplDesc kPluginImpl = { "omni.physx.asset_validator.plugin", "Physics", "NVIDIA",
                                                  carb::PluginHotReload::eDisabled, "dev" };
CARB_PLUGIN_IMPL(kPluginImpl, omni::physx::IPhysxAssetValidator)
CARB_PLUGIN_IMPL_DEPS(omni::physx::IPhysx)


CARB_EXPORT void carbOnPluginStartup()
{
}

CARB_EXPORT void carbOnPluginShutdown()
{
}

namespace omni
{
namespace physx
{
extern void runBackwardCompatibilityOnStage(pxr::UsdStageWeakPtr stage);
extern bool checkBackwardCompatibilityOnStage(pxr::UsdStageWeakPtr stage);
extern const char* getBackwardsCompatibilityCheckLog();
} // namespace physx
} // namespace omni

bool backwardCompatibilityChecker(uint64_t stageId)
{
    pxr::UsdStageRefPtr stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(long(stageId)));
    if(!stage)
    {
        CARB_LOG_ERROR("PhysXAssetValidator: Invalid stageId \"%lld\" passed", stageId);
        return false;
    }
    return omni::physx::checkBackwardCompatibilityOnStage(stage);
}

const char* getBackwardCompatibilityLog()
{
    return omni::physx::getBackwardsCompatibilityCheckLog();
}

void runBackwardCompatibilityCheck(uint64_t stageId)
{
    pxr::UsdStageRefPtr stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(long(stageId)));
    if(!stage)
    {
        CARB_LOG_ERROR("PhysXAssetValidator: Invalid stageId \"%lld\" passed", stageId);
        return;
    }
    omni::physx::runBackwardCompatibilityOnStage(stage);
}

// Defined in JointStateChecker.cpp
extern bool jointStateChecker(uint64_t stageId, uint64_t primId, bool applyModification);
// Defined in ConvexGPUCompatibilityChecker.cpp
extern bool convexGPUCompatibilityChecker(uint64_t stageId, uint64_t primId);

void fillInterface(omni::physx::IPhysxAssetValidator& iface)
{
    iface.jointStateIsValid = [](uint64_t stageId, uint64_t primId) { return jointStateChecker(stageId, primId, false); };
    iface.jointStateApplyFix = [](uint64_t stageId, uint64_t primId) { jointStateChecker(stageId, primId, true); };
    iface.backwardCompatibilityCheck = &backwardCompatibilityChecker;
    iface.getBackwardCompatibilityLog = &getBackwardCompatibilityLog;
    iface.runBackwardCompatibilityCheck = &runBackwardCompatibilityCheck;
    iface.convexGPUCompatibilityIsValid = [](uint64_t stageId, uint64_t primId) { return convexGPUCompatibilityChecker(stageId, primId); };
}
