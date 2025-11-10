// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "FabricSync.h"
#include "OmniPhysX.h"

#include <omni/core/ITypeFactory.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/fabric/stage/StageReaderWriter.h>

#include <usdrt/scenegraph/usd/usd/stage.h>

namespace omni
{
namespace physx
{

void flushUsdToFabric(const omni::fabric::UsdStageId& stageId, bool changeTime, double timeCode)
{
    omni::fabric::StageReaderWriter stageInProgress = OmniPhysX::getInstance().getIStageReaderWriter()->get(stageId);
    if (stageInProgress.getId() != omni::fabric::kInvalidStageReaderWriterId)
    {
        // Creation of a UsdStage object is lightweight, it's basically a view over the USD and Fabric stages.
        usdrt::UsdStageRefPtr stage = usdrt::UsdStage::Attach(stageId, stageInProgress.getId());
        if (stage)
        {
            if (changeTime)
            {
                stage->SynchronizeToFabric(usdrt::TimeChange::LazyUpdate, usdrt::UsdTimeCode(timeCode));
            }
            else
            {
                stage->SynchronizeToFabric(usdrt::TimeChange::NoUpdate);
            }
        }
        else
        {
            CARB_LOG_ERROR("PhysX was not able to attach USDRT stage. Cannot flush to Fabric.");
        }
    }
}

void flushUsdToFabric(const PXR_NS::UsdStageWeakPtr& stage, bool changeTime, double timeCode)
{
    if (stage)
    {
        omni::fabric::UsdStageId stageId = { uint64_t(pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt()) };
        flushUsdToFabric(stageId, changeTime, timeCode);
    }
}

}
}
