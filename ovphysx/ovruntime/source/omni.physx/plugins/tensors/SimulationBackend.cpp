// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "tensors/GlobalsAreBad.h"
#include "tensors/SimulationBackend.h"
#include "tensors/cpu/CpuSimulationView.h"
#include "tensors/gpu/GpuSimulationView.h"
#include "tensors/gpu/CudaCommon.h"

#include <carb/Framework.h>
#include <carb/logging/Log.h>
#include <carb/InterfaceUtils.h>
#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxPrivate.h>
#include <omni/physx/IPhysxSimulation.h>

#include <limits>

// so we can determine the physx device ordinal
#include <carb/events/IEvents.h>
#include <common/utilities/PhysXErrorCallback.h>
static CarbPhysXErrorCallback gErrorCallback;

using namespace PXR_NS;
using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

ISimulationView* SimulationBackend::createSimulationView(long stageId)
{
    if (!g_physx)
    {
        CARB_LOG_ERROR("Failed to create simulation view: physics interface is not available");
        return nullptr;
    }

    UsdStageRefPtr usdStage;

    if (stageId == -1)
    {
        // By default, get the stage currently attached to the PhysX simulation interface.
        if (!g_physxSimulation)
        {
            CARB_LOG_ERROR("Failed to get attached USD stage: PhysX simulation interface is not available");
            return nullptr;
        }

        stageId = g_physxSimulation->getAttachedStage();
        if (stageId == 0 || stageId == -1)
        {
            CARB_LOG_ERROR("Failed to get a valid attached USD stage id from PhysX simulation");
            return nullptr;
        }
    }

    usdStage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(stageId));
    if (!usdStage)
    {
        CARB_LOG_ERROR("Failed to find stage id %ld in the stage cache", stageId);
        return nullptr;
    }

    PxScene* scene = findPhysicsScene(usdStage);
    if (!scene)
    {
        CARB_LOG_ERROR("Failed to create simulation view: no active physics scene found");
        return nullptr;
    }

    bool useGpuPipeline = scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);

    std::lock_guard<std::mutex> lock(mStageDataMutex);

    if (useGpuPipeline)
    {
        auto& gpuData = mGpuSimDataByStage[stageId];
        if (!gpuData)
        {
            gpuData = std::make_shared<GpuSimulationData>(*this, stageId);
            if (!gpuData->init(scene))
            {
                CARB_LOG_ERROR("Failed to initialize GPU simulation data");
                // erase() destroys the held shared_ptr; no separate reset() needed.
                mGpuSimDataByStage.erase(stageId);
                return nullptr;
            }
        }
        GpuSimulationView* simView = new GpuSimulationView(usdStage, gpuData);
        mViewsByStage[stageId].push_back(simView);
        return simView;
    }
    else
    {
        auto& cpuData = mCpuSimDataByStage[stageId];
        if (!cpuData)
        {
            cpuData = std::make_shared<CpuSimulationData>(*this, stageId);
        }
        CpuSimulationView* simView = new CpuSimulationView(usdStage, cpuData);
        mViewsByStage[stageId].push_back(simView);
        return simView;
    }
}

void SimulationBackend::removeSimulationView(ISimulationView* view)
{
    std::lock_guard<std::mutex> lock(mStageDataMutex);

    // Remove from the per-stage index. Linear scan is acceptable: views per stage
    // are few and removeSimulationView is not on the hot path.
    for (auto& [sid, views] : mViewsByStage)
    {
        auto it = std::find(views.begin(), views.end(), view);
        if (it != views.end())
        {
            views.erase(it);
            break;
        }
    }
}

void SimulationBackend::reset()
{
    std::lock_guard<std::mutex> lock(mStageDataMutex);

    mCpuSimDataByStage.clear();
    mGpuSimDataByStage.clear();

    for (auto& [sid, views] : mViewsByStage)
        for (auto* view : views)
            view->invalidate();
    mViewsByStage.clear();

    mManualStepCount = 0;
}

void SimulationBackend::resetStage(long stageId)
{
    if (stageId <= 0)
        return;

    std::lock_guard<std::mutex> lock(mStageDataMutex);

    // Invalidate and remove all views for this stage before releasing the data.
    // Views hold shared_ptrs to the data; invalidating them here ensures they
    // stop operating on data this backend considers released.
    auto viewIt = mViewsByStage.find(stageId);
    if (viewIt != mViewsByStage.end())
    {
        for (auto* view : viewIt->second)
            view->invalidate();
        mViewsByStage.erase(viewIt);
    }

    mCpuSimDataByStage.erase(stageId);
    mGpuSimDataByStage.erase(stageId);
    // mManualStepCount intentionally NOT cleared here: it is a process-wide
    // accumulator for steps driven outside the Kit event loop (incremented via
    // BaseSimulationView::incrementStepCount). It offsets getTimestamp()/getStepCount()
    // for ALL stages and must remain monotonic across per-stage resets.
}

PxScene* SimulationBackend::findPhysicsScene(const UsdStageRefPtr& usdStage) const
{
    // try our private backdoor first...
    if (g_physxPrivate)
    {
        return g_physxPrivate->getPhysXScene();
    }

    if (!g_physx || !usdStage)
    {
        return nullptr;
    }

    static std::vector<SdfPath> quickPaths
    {
        SdfPath("/physicsScene"),
        SdfPath("/World/physicsScene"),
    };

    // check low-hanging fruit before full blown search
    for (const SdfPath& path : quickPaths)
    {
        PxScene* scene = static_cast<PxScene*>(g_physx->getPhysXPtr(path, omni::physx::ePTScene));
        if (scene)
        {
            return scene;
        }
    }

    // search for a physics scene prim
    UsdPrimSubtreeRange range = usdStage->GetPseudoRoot().GetDescendants();
    for (auto prim : range)
    {
        if (prim.IsA<UsdPhysicsScene>())
        {
            PxScene* scene = static_cast<PxScene*>(g_physx->getPhysXPtr(prim.GetPath(), omni::physx::ePTScene));
            if (scene)
            {
                return scene;
            }
        }
    }

    return nullptr;
}

void SimulationBackend::prePhysicsUpdate()
{
    // TODO: make it a setting
    bool enableAutoFlush = false;
    if (!enableAutoFlush)
        return;

    std::lock_guard<std::mutex> lock(mStageDataMutex);
    for (auto& kv : mGpuSimDataByStage)
    {
        if (kv.second)
            kv.second->flush();
    }
}

int64_t SimulationBackend::getTimestamp() const
{
    if (g_physxSimulation)
    {
        return static_cast<int64_t>(g_physxSimulation->getSimulationTimestamp() + mManualStepCount);
    }
    return 0;
}

int64_t SimulationBackend::getStepCount() const
{
    if (g_physxSimulation)
    {
        return static_cast<int64_t>(g_physxSimulation->getSimulationStepCount() + mManualStepCount);
    }
    return 0;
}

}
}
}
