// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "GlobalsAreBad.h"
#include "SimulationBackend.h"
#include "cpu/CpuSimulationView.h"
#include "gpu/GpuSimulationView.h"
#include "gpu/CudaCommon.h"

#include <carb/Framework.h>
#include <carb/logging/Log.h>
//#include <carb/settings/ISettings.h>
#include <carb/InterfaceUtils.h>
#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxPrivate.h>
#include <omni/physx/IPhysxSimulation.h>
//#include <omni/physx/IPhysxSettings.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>

#include <limits>

// so we can determine the physx device ordinal
#include <carb/events/IEvents.h>
#include <common/utilities/PhysXErrorCallback.h>
static CarbPhysXErrorCallback gErrorCallback;

/*
// !!!
#include <PxScene.h>
#include <task/PxCpuDispatcher.h>
*/

using namespace pxr;
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

    if (stageId != -1)
    {
        // get the inicated stage from the stage cache
        usdStage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(stageId));
        if (!usdStage)
        {
            CARB_LOG_ERROR("Failed to find stage id %ld in the stage cache", stageId);
            return nullptr;
        }
    }
    else
    {
        // by default, get the stage currently attached to the usd context
        omni::usd::UsdContext* usdCtx = omni::usd::UsdContext::getContext();
        if (!usdCtx)
        {
            CARB_LOG_ERROR("Failed to get USD context");
            return nullptr;
        }

        usdStage = usdCtx->getStage();
        if (!usdStage)
        {
            CARB_LOG_ERROR("Failed to get active stage in the USD context");
            return nullptr;
        }

        stageId = usdCtx->getStageId();
    }

    // reset stale simulation data, if needed
    if (mStageId != stageId)
    {
        if (mCpuSimData || mGpuSimData)
        {
            //CARB_LOG_WARN("*!*!*! STALE SIMULATION DATA DETECTED");
            mCpuSimData.reset();
            mGpuSimData.reset();
            mManualStepCount = 0;
        }
    }

    mUsdStage = usdStage;
    mStageId = stageId;

    // find the physics scene
    PxScene* scene = findPhysicsScene();
    if (!scene)
    {
        CARB_LOG_ERROR("Failed to create simulation view: no active physics scene found");
        return nullptr;
    }

    /*
    {
        PxCpuDispatcher* dispatcher = scene->getCpuDispatcher();
        if (dispatcher)
        {
            printf("~!~!~! PhysX worker count: %u\n", dispatcher->getWorkerCount());
        }
    }
    */

    //carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
    //useGpuPipeline = settings->getAsBool(omni::physx::kSettingSuppressReadback);

    //bool useGpuPipeline = g_physx->isReadbackSuppressed();
    bool useGpuPipeline = scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);

    //printf("~!~!~! Readback suppressed: %d\n", int(useGpuPipeline));

    // figure out the simulation and tensor API device
    if (useGpuPipeline)
    {
        /*
        // get PhysX device ordinal
        int deviceOrdinal = PxGetSuggestedCudaDeviceOrdinal(gErrorCallback);
        CARB_LOG_VERBOSE("~~~~~~~~~~~ PxGetSuggestedCudaDeviceOrdinal: %d", deviceOrdinal);
        if (deviceOrdinal < 0)
        {
            CARB_LOG_ERROR("Failed to get suggested PhysX device ordinal, assuming 0.");
            deviceOrdinal = 0;
        }
        */

        if (!mGpuSimData)
        {
            mGpuSimData = std::make_shared<GpuSimulationData>(*this, stageId);
            if (!mGpuSimData->init(scene))
            {
                CARB_LOG_ERROR("Failed to initialize GPU simulation data");
                mGpuSimData.reset();
                return nullptr;
            }
        }
        GpuSimulationView* simView = new GpuSimulationView(usdStage, mGpuSimData);
        simViews.insert(simView);
        return simView ;
    }
    else
    {
        if (!mCpuSimData)
        {
            mCpuSimData = std::make_shared<CpuSimulationData>(*this, stageId);
        }

        CpuSimulationView* simView = new CpuSimulationView(usdStage, mCpuSimData);
        simViews.insert(simView);
        return simView;
    }
}

void SimulationBackend::removeSimulationView(ISimulationView* view)
{
    auto it = std::find(simViews.begin(), simViews.end(), view);
    if (it != simViews.end())
    {
        simViews.erase(view);
    }
}

void SimulationBackend::reset()
{
    // simulation stopped, so release the data
    mCpuSimData.reset();
    mGpuSimData.reset();

    // reset stage info
    mUsdStage.Reset();

    for (auto simView : simViews)
        simView->invalidate();
    simViews.clear();

    mStageId = -1;

    mManualStepCount = 0;
}

PxScene* SimulationBackend::findPhysicsScene() const
{
    // try our private backdoor first...
    if (g_physxPrivate)
    {
        return g_physxPrivate->getPhysXScene();
    }

    if (!g_physx || !mUsdStage)
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
            // printf("~!~!~! Found physics scene %p at '%s'\n", scene, path.GetText());
            return scene;
        }
    }

    // search for a physics scene prim
    UsdPrimSubtreeRange range = mUsdStage->GetPseudoRoot().GetDescendants();
    for (auto prim : range)
    {
        if (prim.IsA<UsdPhysicsScene>())
        {
            PxScene* scene = static_cast<PxScene*>(g_physx->getPhysXPtr(prim.GetPath(), omni::physx::ePTScene));
            if (scene)
            {
                // printf("~!~!~! Found physics scene %p at '%s'\n", scene, prim.GetPath().GetText());
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

    // see if we should flush the GPU data
    if (mGpuSimData && enableAutoFlush)
    {
        mGpuSimData->flush();
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
