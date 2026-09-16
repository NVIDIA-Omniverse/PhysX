// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-6 AC-10
 *
 * `PhysXSetup::getPhysics()` derives the PhysX tolerances scale from the attached
 * SOURCE's `metersPerUnit` (AC-6), and its no-attach arm states the 1.0 default at
 * the site instead of routing a known-null stage through a transient `UsdSource`
 * that would answer with the same default indistinguishably (AC-10).
 */

/**
 * @implements REQ-SDK-LIFECYCLE-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-OMNIPVD-TRANSPORT-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-OMNIPVD-LATE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10
 *
 * @implements REQ-SIM-NVTX-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-COOK-LIFETIME-001
 * @covers AC-5
 */

#include <carb/Defines.h>
#include <cudamanager/PxCudaContextManager.h> // acquireReference() in acquireCudaContextManager()
#include "PhysXTools.h"
#include "Setup.h"
#include <common/utilities/PhysXErrorCallback.h>
#include <common/utilities/MemoryMacros.h>
#include <cstring>
#include <string>
#include <filesystem>

#include "omnipvd/PxOmniPvd.h"
#include "OmniPvdLibraryFunctions.h"
#include "OmniPvdWriter.h"
#include "OmniPvdFileWriteStream.h"
#include "OmniPvdSocketWriteStream.h"

#include "PhysXScene.h"
#include "PhysXDefines.h"
#include "PhysXUSDProperties.h"
#include "OmniPhysX.h"
#include "usdLoad/LoadUsd.h"
#include "ContactReport.h"
#include "SceneMultiGPUMode.h"

#include "internal/Internal.h"
#include "CookingDataAsync.h"
#include "MeshCache.h"
#include "ConeCylinderConvexMesh.h"
#include "attachment/PhysXPoissonSampling.h"
#include "particles/PhysXParticlePost.h"

#include "PhysXCarbCPUDispatcher.h"
#include "service/CookingComputeService.h"
#include "service/CookingTask.h"
#include "utility/MeshSimplifyInternal.h"
#include "utils/Nvtx.h"
#include "utils/ProfileZoneBalance.h"

#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>

#include <omni/physx/IPhysxSettings.h>
#include <private/omni/physx/IPhysxCookingServicePrivate.h>
#include <omni/physx/IPhysxFoundation.h>
#include <omni/physx/IOptionalCuda.h>
#include "PhysXFoundation.h"

bool addLastSlash(std::string& fileDirectory)
{
    if ((fileDirectory.back() != '/') && (fileDirectory.back() != '\\'))
    {
        if (fileDirectory.find('\\') != std::string::npos)
        {
            fileDirectory += '\\';
        }
        else
        {
            fileDirectory += '/';
        }
        return true;
    }
    return false;
}

using namespace physx;
using namespace omni::physx::internal;
using namespace omni::physx;
using namespace omni::physx::usdparser;

namespace
{

bool isOmniPvdRecordingCapable(carb::settings::ISettings& settings)
{
    return (settings.getAsBool(kOmniPvdOutputEnabled) && !settings.getAsBool(kOmniPvdIsOVDStage)) ||
           settings.getAsBool(kOmniPvdRecordingCapable);
}

omni::physx::ICookingComputeService* getOwnedCookingService()
{
    return omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingComputeService();
}

// Handed to the cooking service as its AcquireSharedCudaContextManagerFn: returns the manager with
// a reference already taken, so a cooking job on a UJITSO worker can never end up holding a
// manager that setupGPU()/clearCudaContextManagers() destroyed on the main thread meanwhile.
//
// @implements REQ-COOK-CUDACTX-001
// @covers AC-1
::physx::PxCudaContextManager* acquireOwnedCudaContextManager()
{
    return omni::physx::OmniPhysX::getInstance().getPhysXSetup().acquireCudaContextManager();
}

uint32_t CARB_ABI cookingPumpAsyncContext(omni::physx::PhysxCookingAsyncContext context)
{
    if (auto* service = getOwnedCookingService())
    {
        return service->pumpAsyncContext(context);
    }
    return 0;
}

bool CARB_ABI cookingCancelTask(omni::physx::PhysxCookingOperationHandle handle, bool invokeCallbackAnyway)
{
    if (auto* service = getOwnedCookingService())
    {
        return service->cancelTask(handle, invokeCallbackAnyway);
    }
    return false;
}

uint32_t CARB_ABI cookingCancelAllTasks(omni::physx::PhysxCookingAsyncContext context)
{
    if (auto* service = getOwnedCookingService())
    {
        return service->cancelAllTasks(context);
    }
    return 0;
}

bool CARB_ABI cookingWaitForTaskToFinish(omni::physx::PhysxCookingOperationHandle handle, int64_t timeoutMs)
{
    if (auto* service = getOwnedCookingService())
    {
        return service->waitForTaskToFinish(handle, timeoutMs);
    }
    return false;
}

omni::physx::PhysxCookingAsyncContext CARB_ABI cookingCreateAsyncContext(
    omni::physx::PhysxCookingAsyncContextParameters& parameters)
{
    if (auto* service = getOwnedCookingService())
    {
        return service->createAsyncContext(parameters);
    }
    return nullptr;
}

void CARB_ABI cookingDestroyAsyncContext(omni::physx::PhysxCookingAsyncContext context)
{
    if (auto* service = getOwnedCookingService())
    {
        service->destroyAsyncContext(context);
    }
}

omni::physx::PhysxCookingOperationHandle CARB_ABI cookingRequestTriangleMeshCookedData(
    omni::physx::PhysxCookingAsyncContext context,
    const omni::physx::PhysxCookingComputeRequest& request,
    const omni::physx::TriangleMeshCookingParams& triangleMeshCookingParams)
{
    if (auto* service = getOwnedCookingService())
    {
        omni::physx::SdfMeshCookingParams sdfMeshCookingParams;
        sdfMeshCookingParams.sdfResolution = 0;
        return service->requestTriangleMeshCookedData(context, request, triangleMeshCookingParams, sdfMeshCookingParams,
                                                      nullptr);
    }
    return nullptr;
}

omni::physx::PhysxCookingOperationHandle CARB_ABI cookingRequestSdfMeshCookedData(
    omni::physx::PhysxCookingAsyncContext context,
    const omni::physx::PhysxCookingComputeRequest& request,
    const omni::physx::TriangleMeshCookingParams& triangleMeshCookingParams,
    const omni::physx::SdfMeshCookingParams& sdfMeshCookingParams)
{
    if (auto* service = getOwnedCookingService())
    {
        return service->requestTriangleMeshCookedData(context, request, triangleMeshCookingParams, sdfMeshCookingParams,
                                                      nullptr);
    }
    return nullptr;
}

omni::physx::PhysxCookingOperationHandle CARB_ABI cookingRequestConvexMeshCookedData(
    omni::physx::PhysxCookingAsyncContext context,
    const omni::physx::PhysxCookingComputeRequest& request,
    const omni::physx::ConvexMeshCookingParams& desc)
{
    if (auto* service = getOwnedCookingService())
    {
        return service->requestConvexMeshCookedData(context, request, desc);
    }
    return nullptr;
}

omni::physx::PhysxCookingOperationHandle CARB_ABI cookingRequestConvexMeshDecompositionCookedData(
    omni::physx::PhysxCookingAsyncContext context,
    const omni::physx::PhysxCookingComputeRequest& request,
    const omni::physx::ConvexDecompositionCookingParams& desc)
{
    if (auto* service = getOwnedCookingService())
    {
        return service->requestConvexMeshDecompositionCookedData(context, request, desc);
    }
    return nullptr;
}

omni::physx::PhysxCookingOperationHandle CARB_ABI cookingRequestSphereFillCookedData(
    omni::physx::PhysxCookingAsyncContext context,
    const omni::physx::PhysxCookingComputeRequest& request,
    const omni::physx::SphereFillCookingParams& desc)
{
    if (auto* service = getOwnedCookingService())
    {
        return service->requestSphereFillCookedData(context, request, desc);
    }
    return nullptr;
}

uint32_t CARB_ABI cookingGetActiveTaskCount(omni::physx::PhysxCookingAsyncContext context)
{
    if (auto* service = getOwnedCookingService())
    {
        return service->getActiveTaskCount(context);
    }
    return 0;
}

uint32_t CARB_ABI cookingGetFinishedCookingTasksCount()
{
    if (auto* service = getOwnedCookingService())
    {
        return service->getFinishedCookingTasksCount();
    }
    return 0;
}

void CARB_ABI cookingResetMeshCacheContents()
{
    if (auto* service = getOwnedCookingService())
    {
        service->resetMeshCacheContents();
    }
}

bool CARB_ABI cookingComputeConformingTetrahedralMesh(
    const omni::physx::PhysxCookingTetrahedralMeshInput& meshInput,
    omni::physx::PhysxCookingTetrahedralMeshOutput& meshOutput)
{
    return omni::physx::computeConformingTetrahedralMeshInternal(
        meshOutput.dstTetPoints, meshOutput.dstTetPointsSize, meshOutput.dstTetIndices, meshOutput.dstTetIndicesSize,
        meshInput.srcTriPoints, meshInput.srcTriPointsSize, meshInput.srcTriIndices, meshInput.srcTriIndicesSize,
        meshOutput.allocateBytes);
}

bool CARB_ABI cookingComputeVoxelTetrahedralMesh(
    const omni::physx::PhysxCookingTetrahedralMeshInput& meshInput,
    omni::physx::PhysxCookingTetrahedralMeshOutput& meshOutput,
    omni::physx::PhysxCookingTetrahedralVoxelMeshParameters parameters)
{
    return omni::physx::computeVoxelTetrahedralMeshInternal(
        meshOutput.dstTetPoints, meshOutput.dstTetPointsSize, meshOutput.dstTetIndices, meshOutput.dstTetIndicesSize,
        meshOutput.dstEmbedding, meshOutput.dstEmbeddingSize, meshInput.srcTriPoints, meshInput.srcTriPointsSize,
        meshInput.srcTriIndices, meshInput.srcTriIndicesSize, parameters.voxelResolution, parameters.numTetsPerVoxel,
        parameters.anchorNodes, meshOutput.allocateBytes);
}

omni::physx::PhysxCookingOperationHandle CARB_ABI cookingRequestSdfMeshCookedDataPrivate(
    omni::physx::PhysxCookingAsyncContext context,
    const omni::physx::PhysxCookingComputeRequest& request,
    const omni::physx::TriangleMeshCookingParams& triangleMeshCookingParams,
    const omni::physx::SdfMeshCookingParams& sdfMeshCookingParams,
    ::physx::PxCudaContextManager* cudaContextManager)
{
    if (auto* service = getOwnedCookingService())
    {
        return service->requestTriangleMeshCookedData(context, request, triangleMeshCookingParams, sdfMeshCookingParams,
                                                      cudaContextManager);
    }
    return nullptr;
}

omni::physx::PhysxCookingOperationHandle CARB_ABI cookingRequestParticlePoissonSamplingCookedData(
    omni::physx::PhysxCookingAsyncContext context,
    const omni::physx::PhysxCookingComputeRequest& request,
    const omni::physx::ParticlePoissonSamplingCookingParams& params)
{
    if (auto* service = getOwnedCookingService())
    {
        return service->requestParticlePoissonSamplingCookedData(context, request, params);
    }
    return nullptr;
}

void CARB_ABI cookingReadParticlePoissonSamplingData(
    omni::physx::PhysxCookingParticlePoissonSamplingData& out,
    const omni::physx::PhysxCookedDataSpan& cookedData)
{
    cookingtask::readParticlePoissonSamplingData(out, cookedData);
}

bool CARB_ABI cookingIsOVCNodeDeprecated()
{
    CARB_LOG_WARN("isOVCNode is deprecated and will be removed in a future release");
    auto* settings = carb::getCachedInterface<carb::settings::ISettings>();
    return settings && settings->getAsBool("/app/ovc_deployment");
}

omni::physx::PhysxCookingOperationHandle CARB_ABI cookingRequestDeformableVolumeMeshCookedData(
    omni::physx::PhysxCookingAsyncContext context,
    const omni::physx::PhysxCookingComputeRequest& request,
    const omni::physx::DeformableVolumeMeshCookingParams& params)
{
    if (auto* service = getOwnedCookingService())
    {
        return service->requestDeformableVolumeMeshCookedData(context, request, params);
    }
    return nullptr;
}

omni::physx::PhysxCookingOperationHandle CARB_ABI cookingRequestVolumeDeformableBodyCookedData(
    omni::physx::PhysxCookingAsyncContext context,
    const omni::physx::PhysxCookingComputeRequest& request,
    const omni::physx::VolumeDeformableBodyCookingParams& params)
{
    if (auto* service = getOwnedCookingService())
    {
        return service->requestVolumeDeformableBodyCookedData(context, request, params);
    }
    return nullptr;
}

omni::physx::PhysxCookingOperationHandle CARB_ABI cookingRequestSurfaceDeformableBodyCookedData(
    omni::physx::PhysxCookingAsyncContext context,
    const omni::physx::PhysxCookingComputeRequest& request,
    const omni::physx::SurfaceDeformableBodyCookingParams& params)
{
    if (auto* service = getOwnedCookingService())
    {
        return service->requestSurfaceDeformableBodyCookedData(context, request, params);
    }
    return nullptr;
}

void CARB_ABI cookingReadVolumeDeformableBodyData(omni::physx::PhysxCookingVolumeDeformableBodyData& out,
                                                  const omni::physx::PhysxCookedDataSpan& cookedData)
{
    cookingtask::readVolumeDeformableBodyData(out, cookedData);
}

void CARB_ABI cookingReadSurfaceDeformableBodyData(omni::physx::PhysxCookingSurfaceDeformableBodyData& out,
                                                   const omni::physx::PhysxCookedDataSpan& cookedData)
{
    cookingtask::readSurfaceDeformableBodyData(out, cookedData);
}

void fillCookingServiceInterfaces(omni::physx::IPhysxCookingService& service,
                                  omni::physx::IPhysxCookingServicePrivate& privateService)
{
    service.pumpAsyncContext = cookingPumpAsyncContext;
    service.cancelTask = cookingCancelTask;
    service.cancelAllTasks = cookingCancelAllTasks;
    service.waitForTaskToFinish = cookingWaitForTaskToFinish;
    service.createAsyncContext = cookingCreateAsyncContext;
    service.destroyAsyncContext = cookingDestroyAsyncContext;
    service.requestTriangleMeshCookedData = cookingRequestTriangleMeshCookedData;
    service.requestSdfMeshCookedData = cookingRequestSdfMeshCookedData;
    service.requestConvexMeshCookedData = cookingRequestConvexMeshCookedData;
    service.requestConvexMeshDecompositionCookedData = cookingRequestConvexMeshDecompositionCookedData;
    service.requestSphereFillCookedData = cookingRequestSphereFillCookedData;

    privateService.getActiveTaskCount = cookingGetActiveTaskCount;
    privateService.getFinishedCookingTasksCount = cookingGetFinishedCookingTasksCount;
    privateService.resetMeshCacheContents = cookingResetMeshCacheContents;
    privateService.computeConformingTetrahedralMesh = cookingComputeConformingTetrahedralMesh;
    privateService.computeVoxelTetrahedralMesh = cookingComputeVoxelTetrahedralMesh;
    privateService.requestSdfMeshCookedData = cookingRequestSdfMeshCookedDataPrivate;
    privateService.requestParticlePoissonSamplingCookedData = cookingRequestParticlePoissonSamplingCookedData;
    privateService.readParticlePoissonSamplingData = cookingReadParticlePoissonSamplingData;
    privateService.isOVCNodeDeprecated = cookingIsOVCNodeDeprecated;
    privateService.requestDeformableVolumeMeshCookedData = cookingRequestDeformableVolumeMeshCookedData;
    privateService.requestVolumeDeformableBodyCookedData = cookingRequestVolumeDeformableBodyCookedData;
    privateService.requestSurfaceDeformableBodyCookedData = cookingRequestSurfaceDeformableBodyCookedData;
    privateService.readVolumeDeformableBodyData = cookingReadVolumeDeformableBodyData;
    privateService.readSurfaceDeformableBodyData = cookingReadSurfaceDeformableBodyData;
}

} // namespace

#if USE_PHYSX_GPU
#include <cuda.h>
#endif

#if !CARB_PLATFORM_WINDOWS
#define sprintf_s snprintf
#endif


static const ::physx::PxF32 tConeOrCylinderWidth = 2.0f;
static const ::physx::PxF32 tConeOrCylinderRadius = 1.0f;

static const ::physx::PxU32 tCylinderNumCirclePoints = 30;  // with 32 it crashes on GPU
static const ::physx::PxU32 tConeNumCirclePoints = 30;


// Bridges the PhysX SDK profile zones to the Carbonite profiler and to NVTX. The
// two sinks are gated independently, so either, both, or neither can be active;
// the SDK allows only one PxProfilerCallback, which is why they share one object.
//
// Detached zones may end on a different thread than they started on and do not
// close in LIFO order, so NVTX publishes them as explicit start/end ranges keyed by
// name and context id rather than through the per-thread push/pop stack. profilerData
// is deliberately unused for that: PX_PROFILE_STOP_CROSSTHREAD always passes NULL.
//
// The NVTX gate is read once per non-detached zone, when it opens, and the answer is
// recorded for the matching close. It is mutable -- creating a second PhysX SDK
// writes it, and that may happen while another thread is mid-step -- so a second
// read at close time could pop a range that was never pushed, closing the enclosing
// range instead.
class OmniPhysxProfileCallback : public PxProfilerCallback
{
    virtual void* zoneStart(const char* eventName, bool detached, uint64_t contextId)
    {
        if (detached)
        {
            // Gated inside, on the same map that pairs the range ids.
            nvtx::startDetachedZone(eventName, contextId);
            return nullptr;
        }

        const bool pushNvtx = profilebalance::onZoneStart(nvtx::isEnabled());
        CARB_PROFILE_BEGIN(kPhysicsProfilerMask, "%s",eventName);
        if (pushNvtx)
            nvtx::pushZone(eventName);
        return nullptr;
    }

    virtual void zoneEnd(void* profilerData, const char* eventName, bool detached, uint64_t contextId)
    {
        if (detached)
        {
            nvtx::endDetachedZone(eventName, contextId);
            return;
        }

        // Both sinks below are per-thread stacks, so an unmatched close has to be
        // dropped rather than published: popping would close a zone this thread
        // does not own. onZoneEnd() reports the first occurrence, and hands back the
        // NVTX decision this zone was opened with.
        bool popNvtx = false;
        if (!profilebalance::onZoneEnd(popNvtx))
            return;

        CARB_PROFILE_END(kPhysicsProfilerMask);
        if (popNvtx)
            nvtx::popZone();
    }

    virtual void recordData(int32_t value, const char* valueName, uint64_t contextId)
    {
        CARB_PROFILE_VALUE(value, kPhysicsProfilerMask, "%s (%lu)", valueName, (unsigned long)contextId);
    }

    virtual void recordData(float value, const char* valueName, uint64_t contextId)
    {
        CARB_PROFILE_VALUE(value, kPhysicsProfilerMask, "%s (%lu)", valueName, (unsigned long)contextId);
    }

    virtual void recordFrame(const char* name, uint64_t contextId)
    {
        CARB_PROFILE_FRAME(kPhysicsProfilerMask, "Frame: %s (%lu)", name, (unsigned long)contextId);
    }

} gProfilerCallback;


struct OmniPvdRecordingTime
{
    OmniPvdRecordingTime()
    {
        mWasSet = false;
    }

    bool wasSet()
    {
        return mWasSet;
    }

    bool isSame(OmniPvdRecordingTime& ts)
    {
        return ((year == ts.year)
            && (month == ts.month)
            && (day == ts.day)
            && (hour == ts.hour)
            && (minute == ts.minute)
            && (second == ts.second));
    }

    void setWithDiffCounterIncreaseIfSame(OmniPvdRecordingTime& ts)
    {
        if (wasSet()) {
            if (isSame(ts))
            {
                diffCounter++;
            }
            else
            {
                diffCounter = 1;
            }
        }
        else
        {
            diffCounter = 1;
        }

        year = ts.year;
        month = ts.month;
        day = ts.day;
        hour = ts.hour;
        minute = ts.minute;
        second = ts.second;

        mWasSet = true;
    }

    uint32_t year;
    uint32_t month;
    uint32_t day;
    uint32_t hour;
    uint32_t minute;
    uint32_t second;

    uint32_t diffCounter;

    bool mWasSet;
};

#ifdef OMNI_PVD_WIN

void getLocalOmniPvdTime(OmniPvdRecordingTime& timeStamp)
{
    SYSTEMTIME lt;
    GetLocalTime(&lt);
    // Maybe to use UTC at some point?
    //GetSystemTime(&lt);

    timeStamp.year = lt.wYear;
    timeStamp.month = lt.wMonth;
    timeStamp.day = lt.wDay;
    timeStamp.hour = lt.wHour;
    timeStamp.minute = lt.wMinute;
    timeStamp.second = lt.wSecond;
}

#else

void getLocalOmniPvdTime(OmniPvdRecordingTime& timeStamp)
{
    struct timeval tv;
    struct tm *tm;
    gettimeofday(&tv, NULL);
    // Maybe to use UTC at some point?
    tm = localtime(&tv.tv_sec);

    timeStamp.year = tm->tm_year + 1900;
    timeStamp.month = tm->tm_mon + 1;
    timeStamp.day = tm->tm_mday;
    timeStamp.hour = tm->tm_hour;
    timeStamp.minute = tm->tm_min;
    timeStamp.second = tm->tm_sec;
}

#endif

static OmniPvdRecordingTime gLastOmniPvdTime;

//void logFunc(char *logLine)
//{
//    std::cout << logLine << std::endl;
//}

namespace omni
{
    namespace physx
    {
        PhysXSetup::PhysXSetup() : mDefaultScene(nullptr)
        {
            mPhysxFoundation = &omni::physx::foundation::getInterface();
            initializeCookingServiceInterfaces();

            mErrorCallback = new CarbPhysXErrorCallback();


            // Subscribe to changes to both kSettingPVDEnabled and kOmniPvdOutputEnabled so we can regenerate the SDK
            // connection to the legacy PVD or OmniPVD in case settings change
            {
                mISettings = carb::getCachedInterface<carb::settings::ISettings>();

                carb::dictionary::SubscriptionId* subID;
                auto legacyPvdEnabledChangedLambda = [](const carb::dictionary::Item* changedItem,
                                                        carb::dictionary::ChangeEventType eventType, void* userData) {
                    if (!OmniPhysX::isStarted()) return;
                    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
                    OmniCachedSettings& cachedSettings = omniPhysX.getCachedSettings();
                    PhysXSetup *ptr = static_cast<PhysXSetup*>(userData);
                    bool setting = omniPhysX.getISettings()->getAsBool(kSettingPVDEnabled);
                    if (setting != (bool)ptr->getPvd())
                    {

                        ptr->changePVDSettings(setting, ptr->isOmniPvdRecording());
                    }
                };
                subID = mISettings->subscribeToNodeChangeEvents(kSettingPVDEnabled, legacyPvdEnabledChangedLambda, this);
                mSubscribedSettings.push_back(subID);

                auto omniPvdEnabledChangedLambda = [](const carb::dictionary::Item* changedItem,
                                                      carb::dictionary::ChangeEventType eventType, void* userData) {
                    if (!OmniPhysX::isStarted()) return;
                    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
                    OmniCachedSettings& cachedSettings = omniPhysX.getCachedSettings();
                    PhysXSetup *ptr = static_cast<PhysXSetup*>(userData);
                    bool setting = omniPhysX.getISettings()->getAsBool(kOmniPvdOutputEnabled);
                    // check if the stage is OmniPVD, or switched from being an OmniPVD stage to a non OmniPVD stage
                    // not sure what is required really
                    bool isOVDStage = omniPhysX.getISettings()->getAsBool(kOmniPvdIsOVDStage);
                    // If isOVDStage -> should force omniPVD to be false
                    // If isOVDStage is false -> should function as it used to
                    if (!isOVDStage)
                    {
                        if (setting != ptr->isOmniPvdRecording())
                        {
                            ptr->changePVDSettings((bool)ptr->getPvd(), setting);
                        }
                    }
                    else
                    {
                        if (ptr->isOmniPvdRecording())
                        {
                            ptr->changePVDSettings((bool)ptr->getPvd(), false);
                        }
                    }
                };
                subID = mISettings->subscribeToNodeChangeEvents(kOmniPvdOutputEnabled, omniPvdEnabledChangedLambda, this);
                mSubscribedSettings.push_back(subID);

                subID = mISettings->subscribeToNodeChangeEvents(kOmniPvdIsOVDStage, omniPvdEnabledChangedLambda, this);
                mSubscribedSettings.push_back(subID);

            }

            clearCudaContextManagersAndRefillWithNull(1);
        }

        PhysXSetup::~PhysXSetup()
        {
            releaseCookingService();
            delete mErrorCallback;
            mErrorCallback = nullptr;

            for (auto subId : mSubscribedSettings)
            {
                mISettings->unsubscribeToChangeEvents(subId);
            }
            mSubscribedSettings.clear();

            mISettings = nullptr;
        }

        void PhysXSetup::changePVDSettings(bool enableLegacyPVD, bool enableOmniPVD)
        {
            if (enableLegacyPVD == (bool)mVisualDebugger && enableOmniPVD == isOmniPvdRecording())
            {
                return; // do nothing, we're already set with the correct pvd options
            }
            // PxScene instances are owned by the current PxPhysics instance even when their steppers are complete.
            // Recreating PxPhysics while a scene is alive leaves PhysXScene::mScene dangling.
            if (!mPhysXScenes.empty())
            {
                CARB_LOG_WARN(
                    "PVD settings change deferred until next attach; one or more PhysX scenes are still alive");
                return;
            }
            cleanupPhysics();
            getPhysics(); // Regenerate PhysXSDK object together with all necessary PVD connections
        }

        void PhysXSetup::resetPhysXErrorCounter()
        {
            mErrorCallback->resetErrorCounter();
        }

        void PhysXSetup::setMaxNumberOfPhysXErrors(uint32_t maxNumberOfPhysXErrors)
        {
            mErrorCallback->setMaxNumErrors(maxNumberOfPhysXErrors);
        }

        void PhysXSetup::clearCudaContextManagersLocked()
        {
            for (auto& cudaContextManager : mCudaContextManagers)
            {
                // Drops our reference only. A manager an in-flight cooking task still uses
                // survives on that task's own reference (REQ-COOK-CUDACTX-001) and is destroyed
                // when the last holder releases it.
                SAFE_RELEASE(cudaContextManager);
            }
            mCudaContextManagers.clear();
        }

        void PhysXSetup::clearCudaContextManagers()
        {
            std::lock_guard<std::mutex> lock(mCudaContextManagerMutex);
            clearCudaContextManagersLocked();
        }

        void PhysXSetup::clearCudaContextManagersAndRefillWithNull(int numToFill)
        {
            std::lock_guard<std::mutex> lock(mCudaContextManagerMutex);
            clearCudaContextManagersLocked();

            for (int ord = 0; ord < numToFill; ++ord)
            {
                mCudaContextManagers.push_back(nullptr);
            }
        }

        void PhysXSetup::setupGPU()
        {
#if USE_PHYSX_GPU
            if (mPhysxFoundation && mPhysxFoundation->isCpuMode && mPhysxFoundation->isCpuMode())
            {
                // CPU-only mode: never call into the CUDA driver API.
                return;
            }
            // AD: OM-117654 - the code in this function can throw a PhysX error, and the callback might access an outdated cuda context in that case.
            mErrorCallback->setCudaContextManager(nullptr);

            // OMPE-95128: NpScene caches the PxCudaContextManager* at construction; releasing or replacing it
            // while scenes are alive leaves dangling vtable pointers. Defer all manager changes to the next attach.
            // OMPE-102199: mPhysXScenes is emptied before releasePhysXScenes() destroys the scenes, so consult
            // the teardown flag too - the scenes in that window still hold the manager.
            const bool hasLiveScenes = !mPhysXScenes.empty() || mReleasingPhysXScenes;

            // When a refresh is deferred under live scenes, still mirror createOrRefreshPxCudaContextManager's
            // abort-mode reset so resume can recover from a prior GPU error.
            // Call only while scenes are quiesced - setAbortMode() writes non-atomic state shared with GPU
            // worker threads, so it would race with a live simulate().
            auto clearAbortMode = [](PxCudaContextManager* mgr)
            {
                if (!mgr)
                    return;
                PxCudaContext* ctx = mgr->getCudaContext();
                if (ctx && ctx->isInAbortMode())
                    ctx->setAbortMode(false);
            };

            // Keep the getNextCudaContextManager() round-robin cursor while the manager vector is unchanged;
            // reset it only when the vector is rebuilt.
            if (!hasLiveScenes)
                mNextCudaContextManagerId = 0;
            const bool enableSynchronousKernelLaunches = OmniPhysX::getInstance().getCachedSettings().enableSynchronousKernelLaunches;
            const int multiGPUMode = OmniPhysX::getInstance().getISettings()->getAsInt(kSettingSceneMultiGPUMode);

            if ((mCudaLaunchSynchronous != enableSynchronousKernelLaunches) || (mSceneMultiGPUMode != multiGPUMode))
            {
                if (hasLiveScenes)
                {
                    // Sticky: warn once per deferral; flag cleared on rebuild or in releasePhysXScenes.
                    if (!mCudaSettingsDeferralWarned)
                    {
                        CARB_LOG_WARN("PhysX: enableSynchronousKernelLaunches/sceneMultiGPUMode change deferred until next attach; "
                                      "live scenes still reference the current PxCudaContextManager. "
                                      "Stop the simulation for the new setting to take effect.");
                        mCudaSettingsDeferralWarned = true;
                    }
                }
                else
                {
                    // clear cuda context managers so they can be recreated with correct params with one default space
                    clearCudaContextManagersAndRefillWithNull(1);
                    mCudaLaunchSynchronous = enableSynchronousKernelLaunches;
                    mSceneMultiGPUMode = multiGPUMode;
                    mCudaSettingsDeferralWarned = false;
                }
            }

            // check MultiGPU mode settings
            int availableDeviceCount = 0;
            omni::physx::foundation::getOptionalCudaInterface().deviceGetCount(&availableDeviceCount, nullptr);
            if ((mSceneMultiGPUMode != (int)SceneMultiGPUMode::eDisabled) && availableDeviceCount < 2)
            {
                CARB_LOG_WARN("Warning: MultiGpuMode set to %d, but less than 2 CUDA devices found. Reverting to default!", mSceneMultiGPUMode);
                OmniPhysX::getInstance().getISettings()->setInt(kSettingSceneMultiGPUMode, (int)SceneMultiGPUMode::eDisabled);
                mSceneMultiGPUMode = 0;
            }

            if (mSceneMultiGPUMode)
            {
                int32_t firstOrdinal = 0;

                // skip the first device
                if (mSceneMultiGPUMode == 2)
                {
                    firstOrdinal = 1;
                    availableDeviceCount--;
                }

                if (mCudaContextManagers.size() != availableDeviceCount)
                {
                    if (hasLiveScenes)
                    {
                        // Sticky: log once per deferral; re-armed on rebuild or in releasePhysXScenes.
                        if (!mCudaDeviceCountDeferralWarned)
                        {
                            CARB_LOG_INFO("PhysX: CUDA device count change deferred until next attach; "
                                          "live scenes still reference the current PxCudaContextManager.");
                            mCudaDeviceCountDeferralWarned = true;
                        }
                    }
                    else
                    {
                        clearCudaContextManagersAndRefillWithNull(availableDeviceCount);
                        mCudaDeviceCountDeferralWarned = false;
                    }
                }

                for (int32_t off = 0, ord = firstOrdinal; off < availableDeviceCount; ++off, ++ord)
                {
                    // Resize was skipped under hasLiveScenes; the vector may be smaller than availableDeviceCount.
                    if (off >= (int32_t)mCudaContextManagers.size())
                        break;

                    if (hasLiveScenes && mCudaContextManagers[off] != nullptr)
                    {
                        // Recover the OOM/abort case in place; a non-OOM sticky error can only be cleared by
                        // recreating the manager (unsafe with live scenes), so warn and defer it to next attach.
                        clearAbortMode(mCudaContextManagers[off]);
                        PxCudaContext* ctx = mCudaContextManagers[off]->getCudaContext();
                        if (ctx && ctx->getLastError() != CUDA_SUCCESS)
                            CARB_LOG_WARN("PhysX: PxCudaContextManager for device ordinal %d has a sticky CUDA error "
                                          "that cannot be cleared while scenes are live; stop and restart the "
                                          "simulation to recover.", ord);
                        continue;
                    }

                    // spawn a separate context and a context manager for each available cuda device
                    omni::physx::PhysxFoundationDeviceOrdinal ordinal;
                    ordinal.mode = omni::physx::PhysxFoundationDeviceOrdinal::eMODE_DEVICE_ORDINAL;
                    ordinal.deviceOrdinal = ord;
                    // createOrRefreshPxCudaContextManager() may release and replace the entry, so it
                    // mutates mCudaContextManagers and has to be serialized against
                    // acquireCudaContextManager() on the cooking worker threads.
                    {
                        std::lock_guard<std::mutex> lock(mCudaContextManagerMutex);
                        if(!mPhysxFoundation->createOrRefreshPxCudaContextManager(ordinal, mFoundation, mCudaContextManagers[off], enableSynchronousKernelLaunches))
                        {
                            SAFE_RELEASE(mCudaContextManagers[off]);
                            CARB_LOG_ERROR("Unable to create PxCudaContextManager for device with ordinal %d!", ord);
                        }
                    }
                }

                // set the first device as default
                mPhysxFoundation->setCudaDevice(0);
            }
            else
            {
                omni::physx::PhysxFoundationDeviceOrdinal ordinal;
                mPhysxFoundation->getSingleCudaContextManagerOrdinal(ordinal);
                if (hasLiveScenes && !mCudaContextManagers.empty() && mCudaContextManagers[0] != nullptr)
                {
                    // Recover the OOM/abort case in place; a non-OOM sticky error can only be cleared by
                    // recreating the manager (unsafe with live scenes), so warn and defer it to next attach.
                    clearAbortMode(mCudaContextManagers[0]);
                    PxCudaContext* ctx = mCudaContextManagers[0]->getCudaContext();
                    if (ctx && ctx->getLastError() != CUDA_SUCCESS)
                        CARB_LOG_WARN("PhysX: PxCudaContextManager has a sticky CUDA error that cannot be cleared "
                                      "while scenes are live; stop and restart the simulation to recover.");
                }
                else
                {
                    // Mutates mCudaContextManagers (release + replace); serialize against
                    // acquireCudaContextManager() on the cooking worker threads.
                    std::lock_guard<std::mutex> lock(mCudaContextManagerMutex);
                    if (!mPhysxFoundation->createOrRefreshPxCudaContextManager(ordinal, mFoundation, mCudaContextManagers[0], enableSynchronousKernelLaunches))
                    {
                        SAFE_RELEASE(mCudaContextManagers[0]);
                        CARB_LOG_ERROR("Unable to create PxCudaContextManager!");
                    }
                }
            }

            mErrorCallback->setCudaContextManager(getCudaContextManager());
#endif
        }

        void PhysXSetup::createPhysics(const PxTolerancesScale& tolerances)
        {
            releaseCookingAsyncContext();

            // ### DEFENSIVE OMPE-102186
            // createPhysics() returns without creating a foundation if PxCreateFoundation fails,
            // and CARB_ASSERT compiles out in release, so everything below - PxCreatePvd,
            // PxCreatePhysics, setupGPU - would dereference a null foundation. Bail out with an
            // error instead; the underlying failure is already reported by createPhysics().
            CARB_ASSERT(mFoundation);
            if (!mFoundation)
            {
                CARB_LOG_ERROR("Cannot create PhysX: foundation was not created.");
                return;
            }

            const bool omniPVDWasActive = mOmniPvd && mOmniPvd->isSampling();
            unregisterVehiclePvdCallback();
            releaseVehiclePvdTelemetry();
            SAFE_RELEASE(mPhysics);
            carb::settings::ISettings* iSettings = OmniPhysX::getInstance().getISettings();
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();

            if (iSettings->getAsBool(kSettingPVDEnabled) && !mVisualDebugger)
            {
                mVisualDebugger = PxCreatePvd(*mFoundation);
            }
            if (!iSettings->getAsBool(kSettingPVDEnabled) && mVisualDebugger)
            {
                SAFE_RELEASE(mVisualDebugger)
            }

            SAFE_RELEASE(mOmniPvd)
            const bool omniPVDFileWasActive = mOmniPvdStreamKind == OmniPvdStreamKind::eStartupFile;
            const bool writeStreamClosedOk = releaseOmniPvdWriteStream();
            writeOutOmniPVDFile(omniPVDWasActive && omniPVDFileWasActive, writeStreamClosedOk);

            const bool omniPVDOutputEnabled = iSettings->getAsBool(kOmniPvdOutputEnabled);
            const bool omniPVDIsOVDStage = iSettings->getAsBool(kOmniPvdIsOVDStage);
            bool isOmniPVDRecording = false;
            std::string ovdRecordingFileName;
            if (omniPVDOutputEnabled && !omniPVDIsOVDStage)
            {
                carb::settings::ScopedRead settingsRead(iSettings);
                size_t transportLength = 0;
                size_t outputDirectoryLength = 0;
                size_t tcpAddressLength = 0;
                const char* transport = iSettings->getStringBuffer(kOmniPvdTransport, &transportLength);
                const char* outputDirectory =
                    iSettings->getStringBuffer(kOmniPvdOvdRecordingDirectory, &outputDirectoryLength);
                const char* tcpAddress = iSettings->getStringBuffer(kOmniPvdTcpAddress, &tcpAddressLength);
                OmniPvdDestination destination;
                const char* destinationError = nullptr;
                const bool destinationValid = normalizeOmniPvdDestination(
                    transport, transportLength,
                    outputDirectory, outputDirectoryLength,
                    tcpAddress, tcpAddressLength,
                    iSettings->getAsInt(kOmniPvdTcpPort), iSettings->getAsInt(kOmniPvdTcpTimeoutMs),
                    destination, destinationError);
                if (!destinationValid)
                {
                    CARB_LOG_ERROR("OmniPvd: invalid startup destination: %s", destinationError);
                }
                else
                {
                    mOmniPvd = PxCreateOmniPvd(*mFoundation);
                    OmniPvdWriter* omniWriter = mOmniPvd ? mOmniPvd->getWriter() : nullptr;
                    if (!omniWriter)
                    {
                        CARB_LOG_ERROR("OmniPvd writer not created");
                        SAFE_RELEASE(mOmniPvd);
                    }
                    else if (destination.transport == OmniPvdTransport::eTcp)
                    {
                        OmniPvdSocketWriteStream* socketStream = createOmniPvdSocketWriteStream(
                            destination.tcpAddress.c_str(), destination.tcpPort, destination.tcpTimeoutMs);
                        if (socketStream && socketStream->openStream())
                        {
                            mOmniPvdWriteStream = socketStream;
                            mOmniPvdStreamKind = OmniPvdStreamKind::eTcp;
                            omniWriter->setWriteStream(*socketStream);
                        }
                        else
                        {
                            CARB_LOG_ERROR("OmniPvd: failed to connect TCP startup stream to %s:%u",
                                destination.tcpAddress.c_str(), static_cast<unsigned>(destination.tcpPort));
                            if (socketStream)
                            {
                                socketStream->closeStream();
                                destroyOmniPvdSocketWriteStream(*socketStream);
                            }
                            SAFE_RELEASE(mOmniPvd);
                        }
                    }
                    else
                    {
                        OmniPvdFileWriteStream* fileStream = createOmniPvdFileWriteStream();
                        if (fileStream)
                        {
                            mOmniPvdWriteStream = fileStream;
                            mOmniPvdStreamKind = OmniPvdStreamKind::eStartupFile;
                            omniWriter->setWriteStream(*fileStream);
                        }
                        if (fileStream && !destination.fileTarget.empty())
                        {
                            std::string formattedOutputDir = destination.fileTarget;
                            addLastSlash(formattedOutputDir);
                            bool directoryCreationOk = true;
                            try
                            {
                                std::filesystem::create_directories(formattedOutputDir);
                            }
                            catch (const std::filesystem::filesystem_error& e)
                            {
                                directoryCreationOk = false;
                                CARB_LOG_ERROR("Failed to create output directory: %s", e.what());
                            }
                            if (directoryCreationOk)
                            {
                                OmniPvdRecordingTime nowTs;
                                getLocalOmniPvdTime(nowTs);
                                gLastOmniPvdTime.setWithDiffCounterIncreaseIfSame(nowTs);
                                char buffer[200];
                                sprintf_s(buffer, 200, "%04d_%02d_%02d_%02d_%02d_%02d_%02d",
                                    gLastOmniPvdTime.year, gLastOmniPvdTime.month, gLastOmniPvdTime.day,
                                    gLastOmniPvdTime.hour, gLastOmniPvdTime.minute, gLastOmniPvdTime.second,
                                    gLastOmniPvdTime.diffCounter);
                                ovdRecordingFileName = formattedOutputDir + "tmp.ovd";
                                setOmniPVDoutputDirectory(formattedOutputDir.c_str());
                                setOmniPVDTimeStampedFileName(buffer);
                                fileStream->setFileName(ovdRecordingFileName.c_str());
                            }
                            else
                            {
                                SAFE_RELEASE(mOmniPvd);
                                releaseOmniPvdWriteStream();
                            }
                        }
                        else if (!fileStream)
                            CARB_LOG_ERROR("OmniPvd writeStream not set!");
                        else
                            CARB_LOG_ERROR("OmniPvd: reading output directory failed!");
                        if (!fileStream || destination.fileTarget.empty())
                        {
                            SAFE_RELEASE(mOmniPvd);
                            releaseOmniPvdWriteStream();
                        }
                    }
                }
            }
            const bool omniPVDRecordingCapable = isOmniPvdRecordingCapable(*iSettings);
            if (omniPVDRecordingCapable && !mOmniPvd)
            {
                mOmniPvd = PxCreateOmniPvd(*mFoundation);
                if (!mOmniPvd || !mOmniPvd->getWriter())
                {
                    CARB_LOG_ERROR("OmniPvd writer not created");
                    SAFE_RELEASE(mOmniPvd);
                }
            }

#if USE_PHYSX_GPU
            // GPU context creation is deferred to the first GPU-requesting scene
            // attach (createPhysicsScene) so a CPU-only process never opens a CUDA
            // context at startup. Nothing to do here.
#endif

            mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, tolerances, true, mVisualDebugger, mOmniPvd);
            if (!mPhysics)
            {
                CARB_LOG_ERROR("Unable to create PhysX SDK.");
                releaseVehiclePvdRegistrationHandles();
                SAFE_RELEASE(mOmniPvd);
                releaseOmniPvdWriteStream();
                iSettings->setBool(kOmniPvdIsRecording, false);
                return;
            }

            if (!mExtensionsInitialized)
            {
                PxInitExtensions(*mPhysics, mVisualDebugger);
                mExtensionsInitialized = true;
            }

            if (mOmniPvd)
                registerVehiclePvdCallback();

            if (mOmniPvd && mOmniPvdWriteStream)
            {
                // PxInitExtensions and the Vehicle callback must be registered before the first
                // full-state snapshot so startup recordings include every producer.
                mVehiclePvdSnapshotSucceeded = false;
                OmniPvdWriter* omniWriter = mOmniPvd->getWriter();
                const bool samplingStarted = mOmniPvd->startSampling();
                const bool writerSucceeded =
                    !(omniWriter->getStatus() & OmniPvdWriterStatusFlag::eSTREAM_WRITE_FAILURE);
                if (samplingStarted && mVehiclePvdSnapshotSucceeded && writerSucceeded)
                {
                    isOmniPVDRecording = true;
                }
                else
                {
                    CARB_LOG_ERROR("OmniPvd error starting recording");
                    releaseVehiclePvdTelemetry();
                    if (mOmniPvd->isSampling())
                        mOmniPvd->stopSampling();
                    // A failed startup capture is finalized but not published as
                    // a completed recording. Do not retain it for a later start.
                    releaseOmniPvdWriteStream();
                }
            }
            iSettings->setBool(kOmniPvdIsRecording, isOmniPVDRecording);

            mDefaultCookingParams = getCookingParams(tolerances);
            createCookingAsyncContext();
            IPhysxCookingServicePrivate* cookingServicePrivate = getCookingServicePrivateInterface();
            IPhysxCookingService* cookingService = getCookingServiceInterface();
            if (mCookingServiceContext && cookingServicePrivate && cookingService)
            {
                mCookingDataAsync = cookingdataasync::createCookingDataAsync(
                    *mPhysics, *cookingServicePrivate, *cookingService, mCookingServiceContext);

                // The change feed holds a WEAK reference to whichever cooking driver registered
                // its interest (ADR-0003), and this function releases the previous driver above,
                // so after a mid-attach PxPhysics recreate (createPhysXScene's tolerances check
                // does exactly that) the old registration is inert and cooking would never see
                // another source change -- no recook scheduling, and no mesh-key cache
                // invalidation, leaving a re-authored mesh cooking off its stale CRC. The feed
                // outlives the swap, so re-register the new driver here.
                if (mCookingDataAsync)
                {
                    if (AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage())
                        mCookingDataAsync->registerOnChangeFeed(*attachedStage);
                }
            }
            else
            {
                CARB_LOG_ERROR("Unable to create PhysX asynchronous cooking data.");
            }

            // profiler callback: one callback object feeds both the Carbonite
            // profiler and NVTX, so install it when either sink is asked for.
            nvtx::setEnabled(iSettings->getAsBool(kSettingNvtxEnabled));
            if (iSettings->getAsBool(kSettingExposeProfilerData) || nvtx::isEnabled())
            {
                PxSetProfilerCallback(&gProfilerCallback);
            }
            else
            {
                PxSetProfilerCallback(nullptr);
            }

            disconnectPVD();
            connectPVD();
        }

        void PhysXSetup::createCookingAsyncContext()
        {
            IPhysxCookingService* cookingService = getCookingServiceInterface();
            IPhysxCookingServicePrivate* cookingServicePrivate = getCookingServicePrivateInterface();
            if (mCookingServiceContext || !cookingService || !cookingServicePrivate)
            {
                return;
            }

            omni::physx::PhysxCookingAsyncContextParameters contextParams;
            contextParams.contextName = { "omni.physx", strlen("omni.physx") };
            mCookingServiceContext = cookingService->createAsyncContext(contextParams);
        }

        void PhysXSetup::releaseCookingAsyncContext()
        {
            SAFE_RELEASE(mCookingDataAsync);

            // The service context owns queued cooking tasks and CRC state for this PhysX SDK lifetime.
            if (mCookingServiceContext && mCookingComputeService)
            {
                mCookingComputeService->destroyAsyncContext(mCookingServiceContext);
                mCookingServiceContext = nullptr;
            }
        }

        void PhysXSetup::initializeCookingServiceInterfaces()
        {
            fillCookingServiceInterfaces(mCookingService, mCookingServicePrivate);
        }

        ICookingComputeService* PhysXSetup::getCookingComputeService()
        {
            if (!mCookingComputeService)
            {
                if (mPhysicsReleased)
                {
                    return nullptr;
                }
                if (!mFoundation)
                {
                    CARB_LOG_ERROR("PhysX foundation is not initialized for the cooking compute service.");
                    return nullptr;
                }
                mCookingComputeService = createDefaultCookingComputingService(*mFoundation, acquireOwnedCudaContextManager);
                if (!mCookingComputeService)
                {
                    CARB_LOG_ERROR("Unable to create PhysX cooking compute service.");
                }
            }
            return mCookingComputeService;
        }

        IPhysxCookingService* PhysXSetup::getCookingServiceInterface()
        {
            return getCookingComputeService() ? &mCookingService : nullptr;
        }

        IPhysxCookingServicePrivate* PhysXSetup::getCookingServicePrivateInterface()
        {
            return getCookingComputeService() ? &mCookingServicePrivate : nullptr;
        }

        void PhysXSetup::releaseCookingService()
        {
            releaseCookingAsyncContext();
            if (mCookingComputeService)
            {
                releaseCookingComputingService(mCookingComputeService);
                mCookingComputeService = nullptr;
            }
        }

        void PhysXSetup::setOmniPVDoutputDirectory(const char* directory)
        {
            mOmniPVDOutputDirectory = directory;
        }

        void PhysXSetup::setOmniPVDTimeStampedFileName(const char* fileName)
        {
            mOmniPVDTimeStampedFileName = fileName;
        }


        void PhysXSetup::writeOutOmniPVDFile(bool omniPVDWasActive, bool fileWriteStreamClosedOk)
        {

            carb::settings::ISettings* iSettings = OmniPhysX::getInstance().getISettings();
            if (!iSettings)
            {
                return;
            }

            if (omniPVDWasActive && (OmniPhysX::getInstance().getSimulationStepCount() > 0) && iSettings->getAsBool(kOmniPvdIsRecording))
            {
                // Rename mOmniPVDOutputDirectory + tmp.ovd into outputDirFinal + mOmniPVDTimeStampedFileName.
                std::string outputDirFinal = mOmniPVDOutputDirectory;
                if (addLastSlash(outputDirFinal))
                {
                    setOmniPVDoutputDirectory(outputDirFinal.c_str());
                }

                // The output directory could have been changed by the user since the tmp file was
                // created, so adjust if necessary.
                const char* outputDirectory = iSettings->getStringBuffer(kOmniPvdOvdRecordingDirectory);                
                if ((*outputDirectory) != 0)
                {
                    std::string formattedOutputDir = outputDirectory;
                    addLastSlash(formattedOutputDir);
                    outputDirFinal = formattedOutputDir;
                }

                bool directoryCreationOk = true;
                // Ensure the output directory exists
                try {
                    std::filesystem::create_directories(outputDirFinal);
                }
                catch (const std::filesystem::filesystem_error& e) {
                    directoryCreationOk = false;
                    CARB_LOG_ERROR("Failed to create output directory: %s", e.what());
                }

                if (directoryCreationOk)
                {
                    std::string tmpFilePath = mOmniPVDOutputDirectory + "tmp.ovd";
                    std::string finalFilePath = outputDirFinal + mOmniPVDTimeStampedFileName + "_rec.ovd";
                    rename(tmpFilePath.c_str(), finalFilePath.c_str());

                    if (fileWriteStreamClosedOk)
                    {
                        iSettings->setString("/persistent/physics/omniPvdImportDirectory", outputDirFinal.c_str());
                    }
                    else
                    {
                        // The final flush/close of the capture stream failed, so the renamed
                        // recording may be truncated or incomplete. Keep the file on disk so the
                        // partial capture stays inspectable, but do not publish the import
                        // directory: downstream tooling would otherwise auto-import a corrupt recording.
                        CARB_LOG_ERROR(
                            "OmniPvd: failed to finalize the capture stream; the recording '%s' may be truncated or incomplete. "
                            "Skipping auto-import publication to avoid importing a corrupt recording.",
                            finalFilePath.c_str());
                    }
                }
            }
        }

        OmniPvdRecordingResult PhysXSetup::startOmniPvdRecording(const OmniPvdDestination& destination)
        {
            if (!mPhysics)
            {
                carb::settings::ISettings* settings = OmniPhysX::getInstance().getISettings();
                return isOmniPvdRecordingCapable(*settings) ? OmniPvdRecordingResult::eInvalidState :
                                                              OmniPvdRecordingResult::eNotCapable;
            }
            if (!mOmniPvd || !mOmniPvd->getWriter())
                return OmniPvdRecordingResult::eNotCapable;
            if (mOmniPvd->isSampling())
                return OmniPvdRecordingResult::eInvalidState;

            // A consumer may have stopped PxOmniPvd directly. Retire Vehicle
            // objects against the old writer before setWriteStream() resets the
            // schema and object handles for the new destination.
            releaseVehiclePvdTelemetry();

            OmniPvdWriteStream* stream = nullptr;
            OmniPvdStreamKind streamKind = OmniPvdStreamKind::eNone;
            if (destination.transport == OmniPvdTransport::eTcp)
            {
                OmniPvdSocketWriteStream* socketStream = createOmniPvdSocketWriteStream(
                    destination.tcpAddress.c_str(), destination.tcpPort, destination.tcpTimeoutMs);
                if (!socketStream)
                    return OmniPvdRecordingResult::eError;
                if (!socketStream->openStream())
                {
                    socketStream->closeStream();
                    destroyOmniPvdSocketWriteStream(*socketStream);
                    return OmniPvdRecordingResult::eError;
                }
                stream = socketStream;
                streamKind = OmniPvdStreamKind::eTcp;
            }
            else
            {
                OmniPvdFileWriteStream* fileStream = createOmniPvdFileWriteStream();
                if (!fileStream)
                    return OmniPvdRecordingResult::eError;
                fileStream->setFileName(destination.fileTarget.c_str());
                if (!fileStream->openStream())
                {
                    fileStream->closeStream();
                    destroyOmniPvdFileWriteStream(*fileStream);
                    return OmniPvdRecordingResult::eError;
                }
                stream = fileStream;
                streamKind = OmniPvdStreamKind::eFile;
            }

            OmniPvdWriter* writer = mOmniPvd->getWriter();
            writer->setWriteStream(*stream);
            if (mOmniPvdWriteStream && !releaseOmniPvdWriteStream())
                CARB_LOG_ERROR("OmniPvd: failed to finalize the previous recording stream");
            mOmniPvdWriteStream = stream;
            mOmniPvdStreamKind = streamKind;
            mVehiclePvdSnapshotSucceeded = false;
            const bool samplingStarted = mOmniPvd->startSampling();
            const bool writerSucceeded =
                !(writer->getStatus() & OmniPvdWriterStatusFlag::eSTREAM_WRITE_FAILURE);
            if (!samplingStarted || !mVehiclePvdSnapshotSucceeded || !writerSucceeded)
            {
                releaseVehiclePvdTelemetry();
                if (mOmniPvd->isSampling())
                    mOmniPvd->stopSampling();
                releaseOmniPvdWriteStream();
                OmniPhysX::getInstance().getISettings()->setBool(kOmniPvdIsRecording, false);
                return OmniPvdRecordingResult::eError;
            }
            OmniPhysX::getInstance().getISettings()->setBool(kOmniPvdIsRecording, true);
            return OmniPvdRecordingResult::eSuccess;
        }

        OmniPvdRecordingResult PhysXSetup::stopOmniPvdRecording()
        {
            if (!mOmniPvd || !mOmniPvd->isSampling())
                return OmniPvdRecordingResult::eInvalidState;

            OmniPvdWriter* writer = mOmniPvd->getWriter();
            releaseVehiclePvdTelemetry();
            const bool samplingStopped = mOmniPvd->stopSampling();
            const bool writerSucceeded =
                !(writer->getStatus() & OmniPvdWriterStatusFlag::eSTREAM_WRITE_FAILURE);
            const bool startupFile = mOmniPvdStreamKind == OmniPvdStreamKind::eStartupFile;
            const bool closeStreamOk = releaseOmniPvdWriteStream();
            if (startupFile)
                writeOutOmniPVDFile(true, closeStreamOk);
            OmniPhysX::getInstance().getISettings()->setBool(kOmniPvdIsRecording, false);
            return samplingStopped && writerSucceeded && closeStreamOk ? OmniPvdRecordingResult::eSuccess :
                                                                         OmniPvdRecordingResult::eError;
        }

        bool PhysXSetup::isOmniPvdRecording() const
        {
            return mOmniPvd && mOmniPvd->isSampling();
        }

        bool PhysXSetup::closeOmniPvdWriteStream()
        {
            bool closeStreamOk = true;
            if (mOmniPvdStreamKind == OmniPvdStreamKind::eStartupFile ||
                mOmniPvdStreamKind == OmniPvdStreamKind::eFile)
            {
                OmniPvdFileWriteStream* stream = static_cast<OmniPvdFileWriteStream*>(mOmniPvdWriteStream);
                closeStreamOk = !stream || stream->closeStream();
            }
            else if (mOmniPvdStreamKind == OmniPvdStreamKind::eTcp)
            {
                OmniPvdSocketWriteStream* stream = static_cast<OmniPvdSocketWriteStream*>(mOmniPvdWriteStream);
                closeStreamOk = !stream || stream->closeStream();
            }
            return closeStreamOk;
        }

        bool PhysXSetup::releaseOmniPvdWriteStream()
        {
            // Reports whether the final flush/close of the capture stream succeeded so the
            // caller can decide whether the recording on disk is complete. The PxOmniPvd
            // writer only borrows this stream. Sampling must be stopped, the writer rebound,
            // or the provider released before this function destroys the stream.
            const bool closeStreamOk = closeOmniPvdWriteStream();
            if (mOmniPvdWriteStream)
            {
                if (mOmniPvdStreamKind == OmniPvdStreamKind::eStartupFile ||
                    mOmniPvdStreamKind == OmniPvdStreamKind::eFile)
                {
                    OmniPvdFileWriteStream* stream = static_cast<OmniPvdFileWriteStream*>(mOmniPvdWriteStream);
                    destroyOmniPvdFileWriteStream(*stream);
                }
                else if (mOmniPvdStreamKind == OmniPvdStreamKind::eTcp)
                {
                    OmniPvdSocketWriteStream* stream = static_cast<OmniPvdSocketWriteStream*>(mOmniPvdWriteStream);
                    destroyOmniPvdSocketWriteStream(*stream);
                }
                mOmniPvdWriteStream = nullptr;
                mOmniPvdStreamKind = OmniPvdStreamKind::eNone;
            }
            return closeStreamOk;
        }

        void PhysXSetup::onStartSampling(::physx::PxOmniPvd& omniPvd)
        {
            mVehiclePvdSnapshotSucceeded = false;
            CARB_ASSERT(&omniPvd == mOmniPvd);
            if (&omniPvd != mOmniPvd)
                return;

            ::physx::PxOmniPvd::ScopedExclusiveWriter writerScope(&omniPvd);
            OmniPvdWriter* writer = writerScope.getWriter();
            CARB_ASSERT(!mVehiclePvdRegistrationHandles);
            if (!writer || mVehiclePvdRegistrationHandles)
                return;

            mVehiclePvdRegistrationHandles = ::physx::PxVehiclePvdAttributesCreate(mAllocator, *writer);
            if (!mVehiclePvdRegistrationHandles)
                return;

            bool snapshotSucceeded = true;
            for (PhysXScenesMap::reference ref : mPhysXScenes)
            {
                InternalScene* scene = ref.second->getInternalScene();
                ::physx::PxVehiclePhysXSimulationContext& context = scene->getVehicleContext().getContext();
                context.pvdContext.attributeHandles = mVehiclePvdRegistrationHandles;
                context.pvdContext.writer = writer;

                for (InternalVehicle* vehicle : scene->mVehicles)
                {
                    if (!vehicle->mPhysXVehicle)
                        continue;
                    CARB_ASSERT(!vehicle->mPhysXVehicle->getPvdObjectHandles());
                    if (vehicle->mPhysXVehicle->getPvdObjectHandles())
                    {
                        snapshotSucceeded = false;
                        continue;
                    }

                    vehicle->mPhysXVehicle->createPvdObjectHandles(mAllocator);
                    ::physx::PxVehiclePVDComponent* component =
                        static_cast<::physx::PxVehiclePVDComponent*>(vehicle->mPhysXVehicle);
                    snapshotSucceeded = component->update(0.0f, context) && snapshotSucceeded;
                }
            }
            mVehiclePvdSnapshotSucceeded = snapshotSucceeded &&
                !(writer->getStatus() & OmniPvdWriterStatusFlag::eSTREAM_WRITE_FAILURE);
        }

        void PhysXSetup::registerVehiclePvdCallback()
        {
            if (mOmniPvd)
                mOmniPvd->addEventCallback(*this);
        }

        void PhysXSetup::unregisterVehiclePvdCallback()
        {
            if (mOmniPvd)
                mOmniPvd->removeEventCallback(*this);
        }

        void PhysXSetup::releaseVehiclePvdRegistrationHandles()
        {
            if (mVehiclePvdRegistrationHandles)
            {
                ::physx::PxVehiclePvdAttributesRelease(mAllocator, *mVehiclePvdRegistrationHandles);
                mVehiclePvdRegistrationHandles = nullptr;
            }
        }

        void PhysXSetup::releaseVehiclePvdTelemetry()
        {
            if (!mVehiclePvdRegistrationHandles)
                return;

            ::physx::PxOmniPvd::ScopedExclusiveWriter writerScope(mOmniPvd);
            OmniPvdWriter* writer = writerScope.getWriter();
            CARB_ASSERT(writer);
            if (!writer)
                return;

            for (PhysXScenesMap::reference ref : mPhysXScenes)
            {
                InternalScene* scene = ref.second->getInternalScene();
                for (InternalVehicle* vehicle : scene->mVehicles)
                {
                    if (vehicle->mPhysXVehicle && vehicle->mPhysXVehicle->getPvdObjectHandles())
                        vehicle->mPhysXVehicle->releasePvdObjectHandles(*writer, mAllocator);
                }
                ::physx::PxVehiclePhysXSimulationContext& context = scene->getVehicleContext().getContext();
                context.pvdContext.attributeHandles = nullptr;
                context.pvdContext.writer = nullptr;
            }
            releaseVehiclePvdRegistrationHandles();
        }

        void PhysXSetup::cleanupPhysics()
        {
            releaseCookingAsyncContext();
            getMeshCache()->release();
            SAFE_RELEASE(mCylinderMeshX);
            SAFE_RELEASE(mCylinderMeshY);
            SAFE_RELEASE(mCylinderMeshZ);
            SAFE_RELEASE(mConeMeshX);
            SAFE_RELEASE(mConeMeshY);
            SAFE_RELEASE(mConeMeshZ);

            if (mExtensionsInitialized)
            {
                PxCloseExtensions();
                mExtensionsInitialized = false;
            }

            const bool omniPVDWasActive = mOmniPvd && mOmniPvd->isSampling();
            unregisterVehiclePvdCallback();
            releaseVehiclePvdTelemetry();
            SAFE_RELEASE(mSerializationRegistry)
            SAFE_RELEASE(mPhysics)

            SAFE_RELEASE(mOmniPvd)
            const bool omniPVDFileWasActive = mOmniPvdStreamKind == OmniPvdStreamKind::eStartupFile;
            const bool writeStreamClosedOk = releaseOmniPvdWriteStream();
            writeOutOmniPVDFile(omniPVDWasActive && omniPVDFileWasActive, writeStreamClosedOk);
            if (mISettings)
                mISettings->setBool(kOmniPvdIsRecording, false);
        }

        void PhysXSetup::createPhysics()
        {
            if (mFoundation)
            {
                return;
            }

            mPhysicsReleased = false;
            mErrorCallback->setEventStream(OmniPhysX::getInstance().getErrorEventStream());
            mErrorCallback->setErrorEventType(ePhysxError);

            // A.B. TODO
            // we really should have a common memory manager and pass it to PhysX too
            mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, mAllocator, *mErrorCallback);
            if (!mFoundation)
            {
                CARB_LOG_ERROR("Unable to create PhysX foundation.");
                return;
            }

            createCpuDispatcher(mThreadCount);

            // Initialize the vehicle SDK.
            if (!mVehicleSDKInitialized)
            {
                mVehicleSDKInitialized = ::physx::PxInitVehicleExtension(*mFoundation);
            }
        }

        // final release, should be called when app is closing
        void PhysXSetup::releasePhysics()
        {
            mPhysicsReleased = true;
            omni::sampling::notifyPhysXSceneRelease();
            omni::physx::particles::notifyPhysXRelease();
            releaseCookingService();

            // Wait for all stepper tasks to complete before destroying scenes
            // and releasing the CUDA context, to avoid use-after-free on
            // the CudaContextManager from in-flight stepper fibers.
            for (PhysXScenesMap::reference ref : mPhysXScenes)
            {
                ref.second->waitForCompletion(false);
            }

            // Clear scene contexts and release Vehicle objects while the
            // provider and every scene are still alive and quiescent.
            unregisterVehiclePvdCallback();
            releaseVehiclePvdTelemetry();

            for (PhysXScenesMap::reference ref : mPhysXScenes)
            {
                delete ref.second;
            }
            mPhysXScenes.clear();

            releaseCpuDispatcher();
#if USE_PHYSX_GPU
            mErrorCallback->setCudaContextManager(nullptr); //OM-111223 - crash because we dereference stale pointer if we output an error during shutdown.
            clearCudaContextManagers();
#endif

            if (mVehicleSDKInitialized)
            {
                ::physx::PxCloseVehicleExtension();
                mVehicleSDKInitialized = false;
            }

            if (mExtensionsInitialized)
            {
                PxCloseExtensions();
                mExtensionsInitialized = false;
            }

            const bool omniPVDWasActive = mOmniPvd && mOmniPvd->isSampling();
            SAFE_RELEASE(mSerializationRegistry)
            SAFE_RELEASE(mPhysics)
            SAFE_RELEASE(mVisualDebugger)

            SAFE_RELEASE(mOmniPvd)
            const bool omniPVDFileWasActive = mOmniPvdStreamKind == OmniPvdStreamKind::eStartupFile;
            const bool writeStreamClosedOk = releaseOmniPvdWriteStream();
            SAFE_RELEASE(mFoundation)

            writeOutOmniPVDFile(omniPVDWasActive && omniPVDFileWasActive, writeStreamClosedOk);
            if (mISettings)
                mISettings->setBool(kOmniPvdIsRecording, false);

            mErrorCallback->invalidateEventStream();
        }

        PxPhysics* PhysXSetup::getPhysics()
        {
            // Units come from the attached SOURCE, not from a USD stage. A stageless
            // attach has no stage, and resolving units through one there silently
            // yields the 1.0 default -- while the scene is later built from the
            // source's real metersPerUnit. PhysX then rejects the scene outright
            // ("PxTolerancesScale must be the same as used for creation of
            // PxPhysics"), so on non-metre content a USD-free consumer could not
            // create a scene at all.
            if (const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage())
            {
                if (!mPhysics)
                    createPhysics(getDefaultTolerances(double(attachedStage->getSourceUnits().metersPerUnit)));
                return mPhysics;
            }

            // Nothing is attached, so there is no source to read units from. Create at the
            // default tolerances rather than routing the null through a source that cannot
            // answer.
            // createPhysXScene re-derives them from the scene's real metersPerUnit while
            // no scene exists yet.
            if (!mPhysics)
                createPhysics(getDefaultTolerances(1.0));
            return mPhysics;
        }

        ::physx::PxCudaContextManager* PhysXSetup::getCudaContextManager(size_t id) const
        {
            if (id >= mCudaContextManagers.size())
                return nullptr;
            return mCudaContextManagers[id];
        }

        /**
         * @implements REQ-COOK-CUDACTX-001
         * @covers AC-1
         */
        ::physx::PxCudaContextManager* PhysXSetup::acquireCudaContextManager(size_t id)
        {
            std::lock_guard<std::mutex> lock(mCudaContextManagerMutex);
            if (id >= mCudaContextManagers.size())
                return nullptr;
            ::physx::PxCudaContextManager* cudaContextManager = mCudaContextManagers[id];
            if (cudaContextManager)
            {
                // Taken under the same lock that guards every release below, so the manager cannot
                // be destroyed between the read above and this increment.
                cudaContextManager->acquireReference();
            }
            return cudaContextManager;
        }

        ::physx::PxCudaContextManager* PhysXSetup::getNextCudaContextManager()
        {
            if (mCudaContextManagers.empty())
                return nullptr;
            // OMPE-102199: reduce the cursor itself, not just its successor. If the vector ever shrinks while the
            // cursor sits past the new end, indexing it raw reads heap bytes from beyond the vector and returns a
            // non-null garbage pointer. That passes the null check in createPhysXScene(), reaches
            // PxSceneDesc::cudaContextManager, and is virtual-called as a PxCudaContextManager.
            const size_t id = mNextCudaContextManagerId % mCudaContextManagers.size();
            mNextCudaContextManagerId = (id + 1) % mCudaContextManagers.size();
            return mCudaContextManagers[id];
        }

        void PhysXSetup::connectPVD()
        {
            static bool wasPVDFileOutput = false;
            if (mVisualDebugger)
            {
                carb::settings::ISettings* iSettings = OmniPhysX::getInstance().getISettings();

                if (iSettings->getAsBool(kSettingPVDStreamToFile))
                {
                    std::string outputFilePath = iSettings->getStringBuffer(kSettingPVDOutputDirectory);
                    addLastSlash(outputFilePath);
                    outputFilePath += "tmp.pvd";
                    mPvdTransport = PxDefaultPvdFileTransportCreate(outputFilePath.c_str());
                }
                else
                    mPvdTransport = PxDefaultPvdSocketTransportCreate(iSettings->getStringBuffer(kSettingPVDIPAddress), 5425, 10);

                PxPvdInstrumentationFlags instrumentationFlags(0);
                if (iSettings->getAsBool(kSettingPVDProfile))
                    instrumentationFlags |= PxPvdInstrumentationFlag::ePROFILE;
                if (iSettings->getAsBool(kSettingPVDMemory))
                    instrumentationFlags |= PxPvdInstrumentationFlag::eMEMORY;
                if (iSettings->getAsBool(kSettingPVDDebug))
                    instrumentationFlags |= PxPvdInstrumentationFlag::eDEBUG;

                wasPVDFileOutput = false;

                if (instrumentationFlags && mPvdTransport != nullptr)
                {
                    bool success = mVisualDebugger->connect(*mPvdTransport, instrumentationFlags);
                    if (!success)
                    {
                        CARB_LOG_WARN("PVD was enabled but the connection failed");
                        if (iSettings->getAsBool(kSettingPVDStreamToFile))
                        {
                            CARB_LOG_WARN("Stream to file is enabled for PVD - maybe double check output file location/access privileges?");
                        }
                    }
                    else if (iSettings->getAsBool(kSettingPVDStreamToFile))
                    {
                        wasPVDFileOutput = true;
                    }
                }
            }
            else
            {
                wasPVDFileOutput = false;
            }
        }

        void PhysXSetup::disconnectPVD()
        {
            if (mVisualDebugger)
            {
                // rename the temporary file to the final destination, if PVD was outputting to a file and there was any simulation done
                mVisualDebugger->disconnect();
                if (mPvdTransport)
                {
                    mPvdTransport->release();
                    mPvdTransport = nullptr;
                    if (OmniPhysX::getInstance().getSimulationStepCount() > 0)
                    {
                        carb::settings::ISettings* iSettings = OmniPhysX::getInstance().getISettings();
                        if (iSettings->getAsBool(kSettingPVDStreamToFile))
                        {
                            std::string outputFileDir = iSettings->getStringBuffer(kSettingPVDOutputDirectory);
                            if (!outputFileDir.empty())
                            {
                                OmniPvdRecordingTime nowTs;
                                getLocalOmniPvdTime(nowTs);
                                gLastOmniPvdTime.setWithDiffCounterIncreaseIfSame(nowTs);
                                addLastSlash(outputFileDir);

                                char buffer[200];
                                sprintf_s(buffer, 200, "%04d_%02d_%02d_%02d_%02d_%02d_%02d",
                                    gLastOmniPvdTime.year, gLastOmniPvdTime.month, gLastOmniPvdTime.day, gLastOmniPvdTime.hour, gLastOmniPvdTime.minute, gLastOmniPvdTime.second, gLastOmniPvdTime.diffCounter);
                                std::string outputFileName = outputFileDir + "rec_" + buffer + ".pxd2";

                                std::string tmpFilepath    = outputFileDir + "tmp.pvd";

                                rename(tmpFilepath.c_str(), outputFileName.c_str());
                            }
                        }
                    }
                }
            }
        }

        ::physx::PxCookingParams PhysXSetup::getCookingParams(const ::physx::PxTolerancesScale& tolerances)
        {
            PxCookingParams params(tolerances);
            params.buildGPUData = false;
            params.buildTriangleAdjacencies = true;
            return params;
        }

        ::physx::PxTolerancesScale    PhysXSetup::getDefaultTolerances(double metersPerUnit)
        {
            return ::physx::PxTolerancesScale(float(1.0 / metersPerUnit), float(10.0 / metersPerUnit));
        }

        void PhysXSetup::createCpuDispatcher(uint32_t numThreads)
        {
            releaseCpuDispatcher();
            if (!mPhysXDispatcher)
                mDispatcher = new PhysXCarbCpuDispatcher(mThreadCount);
            else
            {
                PxDefaultCpuDispatcher* physxDispatcher = PxDefaultCpuDispatcherCreate(mThreadCount);
                physxDispatcher->setRunProfiled(true);
                mDispatcher = physxDispatcher;
            }
        }

        void PhysXSetup::releaseCpuDispatcher()
        {
            if (!mPhysXDispatcher)
            {
                if (mDispatcher)
                {
                    ((PhysXCarbCpuDispatcher*)mDispatcher)->release();
                    delete mDispatcher;
                    mDispatcher = nullptr;
                }
            }
            else
            {
                if (mDispatcher)
                {
                    ((PxDefaultCpuDispatcher*)mDispatcher)->release();
                    mDispatcher = nullptr;
                }
            }
        }

        void PhysXSetup::releasePhysXScenes()
        {
            mFilteredPairs.clear();
            mCollisionsGroupFilteredPairs.clear();
            // Re-arm the sticky deferral logs; live references are gone.
            mCudaSettingsDeferralWarned = false;
            mCudaDeviceCountDeferralWarned = false;

            disconnectPVD();

            omni::sampling::notifyPhysXSceneRelease();
            
            // we first make sure that we wait for completion of all scenes
            for (PhysXScenesMap::reference ref : mPhysXScenes)
            {
                ref.second->waitForCompletion(false);
            }
            // then we move them as in the InternalScene destructor we will
            // be calling waitForCompletion that is looping all scenes, potentially
            // accessing some of them that will be already freed
            PhysXScenesMap physxScenes = std::move(mPhysXScenes);
            {
                // OMPE-102199: the move above empties mPhysXScenes, but the scenes below are still alive and
                // still cache the PxCudaContextManager they were created with. setupGPU() derives hasLiveScenes
                // from that map, so without this flag a setupGPU() reaching us from a scene destructor (carb
                // event dispatch re-entering physXResume) would see "no live scenes" and be free to release and
                // recreate the manager underneath them.
                mReleasingPhysXScenes = true;
                for (PhysXScenesMap::reference ref : physxScenes)
                {
                    delete ref.second;
                }
                mReleasingPhysXScenes = false;
            }
            SAFE_RELEASE(mVehicleWheelCylinderMeshX);
            SAFE_RELEASE(mVehicleWheelCylinderMeshY);
            SAFE_RELEASE(mVehicleWheelCylinderMeshZ);

            cleanupPhysics();

            mDefaultScene = nullptr;
        }

        PhysXScene* PhysXSetup::createPhysXScene(const usdparser::AttachedStage& attachedStage, size_t sceneId, double metersPerUnit, double kilogramsPerUnit, const usdparser::PhysxSceneDesc& sceneDesc)
        {
            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            carb::settings::ISettings* iSettings = omniPhysX.getISettings();
            PxPhysics* physics = getPhysics();

            if (mPhysXScenes.empty())
            {
                const uint32_t numThreads = iSettings->isAccessibleAs(carb::dictionary::ItemType::eInt, (kSettingNumThreads)) ?
                    iSettings->getAsInt(kSettingNumThreads) :
                    mThreadCount;

                const bool newDispatcher = iSettings->isAccessibleAs(carb::dictionary::ItemType::eBool, (kSettingPhysxDispatcher)) ? iSettings->getAsBool(kSettingPhysxDispatcher) : false;
                if ((mThreadCount != numThreads || newDispatcher != mPhysXDispatcher) && numThreads < 256)
                {
                    releaseCpuDispatcher();
                    mPhysXDispatcher = newDispatcher;
                    mThreadCount = numThreads;
                    createCpuDispatcher(numThreads);
                }

                omniPhysX.setGpuPipelineOverride(iSettings->isAccessibleAs(carb::dictionary::ItemType::eInt, (kSettingOverrideGPU)) ?
                    iSettings->getAsInt(kSettingOverrideGPU) :
                    omniPhysX.getGpuPipelineOverride());
            }

            // tolerances check, they could have changed
            PxTolerancesScale physicsTolerances = physics->getTolerancesScale();
            PxTolerancesScale sceneTol = getDefaultTolerances(metersPerUnit);
            if (fabsf(physicsTolerances.length - sceneTol.length) > 0.001f)
            {
                // tolerances changed recreate physics
                if (mPhysXScenes.empty())
                {
                    cleanupPhysics();
                    getPhysics();
                }
                else
                {
                    CARB_LOG_ERROR("Cant change metersPerUnit for existing scenes.");
                    metersPerUnit = 1.0f / physicsTolerances.length;
                }
            }

            PhysXScene* scene = PhysXScene::createPhysXScene(attachedStage, sceneId, *this, metersPerUnit, kilogramsPerUnit, sceneDesc);
            CARB_ASSERT(scene);
            if (scene)
            {
                if (mPhysXScenes.empty())
                {
                    mDefaultScene = scene;
                }
                mPhysXScenes[sceneId] = scene;

                getCylinderConvexMesh(omni::physx::usdparser::eX);
                getCylinderConvexMesh(omni::physx::usdparser::eY);
                getCylinderConvexMesh(omni::physx::usdparser::eZ);

                getConeConvexMesh(omni::physx::usdparser::eX);
                getConeConvexMesh(omni::physx::usdparser::eY);
                getConeConvexMesh(omni::physx::usdparser::eZ);

                getVehicleWheelCylinderConvexMesh(omni::physx::usdparser::eX);
                getVehicleWheelCylinderConvexMesh(omni::physx::usdparser::eY);
                getVehicleWheelCylinderConvexMesh(omni::physx::usdparser::eZ);
            }

            return scene;
        }


        ::physx::PxConvexMesh* PhysXSetup::getConeConvexMesh(omni::physx::usdparser::Axis axis) const
        {
            if (axis == eX)
            {
                if (!mConeMeshX)
                {
                    mConeMeshX = createConeConvexMesh(tConeOrCylinderWidth, tConeOrCylinderRadius, tConeNumCirclePoints, eX);
                }
                return mConeMeshX;
            }
            else if (axis == eY)
            {
                if (!mConeMeshY)
                {
                    mConeMeshY = createConeConvexMesh(tConeOrCylinderWidth, tConeOrCylinderRadius, tConeNumCirclePoints, eY);
                }
                return mConeMeshY;
            }
            else
            {
                if (!mConeMeshZ)
                {
                    mConeMeshZ = createConeConvexMesh(tConeOrCylinderWidth, tConeOrCylinderRadius, tConeNumCirclePoints, eZ);
                }
                return mConeMeshZ;
            }
        }

        ::physx::PxConvexMesh* PhysXSetup::getCylinderConvexMesh(Axis axis) const
        {
            if (axis == eX)
            {
                if (!mCylinderMeshX)
                {
                    mCylinderMeshX = createCylinderConvexMesh(tConeOrCylinderWidth, tConeOrCylinderRadius, tCylinderNumCirclePoints, eX);
                }
                return mCylinderMeshX;
            }
            else if (axis == eY)
            {
                if (!mCylinderMeshY)
                {
                    mCylinderMeshY = createCylinderConvexMesh(tConeOrCylinderWidth, tConeOrCylinderRadius, tCylinderNumCirclePoints, eY);
                }
                return mCylinderMeshY;
            }
            else
            {
                if (!mCylinderMeshZ)
                {
                    mCylinderMeshZ = createCylinderConvexMesh(tConeOrCylinderWidth, tConeOrCylinderRadius, tCylinderNumCirclePoints, eZ);
                }
                return mCylinderMeshZ;
            }
        }

        ::physx::PxConvexMesh* PhysXSetup::getVehicleWheelCylinderConvexMesh(Axis axis) const
        {
            constexpr float cylinderWidth = 2.0f;
            constexpr float cylinderRadius = 1.0f;
            constexpr uint32_t cylinderNumCirclePoints = tCylinderNumCirclePoints;

            if (axis == eX)
            {
                if (!mVehicleWheelCylinderMeshX)
                {
                    mVehicleWheelCylinderMeshX = createCylinderConvexMesh(cylinderWidth, cylinderRadius, cylinderNumCirclePoints, eX);
                }
                return mVehicleWheelCylinderMeshX;
            }
            else if (axis == eY)
            {
                if (!mVehicleWheelCylinderMeshY)
                {
                    mVehicleWheelCylinderMeshY = createCylinderConvexMesh(cylinderWidth, cylinderRadius, cylinderNumCirclePoints, eY);
                }
                return mVehicleWheelCylinderMeshY;
            }
            else
            {
                if (!mVehicleWheelCylinderMeshZ)
                {
                    mVehicleWheelCylinderMeshZ = createCylinderConvexMesh(cylinderWidth, cylinderRadius, cylinderNumCirclePoints, eZ);
                }
                return mVehicleWheelCylinderMeshZ;
            }
        }
    } // namespace physx
} // namespace omni
