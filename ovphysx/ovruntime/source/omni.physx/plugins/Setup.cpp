// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-FEED-006
 * @covers AC-6
 */
// @implements REQ-OMNIPVD-CONTROL-001
// @covers AC-2 AC-3 AC-4 AC-5

// @implements REQ-OMNIPVD-CAPTURE-001
// @covers AC-1 AC-3 AC-4 AC-5

// @implements REQ-COOK-CUDACTX-001
// @covers AC-1

// @implements REQ-ATTACH-RESOLVE-001
// @covers AC-4 AC-6 AC-8

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
 * @implements REQ-SDK-INIT-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-SDK-LIFECYCLE-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-SDK-LIFECYCLE-002
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-OMNIPVD-TRANSPORT-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-OMNIPVD-LATE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10
 *
 * @implements REQ-SIM-PROFILER-001
 * @covers AC-1 AC-2 AC-3 AC-5 AC-6
 *
 * @implements REQ-COOK-LIFETIME-001
 * @covers AC-5
 */

/**
 * @implements REQ-ATTACH-SERVICES-001
 * @covers AC-1 AC-2 AC-3
 * @implements REQ-ATTACH-EXECUTION-001
 * @covers AC-4
 */

// @implements REQ-CUSTOM-LIFETIME-001
// @covers AC-2 AC-3

// @implements REQ-RUNTIME-ERROR-001
// @covers AC-3

#include <carb/Defines.h>
#include <omni/physx/RuntimeError.h>
#include <cudamanager/PxCudaContextManager.h>
#include "PhysXTools.h"
#include "Setup.h"
#include "RuntimeExecution.h"
#include "utils/OmniPvdCaptureObjects.h"
#include "PhysXCustomJoint.h"
#include "internal/InternalActor.h"
#include "internal/InternalDeformable.h"
#include <cmath>
#include <common/utilities/PhysXErrorCallback.h>
#include <common/utilities/MemoryMacros.h>
#include <algorithm>
#include <cstring>
#include <string>
#include <unordered_set>
#include <filesystem>

#include "omnipvd/PxOmniPvd.h"
#include "OmniPvdLibraryFunctions.h"
#include "OmniPvdWriter.h"
#include "OmniPvdDirectoryWriteStream.h"
#include "OmniPvdSocketWriteStream.h"

#include "PhysXScene.h"
#include "PhysXDefines.h"
#include "PhysXUSDProperties.h"
#include "OmniPhysX.h"
#include "PhysXCustomGeometry.h"
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

#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>

#if CARB_PLATFORM_WINDOWS
// SYSTEMTIME and GetLocalTime, for the PVD recording timestamp below. NOMINMAX
// first: <Windows.h> defines min/max as macros otherwise, which breaks the
// std::numeric_limits uses further down.
#    ifndef NOMINMAX
#        define NOMINMAX
#    endif
#    include <Windows.h>
#endif

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

void omni::physx::omniPvdAppendCaptureObjects(
    PhysXType type, void* pointer, std::vector<const ::physx::PxBase*>& objects)
{
    const ::physx::PxBase* object = omniPvdCaptureObject(type, pointer);
    if (object)
        objects.push_back(object);
}

RuntimeResult omni::physx::omniPvdResolveCaptureObjects(
    omni::physics::AttachHandle attach, const std::vector<std::string>& paths,
    std::vector<const ::physx::PxBase*>& objects)
{
    RuntimeExecutionScope operation(attach);
    if (!operation)
        return RuntimeResult::eBusy;
    const std::shared_ptr<AttachedStage> attachment = UsdLoad::getUsdLoad()->acquireAttachment(attach);
    if (!attachment)
        return RuntimeResult::eInvalidAttachment;
    InternalPhysXDatabase& database = attachment->getInternalPhysXDatabase();
    using Record = InternalDatabase::Record;
    using ObjectKey = omni::physics::parse::ObjectKey;
    std::vector<ObjectKey> keys;
    std::unordered_map<ObjectKey, std::vector<const Record*>, ObjectKey::Hash> recordsByKey;
    recordsByKey.reserve(paths.size());
    for (const std::string& path : paths)
    {
        // Paths were validated by the C caller. Resolve only this attachment.
        const ObjectKey key = attachment->keyFor(std::string_view(path));
        keys.push_back(key);
        recordsByKey.emplace(key, std::vector<const Record*>{});
    }
    // Index only requested keys, once for the entire descriptor. Never scan
    // the complete owner database separately for every requested path.
    for (const Record& record : database.getRecords())
    {
        if (!record.mPtr)
            continue;
        const auto found = recordsByKey.find(record.mKey);
        if (found != recordsByKey.end())
            found->second.push_back(&record);
    }
    std::unordered_set<const ::physx::PxBase*> uniqueObjects;
    for (const ObjectKey key : keys)
    {
        std::vector<const ::physx::PxBase*> pathObjects;
        const auto appendRecord = [&pathObjects](const Record& record) {
            omniPvdAppendCaptureObjects(record.mType, record.mPtr, pathObjects);
            if (record.mType == ePTActor && record.mInternalPtr)
            {
                const InternalActor& actor = *static_cast<const InternalActor*>(record.mInternalPtr);
                for (const MirrorActor& mirror : actor.mMirrors)
                    omniPvdAppendCaptureObjects(ePTActor, mirror.actor, pathObjects);
            }
        };
        const ObjectIdMap* entries = attachment->getObjectIds(key);
        if (entries)
        {
            for (ObjectIdMap::const_reference entry : *entries)
            {
                for (const PhysXType type : kOmniPvdCaptureObjectTypes)
                {
                    const Record* record = database.getFullTypedRecord(type, entry.second);
                    if (record)
                        appendRecord(*record);
                }
            }
        }
        for (const Record* record : recordsByKey.at(key))
            appendRecord(*record);
        if (pathObjects.empty())
            return RuntimeResult::eInvalidArgument;
        for (const ::physx::PxBase* object : pathObjects)
            if (uniqueObjects.insert(object).second)
                objects.push_back(object);
    }
    return RuntimeResult::eSuccess;
}

namespace
{

// Keep runtime admission out of the shared error utility. Both ordinary errors
// and abort/too-many-errors push subscribers execute inside this task-aware scope.
class RuntimePhysXErrorCallback final : public CarbPhysXErrorCallback
{
public:
    void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line) override
    {
        RuntimeCallbackScope callbackScope;
        CarbPhysXErrorCallback::reportError(code, message, file, line);
    }
};

bool isOmniPvdRecordingCapable(carb::settings::ISettings& settings)
{
    return settings.getAsBool(kOmniPvdRecordingCapable);
}

omni::physx::ICookingComputeService* getOwnedCookingService()
{
    return omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingComputeService();
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


// Bridges the PhysX SDK profile zones to carb::profiler, which is the only sink:
// whichever backend the process bound decides where they land.
//
// The SDK's cross-thread ("detached") zones are dropped rather than published.
// carb::profiler cannot express them -- its zones are a per-thread begin/end pair,
// and a detached zone may close on a different thread than it opened on, with
// sibling phases that do not close in LIFO order. Publishing them as ordinary zones
// would begin on one thread and end on another, corrupting that thread's stack, so
// the branch below discards them and must stay: deleting it would let them fall
// through to the sink they cannot use.
//
// The zone id rides back to zoneEnd in profilerData, which the SDK round-trips for
// non-detached zones (PxProfileScoped). Handing it to CARB_PROFILE_END is what lets
// Carbonite verify that the zone being closed is the one that was opened, and it
// makes an end whose begin never ran a no-op: with no profiler bound the begin
// yields kNoZoneId and the end skips itself, so a backend that binds midway through
// a zone cannot produce an unmatched close.
class OmniPhysxProfileCallback : public PxProfilerCallback
{
    virtual void* zoneStart(const char* eventName, bool detached, uint64_t /*contextId*/)
    {
        if (detached)
        {
            return nullptr;
        }

        const carb::profiler::ZoneId zone = CARB_PROFILE_BEGIN(kPhysicsProfilerMask, "%s",eventName);
        return reinterpret_cast<void*>(zone);
    }

    virtual void zoneEnd(void* profilerData, const char* /*eventName*/, bool detached, uint64_t /*contextId*/)
    {
        if (detached)
        {
            return;
        }

        CARB_PROFILE_END(kPhysicsProfilerMask, reinterpret_cast<carb::profiler::ZoneId>(profilerData));
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

// Reads the sink gate and installs (or clears) the SDK profiler callback to match.
//
// Called when the PhysX SDK is created and again whenever a host asks for the zones,
// which can happen later: the SDK is created once per process, so a decision taken
// only at creation goes stale the moment a later instance turns profiling on, and an
// uninstalled callback drops every SDK zone.
void installProfilerCallback(carb::settings::ISettings* iSettings)
{
    if (!iSettings)
    {
        return;
    }

    // Idempotent: PxSetProfilerCallback replaces whatever was installed, so turning
    // the sink back off also uninstalls.
    if (iSettings->getAsBool(kSettingExposeProfilerData))
    {
        PxSetProfilerCallback(&gProfilerCallback);
    }
    else
    {
        PxSetProfilerCallback(nullptr);
    }
}


struct PvdRecordingTime
{
    PvdRecordingTime()
    {
        mWasSet = false;
    }

    bool wasSet()
    {
        return mWasSet;
    }

    bool isSame(PvdRecordingTime& ts)
    {
        return ((year == ts.year)
            && (month == ts.month)
            && (day == ts.day)
            && (hour == ts.hour)
            && (minute == ts.minute)
            && (second == ts.second));
    }

    void setWithDiffCounterIncreaseIfSame(PvdRecordingTime& ts)
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

void getLocalPvdTime(PvdRecordingTime& timeStamp)
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

void getLocalPvdTime(PvdRecordingTime& timeStamp)
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

static PvdRecordingTime gLastPvdTime;

//void logFunc(char *logLine)
//{
//    std::cout << logLine << std::endl;
//}

namespace omni
{
    namespace physx
    {
        PhysXSetup::PhysXSetup()
        {
            mPhysxFoundation = &omni::physx::foundation::getInterface();
            initializeCookingServiceInterfaces();

            mErrorCallback = new RuntimePhysXErrorCallback();

            // The settings subscriptions belong to the started runtime, not to this object's
            // lifetime: see subscribeToSettingsChangeEvents().
            mISettings = carb::getCachedInterface<carb::settings::ISettings>();
        }

        PhysXSetup::~PhysXSetup()
        {
            releaseCookingService();
            delete mErrorCallback;
            mErrorCallback = nullptr;

            unsubscribeFromSettingsChangeEvents();
            mISettings = nullptr;
        }

        void PhysXSetup::subscribeToSettingsChangeEvents()
        {
            if (!mSubscribedSettings.empty())
            {
                return;
            }
            if (!mISettings)
            {
                mISettings = carb::getCachedInterface<carb::settings::ISettings>();
            }
            CARB_ASSERT(mISettings);
            if (!mISettings)
            {
                return;
            }

            // Regenerate the legacy PVD connection when its setting changes.
            auto legacyPvdEnabledChangedLambda = [](const carb::dictionary::Item* changedItem,
                                                    carb::dictionary::ChangeEventType eventType, void* userData) {
                if (!OmniPhysX::isStarted()) return;
                OmniPhysX& omniPhysX = OmniPhysX::getInstance();
                PhysXSetup *ptr = static_cast<PhysXSetup*>(userData);
                bool setting = omniPhysX.getISettings()->getAsBool(kSettingPVDEnabled);
                if (setting != (bool)ptr->getPvd())
                {
                    ptr->changePVDSettings(setting);
                }
            };
            carb::dictionary::SubscriptionId* subID =
                mISettings->subscribeToNodeChangeEvents(kSettingPVDEnabled, legacyPvdEnabledChangedLambda, this);
            mSubscribedSettings.push_back(subID);
        }

        void PhysXSetup::unsubscribeFromSettingsChangeEvents()
        {
            if (mISettings)
            {
                for (carb::dictionary::SubscriptionId* subId : mSubscribedSettings)
                {
                    mISettings->unsubscribeToChangeEvents(subId);
                }
            }
            mSubscribedSettings.clear();
        }

        void PhysXSetup::changePVDSettings(bool enableLegacyPVD)
        {
            if (enableLegacyPVD == (bool)mVisualDebugger)
            {
                return; // do nothing, we're already set with the correct pvd options
            }
            // PxScene instances are owned by the current PxPhysics instance even when their steppers are complete.
            // Recreating PxPhysics while a scene is alive leaves PhysXScene::mScene dangling.
            if (hasResourceBorrowers())
            {
                CARB_LOG_WARN(
                    "PVD settings change deferred until next attach; shared SDK borrowers are still alive");
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

        void PhysXSetup::createPhysics(const PxTolerancesScale& tolerances)
        {
            if (mPhysics && hasResourceBorrowers())
                return;
            if (!tolerances.isValid() || !std::isfinite(tolerances.length) ||
                !std::isfinite(tolerances.speed) || tolerances.speed <= 0.0f)
            {
                CARB_LOG_ERROR("PhysX SDK initialization failed: invalid tolerance scale.");
                return;
            }
            releaseCookingAsyncContext();

            // ### DEFENSIVE OMPE-102186
            // createPhysics() returns without creating a foundation if PxCreateFoundation fails,
            // and CARB_ASSERT compiles out in release, so everything below - PxCreatePvd,
            // PxCreatePhysics - would dereference a null foundation. Bail out with an
            // error instead; the underlying failure is already reported by createPhysics().
            CARB_ASSERT(mFoundation);
            if (!mFoundation)
            {
                CARB_LOG_ERROR("Cannot create PhysX: foundation was not created.");
                return;
            }

            unregisterVehiclePvdCallback();
            releaseVehiclePvdTelemetry();
            SAFE_RELEASE(mPhysics);
            mAttachmentCudaResources.clear();
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
            releaseOmniPvdWriteStream();

            // Recording capability installs dormant producers. Only an explicit
            // start binds a destination and begins sampling.
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
                CARB_LOG_ERROR("PhysX SDK creation failed (length=%g, speed=%g). See the preceding PhysX errors.",
                    tolerances.length, tolerances.speed);
                cleanupPhysics();
                return;
            }

            mPhysics->registerDeletionListener(OmniPhysX::getInstance().getCustomGeometryManager(),
                PxDeletionEventFlag::eMEMORY_RELEASE, false);

            if (!mExtensionsInitialized)
            {
                if (!PxInitExtensions(*mPhysics, mVisualDebugger))
                {
                    CARB_LOG_ERROR("PhysX extension initialization failed.");
                    cleanupPhysics();
                    return;
                }
                mExtensionsInitialized = true;
            }

            if (mOmniPvd)
                registerVehiclePvdCallback();

            iSettings->setBool(kOmniPvdIsRecording, false);

            mDefaultCookingParams = getCookingParams(tolerances);
            createCookingAsyncContext();
            IPhysxCookingServicePrivate* cookingServicePrivate = getCookingServicePrivateInterface();
            IPhysxCookingService* cookingService = getCookingServiceInterface();
            if (mCookingServiceContext && cookingServicePrivate && cookingService)
            {
                mCookingDataAsync = cookingdataasync::createCookingDataAsync(
                    *mPhysics, *cookingServicePrivate, *cookingService, mCookingServiceContext);


            }
            else
            {
                CARB_LOG_ERROR("Unable to create PhysX asynchronous cooking data.");
            }

            // Profiler callback, installed when the sink is asked for. A host that
            // turns it on later calls refreshProfilerCallback(), which repeats
            // this step.
            installProfilerCallback(iSettings);

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
                mCookingComputeService = createDefaultCookingComputingService(*mFoundation);
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

        OmniPvdRecordingResult PhysXSetup::startOmniPvdRecording(
            const OmniPvdDestination& destination,
            const ::physx::PxOmniPvdFilterDesc* capture, AttachHandle selectionOwner, bool discovery)
        {
            if (!mPhysics || UsdLoad::getUsdLoad()->getAttachedStages().empty())
            {
                carb::settings::ISettings* settings = OmniPhysX::getInstance().getISettings();
                return isOmniPvdRecordingCapable(*settings) ? OmniPvdRecordingResult::eInvalidState :
                                                              OmniPvdRecordingResult::eNotCapable;
            }
            if (!mOmniPvd || !mOmniPvd->getWriter())
                return OmniPvdRecordingResult::eNotCapable;
            if (isOmniPvdRecording())
                return OmniPvdRecordingResult::eInvalidState;
            if (discovery && (destination.transport != OmniPvdTransport::eTcp ||
                              selectionOwner == omni::physics::kNoAttach))
                return OmniPvdRecordingResult::eInvalidState;
            if (capture && !mOmniPvd->validateFilterDesc(*capture))
                return OmniPvdRecordingResult::eError;

            ::physx::PxOmniPvdFilterDesc discoveryScope = capture ? *capture : ::physx::PxOmniPvdFilterDesc();
            std::vector<const ::physx::PxScene*> discoveryScenes;
            if (discovery && (!restrictOmniPvdCaptureToOwner(discoveryScope, selectionOwner, discoveryScenes) ||
                              !mOmniPvd->validateFilterDesc(discoveryScope)))
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
                if (destination.fileTarget.empty())
                    return OmniPvdRecordingResult::eError;
                try
                {
                    const std::filesystem::path target = std::filesystem::u8path(destination.fileTarget);
                    const bool namedFile = destination.fileTarget.size() >= 4 &&
                        destination.fileTarget.compare(destination.fileTarget.size() - 4, 4, ".ovd") == 0;
                    const std::filesystem::path directory = namedFile ? target.parent_path() : target;
                    if (!directory.empty())
                        std::filesystem::create_directories(directory);

                    std::unique_ptr<OmniPvdDirectoryWriteStream> fileStream;
                    if (namedFile)
                    {
                        fileStream = std::make_unique<OmniPvdDirectoryWriteStream>(target);
                    }
                    else
                    {
                        PvdRecordingTime timestamp;
                        getLocalPvdTime(timestamp);
                        char prefix[32];
                        std::snprintf(prefix, sizeof(prefix), "%04u_%02u_%02u_%02u_%02u_%02u",
                                      timestamp.year, timestamp.month, timestamp.day, timestamp.hour,
                                      timestamp.minute, timestamp.second);
                        fileStream = std::make_unique<OmniPvdDirectoryWriteStream>(directory, prefix);
                    }
                    if (!fileStream->openStream())
                    {
                        const std::string path = fileStream->getFilePath().u8string();
                        CARB_LOG_ERROR("OmniPvd: failed to open recording '%s': %s (errno %d)",
                                       path.c_str(), std::strerror(fileStream->getOpenError()),
                                       fileStream->getOpenError());
                        return OmniPvdRecordingResult::eError;
                    }
                    stream = fileStream.release();
                    streamKind = OmniPvdStreamKind::eRuntimeFile;
                }
                catch (const std::exception& error)
                {
                    CARB_LOG_ERROR("OmniPvd: failed to prepare recording path '%s': %s",
                                   destination.fileTarget.c_str(), error.what());
                    return OmniPvdRecordingResult::eError;
                }
            }

            OmniPvdWriter* writer = mOmniPvd->getWriter();
            writer->setWriteStream(*stream);
            if (mOmniPvdWriteStream && !releaseOmniPvdWriteStream())
                CARB_LOG_ERROR("OmniPvd: failed to finalize the previous recording stream");
            mOmniPvdWriteStream = stream;
            mOmniPvdStreamKind = streamKind;
            mVehiclePvdSnapshotSucceeded = false;
            const bool samplingStarted = discovery ? mOmniPvd->startCaptureDiscovery(&discoveryScope) :
                                                     mOmniPvd->startSampling(capture);
            const bool writerSucceeded =
                !(writer->getStatus() & OmniPvdWriterStatusFlag::eSTREAM_WRITE_FAILURE);
            if (!samplingStarted || (!discovery && !mVehiclePvdSnapshotSucceeded) || !writerSucceeded)
            {
                releaseVehiclePvdTelemetry();
                if (isOmniPvdRecording())
                    mOmniPvd->stopSampling();
                if (mOmniPvdStreamKind == OmniPvdStreamKind::eRuntimeFile)
                {
                    OmniPvdDirectoryWriteStream* fileStream =
                        static_cast<OmniPvdDirectoryWriteStream*>(mOmniPvdWriteStream);
                    const std::string path = fileStream->getFilePath().u8string();
                    std::error_code removeError;
                    if (!fileStream->discardIncompleteFile(removeError))
                        CARB_LOG_ERROR("OmniPvd: failed to discard incomplete recording '%s': %s",
                                       path.c_str(), removeError ? removeError.message().c_str() : "close failed");
                }
                releaseOmniPvdWriteStream();
                OmniPhysX::getInstance().getISettings()->setBool(kOmniPvdIsRecording, false);
                return OmniPvdRecordingResult::eError;
            }
            OmniPhysX::getInstance().getISettings()->setBool(kOmniPvdIsRecording, true);
            mOmniPvdCaptureOwner.store(destination.transport == OmniPvdTransport::eTcp ? selectionOwner :
                                          omni::physics::kNoAttach, std::memory_order_release);
            mOmniPvdCaptureControlEnabled.store(destination.transport == OmniPvdTransport::eTcp,
                                               std::memory_order_release);
            if (mOmniPvdStreamKind == OmniPvdStreamKind::eRuntimeFile)
            {
                const OmniPvdDirectoryWriteStream* fileStream =
                    static_cast<const OmniPvdDirectoryWriteStream*>(mOmniPvdWriteStream);
                const std::string path = fileStream->getFilePath().u8string();
                CARB_LOG_INFO(OMNI_LOG_DEFAULT_CHANNEL, "OmniPvd: recording to '%s'", path.c_str());
            }
            return OmniPvdRecordingResult::eSuccess;
        }

        OmniPvdRecordingResult PhysXSetup::stopOmniPvdRecording()
        {
            if (!isOmniPvdRecording())
                return OmniPvdRecordingResult::eInvalidState;

            OmniPvdWriter* writer = mOmniPvd->getWriter();
            releaseVehiclePvdTelemetry();
            const bool samplingStopped = mOmniPvd->stopSampling();
            const bool writerSucceeded =
                !(writer->getStatus() & OmniPvdWriterStatusFlag::eSTREAM_WRITE_FAILURE);
            const bool closeStreamOk = releaseOmniPvdWriteStream();
            OmniPhysX::getInstance().getISettings()->setBool(kOmniPvdIsRecording, false);
            return samplingStopped && writerSucceeded && closeStreamOk ? OmniPvdRecordingResult::eSuccess :
                                                                         OmniPvdRecordingResult::eError;
        }

        bool PhysXSetup::isOmniPvdRecording() const
        {
            return mOmniPvd && (mOmniPvd->isSampling() || mOmniPvd->isCaptureDiscoveryActive());
        }

        bool PhysXSetup::closeOmniPvdWriteStream()
        {
            bool closeStreamOk = true;
            if (mOmniPvdStreamKind == OmniPvdStreamKind::eRuntimeFile)
            {
                closeStreamOk = !mOmniPvdWriteStream || mOmniPvdWriteStream->closeStream();
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
            mOmniPvdCaptureControlEnabled.store(false, std::memory_order_release);
            mOmniPvdCaptureOwner.store(omni::physics::kNoAttach, std::memory_order_release);
            mOmniPvdCaptureScenes.clear();
            // Reports whether the final flush/close of the capture stream succeeded so the
            // caller can decide whether the recording on disk is complete. The PxOmniPvd
            // writer only borrows this stream. Sampling must be stopped, the writer rebound,
            // or the provider released before this function destroys the stream.
            const bool closeStreamOk = closeOmniPvdWriteStream();
            if (mOmniPvdWriteStream)
            {
                if (mOmniPvdStreamKind == OmniPvdStreamKind::eRuntimeFile)
                {
                    delete mOmniPvdWriteStream;
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

        bool PhysXSetup::hasPendingOmniPvdCaptureRequest(AttachHandle attach)
        {
            return attach != omni::physics::kNoAttach && shouldProcessOmniPvdCaptureRequests(attach) &&
                   mOmniPvd && mOmniPvd->hasPendingCaptureRequest();
        }

        void PhysXSetup::processOmniPvdCaptureRequests(AttachHandle attach)
        {
            if (attach == omni::physics::kNoAttach || !shouldProcessOmniPvdCaptureRequests(attach) || !mOmniPvd)
                return;
            if (!mOmniPvd->processCaptureRequests(this) && !isOmniPvdRecording())
            {
                // Native sampling stopped after a terminal forward/control error.
                // Retire module telemetry and the borrowed stream at this same
                // admitted safe point so status/restart see an inactive session.
                releaseVehiclePvdTelemetry();
                releaseOmniPvdWriteStream();
                OmniPhysX::getInstance().getISettings()->setBool(kOmniPvdIsRecording, false);
                CARB_LOG_WARN("OmniPvd: recording stopped after a terminal TCP error");
            }
        }

        bool PhysXSetup::filterCaptureRequest(::physx::PxOmniPvdFilterDesc& filter)
        {
            return restrictOmniPvdCaptureToOwner(filter, getOmniPvdCaptureOwner(), mOmniPvdCaptureScenes,
                                               !mOmniPvd->isCaptureDiscoveryActive());
        }

        bool PhysXSetup::restrictOmniPvdCaptureToOwner(
            ::physx::PxOmniPvdFilterDesc& filter, AttachHandle owner,
            std::vector<const ::physx::PxScene*>& sceneStorage, bool expandEmpty)
        {
            if (owner == omni::physics::kNoAttach)
                return false;
            sceneStorage.clear();
            for (PhysXScenesMap::const_reference entry : mPhysXScenes)
                if (entry.second->getAttachedStage().getAttachHandle() == owner)
                    sceneStorage.push_back(entry.second->getScene());
            if (sceneStorage.empty())
                return false;
            const std::unordered_set<const ::physx::PxScene*> ownedScenes(
                sceneStorage.begin(), sceneStorage.end());
            const auto ownsScene = [&ownedScenes](const ::physx::PxScene* scene) {
                return ownedScenes.find(scene) != ownedScenes.end();
            };
            for (::physx::PxU32 index = 0; index < filter.nbScenes; ++index)
                if (!ownsScene(filter.scenes[index]))
                    return false;
            if (filter.nbObjects)
            {
                const std::shared_ptr<AttachedStage> attachment = UsdLoad::getUsdLoad()->acquireAttachment(owner);
                if (!attachment)
                    return false;
                std::unordered_set<const ::physx::PxBase*> ownedBuffers;
                for (const InternalDatabase::Record& record : attachment->getInternalPhysXDatabase().getRecords())
                    if (record.mType == ePTParticleSet && record.mPtr)
                        ownedBuffers.insert(omniPvdCaptureObject(record.mType, record.mPtr));
                for (::physx::PxU32 index = 0; index < filter.nbObjects; ++index)
                {
                    // The native API has already resolved each handle against live SDK objects.
                    // Actor and whole-articulation roots must belong to this attachment's scenes.
                    const ::physx::PxBase& object = *filter.objects[index];
                    if (const ::physx::PxActor* actor = object.is<::physx::PxActor>())
                    {
                        if (!ownsScene(actor->getScene()))
                            return false;
                    }
                    else if (const ::physx::PxArticulationReducedCoordinate* articulation =
                                 object.is<::physx::PxArticulationReducedCoordinate>())
                    {
                        if (!ownsScene(articulation->getScene()))
                            return false;
                    }
                    else if (ownedBuffers.find(&object) == ownedBuffers.end())
                        return false;
                }
            }
            // Native discovery resolves empty selectors within its original maximum scope.
            if (expandEmpty && filter.nbScenes == 0 && filter.nbObjects == 0)
            {
                filter.scenes = sceneStorage.data();
                filter.nbScenes = static_cast<::physx::PxU32>(sceneStorage.size());
            }
            return true;
        }

        void PhysXSetup::onCaptureReset(::physx::PxOmniPvd& omniPvd, const ::physx::PxOmniPvdFilterDesc& /*filter*/)
        {
            CARB_ASSERT(&omniPvd == mOmniPvd);
            if (&omniPvd == mOmniPvd)
            {
                releaseVehiclePvdTelemetry();
            }
        }

        void PhysXSetup::onStartSampling(::physx::PxOmniPvd& omniPvd, const ::physx::PxOmniPvdFilterDesc* filter)
        {
            mVehiclePvdSnapshotSucceeded = false;
            CARB_ASSERT(&omniPvd == mOmniPvd);
            if (&omniPvd != mOmniPvd)
                return;

            mOmniPvdRecordSecondaryData = !filter || filter->recordSecondaryData;
            if (!isOmniPvdSecondaryDataEnabled())
            {
                mVehiclePvdSnapshotSucceeded = true;
                return;
            }

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
                    if (!vehicle->mPhysXVehicle ||
                        !omniPvd.isObjectRecorded(*vehicle->mPhysXVehicle->getRigidDynamicActor()))
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
            if (hasResourceBorrowers())
                return;
            releaseCookingAsyncContext();
            getMeshCache()->release();
            SAFE_RELEASE(mCylinderMeshX);
            SAFE_RELEASE(mCylinderMeshY);
            SAFE_RELEASE(mCylinderMeshZ);
            SAFE_RELEASE(mConeMeshX);
            SAFE_RELEASE(mConeMeshY);
            SAFE_RELEASE(mConeMeshZ);
            // These meshes borrow this SDK too. Tolerance-driven recreation can
            // reach cleanup without the process-wide scene-release wrapper.
            SAFE_RELEASE(mVehicleWheelCylinderMeshX);
            SAFE_RELEASE(mVehicleWheelCylinderMeshY);
            SAFE_RELEASE(mVehicleWheelCylinderMeshZ);

            if (mExtensionsInitialized)
            {
                PxCloseExtensions();
                mExtensionsInitialized = false;
            }

            unregisterVehiclePvdCallback();
            releaseVehiclePvdTelemetry();
            SAFE_RELEASE(mSerializationRegistry)
            SAFE_RELEASE(mPhysics)
            mAttachmentCudaResources.clear();

            SAFE_RELEASE(mOmniPvd)
            releaseOmniPvdWriteStream();
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
            omni::physx::particles::notifyAllPhysXRelease();
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
                ref.second->getAttachedStage().getFilteredPairs().clear();
                ref.second->getAttachedStage().getCollisionGroupFilteredPairs().clear();
                ref.second->getAttachedStage().unregisterPhysXScene(ref.second);
                delete ref.second;
            }
            mPhysXScenes.clear();



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

            SAFE_RELEASE(mSerializationRegistry)
            SAFE_RELEASE(mPhysics)
            mAttachmentCudaResources.clear();
            SAFE_RELEASE(mVisualDebugger)

            SAFE_RELEASE(mOmniPvd)
            releaseOmniPvdWriteStream();
            SAFE_RELEASE(mFoundation)
            if (mISettings)
                mISettings->setBool(kOmniPvdIsRecording, false);

            mErrorCallback->invalidateEventStream();
        }

        bool PhysXSetup::hasResourceBorrowers() const
        {
            std::lock_guard<carb::tasking::RecursiveMutexWrapper> resources(mResourceMutex);
            {
                std::lock_guard<std::mutex> scenes(mSceneRegistryMutex);
                if (!mPhysXScenes.empty() || mReleasingPhysXScenes)
                    return true;
            }
            // Cooking task accounting can suspend; do not hold the native registry mutex here.
            return isOmniPvdRecording() || !mAttachmentBorrowers.empty() ||
                (mCookingDataAsync && mCookingDataAsync->getActiveTaskCount() != 0);
        }

        PhysxSharedSettings PhysXSetup::getSharedSettingsDefaults() const
        {
            carb::settings::ISettings* settings = OmniPhysX::getInstance().getISettings();
            PhysxSharedSettings result;
            result.threadCount = settings->isAccessibleAs(carb::dictionary::ItemType::eInt, kSettingNumThreads) ?
                settings->getAsInt(kSettingNumThreads) : mThreadCount;
            result.usePhysxDispatcher = settings->getAsBool(kSettingPhysxDispatcher);
            result.cudaDevice = settings->isAccessibleAs(carb::dictionary::ItemType::eInt, kSettingCudaDevice) ?
                settings->getAsInt(kSettingCudaDevice) : -1;
            result.useActiveCudaContext = settings->getAsBool(kSettingUseActiveCudaContext);
            result.synchronousCudaLaunches = settings->getAsBool(kSettingSynchronousKernelLaunches);
            result.sceneMultiGpuMode = settings->getAsInt(kSettingSceneMultiGPUMode);
            return result;
        }



        bool PhysXSetup::acquireAttachmentResources(usdparser::AttachedStage& attachment)
        {
            std::lock_guard<carb::tasking::RecursiveMutexWrapper> resources(mResourceMutex);
            const double metersPerUnit = attachment.getSourceUnits().metersPerUnit;
            const double kilogramsPerUnit = attachment.getSourceUnits().kilogramsPerUnit;
            if (!std::isfinite(metersPerUnit) || metersPerUnit <= 0.0 ||
                !std::isfinite(kilogramsPerUnit) || kilogramsPerUnit <= 0.0)
            {
                OVX_RUNTIME_ERROR("Attachment units must be finite and positive.");
                return false;
            }
            const PxTolerancesScale required = getDefaultTolerances(metersPerUnit);
            if (!required.isValid() || !std::isfinite(required.length) || !std::isfinite(required.speed))
            {
                OVX_RUNTIME_ERROR("Attachment units cannot be represented by a valid tolerance scale.");
                return false;
            }
            const PhysxSharedSettings& requested = attachment.getSharedSettings();
            if (requested.threadCount >= 256 || requested.cudaDevice < -1 ||
                requested.sceneMultiGpuMode < 0 || requested.sceneMultiGpuMode > 2)
                return false;
            carb::settings::ISettings* settings = OmniPhysX::getInstance().getISettings();
            const bool providerChanged = mPhysics &&
                (isOmniPvdRecordingCapable(*settings) != (mOmniPvd != nullptr) ||
                 settings->getAsBool(kSettingPVDEnabled) != (mVisualDebugger != nullptr));
            if (providerChanged)
            {
                if (hasResourceBorrowers())
                {
                    CARB_LOG_ERROR("Attachment diagnostic providers are incompatible with shared SDK borrowers.");
                    return false;
                }
                cleanupPhysics();
            }
            if (mPhysics)
            {
                const PxTolerancesScale existing = mPhysics->getTolerancesScale();
                if (existing.length != required.length || existing.speed != required.speed)
                {
                    if (hasResourceBorrowers())
                    {
                        CARB_LOG_ERROR("Attachment tolerance scale is incompatible with shared SDK borrowers "
                            "(attachments=%zu, scenes=%zu, releasing=%d, cooking=%u, recording=%d).",
                            mAttachmentBorrowers.size(), mPhysXScenes.size(), int(mReleasingPhysXScenes),
                            mCookingDataAsync ? mCookingDataAsync->getActiveTaskCount() : 0u, int(isOmniPvdRecording()));
                        return false;
                    }
                    cleanupPhysics();
                }
            }
            if (requested.usePhysxDispatcher)
            {
                auto* dispatcher = PxDefaultCpuDispatcherCreate(requested.threadCount);
                if (!dispatcher)
                    return false;
                dispatcher->setRunProfiled(true);
                attachment.cpuDispatcher = std::shared_ptr<PxCpuDispatcher>(dispatcher,
                    [](PxCpuDispatcher* value) { static_cast<PxDefaultCpuDispatcher*>(value)->release(); });
            }
            else
            {
                attachment.cpuDispatcher = std::shared_ptr<PxCpuDispatcher>(new PhysXCarbCpuDispatcher(requested.threadCount),
                    [](PxCpuDispatcher* value) { static_cast<PhysXCarbCpuDispatcher*>(value)->release(); delete value; });
            }
            if (!mPhysics)
                createPhysics(required);
            if (!mPhysics)
                return false;
            mAttachmentBorrowers.insert(&attachment);
            return true;
        }

        void PhysXSetup::releaseAttachmentResources(const usdparser::AttachedStage& attachment)
        {
            std::lock_guard<carb::tasking::RecursiveMutexWrapper> resources(mResourceMutex);
            mAttachmentBorrowers.erase(&attachment);
        }

        PxCudaContextManager* PhysXSetup::acquireAttachmentCudaContext(usdparser::AttachedStage& owner)
        {
#if USE_PHYSX_GPU
            std::lock_guard<carb::tasking::RecursiveMutexWrapper> resources(mResourceMutex);
            if (owner.cudaContextManagers.empty())
            {
                const PhysxSharedSettings& settings = owner.getSharedSettings();
                int deviceCount = 0;
                omni::physx::foundation::getOptionalCudaInterface().deviceGetCount(&deviceCount, nullptr);
                const bool multi = settings.sceneMultiGpuMode != 0 && deviceCount > 1;
                const int first = multi && settings.sceneMultiGpuMode == 2 ? 1 : 0;
                const int count = multi ? deviceCount : 1;
                std::vector<std::shared_ptr<PxCudaContextManager>> managers;
                for (int device = first; device < count; ++device)
                {
                    PhysxFoundationDeviceOrdinal ordinal;
                    ordinal.deviceOrdinal = multi ? device : settings.cudaDevice;
                    ordinal.mode = !multi && settings.useActiveCudaContext ?
                        PhysxFoundationDeviceOrdinal::eMODE_ACTIVE_CONTEXT :
                        (ordinal.deviceOrdinal >= 0 ? PhysxFoundationDeviceOrdinal::eMODE_DEVICE_ORDINAL :
                                                     PhysxFoundationDeviceOrdinal::eMODE_PHYSX_DEFAULTS);
                    uintptr_t externalContext = 0;
                    if (ordinal.mode == PhysxFoundationDeviceOrdinal::eMODE_ACTIVE_CONTEXT &&
                        (!omni::physx::foundation::getOptionalCudaInterface().ctxGetCurrent(&externalContext, nullptr) ||
                         externalContext == 0))
                        return nullptr;
                    auto compatible = std::find_if(mAttachmentCudaResources.begin(), mAttachmentCudaResources.end(),
                        [&](const AttachmentCudaResource& resource) {
                            if (resource.mode != int(ordinal.mode) || resource.ordinal != ordinal.deviceOrdinal ||
                                resource.externalContext != externalContext ||
                                resource.synchronous != settings.synchronousCudaLaunches)
                                return false;
                            PxScopedCudaLock contextLock(*resource.manager);
                            PxCudaContext* context = resource.manager->getCudaContext();
                            if (!context)
                                return false;
                            const bool aborted = context->isInAbortMode();
                            if (aborted)
                            {
                                // Only the SDK pool may retain it: every owner/candidate
                                // lease must have retired before clearing an allocation abort.
                                if (resource.manager.use_count() != 1)
                                    return false;
                                // SDK getLastError masks the underlying result with OOM
                                // while aborted. Clearing the latch also resets recoverable
                                // OOM, but preserves a fatal underlying CUDA error.
                                context->setAbortMode(false);
                            }
                            if (context->getLastError() != 0)
                            {
                                if (aborted)
                                    context->setAbortMode(true);
                                // Keep faulted managers alive for SDK caches, but do not reuse.
                                return false;
                            }
                            return true;
                        });
                    if (compatible != mAttachmentCudaResources.end())
                    {
                        managers.push_back(compatible->manager);
                        continue;
                    }
                    PxCudaContextManager* manager = nullptr;
                    if (!mPhysxFoundation->createOrRefreshPxCudaContextManager(
                            ordinal, mFoundation, manager, settings.synchronousCudaLaunches))
                    {
                        if (manager)
                            manager->release();
                        return nullptr;
                    }
                    std::shared_ptr<PxCudaContextManager> retained = std::shared_ptr<PxCudaContextManager>(manager,
                        [](PxCudaContextManager* value) { value->release(); });
                    mAttachmentCudaResources.push_back({int(ordinal.mode), ordinal.deviceOrdinal, externalContext,
                        settings.synchronousCudaLaunches, retained});
                    managers.push_back(std::move(retained));
                }
                owner.cudaContextManagers = std::move(managers);
            }
            return owner.cudaContextManagers[owner.nextCudaContext++ % owner.cudaContextManagers.size()].get();
#else
            return nullptr;
#endif
        }

        void PhysXSetup::reportCudaContextError(const usdparser::AttachedStage& owner, PxCudaContextManager* manager)
        {
#if USE_PHYSX_GPU
            if (!manager || !manager->getCudaContext())
                return;
            const PxCUresult error = manager->getCudaContext()->getLastError();
            // Allocation failures may legitimately trigger CPU fallback during scene creation.
            if (error == 0 || error == CUDA_OUT_OF_MEMORY)
                return;
            const char* message = "CUDA context error while fetching attachment simulation results";
            recordRuntimeError(message);
            CARB_LOG_ERROR("%s (attachment=%llu, CUDA error=%d)", message,
                static_cast<unsigned long long>(owner.getAttachHandle()), int(error));
            RuntimeCallbackScope callbackScope;
            sendErrorEvent(OmniPhysX::getInstance().getErrorEventStream(), ePhysxCudaError,
                std::make_pair("errorString", message),
                std::make_pair("attachHandle", int64_t(owner.getAttachHandle())));
#endif
        }

        PxPhysics* PhysXSetup::getPhysics()
        {
            // Realized attachments prepare the shared SDK from their source units
            // before loading. Source-free clients use the process default scale.
            if (!mPhysics)
                createPhysics(getDefaultTolerances(1.0));
            return mPhysics;
        }



        /**
         * @implements REQ-COOK-CUDACTX-001
         * @covers AC-1
         */




        void PhysXSetup::connectPVD()
        {
            mPvdHasSimulation.store(false);
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
                    if (mPvdHasSimulation.load())
                    {
                        carb::settings::ISettings* iSettings = OmniPhysX::getInstance().getISettings();
                        if (iSettings->getAsBool(kSettingPVDStreamToFile))
                        {
                            std::string outputFileDir = iSettings->getStringBuffer(kSettingPVDOutputDirectory);
                            if (!outputFileDir.empty())
                            {
                                PvdRecordingTime nowTs;
                                getLocalPvdTime(nowTs);
                                gLastPvdTime.setWithDiffCounterIncreaseIfSame(nowTs);
                                addLastSlash(outputFileDir);

                                char buffer[200];
                                sprintf_s(buffer, 200, "%04d_%02d_%02d_%02d_%02d_%02d_%02d",
                                    gLastPvdTime.year, gLastPvdTime.month, gLastPvdTime.day, gLastPvdTime.hour, gLastPvdTime.minute, gLastPvdTime.second, gLastPvdTime.diffCounter);
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
            if (std::isfinite(metersPerUnit) && metersPerUnit > 0.0)
            {
                const double length = 1.0 / metersPerUnit;
                const double speed = 10.0 / metersPerUnit;
                // Check before narrowing: a finite double can exceed the float range.
                if (std::isfinite(length) && std::isfinite(speed) && length <= PX_MAX_F32 && speed <= PX_MAX_F32)
                {
                    const ::physx::PxTolerancesScale tolerances{ float(length), float(speed) };
                    if (tolerances.isValid() && tolerances.speed > 0.0f)
                        return tolerances;
                }
            }
            CARB_LOG_ERROR("Invalid stage metersPerUnit (%g): expected a finite positive value producing "
                "finite positive PhysX tolerance scales.", metersPerUnit);
            return ::physx::PxTolerancesScale(0.0f, 0.0f);
        }





        void PhysXSetup::releasePhysXScenes(usdparser::AttachedStage& attachedStage)
        {
            std::vector<PhysXScene*> retiring;
            retiring.reserve(attachedStage.getPhysXScenes().size());
            for (usdparser::AttachedStage::SceneMap::const_reference entry : attachedStage.getPhysXScenes())
                retiring.push_back(entry.second);
            for (PhysXScene* scene : retiring)
                scene->waitForCompletion(false);

            // Remove every retiring scene from global iteration before any destructor
            // can wait on peers. The attachment and its source remain alive throughout.
            {
                std::lock_guard<std::mutex> registry(mSceneRegistryMutex);
                for (PhysXScene* scene : retiring)
                    mPhysXScenes.erase(scene->getScene());
            }
            attachedStage.getFilteredPairs().clear();
            attachedStage.getCollisionGroupFilteredPairs().clear();
            for (PhysXScene* scene : retiring)
            {
                attachedStage.unregisterPhysXScene(scene);
                delete scene;
            }
        }

        void PhysXSetup::releasePhysXScenes()
        {
            // Re-arm the sticky deferral logs; live references are gone.

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
                // still retain their attachment CUDA resources until native teardown finishes.
                mReleasingPhysXScenes = true;
                for (PhysXScenesMap::reference ref : physxScenes)
                {
                    ref.second->getAttachedStage().getFilteredPairs().clear();
                    ref.second->getAttachedStage().getCollisionGroupFilteredPairs().clear();
                    ref.second->getAttachedStage().unregisterPhysXScene(ref.second);
                    delete ref.second;
                }
                mReleasingPhysXScenes = false;
            }

            cleanupPhysics();

        }

        PhysXScene* PhysXSetup::createPhysXScene(usdparser::AttachedStage& attachedStage, size_t sceneId, double metersPerUnit, double kilogramsPerUnit, const usdparser::PhysxSceneDesc& sceneDesc)
        {
            std::lock_guard<carb::tasking::RecursiveMutexWrapper> resources(mResourceMutex);
            // Validate before SDK/dispatcher changes or native scene allocation. A peer
            // keeps the exact shared scale it was constructed against.
            if (!std::isfinite(metersPerUnit) || metersPerUnit <= 0.0 ||
                !std::isfinite(kilogramsPerUnit) || kilogramsPerUnit <= 0.0)
            {
                CARB_LOG_ERROR("Physics scene units must be finite and positive.");
                return nullptr;
            }
            const PxTolerancesScale sceneTol = getDefaultTolerances(metersPerUnit);
            if (!sceneTol.isValid() || !std::isfinite(sceneTol.length) || !std::isfinite(sceneTol.speed))
            {
                CARB_LOG_ERROR("Physics scene units cannot be represented by a valid tolerance scale.");
                return nullptr;
            }
            if (attachedStage.getPhysXScenes().find(sceneId) != attachedStage.getPhysXScenes().end())
            {
                CARB_LOG_ERROR("Duplicate physics scene ID in attachment.");
                return nullptr;
            }
            const PhysxSharedSettings& requested = attachedStage.getSharedSettings();
            if (mPhysics)
            {
                const PxTolerancesScale existing = mPhysics->getTolerancesScale();
                if (existing.length != sceneTol.length || existing.speed != sceneTol.speed)
                {
                    if (hasResourceBorrowers())
                    {
                        CARB_LOG_ERROR("Physics scene tolerance scale is incompatible with shared SDK borrowers.");
                        return nullptr;
                    }
                    // All attachment, native-scene and pending cooking borrowers are gone.
                    cleanupPhysics();
                    createPhysics(sceneTol);
                }
            }
            else
                createPhysics(sceneTol);
            if (!mPhysics)
                return nullptr;
            if (mCookingDataAsync)
                mCookingDataAsync->registerOnChangeFeed(attachedStage);

            PhysXScene* scene = PhysXScene::createPhysXScene(attachedStage, sceneId, *this, metersPerUnit, kilogramsPerUnit, sceneDesc);
            CARB_ASSERT(scene);
            if (scene)
            {
                try
                {
                    attachedStage.registerPhysXScene(sceneId, scene);
                    std::lock_guard<std::mutex> registry(mSceneRegistryMutex);
                    mPhysXScenes.emplace(scene->getScene(), scene);
                }
                catch (...)
                {
                    attachedStage.unregisterPhysXScene(scene);
                    delete scene;
                    throw;
                }

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

namespace omni
{
namespace physx
{
namespace runtime
{

// Defined here rather than beside the other runtime entry points in PhysX.cpp: the
// decisions it refreshes live in this file, next to the callback that reads them.
void refreshProfilerCallback()
{
    installProfilerCallback(carb::getCachedInterface<carb::settings::ISettings>());
}

} // namespace runtime
} // namespace physx
} // namespace omni
