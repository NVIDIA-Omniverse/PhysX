// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-COOK-CRC-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-COOK-CUDACTX-001
 * @covers AC-1 AC-2 AC-4 AC-5
 */

#include "CookingComputeService.h"

#include <list>
#include <unordered_map>
#include <unordered_set>

#include <omni/physx/MeshKey.h>
#include <private/omni/physx/PhysxUsd.h> // ErrorCode
#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>
#include <carb/extras/Timer.h>

#include <common/utilities/MemoryMacros.h>
#include <common/utilities/PhysXErrorCallback.h>

#include "CookingHashing.h"
#include "CookingTask.h"

#include <omni/convexdecomposition/ConvexDecomposition.h>
#include <omni/physx/IPhysxFoundation.h>
#include "PhysXFoundation.h"
#include <carb/Defines.h>
#include <carb/profiler/Profile.h>

#define USE_PHYSX_GPU 1 // GPU Rigid Bodies

// Scoped mutex lock
using lock_guard = std::lock_guard<carb::tasking::MutexWrapper>;
using namespace physx;
namespace omni
{
namespace physx
{

using MeshHashSet = std::unordered_set<omni::physx::usdparser::MeshKey, omni::physx::usdparser::MeshKeyHash>;

// Queue of outstanding cooking tasks
using CookingTaskQueue = std::list<cookingtask::CookingTask*>;

// Map from a task's opaque identity key (cookingtask::computeCookingTaskKey) to its active
// CookingTask. Only one active task per key; a task can also hold a single 'pending' task
// representing the next one to run once the current one completes.
using CookingTaskMap = std::unordered_map<std::string, cookingtask::CookingTask*>;

struct CookingComputeService : public ICookingComputeService
{
    explicit CookingComputeService(::physx::PxFoundation& foundation,
                                   AcquireSharedCudaContextManagerFn acquireSharedCudaContextManagerFn)
        : mPxFoundation(&foundation), mAcquireSharedCudaContextManagerFn(acquireSharedCudaContextManagerFn)
    {
        mPhysxFoundation = &omni::physx::foundation::getInterface();
        mTasking = carb::getCachedInterface<carb::tasking::ITasking>();

        // List of API improvements that will be applied when the next ABI Break will become necessary:
        // - triangulationMaxMaterialIndex --> to be moved inside PhysxCookingMeshTriangulationView
        // - isSynchronousResult and rightHandedOrientation need to become Flags
        // - Replace PhysxCookedDataSpan with omni::span
        // - PhysxCookingAsyncContextParameters::contextName must be changed from omni::span to omni::string_view
        // - Allow passing maxMaterialIndex inside PhysxCookingMeshView when doing input from
        // eINPUT_MODE_FROM_PRIM_MESH_VIEW

        // If any of these fails, you probably have been breaking the ABI. The two platforms are
        // pinned to different PhysxCookingComputeRequest sizes on purpose, not by oversight: this
        // struct embeds an omni::function (the onFinished callback), whose internal FunctionBuffer
        // is alignas(alignof(std::max_align_t)) (see omni/detail/FunctionImpl.h). alignof(max_align_t)
        // is itself platform-dependent -- 16 on GCC/Linux (it has to fit long double / __int128),
        // 8 on MSVC/Windows (whose long double is just double) -- so the compiler inserts 8 extra
        // bytes of padding before onFinished on Linux that Windows never needs. A binary never
        // crosses this boundary (a Windows client only ever links a Windows-built cooking service),
        // so per-platform constants here are correct ABI enforcement, not a workaround.
        //
        // 384/376 -> 360/368: removing DataInputMode dataInputMode and double primTimeCode (16
        // bytes, both ahead of the onFinished padding boundary) when eINPUT_MODE_FROM_PRIM_ID was
        // deleted (REQ-COOK-SOURCE-001). Linux value (368) measured directly; Windows value (360)
        // is the same -16 delta applied to the prior 376, not compiler-verified (no Windows
        // toolchain in this sandbox) -- the 8-byte Linux/Windows padding gap is isolated to the
        // onFinished boundary, untouched by these two members, which sit earlier in the struct.
#if CARB_PLATFORM_WINDOWS
        static_assert(sizeof(PhysxCookingComputeRequest) == 360, "sizeof(PhysxCookingComputeRequest)");
        static_assert(sizeof(PhysxCookingComputeResult) == 152, "sizeof(PhysxCookingComputeResult)");
#else
        static_assert(sizeof(PhysxCookingComputeRequest) == 368, "sizeof(PhysxCookingComputeRequest)");
        static_assert(sizeof(PhysxCookingComputeResult) == 152, "sizeof(PhysxCookingComputeResult)");
#endif
        static_assert(offsetof(PhysxCookingComputeResult, resultSource) == 136, "ABI Broken");
    }

    ~CookingComputeService(void)
    {
        // Cancel active tasks first (their results won't be processed anymore), then delete them; each
        // task's destructor blocks until its background thread (if any) has completed.
        for (auto& it : mAsyncContext)
        {
            AsyncContext& asyncContext = *it.second.get();
            cancelAllTasks(&asyncContext);
            finalizeAllTasks(asyncContext);
        }

        // mPxCudaContextManager is always a reference we own -- one we created ourselves, or one
        // the provider handed us pre-acquired. Releasing it unconditionally is therefore correct
        // for both: the provider-owned manager survives this because the provider holds its own
        // reference. Tasks that outlived us are impossible (they were cancelled and deleted
        // above), and any that did would hold their own reference anyway.
        SAFE_RELEASE(mPxCudaContextManager);
        mPxFoundation = nullptr;
    }
    struct AsyncContext
    {
        std::string name;
        CookingTaskQueue mTasks;
        CookingTaskMap mTaskMap;
        MeshHashSet mCookedDataCRCSet;
    };

    void finalizeAllTasks(AsyncContext& asyncContext)
    {
        lock_guard globalLock(mGlobalMutex);
        for (auto& i : asyncContext.mTasks)
        {
            delete i;
        }
    }

    /**
     * This method is called once per logical 'frame' from the main thread to
     * dispatch new cooking tasks as well as process the results of cooking tasks
     * which have completed.
     *
     * @return : Returns the number of cooking tasks still active/pending
     */
    virtual uint32_t pumpAsyncContext(PhysxCookingAsyncContext context) override final
    {
        CARB_PROFILE_ZONE(0, "CookingComputeService::pumpAsyncContext");
        if (validateContext(context))
        {
            dispatchAsyncTasks(context);
        }
        return getActiveTaskCount(context);
    }

    void dispatchAsyncTasks(PhysxCookingAsyncContext context)
    {
        lock_guard globalLock(mGlobalMutex);
        AsyncContext& asyncContext = *reinterpret_cast<AsyncContext*>(context);
        if (!asyncContext.mTasks.empty())
        {
            // Time-box result processing below so we don't stall the editor if too many tasks finish at once.
            carb::extras::Timer timer;
            timer.start();

            uint32_t count = 0;

            CookingTaskQueue::iterator i = asyncContext.mTasks.begin();

            // Tasks that were pending and need to be (re-)scheduled once their predecessor finishes below
            std::vector<cookingtask::CookingTask*> newTasks;

            while (i != asyncContext.mTasks.end() && count < MAX_ACTIVE_TASK_COUNT)
            {
                cookingtask::CookingTask* t = (*i);
                if (t->getAsyncContext() != &asyncContext) // task belongs to a different context, skip it
                {
                    i++;
                    continue;
                }
                // pump() returns true once finished; it also starts the background task if not started yet
                bool finished = t->pump(mTasking);
                if (finished)
                {
                    i = asyncContext.mTasks.erase(i);
                    cookingtask::CookingTask* nt = t->getPendingTask();
                    if (nt)
                    {
                        newTasks.push_back(nt);
                    }
                    // Finalize on the main thread and delete the task instance
                    removeCookingTask(t);

                    // Bail out if finalizing tasks has taken more than 16ms, to avoid stalling the frame
                    auto dtime = timer.getElapsedTime<int64_t>();
                    if (dtime >= 16)
                    {
                        break;
                    }
                }
                else
                {
                    i++;
                    count++;
                }
            }
            for (auto& i : newTasks)
            {
                addCookingTask(i, asyncContext);
            }
        }
    }

    virtual void resetMeshCacheContents() override final
    {

    }

    template <typename DeriveCRCFunction, typename CreateTaskFunction>
    PhysxCookingOperationHandle requestCookedData(PhysxCookingDataType::Enum DataType,
                                                  PhysxCookingAsyncContext context,
                                                  const PhysxCookingComputeRequest& request,
                                                  DeriveCRCFunction deriveCRCFunction,
                                                  CreateTaskFunction createTaskFunction,
                                                  bool skipMeshProcessing = false)
    {
        PhysxCookingComputeResult result;
        PhysxCookingComputeRequest requestCopy = request;
        result.request = &requestCopy;
        requestCopy.dataType = DataType;
        if (!validateContext(result, context))
            return nullptr;
        AsyncContext* asyncContext = reinterpret_cast<AsyncContext*>(context);

        // Compute Mesh Key and CRC
        if (!skipMeshProcessing)
        {
            if (!computeMeshKeyIfNeeded(result, requestCopy))
                return nullptr;
        }

        result.cookedDataCRC = deriveCRCFunction(result);

        // The unit scale reaches the cooker as a PxTolerancesScale and changes the cooked
        // geometry, so it belongs in the key that identifies that geometry. See
        // MeshCRCComputation::foldMetersPerUnit for why it is folded here and not per data type,
        // and for the one-time cache invalidation this costs.
        MeshCRCComputation::foldMetersPerUnit(result.cookedDataCRC, requestCopy.primMeshMetersPerUnit);

        if (!result.request->options.hasFlag(PhysxCookingComputeRequest::Options::kComputeGPUCookingData))
            result.cookedDataCRC.setComputeGPUData(false);

        if (request.mode == PhysxCookingComputeRequest::eMODE_COMPUTE_CRC)
        {
            result.result = PhysxCookingResult::eVALID;
            request.onFinished(result);
            return nullptr;
        }

        if (haveCookedDataCRC(asyncContext, result.cookedDataCRC))
        {
            // A task already exists with same CRC so no need to spawn a new one.
            const std::string taskKey = cookingtask::computeCookingTaskKey(result.request->primId, std::string());
            cookingtask::CookingTask* task = findOpenTask(asyncContext, taskKey, result.cookedDataCRC, true);
            if (task)
            {
                if (result.request->options.hasFlag(PhysxCookingComputeRequest::Options::kComputeAsynchronously))
                {
                    // Before returning an handle however, we must check if it belongs to the same context
                    // otherwise we would be invoking its callback from some wrong thread later on.
                    if (task->getAsyncContext() == context)
                    {
                        // We save callback from this request to call it later
                        task->saveCallbackFromRequest(*result.request);
                        return task;
                    }
                }
                else
                {
                    // If synchronous, wait for the task to finish, so it should write to cache
                    // and make next call to loadingDataFromCacheSucceeds actually return data
                    completeOpenTask(*task);
                }
            }
        }

        cookingtask::CookingTask* task = createTaskFunction(result);
        return tryQueueingOrRunningTask(task->getResultObject(), requestCopy, task, asyncContext, skipMeshProcessing);
    }

    virtual bool acquireCudaContextManager(PhysxCookingDataType::Enum dataType,
                                           const PhysxCookingComputeRequest& request,
                                           ::physx::PxCudaContextManager*& cudaContextManager,
                                           ::physx::PxPhysicsGpu*& physicsGPU) override final
    {
#if USE_PHYSX_GPU
        const bool executeCookingOnGPU =
            dataType == PhysxCookingDataType::eSDF_TRIANGLE_MESH ?
                request.options.hasFlag(PhysxCookingComputeRequest::Options::kExecuteCookingOnGPU) :
                false;
        if (executeCookingOnGPU)
        {
            if (cudaContextManager)
            {
                // Caller-supplied manager: it guarantees the pointer is live for this call, so
                // taking our own reference here is safe and makes the handoff to the task
                // symmetric with the resolved path below.
                cudaContextManager->acquireReference();
            }
            else
            {
                lock_guard globalLock(mGlobalMutex);

                if (mAcquireSharedCudaContextManagerFn)
                {
                    // The provider hands back a manager with a reference already taken (see
                    // AcquireSharedCudaContextManagerFn), so mPxCudaContextManager below always
                    // holds a reference we own, whatever its origin.
                    if (::physx::PxCudaContextManager* shared = mAcquireSharedCudaContextManagerFn())
                    {
                        if (mPxCudaContextManager == shared)
                        {
                            // Already tracking this one; we do not need a second reference.
                            shared->release();
                        }
                        else
                        {
                            // Drop the one we were tracking. It stays alive for as long as any
                            // in-flight task still holds its own reference, so replacing it here
                            // can no longer strand a cooking job on a destroyed manager.
                            SAFE_RELEASE(mPxCudaContextManager);
                            mPxCudaContextManager = shared;
                        }
                        mUsingSharedContextManager = true;
                    }
                    else if (mUsingSharedContextManager)
                    {
                        // The provider no longer has a manager: drop ours and fall through to
                        // creating our own on the next request.
                        SAFE_RELEASE(mPxCudaContextManager);
                        mUsingSharedContextManager = false;
                    }
                }

                if (!mUsingSharedContextManager)
                {
                    // Only a manager we created ourselves may be refreshed here:
                    // createOrRefreshPxCudaContextManager() consumes the reference it is handed
                    // and may release+replace it, which would leave the provider pointing at a
                    // manager it no longer owns.
                    omni::physx::PhysxFoundationDeviceOrdinal ordinal;
                    mPhysxFoundation->getSingleCudaContextManagerOrdinal(ordinal);

                    if (!mPhysxFoundation->createOrRefreshPxCudaContextManager(ordinal, mPxFoundation, mPxCudaContextManager, false))
                    {
                        PhysxCookingComputeResult result;
                        PhysxCookingComputeRequest requestCopy = request;
                        result.request = &requestCopy;
                        requestCopy.dataType = dataType;
                        CARB_LOG_ERROR("Cannot create PxCudaContextManager");
                        result.result = PhysxCookingResult::eERROR_CUDA_CONTEXT_MANAGER;
                        result.request->onFinished(result);
                        return false;
                    }
                }

                cudaContextManager = mPxCudaContextManager;

                // Hand the caller its own reference, taken while we still hold mGlobalMutex and
                // while our own reference guarantees the manager is alive. This is the reference
                // the caller owes a release() for.
                if (cudaContextManager)
                {
                    cudaContextManager->acquireReference();
                }
            }
        }
        else
        {
            cudaContextManager = nullptr;
        }
#else
        // No GPU support compiled in: report "no manager" rather than echoing a caller-supplied
        // pointer back out, so the ownership contract ("non-null out means you owe a release()")
        // holds identically in both build configurations.
        cudaContextManager = nullptr;
#endif
        physicsGPU = cudaContextManager ? PxGetPhysicsGpu() : nullptr;
        return true;
    }

    virtual PhysxCookingOperationHandle requestTriangleMeshCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::TriangleMeshCookingParams& triangleMeshCookingParams,
        const omni::physx::SdfMeshCookingParams& sdfMeshCookingParams,
        ::physx::PxCudaContextManager* cudaContextManager) override
    {
        CARB_PROFILE_ZONE(0, "CookingComputeService::requestTriangleMeshCookedData");
        PhysxCookingDataType::Enum dataType = sdfMeshCookingParams.sdfResolution > 0 ?
                                                  PhysxCookingDataType::eSDF_TRIANGLE_MESH :
                                                  PhysxCookingDataType::eTRIANGLE_MESH;
        ::physx::PxPhysicsGpu* physicsGPU = nullptr;
        if (!acquireCudaContextManager(dataType, request, cudaContextManager, physicsGPU))
            return nullptr;

        const PhysxCookingOperationHandle handle = requestCookedData(
            dataType, context, request,
            [&](const PhysxCookingComputeResult& result) {
                auto meshKeyWithOrientation = result.meshKey;
                meshKeyWithOrientation.setRightHandedOrientation(result.request->primMeshView.rightHandedOrientation);
                return MeshCRCComputation::deriveTriangleMeshCRC(
                    meshKeyWithOrientation, triangleMeshCookingParams, sdfMeshCookingParams);
            },
            [&](PhysxCookingComputeResult& result) {
                cookingtask::CookingTask* task = cookingtask::createTriangleMeshCookingTask(
                    triangleMeshCookingParams, sdfMeshCookingParams, result);
                // Takes its own reference and holds it until the task is destroyed, which is what
                // keeps the manager alive for a queued task running later on a worker thread.
                task->setPxCudaAndGPUPointers(cudaContextManager, physicsGPU);
                return task;
            });

        // Drop the reference acquireCudaContextManager() handed us. This also covers the paths
        // where the task-creating lambda never ran (cache hit, early-out), which would otherwise
        // leak a reference and keep the manager - and its CUDA context - alive forever.
        SAFE_RELEASE(cudaContextManager);
        return handle;
    }

    virtual PhysxCookingOperationHandle requestConvexMeshCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::ConvexMeshCookingParams& convexCookingParams) override
    {
        return requestCookedData(
            PhysxCookingDataType::eCONVEX_MESH, context, request,
            [&](const PhysxCookingComputeResult& result) {
                return MeshCRCComputation::deriveConvexMeshCRC(result.meshKey, convexCookingParams);
            },
            [&](PhysxCookingComputeResult& result) {
                return cookingtask::createConvexMeshCookingTask(convexCookingParams, result);
            });
    }

    virtual PhysxCookingOperationHandle requestConvexMeshDecompositionCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::ConvexDecompositionCookingParams& convexDecompositionCookingParams) override
    {
        return requestCookedData(
            PhysxCookingDataType::eCONVEX_DECOMPOSITION, context, request,
            [&](const PhysxCookingComputeResult& result) {
                auto meshKeyWithOrientation = result.meshKey;
                meshKeyWithOrientation.setRightHandedOrientation(result.request->primMeshView.rightHandedOrientation);
                return MeshCRCComputation::deriveConvexDecompositionCRC(
                    meshKeyWithOrientation, convexDecompositionCookingParams);
            },
            [&](PhysxCookingComputeResult& result) {
                return cookingtask::createConvexDecompositionCookingTask(
                    convexDecompositionCookingParams, result, mConvexDecomposition);
            });
    }

    virtual PhysxCookingOperationHandle requestSphereFillCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::SphereFillCookingParams& sphereFillCookingParams) override
    {
        return requestCookedData(
            PhysxCookingDataType::eSPHERE_FILL, context, request,
            [&](const PhysxCookingComputeResult& result) {
                auto meshKeyWithOrientation = result.meshKey;
                meshKeyWithOrientation.setRightHandedOrientation(result.request->primMeshView.rightHandedOrientation);
                return MeshCRCComputation::deriveSphereFillCRC(meshKeyWithOrientation, sphereFillCookingParams);
            },
            [&](PhysxCookingComputeResult& result) {
                return cookingtask::createSphereFillCookingTask(sphereFillCookingParams, result, mConvexDecomposition);
            });
    }

    virtual PhysxCookingOperationHandle requestParticlePoissonSamplingCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::ParticlePoissonSamplingCookingParams& params)
    {
        return requestCookedData(
            PhysxCookingDataType::ePARTICLE_POISSON_SAMPLING, context, request,
            [&](const PhysxCookingComputeResult& result) {
                return MeshCRCComputation::deriveParticlePoissonSamplingCRC(
                    result.meshKey, params);
            },
            [&](PhysxCookingComputeResult& result) {
                return cookingtask::createPoissonSamplingCookingTask(params, result);
            });
    }

    virtual PhysxCookingOperationHandle requestDeformableVolumeMeshCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::DeformableVolumeMeshCookingParams& params) override
    {
        const bool skipMeshProcessing = true;
        return requestCookedData(
            PhysxCookingDataType::eDEFORMABLE_VOLUME_MESH, context, request,
            [&](const PhysxCookingComputeResult& result) {
                return MeshCRCComputation::computeDeformableVolumeMeshCRC(params);
            },
            [&](PhysxCookingComputeResult& result) {
                return cookingtask::createDeformableVolumeMeshCookingTask(params, result);
            },
            skipMeshProcessing);
    }

    virtual PhysxCookingOperationHandle requestVolumeDeformableBodyCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::VolumeDeformableBodyCookingParams& params)
    {
        return requestCookedData(
            PhysxCookingDataType::eVOLUME_DEFORMABLE_BODY, context, request,
            [&](const PhysxCookingComputeResult& result) {
                return MeshCRCComputation::deriveVolumeDeformableBodyCRC(result.meshKey, params);
            },
            [&](PhysxCookingComputeResult& result) {
                return cookingtask::createVolumeDeformableBodyCookingTask(params, result);
            });
    }

    virtual PhysxCookingOperationHandle requestSurfaceDeformableBodyCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::SurfaceDeformableBodyCookingParams& params)
    {
        return requestCookedData(
            PhysxCookingDataType::eSURFACE_DEFORMABLE_BODY, context, request,
            [&](const PhysxCookingComputeResult& result) {
                return MeshCRCComputation::deriveSurfaceDeformableBodyCRC(result.meshKey, params);
            },
            [&](PhysxCookingComputeResult& result) {
                return cookingtask::createSurfaceDeformableBodyCookingTask(params, result);
            });
    }

    virtual void release(void) override final
    {
        delete this;
    }

    void addCookingTask(cookingtask::CookingTask* t, AsyncContext& asyncContext)
    {
        t->getResultObject().isSynchronousResult = false;
        t->setAsyncContext(&asyncContext);
        CookingTaskMap::iterator found = asyncContext.mTaskMap.find(t->getTaskKey());
        if (found == asyncContext.mTaskMap.end())
        {
            asyncContext.mTaskMap[t->getTaskKey()] = t;
            asyncContext.mTasks.push_back(t);
            omni::physx::usdparser::MeshKey crc;
            t->getCRC(crc);
            addCookedDataCRC(asyncContext, crc);
        }
        else
        {
            cookingtask::CookingTask* ct = (*found).second;
            ct->cancel(false); // this new one takes precedence
            omni::physx::usdparser::MeshKey crc;
            ct->getCRC(crc);
            removeCookedDataCRC(asyncContext, crc);
            // Unregister the CRC of any previous pending task being overwritten
            cookingtask::CookingTask* p = ct->getPendingTask();
            if (p)
            {
                p->getCRC(crc);
                removeCookedDataCRC(asyncContext, crc);
                // getPendingTask() extracts rather than peeks - it clears ct->m_pending - so the
                // cancel+delete inside addPendingTask() below never sees this task. Nothing else
                // owns it either: a pending task is in neither mTasks nor mTaskMap. It has to be
                // dropped here, or ~CookingTaskImpl() never runs and the PxCudaContextManager
                // reference the task holds (REQ-COOK-CUDACTX-001) is never released, pinning the
                // CUDA context, GPU kernel modules and device memory for the process lifetime.
                // cancel(false) first: without it ~TriangleMeshCookingTask() -> finalize() reports
                // eERROR_COOKING_FAILED to onFinished for a task we are deliberately discarding.
                // fireFinishedCallback() stays silent only for eERROR_CANCELED with
                // invokeCallbackAnyway false, which is exactly what cancel(false) sets up.
                p->cancel(false);
                delete p;
            }
            ct->addPendingTask(t);
            t->getCRC(crc);
            addCookedDataCRC(asyncContext, crc);
        }
    }

    void removeCookingTask(cookingtask::CookingTask* t)
    {
        AsyncContext& asyncContext = *reinterpret_cast<AsyncContext*>(t->getAsyncContext());

        CookingTaskMap::iterator found = asyncContext.mTaskMap.find(t->getTaskKey());
        if (found != asyncContext.mTaskMap.end())
        {
            omni::physx::usdparser::MeshKey crc;
            t->getCRC(crc);
            removeCookedDataCRC(asyncContext, crc);
            asyncContext.mTaskMap.erase(found);
        }
        delete t;
    }

    /**
     * Returns not only the number of active tasks but those which we have scheduled to be inspected due to property
     * changes as well.
     *
     * @return : Returns the sum of the number of pending cooking tasks plus the number of paths we want to re-inspect
     */
    virtual uint32_t getActiveTaskCount(PhysxCookingAsyncContext context) override final
    {
        lock_guard globalLock(mGlobalMutex);
        if (!isValidContext(globalLock, context))
            return 0;
        AsyncContext& asyncContext = *reinterpret_cast<AsyncContext*>(context);
        return static_cast<uint32_t>(asyncContext.mTasks.size());
    }

    /**
     * Mark all cooking tasks as being canceled. Their results will be thrown away.
     *
     * @return : Returns the number of active tasks which were marked for cancelation
     */
    virtual uint32_t cancelAllTasks(PhysxCookingAsyncContext context) override final
    {
        lock_guard globalLock(mGlobalMutex);
        if (!isValidContext(globalLock, context))
            return 0;

        AsyncContext& asyncContext = *reinterpret_cast<AsyncContext*>(context);
        uint32_t ret = 0;
        for (auto& i : asyncContext.mTasks)
        {
            i->cancel(false);
            ret++;
        }

        return ret;
    }

    virtual bool cancelTask(PhysxCookingOperationHandle handle, bool invokeCallbackAnyway) override final
    {
        lock_guard globalLock(mGlobalMutex);
        for (auto& context : mAsyncContext)
        {
            for (cookingtask::CookingTask* task : context.second.get()->mTasks)
            {
                if (task == handle)
                {
                    task->cancel(invokeCallbackAnyway);
                    return true;
                }
            }
        }
        CARB_LOG_WARN("Trying to cancel a task with handle %p that doesn't exist", handle);
        return false;
    }

    virtual bool waitForTaskToFinish(PhysxCookingOperationHandle handle, int64_t timeoutMs) override final
    {
        lock_guard globalLock(mGlobalMutex);
        for (auto& context : mAsyncContext)
        {
            for (cookingtask::CookingTask* task : context.second.get()->mTasks)
            {
                if (task == handle)
                {
                    AsyncContext& context = *reinterpret_cast<AsyncContext*>(task->getAsyncContext());
                    if (task->isCanceled())
                    {
                        return true;
                    }

                    if (!task->hasStarted())
                        task->pump(mTasking); // start task in another thread

                    if (task->futureWait(timeoutMs))
                    {
                        return true;
                    }
                    CARB_LOG_WARN("Timeout occurred (%dms) while waiting for task %p from \"%s\" context",
                                  static_cast<int>(timeoutMs), handle, context.name.c_str());
                    return false;
                }
            }
        }
        CARB_LOG_WARN("Trying to wait for a task with handle %p that is not in the list of active tasks", handle);
        return false;
    }
    /**
     * Returns the total number of cooking tasks which have been performed since the
     * start of the application. This is used by debug visualization (omni.physx.ui) to
     * know whether or not it should refresh the debug visualization of a primitive because
     * the cooking state has changed since the last time.
     */
    virtual uint32_t getFinishedCookingTasksCount() const override final
    {
        return mFinishedCookingTasksCount;
    }

    bool addCookedDataCRC(AsyncContext& asyncContext, const omni::physx::usdparser::MeshKey& key)
    {
        bool ret = false;
        MeshHashSet::iterator found = asyncContext.mCookedDataCRCSet.find(key);
        if (found == asyncContext.mCookedDataCRCSet.end())
        {
            ret = true;
            asyncContext.mCookedDataCRCSet.insert(key);
        }
        return ret;
    }

    bool removeCookedDataCRC(AsyncContext& asyncContext, const omni::physx::usdparser::MeshKey& key)
    {
        bool ret = false;
        MeshHashSet::iterator found = asyncContext.mCookedDataCRCSet.find(key);
        if (found != asyncContext.mCookedDataCRCSet.end())
        {
            ret = true;
            asyncContext.mCookedDataCRCSet.erase(found);
        }
        return ret;
    }

    bool haveCookedDataCRC(AsyncContext* asyncContext, const omni::physx::usdparser::MeshKey& key) const
    {
        if (asyncContext)
        {
            return haveCookedDataCRCInContext(*asyncContext, key);
        }
        for (auto& context : mAsyncContext)
        {
            if (haveCookedDataCRCInContext(*context.second.get(), key))
            {
                return true;
            }
        }
        return false;
    }

    bool haveCookedDataCRCInContext(AsyncContext& asyncContext, const omni::physx::usdparser::MeshKey& key) const
    {
        bool ret = false;

        MeshHashSet::const_iterator found = asyncContext.mCookedDataCRCSet.find(key);
        if (found != asyncContext.mCookedDataCRCSet.end())
        {
            ret = true;
        }

        return ret;
    }

    /**
     * Find an open task with matching task key and MeshKey which hasn't been cancelled.
     *
     * @param taskKey : opaque task identity (cookingtask::computeCookingTaskKey) identifying the task.
     * @param crc : MeshKey identifying the task.
     * @return : pending task if found, nullptr otherwise.
     */
    cookingtask::CookingTask* findOpenTask(AsyncContext* asyncContext,
                                           const std::string& taskKey,
                                           const omni::physx::usdparser::MeshKey& crc,
                                           bool crcOnly = false)
    {
        if (asyncContext != nullptr)
        {
            return findOpenTaskInContext(*asyncContext, taskKey, crc, crcOnly);
        }
        else
        {
            for (auto& context : mAsyncContext)
            {
                cookingtask::CookingTask* task = findOpenTaskInContext(*context.second.get(), taskKey, crc, crcOnly);
                if (task)
                {
                    return task;
                }
            }
        }
        return nullptr;
    }

    cookingtask::CookingTask* findOpenTaskInContext(AsyncContext& asyncContext,
                                                    const std::string& taskKey,
                                                    const omni::physx::usdparser::MeshKey& crc,
                                                    bool crcOnly)
    {

        CookingTaskMap::iterator found = asyncContext.mTaskMap.find(taskKey);
        if (found == asyncContext.mTaskMap.end())
        {
            // Deformables and other "not 100% aligned" cooking approximation have
            // issues with matching just by CRC only for some reason
            if (crcOnly)
            {
                // Let's try to find another in flight task with the same CRC,
                // but having a different task key
                for (const auto& it : asyncContext.mTaskMap)
                {
                    cookingtask::CookingTask* ct = it.second;
                    omni::physx::usdparser::MeshKey ct_crc;
                    ct->getCRC(ct_crc);
                    if (ct_crc == crc && !ct->isCanceled())
                    {
                        return ct;
                    }
                }
            }
        }
        else
        {
            cookingtask::CookingTask* ct = (*found).second;
            omni::physx::usdparser::MeshKey ct_crc;
            ct->getCRC(ct_crc);
            if (ct_crc == crc && !ct->isCanceled())
            {
                return ct;
            }
        }
        return nullptr;
    }

    /**
     * Block until open task is completed, or execute manually if it hasn't started yet.
     *
     * @param task : task to be completed
     */
    void completeOpenTask(cookingtask::CookingTask& task)
    {
        if (task.isCanceled())
        {
            return;
        }

        if (task.hasStarted())
        {
            task.futureWait(-1);
        }
        else
        {
            // if the task hasn't started yet, we need to execute it manually.
            task.performTask();
        }
        finalizeTask(task);
    }

    void finalizeTask(cookingtask::CookingTask& task)
    {
        mFinishedCookingTasksCount++;
        // We must protect from the cache being reset while this is run from an arbitrary thread
        lock_guard globalLock(mGlobalMutex);
        task.finalize();
    }

    PhysxCookingOperationHandle tryQueueingOrRunningTask(PhysxCookingComputeResult& result,
                                                         PhysxCookingComputeRequest& request,
                                                         cookingtask::CookingTask* task,
                                                         AsyncContext* asyncContext,
                                                         bool skipMeshProcessing)
    {
        CARB_PROFILE_ZONE(0, "ICookingComputeService::tryQueueingOrRunningTask");
        if (!skipMeshProcessing)
        {
            // primMeshView is already filled by the caller (eINPUT_MODE_FROM_PRIM_ID removed).
            result.triangulationMaxMaterialIndex = CookingComputeService::getMaxMaterialIndex(request.primMeshView);
        }

        if (task->setupTaskFromRequest(request, skipMeshProcessing))
        {
            if (request.options.hasFlag(PhysxCookingComputeRequest::Options::kComputeAsynchronously))
            {
                addCookingTask(task, *asyncContext);
            }
            else
            {
                task->performTask();
                finalizeTask(*task);
                delete task;
                task = nullptr;
            }
        }
        else
        {
            delete task;
            task = nullptr;
            CARB_LOG_ERROR("PhysX could not copy USD data for cooking task!");
            result.result = PhysxCookingResult::eERROR_INVALID_PRIM;
            request.onFinished(result);
        }
        request.primMeshView = PhysxCookingMeshView(); // The view points at data that will end with parent scope
        request.volumeMeshView = PhysxCookingDeformableVolumeMeshView();
        request.volumeDeformableBodyView = PhysxCookingDeformableBodyView();
        request.surfaceDeformableBodyView = PhysxCookingDeformableBodyView();
        return task;
    }

    virtual PhysxCookingAsyncContext createAsyncContext(PhysxCookingAsyncContextParameters& parameters) override
    {
        lock_guard globalLock(mGlobalMutex);
        std::string name;
        name.insert(name.begin(), parameters.contextName.data(),
                    parameters.contextName.data() + parameters.contextName.size_bytes());

        if (mAsyncContext.find(name) != mAsyncContext.end())
        {
            CARB_LOG_ERROR("Trying to create a context with name \"%s\" that already exists", name.c_str());
            return nullptr;
        }
        std::unique_ptr<AsyncContext> context = std::make_unique<AsyncContext>();
        context->name = std::move(name);
        PhysxCookingAsyncContext handle = context.get();
        std::string& key = context->name;
        mAsyncContext[key] = std::move(context);
        return handle;
    }

    virtual void destroyAsyncContext(PhysxCookingAsyncContext context) override
    {
        lock_guard globalLock(mGlobalMutex);
        if (isValidContext(globalLock, context))
        {
            AsyncContext& asyncContext = *reinterpret_cast<AsyncContext*>(context);
            bool someTaskExists = false;
            int numActiveTasks = 0;
            for (auto it = asyncContext.mTasks.begin(); it != asyncContext.mTasks.end();)
            {
                cookingtask::CookingTask* task = *it;
                it = asyncContext.mTasks.erase(it);
                numActiveTasks++;
                someTaskExists = true;
                task->cancel(false);
                removeCookingTask(task);
            }

            if (someTaskExists)
            {
                AsyncContext* asyncContext = reinterpret_cast<AsyncContext*>(context);
                CARB_LOG_WARN("Destroying async context \"%s\" holding %d active async tasks.",
                              asyncContext->name.c_str(), numActiveTasks);
            }
            std::string key = reinterpret_cast<AsyncContext*>(context)->name;
            mAsyncContext.erase(key);
        }
        else
        {
            CARB_LOG_ERROR(
                "omni.physx.cooking: trying to delete a context with handle \"%p\" that doesn't exist", context);
        }
    }

    bool isValidContext(lock_guard& guard, PhysxCookingAsyncContext context)
    {
        for (auto& item : mAsyncContext)
        {
            if (item.second.get() == context)
            {
                return true;
            }
        }
        return false;
    }

    bool validateContext(PhysxCookingAsyncContext context)
    {
        bool invalidContext;
        {
            lock_guard globalLock(mGlobalMutex);
            invalidContext = context == nullptr || !isValidContext(globalLock, context);
        }
        if (invalidContext)
        {
            CARB_LOG_ERROR("omni.physx.cooking: Invalid context \"%p\" used", context);
            return false;
        }
        return true;
    }

    bool validateContext(omni::physx::PhysxCookingComputeResult& result, PhysxCookingAsyncContext context)
    {
        if (result.request->options.hasFlag(PhysxCookingComputeRequest::Options::kComputeAsynchronously))
        {
            if (!validateContext(context))
            {
                result.result = PhysxCookingResult::eERROR_INVALID_CONTEXT;
                result.request->onFinished(result);
                return false;
            }
        }
        return true;
    }

private:
    std::unordered_map<std::string, std::unique_ptr<AsyncContext>> mAsyncContext;
    // A reference we own, whatever its origin: self-created, or handed over pre-acquired by
    // mAcquireSharedCudaContextManagerFn. Replacing it only drops our own reference - a manager
    // an in-flight task still uses stays alive on that task's reference, which is why no
    // "retired managers" deferral list is needed here.
    ::physx::PxCudaContextManager* mPxCudaContextManager = nullptr;
    // True while mPxCudaContextManager came from the provider. Not a lifetime flag (we hold a
    // reference either way) - it gates whether we may refresh/replace the manager in place.
    bool mUsingSharedContextManager = false;
    ::physx::PxFoundation* mPxFoundation = nullptr;

    carb::tasking::ITasking* mTasking = nullptr;
    omni::physx::IPhysxFoundation* mPhysxFoundation = nullptr;
    AcquireSharedCudaContextManagerFn mAcquireSharedCudaContextManagerFn = nullptr;
    std::atomic_uint32_t mFinishedCookingTasksCount = { 0 };
    carb::tasking::MutexWrapper mGlobalMutex;
    omni::convexdecomposition::ConvexDecomposition mConvexDecomposition;
    uint32_t mLocalMeshCacheSizeMB = 1024;
    cookedcache::CookedCache* mCookedCache = nullptr;
};

ICookingComputeService* createCookingComputingService(::physx::PxFoundation& foundation,
                                                      AcquireSharedCudaContextManagerFn acquireSharedCudaContextManagerFn)
{
    return new CookingComputeService(foundation, acquireSharedCudaContextManagerFn);
}

void releaseCookingComputingService(ICookingComputeService* service)
{
    service->release();
}

bool ICookingComputeService::computeMeshKeyIfNeeded(PhysxCookingComputeResult& result,
                                                    PhysxCookingComputeRequest& request)
{
    CARB_PROFILE_ZONE(0, "ICookingComputeService::computeMeshKeyIfNeeded");
    // Every request is mesh-view mode now (eINPUT_MODE_FROM_PRIM_ID removed, REQ-COOK-SOURCE-001):
    // the caller reads geometry through IPhysicsSource before submitting, so there is no USD stage
    // for the service itself to resolve.
    result.requestSource = PhysxCookingComputeResult::eREQUEST_SOURCE_MESHVIEW;

    if (request.meshKey != omni::physx::usdparser::MeshKey())
    {
        result.meshKey = request.meshKey;
        return true;
    }

    // A deformable volume mesh cook has no source triangle mesh: its input is the tet geometry
    // in volumeMeshView, and the mesh processor explicitly skips triangulation for this data
    // type. Demanding a primMeshView here would reject every caller-supplied volume mesh.
    // Every other data type still requires one.
    const bool needsPrimMeshView = request.dataType != PhysxCookingDataType::eDEFORMABLE_VOLUME_MESH;
    if ((needsPrimMeshView && request.primMeshView.isEmpty())
        || ((request.dataType == PhysxCookingDataType::eDEFORMABLE_VOLUME_MESH) && request.volumeMeshView.isEmpty())
        || ((request.dataType == PhysxCookingDataType::eVOLUME_DEFORMABLE_BODY) && request.volumeDeformableBodyView.isEmpty())
        || ((request.dataType == PhysxCookingDataType::eSURFACE_DEFORMABLE_BODY) && request.surfaceDeformableBodyView.isEmpty()))
    {
        result.result = PhysxCookingResult::eERROR_INVALID_PRIM;
        request.onFinished(result);
        return false;
    }
    result.triangulationMaxMaterialIndex = CookingComputeService::getMaxMaterialIndex(request.primMeshView);

    result.meshKey = MeshKeyComputation::computeMeshKey(request.primMeshView);
    return true;
}

uint16_t ICookingComputeService::getMaxMaterialIndex(const PhysxCookingMeshView& meshView)
{
    // TODO: When breaking the ABI, we should allow passing number of used materials inside PhysxCookingMeshView
    if (meshView.faceMaterials.size() > 0)
    {
        // For now, just brute forcing to find max material index
        uint16_t maxMaterialIndex = 0;
        for (size_t idx = 0; idx < meshView.faceMaterials.size(); ++idx)
        {
            if (meshView.faceMaterials[idx] > maxMaterialIndex)
            {
                maxMaterialIndex = meshView.faceMaterials[idx];
            }
        }
        return maxMaterialIndex;
    }
    return 0;
}

} // namespace physx
} // namespace omni
