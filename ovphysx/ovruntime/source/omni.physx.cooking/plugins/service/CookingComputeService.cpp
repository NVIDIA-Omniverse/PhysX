// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include "UsdPCH.h"

#include "CookingComputeService.h"

#include <omni/physx/MeshKey.h>
#include <private/omni/physx/PhysxUsd.h> // ErrorCode
#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>
#include <carb/extras/Timer.h>

#include <common/utilities/Utilities.h> // asInt
#include <common/utilities/MemoryMacros.h>
#include <common/utilities/PhysXErrorCallback.h>

#include "CookingHashing.h"
#include "CookingTask.h"
#include "../utility/TriangulateUsdMeshPrim.h"

#include <omni/convexdecomposition/ConvexDecomposition.h>
#include <omni/physx/IPhysxFoundation.h>
#include "PhysXFoundation.h"
#include <carb/Defines.h>
#include <carb/profiler/Profile.h>

#define USE_PHYSX_GPU 1 // GPU Rigid Bodies

// Scoped mutex lock
using lock_guard = std::lock_guard<carb::tasking::MutexWrapper>;
using namespace PXR_NS;
using namespace physx;
namespace omni
{
namespace physx
{

using MeshHashSet = std::unordered_set<omni::physx::usdparser::MeshKey, omni::physx::usdparser::MeshKeyHash>;

// Queue of outstanding cooking tasks
using CookingTaskQueue = std::list<cookingtask::CookingTask*>;

// Map from prim path to its active CookingTask. Only one active task per UsdPrim; a task can also hold a single
// 'pending' task representing the next one to run once the current one completes.
using CookingTaskMap = std::unordered_map<PXR_NS::SdfPath, cookingtask::CookingTask*, PXR_NS::SdfPath::Hash>;

struct CookingComputeService : public ICookingComputeService
{
    explicit CookingComputeService(::physx::PxFoundation& foundation,
                                   SharedCudaContextManagerFn sharedCudaContextManagerFn)
        : mPxFoundation(&foundation), mSharedCudaContextManagerFn(sharedCudaContextManagerFn)
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

        // If any of these fails, you probably have been breaking the ABI
#if CARB_PLATFORM_WINDOWS
        static_assert(sizeof(PhysxCookingComputeRequest) == 368, "sizeof(PhysxCookingComputeRequest)");
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

        // If we did not create the context internally, do not attempt to release it
        if (mPxCudaContextManager && !mUsingSharedContextManager)
        {
            mPxCudaContextManager->release();
            mPxCudaContextManager = nullptr;
        }
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
        CookingStageAndPrim stageAndPrim;
        if (!skipMeshProcessing)
        {
            if (!computeMeshKeyIfNeeded(result, requestCopy, stageAndPrim))
                return nullptr;
        }
        else if (DataType == PhysxCookingDataType::eDEFORMABLE_VOLUME_MESH &&
                 requestCopy.dataInputMode == PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_ID &&
                 requestCopy.deformablePathInfo.bodyPrimId != 0)
        {
            if (!getStageAndPrim(result, requestCopy, stageAndPrim))
                return nullptr;
            if (!fillMeshView(result, requestCopy, stageAndPrim))
            {
                result.result = PhysxCookingResult::eERROR_INVALID_PRIM;
                requestCopy.onFinished(result);
                return nullptr;
            }
        }

        result.cookedDataCRC = deriveCRCFunction(result);

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
            const SdfPath meshPath = intToPath(result.request->primId);
            cookingtask::CookingTask* task = findOpenTask(asyncContext, meshPath, result.cookedDataCRC, true);
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
        return tryQueueingOrRunningTask(task->getResultObject(), requestCopy, task, stageAndPrim, asyncContext,
                                        skipMeshProcessing);
    }

    virtual bool lazyGetCudaContextManager(PhysxCookingDataType::Enum dataType,
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
            if (cudaContextManager == nullptr)
            {
                lock_guard globalLock(mGlobalMutex);
                omni::physx::PhysxFoundationDeviceOrdinal ordinal;
                mPhysxFoundation->getSingleCudaContextManagerOrdinal(ordinal);

                mUsingSharedContextManager = false;
                if (mSharedCudaContextManagerFn)
                {
                    mPxCudaContextManager = mSharedCudaContextManagerFn();
                    if (mPxCudaContextManager)
                    {
                        mUsingSharedContextManager = true;
                    }
                }

                if (mPhysxFoundation->createOrRefreshPxCudaContextManager(ordinal, mPxFoundation, mPxCudaContextManager, false))
                {
                    cudaContextManager = mPxCudaContextManager;
                }
                else
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
        }
        else
        {
            cudaContextManager = nullptr;
        }
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
        if (!lazyGetCudaContextManager(dataType, request, cudaContextManager, physicsGPU))
            return nullptr;
        return requestCookedData(
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
                task->setPxCudaAndGPUPointers(cudaContextManager, physicsGPU);
                return task;
            });
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
        CookingTaskMap::iterator found = asyncContext.mTaskMap.find(t->getPrimPath());
        if (found == asyncContext.mTaskMap.end())
        {
            asyncContext.mTaskMap[t->getPrimPath()] = t;
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
            }
            ct->addPendingTask(t);
            t->getCRC(crc);
            addCookedDataCRC(asyncContext, crc);
        }
    }

    void removeCookingTask(cookingtask::CookingTask* t)
    {
        AsyncContext& asyncContext = *reinterpret_cast<AsyncContext*>(t->getAsyncContext());

        CookingTaskMap::iterator found = asyncContext.mTaskMap.find(t->getPrimPath());
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
     * Find an open task with matching UsdPrim and MeshKey which hasn't been cancelled.
     *
     * @param usdPrim : UsdPrim identifying the task.
     * @param crc : MeshKey identifying the task.
     * @return : pending task if found, nullptr otherwise.
     */
    cookingtask::CookingTask* findOpenTask(AsyncContext* asyncContext,
                                           const PXR_NS::SdfPath& primPath,
                                           const omni::physx::usdparser::MeshKey& crc,
                                           bool crcOnly = false)
    {
        if (asyncContext != nullptr)
        {
            return findOpenTaskInContext(*asyncContext, primPath, crc, crcOnly);
        }
        else
        {
            for (auto& context : mAsyncContext)
            {
                cookingtask::CookingTask* task = findOpenTaskInContext(*context.second.get(), primPath, crc, crcOnly);
                if (task)
                {
                    return task;
                }
            }
        }
        return nullptr;
    }

    cookingtask::CookingTask* findOpenTaskInContext(AsyncContext& asyncContext,
                                                    const PXR_NS::SdfPath& primPath,
                                                    const omni::physx::usdparser::MeshKey& crc,
                                                    bool crcOnly)
    {

        CookingTaskMap::iterator found = asyncContext.mTaskMap.find(primPath);
        if (found == asyncContext.mTaskMap.end())
        {
            // Deformables and other "not 100% aligned" cooking approximation have
            // issues with matching just by CRC only for some reason
            if (crcOnly)
            {
                // Let's try to find another in flight task with the same CRC,
                // but having a different Usd Path
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

    static bool isRightHandedOrientation(const UsdGeomMesh& usdMesh)
    {
        PXR_NS::TfToken windingOrient = PXR_NS::UsdGeomTokens->rightHanded;
        usdMesh.GetOrientationAttr().Get(&windingOrient);
        return windingOrient != PXR_NS::UsdGeomTokens->leftHanded;
    }


    PhysxCookingOperationHandle tryQueueingOrRunningTask(PhysxCookingComputeResult& result,
                                                         PhysxCookingComputeRequest& request,
                                                         cookingtask::CookingTask* task,
                                                         CookingStageAndPrim& stageAndPrim,
                                                         AsyncContext* asyncContext,
                                                         bool skipMeshProcessing)
    {
        CARB_PROFILE_ZONE(0, "ICookingComputeService::tryQueueingOrRunningTask");
        if (!skipMeshProcessing)
        {
            switch (request.dataInputMode)
            {
            case PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_ID:
                if (request.primMeshView.isEmpty()) // If user supplied meshKey then meshView will be empty
                {
                    if (!getStageAndPrim(result, request, stageAndPrim) || !fillMeshView(result, request, stageAndPrim))
                    {
                        result.result = PhysxCookingResult::eERROR_INVALID_PRIM;
                        request.onFinished(result);
                        return nullptr;
                    }
                }
                break;
            case PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_MESH_VIEW: // primMeshView is already filled by
                                                                              // caller
                result.triangulationMaxMaterialIndex = CookingComputeService::getMaxMaterialIndex(request.primMeshView);
                break;
            }
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

    static bool fillUSDMeshView(const omni::physx::PhysxCookingComputeRequest& request,
                                omni::physx::PhysxCookingMeshView& meshView,
                                CookingStageAndPrim& stageAndPrim,
                                uint16_t& outMaxMaterialIndex)
    {
        CARB_PROFILE_ZONE(0, "ICookingComputeService::fillUSDMeshView");
        UsdGeomMesh usdMesh(stageAndPrim.usdPrim);
        UsdTimeCode time = request.primTimeCode;
        usdMesh.GetPointsAttr().Get(&stageAndPrim.rigidMesh.pointsValue);
        if (!stageAndPrim.rigidMesh.pointsValue.size())
        {
            time = UsdTimeCode::EarliestTime();
            usdMesh.GetPointsAttr().Get(&stageAndPrim.rigidMesh.pointsValue, time);
            CARB_LOG_ERROR(
                "omni.physx.cooking does not support time sampled points. Ignoring all but first sample. Mesh path: %s",
                usdMesh.GetPrim().GetPrimPath().GetText());
        }
        usdMesh.GetFaceVertexIndicesAttr().Get(&stageAndPrim.rigidMesh.indicesValue, time);
        usdMesh.GetFaceVertexCountsAttr().Get(&stageAndPrim.rigidMesh.facesValue, time);
        usdMesh.GetHoleIndicesAttr().Get(&stageAndPrim.rigidMesh.holesValue, time);
        uint32_t pointCount = uint32_t(stageAndPrim.rigidMesh.pointsValue.size());
        uint32_t indicesCount = uint32_t(stageAndPrim.rigidMesh.indicesValue.size());
        uint32_t facesCount = uint32_t(stageAndPrim.rigidMesh.facesValue.size());
        if (pointCount && indicesCount && facesCount)
        {
            const carb::Float3* points = reinterpret_cast<const carb::Float3*>(&stageAndPrim.rigidMesh.pointsValue[0]);
            const int32_t* indices = stageAndPrim.rigidMesh.indicesValue.data();
            const int32_t* faces = stageAndPrim.rigidMesh.facesValue.data();
            meshView.points = { points, pointCount };
            meshView.indices = { indices, indicesCount };
            meshView.faces = { faces, facesCount };
            if (!stageAndPrim.rigidMesh.holesValue.empty())
            {
                const int32_t* holes = stageAndPrim.rigidMesh.holesValue.data();
                uint32_t holesCount = uint32_t(stageAndPrim.rigidMesh.holesValue.size());
                meshView.holeIndices = { holes, holesCount };
            }
            stageAndPrim.rigidMesh.faceMaterials.resize(facesCount);
            omni::span<uint16_t> faceMaterials = { stageAndPrim.rigidMesh.faceMaterials.data(), facesCount };
            if (triangulateusd::TriangulateUSDPrim::fillFaceMaterials(
                    stageAndPrim.usdPrim, faceMaterials, time, outMaxMaterialIndex))
            {
                meshView.faceMaterials = faceMaterials;
            }
            else
            {
                stageAndPrim.rigidMesh.faceMaterials.clear();
            }
            TfToken orientation;
            usdMesh.GetOrientationAttr().Get(&orientation);
            meshView.rightHandedOrientation = (orientation == UsdGeomTokens->rightHanded);
            return true;
        }
        return false;
    }

    static UsdAttribute getPosePointsAttr(UsdPrim posePrim, const TfType& poseType, TfToken instanceName)
    {
        if (instanceName.IsEmpty())
            return UsdAttribute();

        if (!posePrim.HasAPI(poseType, instanceName))
        {
            CARB_LOG_ERROR("Expected UsdPhysicsDeformablePoseAPI instance %s on %s, but not found.",
                instanceName.GetText(), posePrim.GetPath().GetText());
            return UsdAttribute();
        }

        TfToken attrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
            OmniUsdPhysicsDeformableSchemaTokens->deformablePose_MultipleApplyTemplate_OmniphysicsPoints, instanceName);
        return posePrim.GetAttribute(attrName);
    }

    static UsdAttribute getPosePointsOrPointsAttr(UsdPrim posePrim, const TfType& poseType, TfToken instanceName)
    {
        if (!instanceName.IsEmpty())
        {
            return getPosePointsAttr(posePrim, poseType, instanceName);
        }
        return UsdGeomPointBased(posePrim).GetPointsAttr();
    }

    static UsdAttribute getPosePurposesAttr(UsdPrim posePrim, TfToken instanceName)
    {
        TfToken attrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
            OmniUsdPhysicsDeformableSchemaTokens->deformablePose_MultipleApplyTemplate_OmniphysicsPurposes, instanceName);
        return posePrim.GetAttribute(attrName);
    }

    static TfToken getPoseNameFromPurpose(const UsdPrim prim, const TfToken posePurposeToken)
    {
        TfTokenVector allAPIs = prim.GetAppliedSchemas();

        TfType poseType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformablePoseAPI);
        TfToken poseTypeName = UsdSchemaRegistry::GetAPISchemaTypeName(poseType);

        for (const auto& api : allAPIs)
        {
            std::pair<TfToken, TfToken> typeNameAndInstance = UsdSchemaRegistry::GetTypeNameAndInstance(api);
            if (typeNameAndInstance.first == poseTypeName)
            {
                VtArray<TfToken> candTokens;
                getPosePurposesAttr(prim, typeNameAndInstance.second).Get(&candTokens);
                for (const TfToken candToken : candTokens)
                {
                    if (candToken == posePurposeToken)
                    {
                        return typeNameAndInstance.second;
                    }
                }
            }
        }

        return TfToken();
    }

    static bool fillUSDVolumeDeformableBodyMeshView(const omni::physx::PhysxCookingComputeRequest& request,
        omni::physx::PhysxCookingDeformableBodyView& view,
        CookingStageAndPrim& stageAndPrim)
    {
        CARB_PROFILE_ZONE(0, "ICookingComputeService::fillUSDVolumeDeformableBodyMeshView");

        if (!stageAndPrim.stage)
            return false;

        UsdGeomMesh srcMesh(stageAndPrim.usdPrim);

        TfType simType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsVolumeDeformableSimAPI);
        const SdfPath simMeshPath = intToPath(request.deformablePathInfo.simMeshPrimId);
        UsdPrim simMeshPrim = stageAndPrim.stage->GetPrimAtPath(simMeshPath);
        if (!simMeshPrim || !simMeshPrim.IsA<PXR_NS::UsdGeomTetMesh>() || !simMeshPrim.HasAPI(simType))
        {
            return false;
        }

        VtArray<GfVec3f>& srcPointsInSim = stageAndPrim.volumeDeformableBodyMesh.srcPointsInSim;
        TfType poseType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformablePoseAPI);
        TfToken srcMeshBindPoseToken = getPoseNameFromPurpose(srcMesh.GetPrim(), OmniUsdPhysicsDeformableSchemaTokens->bindPose);
        UsdAttribute simBindPointsAttr = getPosePointsOrPointsAttr(srcMesh.GetPrim(), poseType, srcMeshBindPoseToken);
        if (simBindPointsAttr)
        {
            simBindPointsAttr.Get(&srcPointsInSim);
        }

        // Transform skin points to sim mesh space
        PXR_NS::GfMatrix4d srcToWorld = srcMesh.ComputeLocalToWorldTransform(PXR_NS::UsdTimeCode::Default());
        PXR_NS::GfMatrix4d simToWorld =
            PXR_NS::UsdGeomXformable(simMeshPrim).ComputeLocalToWorldTransform(PXR_NS::UsdTimeCode::Default());
        PXR_NS::GfMatrix4d worldToSim = simToWorld.GetInverse();
        PXR_NS::GfMatrix4d srcToSim = srcToWorld * worldToSim;
        for (size_t i = 0; i < srcPointsInSim.size(); ++i)
        {
            PXR_NS::GfVec3f& srcPoint = srcPointsInSim[i];
            srcPoint = PXR_NS::GfVec3f(srcToSim.Transform(srcPoint));
        }

        uint32_t srcPointCount = uint32_t(srcPointsInSim.size());
        if (srcPointCount)
        {
            const carb::Float3* srcPoints = reinterpret_cast<const carb::Float3*>(&srcPointsInSim[0]);
            view.srcPointsInSim = { srcPoints, srcPointCount };
            return true;
        }   

        return false;
    }

    static bool fillUSDDeformableVolumeMeshView(const omni::physx::PhysxCookingComputeRequest& request,
        omni::physx::PhysxCookingDeformableVolumeMeshView& view,
        CookingStageAndPrim& stageAndPrim)
    {
        CARB_PROFILE_ZONE(0, "ICookingComputeService::fillUSDDeformableVolumeMeshView");

        if (!stageAndPrim.stage)
            return false;

        const SdfPath bodyPrimPath = intToPath(request.deformablePathInfo.bodyPrimId);
        const SdfPath simMeshPath = intToPath(request.deformablePathInfo.simMeshPrimId);
        const SdfPath collMeshPath = intToPath(request.deformablePathInfo.collMeshPrimId);
        UsdPrim bodyPrim = stageAndPrim.stage->GetPrimAtPath(bodyPrimPath);
        UsdPrim simMeshPrim = stageAndPrim.stage->GetPrimAtPath(simMeshPath);
        UsdPrim collMeshPrim = stageAndPrim.stage->GetPrimAtPath(collMeshPath);
        if (!bodyPrim || !simMeshPrim || !collMeshPrim)
        {
            return false;
        }

        GfMatrix4d simToWorld = UsdGeomXformable(simMeshPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default());
        GfMatrix4d worldToSim = simToWorld.GetInverse();

        VtArray<GfVec3f>& simPoints = stageAndPrim.deformableVolumeMesh.simPoints;
        VtArray<GfVec4i>& simIndices = stageAndPrim.deformableVolumeMesh.simIndices;
        VtArray<GfVec3f>& simBindPoints = stageAndPrim.deformableVolumeMesh.simBindPoints;
        VtArray<GfVec3f>& collBindPointsInSim = stageAndPrim.deformableVolumeMesh.collBindPointsInSim;
        VtArray<GfVec4i>& collIndices = stageAndPrim.deformableVolumeMesh.collIndices;
        VtArray<GfVec3i>& collSurfaceIndices = stageAndPrim.deformableVolumeMesh.collSurfaceIndices;

        // read simulation mesh rest shape for cooking (until the SDK supports a proper rest shape)
        // need to make sure the rest shape is compatible with the tetmesh topology
        {
            VtArray<GfVec3f> simPointsTmp;
            VtArray<GfVec4i> simTetVertexIndices;
            VtArray<GfVec3f> simRestShapePoints;
            VtArray<GfVec4i> simRestTetVtxIndices;

            UsdGeomPointBased(simMeshPrim).GetPointsAttr().Get(&simPointsTmp);
            UsdGeomTetMesh(simMeshPrim).GetTetVertexIndicesAttr().Get(&simTetVertexIndices);
            simMeshPrim.GetAttribute(OmniUsdPhysicsDeformableSchemaTokens->omniphysicsRestShapePoints).Get(&simRestShapePoints);
            simMeshPrim.GetAttribute(OmniUsdPhysicsDeformableSchemaTokens->omniphysicsRestTetVtxIndices).Get(&simRestTetVtxIndices);

            bool mismatch = simPointsTmp.size() != simRestShapePoints.size() ||
                simTetVertexIndices.size() != simRestTetVtxIndices.size() ||
                std::memcmp(simTetVertexIndices.data(), simRestTetVtxIndices.data(),
                    sizeof(GfVec4i) * simTetVertexIndices.size()) != 0;

            if (mismatch)
            {
                CARB_LOG_WARN(
                    "ICookingComputeService::fillUSDDeformableVolumeMeshView failed, UsdGeomTetMesh not compatible with rest attributes in DeformableVolumeSimAPI, %s",
                    bodyPrim.GetPath().GetText());
                return false;
            }

            simPoints.swap(simRestShapePoints);
            simIndices.swap(simTetVertexIndices);
        }
        
        uint32_t simPointsCount = uint32_t(simPoints.size());
        uint32_t simIndicesCount = uint32_t(simIndices.size());
        if (!simPointsCount || !simIndicesCount)
        {
            return false;
        }

        const carb::Float3* simPointsPtr = reinterpret_cast<const carb::Float3*>(&stageAndPrim.deformableVolumeMesh.simPoints[0]);
        view.simPoints = { simPointsPtr, simPointsCount };      

        const carb::Int4* simIndicesPtr = reinterpret_cast<const carb::Int4*>(&stageAndPrim.deformableVolumeMesh.simIndices[0]);
        view.simIndices = { simIndicesPtr, simIndicesCount };

        if (collMeshPrim != simMeshPrim)
        {
            // Need to construct embedding for collision mesh:
            // (simulation bind pose, sim mesh indices == rest shape indices, collision bind pose) -> embedding
            // (embedding, sim rest shape points, rest shape topo> -> collision rest shape
            VtArray<GfVec3f> simMeshBindPoints;
            VtArray<GfVec3f> collMeshBindPoints;
            {
                TfType poseType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformablePoseAPI);
                TfToken simMeshBindPoseToken = getPoseNameFromPurpose(simMeshPrim, OmniUsdPhysicsDeformableSchemaTokens->bindPose);
                TfToken collisionMeshBindPoseToken = getPoseNameFromPurpose(collMeshPrim, OmniUsdPhysicsDeformableSchemaTokens->bindPose);
                UsdAttribute simBindPointsAttr = getPosePointsOrPointsAttr(simMeshPrim, poseType, simMeshBindPoseToken);             
                UsdAttribute collBindPointsAttr = getPosePointsOrPointsAttr(collMeshPrim, poseType, collisionMeshBindPoseToken);
                if (!simBindPointsAttr || !collBindPointsAttr)
                {
                    return false;
                }
                simBindPointsAttr.Get(&simMeshBindPoints);
                collBindPointsAttr.Get(&collMeshBindPoints);
            }

            // Transform collision points to sim space
            GfMatrix4d collToWorld = UsdGeomImageable(collMeshPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default());
            GfMatrix4d collToSim = collToWorld * worldToSim;
            for (size_t i = 0; i < collMeshBindPoints.size(); ++i)
            {
                collMeshBindPoints[i] = PXR_NS::GfVec3f(collToSim.Transform(collMeshBindPoints[i]));
            }
            collBindPointsInSim.swap(collMeshBindPoints);
            simBindPoints.swap(simMeshBindPoints);
            UsdGeomTetMesh(collMeshPrim).GetTetVertexIndicesAttr().Get(&collIndices);

            uint32_t simBindPointsCount = uint32_t(simBindPoints.size());
            uint32_t collBindPointsInSimCount = uint32_t(collBindPointsInSim.size());
            uint32_t collIndicesCount = uint32_t(collIndices.size());
            if (!simBindPointsCount || !collBindPointsInSimCount || !collIndicesCount)
            {
                return false;
            }

            if (simBindPoints.size() != simPoints.size())
            {
                CARB_LOG_ERROR("ICookingComputeService::fillUSDDeformableVolumeMeshView failed, sim mesh bind pose points incompatible with points: %s", bodyPrim.GetPath().GetText());
                return false;
            }

            const carb::Float3* simBindPointsPtr = reinterpret_cast<const carb::Float3*>(&stageAndPrim.deformableVolumeMesh.simBindPoints[0]);
            view.simBindPoints = { simBindPointsPtr, simBindPointsCount };

            const carb::Float3* collBindPointsInSimPtr = reinterpret_cast<const carb::Float3*>(&stageAndPrim.deformableVolumeMesh.collBindPointsInSim[0]);
            view.collBindPointsInSim = { collBindPointsInSimPtr, collBindPointsInSimCount };

            const carb::Int4* collIndicesPtr = reinterpret_cast<const carb::Int4*>(&stageAndPrim.deformableVolumeMesh.collIndices[0]);
            view.collIndices = { collIndicesPtr, collIndicesCount };
        }
        else
        {
            view.simBindPoints = {};
            view.collBindPointsInSim = {};
            view.collIndices = {};
        }

        // read surface face vertices be from the collsion mesh, even if sim and coll mesh alias
        PXR_NS::UsdGeomTetMesh(collMeshPrim).GetSurfaceFaceVertexIndicesAttr().Get(&collSurfaceIndices);
        uint32_t collSurfaceIndicesCount = uint32_t(collSurfaceIndices.size());
        if (!collSurfaceIndicesCount)
        {
            CARB_LOG_WARN("ICookingComputeService::fillUSDDeformableVolumeMeshView failed, collision mesh UsdGeomTetMesh needs to have "
                "surfaceFaceVertexIndices set, %s.", collMeshPrim.GetPath().GetText());
            return false;
        }
        const carb::Int3* collSurfaceIndicesPtr = reinterpret_cast<const carb::Int3*>(&stageAndPrim.deformableVolumeMesh.collSurfaceIndices[0]);
        view.collSurfaceIndices = { collSurfaceIndicesPtr, collSurfaceIndicesCount };

        return true;
    }

    static bool fillUSDSurfaceDeformableBodyMeshView(const omni::physx::PhysxCookingComputeRequest& request,
        omni::physx::PhysxCookingDeformableBodyView& view,
        CookingStageAndPrim& stageAndPrim)
    {
        CARB_PROFILE_ZONE(0, "ICookingComputeService::fillUSDSurfaceDeformableBodyMeshView");

        if (!stageAndPrim.stage)
            return false;

        UsdGeomMesh srcMesh(stageAndPrim.usdPrim);

        const SdfPath simMeshPath = intToPath(request.deformablePathInfo.simMeshPrimId);
        UsdPrim simMeshPrim = stageAndPrim.stage->GetPrimAtPath(simMeshPath);
        GfMatrix4d simToWorld = UsdGeomXformable(simMeshPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default());
        GfMatrix4d worldToSim = simToWorld.GetInverse();

        VtArray<GfVec3f>& srcPointsInSim = stageAndPrim.surfaceDeformableBodyMesh.srcPointsInSim;
        TfToken srcMeshBindPoseToken = getPoseNameFromPurpose(srcMesh.GetPrim(), OmniUsdPhysicsDeformableSchemaTokens->bindPose);
        TfType dpType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformablePoseAPI);
        bool hasBindPoseAPI = !srcMeshBindPoseToken.IsEmpty() && srcMesh.GetPrim().HasAPI(dpType, srcMeshBindPoseToken);
        if (hasBindPoseAPI)
        {
            TfToken pointsAttrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
                OmniUsdPhysicsDeformableSchemaTokens->deformablePose_MultipleApplyTemplate_OmniphysicsPoints, srcMeshBindPoseToken);
            srcMesh.GetPrim().GetAttribute(pointsAttrName).Get(&srcPointsInSim);
        }
        else
        {
            srcMesh.GetPointsAttr().Get(&srcPointsInSim);
        }

        // Transform src points to sim mesh space
        GfMatrix4d srcToWorld = srcMesh.ComputeLocalToWorldTransform(UsdTimeCode::Default());
        GfMatrix4d srcToSim = srcToWorld * worldToSim;
        for (size_t i = 0; i < srcPointsInSim.size(); ++i)
        {
            GfVec3f& srcPoint = srcPointsInSim[i];
            srcPoint = GfVec3f(srcToSim.Transform(srcPoint));
        }

        uint32_t srcPointCount = uint32_t(srcPointsInSim.size());
        if (srcPointCount)
        {
            const carb::Float3* srcPoints = reinterpret_cast<const carb::Float3*>(&srcPointsInSim[0]);
            view.srcPointsInSim = { srcPoints, srcPointCount };
            return true;
        }

        return false;
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
    ::physx::PxCudaContextManager* mPxCudaContextManager = nullptr;
    bool mUsingSharedContextManager = false;
    ::physx::PxFoundation* mPxFoundation = nullptr;

    carb::tasking::ITasking* mTasking = nullptr;
    omni::physx::IPhysxFoundation* mPhysxFoundation = nullptr;
    SharedCudaContextManagerFn mSharedCudaContextManagerFn = nullptr;
    std::atomic_uint32_t mFinishedCookingTasksCount = { 0 };
    carb::tasking::MutexWrapper mGlobalMutex;
    omni::convexdecomposition::ConvexDecomposition mConvexDecomposition;
    uint32_t mLocalMeshCacheSizeMB = 1024;
    cookedcache::CookedCache* mCookedCache = nullptr;
};

ICookingComputeService* createCookingComputingService(::physx::PxFoundation& foundation,
                                                      SharedCudaContextManagerFn sharedCudaContextManagerFn)
{
    return new CookingComputeService(foundation, sharedCudaContextManagerFn);
}

void releaseCookingComputingService(ICookingComputeService* service)
{
    service->release();
}

bool ICookingComputeService::getStageAndPrim(PhysxCookingComputeResult& result,
                                             PhysxCookingComputeRequest& request,
                                             CookingStageAndPrim& stageAndPrim)
{
    CARB_PROFILE_ZONE(0, "ICookingComputeService::getStageAndPrim");
    if (!stageAndPrim.stage)
    { // I hope this is safe to do even on a non-main thread, as well as PXR_NS::UsdGeomGetStageMetersPerUnit
        stageAndPrim.stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(request.primStageId)));
    }
    if (!stageAndPrim.stage)
    {
        CARB_LOG_ERROR("PhysX could not find USD stage");
        result.result = PhysxCookingResult::eERROR_INVALID_STAGE;
        request.onFinished(result);
        return false;
    }

    const SdfPath meshPath = intToPath(request.primId);
    const UsdPrim usdPrimSource = stageAndPrim.stage->GetPrimAtPath(meshPath);

    if (!stageAndPrim.usdPrim.IsValid()) // resolve the input mesh prim from USD
    {
        if (request.primId == 0)
            return true; // Workaround for requests without input source
        stageAndPrim.usdPrim = usdPrimSource;
        if (!stageAndPrim.usdPrim || !stageAndPrim.usdPrim.IsValid() || !stageAndPrim.usdPrim.IsA<UsdGeomMesh>())
        {
            CARB_LOG_ERROR("PhysX could not find USD prim or prim is not UsdGeomMesh!");
            result.result = PhysxCookingResult::eERROR_INVALID_PRIM;
            request.onFinished(result);
            return false;
        }
        const char* primPathText = stageAndPrim.usdPrim.GetPrimPath().GetText();
        request.primMeshText = { primPathText, strlen(primPathText) };
        result.requestSource = request.dataInputMode == PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_ID ?
                                   PhysxCookingComputeResult::eREQUEST_SOURCE_USD :
                                   PhysxCookingComputeResult::eREQUEST_SOURCE_MESHVIEW;
    }
    return true;
}

bool ICookingComputeService::fillMeshView(omni::physx::PhysxCookingComputeResult& result,
                                          omni::physx::PhysxCookingComputeRequest& request,
                                          CookingStageAndPrim& stageAndPrim)
{
    bool ret1 = true;
    if (request.primId != 0)
    {
        ret1 = CookingComputeService::fillUSDMeshView(
            request, request.primMeshView, stageAndPrim, result.triangulationMaxMaterialIndex);
    }

    bool ret2 = true;
    if (request.dataType == PhysxCookingDataType::eVOLUME_DEFORMABLE_BODY)
    {
        ret2 = CookingComputeService::fillUSDVolumeDeformableBodyMeshView(request, request.volumeDeformableBodyView, stageAndPrim);
    }
    else if (request.dataType == PhysxCookingDataType::eDEFORMABLE_VOLUME_MESH)
    {
        ret2 = CookingComputeService::fillUSDDeformableVolumeMeshView(request, request.volumeMeshView, stageAndPrim);
    }
    else if (request.dataType == PhysxCookingDataType::eSURFACE_DEFORMABLE_BODY)
    {
        ret2 = CookingComputeService::fillUSDSurfaceDeformableBodyMeshView(request, request.surfaceDeformableBodyView, stageAndPrim);
    }

    return ret1 && ret2;
}

bool ICookingComputeService::computeMeshKeyIfNeeded(PhysxCookingComputeResult& result,
                                                    PhysxCookingComputeRequest& request,
                                                    CookingStageAndPrim& stageAndPrim)
{
    CARB_PROFILE_ZONE(0, "ICookingComputeService::computeMeshKeyIfNeeded");
    switch (request.dataInputMode)
    {
    case PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_ID:
        if (!getStageAndPrim(result, request, stageAndPrim))
        {
            result.result = PhysxCookingResult::eERROR_INVALID_PRIM;
            request.onFinished(result);
            return false;
        }
        request.primMeshMetersPerUnit = PXR_NS::UsdGeomGetStageMetersPerUnit(stageAndPrim.stage);
        if (stageAndPrim.usdPrim) // Workaround for requests without input source
        {
            request.primMeshView.rightHandedOrientation =
                CookingComputeService::isRightHandedOrientation(UsdGeomMesh(stageAndPrim.usdPrim));
        }
        break;
    case PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_MESH_VIEW:
        result.requestSource = PhysxCookingComputeResult::eREQUEST_SOURCE_MESHVIEW;
        break;
    }

    if (request.meshKey != omni::physx::usdparser::MeshKey())
    {
        result.meshKey = request.meshKey;
        return true;
    }

    switch (request.dataInputMode)
    {
    case PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_ID:
        if (!fillMeshView(result, request, stageAndPrim))
        {
            result.result = PhysxCookingResult::eERROR_INVALID_PRIM;
            request.onFinished(result);
            return false;
        }
        break;
    case PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_MESH_VIEW:
        if (request.primMeshView.isEmpty()
            || ((request.dataType == PhysxCookingDataType::eDEFORMABLE_VOLUME_MESH) && request.volumeMeshView.isEmpty())
            || ((request.dataType == PhysxCookingDataType::eVOLUME_DEFORMABLE_BODY) && request.volumeDeformableBodyView.isEmpty())
            || ((request.dataType == PhysxCookingDataType::eSURFACE_DEFORMABLE_BODY) && request.surfaceDeformableBodyView.isEmpty()))
        {
            result.result = PhysxCookingResult::eERROR_INVALID_PRIM;
            request.onFinished(result);
            return false;
        }
        result.triangulationMaxMaterialIndex = CookingComputeService::getMaxMaterialIndex(request.primMeshView);
        break;
    default:
        request.onFinished(result);
        return false;
    }
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
