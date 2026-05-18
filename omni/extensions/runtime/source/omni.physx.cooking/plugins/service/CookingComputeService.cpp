// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "CookingComputeService.h"

#include <omni/physx/MeshKey.h>
#include <private/omni/physx/PhysxUsd.h> // ErrorCode
#include <private/omni/physx/IPhysxPrivate.h>
#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>
#include <carb/extras/Timer.h>
#include <omni/fabric/SimStageWithHistory.h>
#include <omni/fabric/AttrNameAndType.h>
#include <usdrt/scenegraph/usd/usd/stage.h>
#include <omni/fabric/usd/PathConversion.h>

#include <common/utilities/Utilities.h> // asInt
#include <common/utilities/MemoryMacros.h>
#include <common/utilities/PhysXErrorCallback.h>

#include "CookingHashing.h"
#include "CookingTask.h"
#include "../utility/TriangulateUsdMeshPrim.h"

#include <omni/physx/IPhysxFoundation.h>
#include <carb/Defines.h>
#include <carb/profiler/Profile.h>

#define USE_PHYSX_GPU 1 // GPU Rigid Bodies

// Scoped mutex lock
using lock_guard = std::lock_guard<carb::tasking::MutexWrapper>;
using namespace pxr;
using namespace physx;
namespace omni
{
namespace physx
{

using MeshHashSet = std::unordered_set<omni::physx::usdparser::MeshKey, omni::physx::usdparser::MeshKeyHash>;

// Declare the 'CookingTaskQueue' to refer to a queue of outstanding cooking tasks
using CookingTaskQueue = std::list<cookingtask::CookingTask*>;

// Declare an unordered map stored by primitive name and associated with the current active CookingTask
// We only have one active Cooking Task per UsdPrim. That active CookingTask can also represent a single
// 'pending cooking task' which represents the new task to execute when the current active task is completed
using CookingTaskMap = std::unordered_map<pxr::SdfPath, cookingtask::CookingTask*, pxr::SdfPath::Hash>;

struct FabricTokens
{
    omni::fabric::Token PointsAttr = omni::fabric::Token("points");
    omni::fabric::Token IndicesAttr = omni::fabric::Token("faceVertexIndices");
    omni::fabric::Token FacesAttr = omni::fabric::Token("faceVertexCounts");
    omni::fabric::Token HolesAttr = omni::fabric::Token("holeIndices");
    omni::fabric::Token OrientationAttr = omni::fabric::Token("orientation");
    omni::fabric::Token LeftHanded = omni::fabric::Token("leftHanded");
    static FabricTokens& get()
    {
        static FabricTokens globalTokens;
        return globalTokens;
    }
};

struct CookingComputeService : public ICookingComputeService
{
    CookingComputeService()
    {
        // Make sure that foundation is acquired first so that its carbOnPluginStartup loading PhysXGPU_64.dll is called
        mPhysxFoundation = carb::getCachedInterface<omni::physx::IPhysxFoundation>();
        mTasking = carb::getCachedInterface<carb::tasking::ITasking>();
        mPxFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, mPxAllocator, mErrorCallback);

        // List of API improvements that will be applied when the next ABI Break will become necessary:
        // - triangulationMaxMaterialIndex --> to be moved inside PhysxCookingMeshTriangulationView
        // - isSynchronousResult and rightHandedOrientation need to become Flags
        // - Replace PhysxCookedDataSpan with omni::span
        // - PhysxCookingAsyncContextParameters::contextName must be changed from omni::span to omni::string_view
        // - Allow passing maxMaterialIndex inside PhysxCookingMeshView when doing input from
        // eINPUT_MODE_FROM_PRIM_MESH_VIEW

        // If any of these fails, you probably have been breaking the ABI
#if CARB_PLATFORM_WINDOWS
        static_assert(sizeof(PhysxCookingComputeRequest) == 216, "sizeof(PhysxCookingComputeRequest)");
        static_assert(sizeof(PhysxCookingComputeResult) == 152, "sizeof(PhysxCookingComputeResult)");
#else
        static_assert(sizeof(PhysxCookingComputeRequest) == 224, "sizeof(PhysxCookingComputeRequest)");
        static_assert(sizeof(PhysxCookingComputeResult) == 152, "sizeof(PhysxCookingComputeResult)");
#endif
        static_assert(offsetof(PhysxCookingComputeResult, resultSource) == 136, "ABI Broken");
    }

    ~CookingComputeService(void)
    {
        // First, for any active cooking tasks, we issue a 'cancel' request since the results will not
        // be processed now that the cookingdataasync instance is being released
        // We now must delete all of the outstanding cooking tasks. The destructor for each will block until
        // the background thread (if active) has completed.
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
        mPxFoundation->release();
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
        return getActiveTaskCount(context); // We return the total number of current active cooking tasks
    }

    void dispatchAsyncTasks(PhysxCookingAsyncContext context)
    {
        // We grab a mutex lock to be thread safe
        lock_guard globalLock(mGlobalMutex);
        AsyncContext& asyncContext = *reinterpret_cast<AsyncContext*>(context);
        // Now we check on the status of outstanding cooking tasks
        if (!asyncContext.mTasks.empty())
        {
            carb::extras::Timer timer;
            // We start timing how much time we spend here so that we can exit
            // early if processing cooking results is taking too long, to avoid
            // causing excessive hangs or stalls in the editor
            timer.start();

            uint32_t count = 0; // A counter of how many tasks we processed

            // We begin iterating on the queue of active cooking tasks
            CookingTaskQueue::iterator i = asyncContext.mTasks.begin();

            // Iterate through the first tasks in the list and see if any of them are done or need to be started

            // This will contain the set of tasks which were pending and need to be added
            // to the queue
            std::vector<cookingtask::CookingTask*> newTasks;

            // Iterate while there are still cooking tasks and we have not
            // processed more than 'MAX_ACTIVE_TASK_COUNT'
            while (i != asyncContext.mTasks.end() && count < MAX_ACTIVE_TASK_COUNT)
            {
                // Get the CookingTask pointer
                cookingtask::CookingTask* t = (*i);
                if (t->getAsyncContext() != &asyncContext) // If the task doesn't belong to current context, skip it
                {
                    i++;
                    continue;
                }
                // Call the 'pump' method which will return true if this task is 'finished'.
                // The 'pump' method will also start the background task running if needed.
                bool finished = t->pump(mTasking);
                // If this cooking task is complete...
                if (finished)
                {
                    i = asyncContext.mTasks.erase(i); // We remove the ask from the queue
                    // see if this prim had a pending cooking task still to be processed
                    cookingtask::CookingTask* nt = t->getPendingTask();
                    if (nt)
                    {
                        newTasks.push_back(nt); // If so, save it so the new cooking task can be re-scheduled at the
                                                // end.
                    }
                    // Now we finalize the results in the main thread and delete the cooking task instance
                    removeCookingTask(t);

                    // If we have spent more than 16 milliseconds finalizing cooking tasks, exit now
                    auto dtime = timer.getElapsedTime<int64_t>();
                    if (dtime >= 16)
                    {
                        break;
                    }
                }
                else
                {
                    // If the task has not yet finished, we simply continue to the next one
                    i++;
                    count++;
                }
            }
            // If we needed to reschedule pending cooking tasks we do so here.
            for (auto& i : newTasks)
            {
                addCookingTask(i, asyncContext); // add it
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

                omni::physx::IPhysxPrivate* pxPrivate = carb::getCachedInterface<omni::physx::IPhysxPrivate>();
                mUsingSharedContextManager = false;
                if (pxPrivate)
                {
                    mPxCudaContextManager = pxPrivate->getCudaContextManager();
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
                    convexDecompositionCookingParams, result);
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
                return cookingtask::createSphereFillCookingTask(sphereFillCookingParams, result);
            });
    }

    virtual PhysxCookingOperationHandle requestSoftBodyMeshCookedDataDeprecated(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::SoftBodyMeshCookingParamsDeprecated& params) override
    {
        const bool skipMeshProcessing = true;
        return requestCookedData(
            PhysxCookingDataType::eSOFT_BODY_DEPRECATED, context, request,
            [&](const PhysxCookingComputeResult& result) {
                return MeshCRCComputation::computeSoftBodyMeshCRCDeprecated(params);
            },
            [&](PhysxCookingComputeResult& result) {
                return cookingtask::createSoftBodyMeshCookingTaskDeprecated(params, result);
            },
            skipMeshProcessing);
    }


    virtual PhysxCookingOperationHandle requestDeformableBodyTetMeshCookedDataDeprecated(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::DeformableBodyTetMeshCookingParamsDeprecated& params)
    {
        return requestCookedData(
            PhysxCookingDataType::eDEFORMABLE_TETRAHEDRAL_MESH_DEPRECATED, context, request,
            [&](const PhysxCookingComputeResult& result) {
                return MeshCRCComputation::deriveDeformableBodyTetMeshCRCDeprecated(result.meshKey, params);
            },
            [&](PhysxCookingComputeResult& result) {
                return cookingtask::createDeformableBodyTetMeshCookingTaskDeprecated(params, result);
            });
    }

    virtual PhysxCookingOperationHandle requestParticleClothMeshCookedDataDeprecated(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::ParticleClothMeshCookingParamsDeprecated& params)
    {
        return requestCookedData(
            PhysxCookingDataType::ePARTICLE_CLOTH_DEPRECATED, context, request,
            [&](const PhysxCookingComputeResult& result) {
                return MeshCRCComputation::deriveParticleClothMeshCRCDeprecated(result.meshKey, params);
            },
            [&](PhysxCookingComputeResult& result) {
                return cookingtask::createParticleClothMeshCookingTaskDeprecated(params, result);
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
            // Register the fact that we have a task running for this CRC
            omni::physx::usdparser::MeshKey crc;
            t->getCRC(crc);
            addCookedDataCRC(asyncContext, crc);
        }
        else
        {
            cookingtask::CookingTask* ct = (*found).second;
            ct->cancel(false); // cancel this one because this new one takes precedence
            // Remove this CRC since we are canceling the task
            omni::physx::usdparser::MeshKey crc;
            ct->getCRC(crc);
            removeCookedDataCRC(asyncContext, crc); // since we are canceling this task, remove the CRC
            // See if there was already a previous pending task (which we will overwrite)
            // If there was already a previous pending task, we unregister his CRC as well.
            cookingtask::CookingTask* p = ct->getPendingTask();
            if (p)
            {
                p->getCRC(crc);
                removeCookedDataCRC(asyncContext, crc); // since we are canceling this task, remove the CRC
            }
            // Register the new pending task CRC
            ct->addPendingTask(t); // add it as the current active pending task..
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
            removeCookedDataCRC(asyncContext, crc); // since we are canceling this task, remove the CRC
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
                                           const pxr::SdfPath& primPath,
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
                                                    const pxr::SdfPath& primPath,
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
        pxr::TfToken windingOrient = pxr::UsdGeomTokens->rightHanded;
        usdMesh.GetOrientationAttr().Get(&windingOrient);
        return windingOrient != pxr::UsdGeomTokens->leftHanded;
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
                addCookingTask(task, *asyncContext); // Add this cooking task to the active cooking task queue
            }
            else
            {
                task->performTask(); // Do the cooking
                finalizeTask(*task); // Finalize the results
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
        usdMesh.GetPointsAttr().Get(&stageAndPrim.ownedMesh.pointsValue);
        if (!stageAndPrim.ownedMesh.pointsValue.size())
        {
            time = UsdTimeCode::EarliestTime();
            usdMesh.GetPointsAttr().Get(&stageAndPrim.ownedMesh.pointsValue, time);
            CARB_LOG_ERROR(
                "omni.physx.cooking does not support time sampled points. Ignoring all but first sample. Mesh path: %s",
                usdMesh.GetPrim().GetPrimPath().GetText());
        }
        usdMesh.GetFaceVertexIndicesAttr().Get(&stageAndPrim.ownedMesh.indicesValue, time);
        usdMesh.GetFaceVertexCountsAttr().Get(&stageAndPrim.ownedMesh.facesValue, time);
        usdMesh.GetHoleIndicesAttr().Get(&stageAndPrim.ownedMesh.holesValue, time);
        uint32_t pointCount = uint32_t(stageAndPrim.ownedMesh.pointsValue.size());
        uint32_t indicesCount = uint32_t(stageAndPrim.ownedMesh.indicesValue.size());
        uint32_t facesCount = uint32_t(stageAndPrim.ownedMesh.facesValue.size());
        if (pointCount && indicesCount && facesCount)
        {
            const carb::Float3* points = reinterpret_cast<const carb::Float3*>(&stageAndPrim.ownedMesh.pointsValue[0]);
            const int32_t* indices = stageAndPrim.ownedMesh.indicesValue.data();
            const int32_t* faces = stageAndPrim.ownedMesh.facesValue.data();
            meshView.points = { points, pointCount };
            meshView.indices = { indices, indicesCount };
            meshView.faces = { faces, facesCount };
            if (!stageAndPrim.ownedMesh.holesValue.empty())
            {
                const int32_t* holes = stageAndPrim.ownedMesh.holesValue.data();
                uint32_t holesCount = uint32_t(stageAndPrim.ownedMesh.holesValue.size());
                meshView.holeIndices = { holes, holesCount };
            }
            stageAndPrim.ownedMesh.faceMaterials.resize(facesCount);
            omni::span<uint16_t> faceMaterials = { stageAndPrim.ownedMesh.faceMaterials.data(), facesCount };
            if (triangulateusd::TriangulateUSDPrim::fillFaceMaterials(
                    stageAndPrim.usdPrim, faceMaterials, time, outMaxMaterialIndex))
            {
                meshView.faceMaterials = faceMaterials;
            }
            else
            {
                stageAndPrim.ownedMesh.faceMaterials.clear();
            }
            TfToken orientation;
            usdMesh.GetOrientationAttr().Get(&orientation);
            meshView.rightHandedOrientation = (orientation == UsdGeomTokens->rightHanded);
            return true;
        }
        return false;
    }

    static bool fillFabricMeshView(const omni::physx::PhysxCookingComputeRequest& request,
                                   omni::physx::PhysxCookingMeshView& meshView,
                                   CookingStageAndPrim& stageAndPrim,
                                   uint16_t& outMaxMaterialIndex)
    {
        CARB_PROFILE_ZONE(0, "ICookingComputeService::fillFabricMeshView");
        omni::fabric::StageReaderWriter stageReaderWriter = omni::fabric::StageReaderWriterId(stageAndPrim.fabricStageId);
        omni::fabric::StageReaderWriterUsd stageReaderWriterUsd = omni::fabric::StageReaderWriterId(stageAndPrim.fabricStageId);
        omni::fabric::Path pathC = stageReaderWriterUsd.registerPath(omni::fabric::handleToSdfPath(request.primId));
        auto pointsSpan = stageReaderWriter.getArrayAttributeRd<carb::Float3>(pathC, FabricTokens::get().PointsAttr);
        auto indicesSpan = stageReaderWriter.getArrayAttributeRd<int32_t>(pathC, FabricTokens::get().IndicesAttr);
        auto facesSpan = stageReaderWriter.getArrayAttributeRd<int32_t>(pathC, FabricTokens::get().FacesAttr);
        auto holesSpan = stageReaderWriter.getArrayAttributeRd<int32_t>(pathC, FabricTokens::get().HolesAttr);
        auto orientation = stageReaderWriter.getAttributeRd<omni::fabric::Token>(pathC, FabricTokens::get().OrientationAttr);
        if (pointsSpan.empty() || indicesSpan.empty() || facesSpan.empty())
            return false;

        meshView.points = { pointsSpan.data(), pointsSpan.size() };
        meshView.indices = { indicesSpan.data(), indicesSpan.size() };
        meshView.faces = { facesSpan.data(), facesSpan.size() };
        meshView.holeIndices = { holesSpan.data(), holesSpan.size() };
        meshView.rightHandedOrientation = !orientation || *orientation != FabricTokens::get().LeftHanded;
        outMaxMaterialIndex = 0; // TODO: OM-96865 Support face materials with Fabric/USDRT
        return true;
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
    ::physx::PxDefaultAllocator mPxAllocator;
    ::physx::PxFoundation* mPxFoundation = nullptr;
    CarbPhysXErrorCallback mErrorCallback;

    carb::tasking::ITasking* mTasking = nullptr;
    omni::physx::IPhysxFoundation* mPhysxFoundation = nullptr;
    std::atomic_uint32_t mFinishedCookingTasksCount = { 0 };
    carb::tasking::MutexWrapper mGlobalMutex;
    uint32_t mLocalMeshCacheSizeMB = 1024;
    cookedcache::CookedCache* mCookedCache = nullptr;
};

ICookingComputeService* createCookingComputingService()
{
    return new CookingComputeService();
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
    { // I hope this is safe to do even on a non-main thread, as well as pxr::UsdGeomGetStageMetersPerUnit
        stageAndPrim.stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(request.primStageId)));
    }
    if (!stageAndPrim.stage)
    {
        CARB_LOG_ERROR("PhysX could not find USD stage");
        result.result = PhysxCookingResult::eERROR_INVALID_STAGE;
        request.onFinished(result);
        return false;
    }

    // only use fabric if USD prim does not exist
    const SdfPath meshPath = intToPath(request.primId);
    const UsdPrim usdPrimSource = stageAndPrim.stage->GetPrimAtPath(meshPath);
    if (!usdPrimSource || !usdPrimSource.IsValid() || !usdPrimSource.IsA<UsdGeomMesh>())
    {
        if (stageAndPrim.fabricStageId == 0)
        {
            auto iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
            stageAndPrim.fabricStageId = iStageReaderWriter->get(request.primStageId).id;

            if (stageAndPrim.fabricStageId)
            {
                omni::fabric::StageReaderWriterUsd stageReaderWriterUsd =
                    omni::fabric::StageReaderWriterId(stageAndPrim.fabricStageId);
                const omni::fabric::Path fabricPath =
                    stageReaderWriterUsd.registerPath(omni::fabric::handleToSdfPath(request.primId));
             
                if (iStageReaderWriter->primExists(stageAndPrim.fabricStageId, fabricPath))
                {
                    {
                        CARB_PROFILE_ZONE(0, "ICookingComputeService::fabricSync");
                        const omni::fabric::UsdStageId stageId = { uint64_t(request.primStageId) };
                        usdrt::UsdStageRefPtr stage = usdrt::UsdStage::Attach(stageId, stageAndPrim.fabricStageId);
                        stage->SynchronizeToFabric(usdrt::TimeChange::NoUpdate);
                    }

                    omni::fabric::StageReaderWriter stageRW =
                        omni::fabric::StageReaderWriterId(stageAndPrim.fabricStageId);
                    if (stageRW.attributeExists(fabricPath, FabricTokens::get().PointsAttr) &&
                        stageRW.attributeExists(fabricPath, FabricTokens::get().IndicesAttr) &&
                        stageRW.attributeExists(fabricPath, FabricTokens::get().FacesAttr))
                    {
                        const char* primPathText = omni::fabric::toSdfPath(fabricPath).GetText();
                        request.primMeshText = { primPathText, strlen(primPathText) };
                        result.requestSource = PhysxCookingComputeResult::eREQUEST_SOURCE_FABRIC;
                        return true;
                    }
                }
            }
        }
        else
        {
            return true;
        }
    }

    if (!stageAndPrim.usdPrim.IsValid()) // USD Fallback
    {
        if (request.options.hasFlag(omni::physx::PhysxCookingComputeRequest::Options::kForceDisableUSDAccess))
            return false; // User has told us not to try reading USD at all, we're probably on a non-main thread
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
    if (request.primId == 0)
        return true; // Workaround for requests without input source

    return result.requestSource == PhysxCookingComputeResult::eREQUEST_SOURCE_USD ?
               CookingComputeService::fillUSDMeshView(
                   request, request.primMeshView, stageAndPrim, result.triangulationMaxMaterialIndex) :
               CookingComputeService::fillFabricMeshView(
                   request, request.primMeshView, stageAndPrim, result.triangulationMaxMaterialIndex);
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
        request.primMeshMetersPerUnit = pxr::UsdGeomGetStageMetersPerUnit(stageAndPrim.stage);
        if (stageAndPrim.usdPrim) // Workaround for requests without input source
        {
            if (result.requestSource == PhysxCookingComputeResult::eREQUEST_SOURCE_USD)
            {
                request.primMeshView.rightHandedOrientation =
                    CookingComputeService::isRightHandedOrientation(UsdGeomMesh(stageAndPrim.usdPrim));
            }
            else
            {
                omni::fabric::StageReaderWriter stageRW = omni::fabric::StageReaderWriterId(stageAndPrim.fabricStageId);
                omni::fabric::StageReaderWriterUsd stageReaderWriterUsd =
                    omni::fabric::StageReaderWriterId(stageAndPrim.fabricStageId);
                const omni::fabric::Path path = stageReaderWriterUsd.registerPath(omni::fabric::handleToSdfPath(request.primId));
                auto orientation = stageRW.getAttributeRd<omni::fabric::Token>(path, FabricTokens::get().OrientationAttr);
                request.primMeshView.rightHandedOrientation = !orientation || *orientation != FabricTokens::get().LeftHanded;
            }
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
        if (request.primMeshView.isEmpty())
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
