// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "UjitsoMeshCookingContext.inl"
#include "UjitsoMeshProcessors.h"

#include "../service/CookingHashing.h"
#include "UjitsoResourceManager.h"

#include <carb/settings/ISettings.h>
#include <carb/tokens/ITokens.h>
#include <carb/tokens/TokensUtils.h>
#include <carb/extras/Timer.h>
#include <private/omni/physx/IPhysxCookingServicePrivate.h>
#include <omni/physx/IPhysxSettings.h>
#include <carb/profiler/Profile.h>
#include <carb/datastore/DataStoreClearing.h>
#include <carb/ujitso/IDefaultUjitso.h>

using namespace carb::dictionary;
using namespace carb::ujitso;

using lock_guard = std::lock_guard<carb::tasking::MutexWrapper>;

// #define FORCE_UJITSO_CACHE  1   // If defined, forces the cache off (0) or on (1)
// #define FORCE_UJITSO_REMOTE 0   // If defined, forces ujitso remote caching off (0) or on (1)

#define OMNI_HUB_ENABLED    0

namespace omni
{
namespace physx
{

constexpr char kLocalCookingCacheDirectoryName[] = "PhysXLocalMeshCache";
constexpr char kRemoteCookingCacheDirectory[] = "/physx-transient/cooking/remoteCookingCachePath";
constexpr char kUseRemoteCookingCacheDiscovery[] = "/physx-transient/cooking/useRemoteCookingCacheDiscoveryForWrites";
constexpr char kRemoteCookingCacheDiscoveryPath[] = "/physx-transient/cooking/remoteCookingCacheDiscoveryPath";

constexpr int64_t   kDefaultDeveloperKey = 0ll;
constexpr int64_t   kUniqueDeveloperKey = -1ll;


// Cooking compute service implementation

struct UjitsoCookingComputeService : public ICookingComputeService
{
    /**
     * Static create() function to safely produce a valid UjitsoCookingComputeService
     * 
     * \return pointer to a new UjitsoCookingComputeService if successful, nullptr otherwise.
     */
    static UjitsoCookingComputeService* create()
    {
        UjitsoCookingComputeService* service = new UjitsoCookingComputeService();
        if (service && !service->isValid())
        {
            CARB_LOG_WARN("Failed to create a valid UJITSO Cooking Compute Service");
            service->release();
            service = nullptr;
        }
        return service;
    }

    /**
     * Implementation of ICookingComputeService interface follows
     */

    virtual uint32_t pumpAsyncContext(PhysxCookingAsyncContext context) override final
    {
        lock_guard asyncContextLock(m_asyncContextMutex);

        if (!context)
            return 0;

        // Total from fallback context and the context for this service
        uint32_t pumpCount = 0;

        const PhysxCookingAsyncContext fallbackContext = getFallbackContext(context);
        if (fallbackContext)
            pumpCount += m_fallback->pumpAsyncContext(fallbackContext);

        // If using ujitso service, add in contribution from UjitsoAsyncContext
        if (useUjitsoCollisionCooking())
        {
            pumpCount += (uint32_t)((UjitsoAsyncContext*)context)->pump();
        }

        return pumpCount;
    }

    virtual PhysxCookingOperationHandle requestTriangleMeshCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const TriangleMeshCookingParams& triangleMeshCookingParams,
        const SdfMeshCookingParams& sdfMeshCookingParams,
        PxCudaContextManager* cudaContextManager) override final
    {
        CARB_PROFILE_ZONE(0, "UjitsoCookingComputeService::requestTriangleMeshCookedData");

        // If ujitso is disabled, use the fallback service instead
        // Doing it this way to more easily support toggling on the fly
        if (!useUjitsoCollisionCooking())
        {
            if (!fallbackContextIsValid(context)) return nullptr;
            return m_fallback->requestTriangleMeshCookedData(
                getFallbackContext(context), request, triangleMeshCookingParams,
                sdfMeshCookingParams, cudaContextManager
            );
        }

        // Set triangle mesh datatype and cooking version according to sdfMeshCookingParams.sdfResolution
        PhysxCookingDataType::Enum dataType;
        int cookingDataVersion;
        if (sdfMeshCookingParams.sdfResolution == 0)
        {
            dataType = PhysxCookingDataType::eTRIANGLE_MESH;
            cookingDataVersion = PhysxCookingDataVersion_TriangleMesh;
        }
        else
        {
            dataType = PhysxCookingDataType::eSDF_TRIANGLE_MESH;
            cookingDataVersion = PhysxCookingDataVersion_TriangleMeshSDF;
        }

        // Call generic cooking function to handle triangle mesh
        return requestCookedData(dataType, context, request,
            [&](const PhysxCookingComputeResult& result)
            {
                usdparser::MeshKey meshKeyWithOrientation = result.meshKey;
                meshKeyWithOrientation.setRightHandedOrientation(result.request->primMeshView.rightHandedOrientation);
                return MeshCRCComputation::deriveTriangleMeshCRC(
                    meshKeyWithOrientation, triangleMeshCookingParams, sdfMeshCookingParams);
            },
            [&](PhysxCookingComputeResult& result)
            {
                MeshCookingContext* cooking =
                    MeshCookingContext::create(m_resourceManager, cacheBehavior(request), result,
                        cookingDataVersion, PhysxSchemaTokens->triangleMesh);

                addParamsToRequest(cooking->getRequestBuilder(), triangleMeshCookingParams);
                addParamsToRequest(cooking->getRequestBuilder(), sdfMeshCookingParams);

                return cooking;
            });
    }

    virtual PhysxCookingOperationHandle requestConvexMeshCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const ConvexMeshCookingParams& desc) override final
    {
        CARB_PROFILE_ZONE(0, "UjitsoCookingComputeService::requestConvexMeshCookedData");

        // If ujitso is disabled, use the fallback service instead
        // Doing it this way to more easily support toggling on the fly
        if (!useUjitsoCollisionCooking())
        {
            if (!fallbackContextIsValid(context)) return nullptr;
            return m_fallback->requestConvexMeshCookedData(getFallbackContext(context), request, desc);
        }

        // Call generic cooking function to handle convex mesh
        return requestCookedData(PhysxCookingDataType::eCONVEX_MESH, context, request,
            [&](const PhysxCookingComputeResult& result) {
                return MeshCRCComputation::deriveConvexMeshCRC(result.meshKey, desc);
            },
            [&](PhysxCookingComputeResult& result) {
                MeshCookingContext* cooking =
                    MeshCookingContext::create(m_resourceManager, cacheBehavior(request), result,
                        PhysxCookingDataVersion_ConvexMesh, PhysxSchemaTokens->convexHull);
                addParamsToRequest(cooking->getRequestBuilder(), desc);
                return cooking;
            });
    }

    virtual PhysxCookingOperationHandle requestConvexMeshDecompositionCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const ConvexDecompositionCookingParams& desc) override final
    {
        CARB_PROFILE_ZONE(0, "UjitsoCookingComputeService::requestConvexMeshDecompositionCookedData");

        // If ujitso is disabled, use the fallback service instead
        // Doing it this way to more easily support toggling on the fly
        if (!useUjitsoCollisionCooking())
        {
            if (!fallbackContextIsValid(context)) return nullptr;
            return m_fallback->requestConvexMeshDecompositionCookedData(getFallbackContext(context), request, desc);
        }

        // Call generic cooking function to handle convex decomposition
        return requestCookedData(PhysxCookingDataType::eCONVEX_DECOMPOSITION, context, request,
            [&](const PhysxCookingComputeResult& result) {
                usdparser::MeshKey meshKeyWithOrientation = result.meshKey;
                meshKeyWithOrientation.setRightHandedOrientation(result.request->primMeshView.rightHandedOrientation);
                return MeshCRCComputation::deriveConvexDecompositionCRC(meshKeyWithOrientation, desc);
            },
            [&](PhysxCookingComputeResult& result) {
                MeshCookingContext* cooking =
                    MeshCookingContext::create(m_resourceManager, cacheBehavior(request), result,
                        PhysxCookingDataVersion_ConvexDecomposition, PhysxSchemaTokens->convexDecomposition, true);
                addParamsToRequest(cooking->getRequestBuilder(), desc);
                return cooking;
            });
    }

    virtual PhysxCookingOperationHandle requestSphereFillCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const SphereFillCookingParams& desc) override final
    {
        CARB_PROFILE_ZONE(0, "UjitsoCookingComputeService::requestSphereFillCookedData");

        // If ujitso is disabled, use the fallback service instead
        // Doing it this way to more easily support toggling on the fly
        if (!useUjitsoCollisionCooking())
        {
            if (!fallbackContextIsValid(context)) return nullptr;
            return m_fallback->requestSphereFillCookedData(getFallbackContext(context), request, desc);
        }

        // Call generic cooking function to handle sphere fill
        return requestCookedData(PhysxCookingDataType::eSPHERE_FILL, context, request,
            [&](const PhysxCookingComputeResult& result) {
                usdparser::MeshKey meshKeyWithOrientation = result.meshKey;
                meshKeyWithOrientation.setRightHandedOrientation(result.request->primMeshView.rightHandedOrientation);
                return MeshCRCComputation::deriveSphereFillCRC(meshKeyWithOrientation, desc);
            },
            [&](PhysxCookingComputeResult& result) {
                MeshCookingContext* cooking =
                    MeshCookingContext::create(m_resourceManager, cacheBehavior(request), result,
                        PhysxCookingDataVersion_SphereFill, PhysxSchemaTokens->sphereFill);
                addParamsToRequest(cooking->getRequestBuilder(), desc);
                return cooking;
            });
    }

    virtual PhysxCookingOperationHandle requestSoftBodyMeshCookedDataDeprecated(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const SoftBodyMeshCookingParamsDeprecated& params) override final
    {
        // Fallback implementation only for soft body cooking
        if (!fallbackContextIsValid(context)) return nullptr;
        return m_fallback->requestSoftBodyMeshCookedDataDeprecated(
            getFallbackContext(context), request, params
        );
    }

    virtual PhysxCookingOperationHandle requestDeformableBodyTetMeshCookedDataDeprecated(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const DeformableBodyTetMeshCookingParamsDeprecated& params) override final
    {
        // Fallback implementation only for deformable tet mesh cooking
        if (!fallbackContextIsValid(context)) return nullptr;
        return m_fallback->requestDeformableBodyTetMeshCookedDataDeprecated(
            getFallbackContext(context), request, params
        );
    }

    virtual PhysxCookingOperationHandle requestParticleClothMeshCookedDataDeprecated(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::ParticleClothMeshCookingParamsDeprecated& params) override final
    {
        // Fallback implementation only for particle cloth mesh cooking
        if (!fallbackContextIsValid(context)) return nullptr;
        return m_fallback->requestParticleClothMeshCookedDataDeprecated(
            getFallbackContext(context), request, params
        );
    }

    virtual PhysxCookingOperationHandle requestDeformableVolumeMeshCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::DeformableVolumeMeshCookingParams& params) override final
    {
        // Fallback implementation only for deformable volume mesh cooking
        if (!fallbackContextIsValid(context)) return nullptr;
        return m_fallback->requestDeformableVolumeMeshCookedData(
            getFallbackContext(context), request, params
        );
    }

    virtual PhysxCookingOperationHandle requestVolumeDeformableBodyCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::VolumeDeformableBodyCookingParams& params) override final
    {
        // Fallback implementation only for deformable tet mesh cooking
        if (!fallbackContextIsValid(context)) return nullptr;
        return m_fallback->requestVolumeDeformableBodyCookedData(
            getFallbackContext(context), request, params
        );
    }

    virtual PhysxCookingOperationHandle requestSurfaceDeformableBodyCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::SurfaceDeformableBodyCookingParams& params) override final
    {
        // Fallback implementation only for deformable tet mesh cooking
        if (!fallbackContextIsValid(context)) return nullptr;
        return m_fallback->requestSurfaceDeformableBodyCookedData(
            getFallbackContext(context), request, params
        );
    }

    virtual PhysxCookingOperationHandle requestParticlePoissonSamplingCookedData(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::ParticlePoissonSamplingCookingParams& params) override final
    {
        // Fallback implementation only for particle poisson sampling cooking
        if (!fallbackContextIsValid(context)) return nullptr;
        return m_fallback->requestParticlePoissonSamplingCookedData(
            getFallbackContext(context), request, params
        );
    }

    virtual PhysxCookingAsyncContext createAsyncContext(PhysxCookingAsyncContextParameters& parameters) override final
    {
        CARB_PROFILE_ZONE(0, "UjitsoCookingComputeService::createAsyncContext");

        lock_guard asyncContextLock(m_asyncContextMutex);

        /**
         * TEMPORARY
         * 
         * Create a fallback service async context and store a pointer to it in this service's async context
         * 
         * This may be removed if the ujitso service completely replaces the existing service.
         */
        PhysxCookingAsyncContext fallbackContext = m_fallback->createAsyncContext(parameters);
        if (fallbackContext == nullptr)
        {
            CARB_LOG_ERROR("Fallback context unable to be created.");
            return nullptr;
        }

        // parameters.contextName is an omni::span, which is why we're copying the name this way
        std::string name;
        name.insert(name.begin(), parameters.contextName.data(),
                    parameters.contextName.data() + parameters.contextName.size_bytes());

        // Return if a fallback context with this name already exists
        if (m_asyncContexts.count(name))
        {
            CARB_LOG_ERROR("Trying to create a context with name \"%s\" that already exists", name.c_str());
            m_fallback->destroyAsyncContext(fallbackContext);
            return nullptr;
        }

        const int32_t maxRunningProcessCount = std::max(0, m_settings->getAsInt(kSettingUjitsoCookingMaxProcessCount));

        std::unique_ptr<UjitsoAsyncContext> context =
            std::make_unique<UjitsoAsyncContext>(name, (size_t)maxRunningProcessCount);
        UjitsoAsyncContext* asyncContext = context.get();
        m_asyncContexts[name] = std::move(context);

        /**
         * TEMPORARY
         * 
         * This may be removed if the ujitso service completely replaces the existing service.
         */
        asyncContext->m_fallbackContext = fallbackContext;

        return asyncContext;
    }

    virtual void destroyAsyncContext(PhysxCookingAsyncContext context) override final
    {
        CARB_PROFILE_ZONE(0, "UjitsoCookingComputeService::destroyAsyncContext");

        lock_guard asyncContextLock(m_asyncContextMutex);

        /**
         * TEMPORARY
         * 
         * This may be removed if the ujitso service completely replaces the existing service.
         */
        const PhysxCookingAsyncContext fallbackContext = getFallbackContext(context);
        if (fallbackContext)
        {
            ((UjitsoAsyncContext*)context)->m_fallbackContext = nullptr;
            m_fallback->destroyAsyncContext(fallbackContext);
        }

        UjitsoAsyncContext* asyncContext = safeCastContext(asyncContextLock, context);
        if (asyncContext)
        {
            const size_t runningCount = asyncContext->cancelAll();
            if (runningCount)
                CARB_LOG_WARN("Destroying async context \"%s\" holding %d active async tasks.",
                              asyncContext->getName().c_str(), (uint32_t)runningCount);
            m_asyncContexts.erase(asyncContext->getName());
        }
        else
        {
            CARB_LOG_ERROR(
                "omni.physx.cooking: trying to delete a ujitso context with handle \"%p\" that doesn't exist", context);
        }
    }

    virtual void release() override final
    {
        delete this;
    }

    virtual uint32_t getActiveTaskCount(PhysxCookingAsyncContext context) override final
    {
        lock_guard asyncContextLock(m_asyncContextMutex);

        UjitsoAsyncContext* asyncContext = safeCastContext(asyncContextLock, context);
        CHECK_RETURN_VALUE_ON_FAIL(asyncContext, 0);

        // Total the active task counts from the fallback context and the context for this service
        uint32_t activeTaskCount = m_fallback->getActiveTaskCount(asyncContext->m_fallbackContext);

        if (useUjitsoCollisionCooking())
            activeTaskCount += (uint32_t)asyncContext->getQueuedProcessCount();

        return activeTaskCount;
    }

    virtual uint32_t getFinishedCookingTasksCount() const override final
    {
        // Total the finished task count from the fallback service and this service
        return m_fallback->getFinishedCookingTasksCount() + (uint32_t)UjitsoProcessManager::getFinishedProcessCount();
    }

    virtual uint32_t cancelAllTasks(PhysxCookingAsyncContext context) override final
    {
        CARB_PROFILE_ZONE(0, "UjitsoCookingComputeService::cancelAllTasks");

        lock_guard asyncContextLock(m_asyncContextMutex);

        UjitsoAsyncContext* asyncContext = safeCastContext(asyncContextLock, context);
        CHECK_RETURN_VALUE_ON_FAIL(asyncContext, 0);

        // Cancel all tasks for the fallback context and the context for this service
        return m_fallback->cancelAllTasks(asyncContext->m_fallbackContext) + (uint32_t)asyncContext->cancelAll();
    }

    virtual bool cancelTask(PhysxCookingOperationHandle handle, bool invokeCallbackAnyway) override final
    {
        CARB_PROFILE_ZONE(0, "UjitsoCookingComputeService::cancelTask");

        lock_guard asyncContextLock(m_asyncContextMutex);

        if (!useUjitsoCollisionCooking())
        {
            return m_fallback->cancelTask(handle, invokeCallbackAnyway);
        }

        // Otherwise look for the handle in this service's contexts
        for (auto& item : m_asyncContexts)
        {
            UjitsoAsyncContext* asyncContext = item.second.get();
            UjitsoProcessContext* process = (UjitsoProcessContext*)handle;
            // Check that the process is queued first to avoid warning from cancel function if it's not
            if (asyncContext->queued(process) && asyncContext->cancel(process, invokeCallbackAnyway)) return true;
        }

        CARB_LOG_WARN("UjitsoCookingComputeService::cancelTask: Trying to cancel a task with handle %p that doesn't"
                      " exist", handle);

        return false;
    }

    virtual bool waitForTaskToFinish(PhysxCookingOperationHandle handle, int64_t timeoutMs) override final
    {
        CARB_PROFILE_ZONE(0, "UjitsoCookingComputeService::waitForTaskToFinish");

        lock_guard asyncContextLock(m_asyncContextMutex);

        if (!useUjitsoCollisionCooking())
        {
            return m_fallback->waitForTaskToFinish(handle, timeoutMs);
        }

        // Otherwise look for it in this service's contexts
        // Convert timeout to format that ujitso uses
        const uint32_t ujitsoTimeoutMs = timeoutMs >= 0 ? (uint32_t)timeoutMs : kTimeOutInfinite;

        for (auto& item : m_asyncContexts)
        {
            UjitsoAsyncContext* asyncContext = item.second.get();
            if (asyncContext->queued((UjitsoProcessContext*)handle))
                return asyncContext->waitForProcess((UjitsoProcessContext*)handle, ujitsoTimeoutMs);
        }

        CARB_LOG_WARN("UjitsoCookingComputeService::waitForTaskToFinish: Trying to wait for a task with handle %p that"
                      " is not in the list of active tasks", handle);

        return false;
    }

    virtual void resetMeshCacheContents() override final
    {
        CARB_PROFILE_ZONE(0, "UjitsoCookingComputeService::resetLocalMeshCacheContents");

        // request UJITSO to clear its cache if it is set up
        if (useUjitsoCollisionCooking())
        {
            carb::ujitso::DefaultState* defaultState = carb::ujitso::getDefaultState();
            CHECK_RETURN_ON_FAIL(defaultState);
            auto dataStoreConfigurator = defaultState->dataStoreConfigurator;
            CHECK_RETURN_ON_FAIL(dataStoreConfigurator);
            auto dataStore = dataStoreConfigurator->getDataStore();
            CHECK_RETURN_ON_FAIL(dataStore);

            omni::core::ObjectPtr<carb::datastore::IDataStoreClearing> clearingDSInterface =
                omni::core::cast<carb::datastore::IDataStoreClearing>(dataStore);
            if (clearingDSInterface)
            {
                clearingDSInterface->clearPercentage();
            }
            else
            {
                CARB_LOG_WARN(
                    "UjitsoCookingComputeService::resetLocalMeshCacheContents: "
                    "Failed to get IDataStoreClearing interface, local cache won't be cleared. "
                    "Check if Omni Hub is active, it does not support clearing the local cache."
                );
            }
        }
    }

    virtual bool lazyGetCudaContextManager(PhysxCookingDataType::Enum dataType,
                                           const PhysxCookingComputeRequest& request,
                                           PxCudaContextManager*& cudaContextManager,
                                           PxPhysicsGpu*& physicsGPU) override final
    {
        // Use the fallback service to manage the cuda context
        return m_fallback->lazyGetCudaContextManager(dataType, request, cudaContextManager, physicsGPU);
    }

private:
    /**
     * Private constructor ensures that only the (safe) static create() function can produce a new object.
     */
    UjitsoCookingComputeService() : m_resourceManager(nullptr), m_cookingProcessor(nullptr),
        m_triangulationProcessor(nullptr), m_allowCaching(true), m_allowRemoteCaching(false)
    {
        m_fallback = createCookingComputingService();   // Create local implementation
        CHECK_RETURN_ON_FAIL(m_fallback);

        m_settings = carb::getCachedInterface<carb::settings::ISettings>();
        CHECK_RETURN_ON_FAIL(m_settings);

        // check to make sure that the remote cache status matches the UJITSO settings
        auto dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
        auto ujitsoSettings = m_settings->getSettingsDictionary("/UJITSO/datastore/GRPCDataStoreServer");
        const size_t childCount = dict->getItemChildCount(ujitsoSettings);
        const bool useRemoteCache = m_settings->getAsBool(omni::physx::kSettingUjitsoRemoteCacheEnabled);
        if (useRemoteCache != (childCount > 0))
        {
            CARB_LOG_WARN(
                "/UJITSO/datastore/GRPCDataStoreServer settings must be specified when remote cache is enabled "
                "for PhysX.  If remote cache is turned off, UJITSO settings should match, otherwise no caching "
                "will happen."
            );
        }

        carb::dictionary::SubscriptionId* subId = nullptr;

        // get the initial value for settings and subscribe to changes on them
        this->ujitsoCollisionCookingStateChanged(nullptr, carb::dictionary::ChangeEventType::eChanged, this);
        subId = m_settings->subscribeToNodeChangeEvents(kSettingUjitsoCollisionCooking,
                                                        ujitsoCollisionCookingStateChanged, this);
        m_subscribedSettings.push_back(subId);

        this->ujitsoMaxProcessCountChanged(nullptr, carb::dictionary::ChangeEventType::eChanged, this);
        subId = m_settings->subscribeToNodeChangeEvents(kSettingUjitsoCookingMaxProcessCount,
                                                        ujitsoMaxProcessCountChanged, this);
        m_subscribedSettings.push_back(subId);

        this->ujitsoCookingDevKeyChanged(nullptr, carb::dictionary::ChangeEventType::eChanged, this);
        subId = m_settings->subscribeToNodeChangeEvents(kSettingUjitsoCookingDevKey, ujitsoCookingDevKeyChanged, this);
        m_subscribedSettings.push_back(subId);

        this->ujitsoRemoteCacheEnabledChanged(nullptr, carb::dictionary::ChangeEventType::eChanged, this);
        subId = m_settings->subscribeToNodeChangeEvents(
            kSettingUjitsoRemoteCacheEnabled, ujitsoRemoteCacheEnabledChanged, this);
        m_subscribedSettings.push_back(subId);

        // Create processors
        m_cookingProcessor = createPhysxMeshCookingProcessor(*this);
        CHECK_RETURN_ON_FAIL(m_cookingProcessor);
        m_triangulationProcessor = createPhysxMeshTriangulationProcessor();
        CHECK_RETURN_ON_FAIL(m_triangulationProcessor);

        // Register processors
        IRegistry* registry = carb::getCachedInterface<carb::ujitso::IRegistry>();
        CHECK_RETURN_ON_FAIL(registry);
        CHECK_RETURN_ON_FAIL(OperationResult::SUCCESS ==
            registry->registerProcessor(*m_cookingProcessor, getPhysxMeshCookingProcessorFilter()));
        CHECK_RETURN_ON_FAIL(OperationResult::SUCCESS ==
            registry->registerProcessor(*m_triangulationProcessor, getPhysxMeshTriangulationProcessorFilter()));

        // Set up data store.  This must be valid for isValid() to return true.  Keep this in mind if any of
        // the above operations fail - exiting before this point ensures this service will not be used.
        m_resourceManager = UjitsoResourceManager::create();

        // Make sure the finished process count is zeroed out.
        UjitsoProcessManager::zeroFinishedProcessCount();
    }

    ~UjitsoCookingComputeService()
    {
        carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
        if (settings)
        {
            for (auto subId : m_subscribedSettings)
            {
                if (subId) settings->unsubscribeToChangeEvents(subId);
            }
            m_subscribedSettings.clear();
        }

        for (auto& item : m_asyncContexts)
        {
            UjitsoAsyncContext& asyncContext = *item.second.get();
            cancelAllTasks(&asyncContext);
        }

        if (m_resourceManager)
        {
            m_resourceManager->release();
        }

        IRegistry* registry = carb::getCachedInterface<carb::ujitso::IRegistry>();
        if (registry)
        {
            if (m_cookingProcessor)
                registry->unregisterProcessor(*m_cookingProcessor);
            if (m_triangulationProcessor)
                registry->unregisterProcessor(*m_triangulationProcessor);
        }

        releasePhysxMeshCookingProcessor(m_cookingProcessor);
        releasePhysxMeshTriangulationProcessor(m_triangulationProcessor);

        if (m_fallback)
        {
            releaseCookingComputingService(m_fallback);
        }
    }

    bool isValid()
    {
        return m_fallback && m_settings && m_resourceManager;
    }

    void setUjitsoCollisionCooking(bool useUjitso)
    {
        m_ujitsoCollisionCooking = useUjitso;
    }

    bool useUjitsoCollisionCooking() const
    {
#if defined(FORCE_UJITSO_CACHE)
        return !!(FORCE_UJITSO_CACHE);
#else
        return m_ujitsoCollisionCooking;
#endif
    }

    void setMaxProcessCount(size_t maxProcessCount)
    {
        lock_guard asyncContextLock(m_asyncContextMutex);

        for (auto& item : m_asyncContexts)
        {
            UjitsoAsyncContext* asyncContext = item.second.get();
            asyncContext->setMaxRunningProcessCount(maxProcessCount);
        }
    }

    void setUseOmniHub(bool useOmniHub)
    {
#if OMNI_HUB_ENABLED
        if (useOmniHub && !carb::datastore::DataStoreConfigurator::supportsOmniHub())
        {
            useOmniHub = false;
            CARB_LOG_INFO_ONCE("UjitsoCookingComputeService::setUseOmniHub: OmniHub requested but is not supported.  "
                "Requesting LocalDataStore.");
        }

        if (m_dataStoreConfig.m_allowHubDataStore != useOmniHub)
        {
            m_dataStoreConfig.m_allowHubDataStore = useOmniHub;
            if (m_resourceManager)  // Might not be available upon initialization, avoid error log
                recreateResources();
        }
#else
        if (useOmniHub)
            CARB_LOG_INFO_ONCE("UjitsoCookingComputeService::setUseOmniHub currently not supported by this service.");
#endif
    }

    void setDevKey(uint64_t devKey)
    {
        if (!m_resourceManager)
            return;

        switch (devKey)
        {
        case kDefaultDeveloperKey:
            m_resourceManager->setDeveloperKey();
            break;
        case kUniqueDeveloperKey:
            m_resourceManager->createUniqueDeveloperKey();
            break;
        default:
            m_resourceManager->setDeveloperKey(devKey);
            break;
        }
    }

    void setUseRemoteCache(bool useRemote)
    {
#if defined(FORCE_UJITSO_REMOTE)
        useRemote = !!(FORCE_UJITSO_REMOTE);
#endif
        if (m_allowRemoteCaching != useRemote && isValid())
        {
            CARB_LOG_WARN("This needs to be set before UJITSO is initialized to take effect properly");
            if (!useRemote)
            {
                CARB_LOG_WARN(
                    "Disabling this after it is set up initially will disable caching, local caching will NOT be used"
                );
            }
        }
        m_allowRemoteCaching = useRemote;
    }

    CacheBehaviorType cacheBehavior(const PhysxCookingComputeRequest& request) const
    {
        if (!m_allowCaching || !request.options.hasFlag(PhysxCookingComputeRequest::Options::kLoadCookedDataFromCache))
            return CacheBehaviorType::NoCaching;

        return m_allowRemoteCaching ? CacheBehaviorType::Default : CacheBehaviorType::Local;
    }

    template <typename FinishRequestFunction>
    bool finishRequestWithUSDLoadedData(
        PhysxCookingAsyncContext context,
        PhysxCookingComputeRequest& request,
        PhysxCookingComputeResult& result,
        CookingStageAndPrim& stageAndPrim,
        FinishRequestFunction finishRequestFunction)
    {
        CARB_PROFILE_ZONE(0, "UjitsoCookingComputeService::finishRequestWithUSDLoadedData");

        bool success = true;

        std::vector<std::vector<uint8_t>> triangulationResultDataBlocks;
 
        // check if triangulation data is needed for this request
        if (request.triangulation.isNeeded())
        {
            if (request.primMeshView.isEmpty())
            {
                // If the mesh view is empty we require valid prim data
                if (request.dataInputMode != PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_ID)
                {
                    result.result = PhysxCookingResult::eERROR_INVALID_PRIM;
                    return false;
                }

                if (!ICookingComputeService::fillMeshView(result, request, stageAndPrim))
                {
                    result.result = PhysxCookingResult::eERROR_INVALID_PRIM;
                    return false;
                }
            }

            success = false;    // Until proven otherwise

            // build a container for the triangulation data and register it
            PhysicsTriangulationInputContainer* triangulationContainer =
                new PhysicsTriangulationInputContainer(result, request);
            if (triangulationContainer)
            {
                IRegistry* registry = carb::getCachedInterface<carb::ujitso::IRegistry>();
                if (registry)
                {
                    // create ujitso request for the data
                    UjitsoProcessContext triangulation(m_resourceManager, cacheBehavior(request));

                    ContainerHandle containerHandle = registry->registerContainer(*triangulationContainer);

                    addExtDep(triangulation.getRequestBuilder());
                    triangulation.getRequestBuilder().add(PHYSX_TRIANGULATE_MESH_STR);
                    triangulation.getRequestBuilder().add(TRIANGULATION_VERSION_STR,
                                                            (uint32_t)PhysxCookingDataVersion_MeshTriangulation);
                    triangulation.getRequestBuilder().add(PHYSICS_TRIANGULATION_INPUT_CONTAINER_NAME, containerHandle);

                    triangulation.requestBuild();
                    triangulation.waitRequest();

                    // We need to save the triangulation buffers in the upper ujitso context otherwise the
                    // triangulation view will point to memory that goes out of scope at the end this if statement
                    triangulationResultDataBlocks = std::move(triangulation.getResultDataBlocks());

                    // Build triangulation view from triangulationDataBlocks
                    success = triangulationResultDataBlocks.size() == 1 &&
                        buildTriangulationViewFromData(result.triangulationView,
                            result.triangulationMaxMaterialIndex, triangulationResultDataBlocks[0]);

                    // tear the container back down regardless of the success of the triangulation task
                    constexpr uint32_t timeout = 60000;
                    OperationResult unregisterResult = registry->unregisterContainer(containerHandle, timeout);
                    switch (unregisterResult)
                    {
                    case OperationResult::SUCCESS:
                        break;
                    case OperationResult::TIMEOUT_ERROR:
                        CARB_LOG_ERROR("UjitsoCookingComputeService::finishRequestWithUSDLoadedData: Unable to "
                                       "unregister triangulation container in %ds.", timeout / 1000);
                        break;
                    default:
                        CARB_LOG_ERROR("UjitsoCookingComputeService::finishRequestWithUSDLoadedData: "
                                       "unregisterContainer returned error value %d.", (uint32_t)unregisterResult);
                        break;
                    }
                }

                delete triangulationContainer;
            }
        }

        if (success)
            finishRequestFunction(request, result);

        return success;
    }

    // Does not lock, but requires a lock to exist
    UjitsoAsyncContext* safeCastContext(lock_guard& guard, PhysxCookingAsyncContext context)
    {
        if (context)
        {
            for (auto& item : m_asyncContexts)
            {
                UjitsoAsyncContext* asyncContext = item.second.get();
                if (asyncContext == context) return asyncContext;
            }
        }

        return nullptr;
    }

    bool fallbackContextIsValid(PhysxCookingAsyncContext context)
    {
        return (!context || ((UjitsoAsyncContext*)context)->m_fallbackContext);
    }

    PhysxCookingAsyncContext getFallbackContext(PhysxCookingAsyncContext context)
    {
        return (context ? ((UjitsoAsyncContext*)context)->m_fallbackContext : nullptr);
    }

    template <typename DeriveCRCFunction, typename CreateTaskFunction>
    PhysxCookingOperationHandle requestCookedData(PhysxCookingDataType::Enum DataType,
                                                  PhysxCookingAsyncContext context,
                                                  const PhysxCookingComputeRequest& request,
                                                  DeriveCRCFunction deriveCRCFunction,
                                                  CreateTaskFunction createTaskFunction)
    {
        CARB_PROFILE_ZONE(0, "UjitsoCookingComputeService::requestCookedData");

        PhysxCookingComputeResult result;
        PhysxCookingComputeRequest requestCopy = request;
        result.request = &requestCopy;
        requestCopy.dataType = DataType;

        UjitsoAsyncContext* asyncContext = nullptr;
        if (result.request->options.hasFlag(PhysxCookingComputeRequest::Options::kComputeAsynchronously))
        {
            {
                lock_guard asyncContextLock(m_asyncContextMutex);
                if (context)
                    asyncContext = safeCastContext(asyncContextLock, context);
            }
            if (!asyncContext)
            {
                CARB_LOG_ERROR("UjitsoCookingService:: validateContext: Invalid context %p used.", context);
                result.result = PhysxCookingResult::eERROR_INVALID_CONTEXT;
                result.request->onFinished(result);
                return nullptr;
            }
        }
        else
        {
            // For synchronous computation it's safe to simply cast (may be nullptr)
            asyncContext = reinterpret_cast<UjitsoAsyncContext*>(context);
        }
 
        // Compute Mesh Key and CRC
        CookingStageAndPrim stageAndPrim;
        if (!computeMeshKeyIfNeeded(result, requestCopy, stageAndPrim))
            return nullptr;

        result.cookedDataCRC = deriveCRCFunction(result);

        if (!result.request->options.hasFlag(PhysxCookingComputeRequest::Options::kComputeGPUCookingData))
            result.cookedDataCRC.setComputeGPUData(false);

        if (request.mode == PhysxCookingComputeRequest::eMODE_COMPUTE_CRC)
        {
            result.result = PhysxCookingResult::eVALID;
            request.onFinished(result);
            return nullptr;
        }

        UjitsoProcessContext* process = findProcess(asyncContext, result.cookedDataCRC.getFullHash());
        if (process)
        {
            // A task already exists with same CRC so no need to spawn a new one.
            if (result.request->options.hasFlag(PhysxCookingComputeRequest::Options::kComputeAsynchronously))
            {
                // We save callback from this request to call it later
                reinterpret_cast<MeshCookingContext*>(process)->saveCallbackFromRequest(*result.request);
                return process;
            }
            else
            {
                // If synchronous, wait for the task to finish, so it should write to cache
                asyncContext->waitForProcess(process);
            }
        }

        MeshCookingContext* cooking = createTaskFunction(result);
        return cooking->initiateBuild((UjitsoProcessManager*)context) ? cooking : nullptr;
    }

    UjitsoProcessContext* findProcess(UjitsoAsyncContext*& asyncContext,
                                      const UjitsoProcessContext::RequestId& requestId) const
    {
        if (asyncContext) return asyncContext->find(requestId);

        for (auto& item : m_asyncContexts)
        {
            asyncContext = item.second.get();
            UjitsoProcessContext* process = asyncContext->find(requestId);
            if (process) return process;
        }

        asyncContext = nullptr;
        return nullptr;
    }

    static void ujitsoCollisionCookingStateChanged(
        const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData)
    {
        // if the setting is destroyed, fall back to the default value of false
        // otherwise, just update the internal value from the settings
        const carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
        const bool value = (eventType != carb::dictionary::ChangeEventType::eDestroyed && settings &&
                            settings->getAsBool(kSettingUjitsoCollisionCooking));
        ((UjitsoCookingComputeService*)userData)->setUjitsoCollisionCooking(value);
    }

    static void ujitsoMaxProcessCountChanged(
        const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData)
    {
        // if the setting is destroyed, fall back to the default value of false
        // otherwise, just update the internal value from the settings
        carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
        if (eventType != carb::dictionary::ChangeEventType::eDestroyed && settings)
        {
            const int32_t value = std::max(0, settings->getAsInt(kSettingUjitsoCookingMaxProcessCount));
            ((UjitsoCookingComputeService*)userData)->setMaxProcessCount((size_t)value);
        }
    }

    static void ujitsoCookingDevKeyChanged(
        const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData)
    {
        // if the setting is destroyed, fall back to the default value of false
        // otherwise, just update the internal value from the settings
        carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
        if (eventType != carb::dictionary::ChangeEventType::eDestroyed && settings)
        {
            const int64_t value = settings->getAsInt64(kSettingUjitsoCookingDevKey);
            ((UjitsoCookingComputeService*)userData)->setDevKey((uint64_t)value);
        }
    }

    static void ujitsoRemoteCacheEnabledChanged(
        const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData)
    {
        // if the setting is destroyed, fall back to the default value of false
        // otherwise, just update the internal value from the settings
        carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
        if (eventType != carb::dictionary::ChangeEventType::eDestroyed && settings)
        {
            const bool enabled = settings->getAsBool(kSettingUjitsoRemoteCacheEnabled);
            ((UjitsoCookingComputeService*)userData)->setUseRemoteCache(enabled);
        }
    }

    typedef  std::unordered_map<std::string, std::unique_ptr<UjitsoAsyncContext>> AsyncContextMap;

    ICookingComputeService*                         m_fallback;
    carb::settings::ISettings*                      m_settings;
    UjitsoResourceManager*                          m_resourceManager;
    Processor*                                      m_cookingProcessor;
    Processor*                                      m_triangulationProcessor;

    AsyncContextMap                                 m_asyncContexts;
    carb::tasking::MutexWrapper                     m_asyncContextMutex;

    bool                                            m_ujitsoCollisionCooking;
    bool                                            m_allowCaching;
    bool                                            m_allowRemoteCaching;

    std::vector<carb::dictionary::SubscriptionId*>  m_subscribedSettings;
};


// Public functions

ICookingComputeService* createUjitsoCookingComputingService()
{
    return UjitsoCookingComputeService::create();
}

} // namespace physx
} // namespace omni
