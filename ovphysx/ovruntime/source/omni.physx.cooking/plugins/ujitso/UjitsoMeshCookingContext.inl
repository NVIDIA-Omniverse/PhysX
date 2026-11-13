// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-ATTACH-SERVICES-001
 * @covers AC-3
 */

#pragma once

#include "../service/CookingComputeService.h"

#include "UjitsoMeshUtils.inl"
#include "UjitsoHashUtils.inl"

#include "UjitsoTriangulationContainer.h"
#include "UjitsoVolumeDeformableBodyInputContainer.h"
#include "UjitsoDeformableVolumeMeshInputContainer.h"
#include "UjitsoSurfaceDeformableBodyInputContainer.h"
#include "UjitsoProcessManager.h"

#include "LoggingUtils.h"

#include <carb/ujitso/UjitsoUtils.inl>
#include <omni/physx/IPhysxCookingService.h>

#include <atomic>

using namespace carb::ujitso;
using namespace physx;

namespace omni
{
namespace physx
{

typedef UjitsoProcessManager    UjitsoAsyncContext;


// Utilities

// This is needed so UJITSO knows what ext to load up to service the request
static void addExtDep(RequestBuilder<>& req)
{
    req.add(carb::ujitso::IRegistry::Extension, "omni.physx.cooking");
}

/**
 * Definition of MeshCookingContext, derived from UjitsoProcessContext.  It is designed to
 * be used with the UjitsoProcessManager, and only provides a public create function which implements
 * the underlying UjitsoProcessContext with the manager.
 * 
 * The MeshCookingContext used by the UjitsoCookingComputeService to hold the process state and physx
 * request and result structures needed to implement the ICookingComputeService methods for convex mesh,
 * convex decomposition, triangle mesh (including sdf), and sphere fill approximations.
 */
struct MeshCookingContext : public UjitsoProcessContext
{
    /**
     * Public static function to create a new MeshCookingContext
     * 
     * \param[in]   resourceManager             The resource manager used to create a ujitso cooking agent for
     *                                          synchronous events.
     * \param[in]   cacheBehavior               What level of caching to use.
     * \param[in]   cookingResult               The physx cooking result, which must also point to a valid cooking
     *                                          request through its request field.
     * \param[in]   cookingDataVersion          The data version requested for the cooking type given in the request.
     * \param[in]   collisionTypeSchemaToken    The USD schema token text (e.g. PhysxSchemaTokens->triangleMesh.GetText())
     *                                          for the cooked data type. Stored, currently unread by this class.
     * \param[in]   canHaveMultipleBuffers      Whether or not the data type may be stored using multiple buffers.
     *                                          Currently only convex decomposition requires that this be true.
     *
     * \return a new MeshCookingContext.
     */
    static MeshCookingContext* create(UjitsoResourceManager* resourceManager, CacheBehaviorType cacheBehavior,
                                      const PhysxCookingComputeResult& cookingResult, int cookingDataVersion,
                                      const char* collisionTypeSchemaToken, bool canHaveMultipleBuffers = false)
    {
        return new MeshCookingContext(resourceManager, cacheBehavior, cookingResult, cookingDataVersion,
                                      collisionTypeSchemaToken, canHaveMultipleBuffers);
    }

    /**
     * If multiple requests for the same build are made, new build processes will not be generated, but we can store
     * their onFinished callbacks so that they are called when the one build process is completed.
     */
    void saveCallbackFromRequest(const PhysxCookingComputeRequest& request)
    {
        if (request.onFinished) m_requestCallbacks.push_back(request.onFinished);
    }

    /**
     * Start the ujitso build.  If a synchronous build is requested, wait until the build completes before retuning.
     * For asynchronous builds, schedule this process context with its process manager.
     * 
     * \param[in]   processManager  If the cooking request is asynchronous, this must be a valid pointer to a
     *                              UjitsoProcessManager.
     * 
     * \return true if all request, wait, and schedule calls return a success value.
     */
    bool initiateBuild(UjitsoProcessManager* processManager = nullptr)
    {
        CARB_PROFILE_ZONE(0, "MeshCookingContext::initiateBuild");

        m_cookingResult.isSynchronousResult =
            !m_cookingRequest.options.hasFlag(PhysxCookingComputeRequest::Options::kComputeAsynchronously);

        const bool enableGpuCooking = m_cookingRequest.dataType == PhysxCookingDataType::eSDF_TRIANGLE_MESH &&
            m_cookingRequest.options.hasFlag(PhysxCookingComputeRequest::Options::kExecuteCookingOnGPU);

        getRequestBuilder().add(PHYSX_UJITSO_ENABLE_GPU_COOKING, enableGpuCooking);

        if (m_cookingResult.isSynchronousResult)
        {
            CHECK_RETURN_FALSE_ON_FAIL(requestBuild() == BuildRequestResult::eSuccess);
            CHECK_RETURN_FALSE_ON_FAIL(waitRequest());
            onComplete();
        }
        else
        {
            CHECK_RETURN_FALSE_ON_FAIL(processManager);
            CHECK_RETURN_FALSE_ON_FAIL(processManager->schedule(this));
        }

        return true;
    }

private:
    // Private so that construction must take place in create(...) with new, and can be deleted in onComplete()
    // See the definition of create(...) for a description of all parameters
    MeshCookingContext(UjitsoResourceManager* resourceManager, CacheBehaviorType cacheBehavior,
                       const PhysxCookingComputeResult& cookingResult, int cookingDataVersion,
                       const char* collisionTypeSchemaToken, bool canHaveMultipleBuffers) :
        UjitsoProcessContext(resourceManager, cacheBehavior),
        m_cookingRequest(*cookingResult.request), m_cookingResult(cookingResult),
        m_collisionTypeSchemaToken(collisionTypeSchemaToken), m_canHaveMultipleBuffers(canHaveMultipleBuffers),
        m_cookingDataVersion(cookingDataVersion), m_triangulationContainer(nullptr), m_volumeDeformableBodyInputContainer(nullptr),
        m_deformableVolumeMeshInputContainer(nullptr), m_surfaceDeformableBodyInputContainer(nullptr), m_muteCallbacks(false), m_contextId(0ull)
    {
        CARB_PROFILE_ZONE(0, "MeshCookingContext::MeshCookingContext");

        m_cookingResult.request = &m_cookingRequest;

        saveCallbackFromRequest(m_cookingRequest);

        // A deformable volume mesh cook carries its whole input in volumeMeshView and is the one
        // data type the processor never triangulates, so an empty primMeshView is expected here.
        const bool inputIsSelfContained = m_cookingRequest.dataType == PhysxCookingDataType::eDEFORMABLE_VOLUME_MESH;

        if (!inputIsSelfContained && m_cookingRequest.primMeshView.isEmpty())    // If user supplied optionalMeshKey then meshView will be empty
        {
            // REQ-COOK-SOURCE-001: every request is mesh-view mode now (eINPUT_MODE_FROM_PRIM_ID
            // removed) -- an empty mesh view here is unrecoverable, there is no USD re-read fallback.
            m_cookingResult.result = PhysxCookingResult::eERROR_INVALID_PRIM;
            return;
        }

        // An asynchronous eINPUT_MODE_FROM_PRIM_MESH_VIEW request cannot be allowed to read the
        // caller's view once this constructor returns: the build is only queued here and first
        // runs from a later pumpAsyncContext(), long after the submitting frame released the
        // memory the view points at. The copy is taken by each input container's constructor --
        // below, and still on this (the submitting) thread -- see
        // PhysicsInputContainerBase::shouldSnapshotInputNow().
        m_triangulationContainer =
            new PhysicsTriangulationInputContainer(m_cookingResult, m_cookingRequest);

        // Request cooking data
        addExtDep(getRequestBuilder());
        getRequestBuilder().add(PHYSX_COOK_MESH_STR);
        getRequestBuilder().add(DATA_TYPE_STR, (uint32_t)m_cookingRequest.dataType);
        getRequestBuilder().add(COOKING_VERSION_STR, (uint32_t)cookingDataVersion);
        getRequestBuilder().add(TRIANGULATION_VERSION_STR, (uint32_t)PhysxCookingDataVersion_MeshTriangulation);
        getRequestBuilder().add(METERS_PER_UNIT_STR, m_cookingRequest.primMeshMetersPerUnit);
        getRequestBuilder().add(BUILD_GPU_DATA_STR, m_cookingRequest.options.hasFlag(
            omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData));
        getRequestBuilder().add(BUILD_TRIANGLE_ADJACENCIES_STR, m_cookingRequest.options.hasFlag(
            omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData));
        if (!m_cookingRequest.primMeshText.empty())
        {
            std::string primMeshString;
            primMeshString.insert(primMeshString.begin(), m_cookingRequest.primMeshText.data(),
                m_cookingRequest.primMeshText.data() + m_cookingRequest.primMeshText.size_bytes());
            getRequestBuilder().add(PRIM_MESH_TEXT, primMeshString.c_str());
        }

        const PhysxCookingDataType::Enum dataType =
            (PhysxCookingDataType::Enum)getRequestBuilder().getRequest().getAsDefault(DATA_TYPE_STR, (uint32_t)0);
        if (dataType == PhysxCookingDataType::eVOLUME_DEFORMABLE_BODY)
        {
            m_volumeDeformableBodyInputContainer = new PhysicsVolumeDeformableBodyInputContainer(m_cookingResult, m_cookingRequest);       
        }
        else if (dataType == PhysxCookingDataType::eDEFORMABLE_VOLUME_MESH)
        {
            m_deformableVolumeMeshInputContainer = new PhysicsDeformableVolumeMeshInputContainer(m_cookingResult, m_cookingRequest);
        }
        else if (dataType == PhysxCookingDataType::eSURFACE_DEFORMABLE_BODY)
        {
            m_surfaceDeformableBodyInputContainer = new PhysicsSurfaceDeformableBodyInputContainer(m_cookingResult, m_cookingRequest);
        }

        // Create a unique ID to see if the result was previously cached
        m_contextId = generateRandomKey<uint64_t>();
        getRequestBuilder().add(CONTEXT_ID, m_contextId);
    }

    virtual ~MeshCookingContext()
    {
        // The registry holds a reference to each registered container; it must let go of them
        // before they are deleted here, even on a path that reached neither postbuild() nor
        // cancel() (e.g. agent->requestBuild() failing after prebuild() registered them).
        unregisterContainers("~MeshCookingContext");

        // The containers are reference counted and registerContainer() acquires them. Release ours
        // instead of deleting them: when a cook is canceled while it runs, the registry can rarely
        // still hold its reference after unregisterContainer() succeeded, and drops it only when
        // the agent shuts down -- deleting the container here left it a dangling pointer.
        releaseContainer(m_triangulationContainer);
        releaseContainer(m_volumeDeformableBodyInputContainer);
        releaseContainer(m_deformableVolumeMeshInputContainer);
        releaseContainer(m_surfaceDeformableBodyInputContainer);
    }

    static void releaseContainer(carb::ujitso::IContainer* container)
    {
        if (container)
            container->release();
    }

    // Using the prim ID as the process owner ID
    virtual OwnerId ownerId() const override
    {
        return m_cookingRequest.primId;
    }

    // Using the cooking result cookedDataCRC as the request ID
    virtual RequestId requestId() const override
    {
        return m_cookingResult.cookedDataCRC.getFullHash();
    }

    // Performed finishing tasks that must be called from the main thread.
    // Called after all process steps are completed
    virtual void onComplete() override
    {
        CARB_PROFILE_ZONE(0, "MeshCookingContext::onComplete");

        // Call base context version do common state setting
        UjitsoProcessContext::onComplete();

        // The build's callback has returned by now, so it can no longer overwrite the result
        if (m_canceledBeforeFinished)
            m_cookingResult.result = PhysxCookingResult::eERROR_CANCELED;

        // Keep this outside of the if () block below, so that its data is available for any callbacks that are fired
        std::vector<PhysxCookedDataSpan> spans;

        if (m_cookingResult.result == PhysxCookingResult::eVALID)
        {
            // BRG - the getResultDataBlocks().size() > 0 check shouldn't strictly be needed, but mesh cooking
            // should always produce data blocks, so it's kept here as redundancy.
            if (getProcessResult() == OperationResult::SUCCESS && getResultDataBlocks().size() > 0)
            {
                // Build span array from cookingDataBlocks
                spans.reserve(getResultDataBlocks().size());
                for (std::vector<uint8_t>& block : getResultDataBlocks())
                    spans.push_back(PhysxCookedDataSpan(block.data(), block.size()));
                m_cookingResult.cookedData = spans.data();
                m_cookingResult.cookedDataNumElements = spans.size();
            }
            else
            {
                m_cookingResult.result = PhysxCookingResult::eERROR_COOKING_FAILED;
            }
        }

        // Finish request
        if (!m_muteCallbacks)
            for(auto& requestCallback: m_requestCallbacks)
                requestCallback(m_cookingResult);

        delete this;
    }

    // Executes necessary prebuild steps before the base class fires off the ujitso cooking request
    // Mesh cooking requires the mesh representation; currently this is passed through a ujitso container
    virtual void prebuild() override
    {
        CARB_PROFILE_ZONE(0, "MeshCookingContext::prebuild");

        UjitsoProcessContext::prebuild();

        // Kept for unregisterContainers(), which may run after the agent has been returned
        m_registry = getAgent()->registry;

        // Build the container to pass data through to the processor
        m_containerHandle = m_registry->registerContainer(*m_triangulationContainer);
        getRequestBuilder().add(PHYSICS_TRIANGULATION_INPUT_CONTAINER_NAME, m_containerHandle);

        const PhysxCookingDataType::Enum dataType =
            (PhysxCookingDataType::Enum)getRequestBuilder().getRequest().getAsDefault(DATA_TYPE_STR, (uint32_t)0);
        if (dataType == PhysxCookingDataType::eVOLUME_DEFORMABLE_BODY)
        {
            m_volumeDeformableBodyContainerHandle = m_registry->registerContainer(*m_volumeDeformableBodyInputContainer);
            getRequestBuilder().add(PHYSICS_VOLUME_DEFORMABLE_BODY_INPUT_CONTAINER_NAME, m_volumeDeformableBodyContainerHandle);
        }
        else if (dataType == PhysxCookingDataType::eDEFORMABLE_VOLUME_MESH)
        {
            m_deformableVolumeMeshContainerHandle = m_registry->registerContainer(*m_deformableVolumeMeshInputContainer);
            getRequestBuilder().add(PHYSICS_DEFORMABLE_VOLUME_MESH_INPUT_CONTAINER_NAME, m_deformableVolumeMeshContainerHandle);
        }
        else if (dataType == PhysxCookingDataType::eSURFACE_DEFORMABLE_BODY)
        {
            m_surfaceDeformableBodyContainerHandle = m_registry->registerContainer(*m_surfaceDeformableBodyInputContainer);
            getRequestBuilder().add(PHYSICS_SURFACE_DEFORMABLE_BODY_INPUT_CONTAINER_NAME, m_surfaceDeformableBodyContainerHandle);
        }

        // Publish the handles to unregisterContainers() only once all of them are stored
        m_containersRegistered.store(true, std::memory_order_release);
    }

    bool unregisterContainerWithLogging(const ContainerHandle& handle, const char* containerName, const char* caller)
    {
        constexpr uint32_t timeout = 60000;
        const OperationResult result = m_registry->unregisterContainer(handle, timeout);
        switch (result)
        {
        case OperationResult::SUCCESS:
            return true;
        case OperationResult::TIMEOUT_ERROR:
            CARB_LOG_ERROR("MeshCookingContext::%s: Unable to unregister %s in %ds.",
                caller, containerName, timeout / 1000);
            return false;
        default:
            CARB_LOG_ERROR("MeshCookingContext::%s: unregisterContainer returned error value %d.",
                caller, (uint32_t)result);
            return false;
        }
    }

    // Unregisters every container prebuild() registered, exactly once.
    //
    // A cook canceled while it is running gets here several times: from postbuild(), on the
    // UJITSO worker that completes the build, and from cancel(), on the thread retiring the
    // cooking context -- which a stage detach calls more than once (cancelAllTasks, then
    // destroyAsyncContext and ~UjitsoProcessManager) -- possibly while postbuild() is running.
    // unregisterContainer() releases the registry's reference to the container, so a second
    // call for the same handle is refused once the first has finished, and over-releases the
    // registry's HandleDatabase entry while it has not: carb then aborts on the worker with
    // "HandleDatabase::release: Precondition failure: `mapped` is already destroyed".
    // Whichever caller claims the registration first tears the containers down; the others
    // find it gone.
    void unregisterContainers(const char* caller)
    {
        if (!m_containersRegistered.exchange(false, std::memory_order_acq_rel))
            return;

        unregisterContainerWithLogging(m_containerHandle, "triangulation container", caller);

        if (m_cookingRequest.dataType == PhysxCookingDataType::eVOLUME_DEFORMABLE_BODY)
        {
            unregisterContainerWithLogging(m_volumeDeformableBodyContainerHandle, "volume deformable body", caller);
        }
        else if (m_cookingRequest.dataType == PhysxCookingDataType::eDEFORMABLE_VOLUME_MESH)
        {
            unregisterContainerWithLogging(m_deformableVolumeMeshContainerHandle, "deformable volume mesh", caller);
        }
        else if (m_cookingRequest.dataType == PhysxCookingDataType::eSURFACE_DEFORMABLE_BODY)
        {
            unregisterContainerWithLogging(m_surfaceDeformableBodyContainerHandle, "surface deformable body", caller);
        }
    }


    // Execute necessary postbuild steps, called from the ujitso build finished callback
    // Here we extract the data from the ujitso build and fill the physx cooking result structs
    virtual bool postbuild(RequestHandle requestHandle, ResultHandle resultHandle, OperationResult result) override
    {
        CARB_PROFILE_ZONE(0, "MeshCookingContext::postbuild");

        // Call base class callback; this will fill data buffers
        bool success = UjitsoProcessContext::postbuild(requestHandle, resultHandle, result);

        // Assume it's a cache miss until proven otherwise
        m_cookingResult.resultSource = PhysxCookingComputeResult::eRESULT_CACHE_MISS;

        if (success)
        {
            // Extract context ID from metadata
            const std::vector<uint8_t>& metadata = getResultMetaData();
            success = (metadata.size() == sizeof(uint64_t));
            if (success)
            {
                const uint64_t contextId = *(const uint64_t*)metadata.data();
                if (contextId != m_contextId)
                    m_cookingResult.resultSource = PhysxCookingComputeResult::eRESULT_CACHE_HIT_UJITSO;

                m_cookingResult.result = PhysxCookingResult::eVALID;

                // Request triangulation data if needed
                if (m_cookingRequest.triangulation.isNeeded())
                {
                    UjitsoProcessContext triangulation(getResourceManager(), cacheBehavior());
                    addExtDep(triangulation.getRequestBuilder());
                    triangulation.getRequestBuilder().add(PHYSX_TRIANGULATE_MESH_STR);
                    triangulation.getRequestBuilder().add(TRIANGULATION_VERSION_STR,
                                                            (uint32_t)PhysxCookingDataVersion_MeshTriangulation);
                    triangulation.getRequestBuilder().add(PHYSICS_TRIANGULATION_INPUT_CONTAINER_NAME, m_containerHandle);

                    triangulation.requestBuild(kTimeOutInfinite, getAgent());
                    triangulation.waitRequest();

                    // We need to save the triangulation buffers in the upper ujitso context otherwise the triangulation
                    // view will point to memory that goes out of scope at the end this if statement
                    m_triangulationResultDataBlocks = std::move(triangulation.getResultDataBlocks());

                    // Build triangulation view from the freshly moved getAdditionalResultDataBlocks
                    success = m_triangulationResultDataBlocks.size() == 1 && 
                         buildTriangulationViewFromData(m_cookingResult.triangulationView,
                                                        m_cookingResult.triangulationMaxMaterialIndex,
                                                        m_triangulationResultDataBlocks[0]);
                    if (!success)
                    {
                        CARB_LOG_ERROR("MeshCookingContext::postbuild: triangulation data request failed.");
                        m_cookingResult.result = PhysxCookingResult::eERROR_COOKING_FAILED;
                    }
                }
            }
            else
                CARB_LOG_ERROR("metadata.size() == sizeof(uint64_t) is false.");
        }
        else
        if (result == OperationResult::FAILURE)
        {
            m_cookingResult.result = PhysxCookingResult::eERROR_COOKING_FAILED;
            const char* text = m_cookingRequest.primMeshText.data();
            CARB_LOG_WARN("UjitsoMeshCookingContext: cooking failure%s%s", text ? " for " : "", text ? text : "");
        }

        // tear down the input containers, unless a concurrent cancel() already has
        unregisterContainers("postbuild");

        return success;
    }

    // Updates the cooking result when cancel is requested
    // If invokeCallbacks is false, all stored callbacks are erased to prevent them from being called
    virtual void cancel(bool invokeCallbacks) override
    {
        CARB_PROFILE_ZONE(0, "MeshCookingContext::cancel");

        if (!invokeCallbacks)
            m_muteCallbacks = true;

        // A build that has not finished yet is reported as canceled. Its callback may still be running
        // and write m_cookingResult, so onComplete() sets the result once that has returned.
        if (getState() != Finishing)
            m_canceledBeforeFinished = true;

        UjitsoProcessContext::cancel(invokeCallbacks);

        // tear down the input containers; a no-op if prebuild() has not registered them yet, or if
        // postbuild() or an earlier cancel() already did
        unregisterContainers("cancel");
    }

    typedef std::vector<omni::function<void(const PhysxCookingComputeResult& result)>> CallbackRequestVector;

    PhysxCookingComputeRequest          m_cookingRequest;
    PhysxCookingComputeResult           m_cookingResult;
    const char*                         m_collisionTypeSchemaToken;
    bool                                m_canHaveMultipleBuffers;
    int                                 m_cookingDataVersion;

    std::vector<std::vector<uint8_t>>   m_triangulationResultDataBlocks;
 
    PhysicsTriangulationInputContainer* m_triangulationContainer;
    ContainerHandle                     m_containerHandle;

    PhysicsVolumeDeformableBodyInputContainer*  m_volumeDeformableBodyInputContainer;
    ContainerHandle                             m_volumeDeformableBodyContainerHandle;

    PhysicsDeformableVolumeMeshInputContainer*  m_deformableVolumeMeshInputContainer;
    ContainerHandle                             m_deformableVolumeMeshContainerHandle;

    PhysicsSurfaceDeformableBodyInputContainer* m_surfaceDeformableBodyInputContainer;
    ContainerHandle                             m_surfaceDeformableBodyContainerHandle;

    carb::ujitso::IRegistry*            m_registry = nullptr;
    // Set by prebuild() once the containers above are registered; claimed by unregisterContainers()
    std::atomic<bool>                   m_containersRegistered{ false };

    CallbackRequestVector               m_requestCallbacks;
    bool                                m_muteCallbacks;
    // Set by cancel() before the build finished; onComplete() then reports eERROR_CANCELED
    bool                                m_canceledBeforeFinished = false;

    uint64_t                            m_contextId;
};

} // namespace physx
} // namespace omni
