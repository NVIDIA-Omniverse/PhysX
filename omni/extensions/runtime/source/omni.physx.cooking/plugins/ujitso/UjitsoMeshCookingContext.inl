// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../service/CookingComputeService.h"

#include "UjitsoMeshUtils.inl"
#include "UjitsoHashUtils.inl"

#include "UjitsoTriangulationContainer.h"
#include "UjitsoProcessManager.h"

#include "LoggingUtils.h"

#include <carb/ujitso/UjitsoUtils.inl>
#include <omni/physx/IPhysxCookingService.h>
#include <common/utilities/Utilities.h> // intToPath

using namespace carb::ujitso;
using namespace pxr;
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

// Uses copyUsdMeshDataAndRespan to create a new copy of data pointed to by a view, and modifies the view
// to point to the new copy
static void buildUsdMeshAndRemakeView(CookingInputUSDMesh& mesh, PhysxCookingMeshView& view)
{
    // Copy carb::Float3 -> pxr::GfVec3f
    // Since both are simple classes with no inheritance and a simple 3-float data layout, this should be safe
    // even though GfVec3 may not be trivially copyable by its strict definition
    static_assert(sizeof(GfVec3f) == sizeof(carb::Float3), "copyIntDataAndRespan: Data size mismatch");
    mesh.pointsValue.assign(reinterpret_cast<const GfVec3f*>(view.points.data()),
                            reinterpret_cast<const GfVec3f*>(view.points.data()) + view.points.size());

    // Cast and assign to mesh arrays, int and uint32_t alias (assuming they're the same size)
    static_assert(sizeof(int) == sizeof(uint32_t), "copyIntDataAndRespan: Data size mismatch");
    mesh.indicesValue.assign(reinterpret_cast<const int*>(view.indices.data()),
                             reinterpret_cast<const int*>(view.indices.data()) + view.indices.size());
    mesh.facesValue.assign(reinterpret_cast<const int*>(view.faces.data()),
                           reinterpret_cast<const int*>(view.faces.data()) + view.faces.size());
    mesh.holesValue.assign(reinterpret_cast<const int*>(view.holeIndices.data()),
                           reinterpret_cast<const int*>(view.holeIndices.data()) + view.holeIndices.size());

    // These are the same data type
    mesh.faceMaterials.assign(view.faceMaterials.begin(), view.faceMaterials.end());
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
     * \param[in]   collisionTypeSchemaToken    The USD schema token for the cooked data type.
     * \param[in]   canHaveMultipleBuffers      Whether or not the data type may be stored using multiple buffers.
     *                                          Currently only convex decomposition requires that this be true.
     * 
     * \return a new MeshCookingContext.
     */
    static MeshCookingContext* create(UjitsoResourceManager* resourceManager, CacheBehaviorType cacheBehavior,
                                      const PhysxCookingComputeResult& cookingResult, int cookingDataVersion,
                                      TfToken collisionTypeSchemaToken, bool canHaveMultipleBuffers = false)
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
                       TfToken collisionTypeSchemaToken, bool canHaveMultipleBuffers) :
        UjitsoProcessContext(resourceManager, cacheBehavior),
        m_cookingRequest(*cookingResult.request), m_cookingResult(cookingResult),
        m_collisionTypeSchemaToken(collisionTypeSchemaToken), m_canHaveMultipleBuffers(canHaveMultipleBuffers),
        m_cookingDataVersion(cookingDataVersion), m_triangulationContainer(nullptr), m_muteCallbacks(false),
        m_contextId(0ull)
    {
        CARB_PROFILE_ZONE(0, "MeshCookingContext::MeshCookingContext");

        m_cookingResult.request = &m_cookingRequest;

        saveCallbackFromRequest(m_cookingRequest);

        CookingStageAndPrim stageAndPrim;

        if (m_cookingRequest.primMeshView.isEmpty())    // If user supplied optionalMeshKey then meshView will be empty
        {
            // If the mesh view is empty we require valid prim data
            if (m_cookingRequest.dataInputMode != PhysxCookingComputeRequest::eINPUT_MODE_FROM_PRIM_ID)
            {
                m_cookingResult.result = PhysxCookingResult::eERROR_INVALID_PRIM;
                return;
            }

            if (!ICookingComputeService::getStageAndPrim(m_cookingResult, m_cookingRequest, stageAndPrim) ||
                !ICookingComputeService::fillMeshView(m_cookingResult, m_cookingRequest, stageAndPrim))
            {
                m_cookingResult.result = PhysxCookingResult::eERROR_INVALID_PRIM;
                m_cookingRequest.onFinished(m_cookingResult);
                return;
            }
        }
        else
        {
            // If async, we can't be sure the data referenced by the mesh view will be valid when processing
            // actually occurs, so copy the data
            if (m_cookingRequest.options.hasFlag(PhysxCookingComputeRequest::Options::kComputeAsynchronously))
                buildUsdMeshAndRemakeView(stageAndPrim.ownedMesh, m_cookingRequest.primMeshView);
        }

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

        // Create a unique ID to see if the result was previously cached
        m_contextId = generateRandomKey<uint64_t>();
        getRequestBuilder().add(CONTEXT_ID, m_contextId);
    }

    virtual ~MeshCookingContext()
    {
        delete m_triangulationContainer;
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

        // Keep this outside of the if () block below, so that its data is available for any callbacks that are fired
        std::vector<PhysxCookedDataSpan> spans;

        if (m_cookingResult.result == PhysxCookingResult::eVALID)
        {
            // BRG - the second term in the conditional, getResultDataBlocks().size() > 0, should
            // not be needed and someday may prove to be unwanted.  But if we assume that mesh cooking
            // should always produce data blocks, then it's safe to keep here and provides some redundancy.
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

        // Build the container to pass data through to the processor
        m_containerHandle = getAgent()->registry->registerContainer(*m_triangulationContainer);
        getRequestBuilder().add(PHYSICS_TRIANGULATION_INPUT_CONTAINER_NAME, m_containerHandle);
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

        // tear down the triangulation container
        constexpr uint32_t timeout = 60000;
        const OperationResult unregisterResult = getAgent()->registry->unregisterContainer(m_containerHandle, timeout);
        switch (unregisterResult)
        {
        case OperationResult::SUCCESS:
            break;
        case OperationResult::TIMEOUT_ERROR:
            CARB_LOG_ERROR("MeshCookingContext::postbuild: Unable to unregister triangulation container in %ds.",
                           timeout / 1000);
            break;
        default:
            CARB_LOG_ERROR("MeshCookingContext::postbuild: unregisterContainer returned error value %d.",
                           (uint32_t)unregisterResult);
            break;
        }

        return success;
    }

    // Updates the cooking result when cancel is requested
    // If invokeCallbacks is false, all stored callbacks are erased to prevent them from being called
    virtual void cancel(bool invokeCallbacks) override
    {
        CARB_PROFILE_ZONE(0, "MeshCookingContext::cancel");

        if (!invokeCallbacks)
            m_muteCallbacks = true;

        if (getState() != Finishing)
            m_cookingResult.result = PhysxCookingResult::eERROR_CANCELED;

        UjitsoProcessContext::cancel(invokeCallbacks);

        // tear down the triangulation container if agent is available
        // it's apparently possible to cancel before prebuild is called)
        carb::ujitso::Agent* agent = getAgent();
        if (agent != nullptr)
        {
            constexpr uint32_t timeout = 60000;
            const OperationResult unregisterResult = agent->registry->unregisterContainer(m_containerHandle, timeout);
            switch (unregisterResult)
            {
            case OperationResult::SUCCESS:
                break;
            case OperationResult::TIMEOUT_ERROR:
                CARB_LOG_ERROR(
                    "MeshCookingContext::cancel: Unable to unregister triangulation container in %ds.", timeout / 1000);
                break;
            default:
                CARB_LOG_ERROR("MeshCookingContext::cancel: unregisterContainer returned error value %d.",
                               (uint32_t)unregisterResult);
                break;
            }
        }
    }

    typedef std::vector<omni::function<void(const PhysxCookingComputeResult& result)>> CallbackRequestVector;

    PhysxCookingComputeRequest          m_cookingRequest;
    PhysxCookingComputeResult           m_cookingResult;
    TfToken                             m_collisionTypeSchemaToken;
    bool                                m_canHaveMultipleBuffers;
    int                                 m_cookingDataVersion;

    std::vector<std::vector<uint8_t>>   m_triangulationResultDataBlocks;
 
    PhysicsTriangulationInputContainer* m_triangulationContainer;
    ContainerHandle                     m_containerHandle;

    CallbackRequestVector               m_requestCallbacks;
    bool                                m_muteCallbacks;

    uint64_t                            m_contextId;
};

} // namespace physx
} // namespace omni
