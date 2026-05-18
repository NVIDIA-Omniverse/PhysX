// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/ujitso/UjitsoUtils.inl>
#include <carb/extras/Hash.h>

namespace omni
{
namespace physx
{

// Forward declaration
struct UjitsoProcessManager;
struct UjitsoResourceManager;

/**
 * This object holds the context for a ujitso build process.  For async usage, a UjitsoProcessContext
 * should be scheduled with the UjitsoProcessManager.
 */
struct UjitsoProcessContext : public carb::ujitso::RequestCallbackData
{
    /**
     * The owner ID is used to identify the target of a build, e.g. a mesh prim path that will
     * use cooked mesh data.  If a process is scheduled with an owner ID that matches an already-running
     * process, then this ensures that only one re-scheduled job for the owner is kept.  This prevents thrashing
     * if several build parameter changes are fired off in a row.  If the ID is zero, it is ignored.
     */
    typedef uint64_t OwnerId;

    /**
     * The request ID should uniquely identify the data and parameters that go into the build process.  This is
     * typically a hash.  This allows for searching and prevents duplication of requests.
     */
    typedef carb::extras::hash128_t RequestId;

    /**
     * Result of requestBuild().
     */
    enum class BuildRequestResult
    {
        eContextInUse, ///< The context has already started a process
        eSuccess, ///< Agent was acquired and build request succeeded
        eResourcesUnavailable, ///< Build resources not available
        eTimedOut, ///< Build resources could not be acquired in the requested time
        eUjitsoRequestFailed ///< Agent was acquired but ujitso build request failed
    };

    /**
     * ctor
     *
     * \param[in]   resourceManager A valid UjitsoResourceManager pointer.
     * \param[in]   cacheBehavior   What level of caching to use.
     */
    UjitsoProcessContext(UjitsoResourceManager* resourceManager, carb::ujitso::CacheBehaviorType cacheBehavior);

    /**
     * dtor
     */
    virtual ~UjitsoProcessContext();

    /**
     * Access request builder which will be used to generate the ujitso build request.
     *
     * \return  reference to internal request builder.
     */
    carb::ujitso::RequestBuilder<>& getRequestBuilder()
    {
        return m_requestBuilder;
    }

    /**
     * Request ujitso build.  Use waitRequest to let the process finish if the function
     * returns BuildRequestResult::eSuccess.
     *
     * \param[in]   timeOutMs   (optional) How long to wait (in milliseconds) for resources to become available.
     *                          If they do not become available in this time, the function returns
     *                          BuildRequestResult::eTimedOut.  Defaults to infinite wait time.
     * \param[in]   agent       (optional) If not NULL, the agent to use for the build.  Otherwise an agent is
     *                          acquired from the resource manager.  Defaults to NULL.
     * \return build result (see BuildRequestResult).
     */
    BuildRequestResult requestBuild(uint32_t timeOutMs = carb::ujitso::kTimeOutInfinite,
                                    carb::ujitso::Agent* agent = nullptr);

    /**
     * Destroy the request associated with this context.
     *
     * \return true iff operation is successful.
     */
    bool destroyRequest();

    /**
     * Wait for the specified time for the process to finish.
     *
     * \param[in] timeoutMs The time in milliseconds to wait for process completion.
     *
     * \return true iff it completes withing the given time.
     */
    bool waitRequest(uint32_t timeoutMs = carb::ujitso::kTimeOutInfinite);

    /**
     * After waitRequest returns successfully, this function returns the build result.
     *
     * \return the build result.
     */
    carb::ujitso::OperationResult getProcessResult() const
    {
        return m_processResult;
    }

    /**
     * After waitRequest returns successfully, if getProcessResult() == carb::ujitso::OperationResult::SUCCESS,
     * then this returns a buffer containing the build result metadata.
     *
     * \return a buffer containing the build result metadata.
     */
    std::vector<uint8_t>& getResultMetaData()
    {
        return m_resultMetaData;
    }

    /**
     * After waitRequest returns successfully, if getProcessResult() == carb::ujitso::OperationResult::SUCCESS,
     * then this returns multiple data block buffers holding the build external data.
     *
     * \return multiple data block buffers containing the build external data.
     */
    std::vector<std::vector<uint8_t>>& getResultDataBlocks()
    {
        return m_resultDataBlocks;
    }

protected:
    /**
     * The process states, traversed during pump().
     */
    enum State
    {
        Unqueued,
        Pending,
        Running,
        Finishing
    };

    UjitsoResourceManager* getResourceManager()
    {
        return m_resourceManager;
    }

    carb::ujitso::Agent* getAgent()
    {
        return m_agent;
    }

    State getState() const
    {
        return (State)m_state.load();
    }

    carb::ujitso::CacheBehaviorType cacheBehavior() const
    {
        return m_cacheBehavior;
    }

    void onBuildFinished();

    virtual void prebuild();

    virtual bool postbuild(carb::ujitso::RequestHandle requestHandle,
                           carb::ujitso::ResultHandle resultHandle,
                           carb::ujitso::OperationResult result);

    virtual void cancel(bool invokeCallbacks);

    virtual void onComplete();

private:
    virtual OwnerId ownerId() const
    {
        return 0; /* invalid */
    } // For rescheduling pending work

    virtual RequestId requestId() const
    {
        return { 0, 0 }; /* invalid */
    } // To keep requests unique

    void onQueue()
    {
        m_state = Pending;
    }

    void setBuildFinished();

    static void requestCallback(void* ctx0,
                                void* ctx1,
                                carb::ujitso::RequestHandle requestHandle,
                                carb::ujitso::ResultHandle resultHandle,
                                carb::ujitso::OperationResult result);

    UjitsoResourceManager* m_resourceManager;
    carb::ujitso::Agent* m_agent;
    bool m_agentIsExternal;
    carb::ujitso::RequestBuilder<> m_requestBuilder;
    carb::ujitso::RequestHandle m_requestHandle;
    std::vector<uint8_t> m_resultMetaData;
    std::vector<std::vector<uint8_t>> m_resultDataBlocks;
    carb::ujitso::OperationResult m_processResult;
    std::atomic<int32_t> m_state;
    carb::ujitso::CacheBehaviorType m_cacheBehavior;

    // The process manager has permission to access low-level (private) processing functions.
    friend struct UjitsoProcessManager;
};

} // namespace physx
} // namespace omni
