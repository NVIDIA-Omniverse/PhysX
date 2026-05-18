// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/profiler/Profile.h>

#include "UjitsoProcessManager.h"
#include "UjitsoResourceManager.h"
#include "UjitsoMeshUtils.inl"
#include "UjitsoHashUtils.inl"
#include "LoggingUtils.h"

using namespace carb::ujitso;
using namespace carb::datastore;

namespace omni
{
namespace physx
{

// UjitsoProcessContext methods

UjitsoProcessContext::UjitsoProcessContext(UjitsoResourceManager* resourceManager, CacheBehaviorType cacheBehavior) :
    RequestCallbackData({requestCallback, this, nullptr}),
    m_resourceManager(resourceManager), m_agent(nullptr), m_agentIsExternal(false), m_requestHandle{0},
    m_processResult(OperationResult::NOTBUILT_ERROR), m_state(Unqueued), m_cacheBehavior(cacheBehavior)
{
    CHECK_RETURN_ON_FAIL(m_resourceManager);    // The resource manager must be valid
}

UjitsoProcessContext::~UjitsoProcessContext()
{
    if (m_state == Running)
        CARB_LOG_WARN("UjitsoProcessContext::~UjitsoProcessContext: process still running.");

    // make sure the request is released and agent returned before the context goes away
    onBuildFinished();
}

UjitsoProcessContext::BuildRequestResult UjitsoProcessContext::requestBuild(uint32_t timeOutMs /*= kTimeOutInfinite*/,
                                                                            carb::ujitso::Agent* agent /*= nullptr*/)
{
    CARB_PROFILE_ZONE(0, "UjitsoProcessContext::requestBuild");

    CHECK_RETURN_VALUE_ON_FAIL(m_resourceManager, BuildRequestResult::eResourcesUnavailable);

    CHECK_RETURN_VALUE_ON_FAIL(!m_agent, BuildRequestResult::eContextInUse);

    if (!agent)
    {
        m_agentIsExternal = false;
        m_agent = m_resourceManager->acquireAgent(timeOutMs);
        if (!m_agent)
            return BuildRequestResult::eTimedOut;
    }
    else
    {
        m_agentIsExternal = true;
        m_agent = agent;
    }

    prebuild();

    // Add developer key to request
    m_requestBuilder.add(PHYSX_COOKING_REQUEST_DEVELOPER_KEY, m_resourceManager->getDeveloperKey());

    // Set request cache behavior
    m_requestBuilder.add(PHYSX_UJITSO_CACHE_BEHAVIOR, (uint32_t)m_cacheBehavior);

    // Request build from ujitso
    if (m_state == Unqueued)
    {
        m_state = Pending;
    }
    OperationResult ujitsoResult = m_agent->agent->requestBuild(
        *m_agent, m_requestBuilder.getRequest().getRequest(), *this, &m_requestHandle);

    CHECK_RETURN_VALUE_ON_FAIL(ujitsoResult == OperationResult::SUCCESS, BuildRequestResult::eUjitsoRequestFailed);

    int32_t expected = Pending;
    if (!m_state.compare_exchange_strong(expected, (int32_t)Running))
    {
        if (expected != Finishing)
        {
            CARB_LOG_ERROR("UjitsoProcessContext::requestBuild: m_state after agent->requestBuild is %d. "
                           "Expected either Pending (%d) or Finishing (%d).", expected, (int)Pending, (int)Finishing);
        }
    }

    return BuildRequestResult::eSuccess;
}

bool UjitsoProcessContext::destroyRequest()
{
    OperationResult res = OperationResult::INVALIDHANDLE_ERROR;
    if (m_agent && m_agent->agent && m_requestHandle.value != 0)
    {
        res = m_agent->agent->destroyRequest(m_requestHandle);
        m_requestHandle = {0};
    }

    return res == OperationResult::SUCCESS;
}

bool UjitsoProcessContext::waitRequest(uint32_t timeoutMs /*= kTimeOutInfinite*/)
{
    CARB_PROFILE_ZONE(0, "UjitsoProcessContext::waitRequest");

    return m_agent->agent->waitRequest(m_requestHandle, timeoutMs) == OperationResult::SUCCESS;
}

void UjitsoProcessContext::cancel(bool invokeCallbacks)
{
    CARB_UNUSED(invokeCallbacks);

    m_state = Finishing;
}

void UjitsoProcessContext::onComplete()
{
    // In case we got here through synchronous calls
    if (getState() == Finishing)
    {
        onBuildFinished();
    }
}

void UjitsoProcessContext::setBuildFinished()
{
    m_state = Finishing;
}

void UjitsoProcessContext::onBuildFinished()
{
    CARB_PROFILE_ZONE(0, "UjitsoProcessContext::onBuildFinished");

    if (m_agent)
    {
        // Make sure the request is destroyed before the agent
        destroyRequest();

        if (!m_agentIsExternal)
        {
            m_resourceManager->releaseAgent(m_agent);
        }

        m_agent = nullptr;
        m_agentIsExternal = false;
    }
}

void UjitsoProcessContext::prebuild()
{
}

bool UjitsoProcessContext::postbuild(RequestHandle requestHandle, ResultHandle resultHandle, OperationResult result)
{
    CARB_PROFILE_ZONE(0, "UjitsoProcessContext::postbuild");

    CHECK_RETURN_FALSE_ON_FAIL(m_agent);

    // Make sure the operation was successful
    m_processResult = result;
    CHECK_RETURN_FALSE_ON_FAIL(m_processResult == OperationResult::SUCCESS && "result parameter", carb::logging::kLevelWarn);

    // Get metadata
    uint32_t metaDataSize;
    m_processResult = m_agent->agent->getRequestMetaData(resultHandle, nullptr, &metaDataSize);
    CHECK_RETURN_FALSE_ON_FAIL(m_processResult == OperationResult::SUCCESS && "getRequestMetaData (size)");
    m_resultMetaData.resize(metaDataSize);
    if (metaDataSize)
    {
        m_processResult = m_agent->agent->getRequestMetaData(resultHandle, m_resultMetaData.data(), &metaDataSize);
        CHECK_RETURN_FALSE_ON_FAIL(m_processResult == OperationResult::SUCCESS && "getRequestMetaData (data)");
    }

    // Get external storage size
    uint32_t externalStorageCount = 0;
    m_processResult = m_agent->agent->getRequestExternalData(resultHandle, nullptr, &externalStorageCount);
    CHECK_RETURN_FALSE_ON_FAIL(m_processResult == OperationResult::SUCCESS && "getRequestExternalData (count)");

    // Get external storage
    std::vector<ExternalStorage> storage(externalStorageCount);
    m_processResult = m_agent->agent->getRequestExternalData(resultHandle, storage.data(), &externalStorageCount);
    CHECK_RETURN_FALSE_ON_FAIL(m_processResult == OperationResult::SUCCESS && "getRequestExternalData (data)");

    // Retrieve data blocks
    m_resultDataBlocks.resize(0);
    struct CBContext { std::vector<std::vector<uint8_t>>& blocks; } cbContext{ m_resultDataBlocks };
    cbContext.blocks.reserve(externalStorageCount);
    m_processResult = m_agent->store->get(getDefaultStorageContext(""),
        GetOptionsBuilder(), storage.data(), externalStorageCount, nullptr,
        [](void* context, const carb::datastore::GetCallbackResult& results)
        {
            if (results.result == OperationResult::SUCCESS && results.keyCount)
            {
                CBContext& cbContext = *(CBContext*)context;
                cbContext.blocks.push_back(std::vector<uint8_t>());
                std::vector<uint8_t>& block = cbContext.blocks.back();
                block.assign(results.dataBlocks[0], results.dataBlocks[0] + results.sizesInBytes[0]);
            }
        }, &cbContext
    );

    CHECK_RETURN_FALSE_ON_FAIL(m_processResult == OperationResult::SUCCESS && "get");

    // Sanity check
    if (m_resultDataBlocks.size() != (size_t)externalStorageCount)
        m_processResult = OperationResult::FAILURE;

    return true;
}

// UjitsoProcessContext static methods

void UjitsoProcessContext::requestCallback(
    void* ctx0, void* ctx1, RequestHandle requestHandle, ResultHandle resultHandle, OperationResult result)
{
    CARB_PROFILE_ZONE(0, "UjitsoProcessContext::requestCallback");

    UjitsoProcessContext* context = reinterpret_cast<UjitsoProcessContext*>(ctx0);

    CARB_UNUSED(ctx1);

    context->postbuild(requestHandle, resultHandle, result);

    context->setBuildFinished();
}

} // namespace physx
} // namespace omni
