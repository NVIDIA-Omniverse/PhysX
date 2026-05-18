// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"
#include "UjitsoResourceManager.h"
#include "UjitsoServiceUtils.h"
#include "UjitsoHashUtils.inl"
#include "LoggingUtils.h"

#include <carb/datasource/IDataSource.h>
#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <carb/InterfaceUtils.h>
#include <carb/ujitso/IDefaultUjitso.h>

#include <private/omni/physx/PhysxUsd.h>

#include <pxr/usd/usdGeom/mesh.h>

using namespace carb::ujitso;
using namespace carb::settings;
using namespace carb::filesystem;
using namespace carb::datastore;

namespace omni
{
namespace physx
{


// Resource reference is not allowed to reach or exceed this value
static constexpr int32_t k_resourceCountMaxValue =  0x3fffffff;


// Utility to test for equal datastore configs
static bool dataStoreConfigsEqual(const UjitsoResourceManager::DataStoreConfig& a,
                                  const UjitsoResourceManager::DataStoreConfig& b)
{
#define DSC_RETURN_NEQ_VAL(_p)  if (a._p != b._p) return false
#define DSC_RETURN_NEQ_STR(_p)  if (!a._p != !b._p || (a._p && std::strcmp(a._p, b._p))) return false

    // Enable GRPC datastore
    DSC_RETURN_NEQ_VAL(m_allowGRPCDataStore);

    // For OmniHub
    DSC_RETURN_NEQ_VAL(m_allowHubDataStore);

    // For Local DataStore
    DSC_RETURN_NEQ_STR(m_localCachePath);
    DSC_RETURN_NEQ_VAL(m_localMemoryCacheSize);

    DSC_RETURN_NEQ_VAL(m_allowNucleusDataStore);
    DSC_RETURN_NEQ_STR(m_remoteCachePath);
    DSC_RETURN_NEQ_VAL(m_useRemoteCacheDiscoveryForWrites);
    DSC_RETURN_NEQ_STR(m_remoteCacheDiscoveryPath);

    // To enable .csv profile dumping
    DSC_RETURN_NEQ_VAL(m_profile);

    return true;
}

/* UjitsoResourceManager methods */

UjitsoResourceManager* UjitsoResourceManager::create()
{
    UjitsoResourceManager* ujitsoManager = new UjitsoResourceManager();

    // Check that the new UjitsoResourceManager is valid, if not destroy and return nullptr
    if (!ujitsoManager->m_settings || !ujitsoManager->m_ujitsoFactory || !ujitsoManager->m_fileSystem ||
        !ujitsoManager->m_resources.valid())
    {
        ujitsoManager->release();
        ujitsoManager = nullptr;
    }

    return ujitsoManager;
}

void UjitsoResourceManager::release()
{
    delete this;
}

Agent* UjitsoResourceManager::acquireAgent(uint32_t timeOutMs /*= carb::ujitso::kTimeOutInfinite*/)
{
    CARB_PROFILE_ZONE(0, "UjitsoResourceManager::acquireAgent");

    if (timeOutMs == carb::ujitso::kTimeOutInfinite)
        m_resourceMutex.lock_shared();
    else
    if (!m_tasking->timedLockSharedMutex(m_resourceMutex, 1000000*timeOutMs))
        return nullptr; // Timed out

    Agent* agent = nullptr;

    if (m_resourceRefCount++ < k_resourceCountMaxValue)
    {
        agent = m_resources.agent;
    }
    else
    {
        m_resourceRefCount--;
        CARB_LOG_ERROR("UjitsoResourceManager::acquireAgent: overflow, agent not acquired.");
    }
    
    m_resourceMutex.unlock_shared();

    return agent;
}

bool UjitsoResourceManager::releaseAgent(Agent* agent)
{
    CARB_PROFILE_ZONE(0, "UjitsoResourceManager::releaseAgent");

    // Atomically catch the transition from 1 to 0, signalling that resources may be replaced
    int32_t condition = 1;
    if (m_resourceRefCount.compare_exchange_strong(condition, 0))
    {
        std::lock_guard<carb::tasking::MutexWrapper> requestLock(m_requestMutex);
        m_requestInstruction = 1;
        m_requestCondition.notify_one();
    }
    else
    if (m_resourceRefCount-- <= 0)
    {
        m_resourceRefCount++;
        CARB_LOG_ERROR("UjitsoResourceManager::releaseAgent: underflow.");
        return false;
    }
    
    return true;
}

void UjitsoResourceManager::setDeveloperKey(uint64_t key /*= 0ull*/)
{
    m_developerKey = key;
}

void UjitsoResourceManager::createUniqueDeveloperKey()
{
    m_developerKey = generateRandomKey<uint64_t>();
}

uint64_t UjitsoResourceManager::getDeveloperKey()
{
    return m_developerKey;
}

UjitsoResourceManager::UjitsoResourceManager() : m_developerKey(0),
    m_resourceRefCount(0), m_requestInstruction(0), m_processInstruction(0), m_settings(nullptr),
    m_ujitsoFactory(nullptr), m_fileSystem(nullptr), m_tasking(nullptr)
{
    // Acquire various interfaces used by this manager and store them off

    m_settings = carb::getCachedInterface<ISettings>();
    CHECK_RETURN_ON_FAIL(m_settings);

    m_ujitsoFactory = carb::getFramework()->tryAcquireInterface<IFactory>("carb.ujitsoagent.plugin");
    CHECK_RETURN_ON_FAIL(m_ujitsoFactory);

    m_fileSystem = carb::getCachedInterface<IFileSystem>();
    CHECK_RETURN_ON_FAIL(m_fileSystem);

    m_tasking = carb::getCachedInterface<carb::tasking::ITasking>();
    CHECK_RETURN_ON_FAIL(m_tasking);

    // Grab some resource info from UJITSO, it owns the resources now
    initResources(m_resources);
}

UjitsoResourceManager::~UjitsoResourceManager()
{
    // Reset the current request
    finishResourceRequest(RecreateResult::eCanceled);

    // Signal the recreate resource process that it may execute (it will do nothing since the request is now reset)
    {
        std::lock_guard<carb::tasking::MutexWrapper> processLock(m_processMutex);
        m_processInstruction = -1;
        m_processCondition.notify_one();
    }

    {
        std::lock_guard<carb::tasking::MutexWrapper> requestLock(m_requestMutex);
        m_requestInstruction = -1;
        m_requestCondition.notify_one();
    }

    // Wait for the recreate resource process
    constexpr int waitTimeSeconds = 300;
    if (m_recreateResourceTask.valid() && !m_recreateResourceTask.wait_for(std::chrono::seconds(waitTimeSeconds)))
        CARB_LOG_ERROR("UjitsoResourceManager::~UjitsoResourceManager: m_recreateResourceTask did not terminate "
                       "in %d seconds.", waitTimeSeconds);

    // Destroy all resources
    std::lock_guard<carb::tasking::SharedMutexWrapper> resourceLock(m_resourceMutex);
    clearResources(m_resources);
}

bool UjitsoResourceManager::initResources(Resources& resources)
{
    CARB_PROFILE_ZONE(0, "UjitsoResourceManager::initResources");

    CHECK_RETURN_FALSE_ON_FAIL(!resources.valid());

    // Get dataStoreConfigurator from UJITSO
    carb::ujitso::DefaultState* defaultState = carb::ujitso::getDefaultState();
    CHECK_RETURN_FALSE_ON_FAIL(defaultState);
    resources.dataStoreConfigurator = defaultState->dataStoreConfigurator;
    CHECK_RETURN_FALSE_ON_FAIL(resources.dataStoreConfigurator);
    if (!resources.dataStoreConfigurator->getDataStore())
    {
        clearResources(resources);
        return false;
    }

    resources.agent = carb::ujitso::getDefaultAgent();
    if (!resources.agent)
    {
        clearResources(resources);
        return false;
    }

    return true;
}

void UjitsoResourceManager::clearResources(Resources& resources)
{
    // clear out the agent, it is using the default agent, so don't delete it
    resources.agent = nullptr;

    // Datastore configurator is also owned by UJITSO
    resources.dataStoreConfigurator = nullptr;
}

void UjitsoResourceManager::finishResourceRequest(RecreateResult::Enum result)
{
    ResourceRequestCallback callback = m_request.getCallback();
    if (callback)
        callback(result);

    m_request.reset();
}


/* UjitsoResourceManager::ResourceConfig methods */

UjitsoResourceManager::ResourceConfig::ResourceConfig(const ResourceConfig& other) :
    m_dataStoreConfig(other.m_dataStoreConfig), m_set(other.m_set)
{
    bufferAndReferenceStrings();
}

UjitsoResourceManager::ResourceConfig& UjitsoResourceManager::ResourceConfig::operator = (const ResourceConfig& other)
{
    m_dataStoreConfig = other.m_dataStoreConfig;
    m_set = other.m_set;
    bufferAndReferenceStrings();
    return *this;
}

void UjitsoResourceManager::ResourceConfig::setDataStoreConfig(const DataStoreConfig& config)
{
    m_dataStoreConfig = config;
    bufferAndReferenceStrings();
    m_set = true;
}

#define COPY_AND_REF(_name) \
    if (m_dataStoreConfig._name) \
    { \
        _name##Storage = m_dataStoreConfig._name; \
        m_dataStoreConfig._name = _name##Storage.c_str(); \
    } \
    else \
        _name##Storage = ""

void UjitsoResourceManager::ResourceConfig::bufferAndReferenceStrings()
{
    COPY_AND_REF(m_localCachePath);
    COPY_AND_REF(m_remoteCachePath);
    COPY_AND_REF(m_remoteCacheDiscoveryPath);
}

void UjitsoResourceManager::ResourceConfig::reset()
{
    m_dataStoreConfig = DataStoreConfig();
    bufferAndReferenceStrings();
    m_set = false;
}


/* UjitsoResourceManager::ResourceRequest methods */

void UjitsoResourceManager::ResourceRequest::set(const DataStoreConfig& config, ResourceRequestCallback callback)
{
    m_config.setDataStoreConfig(config);
    m_callback = callback;
}

void UjitsoResourceManager::ResourceRequest::reset()
{
    m_config.reset();
    m_callback = nullptr;
}

}   // namespace physx
}   // namespace omni
