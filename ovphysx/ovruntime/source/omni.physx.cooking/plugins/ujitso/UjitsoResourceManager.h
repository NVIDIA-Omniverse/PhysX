// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/ujitso/UjitsoAgent.h>
#include <carb/settings/SettingsUtils.h>
#include <carb/extras/Path.h>
#include <carb/datastore/DataStoreConfigurator.h>
#include <carb/tasking/TaskingUtils.h>
#include <omni/Function.h>

#include <string>
#include <set>
#include <map>
#include <mutex>

// Forward declarations
namespace carb
{
namespace dad
{
struct IDataGrid;
}

namespace filesystem
{
inline namespace v1
{
struct IFileSystem;
}
}
} // namespace carb

namespace omni
{
namespace physx
{

/**
 * Utility class to manage ujitso components and act as a resource to supply agents
 */
struct UjitsoResourceManager
{
    /**
     * Alias for datastore config parameters, used when creating/recreating resources
     */
    typedef carb::datastore::DataStoreConfigurator::Config DataStoreConfig;

    /**
     * Alias for datastore features
     */
    typedef carb::datastore::DataStoreConfigurator::Features DataStoreFeatures;

    /**
     * Result of resource request after it is processed.
     */
    struct RecreateResult
    {
        enum Enum
        {
            eSuccess, // Resources based on the request are now set to be used when acquireAgent() is called.
            eIgnored, // Resources based on the request already exist, no new resources were made.
            eCanceled, // Another request has superseded this one before the request could be executed.
            eFailed // Resources could not be created with the requested parameters
        };
    };

    /**
     * Alias for resource request callback
     */
    typedef omni::function<void(RecreateResult::Enum)> ResourceRequestCallback;

    /**
     * Create a new UjitsoResourceManager.
     *
     * \return a new UjitsoResourceManager if successful, nullptr otherwise.
     */
    static UjitsoResourceManager* create();

    /**
     * Releases the calling UjitsoResourceManager and deletes all allocated memory.  The pointer to
     * the object will be invalid after this call.
     */
    void release();

    /**
     * Acquire the persistent agent.
     *
     * \param[in]   timeOutMs   The time to wait for an agent to be available.
     *
     * \return the agent if successful, nullptr otherwise.
     */
    carb::ujitso::Agent* acquireAgent(uint32_t timeOutMs = carb::ujitso::kTimeOutInfinite);

    /**
     * Return the persistent agent.
     *
     * \param[in]   agent   The agent to return.
     *
     * \return true iff successful.
     */
    bool releaseAgent(carb::ujitso::Agent* agent);

    /**
     * Set a key which can be added to a request in order to make subsequent build requests unique.
     * The key is retrieved using getDeveloperKey().
     *
     * \param[in]   key 64-bit developer key.  The default value is 0.
     */
    void setDeveloperKey(uint64_t key = 0ull);

    /**
     * Attempt to create a globally unique 64-bit developer key.
     * The key is retrieved using getDeveloperKey().
     */
    void createUniqueDeveloperKey();

    /**
     * Retrieve the build request developer key set by setDeveloperKey() or createUniqueDeveloperKey().
     * If neither of those have been called, the default value of 0 is returned.
     */
    uint64_t getDeveloperKey();

private:
    struct ResourceConfig
    {
        ResourceConfig() : m_set(false)
        {
        }
        ResourceConfig(const ResourceConfig& other);

        ResourceConfig& operator=(const ResourceConfig& other);

        void setDataStoreConfig(const DataStoreConfig& config);

        const DataStoreConfig& getDataStoreConfig() const
        {
            return m_dataStoreConfig;
        }

        bool isSet() const
        {
            return m_set;
        }

        void reset();

        const std::string& getLocalDataStorePath() const
        {
            return m_localCachePathStorage;
        }

    private:
        void bufferAndReferenceStrings();

        DataStoreConfig m_dataStoreConfig;
        std::string m_localCachePathStorage;
        std::string m_remoteCachePathStorage;
        std::string m_remoteCacheDiscoveryPathStorage;

        bool m_set;
    };

    struct ResourceRequest
    {
        ResourceRequest() : m_callback(nullptr)
        {
        }

        void set(const DataStoreConfig& config, ResourceRequestCallback callback);
        void reset();
        bool isSet() const
        {
            return m_config.isSet();
        }

        const ResourceConfig& getConfig() const
        {
            return m_config;
        }
        ResourceRequestCallback getCallback() const
        {
            return m_callback;
        }

    private:
        ResourceConfig m_config;
        ResourceRequestCallback m_callback;
    };

    struct Resources
    {
        carb::datastore::DataStoreConfigurator* dataStoreConfigurator = nullptr;
        carb::ujitso::Agent* agent = nullptr;

        bool valid() const
        {
            return !!agent;
        }
    };

    // Private ctor & dtor, to ensure only a valid object is returned
    // and release() method is used
    UjitsoResourceManager();
    ~UjitsoResourceManager();

    // Internal functions to create/destroy persistent data store and agent
    bool initResources(Resources& resources);
    void clearResources(Resources& resources);

    // Internal misc.
    void finishResourceRequest(RecreateResult::Enum result);

    Resources m_resources;
    ResourceRequest m_request;
    uint64_t m_developerKey;

    carb::tasking::Future<> m_recreateResourceTask;

    // Resource synchronization
    std::atomic<int32_t> m_resourceRefCount;
    carb::tasking::SharedMutexWrapper m_resourceMutex;
    carb::tasking::MutexWrapper m_requestMutex;
    carb::tasking::ConditionVariableWrapper m_requestCondition;
    int m_requestInstruction; // 1 => proceed, 0 => wait, -1 => cancel
    carb::tasking::MutexWrapper m_processMutex;
    carb::tasking::ConditionVariableWrapper m_processCondition;
    int m_processInstruction; // 1 => true, 0 => false, -1 => terminate

    // Acquired interfaces
    carb::settings::ISettings* m_settings;
    carb::ujitso::IFactory* m_ujitsoFactory;
    carb::filesystem::IFileSystem* m_fileSystem;
    carb::tasking::ITasking* m_tasking;
};

} // namespace physx
} // namespace omni
