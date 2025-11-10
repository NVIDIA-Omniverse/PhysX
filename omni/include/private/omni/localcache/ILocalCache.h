// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Defines.h>

namespace omni
{
namespace localcache
{

struct ILocalCache;

struct LocalCacheInstance
{
    ILocalCache* m_interface{ nullptr };
};

/**
 * Callback for a scan result, if 'key' is null, then it signals the end of the scan results
 *
 * @param searchKey : The original search key provided by the user from the 'scan' operation
 * @param key : Key which matched the scan request or, if null, indicates the scan operation is complete
 * @return Return true to continue scanning, false to abort the scan operation early
 * @param userPtr : User supplied user pointer
 */
using ScanCallback = bool(CARB_ABI*)(const char* searchKey, const char* key, void* userPtr);

struct ILocalCache
{
    CARB_PLUGIN_INTERFACE("omni::localcache::ILocalCache", 1, 0)

    /**
     * Create an instance of a local cache
     *
     * @param cacheName: The name of the local cache (must be a valid folder name)
     * @param maximumCacheSize: The maximum size of all records which should be stored at any give time
     * @param deleteOnStartup : Debug option to force the cached to be reset to an empty state on startup
     *
     * @return : Returns the instance of the LocalCache is it could be created
     */
    LocalCacheInstance*(CARB_ABI* create)(const char* cacheName, uint64_t maximumCacheSize, bool deleteOnStartup);

    /**
     * Set an item in the cache
     *
     * @param key: The unique key name for this cached item
     * @param data: The data to be cached
     * @param dataLen: The length of the data blob to be cached
     */
    void(CARB_ABI* setCacheData)(LocalCacheInstance* lci, const char* key, const void* data, uint64_t dataLen) = 0;

    /**
     * Retrieve an item from the cache
     *
     * @param key: The unique key name for the cached item
     * @param dataLen : A reference that will get the length of the cached data item
     *
     * @return : Returns a pointer to the cached data, caller should use 'releaseCacheData' when they are done with it.
     *           Failure to call 'releaseCacheData' will result in a memory leak, so be sure to pair them up.
     */
    const void*(CARB_ABI* getCacheData)(LocalCacheInstance* lci, const char* key, uint64_t& dataLen);

    /**
     * Release memory associated with a previous call to 'getCacheData'
     */
    void(CARB_ABI* releaseCacheData)(LocalCacheInstance* lci, const void* data);

    /**
     * Reports the maximum size of the cache
     *
     * @return : Returns the maximum size of this cache
     */
    uint64_t(CARB_ABI* getMaximumCacheSize)(LocalCacheInstance* lci);

    /**
     * Reports the current size of all records in the cache
     *
     * @return : Returns the current size of all records in the cache
     */
    uint64_t(CARB_ABI* getCurrentCacheSize)(LocalCacheInstance* lci);

    /**
     * Reports the number of records stored in the cache
     *
     * @return : Returns the number of records currently stored in the cache
     */
    uint64_t(CARB_ABI* getCacheDataCount)(LocalCacheInstance* lci);


    /**
     * Scans the database for keys which match the provided string
     *
     * @param kinstance : The instance of the cache database we are operating on
     * @param matchKeyName : The key to match against using '\*' to represent a wildcard match.
     *                       For example, to retrieve all keys beginning with 'KEY' you would use: "KEY\*"
     *                       To get all keys which begin with 'KEY' and end with '.txt' you would use: "KEY\*.txt"
     * @param callback : A pointer to a callback function to retrieve the scan results
     * @param userPtr : A user pointer which is returned in the scan callback
     */
    void(CARB_ABI* scan)(LocalCacheInstance* kinstance, const char* matchKeyName, ScanCallback callback, void* userPtr);

    /**
     * Method to determine if a key exists in the local cache or not
     *
     * @param kinstance : The instance of the cache database we are operating on
     * @param key : The key to match against
     * @return : Returns true if the key exists
     */
    bool(CARB_ABI* existsKey)(LocalCacheInstance* lci, const char* key);


    /**
     * Release the LocalCache instance
     */
    void(CARB_ABI* release)(LocalCacheInstance* lci);
};

} // namespace localcache
} // namespace omni
