// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Defines.h>

namespace omni
{
namespace kvdb
{

struct IKeyValueDatabase;

// A single instance of a key value database
// As a convenience the KeyValueDatabase interface pointer
// is stored in the instance struct
struct KeyValueDatabaseInstance
{
    IKeyValueDatabase* m_interface; // A convenience pointer to the IKeyValueDatabase plugin interface associated with
                                    // this database instance
};


/**
 * Callback when the SetKeyValue operation is complete
 *
 * @param key : Key we set
 * @param success : True if we were successfully able to set the database entry, false if we could not
 * @param userPtr : User provided user pointer
 */
using SetKeyValueCallback = void(CARB_ABI*)(const char* key, bool success, void* userPtr);


/**
 * Callback when a get key value request is ready
 *
 * @param key : Key we were retrieving
 * @param data : Data associated with this key, null if the key did not exist. Data only valid during this callback.
 * Owned by the database plugin.
 * @param dataLen : Size of the data
 * @param userPtr : User provided user pointer
 */
using GetKeyValueCallback = void(CARB_ABI*)(const char* key, const void* data, uint64_t dataLen, void* userPtr);


/**
 * Callback when a delete key operation is completed
 *
 * @param key : Key we were trying to delete
 * @param success : True if we successfully deleted this database entry, false if didn't exist
 * @param userPtr : User provided user pointer
 */
using DeleteKeyCallback = void(CARB_ABI*)(const char* key, bool success, void* userPtr);

/**
 * Callback when an 'exists key' operation is completed
 *
 * @param key : Key we were trying to test for
 * @param success : True if the key exists in the database, false if it does not
 * @param userPtr : User provided user pointer
 */
using ExistsKeyCallback = void(CARB_ABI*)(const char* key, bool exists, void* userPtr);

/**
 * Callback for a scan result, if 'key' is null, then it signals the end of the scan results
 *
 * @param searchKey : The original search key provided by the user from the 'scan' operation
 * @param key : Key which matched the scan request or, if null, indicates the scan operation is complete
 * @return Return true to continue scanning, false to abort the scan operation early
 * @param userPtr : User supplied user pointer
 */
using ScanCallback = bool(CARB_ABI*)(const char* searchKey, const char* key, void* userPtr);

struct IKeyValueDatabase
{

    CARB_PLUGIN_INTERFACE("omni::server::IKeyValueDatabase", 0, 1);

    /**
     * Sets a key value pair in the database
     *
     * @param kinstance : The instance of the key value database
     * @param keyname : The name of the key in the database
     * @param setOnlyIfNew : If this is true, then we only set this key if it does not already exist
     * @param data  : The data (value) associated with this key as a binary blob (no ownership transfered)
     *                The caller still owns this data and may do whatever it wants to with it on function
     *                return. If the database implementation needs it's own copy, it will do a deep copy
     *                internally.
     * @param dataLen : The length of the data associated with this key
     * @param callback : Pointer to the callback function once this operation is completed
     * @param userPtr : User pointer to return in the callback
     */
    void(CARB_ABI* setKeyValue)(KeyValueDatabaseInstance* kinstance,
                                const char* keyname,
                                bool setOnlyIfNew,
                                const void* data,
                                uint64_t dataLen,
                                SetKeyValueCallback callback,
                                void* userPtr);

    /**
     * Gets an entry of the key value database
     *
     * @param kinstance : The instance of the database we are operating on
     * @param keyName : The key we are retrieving
     * @param callback : Pointer to the callback function once the getKeyValue operation is completed, if the key does
     * not exist that will be indicated in the callback as a null pointer to any data
     * @param userPtr : User pointer to return with the callback
     */
    void(CARB_ABI* getKeyValue)(KeyValueDatabaseInstance* kinstance,
                                const char* keyName,
                                GetKeyValueCallback callback,
                                void* userPtr);

    /**
     * Delete a key from the database if it exists
     *
     * @param kinstance : The instance of the kvdb
     * @param keyName : The name of the key to delete
     * @param callback : Pointer to the callback function when the delete operation has completed.
     *                   The callback will occur whether the delete succeeded or not, and the 'success'
     *                   boolean return value will indicate that state.
     * @param userPtr : User pointer to provide in the callback
     */
    void(CARB_ABI* deleteKey)(KeyValueDatabaseInstance* kinstance,
                              const char* keyName,
                              DeleteKeyCallback callback,
                              void* userPtr);

    /**
     * Returns true if the key provided exists in the database
     *
     * @param kinstance : The instance of the database we are operating on
     * @param keyName : The name of the key we are testing for existence
     * @param callback : Callback to return whether the key exists or not
     * @param userPtr : User pointer to provide in the callback
     */
    void(CARB_ABI* existsKey)(KeyValueDatabaseInstance* kinstance,
                              const char* keyName,
                              ExistsKeyCallback callback,
                              void* userPtr);

    /**
     * Scans the database for keys which match the provided string
     *
     * @param kinstance : The instance of the database we are operating on
     * @param matchKeyName : The key to match against using '\*' to represent a wildcard match.
     *                       For example, to retrieve all keys beginning with 'KEY' you would use: "KEY\*"
     *                       To get all keys which begin with 'KEY' and end with '.txt' you would use: "KEY\*.txt"
     * @param callback : A pointer to a callback function to retrieve the scan results
     * @param userPtr : A user pointer which is returned in the scan callback
     */
    void(CARB_ABI* scan)(KeyValueDatabaseInstance* kinstance,
                         const char* matchKeyName,
                         ScanCallback callback,
                         void* userPtr);

    /**
     * Create an instance of a key value database located in the provided system path or connection point
     *
     * @param databasePath : The system path on the local drive where the database is located and/or remote connection
     * point if it's a network database
     * @param argc : Number of arguments passed into the database instance
     * @param argv : Pointers to arguments which control database parameters. Parameters are unique per database type
     *
     * Supported command line arguments are:
     *
     * delete_on_startup = <true/false> signals if the database should be reset on initialization or not.
     *
     * @return Returns a pointer to the key value database instance if it could be created
     */
    KeyValueDatabaseInstance*(CARB_ABI* createInstance)(const char* databasePath, uint32_t argc, const char** argv);

    /**
     * Release a previously created kvdb instance
     *
     * @param kinstance : A pointer to a previously created kvdb instance
     *                    Note, any outstanding database requests will be completed before
     *                    this instance will be released.
     */
    void(CARB_ABI* releaseKeyValueDatabaseInstance)(KeyValueDatabaseInstance* kinstance);
};


} // namespace kvdb
} // namespace omni
