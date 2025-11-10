// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include <private/omni/kvdb/IKeyValueDatabase.h>

#include "PhysicsTools.h"

using namespace omni::kvdb;

struct SuccessData
{
    std::string key;
    bool success;
};

struct GetValueData
{
    const char* key;
    bool data;
    uint64_t dataSize;
};

void setValueCallback(const char* key, bool success, void* userPtr)
{
    SuccessData* sv = (SuccessData*)userPtr;
    sv->key = key;
    sv->success = success;
}

void getValueCallback(const char* key, const void* data, uint64_t dataLen, void* userPtr)
{
    GetValueData* sv = (GetValueData*)userPtr;
    sv->key = key;
    sv->dataSize = dataLen;
    sv->data = *((bool*)data);
}

void deleteValueCallback(const char* key, bool success, void* userPtr)
{
    SuccessData* sv = (SuccessData*)userPtr;
    sv->key = key;
    sv->success = success;
}

void existsKeyCallback(const char* key, bool exists, void* userPtr)
{
    SuccessData* sv = (SuccessData*)userPtr;
    sv->key = key;
    sv->success = exists;
}

bool scanKeyCallback(const char* searchKey, const char* key, void* userPtr)
{
    if (key)
    {
        SuccessData* sv = (SuccessData*)userPtr;
        sv->key = key;
        sv->success = true;
    }
    return false;
}

//-----------------------------------------------------------------------------
// Test KVDB
TEST_CASE("Kvdb",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")

{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    omni::kvdb::IKeyValueDatabase* kvdb = physicsTests.getApp()->getFramework()->tryAcquireInterface<omni::kvdb::IKeyValueDatabase>();
    REQUIRE(kvdb);

    const char* argv[2] = { "delete_on_startup", "true" };
    KeyValueDatabaseInstance* kvdbInst = kvdb->createInstance("test", 2, argv);
    REQUIRE(kvdbInst);

    SUBCASE("Test Set Key")
    {
        bool testValue = true;
        SuccessData sv;
        sv.key = "";
        sv.success = false;
        kvdb->setKeyValue(kvdbInst, "testKey", true, (const void*)&testValue, sizeof(bool), setValueCallback, (void*)&sv);
        CHECK(sv.key == "testKey");
        CHECK(sv.success == true);
    }

    SUBCASE("Test Get Key")
    {
        bool testValue = true;
        kvdb->setKeyValue(kvdbInst, "testKey", true, (const void*)&testValue, sizeof(bool), nullptr, nullptr);

        GetValueData sv;
        kvdb->getKeyValue(kvdbInst, "testKey", getValueCallback, (void*) & sv);

        CHECK(sv.key == "testKey");
        CHECK(sv.data == true);
        CHECK(sv.dataSize == sizeof(bool));
    }

    SUBCASE("Test Delete Key")
    {
        bool testValue = true;
        kvdb->setKeyValue(kvdbInst, "testKey", true, (const void*)&testValue, sizeof(bool), nullptr, nullptr);

        SuccessData sv;
        sv.key = "";
        sv.success = false;
        kvdb->deleteKey(kvdbInst, "testKey", deleteValueCallback, (void*)&sv);

        CHECK(sv.key == "testKey");
        CHECK(sv.success == true);        
    }

    SUBCASE("Test Exists Key")
    {
        bool testValue = true;
        kvdb->setKeyValue(kvdbInst, "testKey", true, (const void*)&testValue, sizeof(bool), nullptr, nullptr);

        SuccessData sv;
        sv.key = "";
        sv.success = false;
        kvdb->existsKey(kvdbInst, "testKey", existsKeyCallback, (void*)&sv);

        CHECK(sv.key == "testKey");
        CHECK(sv.success == true);
    }

    SUBCASE("Test Scan Key")
    {
        bool testValue = true;
        kvdb->setKeyValue(kvdbInst, "testKey", true, (const void*)&testValue, sizeof(bool), nullptr, nullptr);

        SuccessData sv;
        sv.key = "";
        sv.success = false;
        kvdb->scan(kvdbInst, "testKey", scanKeyCallback, (void*)&sv);

        CHECK(sv.key == "testKey");
        CHECK(sv.success == true);
    }

    kvdb->releaseKeyValueDatabaseInstance(kvdbInst);
}
