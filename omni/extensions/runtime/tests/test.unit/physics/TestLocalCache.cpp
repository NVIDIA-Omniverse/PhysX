// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include <private/omni/localcache/ILocalCache.h>

#include "PhysicsTools.h"

using namespace omni::localcache;

//-----------------------------------------------------------------------------
// Test Local cache 
TEST_CASE("Local Cache",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")

{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    omni::localcache::ILocalCache* localCache = physicsTests.getApp()->getFramework()->tryAcquireInterface<omni::localcache::ILocalCache>();
    REQUIRE(localCache);

    LocalCacheInstance* lcInst = localCache->create("test", 1024, true);
    REQUIRE(lcInst);

    SUBCASE("Set/Get Cache Data")
    {
        bool val = true;
        localCache->setCacheData(lcInst, "testKey", (bool*)&val, sizeof(bool));

        CHECK(localCache->getCacheDataCount(lcInst) == 1);

        bool retVal = false;
        uint64_t dataSize = 0;
        const void* retData = localCache->getCacheData(lcInst, "testKey", dataSize);
        REQUIRE(retData);
        CHECK(dataSize == sizeof(bool));

        retVal = *((bool*)retData);
        CHECK(retVal == true);

        localCache->releaseCacheData(lcInst, retData);

        CHECK(localCache->existsKey(lcInst, "testKey"));
    }

    localCache->release(lcInst);
}
