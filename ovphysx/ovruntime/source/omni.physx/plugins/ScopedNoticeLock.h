// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-COOK-LIFETIME-001
 * @covers AC-4
 */

#pragma once

#include "OmniPhysX.h"

namespace cookingdataasync
{
class CookingDataAsync;
}

namespace omni
{
namespace physx
{
namespace usdparser
{
class UsdLoad;
}
class ScopedNoticeBlock
{
public:
    ScopedNoticeBlock()
    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        mUsdLoad = usdparser::UsdLoad::getUsdLoad();
        mUsdLoad->blockUSDUpdate(true);
        cookingdataasync::CookingDataAsync* cookingAsync = omniPhysX.getPhysXSetup().getCookingDataAsync();
        if (cookingAsync)
        {
            cookingAsync->blockUSDUpdate(true);
            mBlockedCooking = true;
        }
    }

    ~ScopedNoticeBlock()
    {
        // The scope typically straddles USD authoring (see InternalScene::updateRenderTransforms),
        // and the change notices sent when the SdfChangeBlock closes dispatch synchronously on this
        // thread, so a listener can re-enter physics and tear down the PhysX SDK together with the
        // async cooking singleton. Re-fetch instead of caching the constructor's pointer, which may
        // be dangling by now. blockUSDUpdate() is underflow-guarded, so releasing against a freshly
        // created singleton does not misbehave in release; an assert-enabled build reports it.
        if (mBlockedCooking)
        {
            cookingdataasync::CookingDataAsync* cookingAsync =
                OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
            if (cookingAsync)
                cookingAsync->blockUSDUpdate(false);
        }
        mUsdLoad->blockUSDUpdate(false);
    }

private:
    usdparser::UsdLoad* mUsdLoad;
    bool mBlockedCooking{ false };
};
} // namespace physx
} // namespace omni
