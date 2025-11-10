// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

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
        omniPhysX.getInternalPhysXDatabase().mXformCache.Clear();
        mUsdLoad->blockUSDUpdate(true);
        mChangeTrackingPaused = mUsdLoad->isChangeTrackingPaused(omniPhysX.getStageId());
        mUsdLoad->pauseChangeTracking(omniPhysX.getStageId(), true);
        mCookingAsync = omniPhysX.getPhysXSetup().getCookingDataAsync();
        if (mCookingAsync)
            mCookingAsync->blockUSDUpdate(true);
    }

    ~ScopedNoticeBlock()
    {
        if (mCookingAsync)
            mCookingAsync->blockUSDUpdate(false);
        mUsdLoad->blockUSDUpdate(false);
        mUsdLoad->pauseChangeTracking(OmniPhysX::getInstance().getStageId(), mChangeTrackingPaused);
    }

private:
    bool mChangeTrackingPaused;
    usdparser::UsdLoad* mUsdLoad;
    cookingdataasync::CookingDataAsync* mCookingAsync;
};
} // namespace physx
} // namespace omni
