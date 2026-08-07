// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
        mUsdLoad->blockUSDUpdate(true);
        mCookingAsync = omniPhysX.getPhysXSetup().getCookingDataAsync();
        if (mCookingAsync)
            mCookingAsync->blockUSDUpdate(true);
    }

    ~ScopedNoticeBlock()
    {
        if (mCookingAsync)
            mCookingAsync->blockUSDUpdate(false);
        mUsdLoad->blockUSDUpdate(false);
    }

private:
    usdparser::UsdLoad* mUsdLoad;
    cookingdataasync::CookingDataAsync* mCookingAsync;
};
} // namespace physx
} // namespace omni
