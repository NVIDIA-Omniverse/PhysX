// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

#include <omni/physics/parse/UsdStageLifecycle.h>

namespace omni::physics::parse
{
namespace
{
std::unique_ptr<IUsdStageLifecycle>& activeSlot()
{
    static std::unique_ptr<IUsdStageLifecycle> slot;
    return slot;
}
} // namespace

void setUsdStageLifecycle(std::unique_ptr<IUsdStageLifecycle> lifecycle)
{
    activeSlot() = std::move(lifecycle);
}

IUsdStageLifecycle* usdStageLifecycle()
{
    return activeSlot().get();
}

} // namespace omni::physics::parse
