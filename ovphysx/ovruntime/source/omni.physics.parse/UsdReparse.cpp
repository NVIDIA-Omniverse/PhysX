// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

#include <omni/physics/parse/UsdReparse.h>

namespace omni::physics::parse
{
namespace
{
std::unique_ptr<IUsdReparse>& activeSlot()
{
    static std::unique_ptr<IUsdReparse> slot;
    return slot;
}
} // namespace

void setUsdReparse(std::unique_ptr<IUsdReparse> reparse)
{
    activeSlot() = std::move(reparse);
}

IUsdReparse* usdReparse()
{
    return activeSlot().get();
}

} // namespace omni::physics::parse
