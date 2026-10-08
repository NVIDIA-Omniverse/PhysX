// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

#include <omni/physics/parse/UsdBackingAuthor.h>

namespace omni::physics::parse
{
namespace
{
std::unique_ptr<IUsdBackingAuthor>& activeSlot()
{
    static std::unique_ptr<IUsdBackingAuthor> slot;
    return slot;
}
} // namespace

void setUsdBackingAuthor(std::unique_ptr<IUsdBackingAuthor> author)
{
    activeSlot() = std::move(author);
}

IUsdBackingAuthor* usdBackingAuthor()
{
    return activeSlot().get();
}

} // namespace omni::physics::parse
