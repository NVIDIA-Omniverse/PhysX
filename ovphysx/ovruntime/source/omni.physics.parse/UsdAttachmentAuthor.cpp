// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

#include <omni/physics/parse/UsdAttachmentAuthor.h>

namespace omni::physics::parse
{
namespace
{
std::unique_ptr<IUsdAttachmentAuthor>& activeSlot()
{
    static std::unique_ptr<IUsdAttachmentAuthor> slot;
    return slot;
}
} // namespace

void setUsdAttachmentAuthor(std::unique_ptr<IUsdAttachmentAuthor> author)
{
    activeSlot() = std::move(author);
}

IUsdAttachmentAuthor* usdAttachmentAuthor()
{
    return activeSlot().get();
}

} // namespace omni::physics::parse
