// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-IO-001
// @covers AC-1 AC-3

/**
 * @implements REQ-COOK-LIFETIME-001
 * @covers AC-4
 */

#pragma once

#include "usdLoad/AttachedStage.h"

namespace omni
{
namespace physx
{
// Suppression belongs to the source that is being authored. Retaining just the
// counters allows scope exit after reentrant owner destruction without touching
// a replacement source or a recreated cooking service.
class ScopedNoticeBlock
{
public:
    explicit ScopedNoticeBlock(const usdparser::AttachedStage& owner, bool source = true, bool cooking = true)
        : mState(owner.getNoticeBlockState()), mSource(source), mCooking(cooking)
    {
        if (mSource)
            ++mState->source;
        if (mCooking)
            ++mState->cooking;
    }
    ~ScopedNoticeBlock()
    {
        if (mCooking)
            --mState->cooking;
        if (mSource)
            --mState->source;
    }
    ScopedNoticeBlock(const ScopedNoticeBlock&) = delete;
    ScopedNoticeBlock& operator=(const ScopedNoticeBlock&) = delete;

private:
    std::shared_ptr<usdparser::AttachedStage::NoticeBlockState> mState;
    bool mSource;
    bool mCooking;
};
} // namespace physx
} // namespace omni
