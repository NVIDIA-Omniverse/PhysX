// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{
namespace physx
{

class PhysXStageUpdate
{
public:
    void attachStageUpdate();
    void detachStageUpdate();

    void attachPropertyQueryStageUpdate()
    {
        mPropertyQueryAttached = true;
    }

    void detachPropertyQueryStageUpdate()
    {
        mPropertyQueryAttached = false;
    }

    bool isAttached() const
    {
        return mAttached;
    }

    void attach(bool val)
    {
        mAttached = val;
    }

    bool isPropertyQueryAttached() const
    {
        return mPropertyQueryAttached;
    }


    uint64_t getAttachedStage() const
    {
        return mAttachedStage;
    }

    void setAttachedStage(uint64_t stageId)
    {
        mAttachedStage = stageId;
    }

private:
    uint64_t mAttachedStage{ 0 };
    bool mAttached{ false };
    bool mPropertyQueryAttached{ false };
};


} // namespace physx
} // namespace omni
