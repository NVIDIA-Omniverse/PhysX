// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
    void attachPropertyQueryStageUpdate()
    {
        mPropertyQueryAttached = true;
    }

    void detachPropertyQueryStageUpdate()
    {
        mPropertyQueryAttached = false;
    }

    bool isPropertyQueryAttached() const
    {
        return mPropertyQueryAttached;
    }


private:
    bool mPropertyQueryAttached{ false };
};


} // namespace physx
} // namespace omni
