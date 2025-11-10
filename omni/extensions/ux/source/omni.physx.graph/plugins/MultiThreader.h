// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once
#include <carb/tasking/ITasking.h>

class MultiThreader
{
    static const uint32_t gMaxNbTasks = 256;

public:
    MultiThreader(uint32_t nbTasks, uint32_t nbToGo) : mNbTasks(0)
    {
        CARB_ASSERT(nbTasks < gMaxNbTasks);
        if (nbToGo)
        {
            nbTasks = nbToGo <= nbTasks ? nbToGo : nbTasks;
            mNbTasks = nbTasks;
            const uint32_t batchSize = nbToGo / nbTasks;

            uint32_t remain = nbToGo;
            uint32_t start = 0;
            for (uint32_t i = 0; i < nbTasks; i++)
            {
                const uint32_t load = i != nbTasks - 1 ? batchSize : remain;
                mStarts[i] = start;
                mLoads[i] = load;
                start += load;
                remain -= load;
            }
        }
    }
    ~MultiThreader()
    {
        for (uint32_t i = 0; i < mNbTasks; i++)
        {
            if (mFutures[i].valid())
                mFutures[i].wait();
        }
    }

    uint32_t mNbTasks;
    carb::tasking::Future<> mFutures[gMaxNbTasks];
    uint32_t mStarts[gMaxNbTasks];
    uint32_t mLoads[gMaxNbTasks];
};
