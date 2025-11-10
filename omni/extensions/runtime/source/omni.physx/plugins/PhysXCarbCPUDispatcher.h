// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <PxPhysicsAPI.h>

#include <carb/profiler/Profile.h>
#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingUtils.h>


class PhysXCarbCpuDispatcher : public ::physx::PxCpuDispatcher
{
public:
    PhysXCarbCpuDispatcher(uint32_t numThreads)
    {
        mTasking = carb::getCachedInterface<carb::tasking::ITasking>();

        const carb::tasking::TaskingDesc& desc = mTasking->getDesc();

        if (desc.threadCount <= numThreads)
        {
            mNumThreads = desc.threadCount;
        }
        else
        {
            mNumThreads = numThreads;
            mSema = mTasking->createSemaphore(mNumThreads);
        }
    }

    virtual void release()
    {
        if (mSema)
        {
            mTasking->destroySemaphore(mSema);
        }
        mTasking = nullptr;
    }

    virtual void submitTask(::physx::PxBaseTask& task)
    {
        if (mNumThreads)
        {
            mTasking->addThrottledTask(mSema, carb::tasking::Priority::eHigh, {}, [&task] {
                CARB_PROFILE_ZONE(0, "%s", task.getName());
                task.run();
                task.release();
            });
        }
        else
        {
            CARB_PROFILE_ZONE(0, "%s", task.getName());
            task.run();
            task.release();
        }
    }

    virtual uint32_t getWorkerCount() const
    {
        return mNumThreads;
    }

private:
    carb::tasking::ITasking* mTasking{ nullptr }; // Pointer to the tasking interface
    carb::tasking::Semaphore* mSema{ nullptr };
    uint32_t mNumThreads{ 0 };
};
