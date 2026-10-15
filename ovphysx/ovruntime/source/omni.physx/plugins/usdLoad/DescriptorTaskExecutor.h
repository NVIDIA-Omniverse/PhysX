// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-3 AC-4
 */
#pragma once

#include <omni/physics/parse/DescriptorRangeExecutor.h>
#include <carb/InterfaceUtils.h>
#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingUtils.h>

namespace omni::physx::usdparser
{

// Reuse the runtime's task scheduler. Standalone parser users without tasking
// still execute the same kernel synchronously, without creating another pool.
class DescriptorTaskExecutor final : public omni::physics::parse::IDescriptorRangeExecutor
{
public:
    void execute(size_t count, size_t tasks, void* context, omni::physics::parse::DescriptorRangeKernel kernel) override
    {
        carb::tasking::ITasking* tasking = carb::getCachedInterface<carb::tasking::ITasking>();
        if (!tasking)
        {
            kernel(context, 0, count);
            return;
        }
        tasking->parallelFor(size_t(0), tasks, [=](size_t task)
        {
            const size_t begin = omni::physics::parse::descriptorRangeBegin(count, tasks, task);
            const size_t end = omni::physics::parse::descriptorRangeBegin(count, tasks, task + 1);
            kernel(context, begin, end);
        });
    }

};

} // namespace omni::physx::usdparser
