// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-3 AC-4
 */
#pragma once

#include "DescriptorPipelineStats.h"
#include <algorithm>
#include <cstddef>
#include <type_traits>
#include <exception>
#include <vector>

namespace omni::physics::parse
{

using DescriptorRangeKernel = void (*)(void* context, size_t begin, size_t end) noexcept;

// execute() is a synchronous phase boundary: every range has finished before
// it returns or throws. Scheduling and joining are outside the supplied kernel.
class IDescriptorRangeExecutor
{
public:
    virtual ~IDescriptorRangeExecutor() = default;
    virtual void execute(size_t count, size_t tasks, void* context, DescriptorRangeKernel kernel) = 0;
};

// Contiguous partitioning avoids completion-order publication and multiplication
// overflow even for counts near SIZE_MAX. Task identities are logical indices.
inline size_t descriptorRangeBegin(size_t count, size_t tasks, size_t task) noexcept
{
    return (count / tasks) * task + std::min(count % tasks, task);
}

struct DescriptorExecutionOptions
{
    // One runs the identical kernel serially. Zero lets the executor choose;
    // explicit values support parity testing and controlled benchmarks.
    size_t tasks = 1;
    size_t grain = 128;
    IDescriptorRangeExecutor* executor = nullptr;
    DescriptorPipelineStats* statistics = nullptr;
    void (*reportStatistics)(const DescriptorPipelineStats&) = nullptr;
};

template <typename Kernel>
void fillDescriptorRanges(size_t count, const DescriptorExecutionOptions& options, Kernel&& kernel)
{
    if (!count)
        return;
    const size_t grain = std::max(size_t(1), options.grain);
    const size_t chunks = count / grain + (count % grain != 0);
    const size_t tasks = options.tasks ? std::min(options.tasks, chunks) : chunks;
    if (options.statistics)
    {
        options.statistics->entries += count;
        options.statistics->tasks += options.executor ? tasks : 1;
    }
    DescriptorPhaseTimer timer(options.statistics ? &options.statistics->fillAndJoinSeconds : nullptr);
    if (!options.executor || tasks <= 1)
    {
        kernel(0, count);
        return;
    }
    // The callable lives on the coordinator's stack until execute has joined.
    // Workers neither copy it nor retain an owning reference to its captures.
    using Callable = std::remove_reference_t<Kernel>;
    if constexpr (std::is_nothrow_invocable_v<Callable&, size_t, size_t>)
    {
        options.executor->execute(count, tasks, &kernel, [](void* context, size_t begin, size_t end) noexcept
        {
            (*static_cast<Callable*>(context))(begin, end);
        });
    }
    else
    {
        // Only potentially throwing callers need exception slots. Dispatch logical
        // task indices so even an executor's serial fallback has exclusive slots.
        std::vector<std::exception_ptr> errors(tasks);
        struct Context { Callable* callable; size_t count, tasks; std::exception_ptr* errors; };
        Context state{&kernel, count, tasks, errors.data()};
        options.executor->execute(tasks, tasks, &state, [](void* context, size_t begin, size_t end) noexcept
        {
            auto& state = *static_cast<Context*>(context);
            for (size_t task = begin; task < end; ++task)
            {
                try
                {
                    (*state.callable)(descriptorRangeBegin(state.count, state.tasks, task),
                                      descriptorRangeBegin(state.count, state.tasks, task + 1));
                }
                catch (...) { state.errors[task] = std::current_exception(); }
            }
        });
        // execute() has joined every task; propagate the first logical failure.
        for (const auto& error : errors) if (error) std::rethrow_exception(error);
    }
}

} // namespace omni::physics::parse
