// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-1 AC-2 AC-3 AC-5 AC-6
 */

#pragma once

#include "internal/sdk/ovphysxSDK.hpp"
#include <limits>
#include <stdexcept>

namespace ovphysx
{

// Create the task receipt and record its outcome.
// Reserve its failure slot before executing any side effects.
// An empty failure string uses a static diagnostic if recording detail fails.
class TaskContext
{
public:
    void create_task(ovphysx_instance_handle_t handle)
    {
        instance = get_instance(handle);
        if (!instance)
            throw std::runtime_error("Invalid task instance");
        const ovphysx_op_index_t index = instance->next_op_index.load();
        if (index == std::numeric_limits<ovphysx_op_index_t>::max())
            throw std::overflow_error("Task index exhausted");
        instance->task_errors.try_emplace(index);
        instance->next_op_index.store(index + 1);
        receipt = {OVPHYSX_API_SUCCESS, index, handle};
        success();
    }

    ovphysx_enqueue_result_t fail(const char* message) noexcept
    {
        return receipt.op_index ? finish(message) : set_enqueue_error(OVPHYSX_API_ERROR, message);
    }

    ovphysx_enqueue_result_t complete() noexcept { return finish(nullptr); }

    ovphysx_enqueue_result_t receipt{};

private:
    ovphysx_enqueue_result_t finish(const char* error) noexcept
    {
        if (instance->pending_simulation.active &&
            instance->pending_simulation.task_index == receipt.op_index)
        {
            if (error)
                instance->pending_simulation.fail(error);
        }
        else if (error)
        {
            try { instance->task_errors.at(receipt.op_index) = error; }
            catch (...) { /* The reserved empty string preserves the failure. */ }
        }
        else
        {
            instance->task_errors.erase(receipt.op_index);
        }
        success();
        return receipt;
    }

    std::shared_ptr<InstanceData> instance;
};

} // namespace ovphysx
