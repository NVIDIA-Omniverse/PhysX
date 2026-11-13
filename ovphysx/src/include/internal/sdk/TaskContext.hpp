// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-1 AC-2 AC-3 AC-5 AC-6
 *
 * @implements REQ-CAPI-EXECUTOR-001
 * @covers AC-4
 *
 * @implements REQ-CAPI-TASK-001
 * @covers AC-6
 *
 * @implements REQ-CAPI-ASYNC-003
 * @covers AC-5
 */

#pragma once

#include "internal/sdk/ovphysxSDK.hpp"
#include <stdexcept>

namespace ovphysx
{

// Create the receipt of work executed on the calling thread and record its outcome.
// Reserve its failure storage before executing any side effects. The receipt lives in
// the instance executor's task table, which worker jobs update concurrently.
// An empty failure string uses a static diagnostic if recording detail fails.
class TaskContext
{
public:
    // Work this call finishes before it returns, under the instance lane: the receipt
    // and its failure storage come from the executor without its mutex or a heap
    // allocation. instance is the one the call already resolved; it must stay alive
    // until complete() or fail().
    void create_task(InstanceData& instance, ovphysx_instance_handle_t handle)
    {
        if (!instance.executor)
            throw std::runtime_error("Invalid task instance");
        const ovphysx_op_index_t index = instance.executor->beginInlineTask(m_slot);
        m_instance = &instance;
        m_inline = true;
        receipt = {OVPHYSX_API_SUCCESS, index, handle};
        success();
    }

    // Work this call runs for an instance it has not resolved itself: the failure slot
    // is reserved in the task table under the executor mutex, and the receipt keeps
    // the instance alive until complete() or fail().
    void create_task(ovphysx_instance_handle_t handle)
    {
        m_owner = get_instance(handle);
        if (!m_owner || !m_owner->executor)
            throw std::runtime_error("Invalid task instance");
        const ovphysx_op_index_t index = m_owner->executor->createInlineTask();
        m_instance = m_owner.get();
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
        if (m_inline)
            m_instance->executor->endInlineTask(receipt.op_index, m_slot, error);
        else
            m_instance->executor->finishInlineTask(receipt.op_index, error);
        success();
        return receipt;
    }

    // Keeps the instance alive for the handle form; the inline form borrows it.
    std::shared_ptr<InstanceData> m_owner;
    InstanceData* m_instance = nullptr;
    InstanceExecutor::FailureSlot m_slot;
    bool m_inline = false;
};

} // namespace ovphysx
