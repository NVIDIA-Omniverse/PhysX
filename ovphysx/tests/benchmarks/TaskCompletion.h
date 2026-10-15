// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <ovphysx/experimental/Helpers.hpp>
#include <cstdio>

// Existing benchmark failure paths inspect the returned execution status.
inline ovphysx_result_t bmCompleteTask(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    ovphysx::physx::WaitResult wait;
    const ovphysx_api_status_t status = task.status == OVPHYSX_API_SUCCESS ?
        wait.waitTask(handle, task) : task.status;
    for (size_t index = 0; index < wait.errorCount(); ++index)
    {
        const ovphysx_string_t error = ovphysx_get_last_task_error(handle, wait.errorTaskAt(index));
        std::fprintf(stderr, "ovphysx task failed: %.*s\n", static_cast<int>(error.length),
                     error.ptr ? error.ptr : "");
    }
    if (status != OVPHYSX_API_SUCCESS && !wait.hasErrors())
    {
        const ovphysx_string_t error = ovphysx_get_last_error();
        std::fprintf(stderr, "ovphysx task rejected or wait failed (%d): %.*s\n", static_cast<int>(status),
                     static_cast<int>(error.length), error.ptr ? error.ptr : "");
    }
    return { status };
}
