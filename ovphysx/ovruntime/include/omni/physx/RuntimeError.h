// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-RUNTIME-ERROR-001
 * @covers AC-1 AC-2 AC-3 AC-4
 */

#pragma once

#include <carb/logging/Log.h>

#include <cstdio>

namespace omni::physx
{

// Each scope captures its own first nonempty runtime error, independently of logging.
// Only the innermost scope on this thread receives new errors. Leaving a nested scope
// resumes the outer scope without copying the inner message into it. Construct and destroy
// scopes on the same thread in stack order. Errors from other threads are not captured,
// and recording an error does not change operation status.
class RuntimeErrorScope
{
public:
    RuntimeErrorScope() noexcept;
    ~RuntimeErrorScope() noexcept;

    RuntimeErrorScope(const RuntimeErrorScope&) = delete;
    RuntimeErrorScope& operator=(const RuntimeErrorScope&) = delete;
    RuntimeErrorScope(RuntimeErrorScope&&) = delete;
    RuntimeErrorScope& operator=(RuntimeErrorScope&&) = delete;

    // Empty until an error is recorded. At most 2047 bytes plus a NUL terminator;
    // the returned storage belongs to this scope and remains valid until destruction.
    const char* message() const noexcept;

private:
    friend void recordRuntimeError(const char* message) noexcept;

    RuntimeErrorScope* mPreviousScope;
    char mMessage[2048];
};

// Copies into the innermost scope on this thread without allocating. Null or empty
// messages, later errors in the same scope, and calls without an active scope are ignored.
void recordRuntimeError(const char* message) noexcept;

} // namespace omni::physx

// Format once for capture and logging. A macro keeps the log's source location at the failure.
#define OVX_RUNTIME_ERROR(...)                                                                                         \
    do                                                                                                                 \
    {                                                                                                                  \
        char ovxRuntimeErrorMessage[2048];                                                                             \
        ovxRuntimeErrorMessage[0] = '\0';                                                                              \
        std::snprintf(ovxRuntimeErrorMessage, sizeof(ovxRuntimeErrorMessage), __VA_ARGS__);                            \
        ::omni::physx::recordRuntimeError(ovxRuntimeErrorMessage);                                                     \
        CARB_LOG_ERROR("%s", ovxRuntimeErrorMessage);                                                                  \
    } while (false)
