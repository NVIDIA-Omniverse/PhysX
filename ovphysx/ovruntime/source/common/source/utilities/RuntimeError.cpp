// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-RUNTIME-ERROR-001
 * @covers AC-1 AC-2 AC-4
 */

#include <omni/physx/RuntimeError.h>

#include <cstdio>

namespace omni::physx
{
namespace
{
thread_local RuntimeErrorScope* t_activeScope = nullptr;
} // namespace

RuntimeErrorScope::RuntimeErrorScope() noexcept : mPreviousScope(t_activeScope)
{
    mMessage[0] = '\0';
    t_activeScope = this;
}

RuntimeErrorScope::~RuntimeErrorScope() noexcept
{
    t_activeScope = mPreviousScope;
}

const char* RuntimeErrorScope::message() const noexcept
{
    return mMessage;
}

void recordRuntimeError(const char* message) noexcept
{
    if (!t_activeScope || !message || !message[0] || t_activeScope->mMessage[0])
        return;

    std::snprintf(t_activeScope->mMessage, sizeof(t_activeScope->mMessage), "%s", message);
}

} // namespace omni::physx
