// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CUSTOM-CONTROL-001
// @covers AC-1 AC-2 AC-3
#pragma once
#include "NativeParameters.h"
namespace omni::physx::native
{
enum class JointAction : uint32_t { MarkDirty, SetFlags, GetFlags, Refresh };
enum class JointResult : uint32_t { Success, Busy, Stale, NotFound, Invalid, Rejected };
using RefreshJoint = bool (*)(uint64_t owner, uint64_t key, const ParameterValue*, uint32_t count, void*) noexcept;
// Uses one exact owner's execution admission. expectedContext identifies the
// registered callback table, preventing edits of another module's joint.
// Refresh copies the owner's current sealed source values before invoking the
// native callback; a rejected callback must leave plugin state unchanged.
// Success marks shader data dirty for the next step. SetFlags supports only bit0
// (ALWAYS_UPDATE); GetFlags writes flags only on success. No pointer escapes.
JointResult controlJoint(uint64_t owner, const char* path, void* expectedContext,
    JointAction action, uint32_t& flags, const ParameterSpec* specs, uint32_t count,
    RefreshJoint refresh, void* context) noexcept;
}
