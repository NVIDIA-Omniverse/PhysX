// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-DEBUG-SNAPSHOT-001
// @covers AC-1 AC-2 AC-3
#pragma once
#include <cstdint>
#include <vector>
namespace omni::physx::native
{
enum class DebugResult : uint32_t { Success, Invalid, Stale, Busy, Unavailable, Limit, Error };
struct DebugConfig
{
    uint64_t parameters = 0; // IPhysxVisualization parameter bits, supported mask below.
    float scale = 1; // Multiplied by attachment viewportGizmoScale.
    bool enabled = false;
    bool culling = false;
    float minimum[3]{}, maximum[3]{}; // World stage units.
};
struct DebugVertex { float position[3]; uint32_t color; }; // Native packed 0xAARRGGBB.
struct DebugFrame
{
    uint64_t attachment = 0, step = 0;
    std::vector<DebugVertex> points, lines, triangles; // 1/2/3 vertices per primitive.
};
// Known PhysX primitives only; eNone/eCullBox/custom visualization are not bits.
uint64_t debugParameterMask() noexcept;
// Idle exact-owner CPU access. Settings affect the next completed step. Host owns
// visualization policy exclusively; false disables it. Existing scope is respected.
DebugResult configureDebug(uint64_t attachment, const DebugConfig& config) noexcept;
// Copies a single idle frame. maxVertices bounds the combined allocation; exceeding
// it returns Limit, never a silently truncated frame. Rejection preserves output.
// World stage units, packed ARGB color. Frame survives owner shutdown.
DebugResult captureDebug(uint64_t attachment, uint64_t maxVertices, DebugFrame& output) noexcept;
}
