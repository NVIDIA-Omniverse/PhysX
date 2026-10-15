// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CCT-CONTROL-001
// @covers AC-1 AC-2 AC-3 AC-4
#pragma once
#include <cstdint>
namespace omni::physx::native
{
enum class CctAction : uint32_t { Describe, Move, Teleport, Resize, Remove };
enum class CctResult : uint32_t { Success, Invalid, Stale, Busy, NotFound, Unavailable, Error };
struct CctCommand
{
    CctAction action = CctAction::Describe;
    uint64_t generation = 0; // Zero is allowed only for initial Describe.
    double position[3]{};    // World stage units, controller center.
    float displacement[3]{}; // World stage units, applied once by Move.
    float height = 0, radius = 0; // Capsule cylinder height and radius, stage units.
    float dt = 0, minDistance = 0;
    uint32_t filters = 7; // bit0 static shapes, bit1 dynamic shapes, bit2 other CCTs.
};
struct CctState
{
    uint64_t key = 0, generation = 0;
    double position[3]{};
    float up[3]{}, gravity[3]{};
    float height = 0, radius = 0;
    uint32_t collisions = 0; // Move result: bit0 sides, bit1 above, bit2 below.
};
// Selects the exact attachment and takes its idle admission. CCTs are created by
// the normal ovstage parser. Every mutation requires a captured record generation;
// deletion/recreation cannot redirect a stale command to a new controller.
// CPU scenes only. Move obeys scene-query shape flags and its explicit category
// mask; simulation collision-group filtering is not implicitly applied. Resize
// preserves the foot position, without collision testing; Teleport sets the center.
// Remove retires only the CCT record, leaving source opinions untouched. Recreate
// through explicit source repopulation or a new attachment. No native pointer escapes.
CctResult controlCct(uint64_t attachment, const char* path, const CctCommand& command, CctState& output) noexcept;
}
