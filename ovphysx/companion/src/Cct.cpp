// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-CCT-001
// @covers AC-1 AC-2 AC-3 AC-4
// @implements REQ-CAPI-COMPANION-001
// @covers AC-1
#include "ovphysx_companion/cct.h"
#include "internal/sdk/CompanionRuntime.hpp"
#include <omni/physx/NativeCct.h>
#include <algorithm>

namespace
{
ovphysx_companion_status_t execute(uint64_t attachment, const char* path,
    const ovphysx_companion_cct_command_t* input, uint32_t size, ovphysx_companion_cct_state_t* output) noexcept
{
    if (!input || !output || input->size != sizeof(*input) || input->version != 1 || input->reserved || size != sizeof(*output))
        return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    CompanionOperationGuard guard(companionRuntime());
    if (!guard) return OVPHYSX_COMPANION_BUSY;
    using namespace omni::physx::native;
    CctCommand command;
    command.action = CctAction(input->action);
    command.generation = input->generation;
    std::copy_n(input->position, 3, command.position);
    std::copy_n(input->displacement, 3, command.displacement);
    command.height = input->height; command.radius = input->radius;
    command.dt = input->dt; command.minDistance = input->min_distance; command.filters = input->filters;
    CctState result;
    switch (controlCct(attachment, path, command, result))
    {
    case CctResult::Invalid: return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    case CctResult::Stale: return OVPHYSX_COMPANION_STALE;
    case CctResult::Busy: return OVPHYSX_COMPANION_BUSY;
    case CctResult::NotFound: return OVPHYSX_COMPANION_NOT_FOUND;
    case CctResult::Unavailable: return OVPHYSX_COMPANION_UNAVAILABLE;
    case CctResult::Error: return OVPHYSX_COMPANION_ERROR;
    case CctResult::Success: break;
    }
    ovphysx_companion_cct_state_t value{};
    value.size = sizeof(value); value.version = 1;
    value.attachment = attachment; value.key = result.key; value.generation = result.generation;
    std::copy_n(result.position, 3, value.position);
    std::copy_n(result.up, 3, value.up); std::copy_n(result.gravity, 3, value.gravity);
    value.height = result.height; value.radius = result.radius; value.collisions = result.collisions;
    *output = value;
    return OVPHYSX_COMPANION_OK;
}
}
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_cct_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_cct_api_t* output)
{
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if (version != 1 || size != sizeof(*output)) return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output = {sizeof(*output), 1, 1, execute};
    return OVPHYSX_COMPANION_OK;
}
