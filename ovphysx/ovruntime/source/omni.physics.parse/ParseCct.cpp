// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-CCT-001
 * @covers AC-1 AC-2 AC-3
 */

// Character-controller reader. Reads
// `physxCharacterController:slopeLimit` (defaulting to 0.0f when
// unauthored) and resolves the simulation-owner relationship to a scene
// `ObjectId`. Capsule geometry + world pose are pre-resolved by the
// walker and passed in via `CctInfo` — keeping the parser source-agnostic
// while letting the walker reuse its existing world-transform / capsule-
// gprim plumbing.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/KnownTokens.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedCct.h>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

namespace omni::physics::parse
{

PreparedCct prepareCct(ParseContext& ctx, ObjectKey key, const CctInfo& info)
{
    PreparedCct input;
    input.key = key;
    input.pos = info.pos;
    input.scale = info.scale;
    input.radius = info.radius;
    input.halfHeight = info.halfHeight;
    input.slopeLimit = prepareAttribute<float>(ctx.source(), key, ctx.knownTokens().physxCharacterControllerSlopeLimit);
    if (info.simulationOwners.size() > 1)
        CARB_LOG_ERROR("parseCct: simulationOwner relationship can have at most 1 entry. (%s)",
                       std::string(ctx.source().sourceKeyToString(key)).c_str());
    if (!info.simulationOwners.empty()) input.owner = info.simulationOwners.front();
    input.prepared = true;
    return input;
}

bool fillCct(const PreparedCct& input, CapsuleCctDesc& output) noexcept
{
    if (!input.prepared) return false;
    output.primKey = input.key;
    output.pos = input.pos;
    output.scale = input.scale;
    output.radius = input.radius;
    output.height = input.halfHeight;
    output.slopeLimit = input.slopeLimit.valueOr(0.0f);
    output.sourceSimulationOwner = input.owner;
    return true;
}

DescPtr<CapsuleCctDesc> parseCct(ParseContext& ctx, ObjectKey key, const CctInfo& info)
{
    DescPtr<CapsuleCctDesc> output = allocateDesc<CapsuleCctDesc>(ctx.descriptorAllocator());
    if (output) fillCct(prepareCct(ctx, key, info), *output);
    return output;
}

} // namespace omni::physics::parse
