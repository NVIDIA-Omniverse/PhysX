// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

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
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

namespace omni::physics::parse
{

DescPtr<CapsuleCctDesc> parseCct(ParseContext& ctx, ObjectKey key, const CctInfo& info)
{
    DescPtr<CapsuleCctDesc> desc =
        allocateDesc<CapsuleCctDesc>(ctx.descriptorAllocator(), info.radius, info.halfHeight);
    desc->primKey              = key;
    desc->pos                   = info.pos;
    desc->scale                 = info.scale;
    desc->slopeLimit            = 0.0f;
    desc->sourceSimulationOwner = ObjectKey{};

    IPhysicsSource& src = ctx.source();
    KnownTokens tok;
    tok.intern(src);

    src.getAttribute(key, tok.physxCharacterControllerSlopeLimit, desc->slopeLimit);

    if (info.simulationOwners.size() > 1)
    {
        CARB_LOG_ERROR("parseCct: simulationOwner relationship can have at most 1 entry. (%s)",
                       std::string(src.sourceKeyToString(key)).c_str());
    }
    if (!info.simulationOwners.empty())
        desc->sourceSimulationOwner = info.simulationOwners.front();

    return desc;
}

} // namespace omni::physics::parse
