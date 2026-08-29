// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-FILTER-001
 * @covers AC-1 AC-2
 */

// Filtered-pairs reader — `PhysicsFilteredPairsAPI:physics:filteredPairs`
// relationship targets, source-agnostic. Used by the rigid-body, collision-
// shape, articulation-root, and deformable-body parsers to populate their
// per-prim filter list.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/KnownTokens.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

namespace omni::physics::parse
{

std::vector<ObjectKey> parseFilteredPairs(ParseContext& ctx, ObjectKey key)
{
    std::vector<ObjectKey> out;

    KnownTokens tok;
    tok.intern(ctx.source());

    if (!ctx.source().hasSchema(key, tok.physicsFilteredPairsAPI))
        return out;

    ctx.source().getRelationshipTargets(key, tok.physicsFilteredPairs, out);
    return out;
}

} // namespace omni::physics::parse
