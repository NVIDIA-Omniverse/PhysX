// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-COLGROUP-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-PARSE-COLGROUP-002
 * @covers AC-3
 */

// Collision-group reader — `physics:filteredGroups` relationship targets +
// the `colliders` collection's pre-resolved member list.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/KnownTokens.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

namespace omni::physics::parse
{

CollisionGroupInfo parseCollisionGroup(ParseContext& ctx, ObjectKey key)
{
    CollisionGroupInfo info;

    KnownTokens tok;
    tok.intern(ctx.source());

    // Filtered groups (other collision-group prims to filter against).
    ctx.source().getRelationshipTargets(key, tok.physicsFilteredGroups, info.filteredGroups);

    // Collection members — source backend computes the include/exclude
    // resolution and returns the flat included-prim list.
    ctx.source().resolveCollection(key, tok.collidersCollectionName, info.members);

    return info;
}

} // namespace omni::physics::parse
