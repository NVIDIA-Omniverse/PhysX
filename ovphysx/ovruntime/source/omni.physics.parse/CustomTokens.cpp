// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * Process-wide custom-token registry implementation (source-agnostic core).
 *
 * @implements REQ-PARSE-CORE-005
 * @covers AC-1 AC-3 AC-4
 */

#include <omni/physics/parse/CustomTokens.h>

#include <mutex>
#include <set>

namespace omni::physics::parse
{

namespace
{

// std::set, not unordered_set: `customTokens()` is a snapshot the ovstage walker
// turns into one query per token, and the resulting descriptors are emitted in
// that order. Lexicographic keeps the emit order independent of the order
// consumers happened to register in.
using TokenSet = std::set<std::string, std::less<>>;

struct Registry
{
    std::mutex mutex;
    TokenSet shapeTokens;
    TokenSet jointTokens;
    TokenSet instancerTokens;

    Registry()
    {
        // Pre-register the well-known internal tokens, baked into the
        // parse-lib so a walker recognizes them even before any
        // consumer-side runtime is loaded (e.g. in unit tests that
        // exercise `scanStage` without booting `omni.physx`).
        //
        // The runtime's `OmniPhysX::onStartup` still calls the
        // `register*Token` API for these tokens; that's now an
        // idempotent no-op on the second insert and continues to
        // serve as the public registration surface for third-party
        // plugin tokens.
        shapeTokens.insert("PhysxMeshMergeCollisionAPI");
        shapeTokens.insert("ConvexMesh");
        shapeTokens.insert("Plane");
        jointTokens.insert("PhysxPhysicsGearJoint");
        jointTokens.insert("PhysxPhysicsRackAndPinionJoint");
        instancerTokens.insert("PhysxPhysicsJointInstancer");
    }
};

Registry& registry()
{
    static Registry r;
    return r;
}

TokenSet& setFor(Registry& r, CustomTokenKind kind)
{
    switch (kind)
    {
    case CustomTokenKind::eJoint:
        return r.jointTokens;
    case CustomTokenKind::ePhysicsInstancer:
        return r.instancerTokens;
    case CustomTokenKind::eShape:
    default:
        return r.shapeTokens;
    }
}

} // namespace

void registerCustomToken(CustomTokenKind kind, std::string_view token)
{
    if (token.empty())
        return;
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    setFor(r, kind).emplace(token);
}

void unregisterCustomToken(CustomTokenKind kind, std::string_view token)
{
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    TokenSet& set = setFor(r, kind);
    // std::less<> makes the set transparent, so the heterogeneous erase below
    // needs no temporary std::string.
    const TokenSet::const_iterator it = set.find(token);
    if (it != set.end())
        set.erase(it);
}

// Snapshot semantics: each lookup acquires the mutex once and tests against the
// current set; a concurrent register / unregister is observed through the same
// mutex.

bool isCustomToken(CustomTokenKind kind, std::string_view token)
{
    if (token.empty())
        return false;
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    const TokenSet& set = setFor(r, kind);
    return set.find(token) != set.end();
}

std::vector<std::string> customTokens(CustomTokenKind kind)
{
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    const TokenSet& set = setFor(r, kind);
    return std::vector<std::string>(set.begin(), set.end());
}

} // namespace omni::physics::parse
