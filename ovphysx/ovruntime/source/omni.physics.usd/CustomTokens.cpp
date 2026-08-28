// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * Process-wide custom-token registry implementation.
 *
 * @implements REQ-PARSE-CORE-005
 * @covers AC-1 AC-2
 */

#include <omni/physics/usd/CustomTokens.h>

#include <mutex>
#include <unordered_set>

namespace omni::physics::usd
{

namespace
{

struct Registry
{
    std::mutex mutex;
    std::unordered_set<PXR_NS::TfToken, PXR_NS::TfToken::HashFunctor> shapeTokens;
    std::unordered_set<PXR_NS::TfToken, PXR_NS::TfToken::HashFunctor> jointTokens;
    std::unordered_set<PXR_NS::TfToken, PXR_NS::TfToken::HashFunctor> instancerTokens;

    Registry()
    {
        // Pre-register the well-known internal tokens, baked into the
        // parse-lib so the walker recognizes them even before any
        // consumer-side runtime is loaded (e.g. in unit tests that
        // exercise `scanStage` without booting `omni.physx`).
        //
        // The runtime's `OmniPhysX::onStartup` still calls the
        // `register*Token` API for these tokens; that's now an
        // idempotent no-op on the second insert and continues to
        // serve as the public registration surface for third-party
        // plugin tokens.
        shapeTokens.insert(PXR_NS::TfToken("PhysxMeshMergeCollisionAPI"));
        shapeTokens.insert(PXR_NS::TfToken("ConvexMesh"));
        shapeTokens.insert(PXR_NS::TfToken("Plane"));
        jointTokens.insert(PXR_NS::TfToken("PhysxPhysicsGearJoint"));
        jointTokens.insert(PXR_NS::TfToken("PhysxPhysicsRackAndPinionJoint"));
        instancerTokens.insert(PXR_NS::TfToken("PhysxPhysicsJointInstancer"));
    }
};

Registry& registry()
{
    static Registry r;
    return r;
}

} // namespace

void registerCustomShapeToken(const PXR_NS::TfToken& token)
{
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    r.shapeTokens.insert(token);
}

void unregisterCustomShapeToken(const PXR_NS::TfToken& token)
{
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    r.shapeTokens.erase(token);
}

void registerCustomJointToken(const PXR_NS::TfToken& token)
{
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    r.jointTokens.insert(token);
}

void unregisterCustomJointToken(const PXR_NS::TfToken& token)
{
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    r.jointTokens.erase(token);
}

void registerCustomPhysicsInstancerToken(const PXR_NS::TfToken& token)
{
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    r.instancerTokens.insert(token);
}

void unregisterCustomPhysicsInstancerToken(const PXR_NS::TfToken& token)
{
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    r.instancerTokens.erase(token);
}

// Internal lookup helpers consumed by `NativeWalker.cpp`. Snapshot
// semantics: each lookup acquires the mutex once, hashes against the
// current set; concurrent register / unregister observes the
// strongly-consistent state via the same mutex.

bool isCustomShapeToken(const PXR_NS::TfToken& token)
{
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    return r.shapeTokens.find(token) != r.shapeTokens.end();
}

bool isCustomJointToken(const PXR_NS::TfToken& token)
{
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    return r.jointTokens.find(token) != r.jointTokens.end();
}

bool isCustomPhysicsInstancerToken(const PXR_NS::TfToken& token)
{
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    return r.instancerTokens.find(token) != r.instancerTokens.end();
}

} // namespace omni::physics::usd
