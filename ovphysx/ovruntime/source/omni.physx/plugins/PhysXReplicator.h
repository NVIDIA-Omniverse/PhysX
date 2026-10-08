// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-SIM-OVSTAGE-ATTACH-001
 * @covers AC-2
 */

#pragma once

#include <omni/physx/IPhysxReplicator.h>
#include <omni/physics/parse/Handles.h>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace omni
{
namespace physx
{
class PhysXReplicator;
class PhysXUsdPhysicsInterface;

// Keyed by the AttachHandle a replicator was registered under (ADR-0016), not by stage id: a
// registration can no longer be picked up by a later re-attach that happens to reuse the same
// stage id. The key is opaque to the map -- OmniPhysX owns the rule that kActiveAttach and the
// lone active attach's own handle name the same attach.
using ReplicatorMap = std::unordered_map<AttachHandle, PhysXReplicator>;
// ObjectKey-keyed, matching
// UsdLoad::attachReplicatorFinish()'s PathSet (usdLoad/LoadTools.h) directly -- a live
// AttachedStage/Source now always exists before attach() below's replicationAttachFn callback
// fires (see attach()'s own comment), so it mints real ObjectKeys instead of building a
// string-path set. Locally-aliased rather than pulling in usdLoad/LoadTools.h's own KeySet name
// here.
using ReplicatorExcludePathSet =
    std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash>;

class PhysXReplicator
{
public:
    PhysXReplicator(const IReplicatorCallback& cb);
    ~PhysXReplicator();

    // Runs at attach time, so it is the one replicator entry point that cannot take an attach
    // handle: with attachStage == true this call is what *creates* the attach (through
    // UsdLoad::attachReplicatorCreateSource() -> UsdLoad::attach() -> UsdUtilsStageCache::Find()),
    // and handles are minted by the attach itself. The USD stage id it needs for that is what its
    // caller holds. attachReplicatorCreateSource() runs before either callback below, so both
    // always see the real attach identity now (see the reportedAttachHandle comment in the .cpp).
    // Returns false if attachReplicatorCreateSource()/attachReplicatorFinish() fails.
    bool attach(uint64_t stageId, PhysXUsdPhysicsInterface* usdPhysicsInt, bool attachStage);

    bool replicate(AttachHandle attachHandle, uint64_t path, uint32_t numReplications, bool useEnvIds);

    // Explicit per-clone target-root world poses supplied by the clone() entrypoint, as a flat
    // [numReplications * 7] array of (px,py,pz, qx,qy,qz,qw) -- position + imaginary-first
    // quaternion. Entry i anchors the exact target subtree root at transforms[i]. An authored
    // target prim pose takes precedence; otherwise the anchor is applied before the co-location
    // fallback. Empty (transforms==null) leaves clones co-located on the source, which is what raw
    // replicate() callers and NULL-transform clone() callers get.
    void setCloneTransforms(const float* transforms, uint32_t numClones)
    {
        if (transforms && numClones)
            mCloneTransforms.assign(transforms, transforms + static_cast<size_t>(numClones) * 7);
        else
            mCloneTransforms.clear();
    }

    // Explicit per-clone LOGICAL environment ids supplied by the clone() entrypoint
    // ([numReplications]). Copy i receives runtime env id envIds[i] + 1 (0 stays the source's
    // creation-time id), so the same caller id maps to the same runtime id in every batch — what
    // lets a multi-call ClonePlan (one clone() per source row) keep same-environment objects on one
    // id and colliding. Empty (envIds==null) keeps the positional numbering (i + 1 + per-stage
    // base), where each batch gets fresh ids.
    void setCloneEnvIds(const uint32_t* envIds, uint32_t numClones)
    {
        if (envIds && numClones)
            mCloneEnvIds.assign(envIds, envIds + numClones);
        else
            mCloneEnvIds.clear();
    }

    void clear();

private:
    IReplicatorCallback mCallback;
    ReplicatorExcludePathSet mExcludePathSet;
    std::vector<float> mCloneTransforms;
    std::vector<uint32_t> mCloneEnvIds;
};
} // namespace physx
} // namespace omni
