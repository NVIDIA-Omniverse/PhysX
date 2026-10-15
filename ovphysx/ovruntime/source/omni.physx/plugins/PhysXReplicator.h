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
#include <memory>
#include <atomic>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace omni
{
namespace physx
{
class PhysXReplicator;
class PhysXUsdPhysicsInterface;

// Exact live attachment keys; no aliases or pending registrations.
using ReplicatorMap = std::unordered_map<AttachHandle, std::shared_ptr<PhysXReplicator>>;
class PhysXReplicator
{
public:
    PhysXReplicator(const IReplicatorCallback& cb);
    ~PhysXReplicator();

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
    void cancel() { mCancelled.store(true, std::memory_order_release); }

private:
    std::atomic<bool> mCancelled{false};
    IReplicatorCallback mCallback;
    std::vector<float> mCloneTransforms;
    std::vector<uint32_t> mCloneEnvIds;
};
} // namespace physx
} // namespace omni
