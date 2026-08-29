// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <omni/physx/IPhysxReplicator.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace omni
{
namespace physx
{
class PhysXReplicator;
class PhysXUsdPhysicsInterface;

using ReplicatorMap = std::unordered_map<uint64_t, PhysXReplicator>;
using PathSet = std::unordered_set<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash>;

class PhysXReplicator
{
public:
    PhysXReplicator(const IReplicatorCallback& cb);
    ~PhysXReplicator();

    void attach(uint64_t stageId, PhysXUsdPhysicsInterface* usdPhysicsInt, bool attachStage);

    bool replicate(uint64_t stageId, uint64_t path, uint32_t numReplications, bool useEnvIds);

    // Explicit per-clone world poses supplied by the clone() entrypoint, as a flat
    // [numReplications * 7] array of (px,py,pz, qx,qy,qz,qw) -- position + imaginary-first
    // quaternion. Copy i is placed at transforms[i], applied to its root actor / articulation
    // root before the authored-prim / co-location fallbacks. Empty (transforms==null) leaves clones
    // co-located on the source, which is what raw replicate() callers and NULL-transform clone()
    // callers get.
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
    PathSet mExcludePathSet;
    std::vector<float> mCloneTransforms;
    std::vector<uint32_t> mCloneEnvIds;
};
} // namespace physx
} // namespace omni
