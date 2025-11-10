// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <omni/physx/IPhysxReplicator.h>
#include <unordered_map>
#include <unordered_set>

namespace omni
{
namespace physx
{
class PhysXReplicator;
class PhysXUsdPhysicsInterface;

using ReplicatorMap = std::unordered_map<uint64_t, PhysXReplicator>;
using PathSet = std::unordered_set<pxr::SdfPath, pxr::SdfPath::Hash>;

class PhysXReplicator
{
public:
    PhysXReplicator(const IReplicatorCallback& cb);
    ~PhysXReplicator();

    void attach(uint64_t stageId, PhysXUsdPhysicsInterface* usdPhysicsInt, bool attachStage);

    bool replicate(uint64_t stageId, uint64_t path, uint32_t numReplications, bool useEnvIds, bool useFabricReplicate);

    void clear();

    bool isFabricReplication() const
    {
        return mFabricReplication;
    }

private:
    IReplicatorCallback mCallback;
    PathSet mExcludePathSet;
    bool mFabricReplication;
};
} // namespace physx
} // namespace omni
