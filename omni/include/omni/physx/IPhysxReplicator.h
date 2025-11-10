// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>


namespace omni
{

namespace physx
{

/// Replication attach function, called when a stage is attached or about to be fully parsed.
///
/// This function can provide a list of exclude paths, these paths are skipped from the stage level parsing.
///
/// (Example: when cloning environemnts that are in hierarchy /World/envs/env0, /World/envs/env1, /World/envs/env2...
///     its is expected that the envs are replicated, hence the exclude path would be /World/envs)
///
/// \param[in] stageId USD stageId (can be retrieved from a stagePtr -
/// pxr::UsdUtilsStageCache::Get().GetId(stagePtr).ToLongInt()) \param[in] numExludePaths Number of exclude paths
/// \param[in] excludePaths Exclude paths
/// \param[in] userData User data
typedef void (*ReplicationAttachFn)(uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData);

/// Replication attach end function, called when a stage has been parsed and is ready for replications.
///
/// \param[in] stageId USD stageId (can be retrieved from a stagePtr -
/// pxr::UsdUtilsStageCache::Get().GetId(stagePtr).ToLongInt()) \param[in] userData User data
typedef void (*ReplicationAttachEndFn)(uint64_t stageId, void* userData);

/// Hierarchy rename function
///
/// When a hierarchy is replicated, the top level path that is replicated is provided and its expected new top level
/// path is returned that matches the new replicated path.
///
/// (Example: when cloning environemnts that are in hierarchy /World/envs/env0, /World/envs/env1, /World/envs/env2...
///     its is expected that if we clone /World/envs/env0, the replicate path is /World/envs/env0 and for index 0
///     the returned path would be expected /World/envs/env1)
///
/// \param[in] replicatePath Base path that was replicated.
/// \param[in] index The current index of replication.
/// \param[in] userData User data.
/// \return New path to the hierarchy that replication should match.
typedef uint64_t (*HierarchyRenameFn)(uint64_t replicatePath, uint32_t index, void* userData);

/// Replicator callback structure holding function pointers for callbacks
struct IReplicatorCallback
{
    ReplicationAttachFn replicationAttachFn = { nullptr };
    ReplicationAttachEndFn replicationAttachEndFn = { nullptr };
    HierarchyRenameFn hierarchyRenameFn = { nullptr };

    void* userData = { nullptr };
};


/// Interface for replication
///
/// Replicator allows replication of given hierarchies, the replication leverages PhysX SDK binary serilization
/// and avoids reparsing of the same hierarchies. It is expected that the replicated hierarchies are identical except
/// for the root transformation.
struct IPhysxReplicator
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxReplicator", 2, 0)

    /// Register replicator to a given stage
    ///
    /// \param[in] id USD stageId (can be retrieved from a stagePtr -
    /// pxr::UsdUtilsStageCache::Get().GetId(stagePtr).ToLongInt())
    /// \param[in] callback IReplicatorCallback structure
    /// with callback functions
    /// \return True if replicator was sucesfully registered.
    bool(CARB_ABI* registerReplicator)(uint64_t stageId, const IReplicatorCallback& callback);

    /// Unregister replicator from a given stage
    ///
    /// \param[in] id USD stageId (can be retrieved from a stagePtr -
    /// pxr::UsdUtilsStageCache::Get().GetId(stagePtr).ToLongInt())
    void(CARB_ABI* unregisterReplicator)(uint64_t stageId);

    /// Replicate given hierarchy.
    ///
    /// \param[in] id USD stageId (can be retrieved from a stagePtr -
    /// pxr::UsdUtilsStageCache::Get().GetId(stagePtr).ToLongInt())
    /// \param[in] path The hierarchy to clone.
    /// \param[in] numReplications Number of times the hierarchy should be cloned.
    /// \param[in] setupEnvIds Setup EnvIds, this enables the possibility of co-location, envs are filtered out
    /// automatically. \param[in] useFabricReplicate Replicate hierarchy through Fabric.
    ///     \note The hierarchy that will be cloned has to be in USD, the replication
    ///     then does not need USD anymore, the newly created replicated hierarchies can be
    ///     in fabric only.
    /// \return True replication was sucessful.
    bool(CARB_ABI* replicate)(
        uint64_t stageId, uint64_t path, uint32_t numReplications, bool setupEnvIds, bool useFabricReplicate);

    /// Check if given stage was replicated
    ///
    /// \param[in] id USD stageId (can be retrieved from a stagePtr -
    /// pxr::UsdUtilsStageCache::Get().GetId(stagePtr).ToLongInt())
    /// \param[out] replicated Sets true if stage is using replicator
    /// \param[out] fabricReplicated Sets true if stage is using replicator and replication happened in fabric
    void(CARB_ABI* isReplicatorStage)(uint64_t stageId, bool& replicated, bool& fabricReplicated);

};

} // namespace physx
} // namespace omni
