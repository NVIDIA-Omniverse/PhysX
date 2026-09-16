// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-002
 * @covers AC-4
 *
 * @implements REQ-REPLICATE-002
 * @covers AC-4
 */
#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <omni/physics/AttachHandle.h>


namespace omni
{

namespace physx
{

static constexpr char kScenePartitionPrimvar[] = "primvars:omni:scenePartition";

/// Replication attach function, called when a stage is attached or about to be fully parsed.
///
/// This function can provide a list of exclude paths, these paths are skipped from the stage level parsing.
///
/// (Example: when cloning environemnts that are in hierarchy /World/envs/env0, /World/envs/env1, /World/envs/env2...
///     its is expected that the envs are replicated, hence the exclude path would be /World/envs)
///
/// \param[in] attachHandle The attach being parsed, as reported to @ref
/// IPhysxReplicator::registerReplicator (matches @ref IPhysxSimulation::getAttachHandle())
/// \param[in] numExludePaths Number of exclude paths
/// \param[in] excludePaths Exclude paths; each entry is an
/// `omni::physics::parse::ObjectKey::handle` (obtainable via @ref IPhysx::resolveObjectKey), no
/// longer an SdfPath-bit encoding -- an intentional, breaking ABI-contract change (ADR-0018),
/// matching `replicate()`'s own \p path contract below.
/// \param[in] userData User data
typedef void (*ReplicationAttachFn)(AttachHandle attachHandle,
                                    uint32_t& numExludePaths,
                                    uint64_t*& excludePaths,
                                    void* userData);

/// Replication attach end function, called when a stage has been parsed and is ready for replications.
///
/// \param[in] attachHandle The attach that finished parsing, as reported to @ref
/// IPhysxReplicator::registerReplicator (matches @ref IPhysxSimulation::getAttachHandle())
/// \param[in] userData User data
typedef void (*ReplicationAttachEndFn)(AttachHandle attachHandle, void* userData);

/// Hierarchy rename function
///
/// When a hierarchy is replicated, the top level path that is replicated is provided and its expected new top level
/// path is returned that matches the new replicated path.
///
/// (Example: when cloning environemnts that are in hierarchy /World/envs/env0, /World/envs/env1, /World/envs/env2...
///     its is expected that if we clone /World/envs/env0, the replicate path is /World/envs/env0 and for index 0
///     the returned path would be expected /World/envs/env1)
///
/// \param[in] replicatePath The base object being replicated, as an `omni::physics::parse::ObjectKey::handle`
/// (obtainable via @ref IPhysx::resolveObjectKey) -- no longer an SdfPath-bit encoding (breaking
/// change, ADR-0018: a caller still computing an `sdfPathToInt()`-style encoding gets wrong
/// results). Matches `replicate()`'s own \p path contract below.
/// \param[in] index The current index of replication.
/// \param[in] userData User data.
/// \return The new hierarchy's identity that replication should match, as an
/// `omni::physics::parse::ObjectKey::handle` (same contract as \p replicatePath above; 0 is the
/// invalid-key sentinel, unchanged from the legacy encoding).
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
    /// Register replicator to a given attach
    ///
    /// \note Registering on an attach handle rather than a stage id is what removes the stage-id
    /// hijack hazard: a lingering entry can no longer be picked up by a later re-attach that happens
    /// to reuse the same stage id (ADR-0016).
    ///
    /// \note Lifetime: a registration is consumed by at most the next attach it applies to, and is
    /// discarded once that attach detaches -- it does not survive a detach/reattach cycle even on the
    /// same stage. A caller that wants the replicator active again after a Kit stop/play cycle, or
    /// any other detachStage()+reattach, must call registerReplicator() again before/after each
    /// attach. Registering again under the same key while an attach is still live replaces the
    /// previously routed callback set; it does not stack or no-op.
    ///
    /// \param[in] attachHandle Attach to register on, from @ref IPhysxSimulation::getAttachHandle(),
    /// or kActiveAttach for the lone active attach.
    /// \param[in] callback IReplicatorCallback structure
    /// with callback functions
    /// \return True if replicator was sucesfully registered.
    bool(CARB_ABI* registerReplicator)(AttachHandle attachHandle, const IReplicatorCallback& callback);

    /// Unregister replicator from a given attach
    ///
    /// \param[in] attachHandle Attach to unregister from, from @ref
    /// IPhysxSimulation::getAttachHandle(), or kActiveAttach for the lone active attach.
    void(CARB_ABI* unregisterReplicator)(AttachHandle attachHandle);

    /// Replicate given hierarchy.
    ///
    /// \param[in] attachHandle Attach holding the hierarchy, from @ref
    /// IPhysxSimulation::getAttachHandle(), or kActiveAttach for the lone active attach.
    /// \param[in] path The hierarchy to clone, as an `omni::physics::parse::ObjectKey::handle`
    /// (obtainable via @ref IPhysx::resolveObjectKey) -- no longer a bit-cast of `pxr::SdfPath`'s
    /// private representation. This is an intentional, breaking ABI-contract change (ADR-0018): a
    /// caller still computing the legacy `sdfPathToInt()`-style encoding will pass a value this
    /// entry point no longer understands and get wrong (or no) results.
    /// \param[in] numReplications Number of times the hierarchy should be cloned.
    /// \param[in] setupEnvIds Setup EnvIds, this enables the possibility of co-location, envs are filtered out
    /// automatically.
    /// \return True replication was sucessful.
    bool(CARB_ABI* replicate)(
        AttachHandle attachHandle, uint64_t path, uint32_t numReplications, bool setupEnvIds);

    /// Check if given attach was replicated
    ///
    /// \param[in] attachHandle Attach to query, from @ref IPhysxSimulation::getAttachHandle(), or
    /// kActiveAttach for the lone active attach.
    /// \param[out] replicated Sets true if the attach is using replicator
    void(CARB_ABI* isReplicatorStage)(AttachHandle attachHandle, bool& replicated);

};

} // namespace physx
} // namespace omni
