// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-ATTACH-LIFETIME-001
 * @covers AC-1 AC-3
 *
 * @implements REQ-ATTACH-EXECUTION-001
 * @covers AC-1 AC-4
 *
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
    /// Register or replace the callbacks on an exact live attachment.
    /// Pass exclusions in PhysxAttachOptions when attaching, then register and replicate
    /// after attach returns successfully. No pre-attach registration or active sentinel exists.
    /// Detach removes only this owner's registration. Callback userdata must remain alive
    /// until unregistration/detach and return of any callback already executing.
    /// Callback reentry and this owner's pending simulation reject registration.
    bool(CARB_ABI* registerReplicator)(AttachHandle attachHandle, const IReplicatorCallback& callback);

    /// Cancel the registration immediately, including from its own callback or
    /// while a peer is pending. Already executing callbacks retain their registration
    /// storage until return; the caller must retain their userdata for that duration.
    ///
    /// \param[in] attachHandle Attach to unregister from, returned by a successful attachment.
    void(CARB_ABI* unregisterReplicator)(AttachHandle attachHandle);

    /// Replicate given hierarchy.
    ///
    /// \param[in] attachHandle Attach holding the hierarchy, returned by a successful attachment.
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
    /// \param[in] attachHandle Attach to query, returned by a successful attachment.
    /// \param[out] replicated Sets true if the attach is using replicator
    void(CARB_ABI* isReplicatorStage)(AttachHandle attachHandle, bool& replicated);

};

} // namespace physx
} // namespace omni
