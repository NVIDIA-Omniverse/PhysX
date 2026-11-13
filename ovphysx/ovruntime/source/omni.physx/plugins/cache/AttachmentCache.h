// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CACHE-INITIAL-001
 * @covers AC-3
 * @implements REQ-CACHE-UPDATE-001
 * @covers AC-6 AC-7 AC-8
 */
/** @implements REQ-CACHE-ATTACH-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-7
 */
#pragma once
#include <omni/physx/AttachOptions.h>
#include <omni/physx/ObjectId.h>
#include <exception>
#include <functional>
#include <stdexcept>
#include <private/omni/physx/CompiledCollection.h>
#include <private/omni/physx/RuntimeArchive.h>
#include <omni/physics/parse/Handles.h>

namespace omni::physx::usdparser
{
class AttachedStage;
}
namespace omni::physics::ovstage
{
class PhysicsInputSnapshot;
}
namespace omni::physx::cache
{
// Invoked under configuration admission on an unpublished candidate. A bundle
// is checked completely before the candidate changes: one it cannot use is
// rejected and the candidate loads ordinarily, as on a miss. A failure while the
// checked bundle is applied makes the caller discard the candidate and attach a
// fresh one ordinarily.
class AttachmentCache
{
public:
    AttachmentCache(usdparser::AttachedStage& owner, const PhysxCompiledStageCacheOptions* options);
    ~AttachmentCache();
    AttachmentCache(const AttachmentCache&) = delete;
    AttachmentCache& operator=(const AttachmentCache&) = delete;
    enum class Restore
    {
        eMiss,
        eHit,
        eFailed
    };
    // Starts a whole-stage restore with what needs no PhysX resources: the input
    // capture's first query, and for a stage of at least 10,000 prims the scene read
    // and the rest of the capture, which keeps running while the caller acquires them.
    // restore() finishes it, or does all of it when not begun.
    void beginRestore();
    Restore restore();
    // After an ordinary load: false when it did not take the scene restore() created
    // ahead. That reports a failed restore, so that the caller discards the candidate.
    bool preparedSceneTaken();
    void store();

private:
    struct Impl;
    std::unique_ptr<Impl> mImpl;
};
// False when the attachment's options keep the cache from storing or restoring
// anything (no external source, a USD stage, excluded paths, a forced scene):
// every attach and insertion then bypasses.
bool eligibleOptions(const usdparser::AttachedStage& owner);
// Returns true when a cache hit or the ordinary load handled this unit.
// Returns false when the caller must perform ordinary loading. A miss calls load
// exactly once while its selected input buffers are retained, then stores the
// initialized subtree before simulation can advance it; a load that returns
// false stores nothing.
bool loadCompiledInsertion(usdparser::AttachedStage& owner,
                           const std::vector<std::string>& roots,
                           const std::function<bool(const std::string&)>& load);
// Membership already discovered for this insertion, plus the records produced
// by its ordinary load. Capture does not rediscover these in the whole scene.
struct InsertionCaptureSelection
{
    std::vector<size_t> records;
    std::vector<omni::physics::parse::ObjectKey> keys;
    const std::vector<std::string>* paths = nullptr;
};
// Capture live state under the owner's admission into an empty roots collection
// and encode it into the required runtimeBytes; fails if encoding validation fails.
// Optional native work runs on the calling thread after the collection is
// complete, possibly alongside metadata encoding, and must leave roots
// unchanged. All work finishes before this returns. Inputs, when given, are the
// selected identity snapshot of this population and supply its prototype names.
bool captureAttachmentData(usdparser::AttachedStage& owner,
                           RuntimeArchive& archive,
                           ::physx::PxCollection& roots,
                           std::string_view subtree,
                           const std::function<bool()>& nativeWork,
                           omni::physics::parse::cache::Bytes* runtimeBytes,
                           const InsertionCaptureSelection* selection = nullptr,
                           const omni::physics::ovstage::PhysicsInputSnapshot* inputs = nullptr);
// Whole-attachment capture; the encoded bytes are validated and discarded.
bool captureAttachment(usdparser::AttachedStage& owner, RuntimeArchive& archive, ::physx::PxCollection& roots);
// The cooked stream that the runtime's mesh cache keeps for a triangle mesh, which
// encodeCollection stores in place of the mesh. A mesh without one sets *missing,
// when given, and is logged, at VERBOSE for an inserted unit, which can repeat for
// every environment.
CookedTriangleMesh cookedTriangleMeshes(bool insertion = false, bool* missing = nullptr);
omni::physics::parse::cache::Digest attachmentCompatibility(const usdparser::AttachedStage& owner);
void initializeChangeFeedBaseline(usdparser::AttachedStage& owner);
// Send the creation notifications ordinary loading sends for these restored
// records: link joints and fixed-base aliases are not announced, and an
// articulation with a fixed base is announced with the fixed-base type.
void notifyRestoredObjects(usdparser::AttachedStage& owner, const std::vector<usdparser::ObjectId>& created);
enum class Hydration
{
    eRejected, // the bundle does not fit; nothing changed
    eFailed, // the checked bundle contradicted itself after the owner changed
    eInterrupted, // the live stage or process failed after the owner changed
    eHydrated
};
// Thrown where a stored bundle contradicts itself, such as a stored ID outside the
// table it indexes. A failed restore removes its bundle only for a bundle at
// fault: rejected checks, eFailed or BundleDefect. Running out of memory, a worker
// that could not start, eInterrupted or any other exception, a std::logic_error
// included, is this process's, and the bundle stays for later attaches.
struct BundleDefect : std::runtime_error
{
    using std::runtime_error::runtime_error;
};
bool bundleAtFault(const std::exception_ptr& error);
// The collection must be deserialized. Call under serialization admission. Every
// check that can reject the bundle runs before the owner changes; *changing is
// set when the first change begins, also if it then throws. An insertion's
// registrations roll back on failure. created receives the committed records in
// creation order; their creation notifications are the caller's, after
// admission ends.
Hydration hydrateAttachment(usdparser::AttachedStage& owner,
                            const RuntimeArchive& archive,
                            std::unique_ptr<DecodedCollection> collection,
                            bool insertion = false,
                            bool* committed = nullptr,
                            std::vector<usdparser::ObjectId>* created = nullptr,
                            bool* changing = nullptr);
} // namespace omni::physx::cache
