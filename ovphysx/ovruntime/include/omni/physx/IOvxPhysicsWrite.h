// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-INPUT-CORE-001
 * @covers AC-5 AC-6 AC-12 AC-13
 */

/**
 * @implements REQ-INPUT-DEVICE-001
 * @covers AC-3 AC-9 AC-10
 */

#pragma once

#include <ovstage/ovx_path_dictionary.h>           // ovx_primpath_list_t / ovx_path_dictionary_t
#include <ovstage/ovstage_api/ovstage_api_types.h> // ovstage_map_group_t / ovstage_cuda_sync_t
#include <ovx/string_types.h>                      // ovx_string_or_token_t

#include <omni/physx/IOvxPhysicsRead.h> // OvxObjectType / OvxObjectScope / OvxAttr / ovxQuery

#include <cstddef>
#include <cstdint>
#include <vector>

// Exported plugin symbols, as IOvxPhysicsRead.h -- see the note there.
#if defined(_MSC_VER)
#    if defined(OMNI_PHYSX_OVX_STATIC)
#        define OMNI_OVX_WRITE_API
#    elif defined(OMNI_PHYSX_OVX_EXPORTS)
#        define OMNI_OVX_WRITE_API __declspec(dllexport)
#    else
#        define OMNI_OVX_WRITE_API __declspec(dllimport)
#    endif
#else
#    define OMNI_OVX_WRITE_API __attribute__((visibility("default")))
#endif

namespace omni::physics::parse
{
class IPhysicsSource;
struct ChangeBatch;
} // namespace omni::physics::parse

namespace omni::physx::usdparser { class AttachedStage; }

namespace omni::physx
{

// One parent write session owns every requested attribute's plans. Each fetched
// group keeps its original ovstage pointer identity and names its request index.
typedef uint64_t OvxWriteHandle;

struct OvxWriteAttributeResult
{
    ovx_token_t attribute = OVX_INVALID_TOKEN;
    // Planned groups (batches), not objects or tensor rows; no promise of commit success.
    size_t groupCount = 0;
};

// Optional write-open summary. The caller owns this struct; attributes borrows
// attributeCount write-owned rows until ovxReleaseWrite releases the session.
// Copying this struct copies the attributes pointer, not its rows. Copy the rows before
// release to retain them longer; their tokens still require the owner's dictionary.
struct OvxWriteResult
{
    size_t totalPrimCount = 0; //!< Matched prim identities, not emitted rows or ready GPU objects.
    size_t attributeCount = 0; //!< Original request length.
    const OvxWriteAttributeResult* attributes = nullptr;
};

struct OvxFetchedWriteGroup
{
    const ovstage_map_group_t* group = nullptr;
    size_t attributeIndex = SIZE_MAX;
};

// Four results distinguish a group, end of iteration, error and Busy. End/error
// clear the group and set attributeIndex to SIZE_MAX; Busy preserves output and cursor.
enum OvxWriteStatus : int32_t
{
    kOvxWriteStatusOk             = 0, //!< `outGroup->group` points at the next group to fill.
    kOvxWriteStatusEndOfIteration = 1, //!< all groups consumed (NOT an error); `outGroup->group` null.
    kOvxWriteStatusBusy = 3, //!< runtime busy; output and cursor are unchanged.
    kOvxWriteStatusError          = 2, //!< bad handle / internal failure; `outGroup->group` null.
};

// PIPELINE SUPPORT. Host scatter serves CPU and GPU-with-readback scenes. DirectGPU
// (`eENABLE_DIRECT_GPU_API`) scatters through the tensor backend's cached GPU
// superset view, which PhysX sizes on the first step. REQ-INPUT-CORE-001 AC-1/AC-2/AC-3
// describe the superset-and-rows shape on the device side.
//
// NEVER AUTO-WARM. The ovstage write never steps the simulation on the caller's
// behalf (the tensor-binding write DOES auto-warm). A pre-step write is applied on
// CPU and on GPU with readback. On DirectGPU, commit is REFUSED until a first step
// has sized the GPU view -- a DirectGPU caveat, not a universal refusal. A caller
// that wants one recipe for all modes calls ovphysx_step() / ovphysx_warmup()
// itself. See ADR-0012's 2026-08-27 (no auto-warm) and 2026-09-14 (mode split)
// amendments.

// Writes require idle selected-owner admission. Release is allowed during that
// owner's pending interval. Callback reentry or concurrent owner entry returns
// false without dropping ownership; retry outside the conflicting call.
extern "C"
{

// An owner reset/reload that changes its record epoch invalidates existing write
// sessions. Discard their groups and open a fresh query/session before writing.
// Open a session for a nonempty attribute array. Token ids take precedence over
// strings and resolve in the query owner's dictionary. Canonical duplicates and
// unsupported/read-only attributes reject the entire plan before publication.
// The optional summary's nested rows belong to the session until release; owner
// teardown invalidates their tokens. A null outSummary skips optional row allocation,
// token interning and query membership counting. No output is published on failure.
OMNI_OVX_WRITE_API OvxWriteHandle ovxWriteAttributes(OvxOutputQueryHandle query,
                                                    const ovx_string_or_token_t* attributes,
                                                    size_t attributeCount,
                                                    OvxWriteResult* outSummary = nullptr);

// ITERATE (ovstage `fetch_map_next` analog): hand back the next group to fill.
//
// PRODUCER-OWNED, unlike ovxFetchReadNext which fills a caller-allocated
// ovstage_read_group_t by value. The write cannot: ovstage_map_group_t has no
// `write_group_id`, so the POINTER is the commit identity (see ovxCommitGroup) and only
// the producer's own address is meaningful. `const` is correct even though this is the
// write path -- the group is a descriptor the caller READS, describing buffers the
// caller FILLS through data.tensors[i].data, which const permits since it does not
// propagate through pointer members. No field of the struct is the caller's to assign.
//
// The group and its borrowed storage stay valid until that group's commit is
// accepted or the session is released, independent of further fetches. Owner
// teardown invalidates the session. An intervening step does not invalidate a live group.
// DISABLING a body the group holds does, however, for a DEVICE-BACKED group: membership is fixed when
// the group is handed out, so a disable landing before the commit leaves that body's now-invalid
// DirectGPU row in the set and that commit is refused as a whole -- enabled peers included. Host-only
// groups (mass, inertia, disableGravity, per-shape properties) carry no DirectGPU row and commit
// through setBodyPropertyOvStage unaffected. Re-plan (open a fresh session) if a body in a live
// device-backed group is disabled before that group is committed.
//
// No group is produced for a scene whose every matched body is filtered out -- on a
// DirectGPU scene a disabled rigid dynamic has no device state and is omitted, exactly as
// the read omits it (`disableSimulation` excepted). End-of-iteration with no group is
// therefore indistinguishable HERE from "no matching body". To tell the two apart,
// request the write-open summary: `totalPrimCount` counts every matching prim
// regardless of disabled state, so a nonzero count with no group means the matches
// were filtered out, not absent.
OMNI_OVX_WRITE_API OvxWriteStatus ovxFetchWriteNext(OvxWriteHandle write,
                                                    OvxFetchedWriteGroup* outGroup);

// Why a commit returned false. The bool alone cannot separate the two, and they call for
// different things from a caller: a group this session never handed out was rejected before
// anything ran, while a live group whose publish failed is spent either way.
enum OvxCommitFailure : int32_t
{
    kOvxCommitFailureBusy    = 3, //!< retryable; the live group remains uncommitted.
    kOvxCommitFailureNone    = 0, //!< the commit succeeded; `outFailure` is only written on false.
    kOvxCommitFailureNotLive = 1, //!< unknown session, or a group unknown, foreign or already
                                  //!< committed. Rejected before any publish, so nothing was written.
    kOvxCommitFailurePublish = 2, //!< the group WAS live and is spent. RuntimeErrorScope captures
                                  //!< the reason, including a preflight refusal when nothing was written.
                                  //!< A later scatter failure can have applied only part of the group.
};

// COMMIT (ovstage `unmap_group` analog): publish `group` and transfer ownership of its
// buffers to physics. The caller must have filled EVERY mapped entry; there is no
// fill-mask, so a partially filled group publishes whatever its unfilled entries hold.
// Once a live group is accepted, its mapped pointers belong to physics even if
// publication fails. Dereferencing them is undefined behavior -- a raw
// data.tensors[i].data access makes no call this could reject. Busy and NotLive
// refusals do not accept a group or consume a producer handoff.
//
// `writeDoneSync` hands off producer completion; it does not select the engine's
// consumer stream. Every nonzero field is honored before any backend preflight:
// this call drains `stream`, then waits for `wait_event`, observing completion on
// the host for both host and device groups. Only `{0, 0}` asserts nothing is
// outstanding and makes no CUDA call. Host-resident storage alone does not imply it.
// Context-relative stream sentinels resolve in the caller's current producer
// context; this handoff does not push the engine context. Keep both supplied
// handles valid until this call returns. A failed wait spends the group with
// kOvxCommitFailurePublish, reads and publishes nothing, and makes release retain
// its unsafe storage. The runtime additionally rejects a commit as Busy while
// an owner step is in flight.
//
// The current inline handoff releases the process-wide output-cache mutex while
// waiting, so independent owners can progress. It retains this owner's admission:
// conflicting operations on the same owner return Busy until the commit returns.
//
// Returns false for a handle or group pointer it does not recognise as LIVE. Commit is
// the mutation itself, so reporting success for a stale pointer would tell the caller
// state was published when nothing was -- this is deliberately NOT idempotent, unlike
// ovxReleaseWrite. Group addresses are unique for the session's life and never recycled
// between groups, which is what makes an already-committed group distinguishable from a
// live one.
//
// `outFailure` is optional and written ONLY on a false return, with which of the two cases above
// applies. A caller that passes null gets the bool and nothing else.
OMNI_OVX_WRITE_API bool ovxCommitGroup(OvxWriteHandle write,
                                       const ovstage_map_group_t* group,
                                       ovstage_cuda_sync_t writeDoneSync,
                                       OvxCommitFailure* outFailure = nullptr);

// Release the session. Every group that was never committed is discarded, not
// published. Finish external CPU and CUDA work filling uncommitted groups before
// release: this call has no producer completion token. Committed groups are not
// rolled back; their storage belongs to the runtime, including after a failed
// producer handoff. An accepted release ends all session borrows.
// Direct runtime callers must release write sessions before detaching their
// owner, while its dictionary is alive, to retire their prim-list references.
// The public ovphysx facade performs this cleanup before detach, reset and destroy.
//
// Idempotent for an already-released or unknown handle, matching ovxReleaseRead.
// False reports rejected admission (the session remains live) or a terminal release failure
// (unsafe device buffers or any group with unproven producer completion are quarantined).
OMNI_OVX_WRITE_API bool ovxReleaseWrite(OvxWriteHandle write);

} // extern "C"

// Outcome of the ovstage drain's value-apply. `false` cannot safely mean both "not handled" and
// "publish failed": a device scatter can fail after writing some rows, and a multi-scene batch can
// commit one scene before a later scene fails. Only `NotHandled` (nothing was committed) may fall back
// to the parse-layer path; `Failed` (something was committed, then a scatter failed) must NOT fall back
// (it would double-apply the committed part) and must stop the drain cursor from advancing so the batch
// can be retried.
enum class DrainResult : int32_t
{
    NotHandled = 0, //!< the drain did not touch PhysX (wrong attribute, cold cache, unsupported shape)
    Applied,        //!< the resolved keys were scattered (any skipped rows are reported in unresolvedRows)
    Failed          //!< a scatter failed after committing part of the batch
};

// The ovstage drain's value-apply. Scatters one value ChangeBatch through the write backend the
// session commit uses (planGroup + scatterGroup), feeding the ovstage value column straight in with a
// data-driven residency bridge (borrow a matching column, stage a mismatched one). onSourceChange
// calls it for a value change and falls back to the parse-layer property path only on NotHandled.
//
// `unresolvedRows`, when non-null, opts the caller into PARTIAL handling: the drain scatters the keys it
// resolves to a live body and appends the batch-row index of each key it must skip (a kinematic body, a
// local-frame velocity, a not-yet-created body, or a link an attribute cannot take) so the caller runs
// its per-object fallback for exactly those. `Applied` then means "the resolved subset was scattered;
// unresolvedRows holds the rows still to handle" (empty => the whole batch). A skipped key is never
// committed, so routing it to the fallback cannot double-apply. With `unresolvedRows` null the older
// all-or-nothing holds: a batch with any skipped key is NotHandled as a whole. When NO key resolves the
// batch is NotHandled either way.
// Not a C entry: it takes references.
OMNI_OVX_WRITE_API DrainResult applyOvstageValueBatch(usdparser::AttachedStage& owner, omni::physics::parse::IPhysicsSource& source,
                                                      const ::omni::physics::parse::ChangeBatch& batch,
                                                      ::std::vector<uint32_t>* unresolvedRows = nullptr);

} // namespace omni::physx
