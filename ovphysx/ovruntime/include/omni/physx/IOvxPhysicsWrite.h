// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-INPUT-CORE-001
 * @covers AC-6
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

namespace omni::physx
{

// The app -> physics write (ADR-0012), the return direction of IOvxPhysicsRead.h.
// It reuses that header's query verbatim -- ovxQuery / ovxQueryDictionary /
// ovxFetchQueryResult / ovxReleaseQuery -- and adds no selection of its own: a write
// session reaches exactly the set the query matched, for ONE attribute.
//
// ONE ATTRIBUTE PER SESSION, unlike ovxReadAttributes which takes an array. Forced by
// the group type: ovstage_map_group_t is prims/data/meta and carries no `attribute`,
// `is_array` or `semantic` field, so a group from a multi-attribute session could not
// say which attribute it represents -- linear and angular velocity would be
// indistinguishable. ovstage's own map_attribute maps one attribute per session for the
// same reason. Writing several attributes over one prim set means several sessions.

typedef uint64_t OvxWriteHandle;

// Three-valued, mirroring OvxReadStatus: a distinct end-of-iteration and a real error
// can each be forwarded upward. On any non-Ok status `*outGroup` is null.
enum OvxWriteStatus : int32_t
{
    kOvxWriteStatusOk             = 0, //!< `*outGroup` points at the next group to fill.
    kOvxWriteStatusEndOfIteration = 1, //!< all groups consumed (NOT an error); `*outGroup` null.
    kOvxWriteStatusError          = 2, //!< bad handle / internal failure; `*outGroup` null.
};

// PIPELINE SUPPORT. The scatter is device-only today: it runs over the tensor backend's
// cached superset view, which SimulationBackend builds for the GPU pipeline and returns
// null for on CPU. A session opened against a CPU scene therefore fails at
// ovxWriteAttribute rather than emitting groups it cannot publish. The host path is
// planned and will not change this header's contract -- only which pipelines accept a
// session. REQ-INPUT-CORE-001 AC-1/AC-2/AC-3 describe the superset-and-rows shape that
// exists on the device side.
//
// STEP-FIRST PRECONDITION, as on the read: DirectGPU structures are sized during the
// first step, so a write issued before one has nothing to scatter into -- and is REFUSED,
// not silently warmed. This mirrors the read, which treats a pre-step scene as a clean
// omission (ADR-0008 Decision 10). The write never steps the simulation on the caller's
// behalf; a caller that wants initial state applied first calls ovphysx_step() /
// ovphysx_warmup() itself. (The tensor-binding write DOES auto-warm; the ovstage write
// deliberately does not -- see ADR-0012's 2026-08-27 amendment.)

extern "C"
{

// WRITE (ovstage `map_attribute` analog): open a session over `query` for ONE attribute,
// given as a string name (see OvxAttr) or an interned token. The write's attribute
// vocabulary is its own -- a name accepted here need not be one the read emits, since
// write-only control inputs such as forces and wrenches have no read counterpart.
// Returns 0 on a bad query handle, a null attribute, an attribute the queried type does
// not accept or that is not writable, or a scene whose pipeline the scatter does not
// serve.
OMNI_OVX_WRITE_API OvxWriteHandle ovxWriteAttribute(OvxOutputQueryHandle query,
                                                    const ovx_string_or_token_t* attr);

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
// The group and its borrowed storage stay valid until that group is committed,
// independent of further fetches; an intervening step does not invalidate a live group.
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
// cross-check the SAME query this write was opened over: `ovxFetchQueryResult`'s
// `total_prim_count` counts every matching prim regardless of disabled state (see
// IOvxPhysicsRead.h), so a nonzero count with no group means the matches were filtered
// out, not absent.
OMNI_OVX_WRITE_API OvxWriteStatus ovxFetchWriteNext(OvxWriteHandle write,
                                                    const ovstage_map_group_t** outGroup);

// Why a commit returned false. The bool alone cannot separate the two, and they call for
// different things from a caller: a group this session never handed out was rejected before
// anything ran, while a live group whose publish failed is spent either way.
enum OvxCommitFailure : int32_t
{
    kOvxCommitFailureNone    = 0, //!< the commit succeeded; `outFailure` is only written on false.
    kOvxCommitFailureNotLive = 1, //!< unknown session, or a group unknown, foreign or already
                                  //!< committed. Rejected before any publish, so nothing was written.
    kOvxCommitFailurePublish = 2, //!< the group WAS live and the publish failed. How much of it
                                  //!< reached the solver is not reported: a device scatter can fail
                                  //!< partway. The runtime logs the specific reason.
};

// COMMIT (ovstage `unmap_group` analog): publish `group` and transfer ownership of its
// buffers to physics. The caller must have filled EVERY mapped entry; there is no
// fill-mask, so a partially filled group publishes whatever its unfilled entries hold.
// After this the mapped pointers belong to physics and dereferencing them is undefined
// behavior -- a raw data.tensors[i].data access makes no call this could reject.
//
// `writeDoneSync` is ovstage's own handoff: a non-zero `wait_event` is waited on, and
// `{stream, 0}` -- queued work with no event -- still drains that stream. Only `{0, 0}`
// asserts nothing is outstanding; a host-resident group does not imply it, since the
// caller may have staged on its own stream. The runtime additionally orders the write
// against an in-flight step.
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

// Release the session. Every group that was never committed is DISCARDED, not published,
// which is why this takes no sync token -- there is nothing left to order against. A
// caller that fails or throws mid-fill therefore publishes nothing from the group it was
// filling, and forgetting to commit is a silent no-op rather than uninitialized data
// reaching the solver. Committed groups are NOT rolled back.
//
// Idempotent for an already-released or unknown handle, matching ovxReleaseRead.
OMNI_OVX_WRITE_API void ovxReleaseWrite(OvxWriteHandle write);

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
OMNI_OVX_WRITE_API DrainResult applyOvstageValueBatch(::omni::physics::parse::IPhysicsSource& source,
                                                      const ::omni::physics::parse::ChangeBatch& batch,
                                                      ::std::vector<uint32_t>* unresolvedRows = nullptr);

} // namespace omni::physx
