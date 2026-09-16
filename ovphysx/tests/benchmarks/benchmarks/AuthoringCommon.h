// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// DEPRECATED (tensor-binding-deprecation): the tensor-binding helpers here retire with the binding.

/**
 * @implements REQ-CAPI-BENCHMARK-001
 * @covers AC-3 AC-4 AC-5
 *
 * @implements REQ-CAPI-BENCHMARK-004
 * @covers AC-2 AC-4
 */

// Shared helpers for the hidden Authoring.* and ContactReport benchmark rows.
//
// These mirror the small static helpers the c_samples each carry privately
// (string views, token names, attribute interning, path queries, bounded
// waits, seal-and-drain, pose readback). The benchmark rows live in one
// binary, so the helpers are factored here once.
//
// Stage-operation waits consume every per-op diagnostic before the next wait
// invalidates it. A bounded timeout fails the row, then blocks for final
// completion so teardown never destroys a stage with live operations or
// benchmark payload references. The outer benchmark-process timeout remains
// the hard bound. Completed stage operations are released. Population
// operations have no release API.
//
// Every release and teardown call returns a status. Discarding it would let a
// row that failed to retire a query, a read session, a tensor binding, a
// path-list reference or the Stage itself still publish a number, so all of
// them route through bmRecordFailure() here.

#pragma once

// OvstageLoad.h is deliberately not included. Only the attachment struct is
// shared with the always-on rows, and taking it from the sample header keeps
// OvstageLoad.h's unbounded-wait helpers out of the dependency graph for the
// rows below. The sample's own unbounded-wait entry points remain in include
// scope but are never called from here. Every wait in this header is bounded.
#include "../../c_samples/common/ovstage_sample.h"
#include "../framework/BmUtils.h"
#include "../BenchmarkFailure.h"

#include <ovphysx/experimental/ovphysx.hpp>
#include <ovx/path_dictionary/path_dictionary.h>
#include <ovx/path_dictionary/path_dictionary_utils.h>

#include <atomic>
#include <cstring>
#include <mutex>
#include <string>
#include <string_view>


namespace authoringbm
{

// Initial bounded wait. On timeout the pending op is recorded before a final
// completion wait keeps payload and stage teardown safe. The benchmark driver's
// outer process timeout remains the hard stop. ovstage timeouts are nanoseconds.
const uint64_t kWaitTimeoutNs = 30ULL * 1000000000ULL;


// Begin a run only when the prior run no longer owns a stage attachment and
// the row has not already failed. A retained attachment may still own pending
// references to benchmark-object payloads, so it must never be zeroed merely
// to make the next setup look clean.
inline bool prepareStageAttachmentForRun(ovphysx_sample_stage_attachment_t& attachment,
                                         const char* row,
                                         bool* leftoverAttachmentReported)
{
    if (attachment.stage)
    {
        if (!leftoverAttachmentReported || !*leftoverAttachmentReported)
        {
            bmRecordFailure(
                row, "previous run still owns an OVStage attachment; preserving attachment and owned payload");
            if (leftoverAttachmentReported)
            {
                *leftoverAttachmentReported = true;
            }
        }
        return false;
    }
    if (bmRowHasFailure(row))
    {
        return false;
    }
    attachment = {};
    return true;
}


inline ovx_string_t stringView(std::string_view s)
{
    ovx_string_t out;
    out.ptr = s.data();
    out.length = s.size();
    return out;
}


inline ovx_string_or_token_t nameString(std::string_view s)
{
    ovx_string_or_token_t name;
    name.token = 0;
    name.string.ptr = s.data();
    name.string.length = s.size();
    return name;
}


inline ovx_string_or_token_t nameToken(ovx_token_t token)
{
    ovx_string_or_token_t name;
    name.token = token;
    name.string.ptr = nullptr;
    name.string.length = 0;
    return name;
}


inline bool internAttr(ovstage_instance_t* stage, const char* name, ovx_token_t* out)
{
    path_dictionary_instance_t* dict = ovstage_get_path_dictionary(stage);
    if (!dict || !out)
    {
        return false;
    }
    ovx_string_t s = stringView(name);
    const ovx_api_result_t result = path_dictionary_create_tokens_from_strings(dict, &s, 1, out);
    return result.status == OVX_API_SUCCESS && *out != 0;
}


// A path list is refcounted storage in the dictionary. Discarding the release
// result neither confirms the reference was dropped nor reports the internal
// error that prevented it. A second release of the same reference would
// over-decrement, so the handle is always invalidated and only the diagnostic
// is conditional.
inline bool releasePathListChecked(path_dictionary_instance_t* dictionary,
                                   ovx_primpath_list_t& list,
                                   const char* row,
                                   const char* what)
{
    if (!dictionary || list == OVX_INVALID_PRIMPATH_LIST)
    {
        list = OVX_INVALID_PRIMPATH_LIST;
        return true;
    }
    const ovx_api_result_t release = path_dictionary_release_path_list_reference(dictionary, list);
    bool released = true;
    if (release.status != OVX_API_SUCCESS)
    {
        bmRecordFailure(row, "%s path-list release failed: status=%d", what, static_cast<int>(release.status));
        released = false;
    }
    list = OVX_INVALID_PRIMPATH_LIST;
    return released;
}


// Defined below, next to the other query helpers. Declared here because
// queryPath() has to retire a query it cannot hand back.
inline bool releaseQueryChecked(ovstage_instance_t* stage, ovstage_query_handle_t query, const char* row);


// queryPath() has two independently fallible steps, creating the query and
// releasing the temporary path-list reference, and only the both-succeeded
// outcome hands the caller a live query. A query that was created but whose
// call is going to return false belongs to queryPath(), because every caller
// reads false as "no handle exists" and would leak it.
//
// Factored out because the transition is unreachable from an integration
// test: queryPath() creates the path list it releases, so nothing outside can
// make that release fail.
inline bool queryPathMustDiscardQuery(bool queryCreated, bool pathListReleased)
{
    return queryCreated && !pathListReleased;
}


// A path list is usable only when the call succeeded and handed back a real
// handle. A success status carrying the invalid sentinel is a construction
// fault, not an empty result, and querying with it would misreport as
// "not found".
inline bool queryPathListIsUsable(ovx_api_status_t status, ovx_primpath_list_t list)
{
    return status == OVX_API_SUCCESS && list != OVX_INVALID_PRIMPATH_LIST;
}


// Returning false has two different meanings. Callers such as
// AuthoringPopulationChurn's remove loop read false as "that prim is already
// gone", a legitimate outcome of ovstage_query_from_path_list() that stays
// silent. Failing to obtain the dictionary or to build the path list is an
// infrastructure fault that says nothing about whether the prim exists, so it
// is recorded against the row and suppresses the row's record.
inline bool queryPath(ovstage_instance_t* stage, const char* path, ovstage_query_handle_t* out, const char* row)
{
    *out = OVSTAGE_INVALID_QUERY_HANDLE;
    path_dictionary_instance_t* dict = ovstage_get_path_dictionary(stage);
    if (!dict)
    {
        bmRecordFailure(row, "no path dictionary available to query %s", path);
        return false;
    }
    ovx_string_t s = stringView(path);
    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    const ovx_api_result_t built = path_dictionary_create_path_list_from_strings(dict, &s, 1, &list);
    if (!queryPathListIsUsable(built.status, list))
    {
        bmRecordFailure(row, "create_path_list_from_strings failed for %s: status=%d", path,
                        static_cast<int>(built.status));
        return false;
    }
    const ovstage_api_status_t status = ovstage_query_from_path_list(stage, list, out);
    const bool released = releasePathListChecked(dict, list, row, "query path-list");
    const bool created = status == OVSTAGE_OK && *out != OVSTAGE_INVALID_QUERY_HANDLE;
    if (created && released)
    {
        return true;
    }
    if (queryPathMustDiscardQuery(created, released))
    {
        // The path-list release already recorded the row failure. This only
        // retires the query, and records again if that too fails.
        releaseQueryChecked(stage, *out, row);
    }
    *out = OVSTAGE_INVALID_QUERY_HANDLE;
    return false;
}


inline bool releaseCompletedStageOp(ovstage_instance_t* stage, ovstage_op_id_t opIndex, const char* row, const char* what)
{
    if (!stage || opIndex == OVSTAGE_INVALID_OP_ID)
    {
        return true;
    }

    const ovstage_api_status_t releaseStatus = ovstage_release_op(stage, opIndex);
    if (releaseStatus != OVSTAGE_OK)
    {
        bmRecordFailure(row, "%s release_op failed: op_id=%llu status=%d (%s)", what,
                        static_cast<unsigned long long>(opIndex), static_cast<int>(releaseStatus),
                        ovstage_get_error_string(stage, releaseStatus));
        return false;
    }
    return true;
}


inline bool isCompletedStageWaitStatus(ovstage_api_status_t status)
{
    return status == OVSTAGE_OK || status == OVSTAGE_ERROR_OP_FAILED;
}


inline bool recordStageWaitErrors(
    ovstage_instance_t* stage, const ovstage_op_wait_result_t& waitResult, const char* row, const char* what)
{
    bool operationFailed = false;
    for (size_t i = 0; i < waitResult.error_op_id_count; ++i)
    {
        const ovstage_op_id_t failedOp = waitResult.error_op_ids[i];
        const ovx_string_t detail = ovstage_get_last_op_error(stage, failedOp);
        bmRecordFailure(row, "%s dependency failed: op_id=%llu %.*s", what, static_cast<unsigned long long>(failedOp),
                        static_cast<int>(detail.length), detail.ptr ? detail.ptr : "");
        operationFailed = true;
    }
    return operationFailed;
}


// Fail-closed: a non-zero error_op_id_count means at least one operation in the
// dependency chain failed, even when the wait itself returned OK. Per-op
// strings are transient and must be consumed before release or another wait.
inline bool waitStage(
    ovstage_instance_t* stage, ovstage_op_id_t opIndex, const char* row, const char* what, bool* outCompleted = nullptr)
{
    if (outCompleted)
    {
        *outCompleted = false;
    }

    ovstage_op_wait_result_t waitResult;
    std::memset(&waitResult, 0, sizeof(waitResult));
    ovstage_api_status_t status = ovstage_wait_op(stage, opIndex, kWaitTimeoutNs, &waitResult);

    bool operationFailed = recordStageWaitErrors(stage, waitResult, row, what);
    const bool timedOut = status == OVSTAGE_ERROR_TIMEOUT;

    if (timedOut)
    {
        bmRecordFailure(row,
                        "%s timed out after %llu s: lowest_pending_op_id=%llu; waiting for final completion before "
                        "teardown",
                        what, static_cast<unsigned long long>(kWaitTimeoutNs / 1000000000ULL),
                        static_cast<unsigned long long>(waitResult.lowest_pending_op_id));
        std::memset(&waitResult, 0, sizeof(waitResult));
        status = ovstage_wait_op(stage, opIndex, OVSTAGE_TIMEOUT_INFINITE, &waitResult);
        operationFailed = recordStageWaitErrors(stage, waitResult, row, what) || operationFailed;
    }

    const bool completed = isCompletedStageWaitStatus(status);
    if (outCompleted)
    {
        *outCompleted = completed;
    }
    if (!completed)
    {
        bmRecordFailure(row, "%s %s failed before completion: op_id=%llu status=%d (%s)", what,
                        timedOut ? "final wait" : "wait",
                        static_cast<unsigned long long>(opIndex), static_cast<int>(status),
                        ovstage_get_error_string(stage, status));
        return false;
    }

    if (status != OVSTAGE_OK && !operationFailed)
    {
        const ovx_string_t detail = ovstage_get_last_op_error(stage, opIndex);
        bmRecordFailure(row, "%s failed: op_id=%llu status=%d %.*s", what, static_cast<unsigned long long>(opIndex),
                        static_cast<int>(status), static_cast<int>(detail.length), detail.ptr ? detail.ptr : "");
        operationFailed = true;
    }

    const bool released = releaseCompletedStageOp(stage, opIndex, row, what);
    return !timedOut && status == OVSTAGE_OK && !operationFailed && released;
}


inline bool recordPopulationWaitErrors(
    const ovstage_population_op_wait_result_t& waitResult, const char* row, const char* what)
{
    bool operationFailed = false;
    for (size_t i = 0; i < waitResult.error_op_id_count; ++i)
    {
        const ovstage_population_op_id_t failedOp = waitResult.error_op_ids[i];
        const ovx_string_t detail = ovstage_population_get_last_op_error(failedOp);
        bmRecordFailure(row, "%s dependency failed: op_id=%llu %.*s", what, static_cast<unsigned long long>(failedOp),
                        static_cast<int>(detail.length), detail.ptr ? detail.ptr : "");
        operationFailed = true;
    }
    return operationFailed;
}


inline bool waitPopulation(ovstage_instance_t* stage, ovstage_population_op_id_t opIndex, const char* row, const char* what)
{
    ovstage_population_op_wait_result_t waitResult;
    std::memset(&waitResult, 0, sizeof(waitResult));
    ovstage_api_status_t status = ovstage_population_wait_op(stage, opIndex, kWaitTimeoutNs, &waitResult);

    bool operationFailed = recordPopulationWaitErrors(waitResult, row, what);
    const bool timedOut = status == OVSTAGE_ERROR_TIMEOUT;

    if (timedOut)
    {
        bmRecordFailure(row,
                        "%s timed out after %llu s: lowest_pending_op_id=%llu; waiting for final completion before "
                        "teardown",
                        what,
                        static_cast<unsigned long long>(kWaitTimeoutNs / 1000000000ULL),
                        static_cast<unsigned long long>(waitResult.lowest_pending_op_id));
        std::memset(&waitResult, 0, sizeof(waitResult));
        status = ovstage_population_wait_op(stage, opIndex, OVSTAGE_TIMEOUT_INFINITE, &waitResult);
        operationFailed = recordPopulationWaitErrors(waitResult, row, what) || operationFailed;
    }
    if (!isCompletedStageWaitStatus(status))
    {
        const ovx_string_t err = ovstage_population_get_last_error();
        bmRecordFailure(row, "%s %s failed before completion: status=%d %.*s", what,
                        timedOut ? "final wait" : "wait", static_cast<int>(status), static_cast<int>(err.length),
                        err.ptr ? err.ptr : "");
        return false;
    }
    if (status != OVSTAGE_OK && !operationFailed)
    {
        const ovx_string_t err = ovstage_population_get_last_error();
        bmRecordFailure(row, "%s failed: status=%d %.*s", what, static_cast<int>(status), static_cast<int>(err.length),
                        err.ptr ? err.ptr : "");
        operationFailed = true;
    }
    return !timedOut && status == OVSTAGE_OK && !operationFailed;
}


inline bool recordOvphysxWaitErrors(const ovphysx::physx::WaitResult& wait, const char* row, const char* what)
{
    bool operationFailed = false;
    for (size_t i = 0; i < wait.errorCount(); ++i)
    {
        const ovphysx_op_index_t failed = wait.errorOpIndexAt(i);
        const ovphysx_string_t detail = ovphysx_get_last_op_error(failed);
        bmRecordFailure(row, "%s dependency failed: op_index=%llu %.*s", what,
                        static_cast<unsigned long long>(failed), static_cast<int>(detail.length),
                        detail.ptr ? detail.ptr : "");
        operationFailed = true;
    }
    return operationFailed;
}


// The ovphysx analogue of waitStage(). ovphysx_wait_op() reports failure two
// independent ways: through its own status (OVPHYSX_API_TIMEOUT, NOT_FOUND for
// an invalid or already-consumed index, ERROR for an internal wait failure)
// and through the per-operation error list for work that failed after its
// enqueue had already returned success. A caller that inspects hasErrors()
// alone reads a timeout or a not-found handle with an empty error list as
// success. Both are checked here.
//
// A bounded wait that times out records the failure and then blocks for final
// completion, so teardown never destroys state a pending operation still
// references. The outer benchmark-process timeout remains the hard bound.
inline bool waitOvphysxOpChecked(ovphysx_handle_t handle, ovphysx_op_index_t opIndex, const char* row, const char* what)
{
    ovphysx::physx::WaitResult wait;
    ovphysx_result_t result = ovphysx_wait_op(handle, opIndex, kWaitTimeoutNs, wait.get());

    bool operationFailed = recordOvphysxWaitErrors(wait, row, what);
    const bool timedOut = result.status == OVPHYSX_API_TIMEOUT;

    if (timedOut)
    {
        bmRecordFailure(row,
                        "%s timed out after %llu s: lowest_pending_op_index=%llu; waiting for final completion "
                        "before teardown",
                        what, static_cast<unsigned long long>(kWaitTimeoutNs / 1000000000ULL),
                        static_cast<unsigned long long>(wait.lowestPendingOpIndex()));
        // A timed-out index is not consumed, so this re-wait is legal and is
        // the only way to reach a state where teardown is safe.
        ovphysx::physx::WaitResult finalWait;
        result = ovphysx_wait_op(handle, opIndex, OVPHYSX_TIMEOUT_INFINITE, finalWait.get());
        operationFailed = recordOvphysxWaitErrors(finalWait, row, what) || operationFailed;
    }

    // A non-SUCCESS status alongside recorded per-op errors is the same fault
    // reported twice, so only the otherwise-silent statuses are reported here.
    if (result.status != OVPHYSX_API_SUCCESS && !operationFailed)
    {
        const ovphysx_string_t err = ovphysx_get_last_error();
        bmRecordFailure(row, "%s %s failed: status=%d %.*s", what, timedOut ? "final wait" : "wait",
                        static_cast<int>(result.status), static_cast<int>(err.length), err.ptr ? err.ptr : "");
    }

    return !timedOut && result.status == OVPHYSX_API_SUCCESS && !operationFailed;
}


inline bool waitOvphysxAllChecked(ovphysx_handle_t handle, const char* row, const char* what)
{
    return waitOvphysxOpChecked(handle, OVPHYSX_OP_INDEX_ALL, row, what);
}


// Retires a write whose completion was proven transitively by a later seal
// wait, or performs one bounded cleanup wait after an intervening failure.
//
// ovstage_write_attribute() returns an op id that the caller owns. Rows that
// wait only on the seal still have to release the write's own tracking state.
// After a completed seal wait the write is known complete and only needs
// releasing. Otherwise it receives a bounded wait followed, on timeout, by the
// final completion wait used by waitStage().
inline bool retireWriteOp(ovstage_instance_t* stage, ovstage_op_id_t opIndex, bool completionProven, const char* row)
{
    if (!stage || opIndex == OVSTAGE_INVALID_OP_ID)
    {
        return true;
    }
    if (!completionProven)
    {
        return waitStage(stage, opIndex, row, "write cleanup");
    }
    return releaseCompletedStageOp(stage, opIndex, row, "completed write");
}


// Seal the write floor through `ordinal`, then drain the matching interval into
// physics. Advances *lastDrained on success.
inline bool sealAndDrain(ovphysx_handle_t handle,
                         ovstage_instance_t* stage,
                         ovstage_ordinal_t* lastDrained,
                         ovstage_ordinal_t ordinal,
                         const char* row,
                         bool* outSealCompleted = nullptr)
{
    if (outSealCompleted)
    {
        *outSealCompleted = false;
    }
    ovstage_write_floor_desc_t writeFloor;
    std::memset(&writeFloor, 0, sizeof(writeFloor));
    writeFloor.ordinal = ordinal;
    writeFloor.scope = OVSTAGE_SCOPE_ALL;

    const ovstage_enqueue_result_t seal = ovstage_advance_write_floor(stage, &writeFloor);
    if (seal.status != OVSTAGE_OK)
    {
        bmRecordFailure(row, "advance_write_floor enqueue rejected: %d", static_cast<int>(seal.status));
        return false;
    }
    if (!waitStage(stage, seal.op_index, row, "advance_write_floor", outSealCompleted))
    {
        return false;
    }

    ovstage_ordinal_range_t range;
    std::memset(&range, 0, sizeof(range));
    range.has_start_ordinal = true;
    range.start_ordinal = (*lastDrained) + 1;
    range.end_ordinal = ordinal;
    if (ovphysx_update_from_ovstage(handle, range).status != OVPHYSX_API_SUCCESS)
    {
        const ovphysx_string_t err = ovphysx_get_last_error();
        bmRecordFailure(row, "update_from_ovstage failed: %.*s", static_cast<int>(err.length), err.ptr ? err.ptr : "");
        return false;
    }

    *lastDrained = ordinal;
    return true;
}


// Every ovphysx release returns a status. Discarding it hides both the
// internal failure that stopped the reclaim and the programming error of
// releasing a handle that was never issued (OVPHYSX_API_NOT_FOUND), so a row
// that leaks a query, read session or read group across the pinned lifecycle
// would still publish a number. The handle is spent whatever the status says,
// so ownership is dropped either way and only the diagnostic is conditional.
inline bool releaseOvphysxResultChecked(const ovphysx_result_t& release, const char* row, const char* what)
{
    if (release.status == OVPHYSX_API_SUCCESS)
    {
        return true;
    }
    const ovphysx_string_t err = ovphysx_get_last_error();
    bmRecordFailure(row, "%s release failed: status=%d %.*s", what, static_cast<int>(release.status),
                    static_cast<int>(err.length), err.ptr ? err.ptr : "");
    return false;
}


// ovphysx_destroy_tensor_binding() documents the handle as invalid after the
// call whatever it returns, so the caller's copy is always cleared. Retrying a
// failed destroy would be a second release of a spent handle.
inline bool destroyTensorBindingChecked(ovphysx_handle_t handle,
                                        ovphysx_tensor_binding_handle_t& binding,
                                        const char* row,
                                        const char* what)
{
    if (binding == 0)
    {
        return true;
    }
    const ovphysx_result_t destroy = ovphysx_destroy_tensor_binding(handle, binding);
    const bool destroyed = releaseOvphysxResultChecked(destroy, row, what);
    binding = 0;
    return destroyed;
}


// Counts simulated rigid bodies across the whole stage.
inline bool countRigidBodies(ovphysx_handle_t handle, uint32_t* outCount, const char* row)
{
    *outCount = 0;

    ovphysx_query_handle_t query = 0;
    if (ovphysx_query(handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query).status != OVPHYSX_API_SUCCESS ||
        query == 0)
    {
        return false;
    }

    const ovx_string_or_token_t name = nameString(OVPHYSX_ATTR_POSITION);
    ovphysx_read_handle_t read = 0;
    if (ovphysx_read(handle, query, &name, 1, &read).status != OVPHYSX_API_SUCCESS || read == 0)
    {
        releaseOvphysxResultChecked(ovphysx_release_query(handle, query), row, "rigid-body query");
        return false;
    }

    bool valid = true;
    for (;;)
    {
        const ovstage_read_group_t* group = nullptr;
        const ovphysx_result_t result = ovphysx_fetch_read_next(handle, read, &group);
        if (result.status == OVPHYSX_API_END_OF_ITERATION)
        {
            break;
        }
        if (result.status != OVPHYSX_API_SUCCESS || !group)
        {
            valid = false;
            break;
        }

        if (!group->is_delete && !group->is_array && group->data.tensor_count == 1 && group->data.tensors)
        {
            const DLTensor* tensor = &group->data.tensors[0];
            if (!tensor->data || tensor->device.device_type != kDLCPU || tensor->dtype.code != kDLFloat ||
                tensor->dtype.bits != 32 || tensor->dtype.lanes < 3 || tensor->ndim != 1 || !tensor->shape)
            {
                valid = false;
            }
            else
            {
                *outCount += static_cast<uint32_t>(tensor->shape[0]);
            }
        }
        if (!releaseOvphysxResultChecked(
                ovphysx_release_group(handle, read, group->read_group_id), row, "rigid-body read group"))
        {
            valid = false;
        }
        if (!valid)
        {
            break;
        }
    }

    if (!releaseOvphysxResultChecked(ovphysx_release_read(handle, read), row, "rigid-body read session"))
    {
        valid = false;
    }
    if (!releaseOvphysxResultChecked(ovphysx_release_query(handle, query), row, "rigid-body query"))
    {
        valid = false;
    }
    return valid;
}


// Pose readback for a single prim path. *outCount is the rigid-body match count
// (0 when the body is gone). When it is >= 1, *outY is the first body's pose Y.
inline bool readBodyPoseAt(ovphysx_handle_t handle, const char* path, int64_t* outCount, float* outY, const char* row)
{
    *outCount = 0;

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc;
    std::memset(&desc, 0, sizeof(desc));
    desc.pattern = ovphysx_cstr(path);
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
    if (ovphysx_create_tensor_binding(handle, &desc, &binding).status != OVPHYSX_API_SUCCESS)
    {
        return false;
    }

    ovphysx_tensor_spec_t spec;
    std::memset(&spec, 0, sizeof(spec));
    if (ovphysx_get_tensor_binding_spec(handle, binding, &spec).status != OVPHYSX_API_SUCCESS)
    {
        destroyTensorBindingChecked(handle, binding, row, "pose readback binding");
        return false;
    }

    // A removed body yields an empty binding rather than an error, so a count of
    // zero is a valid answer. The remove half of the lifecycle case asserts it.
    const int64_t count = (spec.ndim >= 1) ? spec.shape[0] : 0;
    *outCount = count;
    if (count < 1)
    {
        return destroyTensorBindingChecked(handle, binding, row, "pose readback binding");
    }

    float pose[7] = { 0.0f };
    int64_t shape[2] = { 1, 7 };
    DLTensor tensor;
    std::memset(&tensor, 0, sizeof(tensor));
    tensor.data = pose;
    tensor.ndim = 2;
    tensor.shape = shape;
    tensor.dtype.code = kDLFloat;
    tensor.dtype.bits = 32;
    tensor.dtype.lanes = 1;
    tensor.device.device_type = kDLCPU;

    const bool ok = ovphysx_read_tensor_binding(handle, binding, &tensor).status == OVPHYSX_API_SUCCESS;
    const bool destroyed = destroyTensorBindingChecked(handle, binding, row, "pose readback binding");
    if (!ok || !destroyed)
    {
        return false;
    }
    if (outY)
    {
        *outY = pose[1];
    }
    return true;
}


// ovstage_release_query() enqueues a per-handle-ordered operation. Discarding
// its result neither confirms the handle was reclaimed nor retires the
// release's own tracking state. The ovstage contract requires all operations
// and handles to be retired before ovstage_destroy_instance(), and
// benchmarkClearOvstage() only resets ovphysx before destroying the stage.
inline bool releaseQueryChecked(ovstage_instance_t* stage, ovstage_query_handle_t query, const char* row)
{
    if (!stage || query == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        return true;
    }
    const ovstage_enqueue_result_t release = ovstage_release_query(stage, query);
    if (release.status != OVSTAGE_OK)
    {
        bmRecordFailure(row, "release_query enqueue rejected: %d", static_cast<int>(release.status));
        return false;
    }
    return waitStage(stage, release.op_index, row, "release_query");
}


// ---------------------------------------------------------------------------
// Bounded, checked OVStage setup and teardown.
// ---------------------------------------------------------------------------
//
// The attach helper in tests/c_samples/common/ovstage_sample.h waits with
// OVSTAGE_TIMEOUT_INFINITE, reports through fprintf, and after a failed
// write-floor wait destroys the instance without retiring that operation. For
// a standalone sample that is fine. A wedged sample hangs, its process is
// killed, and nothing downstream reads a number out of it.
//
// The benchmark rows cannot use it. An unbounded wait turns a wedge into an
// indefinite hang inside a measured row, and an fprintf diagnostic does not
// reach bmRecordFailure(), so the row would still publish a number. These
// helpers repeat the same operation sequence through the shared bounded waits,
// which record the failure and only then block for final completion so no
// still-pending operation is ever abandoned.

enum class UsdSource
{
    eFile,
    eString,
};


// Retire a Stage that ovphysx does not hold, either never attached or already
// detached by the teardown below. Every wait on the path to here has reached
// final completion, so nothing is pending. Population operations have no
// release API.
//
// On failure the Stage is kept. The caller's `attached` flag stays false, so
// teardown retries the plain destroy rather than attempting an ovphysx detach
// on a Stage ovphysx never took.
inline bool destroyStageChecked(ovphysx_sample_stage_attachment_t& attachment, const char* row)
{
    if (!attachment.stage)
    {
        attachment = {};
        return true;
    }
    const ovstage_api_status_t destroy = ovstage_destroy_instance(attachment.stage);
    if (destroy != OVSTAGE_OK)
    {
        bmRecordFailure(row, "ovstage_destroy_instance failed: %d (%s); retaining caller-owned Stage",
                        static_cast<int>(destroy), ovstage_get_error_string(attachment.stage, destroy));
        return false;
    }
    attachment = {};
    return true;
}


// Populate a fresh Stage from a USD file or from generated USDA text and
// attach it: create, populate, seal the population ordinal with SCOPE_ALL,
// attach, then drain the attach.
//
// *outAttached is set as soon as the attach is accepted, before the wait that
// could still report it failed asynchronously. It selects the teardown path,
// and once ovphysx has taken the Stage that path has to be the detach one even
// if the attach then failed.
inline bool populateAndAttachChecked(ovphysx::PhysX* physx,
                                     UsdSource source,
                                     const std::string& payload,
                                     const char* instanceName,
                                     ovphysx_sample_stage_attachment_t& attachment,
                                     const char* row,
                                     bool* outAttached)
{
    attachment = {};
    attachment.ordinal = 1;
    *outAttached = false;

    if (!physx)
    {
        bmRecordFailure(row, "no ovphysx instance available for ovstage attach");
        return false;
    }

    // The application owns schema registration. The sample helper does it once per process.
    if (!ovphysx_sample_register_physx_schemas())
    {
        bmRecordFailure(row, "PhysX schema registration with ovstage failed");
        return false;
    }

    ovstage_instance_desc_t desc;
    std::memset(&desc, 0, sizeof(desc));
    desc.name = instanceName;
    const ovstage_api_status_t createStatus = ovstage_create_instance(&desc, &attachment.stage);
    if (createStatus != OVSTAGE_OK || !attachment.stage)
    {
        bmRecordFailure(row, "ovstage_create_instance failed: %d", static_cast<int>(createStatus));
        attachment = {};
        return false;
    }

    const ovx_string_t payloadView = stringView(payload);
    const ovstage_population_enqueue_result_t populate =
        source == UsdSource::eFile ?
            ovstage_population_open_usd_from_file(
                attachment.stage, payloadView, attachment.ordinal, 0.0, OVSTAGE_POPULATION_DOMAIN_PHYSICS) :
            ovstage_population_open_usd_from_string(
                attachment.stage, payloadView, attachment.ordinal, 0.0, OVSTAGE_POPULATION_DOMAIN_PHYSICS);
    if (populate.status != OVSTAGE_OK)
    {
        const ovx_string_t err = ovstage_population_get_last_error();
        bmRecordFailure(row, "population open_usd rejected: %d %.*s", static_cast<int>(populate.status),
                        static_cast<int>(err.length), err.ptr ? err.ptr : "");
        destroyStageChecked(attachment, row);
        return false;
    }
    if (!waitPopulation(attachment.stage, populate.op_index, row, "population open_usd"))
    {
        destroyStageChecked(attachment, row);
        return false;
    }

    ovstage_write_floor_desc_t floorDesc;
    std::memset(&floorDesc, 0, sizeof(floorDesc));
    floorDesc.ordinal = attachment.ordinal;
    floorDesc.scope = OVSTAGE_SCOPE_ALL;
    const ovstage_enqueue_result_t floor = ovstage_advance_write_floor(attachment.stage, &floorDesc);
    if (floor.status != OVSTAGE_OK)
    {
        bmRecordFailure(row, "population advance_write_floor enqueue rejected: %d", static_cast<int>(floor.status));
        destroyStageChecked(attachment, row);
        return false;
    }
    // waitStage() releases the completed floor operation, including on the
    // failure paths where the sample abandons it.
    if (!waitStage(attachment.stage, floor.op_index, row, "population advance_write_floor"))
    {
        destroyStageChecked(attachment, row);
        return false;
    }

    if (physx->attachOvstage(attachment.stage, attachment.ordinal) != OVPHYSX_API_SUCCESS)
    {
        const ovphysx_string_t err = ovphysx_get_last_error();
        bmRecordFailure(row, "attach_ovstage failed: %.*s", static_cast<int>(err.length), err.ptr ? err.ptr : "");
        destroyStageChecked(attachment, row);
        return false;
    }
    *outAttached = true;
    return waitOvphysxAllChecked(physx->handle(), row, "attach_ovstage");
}


// Keep the attachment intact if any teardown step fails, so a later run cannot
// silently orphan and replace the caller-owned Stage. Pass a null `physx` for
// a Stage ovphysx never took. Attempting to detach one reports a failure that
// misattributes the original fault.
inline bool clearOvstageChecked(ovphysx::PhysX* physx, ovphysx_sample_stage_attachment_t& attachment, const char* row)
{
    if (!attachment.stage)
    {
        attachment = {};
        return true;
    }
    if (physx)
    {
        // ovphysx_reset_stage() is asynchronous. Detaching or destroying under
        // a pending reset would tear the Stage out from under it, so the reset
        // is waited to completion before either.
        const ovphysx_enqueue_result_t reset = ovphysx_reset_stage(physx->handle());
        if (reset.status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t err = ovphysx_get_last_error();
            bmRecordFailure(row, "reset_stage enqueue rejected: status=%d %.*s; retaining caller-owned Stage",
                            static_cast<int>(reset.status), static_cast<int>(err.length), err.ptr ? err.ptr : "");
            return false;
        }
        if (!waitOvphysxOpChecked(physx->handle(), reset.op_index, row, "reset_stage"))
        {
            return false;
        }
        const ovphysx_result_t detach = ovphysx_detach_ovstage(physx->handle());
        if (detach.status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t err = ovphysx_get_last_error();
            bmRecordFailure(row, "detach_ovstage failed: status=%d %.*s; retaining caller-owned Stage",
                            static_cast<int>(detach.status), static_cast<int>(err.length), err.ptr ? err.ptr : "");
            return false;
        }
    }
    return destroyStageChecked(attachment, row);
}


// ---------------------------------------------------------------------------
// GPU fallback and capacity evidence.
// ---------------------------------------------------------------------------
//
// forceGpu() is only a CLI pass selector and ovphysx_warmup() returns SUCCESS
// even when no CUDA context exists (for example under OVPHYSX_DISABLE_GPU=1).
// Neither is execution-device evidence, so a _gpu row could publish a
// CPU-produced number.
//
// ovphysx exposes no query for the realized dynamics/broadphase device, but the
// runtime logs known CPU fallback paths at WARNING level while attaching. It
// also reports known GPU contact-buffer capacity failures at WARNING level.
// The GPU-broadphase diagnostic is process-wide WARN_ONCE, not a per-scene
// signal, so CPU-fallback evidence stays sticky for the process. Once a
// fallback is observed, every later GPU-labelled row is rejected too. Capacity
// diagnostics can fire per attach and are reset per row. Silence before any
// observed fallback is only negative evidence. It cannot prove that the
// requested GPU pipeline was realized.
//
// This detector matches diagnostic text, so rewording can silently weaken it.
// A first-class realized-device query would be needed for positive evidence.

struct GpuFallbackEvidence
{
    std::atomic<bool> cpuFallback{ false };
    std::atomic<bool> capacityWarning{ false };
    std::mutex mutex;
    std::string evidence;
};

inline bool isGpuCapacityWarningText(const std::string& text)
{
    // Match the loss/overflow diagnostics emitted by the PhysX sources, not
    // bare configuration names that can also appear in benign warnings.
    return text.find("Contact buffer overflow detected") != std::string::npos ||
           text.find("Patch buffer overflow detected") != std::string::npos ||
           text.find("Contacts have been dropped") != std::string::npos ||
           text.find("The application needs to increase PxGpuDynamicsMemoryConfig::foundLostPairsCapacity") !=
               std::string::npos ||
           text.find(
               "The application needs to increase PxGpuDynamicsMemoryConfig::foundLostAggregatePairsCapacity") !=
               std::string::npos ||
           text.find("The application needs to increase PxGpuDynamicsMemoryConfig::totalAggregatePairsCapacity") !=
               std::string::npos;
}

inline void gpuFallbackLogFn(ovphysx_log_level_t /*severity*/,
                             ovphysx_string_t message,
                             ovphysx_string_t /*channel*/,
                             double /*timestamp*/,
                             void* userData)
{
    if (!message.ptr || message.length == 0 || !userData)
    {
        return;
    }
    // The message is NUL-terminated at ptr[length] (REQ-CAPI-LOG-001 AC-2), so
    // strstr would also be safe. The bounded construction honours the length
    // the API supplies rather than depending on that guarantee.
    const std::string text(message.ptr, message.length);
    const bool fallback = text.find("falling back to ePABP") != std::string::npos ||
                          text.find("CUDA context manager") != std::string::npos ||
                          text.find("CUDA context unavailable") != std::string::npos;
    const bool capacity = isGpuCapacityWarningText(text);
    if (!fallback && !capacity)
    {
        return;
    }
    GpuFallbackEvidence* probe = static_cast<GpuFallbackEvidence*>(userData);
    std::lock_guard<std::mutex> lock(probe->mutex);
    if (probe->evidence.empty())
    {
        probe->evidence = text;
    }
    if (fallback)
    {
        probe->cpuFallback.store(true);
    }
    if (capacity)
    {
        probe->capacityWarning.store(true);
    }
}


// The user_data ovphysx_set_log_callback() publishes has to outlive every
// window in which the runtime may still hold it. Under REQ-CAPI-LOG-001 the
// callback and user_data must remain valid until a later replacing or
// disabling call, or a successful ovphysx_shutdown(), returns, and an
// OVPHYSX_API_ERROR may be reported after the replacement was published. A
// registration that reports failure may therefore still have installed the
// pointer.
//
// Scope-owned probe storage cannot be made safe by status checking. A failed
// registration or a failed disable would leave the process-global slot
// pointing at a destroyed benchmark object, and the callback fires from
// runtime threads. The probe lives for the life of the process instead. Only
// one GpuFallbackScope is ever alive at a time because the harness runs rows
// sequentially, so a single shared instance is enough, and the scope resets it
// when it arms.
inline GpuFallbackEvidence& gpuFallbackProbe()
{
    static GpuFallbackEvidence probe;
    return probe;
}


// Scoped capture from construction through destruction. Most Authoring rows
// bound the scope to attach and warmup. ContactReport keeps it through measured
// steps and stage teardown. The callback may fire from any thread, so the probe
// is mutex-guarded and unregistration is explicit. ovphysx guarantees the
// callback is no longer running once the disabling call returns.
class GpuFallbackScope
{
public:
    // `arm` is false for CPU rows. They determine device by construction from
    // the scene and never consult the probe, so the callback slot is left
    // alone for them. The slot is single and LabCartpole also installs a
    // callback. Sorted row order runs and destroys Authoring before
    // LabCartpole, so their lifetimes do not overlap.
    //
    // Every call site passes the row's own `gpu` flag, so `arm == gpu` holds by
    // convention rather than by construction. Getting it wrong is not silent.
    // An unarmed GPU row leaves the detector unregistered, and
    // checkDeviceEvidenceAndWarmup() refuses to publish any GPU row whose
    // detector was not installed.
    GpuFallbackScope(bool arm, const char* row) : mRow(row), mArmed(arm)
    {
        if (!mArmed)
        {
            return;
        }
        {
            GpuFallbackEvidence& probe = gpuFallbackProbe();
            std::lock_guard<std::mutex> lock(probe.mutex);
            // Capacity diagnostics can fire every attach. The GPU-broadphase
            // fallback is WARN_ONCE for the process, so cpuFallback stays sticky.
            probe.capacityWarning.store(false);
            if (!probe.cpuFallback.load())
            {
                probe.evidence.clear();
            }
        }
        // Single-slot API. Setting replaces any existing callback and NULL
        // disables. The harness's sequential row lifetime prevents this scope
        // from overlapping LabCartpole's callback.
        mRegistered =
            ovphysx_set_log_callback(OVPHYSX_LOG_WARNING, nullptr, &gpuFallbackLogFn, &gpuFallbackProbe()).status ==
            OVPHYSX_API_SUCCESS;
    }
    ~GpuFallbackScope()
    {
        finish("GPU detector teardown");
    }
    GpuFallbackScope(const GpuFallbackScope&) = delete;
    GpuFallbackScope& operator=(const GpuFallbackScope&) = delete;

    bool finish(const char* phase)
    {
        // Disabling is the only teardown the single-slot API offers. There is
        // no way to read back and restore a previous callback, so the slot is
        // only disabled by the scope that armed it. A failed arm is disabled
        // too. The error may have been reported after the slot was already
        // published, and leaving a live callback behind would have this row's
        // diagnostics attributed to the next one.
        if (!mArmed || mFinished)
        {
            return evidenceOk(phase);
        }
        // OVPHYSX_LOG_DEFAULT is the documented spelling for "put the severity
        // back to the library default", and is what LabCartpole's probe uses
        // when it releases the same single slot.
        const ovphysx_result_t disable = ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
        if (disable.status != OVPHYSX_API_SUCCESS)
        {
            if (!mDisableFailureReported)
            {
                const ovphysx_string_t err = ovphysx_get_last_error();
                bmRecordFailure(mRow, "log-callback disable failed: status=%d %.*s; the slot may still be live",
                                static_cast<int>(disable.status), static_cast<int>(err.length),
                                err.ptr ? err.ptr : "");
                mDisableFailureReported = true;
            }
            // Keep the scope unfinished so a later finish() call can retry.
            // ContactReport's explicit finish is followed by destruction. The
            // failed call may have left a callback in flight, so do not read
            // final evidence until a later disable succeeds and drains it.
            return false;
        }
        mFinished = true;
        // A warning callback may have been in flight when finish() began.
        // Successful disable waits for it, so this post-disable read closes the
        // capture window without racing the final evidence check.
        return evidenceOk(phase);
    }

    bool sawCpuFallback() const
    {
        return gpuFallbackProbe().cpuFallback.load();
    }
    bool sawCapacityWarning() const
    {
        return gpuFallbackProbe().capacityWarning.load();
    }
    std::string evidence() const
    {
        GpuFallbackEvidence& probe = gpuFallbackProbe();
        std::lock_guard<std::mutex> lock(probe.mutex);
        return probe.evidence;
    }
    bool evidenceOk(const char* phase)
    {
        if (!mArmed)
        {
            return true;
        }
        if (!mRegistered)
        {
            if (!mFailureReported)
            {
                bmRecordFailure(mRow,
                                "could not install GPU fallback/capacity detector; requested-device evidence "
                                "is unavailable");
                mFailureReported = true;
            }
            return false;
        }

        const bool fallback = sawCpuFallback();
        const bool capacity = sawCapacityWarning();
        if (!fallback && !capacity)
        {
            return true;
        }
        if (!mFailureReported)
        {
            const char* kind = fallback && capacity ? "CPU-fallback and capacity" :
                               fallback             ? "CPU-fallback" :
                                                      "capacity";
            const std::string why = evidence();
            bmRecordFailure(mRow, "%s produced a known GPU %s warning (%s)", phase, kind,
                            why.empty() ? "diagnostic text unavailable" : why.c_str());
            mFailureReported = true;
        }
        return false;
    }

private:
    const char* mRow = nullptr;
    bool mArmed = false;
    bool mRegistered = false;
    bool mFailureReported = false;
    bool mDisableFailureReported = false;
    bool mFinished = false;
};


// Runs warmup and rejects a GPU-requested row when a known fallback or capacity
// signal is observed. `scope` must have been alive across the preceding attach.
inline bool checkDeviceEvidenceAndWarmup(ovphysx_handle_t handle, bool gpu, GpuFallbackScope& scope, const char* row)
{
    const ovphysx_result_t warm = ovphysx_warmup(handle);
    if (warm.status != OVPHYSX_API_SUCCESS && warm.status != OVPHYSX_API_GPU_NOT_AVAILABLE)
    {
        const ovphysx_string_t err = ovphysx_get_last_error();
        bmRecordFailure(row, "warmup failed: %.*s", static_cast<int>(err.length), err.ptr ? err.ptr : "");
        return false;
    }

    if (gpu && !scope.evidenceOk("attach/warmup"))
    {
        return false;
    }

    if (gpu && warm.status == OVPHYSX_API_GPU_NOT_AVAILABLE)
    {
        bmRecordFailure(row, "GPU request produced fallback evidence (warmup reported GPU unavailable); "
                             "refusing to publish a GPU-labelled row");
        return false;
    }
    // Report only what the evidence supports. The absence of a fallback
    // diagnostic means "no GPU attempt failed". It is not positive proof that a
    // CUDA pipeline was realized, and a _cpu scene never attempts GPU at all, so
    // it is silent by construction.
    if (gpu)
    {
        printFormatted("%s: device = GPU requested; no process-wide CPU-fallback evidence so far, and no "
                       "capacity diagnostic during attach/warmup",
                       row);
    }
    else
    {
        printFormatted("%s: device = CPU by construction (scene authors enableGPUDynamics=false)", row);
    }
    return true;
}

} // namespace authoringbm
