// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-PYTHON-READ-001
// @covers AC-4

// @implements REQ-CAPI-SESSION-001
// @covers AC-1 AC-5 AC-8
//
// @implements REQ-CAPI-READ-001
// @covers AC-1

// @implements REQ-CAPI-DETACH-001
// @covers AC-6

/**
 * @implements REQ-CAPI-VEHICLE-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-CAPI-SLEEP-001
 * @covers AC-1
 *
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10 AC-11 AC-12 AC-13 AC-14 AC-15 AC-16
 *
 * @implements REQ-CAPI-PROFILER-001
 * @covers AC-5
 */

/**
 * @implements REQ-CAPI-PHYSXPTR-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 */

/**
 * @implements REQ-CAPI-LANE-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-CAPI-REENTRY-001
 * @covers AC-2
 */

/**
 * @implements REQ-CAPI-WRITE-001
 * @covers AC-1 AC-1a AC-2 AC-3 AC-4 AC-5 AC-5a AC-6 AC-7 AC-8 AC-10 AC-13 AC-15 AC-16 AC-17
 *
 * @implements REQ-CAPI-INSTANCE-SCOPE-001
 * @covers AC-2 AC-5
 *
 * The public C write surface: ovphysx_write / ovphysx_fetch_write_next / ovphysx_commit_group /
 * ovphysx_release_write. Validation is C-first (AC-8), so every argument, handle and lifecycle
 * check lives here rather than in a frontend. The Python bindings are a thin mirror and inherit
 * these guarantees instead of restating them.
 */

/**
 * @implements REQ-CAPI-DEBUG-RENDER-001
 * @covers AC-8
 *
 * A debug-render query is not a sidecar query. Its shared dictionary is the
 * attached stage's path dictionary, or null when no stage is attached.
 *
 * @implements REQ-CAPI-READ-CONTACT-001
 * @covers AC-1 AC-3 AC-4 AC-5 AC-6
 *
 * @implements REQ-CAPI-READ-CONTACT-002
 * @covers AC-1 AC-2 AC-4 AC-5
 *
 * The contact sensor's public surface: query, selected-query and writability calls admit the type through
 * their OVPHYSX_QUERY_TARGET_COUNT-bounded gates, and kWritabilityTable classifies every contact column READ_ONLY.
 *
 * @implements REQ-CAPI-SELECT-001
 * @covers AC-1 AC-2 AC-3
 *
 * `ovphysx_query_selection`: the selected-query entry point, its prim_paths-wins precedence, and the
 * selection validation it returns on the receipt.
 *
 * @implements REQ-CAPI-SDF-001
 * @covers AC-1 AC-2
 *
 * `ovphysx_evaluate_sdf_shapes`: the SDF view's successor on the session query, with its tensor checks
 * returned on the receipt.
 */

#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_attributes.h"
#include "ovphysx/ovphysx_geometry_query.h"
#include "LogManager.hpp"
#include "internal/Profiling.h"
#include "internal/sdk/ovphysxSDK.hpp"
#include <omni/physx/IOvxPhysicsRead.h>
#include "internal/sdk/TaskContext.hpp"
#include "internal/sdk/DLPackConvert.h"
#include "internal/sidecar/ovphysxInternalInterop.h"

#include <carb/Framework.h>
#include <ovstage/ovstage_api/ovstage_api.h>
#include <carb/logging/Log.h>
#include <omni/physx/IOvxPhysicsRead.h>
#include <omni/physx/PhysXRuntime.h>
#include <omni/physx/IOptionalCuda.h>
#include <omni/physx/RuntimeError.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

// Physics output read (ADR-0007) sidecar pointers, owned here next to their
// public-API consumers below. The loader writes them during loadInternalSidecar().
std::atomic<OvphysxSidecarOutputQueryFn>      g_sidecarOutputQuery{nullptr};
std::atomic<OvphysxSidecarOutputQuerySelectedFn> g_sidecarOutputQuerySelected{nullptr};
std::atomic<OvphysxSidecarOutputQueryContactsFn> g_sidecarOutputQueryContacts{nullptr};
std::atomic<OvphysxSidecarReadOutputsFn>      g_sidecarReadOutputs{nullptr};
std::atomic<OvphysxSidecarFetchReadNextFn>    g_sidecarFetchReadNext{nullptr};
std::atomic<OvphysxSidecarReleaseGroupFn>     g_sidecarReleaseGroup{nullptr};
std::atomic<OvphysxSidecarReleaseReadFn>      g_sidecarReleaseRead{nullptr};
std::atomic<OvphysxSidecarReleaseQueryFn>     g_sidecarReleaseQuery{nullptr};
std::atomic<OvphysxSidecarWriteAttributeFn>   g_sidecarWriteAttribute{nullptr};
std::atomic<OvphysxSidecarFetchWriteNextFn>   g_sidecarFetchWriteNext{nullptr};
std::atomic<OvphysxSidecarCommitGroupFn>      g_sidecarCommitGroup{nullptr};
std::atomic<OvphysxSidecarCommitGroupMaskedFn> g_sidecarCommitGroupMasked{nullptr};
std::atomic<OvphysxSidecarValidateCommitFn> g_sidecarValidateCommit{nullptr};
std::atomic<OvphysxSidecarReleaseWriteFn>     g_sidecarReleaseWrite{nullptr};
// Debug-visualization sidecar fn-ptrs (loader resolves them; see ovphysxSidecarLoader.cpp).
std::atomic<OvphysxSidecarEnableVisualizationFn> g_sidecarEnableVisualization{nullptr};
std::atomic<OvphysxSidecarSetVizParameterFn>     g_sidecarSetVizParameter{nullptr};
std::atomic<OvphysxSidecarSetVizParameterValueFn> g_sidecarSetVizParameterValue{nullptr};
std::atomic<OvphysxSidecarSetVizScopeTokensFn> g_sidecarSetVizScopeTokens{nullptr};
std::atomic<OvphysxSidecarSetVizScaleFn>         g_sidecarSetVizScale{nullptr};
std::atomic<OvphysxSidecarSetVizCullingBoxFn>    g_sidecarSetVizCullingBox{nullptr};
std::atomic<OvphysxSidecarGetDebugBufferFn>      g_sidecarGetDebugPoints{nullptr};
std::atomic<OvphysxSidecarGetDebugBufferFn>      g_sidecarGetDebugLines{nullptr};
std::atomic<OvphysxSidecarGetDebugBufferFn>      g_sidecarGetDebugTriangles{nullptr};
std::atomic<OvphysxSidecarDebugRenderApplyFn>   g_sidecarDebugRenderApply{nullptr};
std::atomic<OvphysxSidecarDebugRenderCopyFn>    g_sidecarDebugRenderCopy{nullptr};

namespace {

// ---- Output handles ----
//
// ovphysx mints the public handle of every query, read and write session itself, from
// one process-wide counter that never hands out a value twice, when a producer call
// (ovphysx_query, ovphysx_query_selection, ovphysx_query_contacts, ovphysx_read,
// ovphysx_write) is accepted: on the calling thread or queued, the call writes the
// handle before it returns, so a later call can name it at once, even while its producer
// is still queued. Each instance keeps a table of its own handles, their kind,
// producer outcome and opened backend resource:
//   pending  the producer has not run yet;
//   live     the producer opened its backend resource;
//   failed   the producer's task failed. A call that names the handle fails its own
//            task with "input <kind> <handle> came from failed task <producer task>";
//   closed   a detach closed the write session (detach retires write sessions, which
//            borrow scene pointers). A call that names it fails its own task.
// A handle is released once its release was accepted: a call submitted after that which
// names it is rejected with NOT_FOUND, as for an unknown handle, and a later release is a
// successful no-op. The release itself runs in submission order: a pending handle is
// released after its producer ran, a failed or closed one has nothing to release. A handle
// of another instance is rejected with NOT_FOUND when the call is submitted, also while its
// producer is pending.
//
// So whether a call may name a handle depends only on the calls submitted before it, never
// on how far the instance worker got: only submissions enter and release handles, and work
// that runs later changes their status, which the calls that name them report through
// their own tasks.
//
// A handle leaves its instance table when its release ran (one whose backend release failed
// stays released, and only teardown retries the cleanup) and when its instance is destroyed.
// The table lock is a leaf: nothing is taken while it is held, and every backend call runs
// outside it.

using ovphysx::OutputKind;
using ovphysx::OutputRecord;
using ovphysx::OutputStatus;

const char* outputKindName(OutputKind kind) noexcept
{
    switch (kind)
    {
    case OutputKind::kQuery:
        return "query";
    case OutputKind::kRead:
        return "read";
    case OutputKind::kWrite:
    default:
        return "write";
    }
}

enum OutputHandleFlags : uint8_t
{
    // Release was accepted; only teardown may retry cleanup.
    kOutputReleaseAccepted = 1u << 0,
    // A write session that handed out a group in device memory. Its commits run on the
    // calling thread after earlier work: such a commit is ordered on the caller's CUDA stream.
    kOutputDeviceGroups = 1u << 1,
};

// What a call finds for a handle of one kind it names on its instance.
struct OutputLookup
{
    enum class Owner : uint8_t
    {
        kNone, // unknown, or a handle of another kind
        kOther, // a handle of another instance
        kThis,
    };
    Owner owner = Owner::kNone;
    OutputStatus status = OutputStatus::kPending;
    uint8_t flags = 0;
    uint64_t backend = 0;
    uint64_t producer = 0;

    // A call submitted now may name the handle: it belongs to the call's instance and its
    // release was not accepted.
    bool admitted() const noexcept { return owner == Owner::kThis && !(flags & kOutputReleaseAccepted); }
};

// The exceptional foreign-handle path scans peer tables under registry-then-output
// lock order. Normal calls touch only their own instance table.
bool outputExistsOnPeer(const InstanceData& instance, uint64_t handle, OutputKind kind)
{
    std::shared_lock<std::shared_mutex> instancesLock(g_instances_mutex);
    for (const auto& entry : g_instances)
    {
        const InstanceData* peer = entry.second.get();
        if (!peer || peer == &instance)
            continue;
        std::shared_lock<std::shared_mutex> outputLock(peer->output_handles.mutex);
        const auto record = peer->output_handles.records.find(handle);
        if (record != peer->output_handles.records.end() && record->second.kind == kind)
            return true;
    }
    return false;
}

OutputLookup lookupOutput(const InstanceData& instance, uint64_t handle, OutputKind kind)
{
    OutputLookup found;
    {
        std::shared_lock<std::shared_mutex> lock(instance.output_handles.mutex);
        const auto record = instance.output_handles.records.find(handle);
        if (record != instance.output_handles.records.end() && record->second.kind == kind)
        {
            found.owner = OutputLookup::Owner::kThis;
            found.status = record->second.status;
            found.flags = record->second.flags;
            found.backend = record->second.backend;
            found.producer = record->second.producer;
            return found;
        }
    }
    if (outputExistsOnPeer(instance, handle, kind))
        found.owner = OutputLookup::Owner::kOther;
    return found;
}

// Mints a handle of kind for instance and enters it as pending. Returns zero, entering
// nothing, once every handle was used; throws std::bad_alloc, entering nothing.
uint64_t enterPendingOutput(InstanceData& instance, OutputKind kind)
{
    const uint64_t handle = ovphysx::internal::allocateOpaqueObjectHandle();
    if (!handle)
        return 0;
    OutputRecord record;
    record.kind = kind;
    std::unique_lock<std::shared_mutex> lock(instance.output_handles.mutex);
    instance.output_handles.records.emplace(handle, record);
    return handle;
}

// The producer of handle opened backend: the handle is live. Its flags stay.
void settleOutput(InstanceData& instance, uint64_t handle, uint64_t backend) noexcept
{
    std::unique_lock<std::shared_mutex> lock(instance.output_handles.mutex);
    const auto record = instance.output_handles.records.find(handle);
    if (record == instance.output_handles.records.end())
        return;
    record->second.status = OutputStatus::kLive;
    record->second.backend = backend;
}

// The producer of handle failed in task producer.
void failOutput(InstanceData& instance, uint64_t handle, uint64_t producer) noexcept
{
    std::unique_lock<std::shared_mutex> lock(instance.output_handles.mutex);
    const auto record = instance.output_handles.records.find(handle);
    if (record == instance.output_handles.records.end())
        return;
    record->second.status = OutputStatus::kFailed;
    record->second.producer = producer;
}

// Removes a handle from the table: one never handed out, or one whose release ran.
void eraseOutput(InstanceData& instance, uint64_t handle) noexcept
{
    std::unique_lock<std::shared_mutex> lock(instance.output_handles.mutex);
    instance.output_handles.records.erase(handle);
}

// Adds flags to a handle. Called by the lane holder.
void markOutput(InstanceData& instance, uint64_t handle, uint8_t flags) noexcept
{
    std::unique_lock<std::shared_mutex> lock(instance.output_handles.mutex);
    const auto record = instance.output_handles.records.find(handle);
    if (record != instance.output_handles.records.end())
        record->second.flags = static_cast<uint8_t>(record->second.flags | flags);
}

// The cause a call reports when a handle it names is not live: its producer failed, a
// detach closed it, or (defensively) its producer never ran.
std::string unusableInputCause(OutputKind kind, uint64_t handle, const OutputLookup& input)
{
    std::string cause = std::string("input ") + outputKindName(kind) + " " + std::to_string(handle);
    if (input.status == OutputStatus::kFailed)
        return cause + " came from failed task " + std::to_string(input.producer);
    if (input.status == OutputStatus::kClosed)
        return cause + " was closed when its stage was detached";
    return cause + " was never opened";
}

// A task failure as a queued job reports it: "<api> (task <index>): <cause>".
std::string taskFailureText(const char* api, uint64_t task, const std::string& cause)
{
    return std::string(api) + " (task " + std::to_string(task) + "): " + cause;
}

// Refuses an output call made inside a callback (log, runtime or object-change) or on
// the instance's own worker thread, before any lane or wait. Such a callback may run
// under a held lane, or while process-shared backend mutexes are held. Returns
// SUCCESS or INVALID_STATE with the last error set.
ovphysx_api_status_t refuseOutputReentry(const InstanceData& instance, const ovphysx::Caller& caller, const char* what)
{
    const char* where = caller.callback;
    if (!where && instance.executor && instance.executor->isWorkerIdentity(caller.identity))
        where = "on the instance's own worker thread";
    if (!where)
        return OVPHYSX_API_SUCCESS;
    char message[160];
    std::snprintf(message, sizeof(message), "%s cannot reenter runtime execution (%s)", what, where);
    return set_error(OVPHYSX_API_INVALID_STATE, message).status;
}

// A stopped instance worker (after ovphysx_shutdown(), or under a destroy the call raced)
// rejects an output call wherever it would run, as it rejects the submission of a job.
ovphysx_enqueue_result_t stoppedWorker(const char* api)
{
    return set_enqueue_error(OVPHYSX_API_INVALID_STATE,
                             std::string(api) + ": the instance worker is stopped; destroy the instance");
}

// Whether runtime work of the instance's own attachment is executing, which an output call
// under the lane must not reenter. An instance with no attachment has none: the runtime's
// admission for calls on no attachment is process-wide, held by any instance's attach or
// detach, and probing it would refuse this call, or fail that attach, for nothing.
bool attachmentBusy(const InstanceData& instance) noexcept
{
    return instance.attachHandle != omni::physics::tensors::kNoAttach &&
           omni::physx::runtime::isRuntimeBusy(instance.attachHandle, true);
}

// Takes the instance lane for the rest of an output call once its argument checks
// passed: earlier work on the instance, queued and running jobs included, finishes
// first. Peer execution never rejects output. Same-owner runtime reentry still does;
// it is probed under the lane, so a job of this instance makes the call wait instead
// of failing busy. Returns SUCCESS, or a rejection status with the last error set.
ovphysx_api_status_t enterOutputLane(InstanceData& instance, ovphysx::InstanceLane& lane,
                                     const ovphysx::Caller& caller, const char* api)
{
    // outputTask resolved the instance and the caller and refused callback reentry already.
    const ovphysx_api_status_t status = lane.acquire(instance, caller);
    if (status == OVPHYSX_API_INVALID_STATE)
        return status;
    if (status != OVPHYSX_API_SUCCESS)
        return stoppedWorker(api).status;
    if (attachmentBusy(instance))
        return set_error(OVPHYSX_API_INVALID_STATE, "Output operation cannot reenter runtime execution").status;
    return OVPHYSX_API_SUCCESS;
}

ovphysx_enqueue_result_t outputRejection(ovphysx_api_status_t status)
{
    return {status, 0, 0};
}

// Runs work(instance, admission, caller, owner) for an output call of a live instance
// whose caller runs no callback; owner keeps the instance alive for a job.
template <typename Work>
ovphysx_enqueue_result_t outputTask(ovphysx_instance_handle_t handle, Work&& work)
{
    // Declared before the receipt, which borrows it: the instance stays alive while a
    // failure is recorded in the handlers below.
    std::shared_ptr<InstanceData> instance;
    ovphysx::TaskContext admission;
    try
    {
        instance = get_instance(handle);
        if (!instance)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid instance handle");
        // Resolved once for the reentry check and the lane the work takes.
        const ovphysx::Caller caller = ovphysx::currentCaller();
        if (refuseOutputReentry(*instance, caller, "Output operation") != OVPHYSX_API_SUCCESS)
            return outputRejection(OVPHYSX_API_INVALID_STATE);
        return work(*instance, admission, caller, instance);
    }
    catch (const std::exception& error)
    {
        return admission.fail(error.what());
    }
    catch (...)
    {
        return admission.fail("Output operation failed");
    }
}

// Where a short async call runs, chosen by enterShortCall().
enum class ShortCallVenue
{
    // On the calling thread under the instance lane.
    kInline,
    // As a job behind the instance's queued, running or held work.
    kQueued,
    // Refused; the last error is set.
    kRejected,
};

// Chooses where a short async call runs once its argument checks passed. On an idle
// instance (lane free, nothing queued, running or held) it takes the lane with one
// compare-and-swap and runs on the calling thread, exactly as a synchronous call.
// Behind any such work it is queued instead, so the call does not wait for that work
// (an instance without a worker thread runs the queued call before returning) and
// every call still runs in submission order. The private test switch
// (InstanceData::test_execution) queues every call ("queue").
ShortCallVenue enterShortCall(InstanceData& instance, ovphysx::InstanceLane& lane, const ovphysx::Caller& caller,
                              const char* api, ovphysx_api_status_t& rejection)
{
    // outputTask resolved the instance and the caller and refused callback reentry already.
    if (instance.test_execution == ovphysx::TestExecution::kQueue)
        return ShortCallVenue::kQueued;
    if (!lane.tryAcquireIdle(instance, caller))
        return ShortCallVenue::kQueued;
    // Shutdown stops the worker; destroy stops it while it holds the lane and unregisters the
    // instance before releasing it, so a caller that took the lane after it finds it stopped.
    if (instance.executor->stopped())
    {
        lane.release();
        rejection = stoppedWorker(api).status;
        return ShortCallVenue::kRejected;
    }
    return ShortCallVenue::kInline;
}

// How the part of an output call that runs under its instance's lane reports: the start
// of its work and its outcome, and the handle it produces. InlineOutput runs it on the
// calling thread before the call returns; QueuedOutput runs it in the call's job on the
// instance worker, whose receipt the call returned at submission.
//
// One error path for both: a call is rejected, with no task, only for what its submission
// alone decides (its arguments, and whether the handles it names belong to its instance
// and are not released, which submissions alone change; see "Output handles"). Every check
// of state that earlier work sets (the attachment, backend availability, an input whose
// producer failed or that a detach closed) fails the call's task through failTask(), with
// the text a queued job reports, "<api> (task <index>): <cause>", on both paths, so the
// outcome never depends on whether the call ran at once or queued. refuse() rejects what
// the submission decides (handle ownership, and for a commit that always runs on the
// calling thread what the runtime refuses as given): on the calling thread it rejects the
// call; in a job, whose submission checked the handles already, it fails the task.
//
// A producer calls startProducing() before any such check, where a consumer calls
// start(), then produced() with its backend resource; a failure before that leaves its
// handle failed. attachmentCurrent() tells whether the attachment the call was submitted
// for exists (see "Attachment generations" at InstanceData).
class InlineOutput
{
public:
    InlineOutput(InstanceData& instance, ovphysx_instance_handle_t handle, ovphysx::TaskContext& admission,
                 const char* api) noexcept
        : m_instance(instance), m_handle(handle), m_admission(admission), m_api(api)
    {
    }
    // An exception that escaped after the handle was entered fails it with the call.
    ~InlineOutput() { settleFailed(); }
    InlineOutput(const InlineOutput&) = delete;
    InlineOutput& operator=(const InlineOutput&) = delete;

    // On the calling thread a call may name a handle of its instance whose release was
    // not accepted, as when it is submitted.
    bool admits(const OutputLookup& found) const noexcept { return found.admitted(); }
    ovphysx_enqueue_result_t refuse(ovphysx_api_status_t status, const char* message) noexcept
    {
        return set_enqueue_error(status, message);
    }
    ovphysx_enqueue_result_t refuse(ovphysx_api_status_t status, const std::string& message) noexcept
    {
        return set_enqueue_error(status, message);
    }
    void start() { m_admission.create_task(m_instance, m_handle); }
    InstanceData& instance() noexcept { return m_instance; }
    // Mints the handle of kind this producer returns, enters it as pending, creates the
    // receipt and writes the handle to *out. False once every handle was used: the call is
    // rejected with ERROR and the last error set, with no receipt and *out still zero.
    bool startProducing(OutputKind kind, uint64_t* out)
    {
        const uint64_t produced = enterPendingOutput(m_instance, kind);
        if (!produced)
        {
            set_error(OVPHYSX_API_ERROR, std::string(m_api) + ": output handles are exhausted");
            return false;
        }
        try
        {
            start();
        }
        catch (...)
        {
            eraseOutput(m_instance, produced);
            throw;
        }
        m_produced = produced;
        *out = produced;
        return true;
    }
    void produced(uint64_t backend) noexcept
    {
        settleOutput(m_instance, m_produced, backend);
        m_settled = true;
    }
    // Fails the call's task, creating its receipt first if the work had not started, with
    // the text a queued job reports.
    ovphysx_enqueue_result_t fail(const char* cause) noexcept
    {
        try
        {
            if (!m_admission.receipt.op_index)
                start();
            settleFailed();
            return m_admission.fail(taskFailureText(m_api, m_admission.receipt.op_index, cause).c_str());
        }
        catch (...)
        {
            settleFailed();
            return m_admission.fail(cause);
        }
    }
    ovphysx_enqueue_result_t complete() noexcept { return m_admission.complete(); }
    ovphysx_enqueue_result_t failTask(const std::string& cause) noexcept { return fail(cause.c_str()); }
    // The instance is idle, so the call executes as it is submitted.
    bool attachmentCurrent() noexcept
    {
        const ovphysx::SubmittedAttachment submitted = ovphysx::submittedAttachment(m_instance);
        m_lifecycleTask = submitted.lifecycleTask;
        return ovphysx::attachmentCurrent(m_instance, submitted);
    }
    uint64_t lifecycleTask() const noexcept { return m_lifecycleTask; }

private:
    void settleFailed() noexcept
    {
        if (m_produced && !m_settled)
        {
            failOutput(m_instance, m_produced, m_admission.receipt.op_index);
            m_settled = true;
        }
    }

    InstanceData& m_instance;
    ovphysx_instance_handle_t m_handle;
    ovphysx::TaskContext& m_admission;
    const char* m_api;
    uint64_t m_produced = 0;
    bool m_settled = false;
    uint64_t m_lifecycleTask = 0;
};

// What a queued short call carries from its submission into its job.
struct QueuedAdmission
{
    // The handle a producer returned at submission.
    uint64_t produced = 0;
    // The attachment generation the call was submitted for, and the attach or detach it
    // came from (see "Attachment generations" at InstanceData).
    uint64_t generation = 0;
    uint64_t lifecycleTask = 0;
    // The release of the handle the call names was accepted before the call was
    // submitted (a release_group of a released read does nothing).
    bool inputReleased = false;
};

// The admission of a queued short call that names no released handle.
QueuedAdmission queuedAdmission(InstanceData& instance) noexcept
{
    const ovphysx::SubmittedAttachment submitted = ovphysx::submittedAttachment(instance);
    QueuedAdmission admission;
    admission.generation = submitted.generation;
    admission.lifecycleTask = submitted.lifecycleTask;
    return admission;
}

class QueuedOutput
{
public:
    QueuedOutput(ovphysx::InstanceJob& job, const QueuedAdmission& admission) noexcept
        : m_job(job), m_admission(admission)
    {
    }
    // An exception that escaped after the job started fails its handle with the job.
    ~QueuedOutput() { settleFailed(); }
    QueuedOutput(const QueuedOutput&) = delete;
    QueuedOutput& operator=(const QueuedOutput&) = delete;

    // In its job a call may name the handles it was admitted with: a release accepted
    // after its submission is queued behind it and has not run yet.
    bool admits(const OutputLookup& found) const noexcept
    {
        return found.owner == OutputLookup::Owner::kThis && !m_admission.inputReleased;
    }
    ovphysx_enqueue_result_t refuse(ovphysx_api_status_t, const char* message) noexcept { return fail(message); }
    ovphysx_enqueue_result_t refuse(ovphysx_api_status_t, const std::string& message) noexcept
    {
        return fail(message.c_str());
    }
    void start() noexcept {}
    InstanceData& instance() noexcept { return m_job.instance(); }
    // The handle was minted when the call was submitted.
    bool startProducing(OutputKind, uint64_t*) noexcept { return true; }
    void produced(uint64_t backend) noexcept
    {
        settleOutput(m_job.instance(), m_admission.produced, backend);
        m_settled = true;
    }
    ovphysx_enqueue_result_t fail(const char* message) noexcept
    {
        settleFailed();
        m_job.fail(message);
        return {};
    }
    ovphysx_enqueue_result_t complete() noexcept { return {}; }
    // The job adds the "<api> (task <index>): " prefix.
    ovphysx_enqueue_result_t failTask(const std::string& cause) noexcept { return fail(cause.c_str()); }
    bool attachmentCurrent() const noexcept
    {
        return ovphysx::attachmentCurrent(m_job.instance(), {m_admission.generation, m_admission.lifecycleTask});
    }
    uint64_t lifecycleTask() const noexcept { return m_admission.lifecycleTask; }

private:
    void settleFailed() noexcept
    {
        if (m_admission.produced && !m_settled)
        {
            failOutput(m_job.instance(), m_admission.produced, m_job.taskIndex);
            m_settled = true;
        }
    }

    ovphysx::InstanceJob& m_job;
    const QueuedAdmission& m_admission;
    bool m_settled = false;
};

// A short async call queued behind earlier work on its instance. Payload is built in
// place at submission from the call's arguments (copies of their small inputs and the
// caller's output slots for results its job writes) and run(output, instance, handle)
// does the call's work on the lane holder.
template <typename Payload>
class OutputJob final : public ovphysx::InstanceJob
{
public:
    template <typename... Args>
    OutputJob(const char* api, ovphysx_instance_handle_t handle, std::shared_ptr<InstanceData> instance,
              const QueuedAdmission& admission, Args&&... args)
        : InstanceJob(api, handle, std::move(instance)), m_admission(admission), m_payload(std::forward<Args>(args)...)
    {
    }

private:
    void execute() override
    {
        QueuedOutput output(*this, m_admission);
        m_payload.run(output, instance(), handle());
    }

    QueuedAdmission m_admission;
    Payload m_payload;
};

template <typename Payload, typename... Args>
ovphysx_enqueue_result_t queueOutputJob(const std::shared_ptr<InstanceData>& owner, ovphysx_instance_handle_t handle,
                                        const char* api, const QueuedAdmission& admission, Args&&... args)
{
    // Every input the job needs is copied here, before the receipt exists.
    return ovphysx::submitInstanceJob(
        *owner, handle,
        std::unique_ptr<ovphysx::InstanceJob>(
            new OutputJob<Payload>(api, handle, owner, admission, std::forward<Args>(args)...)));
}

// Queues a producer's job like queueOutputJob() and writes the handle it produces, of
// kind, to *out before returning: the handle is minted and entered as pending first, and
// a rejected call removes it again, so it leaves nothing behind. Once every handle was
// used the call is rejected with ERROR.
template <typename Payload, typename... Args>
ovphysx_enqueue_result_t queueProducer(const std::shared_ptr<InstanceData>& owner, ovphysx_instance_handle_t handle,
                                       const char* api, OutputKind kind, uint64_t* out, Args&&... args)
{
    QueuedAdmission admission = queuedAdmission(*owner);
    admission.produced = enterPendingOutput(*owner, kind);
    if (!admission.produced)
        return set_enqueue_error(OVPHYSX_API_ERROR, std::string(api) + ": output handles are exhausted");
    struct Unpublished
    {
        InstanceData& instance;
        uint64_t produced;
        ~Unpublished()
        {
            if (produced)
                eraseOutput(instance, produced);
        }
    } unpublished{*owner, admission.produced};
    const ovphysx_enqueue_result_t receipt =
        queueOutputJob<Payload>(owner, handle, api, admission, std::forward<Args>(args)...);
    if (receipt.status != OVPHYSX_API_SUCCESS)
        return receipt;
    unpublished.produced = 0;
    *out = admission.produced;
    return receipt;
}

// Runs a short async call: runInline(InlineOutput&) on the calling thread when its
// instance is idle, otherwise queue(), which makes the call's submission-time checks and
// queues its job (see enterShortCall()).
template <typename Inline, typename Queue>
ovphysx_enqueue_result_t shortCall(InstanceData& instance, ovphysx_instance_handle_t handle,
                                   ovphysx::TaskContext& admission, const ovphysx::Caller& caller, const char* api,
                                   Inline&& runInline, Queue&& queue)
{
    ovphysx::InstanceLane lane;
    ovphysx_api_status_t rejection = OVPHYSX_API_SUCCESS;
    switch (enterShortCall(instance, lane, caller, api, rejection))
    {
    case ShortCallVenue::kInline:
    {
        InlineOutput output(instance, handle, admission, api);
        return runInline(output);
    }
    case ShortCallVenue::kRejected:
        return outputRejection(rejection);
    case ShortCallVenue::kQueued:
        break;
    }
    return queue();
}

// The arguments of a queued read or write: its attribute array and every string it
// names an attribute by, copied at submission so the caller may free or overwrite them
// once the call returns. A non-NULL string names its attribute and the token beside it
// is never read; a NULL string leaves the token naming it. Built in place inside its
// job, which never moves.
class AttributeCopies
{
public:
    AttributeCopies(const ovx_token_or_string_t* attributes, size_t count)
        : m_items(attributes, attributes + count)
    {
        size_t bytes = 0;
        for (const ovx_token_or_string_t& item : m_items)
            if (item.string.ptr)
                bytes += item.string.length;
        m_text.resize(bytes);
        char* cursor = m_text.data();
        for (ovx_token_or_string_t& item : m_items)
        {
            if (!item.string.ptr)
                continue;
            if (!item.string.length)
            {
                // A present but empty string stays present.
                item.string.ptr = "";
                continue;
            }
            std::memcpy(cursor, item.string.ptr, item.string.length);
            item.string.ptr = cursor;
            cursor += item.string.length;
        }
    }
    AttributeCopies(const AttributeCopies&) = delete;
    AttributeCopies& operator=(const AttributeCopies&) = delete;

    const ovx_token_or_string_t* data() const noexcept { return m_items.data(); }
    size_t size() const noexcept { return m_items.size(); }

private:
    std::vector<ovx_token_or_string_t> m_items;
    std::vector<char> m_text;
};

// Query selections copied at submission for a queued query: what each one uses, its
// prim paths or else its pattern. Sizes first, then one copy into storage that never
// moves (the job that owns it is built in place).
class SelectionCopies
{
public:
    void measure(const ovphysx_query_selection_t& selection) noexcept
    {
        if (selection.prim_paths && selection.prim_paths_count > 0)
            m_pathCount += selection.prim_paths_count;
        else if (selection.pattern.ptr && selection.pattern.length > 0)
            m_textBytes += selection.pattern.length;
    }
    // After every measure(): allocates the storage for the copies.
    void allocate()
    {
        m_paths.resize(m_pathCount);
        m_text.resize(m_textBytes);
        m_nextPath = m_paths.data();
        m_nextText = m_text.data();
    }
    // Copies a measured selection and returns it, pointing at the copies.
    ovphysx_query_selection_t copy(const ovphysx_query_selection_t& selection) noexcept
    {
        ovphysx_query_selection_t copied{};
        if (selection.prim_paths && selection.prim_paths_count > 0)
        {
            std::copy(selection.prim_paths, selection.prim_paths + selection.prim_paths_count, m_nextPath);
            copied.prim_paths = m_nextPath;
            copied.prim_paths_count = selection.prim_paths_count;
            m_nextPath += selection.prim_paths_count;
        }
        else if (selection.pattern.ptr && selection.pattern.length > 0)
        {
            std::memcpy(m_nextText, selection.pattern.ptr, selection.pattern.length);
            copied.pattern = {m_nextText, selection.pattern.length};
            m_nextText += selection.pattern.length;
        }
        return copied;
    }

private:
    std::vector<ovx_primpath_t> m_paths;
    std::vector<char> m_text;
    size_t m_pathCount = 0;
    size_t m_textBytes = 0;
    ovx_primpath_t* m_nextPath = nullptr;
    char* m_nextText = nullptr;
};

template <typename Output>
ovphysx_enqueue_result_t outputFailure(Output& output, const omni::physx::RuntimeErrorScope& error,
                                      const char* fallback)
{
    return output.fail(error.message()[0] ? error.message() : fallback);
}

// Releases the backend resource of handle once its release was accepted, and removes the
// handle unless the backend release failed (then only teardown retries the cleanup).
template <typename Output>
ovphysx_enqueue_result_t releaseBackend(Output& output, uint64_t handle, uint64_t backend, int (*release)(uint64_t))
{
    omni::physx::RuntimeErrorScope error;
    const int released = release(backend);
    if (released >= 0)
        eraseOutput(output.instance(), handle);
    if (released <= 0 || error.message()[0])
        return outputFailure(output, error, "Output release failed; cleanup remains owned by the SDK");
    return output.complete();
}

// How a release finds the handle it names when it is accepted.
enum class ReleaseAcceptance : uint8_t
{
    kNothing, // unknown, or released before: a successful no-op
    kForeign, // a handle of another instance: NOT_FOUND
    kUnavailable, // the backend release entry point is missing: ERROR
    kAccepted,
};

// Accepts the release of handle (of kind, for instance) under the exclusive table lock:
// marks it released. backend, for a release that runs at once on the calling thread,
// receives the handle's backend resource, or zero when its producer failed (nothing to
// release); a queued release takes it in its job instead.
ReleaseAcceptance acceptRelease(InstanceData& instance, uint64_t handle, OutputKind kind, bool canRelease,
                                uint64_t* backend)
{
    {
        std::unique_lock<std::shared_mutex> lock(instance.output_handles.mutex);
        const auto found = instance.output_handles.records.find(handle);
        if (found != instance.output_handles.records.end() && found->second.kind == kind)
        {
            OutputRecord& record = found->second;
            if (record.flags & kOutputReleaseAccepted)
                return ReleaseAcceptance::kNothing;
            if (!canRelease)
                return ReleaseAcceptance::kUnavailable;
            record.flags = static_cast<uint8_t>(record.flags | kOutputReleaseAccepted);
            if (backend && record.status == OutputStatus::kLive)
                *backend = record.backend;
            return ReleaseAcceptance::kAccepted;
        }
    }
    return outputExistsOnPeer(instance, handle, kind) ? ReleaseAcceptance::kForeign : ReleaseAcceptance::kNothing;
}

// Undoes acceptRelease() for a queued release that was then rejected.
void withdrawRelease(InstanceData& instance, uint64_t handle) noexcept
{
    std::unique_lock<std::shared_mutex> lock(instance.output_handles.mutex);
    const auto found = instance.output_handles.records.find(handle);
    if (found != instance.output_handles.records.end())
        found->second.flags = static_cast<uint8_t>(found->second.flags & ~kOutputReleaseAccepted);
}

// For the job of a release accepted at its submission: the handle's backend resource,
// or zero when there is nothing to release (a failed producer, or a handle a detach
// retired since), in which case the handle is removed.
uint64_t takeReleasedBackend(InstanceData& instance, uint64_t handle) noexcept
{
    std::unique_lock<std::shared_mutex> lock(instance.output_handles.mutex);
    const auto found = instance.output_handles.records.find(handle);
    if (found == instance.output_handles.records.end())
        return 0;
    if (found->second.status == OutputStatus::kLive)
        return found->second.backend;
    instance.output_handles.records.erase(found);
    return 0;
}

// The release of an output handle on the calling thread: accepted and run at once.
ovphysx_enqueue_result_t releaseNow(InlineOutput& output, uint64_t resource, OutputKind kind,
                                    int (*release)(uint64_t))
{
    uint64_t backend = 0;
    const ReleaseAcceptance acceptance =
        acceptRelease(output.instance(), resource, kind, release != nullptr, &backend);
    switch (acceptance)
    {
    case ReleaseAcceptance::kForeign:
        return output.refuse(OVPHYSX_API_NOT_FOUND, "Resource belongs to another instance");
    case ReleaseAcceptance::kUnavailable:
        return output.refuse(OVPHYSX_API_ERROR, "Output release API is unavailable");
    case ReleaseAcceptance::kNothing:
    case ReleaseAcceptance::kAccepted:
        break;
    }
    const bool accepted = acceptance == ReleaseAcceptance::kAccepted;
    // The receipt comes before anything that cannot be undone: a call that fails to create
    // it leaves the handle as it was.
    try
    {
        output.start();
    }
    catch (...)
    {
        if (accepted)
            withdrawRelease(output.instance(), resource);
        throw;
    }
    if (!backend)
    {
        // A failed producer opened nothing to release.
        if (accepted)
            eraseOutput(output.instance(), resource);
        return output.complete();
    }
    return releaseBackend(output, resource, backend, release);
}

// A release queued behind earlier work. accepted says whether its submission accepted the
// release (and marked the handle released); otherwise the call is a no-op.
struct ReleaseJob
{
    ReleaseJob(uint64_t resource, int (*release)(uint64_t), bool accepted) noexcept
        : resource(resource), release(release), accepted(accepted)
    {
    }
    void run(QueuedOutput& output, InstanceData& instance, ovphysx_instance_handle_t) const
    {
        if (!accepted)
            return;
        const uint64_t backend = takeReleasedBackend(instance, resource);
        if (!backend)
            return;
        if (!release)
        {
            output.fail("Output release API is unavailable");
            return;
        }
        releaseBackend(output, resource, backend, release);
    }
    uint64_t resource;
    int (*release)(uint64_t);
    bool accepted;
};

ovphysx_enqueue_result_t releaseOutput(ovphysx_instance_handle_t handle, uint64_t resource, OutputKind kind,
                                      int (*release)(uint64_t), const char* api)
{
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission, const ovphysx::Caller& caller,
                                  const std::shared_ptr<InstanceData>& owner)
    {
        // Releases join the lane in submission order.
        return shortCall(
            instance, handle, admission, caller, api,
            [&](InlineOutput& output) { return releaseNow(output, resource, kind, release); },
            [&]() {
                // Accepted now, so a call submitted after this one that names the handle is
                // rejected; the release itself runs in order, after the handle's producer.
                switch (acceptRelease(instance, resource, kind, /*canRelease=*/true, nullptr))
                {
                case ReleaseAcceptance::kForeign:
                    return set_enqueue_error(OVPHYSX_API_NOT_FOUND, "Resource belongs to another instance");
                case ReleaseAcceptance::kNothing:
                case ReleaseAcceptance::kUnavailable:
                    return queueOutputJob<ReleaseJob>(owner, handle, api, QueuedAdmission{}, resource, release, false);
                case ReleaseAcceptance::kAccepted:
                    break;
                }
                struct Withdraw
                {
                    InstanceData& instance;
                    uint64_t accepted;
                    ~Withdraw()
                    {
                        if (accepted)
                            withdrawRelease(instance, accepted);
                    }
                } withdraw{instance, resource};
                const ovphysx_enqueue_result_t receipt =
                    queueOutputJob<ReleaseJob>(owner, handle, api, QueuedAdmission{}, resource, release, true);
                if (receipt.status == OVPHYSX_API_SUCCESS)
                    withdraw.accepted = 0;
                return receipt;
            });
    });
}

} // anonymous namespace

int releaseDebugOrQuery(uint64_t handle)
{
    const OvphysxSidecarReleaseQueryFn release = g_sidecarReleaseQuery.load(std::memory_order_acquire);
    return release ? release(handle) : -1;
}

int releaseDebugOrRead(uint64_t handle)
{
    const OvphysxSidecarReleaseReadFn release = g_sidecarReleaseRead.load(std::memory_order_acquire);
    return release ? release(handle) : -1;
}

int releaseDebugOrWrite(uint64_t handle)
{
    const OvphysxSidecarReleaseWriteFn release = g_sidecarReleaseWrite.load(std::memory_order_acquire);
    return release ? release(handle) : -1;
}

namespace {
// Releases the backend resources of instance's handles outside the table lock: write
// sessions first, then reads, then queries. With writesOnly (a detach) only the write
// sessions opened so far: each stays in the table, closed, until its release runs, also one
// whose release was accepted already, so calls submitted before that release still find it
// and fail their task. Otherwise (instance destruction) every handle of the instance leaves
// the table. Retire a bounded stack batch at a time so cleanup allocates no storage after
// teardown commits.
void retireOutputHandles(InstanceData& instance, bool writesOnly) noexcept
{
    constexpr size_t kBackendBatchSize = 32;
    for (const OutputKind kind : {OutputKind::kWrite, OutputKind::kRead, OutputKind::kQuery})
    {
        for (;;)
        {
            std::array<uint64_t, kBackendBatchSize> backends{};
            size_t backendCount = 0;
            try
            {
                std::unique_lock<std::shared_mutex> lock(instance.output_handles.mutex);
                for (auto record = instance.output_handles.records.begin();
                     record != instance.output_handles.records.end();)
                {
                    OutputRecord& output = record->second;
                    if (output.kind != kind)
                    {
                        ++record;
                        continue;
                    }
                    // A write queued behind a detach opens after it, against what that detach left.
                    if (writesOnly && output.status != OutputStatus::kLive)
                    {
                        ++record;
                        continue;
                    }
                    if (output.status == OutputStatus::kLive)
                        backends[backendCount++] = output.backend;
                    if (writesOnly)
                    {
                        output.status = OutputStatus::kClosed;
                        output.backend = 0;
                        ++record;
                    }
                    else
                    {
                        record = instance.output_handles.records.erase(record);
                    }
                    if (backendCount == backends.size())
                        break;
                }
            }
            catch (...)
            {
                CARB_LOG_ERROR("Output session cleanup failed with an exception");
                return;
            }
            int (*release)(uint64_t) = kind == OutputKind::kWrite ? releaseDebugOrWrite :
                                       kind == OutputKind::kRead  ? releaseDebugOrRead :
                                                                    releaseDebugOrQuery;
            for (size_t index = 0; index < backendCount; ++index)
            {
                try
                {
                    omni::physx::RuntimeErrorScope error;
                    if (release(backends[index]) <= 0 || error.message()[0])
                        CARB_LOG_ERROR(
                            "Output session cleanup failed: %s",
                            error.message()[0] ? error.message() : "backend release unavailable or incomplete");
                }
                catch (...)
                {
                    CARB_LOG_ERROR("Output session cleanup failed with an exception");
                }
            }
            if (backendCount < backends.size())
                break;
        }
        if (writesOnly)
            break;
    }
}
} // namespace

void cleanup_output_writes(InstanceData& instance) noexcept
{
    // Write sessions borrow scene pointers; retire them before the scene detaches.
    retireOutputHandles(instance, /*writesOnly=*/true);
}

void cleanup_output_sessions(ovphysx_instance_handle_t handle, InstanceData& instance) noexcept
{
    // Drop visualization on this instance's scenes before its sessions go away.
    omni::physx::ovxDebugRenderForget(handle);
    // Release before unloading the sidecar. Read buffers retain their CUDA
    // contexts across stage detach.
    retireOutputHandles(instance, /*writesOnly=*/false);
}

// ---- Physics output read (ADR-0007) ----
//
// The public surface validates the handle. Simulated-type queries require an
// attached ovstage Stage and then forward to the sidecar. OVPHYSX_QUERY_TARGET_DEBUG_RENDER
// is an instance target and does not require a stage.
// See docs/ovstage_integration.md for the ordinal-coupling principle.
//
// The calls below are short async calls: on an idle instance each runs on the
// calling thread and returns a finished task; behind queued, running or held work it is
// queued as a job with copies of its small inputs (shortCall()), and its outputs are
// written when it runs. update_articulations_kinematic and evaluate_sdf_shapes, masked
// commits, commits with a CUDA handoff, and the commits of a write session that handed out
// device memory (see kOutputDeviceGroups) always run on the calling thread, after earlier
// work.

namespace
{

// The query a producer opens is written to outQuery when the call returns: by
// startProducing() on the calling thread, by the submission for a queued call (whose job
// passes no slot).
template <typename Output>
ovphysx_enqueue_result_t openDebugRenderQuery(Output& output, ovphysx_instance_handle_t handle,
                                              ovphysx_object_scope_t scope, ovphysx_query_handle_t* outQuery)
{
    if (!output.startProducing(OutputKind::kQuery, outQuery))
        return outputRejection(OVPHYSX_API_ERROR);
    if (!output.attachmentCurrent())
        return output.failTask(ovphysx::attachmentChangedCause(output.lifecycleTask()));
    const uint64_t resource = omni::physx::ovxOpenDebugRenderQuery(handle, static_cast<uint32_t>(scope));
    if (!resource)
        return output.fail("ovphysx_query: failed to open a debug-render query");
    output.produced(resource);
    return output.complete();
}

template <typename Output>
ovphysx_enqueue_result_t openQuery(Output& output, InstanceData& instance, ovphysx_instance_handle_t handle,
                                   ovphysx_query_target_t objectType, ovphysx_object_scope_t scope,
                                   ovphysx_query_handle_t* outQuery, ovstage_query_result_t* outSummary)
{
    if (!output.startProducing(OutputKind::kQuery, outQuery))
        return outputRejection(OVPHYSX_API_ERROR);
    if (!output.attachmentCurrent())
        return output.failTask(ovphysx::attachmentChangedCause(output.lifecycleTask()));
    if (!instance.ovstage_attached)
        return output.failTask("ovphysx_query: attach an ovstage Stage first");
    const OvphysxSidecarOutputQueryFn query = g_sidecarOutputQuery.load(std::memory_order_acquire);
    if (!query)
        return output.failTask("ovphysx_query: read API is unavailable");
    omni::physx::RuntimeErrorScope error;
    // The initial scene parse inserts parsed articulations into their scenes. Without it, a read or
    // write opened over this query before the first step finds none of them.
    if (ovphysx_ensure_physics_attached(handle) != OVPHYSX_API_SUCCESS)
        return outputFailure(output, error, "ovphysx_query: failed to attach physics stage");
    ovstage_query_result_t result{};
    const uint64_t resource = query(instance.attachHandle, static_cast<uint32_t>(objectType),
                                    static_cast<uint32_t>(scope), outSummary ? &result : nullptr);
    if (!resource)
        return outputFailure(output, error, "ovphysx_query: failed to open an output query");
    output.produced(resource);
    if (outSummary)
        *outSummary = result;
    return output.complete();
}

struct QueryJob
{
    QueryJob(ovphysx_query_target_t objectType, ovphysx_object_scope_t scope,
             ovstage_query_result_t* outSummary) noexcept
        : objectType(objectType), scope(scope), outSummary(outSummary)
    {
    }
    void run(QueuedOutput& output, InstanceData& instance, ovphysx_instance_handle_t handle) const
    {
        if (objectType == OVPHYSX_QUERY_TARGET_DEBUG_RENDER)
            openDebugRenderQuery(output, handle, scope, nullptr);
        else
            openQuery(output, instance, handle, objectType, scope, nullptr, outSummary);
    }
    ovphysx_query_target_t objectType;
    ovphysx_object_scope_t scope;
    ovstage_query_result_t* outSummary;
};

} // namespace

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_query(ovphysx_instance_handle_t handle,
    ovphysx_query_target_t object_type, ovphysx_object_scope_t scope, ovphysx_query_handle_t* out_query,
    ovstage_query_result_t* out_summary)
{
    OVPHYSX_API_PROFILE_ZONE(query);
    if (out_query)
        *out_query = 0;
    if (out_summary)
        *out_summary = {};
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission, const ovphysx::Caller& caller,
                                  const std::shared_ptr<InstanceData>& owner)
    {
        if (!out_query)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_query: out_query is NULL");
        if (scope != OVPHYSX_SCOPE_ALL && scope != OVPHYSX_SCOPE_ACTIVE)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_query: invalid object type or scope");
        const bool debugRender = object_type == OVPHYSX_QUERY_TARGET_DEBUG_RENDER;
        // Bounded by the sentinel, not the last type's name, so a newly added type is accepted
        // without editing this line. Debug render is an instance target and never reaches the sidecar.
        if (!debugRender && static_cast<uint32_t>(object_type) >= static_cast<uint32_t>(OVPHYSX_QUERY_TARGET_COUNT))
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_query: invalid object type or scope");
        return shortCall(
            instance, handle, admission, caller, "ovphysx_query",
            [&](InlineOutput& output) {
                return debugRender ? openDebugRenderQuery(output, handle, scope, out_query)
                                   : openQuery(output, instance, handle, object_type, scope, out_query, out_summary);
            },
            [&]() {
                return queueProducer<QueryJob>(owner, handle, "ovphysx_query", OutputKind::kQuery, out_query,
                                               object_type, scope, out_summary);
            });
    });
}

namespace
{

// ovphysx_query_selection under the lane, for a validated selection: the caller's on the
// calling thread, the job's copy in a job.
template <typename Output>
ovphysx_enqueue_result_t openSelectedQuery(Output& output, InstanceData& instance, ovphysx_instance_handle_t handle,
                                           ovphysx_query_target_t objectType, const ovphysx_query_selection_t& selection,
                                           ovphysx_query_handle_t* outQuery, ovstage_query_result_t* outSummary)
{
    if (!output.startProducing(OutputKind::kQuery, outQuery))
        return outputRejection(OVPHYSX_API_ERROR);
    if (!output.attachmentCurrent())
        return output.failTask(ovphysx::attachmentChangedCause(output.lifecycleTask()));
    if (!instance.ovstage_attached)
        return output.failTask("ovphysx_query_selection: attach an ovstage Stage first");
    const OvphysxSidecarOutputQuerySelectedFn querySelected =
        g_sidecarOutputQuerySelected.load(std::memory_order_acquire);
    if (!querySelected)
        return output.failTask("ovphysx_query_selection: read API is unavailable");
    omni::physx::RuntimeErrorScope error;
    // As in ovphysx_query: insert parsed articulations before a read or write opens over this query.
    if (ovphysx_ensure_physics_attached(handle) != OVPHYSX_API_SUCCESS)
        return outputFailure(output, error, "ovphysx_query_selection: failed to attach physics stage");
    // Hand the runtime whichever selector is in play; the two are mutually exclusive, so the other
    // is empty/NULL. The pattern buffer outlives the call. A zero resource means FAILURE only
    // (ovstage is attached); an empty match is still a valid, nonzero query -- discover it
    // in the optional creation summary (total_prim_count == 0).
    const bool usingExplicitPaths = selection.prim_paths != nullptr && selection.prim_paths_count > 0;
    const ovx_primpath_t* primPaths = usingExplicitPaths ? selection.prim_paths : nullptr;
    const size_t primPathCount = usingExplicitPaths ? selection.prim_paths_count : 0u;
    const ovx_string_t glob{ selection.pattern.ptr, selection.pattern.length };
    const ovx_string_t* pattern = usingExplicitPaths ? nullptr : &glob;
    ovstage_query_result_t result{};
    const uint64_t resource = querySelected(instance.attachHandle, static_cast<uint32_t>(objectType),
                                            primPaths, primPathCount, pattern, outSummary ? &result : nullptr);
    if (!resource)
        return outputFailure(output, error, "ovphysx_query_selection: failed to open an output query");
    output.produced(resource);
    if (outSummary)
        *outSummary = result;
    return output.complete();
}

struct SelectedQueryJob
{
    SelectedQueryJob(ovphysx_query_target_t objectType, const ovphysx_query_selection_t& selection,
                     ovstage_query_result_t* outSummary)
        : objectType(objectType), outSummary(outSummary)
    {
        copies.measure(selection);
        copies.allocate();
        this->selection = copies.copy(selection);
    }
    void run(QueuedOutput& output, InstanceData& instance, ovphysx_instance_handle_t handle) const
    {
        openSelectedQuery(output, instance, handle, objectType, selection, nullptr, outSummary);
    }
    SelectionCopies copies;
    ovphysx_query_selection_t selection{};
    ovphysx_query_target_t objectType;
    ovstage_query_result_t* outSummary;
};

} // namespace

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_query_selection(ovphysx_instance_handle_t handle,
    ovphysx_query_target_t object_type, const ovphysx_query_selection_t* selection,
    ovphysx_query_handle_t* out_query, ovstage_query_result_t* out_summary)
{
    OVPHYSX_API_PROFILE_ZONE(query_selection);
    if (out_query)
        *out_query = 0;
    if (out_summary)
        *out_summary = {};
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission, const ovphysx::Caller& caller,
                                  const std::shared_ptr<InstanceData>& owner)
    {
        if (!out_query)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_query_selection: out_query is NULL");
        if (!selection)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                "ovphysx_query_selection: selection is NULL (the whole set is ovphysx_query)");
        // Bounded by the sentinel, not the last type's name, like ovphysx_query's gate.
        // Debug render is not a prim selection; ovphysx_query is its entry point.
        if (static_cast<uint32_t>(object_type) >= static_cast<uint32_t>(OVPHYSX_QUERY_TARGET_COUNT) ||
            object_type == OVPHYSX_QUERY_TARGET_DEBUG_RENDER)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_query_selection: invalid object type");

        // prim_paths wins over pattern; the two are never combined (REQ-CAPI-SELECT-001 AC-2). prim_paths is
        // a list of native ovx_primpath_t handles the caller already resolved: validated like
        // ovphysx_debug_render_set_scope_tokens (no NULL array with a non-zero count, no OVX_INVALID_PRIMPATH
        // entry) and NOT string-checked -- the caller owns path resolution; a queued call copies the array.
        // pattern is still a glob string, validated for the embedded-NUL / oversized-component
        // (REQ-CAPI-PATTERN-001) and relative-path cases; the runtime resolves it at query-open and retains nothing.
        //
        // A NULL array with a non-zero count is a caller bug, rejected before precedence is decided:
        // otherwise it would read as "no explicit list" and quietly fall back to the pattern.
        if (!selection->prim_paths && selection->prim_paths_count > 0)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                "ovphysx_query_selection: prim_paths is NULL with prim_paths_count > 0");
        const bool usingExplicitPaths = (selection->prim_paths != nullptr && selection->prim_paths_count > 0);
        if (usingExplicitPaths)
        {
            for (uint32_t i = 0; i < selection->prim_paths_count; i++)
                if (selection->prim_paths[i] == OVX_INVALID_PRIMPATH)
                    return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                        "ovphysx_query_selection: prim_paths must not contain OVX_INVALID_PRIMPATH");
        }
        else if (selection->pattern.ptr && selection->pattern.length > 0)
        {
            if (hasEmbeddedNul(selection->pattern))
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                    "ovphysx_query_selection: pattern contains an embedded NUL byte");
            if (hasOversizedPathComponent(selection->pattern))
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, oversizedPathComponentMessage("pattern"));
            // The runtime matcher would resolve a relative pattern from the root, but every other
            // pattern-taking entry point requires an absolute one; reject it so one pattern means the same
            // thing everywhere.
            if (selection->pattern.ptr[0] != '/')
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                    "ovphysx_query_selection: pattern must be an absolute path starting with '/'");
        }
        else
        {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                "ovphysx_query_selection: selection must provide a pattern or prim_paths");
        }

        return shortCall(
            instance, handle, admission, caller, "ovphysx_query_selection",
            [&](InlineOutput& output) {
                return openSelectedQuery(output, instance, handle, object_type, *selection, out_query, out_summary);
            },
            [&]() {
                return queueProducer<SelectedQueryJob>(owner, handle, "ovphysx_query_selection", OutputKind::kQuery,
                                                       out_query, object_type, *selection, out_summary);
            });
    });
}

namespace
{
// A contact query's selections, validated like ovphysx_query_selection's one descriptor: the error text for
// the first invalid one, or empty when all are valid. `what` names the selection in the message.
std::string contactSelectionError(const ovphysx_query_selection_t& selection, const std::string& what)
{
    if (selection.prim_paths_count > 0 && !selection.prim_paths)
        return "ovphysx_query_contacts: " + what + " has prim_paths_count > 0 but prim_paths is NULL";
    if (selection.prim_paths && selection.prim_paths_count > 0)
    {
        for (uint32_t i = 0; i < selection.prim_paths_count; i++)
            if (selection.prim_paths[i] == OVX_INVALID_PRIMPATH)
                return "ovphysx_query_contacts: " + what + " prim_paths must not contain OVX_INVALID_PRIMPATH";
        return {};
    }
    if (selection.pattern.ptr && selection.pattern.length > 0)
    {
        if (hasEmbeddedNul(selection.pattern))
            return "ovphysx_query_contacts: " + what + " pattern contains an embedded NUL byte";
        if (hasOversizedPathComponent(selection.pattern))
            return "ovphysx_query_contacts: " + what + " " + oversizedPathComponentMessage("pattern");
        // As ovphysx_query_selection: the runtime matcher would resolve a relative pattern from the root, but
        // every pattern-taking entry point requires an absolute one.
        if (selection.pattern.ptr[0] != '/')
            return "ovphysx_query_contacts: " + what + " pattern must be an absolute path starting with '/'";
        return {};
    }
    return "ovphysx_query_contacts: " + what + " must provide a pattern or prim_paths";
}

// The runtime's view of one validated selection: the explicit handles, or else the glob.
omni::physx::OvxPrimSelection runtimeSelection(const ovphysx_query_selection_t& selection, ovx_string_t& glob)
{
    if (selection.prim_paths && selection.prim_paths_count > 0)
        return { selection.prim_paths, selection.prim_paths_count, nullptr };
    glob = ovx_string_t{ selection.pattern.ptr, selection.pattern.length };
    return { nullptr, 0u, &glob };
}

// ovphysx_query_contacts under the lane, for validated selections: the caller's on the
// calling thread, the job's copies in a job.
template <typename Output>
ovphysx_enqueue_result_t openContactQuery(Output& output, InstanceData& instance,
                                          const ovphysx_contact_sensor_selection_t* selections,
                                          uint32_t selectionCount, uint32_t filtersPerSensor,
                                          ovphysx_query_handle_t* outQuery, ovstage_query_result_t* outSummary)
{
    if (!output.startProducing(OutputKind::kQuery, outQuery))
        return outputRejection(OVPHYSX_API_ERROR);
    if (!output.attachmentCurrent())
        return output.failTask(ovphysx::attachmentChangedCause(output.lifecycleTask()));
    if (!instance.ovstage_attached)
        return output.failTask("ovphysx_query_contacts: attach an ovstage Stage first");
    const OvphysxSidecarOutputQueryContactsFn queryContacts =
        g_sidecarOutputQueryContacts.load(std::memory_order_acquire);
    if (!queryContacts)
        return output.failTask("ovphysx_query_contacts: read API is unavailable");
    omni::physx::RuntimeErrorScope error;
    // The runtime's descriptors point into the selection buffers, which outlive the call; the globs and
    // filter arrays are held here for its duration.
    std::vector<ovx_string_t> globs(size_t(selectionCount) * (1u + filtersPerSensor));
    std::vector<omni::physx::OvxPrimSelection> filters(size_t(selectionCount) * filtersPerSensor);
    std::vector<omni::physx::OvxContactSensorSelection> runtime(selectionCount);
    for (uint32_t s = 0; s < selectionCount; s++)
    {
        ovx_string_t* const slot = &globs[size_t(s) * (1u + filtersPerSensor)];
        runtime[s].sensors = runtimeSelection(selections[s].sensors, slot[0]);
        for (uint32_t f = 0; f < filtersPerSensor; f++)
            filters[size_t(s) * filtersPerSensor + f] = runtimeSelection(selections[s].filters[f], slot[1 + f]);
        runtime[s].filters = filtersPerSensor ? &filters[size_t(s) * filtersPerSensor] : nullptr;
    }
    // A zero resource means FAILURE -- an unpairable filter or a repeated one, named by the runtime's
    // message -- since ovstage is attached; an empty match is still a valid, nonzero query.
    ovstage_query_result_t result{};
    const uint64_t resource = queryContacts(instance.attachHandle, runtime.data(), runtime.size(),
                                             filtersPerSensor, outSummary ? &result : nullptr);
    if (!resource)
        return outputFailure(output, error, "ovphysx_query_contacts: failed to open a contact query");
    output.produced(resource);
    if (outSummary)
        *outSummary = result;
    return output.complete();
}

struct ContactQueryJob
{
    ContactQueryJob(const ovphysx_contact_sensor_selection_t* source, uint32_t selectionCount,
                    uint32_t filtersPerSensor, ovstage_query_result_t* outSummary)
        : selections(selectionCount),
          filters(size_t(selectionCount) * filtersPerSensor),
          filtersPerSensor(filtersPerSensor),
          outSummary(outSummary)
    {
        for (uint32_t s = 0; s < selectionCount; s++)
        {
            copies.measure(source[s].sensors);
            for (uint32_t f = 0; f < filtersPerSensor; f++)
                copies.measure(source[s].filters[f]);
        }
        copies.allocate();
        for (uint32_t s = 0; s < selectionCount; s++)
        {
            selections[s].sensors = copies.copy(source[s].sensors);
            ovphysx_query_selection_t* const own = filtersPerSensor ? &filters[size_t(s) * filtersPerSensor] : nullptr;
            for (uint32_t f = 0; f < filtersPerSensor; f++)
                own[f] = copies.copy(source[s].filters[f]);
            selections[s].filters = own;
        }
    }
    void run(QueuedOutput& output, InstanceData& instance, ovphysx_instance_handle_t) const
    {
        openContactQuery(output, instance, selections.data(), static_cast<uint32_t>(selections.size()),
                         filtersPerSensor, nullptr, outSummary);
    }
    SelectionCopies copies;
    std::vector<ovphysx_contact_sensor_selection_t> selections;
    std::vector<ovphysx_query_selection_t> filters;
    uint32_t filtersPerSensor;
    ovstage_query_result_t* outSummary;
};
} // namespace

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_query_contacts(ovphysx_instance_handle_t handle,
    const ovphysx_contact_sensor_selection_t* selections, uint32_t selection_count, uint32_t filters_per_sensor,
    ovphysx_query_handle_t* out_query, ovstage_query_result_t* out_summary)
{
    OVPHYSX_API_PROFILE_ZONE(query_contacts);
    if (out_query)
        *out_query = 0;
    if (out_summary)
        *out_summary = {};
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission, const ovphysx::Caller& caller,
                                  const std::shared_ptr<InstanceData>& owner)
    {
        if (!out_query)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_query_contacts: out_query is NULL");
        if (!selections || selection_count == 0)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                "ovphysx_query_contacts: selections must name at least one sensor selection");
        if (filters_per_sensor > omni::physx::kOvxMaxContactFilters)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                "ovphysx_query_contacts: filters_per_sensor exceeds 21845");
        for (uint32_t s = 0; s < selection_count; s++)
        {
            const std::string where = "selection " + std::to_string(s);
            std::string error = contactSelectionError(selections[s].sensors, where + " sensors");
            if (error.empty() && filters_per_sensor > 0 && !selections[s].filters)
                error = "ovphysx_query_contacts: " + where + " has no filters while filters_per_sensor is " +
                        std::to_string(filters_per_sensor);
            for (uint32_t f = 0; error.empty() && f < filters_per_sensor; f++)
                error = contactSelectionError(selections[s].filters[f], where + " filter " + std::to_string(f));
            if (!error.empty())
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, error);
        }

        return shortCall(
            instance, handle, admission, caller, "ovphysx_query_contacts",
            [&](InlineOutput& output) {
                return openContactQuery(output, instance, selections, selection_count, filters_per_sensor, out_query,
                                        out_summary);
            },
            [&]() {
                return queueProducer<ContactQueryJob>(owner, handle, "ovphysx_query_contacts", OutputKind::kQuery,
                                                      out_query, selections, selection_count, filters_per_sensor,
                                                      out_summary);
            });
    });
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_update_articulations_kinematic(
    ovphysx_instance_handle_t handle, ovphysx_query_handle_t query, uint32_t flags, ovstage_cuda_sync_t cuda_sync)
{
    OVPHYSX_API_PROFILE_ZONE(update_articulations_kinematic);
    constexpr uint32_t kSupportedFlags = OVPHYSX_ARTICULATION_KINEMATIC_POSITION |
                                         OVPHYSX_ARTICULATION_KINEMATIC_VELOCITY;
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission, const ovphysx::Caller& caller,
                                  const std::shared_ptr<InstanceData>&)
    {
        if (flags & ~kSupportedFlags)
            return set_enqueue_error(
                OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_update_articulations_kinematic: unsupported flags");
        // The query-type probe below runs in the runtime and needs the instance idle, so
        // it comes after the lane finished earlier work, including a step job.
        ovphysx::InstanceLane lane;
        const ovphysx_api_status_t entered =
            enterOutputLane(instance, lane, caller, "ovphysx_update_articulations_kinematic");
        if (entered != OVPHYSX_API_SUCCESS)
            return outputRejection(entered);
        const OutputLookup input = lookupOutput(instance, query, OutputKind::kQuery);
        if (!input.admitted())
            return set_enqueue_error(
                OVPHYSX_API_NOT_FOUND, "ovphysx_update_articulations_kinematic: query is not owned by this instance");
        if (input.status != OutputStatus::kLive)
        {
            admission.create_task(instance, handle);
            return admission.fail(taskFailureText("ovphysx_update_articulations_kinematic", admission.receipt.op_index,
                                                  unusableInputCause(OutputKind::kQuery, query, input))
                                      .c_str());
        }
        if (!omni::physx::ovxUpdateArticulationsKinematic(input.backend, 0))
            return set_enqueue_error(
                OVPHYSX_API_INVALID_ARGUMENT,
                "ovphysx_update_articulations_kinematic: query must select live whole articulations");

        admission.create_task(instance, handle);
        const ovphysx::SubmittedAttachment submitted = ovphysx::submittedAttachment(instance);
        if (!ovphysx::attachmentCurrent(instance, submitted))
            return admission.fail(taskFailureText("ovphysx_update_articulations_kinematic", admission.receipt.op_index,
                                                  ovphysx::attachmentChangedCause(submitted.lifecycleTask))
                                      .c_str());
        omni::physx::RuntimeErrorScope error;
        if (!flags)
            return admission.complete();
        if (ovphysx_ensure_physics_attached(handle) != OVPHYSX_API_SUCCESS)
            return outputFailure(
                admission, error, "ovphysx_update_articulations_kinematic: failed to attach physics stage");
        // No warmup: like a write, the refresh never steps on the caller's behalf (ADR-0052).
        if (!omni::physx::ovxUpdateArticulationsKinematic(input.backend, flags, cuda_sync) || error.message()[0])
            return outputFailure(admission, error, "Failed to update articulation kinematics");
        return admission.complete();
    });
}

// The form ovphysx_evaluate_sdf_shapes needs of its tensors, checked before a task exists: float32 [N, Q, 3]
// points and [N, Q, 4] out, C-contiguous on one CUDA device. Empty when they have it, else the reason. Whether N
// is the query's row count is the task's check, since only the task knows the rows.
static std::string sdfTensorsError(const DLTensor* points, const DLTensor* out)
{
    const DLTensor* const tensors[2] = { points, out };
    const char* const names[2] = { "points", "out" };
    const int64_t lanes[2] = { 3, 4 };
    omni::physics::tensors::TensorDesc descs[2];
    for (int i = 0; i < 2; ++i)
    {
        const ovphysx::internal::DLConvertError converted = ovphysx::internal::dlToTensorDesc(tensors[i], descs[i]);
        if (converted != ovphysx::internal::DLConvertError::Success)
            return std::string(names[i]) + ": " + ovphysx::internal::dlConvertErrorMessage(converted);
        const omni::physics::tensors::TensorDesc& d = descs[i];
        if (d.numDims != 3 || d.dims[2] != lanes[i])
            return std::string(names[i]) + " must have shape [N, Q, " + std::to_string(lanes[i]) + "]";
        if (d.dtype != omni::physics::tensors::TensorDataType::eFloat32)
            return std::string(names[i]) + " must be float32";
        if (d.device < 0)
            return std::string(names[i]) + " must be on a CUDA device";
    }
    if (descs[0].dims[0] != descs[1].dims[0] || descs[0].dims[1] != descs[1].dims[1])
        return "points [N, Q, 3] and out [N, Q, 4] must agree on N and Q";
    if (descs[0].dims[1] < 1)
        return "Q must be at least 1";
    if (descs[0].dims[0] > static_cast<int>(omni::physx::kOvxMaxSdfShapesPerEvaluation))
        return "N exceeds 65535: one evaluation addresses at most 65535 SDF shapes (a PhysX launch limit)";
    if (static_cast<int64_t>(descs[0].dims[0]) * descs[0].dims[1] >
        static_cast<int64_t>(omni::physx::kOvxMaxSdfPointsPerEvaluation))
        return "N x Q exceeds 2147483647: one evaluation addresses at most 2^31 - 1 points (a PhysX limit)";
    if (descs[0].device != descs[1].device)
        return "points and out must be on the same CUDA device";
    return std::string();
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_evaluate_sdf_shapes(ovphysx_instance_handle_t handle,
                                                                 ovphysx_query_handle_t query,
                                                                 const DLTensor* points,
                                                                 DLTensor* out,
                                                                 ovstage_cuda_sync_t points_ready)
{
    OVPHYSX_API_PROFILE_ZONE(evaluate_sdf_shapes);
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission, const ovphysx::Caller& caller,
                                  const std::shared_ptr<InstanceData>&)
    {
        const std::string tensors = sdfTensorsError(points, out);
        if (!tensors.empty())
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                                     ("ovphysx_evaluate_sdf_shapes: " + tensors).c_str());
        ovphysx::InstanceLane lane;
        const ovphysx_api_status_t entered = enterOutputLane(instance, lane, caller, "ovphysx_evaluate_sdf_shapes");
        if (entered != OVPHYSX_API_SUCCESS)
            return outputRejection(entered);
        const OutputLookup input = lookupOutput(instance, query, OutputKind::kQuery);
        if (!input.admitted())
            return set_enqueue_error(OVPHYSX_API_NOT_FOUND,
                                     "ovphysx_evaluate_sdf_shapes: query is not owned by this instance");

        admission.create_task(instance, handle);
        const ovphysx::SubmittedAttachment submitted = ovphysx::submittedAttachment(instance);
        if (!ovphysx::attachmentCurrent(instance, submitted))
            return admission.fail(taskFailureText("ovphysx_evaluate_sdf_shapes", admission.receipt.op_index,
                                                  ovphysx::attachmentChangedCause(submitted.lifecycleTask))
                                      .c_str());
        if (input.status != OutputStatus::kLive)
            return admission.fail(taskFailureText("ovphysx_evaluate_sdf_shapes", admission.receipt.op_index,
                                                  unusableInputCause(OutputKind::kQuery, query, input))
                                      .c_str());
        omni::physx::RuntimeErrorScope error;
        if (ovphysx_ensure_physics_attached(handle) != OVPHYSX_API_SUCCESS)
            return outputFailure(admission, error, "ovphysx_evaluate_sdf_shapes: failed to attach physics stage");
        // A call that will be refused -- a stale, wrong-type or empty query, mismatched rows, a CPU scene -- is refused
        // before the warmup, which steps the simulation, as the SDF view refused a stale view before its own.
        if (!omni::physx::ovxCheckSdfShapes(input.backend, points, out) || error.message()[0])
            return outputFailure(admission, error, "SDF shape evaluation failed");
        const ovphysx_result_t warmup = ovphysx_warmup_if_needed(handle, false);
        if (warmup.status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t cause = ovphysx_get_last_error();
            return admission.fail(cause.ptr ? cause.ptr : "SDF evaluation warmup failed");
        }
        if (!omni::physx::ovxEvaluateSdfShapes(input.backend, points, out, points_ready) || error.message()[0])
            return outputFailure(admission, error, "SDF shape evaluation failed");
        return admission.complete();
    });
}

// Private Python binding ABI: cache checks finish pending execution without collecting task outcomes.
// Ordinary queries retain their original attachment; debug-render queries follow the current attachment.
extern "C" OVPHYSX_API ovphysx_result_t ovphysx_internal_is_query_valid(ovphysx_instance_handle_t handle,
                                                                    ovphysx_query_handle_t query,
                                                                    bool* out_valid)
{
    if (!out_valid)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "internal_is_query_valid: out_valid is NULL");
    *out_valid = false;

    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "internal_is_query_valid: invalid handle");
    const ovphysx::Caller caller = ovphysx::currentCaller();
    if (refuseOutputReentry(*instance, caller, "Output metadata") != OVPHYSX_API_SUCCESS)
        return {OVPHYSX_API_INVALID_STATE};

    // Ownership and validity depend on earlier work, such as a detach, so they are
    // checked under the lane once that work finished.
    ovphysx::InstanceLane lane;
    const ovphysx_api_status_t laneStatus = lane.acquire(*instance, caller);
    if (laneStatus != OVPHYSX_API_SUCCESS)
        return set_error(laneStatus, "internal_is_query_valid: failed to finish pending simulation");
    const OutputLookup input = lookupOutput(*instance, query, OutputKind::kQuery);
    if (!input.admitted())
        return set_error(OVPHYSX_API_NOT_FOUND, "internal_is_query_valid: query is not live on this instance");
    if (attachmentBusy(*instance))
        return set_error(OVPHYSX_API_INVALID_STATE, "Output metadata cannot reenter runtime execution");

    // A query whose producer failed was never opened.
    *out_valid = input.status == OutputStatus::kLive && omni::physx::ovxQueryDictionary(input.backend) != nullptr;
    return success();
}

namespace
{

template <typename Output>
ovphysx_enqueue_result_t openRead(Output& output, ovphysx_instance_handle_t handle, ovphysx_query_handle_t query,
                                  const ovx_token_or_string_t* attributes, size_t attributeCount,
                                  ovstage_cuda_sync_t cudaSync, ovphysx_read_handle_t* outRead,
                                  ovphysx_read_result_t* outSummary)
{
    const OutputLookup input = lookupOutput(output.instance(), query, OutputKind::kQuery);
    if (!output.admits(input))
        return output.refuse(OVPHYSX_API_NOT_FOUND, "ovphysx_read: query is not owned by this instance");
    if (!output.startProducing(OutputKind::kRead, outRead))
        return outputRejection(OVPHYSX_API_ERROR);
    if (!output.attachmentCurrent())
        return output.failTask(ovphysx::attachmentChangedCause(output.lifecycleTask()));
    const OvphysxSidecarReadOutputsFn read = g_sidecarReadOutputs.load(std::memory_order_acquire);
    if (!read)
        return output.failTask("ovphysx_read: read API is unavailable");
    if (input.status != OutputStatus::kLive)
        return output.failTask(unusableInputCause(OutputKind::kQuery, query, input));
    omni::physx::RuntimeErrorScope error;
    ovphysx_read_result_t result{};
    const uint64_t resource = read(input.backend, attributes, attributeCount, cudaSync,
                                   outSummary ? &result : nullptr);
    if (!resource)
        return outputFailure(output, error, "ovphysx_read: failed to open a read session");
    output.produced(resource);
    if (outSummary)
        *outSummary = result;
    return output.complete();
}

struct ReadJob
{
    ReadJob(ovphysx_query_handle_t query, const ovx_token_or_string_t* attributes, size_t attributeCount,
            ovstage_cuda_sync_t cudaSync, ovphysx_read_result_t* outSummary)
        : attributes(attributes, attributeCount), query(query), cudaSync(cudaSync), outSummary(outSummary)
    {
    }
    void run(QueuedOutput& output, InstanceData&, ovphysx_instance_handle_t handle) const
    {
        openRead(output, handle, query, attributes.data(), attributes.size(), cudaSync, nullptr, outSummary);
    }
    AttributeCopies attributes;
    ovphysx_query_handle_t query;
    ovstage_cuda_sync_t cudaSync;
    ovphysx_read_result_t* outSummary;
};

template <typename Output>
ovphysx_enqueue_result_t fetchReadGroup(Output& output, ovphysx_instance_handle_t handle, ovphysx_read_handle_t read,
                                        const ovstage_read_group_t** outGroup)
{
    const OutputLookup input = lookupOutput(output.instance(), read, OutputKind::kRead);
    if (!output.admits(input))
        return output.refuse(OVPHYSX_API_NOT_FOUND, "ovphysx_fetch_read_next: read is not owned by this instance");
    if (!output.attachmentCurrent())
        return output.failTask(ovphysx::attachmentChangedCause(output.lifecycleTask()));
    const OvphysxSidecarFetchReadNextFn fetch = g_sidecarFetchReadNext.load(std::memory_order_acquire);
    if (!fetch)
        return output.failTask("ovphysx_fetch_read_next: read API is unavailable");
    if (input.status != OutputStatus::kLive)
        return output.failTask(unusableInputCause(OutputKind::kRead, read, input));
    output.start();
    omni::physx::RuntimeErrorScope error;
    if (fetch(input.backend, outGroup) < 0)
        return outputFailure(output, error, "ovphysx_fetch_read_next: read failed");
    return output.complete();
}

struct FetchReadJob
{
    FetchReadJob(ovphysx_read_handle_t read, const ovstage_read_group_t** outGroup) noexcept
        : read(read), outGroup(outGroup)
    {
    }
    void run(QueuedOutput& output, InstanceData&, ovphysx_instance_handle_t handle) const
    {
        fetchReadGroup(output, handle, read, outGroup);
    }
    ovphysx_read_handle_t read;
    const ovstage_read_group_t** outGroup;
};

// A group release names a read like a release does: a read that is unknown, released
// before, or never opened has no group to release, and the call is a successful no-op.
template <typename Output>
ovphysx_enqueue_result_t releaseReadGroup(Output& output, ovphysx_instance_handle_t handle, ovphysx_read_handle_t read,
                                          ovstage_read_group_id_t groupId)
{
    const OutputLookup input = lookupOutput(output.instance(), read, OutputKind::kRead);
    if (input.owner == OutputLookup::Owner::kOther)
        return output.refuse(OVPHYSX_API_NOT_FOUND, "ovphysx_release_group: read belongs to another instance");
    if (!output.admits(input) || input.status != OutputStatus::kLive)
    {
        output.start();
        return output.complete();
    }
    const OvphysxSidecarReleaseGroupFn release = g_sidecarReleaseGroup.load(std::memory_order_acquire);
    if (!release)
        return output.failTask("ovphysx_release_group: release API is unavailable");
    output.start();
    omni::physx::RuntimeErrorScope error;
    if (release(input.backend, groupId) <= 0 || error.message()[0])
        return outputFailure(output, error, "ovphysx_release_group: release failed");
    return output.complete();
}

struct ReleaseGroupJob
{
    ReleaseGroupJob(ovphysx_read_handle_t read, ovstage_read_group_id_t groupId) noexcept
        : read(read), groupId(groupId)
    {
    }
    void run(QueuedOutput& output, InstanceData&, ovphysx_instance_handle_t handle) const
    {
        releaseReadGroup(output, handle, read, groupId);
    }
    ovphysx_read_handle_t read;
    ovstage_read_group_id_t groupId;
};

} // namespace

// Discovery is the optional summary returned by ovphysx_query. Simulated-type
// queries require an attached stage. A debug-render query can exist with no stage.
OVPHYSX_API ovphysx_enqueue_result_t ovphysx_read(ovphysx_instance_handle_t handle,
    ovphysx_query_handle_t query, const ovx_token_or_string_t* attributes,
    size_t attribute_count, ovstage_cuda_sync_t cuda_sync, ovphysx_read_handle_t* out_read,
    ovphysx_read_result_t* out_summary)
{
    OVPHYSX_API_PROFILE_ZONE(read);
    if (out_read)
        *out_read = 0;
    if (out_summary)
        *out_summary = {};
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission, const ovphysx::Caller& caller,
                                  const std::shared_ptr<InstanceData>& owner)
    {
        if (!out_read || (attribute_count && !attributes))
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_read: invalid output or attribute array");
        for (size_t i = 0; i < attribute_count; ++i)
            if (attributes[i].token == OVX_INVALID_TOKEN && !attributes[i].string.ptr && attributes[i].string.length)
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_read: NULL attribute string has nonzero length");
        if (!cuda_sync.stream && !cuda_sync.wait_event)
        {
            return shortCall(
                instance, handle, admission, caller, "ovphysx_read",
                [&](InlineOutput& output) {
                    return openRead(output, handle, query, attributes, attribute_count, cuda_sync, out_read, out_summary);
                },
                [&]() {
                    if (!lookupOutput(instance, query, OutputKind::kQuery).admitted())
                        return set_enqueue_error(OVPHYSX_API_NOT_FOUND,
                                                 "ovphysx_read: query is not owned by this instance");
                    return queueProducer<ReadJob>(owner, handle, "ovphysx_read", OutputKind::kRead, out_read, query,
                                                  attributes, attribute_count, cuda_sync, out_summary);
                });
        }

        if (!lookupOutput(instance, query, OutputKind::kQuery).admitted())
            return set_enqueue_error(OVPHYSX_API_NOT_FOUND, "ovphysx_read: query is not owned by this instance");
        ovphysx::InstanceLane lane;
        const ovphysx_api_status_t entered = enterOutputLane(instance, lane, caller, "ovphysx_read");
        if (entered != OVPHYSX_API_SUCCESS)
            return outputRejection(entered);
        InlineOutput output(instance, handle, admission, "ovphysx_read");
        return openRead(output, handle, query, attributes, attribute_count, cuda_sync, out_read, out_summary);
    });
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_fetch_read_next(ovphysx_instance_handle_t handle,
    ovphysx_read_handle_t read, const ovstage_read_group_t** out_group)
{
    OVPHYSX_API_PROFILE_ZONE(fetch_read_next);
    if (out_group)
        *out_group = nullptr;
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission, const ovphysx::Caller& caller,
                                  const std::shared_ptr<InstanceData>& owner)
    {
        if (!out_group)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_fetch_read_next: output is NULL");
        return shortCall(
            instance, handle, admission, caller, "ovphysx_fetch_read_next",
            [&](InlineOutput& output) { return fetchReadGroup(output, handle, read, out_group); },
            [&]() {
                if (!lookupOutput(instance, read, OutputKind::kRead).admitted())
                    return set_enqueue_error(OVPHYSX_API_NOT_FOUND,
                                             "ovphysx_fetch_read_next: read is not owned by this instance");
                return queueOutputJob<FetchReadJob>(owner, handle, "ovphysx_fetch_read_next", queuedAdmission(instance),
                                                    read, out_group);
            });
    });
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_release_group(ovphysx_instance_handle_t handle,
    ovphysx_read_handle_t read, ovstage_read_group_id_t group_id)
{
    OVPHYSX_API_PROFILE_ZONE(release_group);
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission, const ovphysx::Caller& caller,
                                  const std::shared_ptr<InstanceData>& owner)
    {
        // Releases join the lane in submission order.
        return shortCall(
            instance, handle, admission, caller, "ovphysx_release_group",
            [&](InlineOutput& output) { return releaseReadGroup(output, handle, read, group_id); },
            [&]() {
                const OutputLookup input = lookupOutput(instance, read, OutputKind::kRead);
                if (input.owner == OutputLookup::Owner::kOther)
                    return set_enqueue_error(OVPHYSX_API_NOT_FOUND,
                                             "ovphysx_release_group: read belongs to another instance");
                QueuedAdmission queued;
                queued.inputReleased = !input.admitted();
                return queueOutputJob<ReleaseGroupJob>(owner, handle, "ovphysx_release_group", queued, read,
                                                       group_id);
            });
    });
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_release_read(ovphysx_instance_handle_t handle, ovphysx_read_handle_t read)
{
    OVPHYSX_API_PROFILE_ZONE(release_read);
    return releaseOutput(handle, read, OutputKind::kRead, releaseDebugOrRead, "ovphysx_release_read");
}

// --- App -> physics write (ADR-0012) ----------------------------------------
// The return direction of ovphysx_read above, and routed the same way: through the internal
// sidecar to the runtime's ovx* write entry points. Argument and handle validation lives HERE
// rather than in a frontend because REQ-CAPI-WRITE-001 AC-8 makes it C-first. Every frontend
// inherits these checks, and no frontend may add one the C API does not make.

namespace
{

template <typename Output>
ovphysx_enqueue_result_t openWrite(Output& output, ovphysx_instance_handle_t handle, ovphysx_query_handle_t query,
                                   const ovx_token_or_string_t* attributes, size_t attributeCount,
                                   ovphysx_write_handle_t* outWrite, ovphysx_write_result_t* outSummary)
{
    const OutputLookup input = lookupOutput(output.instance(), query, OutputKind::kQuery);
    if (!output.admits(input))
        return output.refuse(OVPHYSX_API_NOT_FOUND, "ovphysx_write: query is not owned by this instance");
    if (!output.startProducing(OutputKind::kWrite, outWrite))
        return outputRejection(OVPHYSX_API_ERROR);
    if (!output.attachmentCurrent())
        return output.failTask(ovphysx::attachmentChangedCause(output.lifecycleTask()));
    const OvphysxSidecarWriteAttributeFn write = g_sidecarWriteAttribute.load(std::memory_order_acquire);
    if (!write)
        return output.failTask("ovphysx_write: write API is unavailable");
    if (input.status != OutputStatus::kLive)
        return output.failTask(unusableInputCause(OutputKind::kQuery, query, input));
    omni::physx::RuntimeErrorScope error;
    ovphysx_write_result_t result{};
    const uint64_t resource = write(input.backend, attributes, attributeCount, outSummary ? &result : nullptr);
    if (!resource)
        return outputFailure(output, error, "ovphysx_write: failed to open a write session");
    output.produced(resource);
    if (outSummary)
        *outSummary = result;
    return output.complete();
}

struct WriteJob
{
    WriteJob(ovphysx_query_handle_t query, const ovx_token_or_string_t* attributes, size_t attributeCount,
             ovphysx_write_result_t* outSummary)
        : attributes(attributes, attributeCount), query(query), outSummary(outSummary)
    {
    }
    void run(QueuedOutput& output, InstanceData&, ovphysx_instance_handle_t handle) const
    {
        openWrite(output, handle, query, attributes.data(), attributes.size(), nullptr, outSummary);
    }
    AttributeCopies attributes;
    ovphysx_query_handle_t query;
    ovphysx_write_result_t* outSummary;
};

} // namespace

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_write(ovphysx_instance_handle_t handle,
    ovphysx_query_handle_t query, const ovx_token_or_string_t* attributes, size_t attribute_count,
    ovphysx_write_handle_t* out_write, ovphysx_write_result_t* out_summary)
{
    OVPHYSX_API_PROFILE_ZONE(write);
    if (out_write)
        *out_write = 0;
    if (out_summary)
        *out_summary = {};
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission, const ovphysx::Caller& caller,
                                  const std::shared_ptr<InstanceData>& owner)
    {
        if (!out_write || !attributes || !attribute_count)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_write: output and nonempty attributes are required");
        for (size_t i = 0; i < attribute_count; ++i)
            if (attributes[i].token == OVX_INVALID_TOKEN && !attributes[i].string.ptr && attributes[i].string.length)
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_write: NULL attribute string has nonzero length");
        return shortCall(
            instance, handle, admission, caller, "ovphysx_write",
            [&](InlineOutput& output) {
                return openWrite(output, handle, query, attributes, attribute_count, out_write, out_summary);
            },
            [&]() {
                if (!lookupOutput(instance, query, OutputKind::kQuery).admitted())
                    return set_enqueue_error(OVPHYSX_API_NOT_FOUND, "ovphysx_write: query is not owned by this instance");
                return queueProducer<WriteJob>(owner, handle, "ovphysx_write", OutputKind::kWrite, out_write, query,
                                               attributes, attribute_count, out_summary);
            });
    });
}

/**
 * @implements REQ-INPUT-COVERAGE-001
 * @covers AC-2
 *
 * The (object type, attribute) writability classification. `ovphysx_writability` and its backing
 * table `kWritabilityTable` are generated from the runtime I/O contract, kept here so
 * the query is bare-process (no instance, scene or step).
 */
namespace
{
// Generated static catalog; it needs no instance, scene or step.
// test_writability_parity.cpp compares it with read/write admission for every
// public attribute and object type. Link/debug admission shares contract metadata;
// parity there checks integration, while live tests cover conditional behavior.
// An attribute absent for its object type is UNCLASSIFIED.
struct WritabilityRow
{
    ovphysx_query_target_t object_type;
    const char* attribute;
    ovphysx_writability_t writability;
};

const WritabilityRow kWritabilityTable[] = {
#include "generated/AttributeWritability.inc"
};

// True iff the NUL-terminated table literal equals name[0..len). Walking the literal and stopping
// at its own NUL never reads past a literal shorter than len, which std::memcmp(row.attribute,
// name, len) would. The trailing check pins its length to len.
bool attributeNameEquals(const char* attribute, const char* name, size_t len)
{
    for (size_t i = 0; i < len; ++i)
        if (attribute[i] == '\0' || attribute[i] != name[i])
            return false;
    return attribute[len] == '\0';
}

ovphysx_writability_t classifyWritability(ovphysx_query_target_t type, const char* name, size_t len)
{
    for (const WritabilityRow& row : kWritabilityTable)
        if (row.object_type == type && attributeNameEquals(row.attribute, name, len))
            return row.writability;
    return OVPHYSX_WRITABILITY_UNCLASSIFIED;
}
} // namespace

OVPHYSX_API ovphysx_result_t ovphysx_writability(ovphysx_query_target_t object_type,
                                                 const ovx_token_or_string_t* attribute,
                                                 ovphysx_writability_t* out_writability)
{
    OVPHYSX_API_PROFILE_ZONE(writability);
    if (!out_writability)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_writability: out_writability is NULL");
    *out_writability = OVPHYSX_WRITABILITY_UNCLASSIFIED;
    if (!attribute)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_writability: attribute is NULL");

    // String-only: writability classifies an attribute by NAME (OVPHYSX_ATTR_*). Resolving an
    // interned token would need a path dictionary, which this scene-free query does not have, so a
    // token-only key is rejected outright rather than silently misclassified as UNCLASSIFIED.
    if (!attribute->string.ptr || attribute->string.length == 0)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                         "ovphysx_writability: attribute must be a string name (OVPHYSX_ATTR_*); "
                         "interned tokens are not supported");

    // Reject an object_type outside ovphysx_query_target_t. The enum
    // is contiguous [RIGID_BODY, last), and the uint32_t cast folds negatives into the high range, so
    // this one bound against the OVPHYSX_QUERY_TARGET_COUNT sentinel covers them and tracks the enum
    // automatically -- a new type extends the valid range without editing this line. Without it an
    // unknown type falls through classifyWritability and returns SUCCESS/UNCLASSIFIED, indistinguishable
    // to a caller from a valid type that simply does not accept the name.
    if (static_cast<uint32_t>(object_type) >= OVPHYSX_QUERY_TARGET_COUNT)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_writability: unknown object type");

    *out_writability = classifyWritability(object_type, attribute->string.ptr, attribute->string.length);
    return success();
}

namespace
{

// Whether a write group's storage is device memory. A group is device-uniform, so its
// first tensor decides.
bool groupInDeviceMemory(const ovstage_map_group_t& group) noexcept
{
    return group.data.tensor_count > 0 && group.data.tensors && group.data.tensors[0].device.device_type != kDLCPU;
}

template <typename Output>
ovphysx_enqueue_result_t fetchWriteGroup(Output& output, ovphysx_instance_handle_t handle,
                                         ovphysx_write_handle_t write, ovphysx_fetched_write_group_t* outGroup)
{
    const OutputLookup input = lookupOutput(output.instance(), write, OutputKind::kWrite);
    if (!output.admits(input))
        return output.refuse(OVPHYSX_API_NOT_FOUND, "ovphysx_fetch_write_next: write is not owned by this instance");
    if (!output.attachmentCurrent())
        return output.failTask(ovphysx::attachmentChangedCause(output.lifecycleTask()));
    const OvphysxSidecarFetchWriteNextFn fetch = g_sidecarFetchWriteNext.load(std::memory_order_acquire);
    if (!fetch)
        return output.failTask("ovphysx_fetch_write_next: write API is unavailable");
    if (input.status != OutputStatus::kLive)
        return output.failTask(unusableInputCause(OutputKind::kWrite, write, input));
    output.start();
    omni::physx::RuntimeErrorScope error;
    if (fetch(input.backend, outGroup) < 0)
        return outputFailure(output, error, "ovphysx_fetch_write_next: fetch failed");
    // From the first group in device memory on, the session's commits run on the calling
    // thread (see kOutputDeviceGroups). Marked before the call completes, so a commit of this
    // group, which needs its completion, sees the mark.
    if (!(input.flags & kOutputDeviceGroups) && outGroup->group && groupInDeviceMemory(*outGroup->group))
        markOutput(output.instance(), write, kOutputDeviceGroups);
    return output.complete();
}

struct FetchWriteJob
{
    FetchWriteJob(ovphysx_write_handle_t write, ovphysx_fetched_write_group_t* outGroup) noexcept
        : write(write), outGroup(outGroup)
    {
    }
    void run(QueuedOutput& output, InstanceData&, ovphysx_instance_handle_t handle) const
    {
        fetchWriteGroup(output, handle, write, outGroup);
    }
    ovphysx_write_handle_t write;
    ovphysx_fetched_write_group_t* outGroup;
};

template <typename Output>
ovphysx_enqueue_result_t commitGroup(Output& output, ovphysx_instance_handle_t handle, ovphysx_write_handle_t write,
                                     const ovstage_map_group_t* group, const DLTensor* rowMask,
                                     ovstage_cuda_sync_t writeDoneSync, const std::string& name)
{
    const OutputLookup input = lookupOutput(output.instance(), write, OutputKind::kWrite);
    if (!output.admits(input))
        return output.refuse(OVPHYSX_API_NOT_FOUND, name + ": write is not owned by this instance");
    if (!output.attachmentCurrent())
        return output.failTask(ovphysx::attachmentChangedCause(output.lifecycleTask()));
    const OvphysxSidecarCommitGroupFn commit = g_sidecarCommitGroup.load(std::memory_order_acquire);
    const OvphysxSidecarCommitGroupMaskedFn commitMasked = g_sidecarCommitGroupMasked.load(std::memory_order_acquire);
    if (rowMask ? !commitMasked : !commit)
        return output.failTask(name + ": write API is unavailable");
    if (input.status != OutputStatus::kLive)
        return output.failTask(unusableInputCause(OutputKind::kWrite, write, input));
    const uint64_t session = input.backend;
    // A commit the runtime would refuse as given -- a mask that does not fit the group, a group applied on the
    // host on a capturing stream -- is refused before it is admitted, so the group stays live: on the calling
    // thread the call is rejected and no task is spent on it, in a job that task fails. The runtime holds the
    // rules: a row's meaning depends on the group's kind.
    {
        const OvphysxSidecarValidateCommitFn validate = g_sidecarValidateCommit.load(std::memory_order_acquire);
        omni::physx::RuntimeErrorScope refusal;
        if (validate && validate(session, group, rowMask, writeDoneSync) == 0)
        {
            const char* const why =
                refusal.message()[0] ? refusal.message() : "the commit was refused; the group is still live";
            return output.refuse(OVPHYSX_API_INVALID_ARGUMENT, name + ": " + why);
        }
    }
    output.start();
    int32_t why = kOvphysxCommitFailureNone;
    omni::physx::RuntimeErrorScope error;
    const int committed = rowMask ? commitMasked(session, group, rowMask, writeDoneSync, &why) :
                                    commit(session, group, writeDoneSync, &why);
    if (!committed)
    {
        std::string context = name + ": outcome is unknown; do not retry the commit";
        if (why == kOvphysxCommitFailureNotLive)
            context = name + ": group is not live; nothing was published";
        else if (why == kOvphysxCommitFailurePublish)
            context = name + ": group is spent; commit is not retryable";
        else if (why == kOvphysxCommitFailureBusy)
            context = name + ": runtime is busy; nothing was published";
        else if (why == kOvphysxCommitFailureInvalidArgument)
            context = name + ": the commit was refused; the group is still live and nothing was published";
        if (error.message()[0])
            return output.fail((context + ": " + error.message()).c_str());
        return output.fail(context.c_str());
    }
    if (error.message()[0])
        return output.fail(error.message());
    return output.complete();
}

// A queued unmasked commit of a host group with nothing outstanding: the group stays
// borrowed until completion.
struct CommitJob
{
    CommitJob(ovphysx_write_handle_t write, const ovstage_map_group_t* group, const char* api) noexcept
        : write(write), group(group), api(api)
    {
    }
    void run(QueuedOutput& output, InstanceData&, ovphysx_instance_handle_t handle) const
    {
        commitGroup(output, handle, write, group, nullptr, ovstage_cuda_sync_t{0, 0}, std::string(api));
    }
    ovphysx_write_handle_t write;
    const ovstage_map_group_t* group;
    const char* api;
};

} // namespace

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_fetch_write_next(ovphysx_instance_handle_t handle,
    ovphysx_write_handle_t write, ovphysx_fetched_write_group_t* out_group)
{
    OVPHYSX_API_PROFILE_ZONE(fetch_write_next);
    if (out_group)
        *out_group = {nullptr, SIZE_MAX};
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission, const ovphysx::Caller& caller,
                                  const std::shared_ptr<InstanceData>& owner)
    {
        if (!out_group)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_fetch_write_next: output is NULL");
        return shortCall(
            instance, handle, admission, caller, "ovphysx_fetch_write_next",
            [&](InlineOutput& output) { return fetchWriteGroup(output, handle, write, out_group); },
            [&]() {
                if (!lookupOutput(instance, write, OutputKind::kWrite).admitted())
                    return set_enqueue_error(OVPHYSX_API_NOT_FOUND,
                                             "ovphysx_fetch_write_next: write is not owned by this instance");
                return queueOutputJob<FetchWriteJob>(owner, handle, "ovphysx_fetch_write_next",
                                                     queuedAdmission(instance), write, out_group);
            });
    });
}

// The commit both public entry points share; `row_mask` is null for an unmasked commit.
static ovphysx_enqueue_result_t commitWriteGroup(ovphysx_instance_handle_t handle,
                                                 ovphysx_write_handle_t write,
                                                 const ovstage_map_group_t* group,
                                                 const DLTensor* row_mask,
                                                 ovstage_cuda_sync_t write_done_sync,
                                                 const char* api)
{
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission, const ovphysx::Caller& caller,
                                  const std::shared_ptr<InstanceData>& owner)
    {
        const std::string name(api);
        if (!group)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, name + ": group is NULL");
        if (row_mask && (row_mask->ndim != 1 || row_mask->dtype.bits != 8 || row_mask->dtype.lanes != 1 ||
                         (row_mask->dtype.code != kDLBool && row_mask->dtype.code != kDLUInt)))
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                                     name + ": row_mask must be a 1-D bool or uint8 tensor");
        ovphysx::InstanceLane lane;
        // Only an unmasked commit with nothing outstanding ({0, 0}) of a session that never
        // handed out device memory is a short async call: the runtime cannot refuse it as
        // given, so its outcome is the same queued or not. Any other is ordered on the
        // caller's CUDA stream, waits on this thread for the caller's producer, or carries a
        // mask the runtime may refuse, so it runs here after earlier work, as a synchronous
        // call, and such a refusal rejects it.
        if (!row_mask && !write_done_sync.stream && !write_done_sync.wait_event)
        {
            ovphysx_api_status_t rejection = OVPHYSX_API_SUCCESS;
            const ShortCallVenue venue = enterShortCall(instance, lane, caller, api, rejection);
            if (venue == ShortCallVenue::kRejected)
                return outputRejection(rejection);
            if (venue == ShortCallVenue::kQueued)
            {
                const OutputLookup input = lookupOutput(instance, write, OutputKind::kWrite);
                if (!input.admitted())
                    return set_enqueue_error(OVPHYSX_API_NOT_FOUND, name + ": write is not owned by this instance");
                if (!(input.flags & kOutputDeviceGroups))
                    return queueOutputJob<CommitJob>(owner, handle, api, queuedAdmission(instance), write, group, api);
            }
        }
        if (!lane.held())
        {
            const ovphysx_api_status_t entered = enterOutputLane(instance, lane, caller, api);
            if (entered != OVPHYSX_API_SUCCESS)
                return outputRejection(entered);
        }
        InlineOutput output(instance, handle, admission, api);
        return commitGroup(output, handle, write, group, row_mask, write_done_sync, name);
    });
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_commit_group(ovphysx_instance_handle_t handle,
    ovphysx_write_handle_t write, const ovstage_map_group_t* group, ovstage_cuda_sync_t write_done_sync)
{
    OVPHYSX_API_PROFILE_ZONE(commit_group);
    return commitWriteGroup(handle, write, group, nullptr, write_done_sync, "ovphysx_commit_group");
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_commit_group_masked(ovphysx_instance_handle_t handle,
                                                                 ovphysx_write_handle_t write,
                                                                 const ovstage_map_group_t* group,
                                                                 const DLTensor* row_mask,
                                                                 ovstage_cuda_sync_t write_done_sync)
{
    OVPHYSX_API_PROFILE_ZONE(commit_group_masked);
    if (!row_mask)
        return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_commit_group_masked: row_mask is NULL");
    return commitWriteGroup(handle, write, group, row_mask, write_done_sync, "ovphysx_commit_group_masked");
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_release_write(ovphysx_instance_handle_t handle, ovphysx_write_handle_t write)
{
    OVPHYSX_API_PROFILE_ZONE(release_write);
    return releaseOutput(handle, write, OutputKind::kWrite, releaseDebugOrWrite, "ovphysx_release_write");
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_release_query(ovphysx_instance_handle_t handle, ovphysx_query_handle_t query)
{
    OVPHYSX_API_PROFILE_ZONE(release_query);
    return releaseOutput(handle, query, OutputKind::kQuery, releaseDebugOrQuery, "ovphysx_release_query");
}

// Private Python binding ABI: check CUDA wait failures before exposing borrowed arrays.
extern "C" OVPHYSX_API ovphysx_result_t ovphysx_internal_cuda_stream_wait_event(uintptr_t stream, uintptr_t event)
{
    // event == 0 means "nothing to wait for". It is answered before the shim is touched so a
    // CPU-only process never reaches CUDA. Stream 0 is not a sentinel, it is the valid NULL stream.
    if (event == 0)
        return success();

    omni::physx::IOptionalCuda* cuda = omni::physx::runtime::tryGetOptionalCudaInterface();
    if (!cuda || !cuda->streamWaitEvent)
        return set_error(OVPHYSX_API_ERROR,
                         "ovphysx_internal_cuda_stream_wait_event: CUDA is not available in this process");

    // No context push: the driver call must run in the CALLER's current context. Stream sentinels 1
    // and 2 are context-relative (the legacy / per-thread default stream of whatever context is
    // current), so making the ovphysx context current would order a stream the caller never used,
    // yet still succeed.

    // Enqueue-only: the driver records the dependency on `stream` and returns, with no host sync.
    int status = 0;
    if (!cuda->streamWaitEvent(stream, event, /*flags=*/0u, &status))
        return set_error(OVPHYSX_API_ERROR,
                         "ovphysx_internal_cuda_stream_wait_event: cuStreamWaitEvent failed with CUDA status " +
                             std::to_string(status));
    return success();
}
