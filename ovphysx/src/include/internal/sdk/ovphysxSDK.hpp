// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CAPI-COMPANION-001
// @covers AC-1 AC-2 AC-3 AC-4

/**
 * @implements REQ-CAPI-BINDING-SELECTION-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-1 AC-2 AC-3 AC-5 AC-6 AC-9 AC-12 AC-13 AC-14 AC-15 AC-16
 *
 * @implements REQ-CAPI-TIMESTEP-002
 * @covers AC-1 AC-4
 */

/**
 * @implements REQ-CAPI-TASK-001
 * @covers AC-1 AC-2 AC-5 AC-6 AC-7
 *
 * @implements REQ-CAPI-STRING-001
 * @covers AC-2
 *
 * @implements REQ-CAPI-INSTANCE-SCOPE-001
 * @covers AC-2 AC-5
 *
 * @implements REQ-CAPI-SDFVIEW-001
 * @covers AC-1
 *
 * @implements REQ-CAPI-LANE-001
 * @covers AC-1 AC-3 AC-4
 *
 * @implements REQ-CAPI-REENTRY-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-CAPI-ASYNC-003
 * @covers AC-1 AC-5 AC-7 AC-8
 */

#pragma once

#include "ovphysx/ovphysx.h"
#include "CarboniteLoader/CarboniteLoader.hpp"
#include "internal/sdk/InstanceExecutor.hpp"
#include "internal/sdk/ovphysxObjectHandleAllocator.hpp"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <map>
#include <memory>
#include <string>
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <cstring>
#include <cstdint>
#include <cstddef> // offsetof
#include <functional>

// AttachHandle is a value type, so it needs the real declaration. ISimulationView.h
// brings kMaxPathPatternComponentLength and forward-declares the view classes.
#include <omni/physics/tensors/TensorApi.h>
#include <omni/physics/tensors/ISimulationView.h>

struct TaskWaitStorage;

namespace carb
{
namespace tasking
{
struct ITasking;
} // namespace tasking
} // namespace carb

struct TensorBindingState
{
    // Identity of the attach this binding was made against, NOT a USD stage id
    // (ADR-0013). A stage id cannot serve here: it is 0 both for a stageless
    // attach and for "detached", so the staleness check below could not tell a
    // live binding from one whose attach had been torn down and replaced.
    omni::physics::tensors::AttachHandle attachHandle = omni::physics::tensors::kNoAttach;
    ovphysx_tensor_type_t tensorType = OVPHYSX_TENSOR_INVALID;
    std::shared_ptr<const std::vector<std::string>> patterns;
    // Attributes share a simulation view, which owns the borrowed child view below.
    std::shared_ptr<omni::physics::tensors::ISimulationView> simView;
    omni::physics::tensors::IRigidBodyView* rbView = nullptr;
    omni::physics::tensors::IArticulationView* artiView = nullptr;
    omni::physics::tensors::IDeformableBodyView* defBodyView = nullptr;
    omni::physics::tensors::IDeformableMaterialView* defMatView = nullptr;
    std::vector<std::string> rigidBodyPrimPathCache;
    std::vector<std::string> articulationPrimPathCache;
    std::vector<std::string> deformableBodyPrimPathCache;

    // Scratch GPU buffer used by ovphysx wrench AoS->SoA conversion.
    // Allocated via omni::physx::IOptionalCuda within the PhysX CUDA context.
    uintptr_t wrenchSoaScratchDev = 0;
    size_t wrenchSoaScratchBytes = 0;
};

struct ContactBindingState
{
    // Attach identity, not a USD stage id. See TensorBindingState::attachHandle.
    omni::physics::tensors::AttachHandle attachHandle = omni::physics::tensors::kNoAttach;
    omni::physics::tensors::ISimulationView* simView = nullptr;
    omni::physics::tensors::IRigidContactView* contactView = nullptr;
    std::vector<std::string> sensorPathCache;
    std::vector<std::string> filterPathCache;
    // Owns the path strings returned by the most recent
    // ovphysx_contact_binding_get_other_actor_paths_from_ids() call.
    // Reused across calls, so the ovphysx_string_t.ptr values handed to the
    // caller remain valid until the next call replaces the cache or the
    // binding is destroyed.
    std::vector<std::string> otherActorPathsCache;
};

// Layout-compatible with omni::physics::ovstage::OvstageAttach, but kept local
// so libovphysx does not take a source-level dependency on ovstage headers.
// The sealed read ordinal is not carried here. It is passed explicitly to
// IPhysxSimulation::attachOvstage(payload, readOrdinal).
//
// The runtime receives this struct as a const void* that the ovstage backend
// reinterprets as OvstageAttach. The two definitions cannot include each other,
// so the shared ABI layout is pinned on both sides: the static_assert below here,
// and the matching guard next to OvstageAttach in OvstageParseBackend.h.
struct OvstageAttachPayload
{
    const void* instance = nullptr;
    void* dict = nullptr; // ovx_path_dictionary_t*
    // Explicit developer compatibility field in the shared layout; production leaves it zero.
    uint64_t usdStageId = 0;
};

// Frozen ABI layout for 64-bit pointers (LP64/LLP64). Must match OvstageAttach.
static_assert(sizeof(OvstageAttachPayload) == 24, "OvstageAttachPayload ABI size drifted from OvstageAttach");
static_assert(offsetof(OvstageAttachPayload, instance) == 0, "OvstageAttachPayload::instance offset drifted");
static_assert(offsetof(OvstageAttachPayload, dict) == 8, "OvstageAttachPayload::dict offset drifted");
static_assert(offsetof(OvstageAttachPayload, usdStageId) == 16, "OvstageAttachPayload::usdStageId offset drifted");

// Lock ordering for InstanceData (to prevent deadlocks):
//   instance lanes (see ovphysx::InstanceLane; several only in ascending handle order)
//   g_lifecycleMutex (attach and detach bodies, destroy's teardown; see ovphysx.cpp)
//   g_createInstanceMutex
//   g_gpuAttachMutex
//   g_recordingMutex (the recording state; see ovphysx.cpp)
//   g_instances_mutex (global - shared or exclusive)
//   InstanceData::tensor_binding_mutex (per-instance)
//   InstanceData::output_handles.mutex (per-instance, leaf)
//   the executor mutex (per-instance task table; see InstanceExecutor)
//   InstanceData::retained_wait_results_mutex (per-instance, innermost)
// Nothing is taken while an output-handle lock is held. A foreign-handle check
// takes g_instances_mutex first, then each peer's output-handle lock.
//
// Nothing waits for a lane while holding a lane or any lock below it: a callback
// running under a held lane may block on those locks. Calls that need several lanes
// decide that before taking any. In particular an attach or detach job holds its
// instance's lane and the lifecycle lock and never waits for another lane; attach and
// detach that need every lane take them all before the lifecycle lock.
//
// get_instance() may release g_instances_mutex after pinning the InstanceData
// lifetime in a shared_ptr. Never acquire the global lock while holding a
// per-instance lock.
//
// IMPORTANT: Always release locks BEFORE calling blocking CUDA functions
// (cuEventSynchronize, cuStreamWaitEvent) to avoid deadlocks and contention.
//
// Note: Move operations assume no concurrent access. Do not move InstanceData
// after it has been published to other threads.

namespace ovphysx
{
// Where an instance runs its short async calls, set for tests only (see
// InstanceData::test_execution).
enum class TestExecution : uint8_t
{
    // On the calling thread when the instance is idle, otherwise queued behind its work.
    kNatural,
    // Every short async call queued, even when idle.
    kQueue,
};

enum class OutputKind : uint8_t
{
    kQuery,
    kRead,
    kWrite,
};

enum class OutputStatus : uint8_t
{
    kPending,
    kLive,
    kFailed,
    kClosed,
};

struct OutputRecord
{
    // Backend resource after the producer opens it.
    uint64_t backend = 0;
    // Producer task index after producer failure.
    uint64_t producer = 0;
    OutputKind kind = OutputKind::kQuery;
    OutputStatus status = OutputStatus::kPending;
    // Release-accepted and device-group state; see ovphysxPhysXInterop.cpp.
    uint8_t flags = 0;
};

struct OutputHandleTable
{
    mutable std::shared_mutex mutex;
    std::unordered_map<uint64_t, OutputRecord> records;
};
} // namespace ovphysx

struct InstanceData
{
    // Companion admission is protected by g_instances_mutex; lease release is
    // atomic. This does not make general same-instance calls concurrent-safe.
    std::atomic<uint32_t> companionLeases{0};
    bool companionDestroying = false;
    std::shared_ptr<ovphysx::CarboniteLoader> carbonite;
    ovphysx_create_args create_args;
    // Canonical instance configuration. The string owns the value returned by
    // ovphysx_get_config; the parsed vector is consumed at stage attach.
    std::string active_cuda_gpus;
    bool active_cuda_gpus_configured = false;
    // Parsed active_cuda_gpus ordinals, resolved once at create time. Empty when
    // the caller did not restrict GPU ordinals. create_args.active_cuda_gpus is a
    // non-owning {ptr,length} view into the caller's buffer, which may be freed
    // once ovphysx_create_instance() returns. Both scene-attach paths read the
    // ordinals lazily, so the parsed result is kept here rather than the
    // dangling string.
    std::vector<int32_t> active_cuda_ordinals;
    // Backing-stage id reported by ovstage for the attached source, or 0 when it
    // has none. Informational (logging, detach bookkeeping) only: ovphysx does no
    // USD work with it, and it is never attach identity. attachHandle is.
    int64_t attachedStageId = 0;
    // Identity of the current attach: nonzero while attached (stageless or not),
    // and distinct for every attach, so a stored copy detects detach/reattach.
    omni::physics::tensors::AttachHandle attachHandle = omni::physics::tensors::kNoAttach;
    bool ovstage_attached = false;
    OvstageAttachPayload ovstage_attach_payload;

    // Target paths already cloned to on the current attach. A repeat clone() to the same path
    // (reused across batches) would create a duplicate object under an existing path, so its
    // task fails. Cleared on stage teardown (detach / unload), so a fresh attach starts empty.
    // Read and written only by the holder of the instance lane: the instance worker for a
    // clone, attach or detach job, or the calling thread for attach and detach while a
    // recording is active and for destroy.
    std::unordered_set<std::string> cloned_target_paths;

    // Simulation state
    // dt of the last successful step that simulated, used by contact
    // read functions (force = impulse / dt). Initialized to 1.0f so pre-step
    // reads are safe. Only positive dts are written: a zero-dt step is a skipped
    // no-op that leaves the previous step's contact impulses in place, so the
    // divisor must stay on that step's dt -- overwriting it would scale every
    // contact force by 1/dt. Never 0, so the conversion cannot divide by zero.
    // Read and written only by the holder of the instance lane: the calling thread,
    // or the instance worker for a step job.
    float last_step_dt = 1.0f;
    // Internal simulation-time counter: the simulation time at the start of the
    // next step. Initialized to 0.0 at construction and advanced by dt per
    // step/step_sync. Fed to simulate() for
    // timestamps and not exposed through the public API. Guarded by the instance
    // lane like last_step_dt.
    float sim_time = 0.0f;
    // Whether the instance worker already chose its CUDA stream-capture mode, which it
    // does once, before the CUDA work of its first job. Touched only by the worker.
    bool worker_capture_mode_set = false;
    // A private switch for tests, never a user setting: the mode the test-only hook
    // ovphysx_set_test_execution_for_test_internal() last set when the instance was created,
    // constant afterwards; the library reads no environment for it. kQueue covers the queued
    // path of every short async call in tests written for an idle instance; CI runs the C++
    // CPU tests once that way (scripts/test_cpp.cmake). Calls that always run on the
    // calling thread and synchronous calls run as usual in every mode.
    ovphysx::TestExecution test_execution = ovphysx::TestExecution::kNatural;

    // The instance worker thread, its FIFO and lane, and the task table: receipt
    // indices, reserved failure slots, unfinished worker jobs and collected ranges.
    // Execution records are retired separately from public acknowledgement. A task
    // reserves its failure slot before execution starts; successful retirement drops
    // the slot, so completed successes need no per-task payload. Every table access
    // holds the executor mutex, because worker jobs publish outcomes concurrently
    // with submission and collection. Created with the instance; teardown stops and
    // joins the worker.
    std::shared_ptr<ovphysx::InstanceExecutor> executor;
    // Collected wait results that still own failure text. Touched only by the
    // externally serialized caller and callback-time error lookup.
    std::mutex retained_wait_results_mutex;
    std::vector<std::weak_ptr<TaskWaitStorage>> retained_wait_results;
    // Wrapper-owned query, read and write handles of this instance. Submission
    // and worker completion may overlap, so this table has its own leaf lock.
    ovphysx::OutputHandleTable output_handles;

    // Attachment generations: the attachment a short async call is submitted for.
    //
    // Every accepted attach or detach takes the next generation, in submission order,
    // whether it is queued or runs on the calling thread while a recording is active:
    // lifecycle_generations counts them, expected_generation becomes that generation and
    // lifecycle_task its receipt. An attach or detach that succeeds stores its generation
    // in executed_generation before its outcome is published; one that fails leaves it
    // unchanged, so executed_generation always names the attachment that exists.
    //
    // Steps, clones, updates and the short async calls other than releases take
    // expected_generation when they are submitted (ovphysx::submittedAttachment()) and do
    // their work only if executed_generation equals it when they execute; otherwise their
    // task fails, naming lifecycle_task. A call queued behind an attach or detach therefore
    // never runs against the attachment that attach or detach failed to replace. On the
    // calling thread the instance is idle, so the call executes at once; a queued call
    // compares in its job.
    //
    // Recovery: once the caller collected the outcome of lifecycle_task (ovphysx_wait_task
    // or ovphysx_wait_all) and it failed (executed_generation differs from
    // expected_generation), the next submission sets expected_generation to
    // executed_generation. Calls admitted before keep failing; calls submitted after see
    // the attachment that exists. A failure that is not collected yet keeps failing new
    // calls on both paths, so the outcome never depends on when the worker ran.
    //
    // lifecycle_generations, expected_generation and lifecycle_task are touched only by
    // the externally serialized caller; executed_generation is written by the lane holder
    // that runs the attach or detach and read by queued jobs and submissions.
    uint64_t lifecycle_generations = 0;
    uint64_t expected_generation = 0;
    uint64_t lifecycle_task = 0;
    std::atomic<uint64_t> executed_generation{0};

    // Tensor binding tracking (TensorAPI-backed)
    std::unordered_map<ovphysx_tensor_binding_handle_t, TensorBindingState> tensor_bindings;
    std::mutex tensor_binding_mutex;  // Guards tensor_bindings, contact_bindings and sdf_views. See lock ordering above.

    // Contact binding tracking (IRigidContactView-backed)
    std::unordered_map<ovphysx_contact_binding_handle_t, ContactBindingState> contact_bindings;


    // SDF view tracking (ISdfShapeView-backed)
    struct SdfViewState {
        // Attach identity, not a USD stage id. See TensorBindingState::attachHandle.
        omni::physics::tensors::AttachHandle attachHandle = omni::physics::tensors::kNoAttach;
        uint32_t maxQueryPoints = 0;
        omni::physics::tensors::ISdfShapeView* view = nullptr;
        // Keep the runtime admission owner alive until its child view is released.
        omni::physics::tensors::ISimulationView* simView = nullptr;
    };
    std::unordered_map<ovphysx_sdf_view_handle_t, SdfViewState> sdf_views;

    // Scene query internal hit buffer, valid until the next scene query call.
    // Not thread-safe: concurrent scene queries on the same instance are a data race.
    // Callers serialize access per the single-threaded instance contract.
    std::vector<ovphysx_scene_query_hit_t> sceneQueryHitBuffer;

    // True after this detach already dropped visualization scope tokens. Destroy's
    // fallback clear must not call the sidecar again when detachStage then fails.
    bool visualizationScopeDropped = false;

    // Warmup tracking: requires at least one simulate() before tensor reads (CPU or GPU).
    // Reset when stage changes. Atomic for thread-safe double-checked locking in read path.
    std::atomic<bool> warmup_done{false};
    // Attach the warmup was performed for. Keyed by attach handle, not stage id:
    // a stage id of 0 means both "stageless attach" and "nothing attached", so a
    // fresh attach would inherit the previous one's warmup flag.
    std::atomic<omni::physics::tensors::AttachHandle> warmup_attach_handle{
        omni::physics::tensors::kNoAttach};

    // Async step or warmup dispatched for this attach, including a dispatch that
    // reported an error. Clone must reject it even if warmup did not succeed.
    // Reset alongside warmup_done wherever that flag is cleared.
    std::atomic<bool> first_step_done{false};

    // Whether physxSim->attachStage() has been called for the current stage.
    // Reset to false on unload / stage change, set to true lazily by the first
    // clone(), simulate(), or warmup() call.
    std::atomic<bool> physics_attached{false};

    // Whether the initial PhysX scene parse (simulate+fetchResults with zero dt)
    // has been performed for the current attach.  Reset alongside physics_attached.
    std::atomic<bool> initial_parse_done{false};


    // Reset all per-stage clone and lifecycle state at the three stage transition
    // sites. Keeping the flags together prevents CPU/GPU ordering guards from
    // diverging and makes a fresh attach eligible to reuse clone target paths.
    //
    // physicsAttached: pass true only from attach_ovstage() when the stage
    // attached successfully (stageId != 0). Every other caller passes false.
    void resetStageFlags(bool physicsAttached = false) {
        cloned_target_paths.clear();
        warmup_done.store(false, std::memory_order_release);
        warmup_attach_handle.store(omni::physics::tensors::kNoAttach, std::memory_order_release);
        first_step_done.store(false, std::memory_order_release);
        physics_attached.store(physicsAttached, std::memory_order_release);
        initial_parse_done.store(false, std::memory_order_release);
    }
    
    InstanceData() = default;
    
    ~InstanceData() = default;
    
    // Not copyable due to unique_ptr and mutex.
    InstanceData(const InstanceData&) = delete;
    InstanceData& operator=(const InstanceData&) = delete;
    
    // Not movable either. InstanceData is stored in a shared_ptr, so only the
    // shared_ptr moves.
    InstanceData(InstanceData&&) = delete;
    InstanceData& operator=(InstanceData&&) = delete;
};

namespace ovphysx
{

// The attachment generation a short async call is submitted for, and the attach or
// detach it came from (see "Attachment generations" at InstanceData).
struct SubmittedAttachment
{
    uint64_t generation = 0;
    uint64_t lifecycleTask = 0;
};

// For the externally serialized caller when it submits a short async call: applies the
// recovery rule, then returns what the call is submitted for. Collection state is read
// without the executor mutex, since only this caller collects.
inline SubmittedAttachment submittedAttachment(InstanceData& instance) noexcept
{
    const uint64_t executed = instance.executed_generation.load(std::memory_order_acquire);
    if (instance.expected_generation != executed && instance.lifecycle_task && instance.executor &&
        instance.executor->tasks().isCollected(instance.lifecycle_task))
        instance.expected_generation = executed;
    return {instance.expected_generation, instance.lifecycle_task};
}

// For the externally serialized caller: records an accepted attach or detach that took
// generation, with receipt task.
inline void noteLifecycleSubmission(InstanceData& instance, uint64_t generation, uint64_t task) noexcept
{
    instance.lifecycle_generations = generation;
    instance.expected_generation = generation;
    instance.lifecycle_task = task;
}

// Whether the attachment a call was submitted for still exists, for the lane holder that
// executes the call.
inline bool attachmentCurrent(const InstanceData& instance, const SubmittedAttachment& submitted) noexcept
{
    return submitted.generation == instance.executed_generation.load(std::memory_order_acquire);
}

// The cause a call reports when the attachment it was submitted for does not exist: the
// attach or detach task lifecycleTask failed.
inline std::string attachmentChangedCause(uint64_t lifecycleTask)
{
    return "attach or detach task " + std::to_string(lifecycleTask) +
           " failed, so the attachment this call was submitted for does not exist; collect that task, then "
           "submit again";
}

} // namespace ovphysx

extern std::unordered_map<ovphysx_instance_handle_t, std::shared_ptr<InstanceData>> g_instances;
extern std::shared_mutex g_instances_mutex;

// True when this process must never touch the GPU: either ovphysx_set_cpu_mode(true)
// forced CPU-only mode or OVPHYSX_DISABLE_GPU is active (live before initialize /
// latched at ovphysx_initialize). Defined in ovphysx.cpp. Use this rather than
// checking OVPHYSX_DISABLE_GPU directly so both inputs are honored.
bool isProcessGpuDisabled();

// How many CUDA driver calls ovphysx made to run long calls on instance workers: the
// capture of the submitting thread's context for an attach, its push and pop on the
// worker, each worker's stream-capture mode, and the capture check of a wait that may run
// queued work itself. None happens in hard CPU-only mode.
// Defined in ovphysx.cpp; read by a test helper.
uint64_t ovphysx_worker_cuda_calls() noexcept;

// Calls fn(user) while holding the process-wide lifecycle lock that attach and detach
// jobs and instance destruction take around their lifecycle work. Defined in
// ovphysx.cpp; used by a test helper.
void ovphysx_run_under_lifecycle_lock(void (*fn)(void*), void* user);

// Called before runtime/dictionary teardown, after simulation completion: releases every
// output session of the instance and forgets its handles.
void cleanup_output_sessions(ovphysx_instance_handle_t handle, InstanceData& instance) noexcept;
// Called by a detach: releases the instance's write sessions opened so far and closes their
// handles, which stay known until released: a call submitted after the detach that names one
// is accepted whether or not the detach already ran, and fails its task. Writes queued behind
// the detach are not opened yet and stay.
void cleanup_output_writes(InstanceData& instance) noexcept;

// Sets the InstanceData::test_execution of instances created from now on (defined in
// ovphysx.cpp, for ovphysx_set_test_execution_for_test_internal()).
void ovphysx_set_test_execution(ovphysx::TestExecution mode) noexcept;

// Runs one simulation step of an attached instance to completion on the calling
// thread, which holds the instance lane: the lazy physics attach and initial parse,
// simulate(), then fetchResults(), which also delivers the step's post-step events
// and callbacks on this thread. Used by warmup on the calling thread (a step job runs
// the same two halves); creates no task record. Returns SUCCESS, or ERROR
// with the first failure as the last error. dispatched reports whether the runtime
// accepted the step: an accepted step advanced the simulation and was fetched even
// when it then reported an error.
ovphysx_api_status_t omni_sdk_physx_simulate_instance(InstanceData& instance, ovphysx_instance_handle_t handle,
                                                      float elapsedTime, float currentTime, bool& dispatched);

namespace ovphysx
{

// A worker job of one instance: the body runs on the instance worker with a profiler
// zone and a RuntimeErrorScope. A failure becomes the task failure as
// "<api> (task <index>): <cause>", where the cause is the recorded message or, when
// the body recorded none, the runtime diagnostic captured during the job.
class InstanceJob : public InstanceExecutor::Job
{
public:
    using Body = std::function<void(InstanceJob&)>;

    InstanceJob(const char* api, ovphysx_instance_handle_t handle, std::shared_ptr<InstanceData> instance, Body body);
    void run() override;

    // Records the first failure of the job body. Later calls are ignored.
    void fail(const char* cause) noexcept;
    void fail(const std::string& cause) noexcept { fail(cause.c_str()); }

    const char* api() const noexcept { return m_api; }
    ovphysx_instance_handle_t handle() const noexcept { return m_handle; }
    InstanceData& instance() const noexcept { return *m_instance; }

protected:
    // For a job type that overrides execute() instead of taking a body.
    InstanceJob(const char* api, ovphysx_instance_handle_t handle, std::shared_ptr<InstanceData> instance);
    // The job body; run() wraps it in the profiler zone, the error scope and the
    // exception guard. The default runs the body given at construction.
    virtual void execute() { m_body(*this); }

private:
    const char* m_api;
    ovphysx_instance_handle_t m_handle;
    std::shared_ptr<InstanceData> m_instance;
    Body m_body;
    std::string m_cause;
};

// Queues job on its instance worker and returns the accepted receipt, or a rejection
// (zero identity, thread-local error set): INVALID_STATE when the worker is stopped
// or its queue is full, ERROR when the task index space is exhausted. An instance created
// without a worker thread (see OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION) runs the job on the
// calling thread before this returns.
ovphysx_enqueue_result_t submitInstanceJob(InstanceData& instance, ovphysx_instance_handle_t handle,
                                           std::unique_ptr<InstanceJob> job);
// submitInstanceJob() in two steps, for a call that queues under a lock its job must not
// run under: queueInstanceJob() only queues, runWithoutWorker() then runs the job on an
// instance without a worker thread and returns receipt.
ovphysx_enqueue_result_t queueInstanceJob(InstanceData& instance, ovphysx_instance_handle_t handle,
                                          std::unique_ptr<InstanceJob> job);
ovphysx_enqueue_result_t runWithoutWorker(InstanceData& instance, const ovphysx_enqueue_result_t& receipt);

// Where the calling native thread or Carbonite task runs a callback: "inside a log
// callback", "inside a runtime callback" (an application callback of the runtime) or
// "inside an object-change callback"; nullptr outside callbacks.
const char* callbackContext() noexcept;

// The calling native thread or Carbonite task of one public call, resolved once with a
// single carb.tasking query: the identity its lane holds record and the callback it runs.
struct Caller
{
    uint64_t identity = 0; // InstanceExecutor::identityOf() of the caller
    const char* callback = nullptr; // callbackContext() of the caller
};
Caller currentCaller() noexcept;

// Calls that take a lane, wait for tasks or act on every instance never run inside a
// callback, which may itself run under a held lane, nor on an instance's own worker
// thread. Returns SUCCESS, or INVALID_STATE with a last error naming the context when
// the calling thread or task runs a callback or is the worker thread of instance
// (instance may be null). Nothing is changed, so the call can be retried after the
// callback returns.
ovphysx_api_status_t refuseBlockingReentry(const InstanceData* instance) noexcept;
// The same for a caller resolved by currentCaller().
ovphysx_api_status_t refuseBlockingReentry(const InstanceData* instance, const Caller& caller) noexcept;

// One public call's hold on its instance's lane, kept for the whole call. Only the
// lane holder runs engine code for the instance. Taking the lane waits until the
// worker finished every queued job, a running step included, and no other caller
// holds it; on a free lane it costs one compare-and-swap. The wait never collects
// an outcome. Internal helpers that run under the hold
// (ovphysx_ensure_physics_attached, ovphysx_warmup_if_needed, the detach body)
// never take it again.
class InstanceLane
{
public:
    InstanceLane() = default;
    InstanceLane(const InstanceLane&) = delete;
    InstanceLane& operator=(const InstanceLane&) = delete;
    ~InstanceLane() { release(); }

    // Takes the lane of handle's instance for the calling thread or Carbonite task.
    // SUCCESS: the lane is held. INVALID_STATE: refused before touching the lane (see
    // refuseBlockingReentry), last error set. NOT_FOUND: unknown or destroyed instance,
    // nothing held, last error unchanged.
    ovphysx_api_status_t acquire(ovphysx_instance_handle_t handle);
    // The same for a caller that already looked the instance up and refused blocking
    // reentry for caller itself; it keeps instance alive until the hold is released.
    ovphysx_api_status_t acquire(InstanceData& instance, const Caller& caller);
    // For a short async call of such a caller: takes the lane only when the instance is
    // idle right now (free, with nothing queued, running or held), with no wait and
    // never nested. Returns whether the lane is held; the call queues its work when not.
    bool tryAcquireIdle(InstanceData& instance, const Caller& caller) noexcept
    {
        release();
        if (instance.executor)
            m_hold = instance.executor->tryAcquireIdleLane(caller.identity);
        return m_hold.held();
    }
    // For instance teardown: takes the lane between jobs, waiting only for a running
    // job; queued jobs stay queued for the caller to cancel. Results as acquire().
    ovphysx_api_status_t preempt(ovphysx_instance_handle_t handle);
    void release() noexcept
    {
        m_hold.release();
        m_instance.reset();
    }
    bool held() const noexcept { return m_hold.held(); }

private:
    ovphysx_api_status_t take(InstanceData& instance, uint64_t identity);

    // Keeps the executor alive while held when this hold looked the instance up itself,
    // even if the instance is destroyed meanwhile. Declared before m_hold, which it outlives.
    std::shared_ptr<InstanceData> m_instance;
    InstanceExecutor::LaneHold m_hold;
};

// Every registered instance's lane, for calls that change state all instances share:
// OmniPVD recording transitions, live contact processing, and configuration that
// needs a detached runtime. Lanes are taken in ascending handle order, each after its
// worker drained, and held until release(); outcomes are never collected. Never
// acquire while holding a lane, g_createInstanceMutex, g_gpuAttachMutex or
// g_instances_mutex.
class AllInstanceLanes
{
public:
    AllInstanceLanes() = default;
    AllInstanceLanes(const AllInstanceLanes&) = delete;
    AllInstanceLanes& operator=(const AllInstanceLanes&) = delete;
    ~AllInstanceLanes() { release(); }

    // Takes every registered instance's lane, or extends the held set to instances
    // registered since. preempted names an instance under teardown, whose lane is
    // preempted instead (see InstanceLane::preempt). Returns SUCCESS, or INVALID_STATE
    // with the last error set when refused, before any lane is taken.
    ovphysx_api_status_t acquire(ovphysx_instance_handle_t preempted = OVPHYSX_INVALID_HANDLE);
    // True when the lane of every currently registered instance is held.
    bool coversRegistered() const;
    void release() noexcept;

private:
    struct Entry
    {
        ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
        std::shared_ptr<InstanceExecutor> executor;
        InstanceExecutor::LaneHold hold;
    };
    bool holds(ovphysx_instance_handle_t handle) const noexcept;

    std::vector<Entry> m_entries; // Ascending handle order.
};

} // namespace ovphysx

// Looks up an instance by handle, taking g_instances_mutex internally. Returns
// nullptr if the handle is not found. The returned shared_ptr keeps the instance
// alive even if another thread calls ovphysx_destroy_instance.
inline std::shared_ptr<InstanceData> get_instance(ovphysx_instance_handle_t handle) {
    std::shared_lock<std::shared_mutex> lock(g_instances_mutex);
    auto it = g_instances.find(handle);
    return (it != g_instances.end()) ? it->second : nullptr;
}

// Raw pointer version for callers that already hold g_instances_mutex. The
// returned pointer becomes invalid after releasing the lock.
inline InstanceData* get_instance_ptr(ovphysx_instance_handle_t handle) {
    auto it = g_instances.find(handle);
    return (it != g_instances.end()) ? it->second.get() : nullptr;
}


// Lazy physics-attach helper (ovphysx.cpp). Triggers attachStage() if not
// yet done for the current stage.
ovphysx_api_status_t ovphysx_ensure_physics_attached(ovphysx_instance_handle_t handle);

// Tensor binding cleanup (ovphysxTensorBinding.cpp)
void ovphysx_tensor_binding_cleanup_instance(InstanceData* instance);

// Contact binding cleanup (ovphysxContactBinding.cpp)
void ovphysx_contact_binding_cleanup_instance(InstanceData* instance);

// SDF view cleanup (ovphysxSdfView.cpp)
void ovphysx_sdf_view_cleanup_instance(InstanceData* instance);

// Object-change subscriptions owned by this handle (ovphysxObjectChangeCallbacks.cpp).
bool ovphysx_object_change_attach_instance(ovphysx_instance_handle_t handle, uint64_t attach);
void ovphysx_object_change_detached_instance(ovphysx_instance_handle_t handle);
void ovphysx_object_change_cleanup_instance(ovphysx_instance_handle_t handle);
// True while the calling native thread, or the calling Carbonite task, runs an
// object-change callback. tasking is the running carb.tasking interface, or null
// before it started; the check never starts it. task is tasking's task context of the
// caller, zero on a native thread.
bool ovphysx_object_change_callback_active(carb::tasking::ITasking* tasking, uint64_t task) noexcept;

// Warmup helper (ovphysxTensorBinding.cpp), also callable from contact/SDF binding.
ovphysx_result_t ovphysx_warmup_if_needed(ovphysx_instance_handle_t handle, bool is_explicit_call);

// ============================================================================
// Thread-local error storage (TLS error model)
// ============================================================================

struct ThreadLocalError {
    std::string last_error;
    ovphysx_string_t fallback_error{};
};

inline ThreadLocalError& tls_error() {
    thread_local ThreadLocalError t_error;
    return t_error;
}

inline ovphysx_result_t set_error(ovphysx_api_status_t status, const char* msg) noexcept {
    ThreadLocalError& error = tls_error();
    error.fallback_error = {};
    try {
        error.last_error = msg ? msg : "";
    } catch (...) {
        error.last_error.clear();
        static constexpr char fallback[] = "Unable to store error detail (allocation failed)";
        error.fallback_error = {fallback, sizeof(fallback) - 1};
    }
    return {status};
}

inline ovphysx_result_t set_error(ovphysx_api_status_t status, const std::string& msg) noexcept {
    return set_error(status, msg.c_str());
}

// Both strings are caller-owned, non-null text. Copy the runtime detail before its scope ends.
inline ovphysx_result_t set_runtime_error(const char* context, const char* detail) {
    return detail[0] ? set_error(OVPHYSX_API_ERROR, std::string(context) + ": " + detail)
                     : set_error(OVPHYSX_API_ERROR, context);
}

inline ovphysx_result_t success() noexcept {
    tls_error().last_error.clear();
    tls_error().fallback_error = {};
    return {OVPHYSX_API_SUCCESS};
}

inline ovphysx_enqueue_result_t set_enqueue_error(ovphysx_api_status_t status, const char* msg) noexcept {
    set_error(status, msg);
    return {status, 0, 0};
}

inline ovphysx_enqueue_result_t set_enqueue_error(ovphysx_api_status_t status, const std::string& msg) noexcept {
    return set_enqueue_error(status, msg.c_str());
}

inline bool isValid(const ovphysx_string_t& s)
{
    return s.ptr != nullptr && s.length > 0;
}

// USD prim paths and glob patterns must not carry embedded NUL bytes.
// ovphysx_string_t is length-preserving, but several downstream consumers
// (const char* ABI boundaries, glob matchers) mishandle embedded NULs.
inline bool hasEmbeddedNul(const ovphysx_string_t& s)
{
    return s.ptr && s.length > 0 && std::memchr(s.ptr, '\0', s.length) != nullptr;
}

// Path patterns are matched one component at a time, where a component is the text
// between '/' separators outside balanced parentheses (the runtime tokenizer keeps a
// group like "(a|b/c)" whole; see splitPatternRespectingGroups in PathPatternMatcher.cpp),
// and the runtime refuses to compile a component longer than
// kMaxPathPatternComponentLength (the regex compile would otherwise overflow the stack).
// Reject such input here, with the same delimiting rule, so the caller gets
// INVALID_ARGUMENT instead of a silently empty result. This is never looser than the
// runtime; it is stricter only for an unclosed '(' followed by a trailing '/', which the
// runtime trims before tokenizing.
inline bool hasOversizedPathComponent(const ovphysx_string_t& s)
{
    if (!s.ptr)
        return false;
    std::size_t componentLength = 0;
    int groupDepth = 0;
    for (std::size_t i = 0; i < s.length; ++i)
    {
        const char c = s.ptr[i];
        if (c == '(')
            ++groupDepth;
        else if (c == ')' && groupDepth > 0)
            --groupDepth;
        if (c == '/' && groupDepth == 0)
        {
            componentLength = 0;
            continue;
        }
        if (++componentLength > omni::physics::tensors::kMaxPathPatternComponentLength)
            return true;
    }
    return false;
}

inline std::string oversizedPathComponentMessage(const char* argument)
{
    return std::string(argument) + " contains a component longer than " +
           std::to_string(omni::physics::tensors::kMaxPathPatternComponentLength) + " characters";
}

inline std::string toStdString(const ovphysx_string_t& s)
{
    return (s.ptr && s.length > 0) ? std::string(s.ptr, s.length) : std::string();
}
